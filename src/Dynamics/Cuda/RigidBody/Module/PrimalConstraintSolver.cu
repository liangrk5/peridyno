#include "PrimalConstraintSolver.h"
#include "SharedFuncsForRigidBody.h"


namespace dyno
{
	IMPLEMENT_TCLASS(PrimalConstraintSolver, TDataType)
	template<typename TDataType>
	PrimalConstraintSolver<TDataType>::PrimalConstraintSolver()
		:ConstraintModule()
	{
		this->inContacts()->tagOptional(true);
	}

	template<typename TDataType>
	PrimalConstraintSolver<TDataType>::~PrimalConstraintSolver()
	{
		
	}

	template<typename TDataType>
	void PrimalConstraintSolver<TDataType>::IterationOneStep(Real dt)
	{
		int constraint_size = 0;
		int joint_size = 0;
		int contact_size = this->inContacts()->size();

		auto topo = this->inDiscreteElements()->constDataPtr();
		int ballAndSocketJoint_size = topo->ballAndSocketJoints().size();
		int hingeJoint_size = topo->hingeJoints().size();
		int pointJoint_size = topo->pointJoints().size();
		int sliderJoint_size = topo->sliderJoints().size();
		int fixedJoint_size = topo->fixedJoints().size();

		if (this->varFrictionEnabled()->getData())
		{
			constraint_size += 3 * this->inContacts()->size();
		}
		else
		{
			constraint_size = this->inContacts()->size();
		}

		if (ballAndSocketJoint_size != 0)
		{
			constraint_size += 3 * ballAndSocketJoint_size;
			joint_size += 3 * ballAndSocketJoint_size;
		}
		
		if (hingeJoint_size != 0)
		{
			constraint_size += 8 * hingeJoint_size;
			joint_size += 8 * hingeJoint_size;
		}
		
		if (pointJoint_size != 0)
		{
			constraint_size += 3 * pointJoint_size;
			joint_size += 3 * pointJoint_size;
		}

		if (sliderJoint_size != 0)
		{
			constraint_size += 6 * sliderJoint_size;
			joint_size += 6 * sliderJoint_size;
		}

		if (fixedJoint_size != 0)
		{
			constraint_size += 6 * fixedJoint_size;
			joint_size += 6 * fixedJoint_size;
		}

		if (mVelocityConstraints.size() != constraint_size)
		{
			mVelocityConstraints.resize(constraint_size);
			mJ.resize(4 * constraint_size);
			mB.resize(4 * constraint_size);
			mD.resize(constraint_size);
			mEffectMass.resize(constraint_size);
		}

		mVelocityConstraints.reset();
		mJ.reset();
		mB.reset();
		mD.reset();
		mEffectMass.reset();


		if (this->inContacts()->size() != 0)
		{
			auto& contacts = this->inContacts()->getData();
			setUpContactAndFrictionConstraints(
				mVelocityConstraints,
				mContactsInLocalFrame,
				mCurrentCenter,
				mCurrentRotationMatrix,
				this->varFrictionEnabled()->getValue()
			);
		}

		if (ballAndSocketJoint_size != 0)
		{
			auto& ballAndSocketJoints = topo->ballAndSocketJoints();
			int begin_index = this->inContacts()->size();
			if (this->varFrictionEnabled()->getValue())
			{
				begin_index += 2 * this->inContacts()->size();
			}

			setUpBallAndSocketJointConstraints(
				mVelocityConstraints,
				ballAndSocketJoints,
				mCurrentCenter,
				mCurrentRotationMatrix,
				begin_index
			);
		}

		if (hingeJoint_size != 0)
		{
			auto& hingeJoints = topo->hingeJoints();
			int begin_index = this->inContacts()->size();
			if (this->varFrictionEnabled()->getValue())
			{
				begin_index += 2 * this->inContacts()->size();
			}

			setUpHingeJointConstraints(
				mVelocityConstraints,
				hingeJoints,
				mCurrentCenter,
				mCurrentRotationMatrix,
				mCurrentQuat,
				begin_index
			);
		}

		if (pointJoint_size != 0)
		{
			auto& pointJoints = topo->pointJoints();
			int begin_index = this->inContacts()->size();
			if (this->varFrictionEnabled()->getValue())
			{
				begin_index += 2 * this->inContacts()->size();
			}

			setUpPointJointConstraints(
				mVelocityConstraints,
				pointJoints,
				mCurrentCenter,
				begin_index
			);
		}

		if (sliderJoint_size != 0)
		{
			auto& sliderJoints = topo->sliderJoints();
			int begin_index = this->inContacts()->size();
			if (this->varFrictionEnabled()->getValue())
			{
				begin_index += 2 * this->inContacts()->size();
			}

			setUpSliderJointConstraints(
				mVelocityConstraints,
				sliderJoints,
				mCurrentCenter,
				mCurrentRotationMatrix,
				mCurrentQuat,
				begin_index
			);
		}

		if (fixedJoint_size != 0)
		{
			auto& fixedJoints = topo->fixedJoints();
			int begin_index = this->inContacts()->size();
			if (this->varFrictionEnabled()->getValue())
			{
				begin_index += 2 * this->inContacts()->size();
			}

			setUpFixedJointConstraints(
				mVelocityConstraints,
				fixedJoints,
				mCurrentRotationMatrix,
				mCurrentQuat,
				begin_index
			);
		}

		auto sizeOfRigidBodies = this->inCenter()->size();

		
		if (mP.size() != 2 * sizeOfRigidBodies)
		{
			mP.resize(2 * sizeOfRigidBodies);
			mImpulseC.resize(2 * sizeOfRigidBodies);
			mPreconditioner.resize(2 * sizeOfRigidBodies);
			mGradient.resize(2 * sizeOfRigidBodies);
		}

		mP.reset();
		mImpulseC.reset();
		mPreconditioner.reset();
		mGradient.reset();


		normalForces.resize(contact_size);
		normalForces.reset();

		calculateJacobianMatrix(
			mJ,
			mB,
			mCurrentCenter,
			mCurrentInertia,
			this->inMass()->getData(),
			mCurrentRotationMatrix,
			mVelocityConstraints
		);

		Real maxEffectMass = 0;

		if (this->varAutoStiffnessType()->getValue() == 1)
		{
			maxEffectMass = calculateDiagnalsForMaxStiffness(
				mEffectMass,
				mJ,
				mB
			);
		}

		else if (this->varAutoStiffnessType()->getValue() == 2)
		{
			calculateDiagnalsForPrivateStiffness(
				mEffectMass,
				mJ,
				mB
			);
		}

		evaluateForceAndDerivatives(
			mImpulseC,
			mP,
			mEffectMass,
			mContactsInLocalFrame,
			mJ,
			mVelocityConstraints,
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			normalForces,
			this->inMass()->getData(),
			mCurrentCenter,
			mCurrentQuat,
			this->inFrictionCoefficients()->getData(),
			this->varStiffness()->getValue(),
			this->varFrictionStiffness()->getValue(),
			dt,
			this->varHertz()->getValue(),
			maxEffectMass,
			contact_size,
			joint_size,
			this->varAutoStiffnessType()->getValue(),
			this->varFrictionEnabled()->getValue()
		);

		buildPreconditioner(
			mPreconditioner,
			mP,
			this->inMass()->getData(),
			mCurrentInertia,
			dt
		);

		calculateGradientPrimal(
			mGradient,
			this->inMass()->getData(),
			mCurrentInertia,
			initialVelocity,
			this->inVelocity()->getData(),
			initialAngularVelocity,
			this->inAngularVelocity()->getData(),
			mImpulseC,
			dt
		);

		updateStatePrimal(
			this->inAttribute()->getData(),
			this->inCenter()->getData(),
			this->inQuaternion()->getData(),
			mCurrentCenter,
			mCurrentQuat,
			mCurrentRotationMatrix,
			mCurrentInertia,
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			this->inInitialInertia()->getData(),
			mPreconditioner,
			mGradient,
			this->varStepSize()->getValue(),
			dt
		);
	}

	template<typename TDataType>
	void PrimalConstraintSolver<TDataType>::constrain()
	{
		uint bodyNum = this->inCenter()->size();

		auto topo = this->inDiscreteElements()->constDataPtr();

		mImpulseExt.resize(bodyNum * 2);
		mImpulseExt.reset();

		if (mContactsInLocalFrame.size() != this->inContacts()->size()) {
			mContactsInLocalFrame.resize(this->inContacts()->size());
		}

		if (!this->inContacts()->isEmpty())
		{
			setUpContactsInLocalFrame(
				mContactsInLocalFrame,
				this->inContacts()->getData(),
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData()
			);
		}

		Real dt = this->inTimeStep()->getData();

		if (this->varGravityEnabled()->getValue())
		{
			setUpGravity(
				mImpulseExt,
				this->varGravityValue()->getValue(),
				dt
			);
		}

		updateVelocity(
			this->inAttribute()->getData(),
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			mImpulseExt,
			this->varLinearDamping()->getValue(),
			this->varAngularDamping()->getValue(),
			dt
		);

		if (!this->inContacts()->isEmpty() || topo->totalJointSize() > 0)
		{
			mCurrentCenter.assign(this->inCenter()->getData());
			mCurrentRotationMatrix.assign(this->inRotationMatrix()->getData());
			mCurrentQuat.assign(this->inQuaternion()->getData());
			mCurrentInertia.assign(this->inInertia()->getData());

			initialVelocity.assign(this->inVelocity()->getData());
			initialAngularVelocity.assign(this->inAngularVelocity()->getData());

			updateGesture(
				this->inAttribute()->getData(),
				mCurrentCenter,
				mCurrentQuat,
				mCurrentRotationMatrix,
				mCurrentInertia,
				this->inVelocity()->getData(),
				this->inAngularVelocity()->getData(),
				this->inInitialInertia()->getData(),
				dt
			);

			for (int i = 0; i < this->varIterationNumberForVelocitySolver()->getData(); i++)
			{
				IterationOneStep(dt);
			}

			this->inCenter()->getData().assign(mCurrentCenter);
			this->inRotationMatrix()->getData().assign(mCurrentRotationMatrix);
			this->inQuaternion()->getData().assign(mCurrentQuat);
			this->inInertia()->getData().assign(mCurrentInertia);
		}

		else
		{
			updateGesture(
				this->inAttribute()->getData(),
				this->inCenter()->getData(),
				this->inQuaternion()->getData(),
				this->inRotationMatrix()->getData(),
				this->inInertia()->getData(),
				this->inVelocity()->getData(),
				this->inAngularVelocity()->getData(),
				this->inInitialInertia()->getData(),
				dt
			);
		}
	}

	DEFINE_CLASS(PrimalConstraintSolver);
}