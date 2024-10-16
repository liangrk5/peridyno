#include "PJSNJSConstraintSolver.h"
#include "SharedFuncsForRigidBody.h"

namespace dyno
{
	IMPLEMENT_TCLASS(PJSNJSConstraintSolver, TDataType)

	template<typename TDataType>
	PJSNJSConstraintSolver<TDataType>::PJSNJSConstraintSolver()
		:ConstraintModule()
	{
		this->inContacts()->tagOptional(true);
	}

	template<typename TDataType>
	PJSNJSConstraintSolver<TDataType>::~PJSNJSConstraintSolver()
	{}

	template<typename TDataType>
	void PJSNJSConstraintSolver<TDataType>::initializeJacobian(Real dt)
	{
		int constraint_size = 0;
		int contact_size = this->inContacts()->size();

		auto topo = this->inDiscreteElements()->constDataPtr();

		int ballAndSocketJoint_size = topo->ballAndSocketJoints().size();
		int sliderJoint_size = topo->sliderJoints().size();
		int hingeJoint_size = topo->hingeJoints().size();
		int fixedJoint_size = topo->fixedJoints().size();
		int pointJoint_size = topo->pointJoints().size();

		if (this->varFrictionEnabled()->getData())
		{
			constraint_size += 3 * contact_size;
		}
		else
		{
			constraint_size = contact_size;
		}

		if (ballAndSocketJoint_size != 0)
		{
			constraint_size += 3 * ballAndSocketJoint_size;
		}

		if (sliderJoint_size != 0)
		{
			constraint_size += 8 * sliderJoint_size;
		}

		if (hingeJoint_size != 0)
		{
			constraint_size += 8 * hingeJoint_size;
		}

		if (fixedJoint_size != 0)
		{
			constraint_size += 6 * fixedJoint_size;
		}

		if (pointJoint_size != 0)
		{
			constraint_size += 3 * pointJoint_size;
		}

		if (constraint_size == 0)
		{
			return;
		}

		mVelocityConstraints.resize(constraint_size);

		if (contact_size != 0)
		{
			auto& contacts = this->inContacts()->getData();
			setUpContactAndFrictionConstraints(
				mVelocityConstraints,
				mContactsInLocalFrame,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData(),
				this->varFrictionEnabled()->getData()
			);
		}

		if (ballAndSocketJoint_size != 0)
		{
			auto& joints = topo->ballAndSocketJoints();
			int begin_index = contact_size;

			if (this->varFrictionEnabled()->getData())
			{
				begin_index += 2 * contact_size;
			}

			setUpBallAndSocketJointConstraints(
				mVelocityConstraints,
				joints,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData(),
				begin_index
			);
		}

		if (sliderJoint_size != 0)
		{
			auto& joints = topo->sliderJoints();
			int begin_index = contact_size;

			if (this->varFrictionEnabled()->getData())
			{
				begin_index += 2 * contact_size;
			}
			begin_index += 3 * ballAndSocketJoint_size;
			setUpSliderJointConstraints(
				mVelocityConstraints,
				joints,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData(),
				begin_index
			);
		}

		if (hingeJoint_size != 0)
		{
			auto& joints = topo->hingeJoints();
			int begin_index = contact_size + 3 * ballAndSocketJoint_size + 8 * sliderJoint_size;
			if (this->varFrictionEnabled()->getData())
			{
				begin_index += 2 * contact_size;
			}
			setUpHingeJointConstraints(
				mVelocityConstraints,
				joints,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData(),
				this->inQuaternion()->getData(),
				begin_index
			);
		}

		if (fixedJoint_size != 0)
		{
			auto& joints = topo->fixedJoints();
			int begin_index = contact_size + 3 * ballAndSocketJoint_size + 8 * sliderJoint_size + 8 * hingeJoint_size;
			if (this->varFrictionEnabled()->getData())
			{
				begin_index += 2 * contact_size;
			}
			setUpFixedJointConstraints(
				mVelocityConstraints,
				joints,
				this->inRotationMatrix()->getData(),
				this->inQuaternion()->getData(),
				begin_index
			);
		}

		if (pointJoint_size != 0)
		{
			auto& joints = topo->pointJoints();
			int begin_index = contact_size + 3 * ballAndSocketJoint_size + 8 * sliderJoint_size + 8 * hingeJoint_size + 6 * fixedJoint_size;
			if (this->varFrictionEnabled()->getData())
			{
				begin_index += 2 * contact_size;
			}
			setUpPointJointConstraints(
				mVelocityConstraints,
				joints,
				this->inCenter()->getData(),
				begin_index
			);
		}

		auto sizeOfRigids = this->inCenter()->size();
		mContactNumber.resize(sizeOfRigids);

		mJ.resize(4 * constraint_size);
		mB.resize(4 * constraint_size);
		mK_1.resize(constraint_size);
		mK_2.resize(constraint_size);
		mK_3.resize(constraint_size);
		mEta.resize(constraint_size);
		mLambda.resize(constraint_size);

		errors_begin.resize(constraint_size);

		mJ.reset();
		mB.reset();
		mK_1.reset();
		mK_2.reset();
		mK_3.reset();
		mEta.reset();
		mLambda.reset();

		mContactNumber.reset();

		calculateJacobianMatrix(
			mJ,
			mB,
			this->inCenter()->getData(),
			this->inInertia()->getData(),
			this->inMass()->getData(),
			this->inRotationMatrix()->getData(),
			mVelocityConstraints
		);

		calculateK(
			mVelocityConstraints,
			mJ,
			mB,
			this->inCenter()->getData(),
			this->inInertia()->getData(),
			this->inMass()->getData(),
			mK_1,
			mK_2,
			mK_3
		);

		calculateEtaVectorForPJSBaumgarteWithErrors(
			mEta,
			mJ,
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			this->inCenter()->getData(),
			this->inQuaternion()->getData(),
			mVelocityConstraints,
			errors_begin,
			this->varSlop()->getValue(),
			this->varBaumgarteBias()->getValue(),
			dt
		);

		initNum = innerDotOfVector(errors_begin, errors_begin);
		initNum = sqrt(initNum);


		if (contact_size != 0)
		{
			calculateContactPoints(
				this->inContacts()->getData(),
				mContactNumber);
		}
	}

	template<typename TDataType>
	void PJSNJSConstraintSolver<TDataType>::initializeJacobianForNJS(int i)
	{
		int constraint_size = 0;
		int contact_size = this->inContacts()->size();

		auto topo = this->inDiscreteElements()->constDataPtr();

		int ballAndSocketJoint_size = topo->ballAndSocketJoints().size();
		int sliderJoint_size = topo->sliderJoints().size();
		int hingeJoint_size = topo->hingeJoints().size();
		int fixedJoint_size = topo->fixedJoints().size();
		int pointJoint_size = topo->pointJoints().size();

		constraint_size += contact_size;

		if (ballAndSocketJoint_size != 0)
		{
			constraint_size += 3 * ballAndSocketJoint_size;
		}

		if (sliderJoint_size != 0)
		{
			constraint_size += 8 * sliderJoint_size;
		}

		if (hingeJoint_size != 0)
		{
			constraint_size += 8 * hingeJoint_size;
		}

		if (fixedJoint_size != 0)
		{
			constraint_size += 6 * fixedJoint_size;
		}

		if (pointJoint_size != 0)
		{
			constraint_size += 3 * pointJoint_size;
		}

		if (constraint_size == 0)
		{
			return;
		}

		mPositionConstraints.resize(constraint_size);

		if (contact_size != 0)
		{
			auto& contacts = this->inContacts()->getData();
			setUpContactConstraints(
				mPositionConstraints,
				mContactsInLocalFrame,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData()
			);
		}

		if (ballAndSocketJoint_size != 0)
		{
			auto& joints = topo->ballAndSocketJoints();
			int begin_index = contact_size;

			setUpBallAndSocketJointConstraints(
				mPositionConstraints,
				joints,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData(),
				begin_index
			);
		}

		if (sliderJoint_size != 0)
		{
			auto& joints = topo->sliderJoints();
			int begin_index = contact_size;

			begin_index += 3 * ballAndSocketJoint_size;
			setUpSliderJointConstraints(
				mPositionConstraints,
				joints,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData(),
				begin_index
			);
		}

		if (hingeJoint_size != 0)
		{
			auto& joints = topo->hingeJoints();
			int begin_index = contact_size + 3 * ballAndSocketJoint_size + 8 * sliderJoint_size;

			setUpHingeJointConstraints(
				mPositionConstraints,
				joints,
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData(),
				this->inQuaternion()->getData(),
				begin_index
			);
		}

		if (fixedJoint_size != 0)
		{
			auto& joints = topo->fixedJoints();
			int begin_index = contact_size + 3 * ballAndSocketJoint_size + 8 * sliderJoint_size + 8 * hingeJoint_size;

			setUpFixedJointConstraints(
				mVelocityConstraints,
				joints,
				this->inRotationMatrix()->getData(),
				this->inQuaternion()->getData(),
				begin_index
			);
		}

		if (pointJoint_size != 0)
		{
			auto& joints = topo->pointJoints();
			int begin_index = contact_size + 3 * ballAndSocketJoint_size + 8 * sliderJoint_size + 8 * hingeJoint_size + 6 * fixedJoint_size;

			setUpPointJointConstraints(
				mPositionConstraints,
				joints,
				this->inCenter()->getData(),
				begin_index
			);
		}

		

		auto sizeOfRigids = this->inCenter()->size();
		mJ_p.resize(4 * constraint_size);
		mB_p.resize(4 * constraint_size);
		mK_1.resize(constraint_size);
		mK_2.resize(constraint_size);
		mK_3.resize(constraint_size);
		mEta_p.resize(constraint_size);
		mLambda.resize(constraint_size);

		errors_after_velocity.resize(constraint_size);
		errors_after_position.resize(constraint_size);

		mJ_p.reset();
		mB_p.reset();
		mK_1.reset();
		mK_2.reset();
		mK_3.reset();
		mEta_p.reset();
		mLambda.reset();

		errors_after_position.reset();
		errors_after_velocity.reset();

		calculateJacobianMatrixForNJS(
			mJ_p,
			mB_p,
			this->inCenter()->getData(),
			this->inInertia()->getData(),
			this->inMass()->getData(),
			this->inRotationMatrix()->getData(),
			mPositionConstraints
		);

		calculateK(
			mPositionConstraints,
			mJ_p,
			mB_p,
			this->inCenter()->getData(),
			this->inInertia()->getData(),
			this->inMass()->getData(),
			mK_1,
			mK_2,
			mK_3
		);

		if (i == 0)
		{
			calculateEtaVectorForNJSWithErrors(
				mEta_p,
				mJ_p,
				this->inCenter()->getData(),
				this->inQuaternion()->getData(),
				mPositionConstraints,
				errors_after_velocity,
				this->varSlop()->getValue(),
				0.2
			);
			this->vel_solve = innerDotOfVector(errors_after_velocity, errors_after_velocity);
			this->vel_solve = sqrt(this->vel_solve);
			this->velocity_solve.push_back(this->vel_solve - initNum);
		}

		else if (i == this->varIterationNumberForPositionSolver()->getValue())
		{
			calculateEtaVectorForNJSWithErrors(
				mEta_p,
				mJ_p,
				this->inCenter()->getData(),
				this->inQuaternion()->getData(),
				mPositionConstraints,
				errors_after_position,
				this->varSlop()->getValue(),
				0.2
			);

			this->pos_solve = innerDotOfVector(errors_after_position, errors_after_position);
			this->pos_solve = sqrt(this->pos_solve);
			position_solve.push_back(pos_solve - vel_solve);
		}

		else
		{
			calculateEtaVectorForNJS(
				mEta_p,
				mJ_p,
				this->inCenter()->getData(),
				this->inQuaternion()->getData(),
				mPositionConstraints,
				this->varSlop()->getValue(),
				0.2
			);
		}
	}

	template<typename TDataType>
	void PJSNJSConstraintSolver<TDataType>::constrain()
	{
		cnt++;
		uint bodyNum = this->inCenter()->size();

		auto topo = this->inDiscreteElements()->constDataPtr();

		mImpulseC.resize(bodyNum * 2);
		mImpulseExt.resize(bodyNum * 2);
		mImpulseC.reset();
		mImpulseExt.reset();

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
			this->inFixedTag()->getData(),
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			mImpulseExt,
			this->varLinearDamping()->getValue(),
			this->varAngularDamping()->getValue(),
			dt
		);

		updateGesture(
			this->inCenter()->getData(),
			this->inQuaternion()->getData(),
			this->inRotationMatrix()->getData(),
			this->inInertia()->getData(),
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			this->inInitialInertia()->getData(),
			dt
		);

		if (!this->inContacts()->isEmpty())
		{
			if (mContactsInLocalFrame.size() != this->inContacts()->size()) {
				mContactsInLocalFrame.resize(this->inContacts()->size());
			}

			setUpContactsInLocalFrame(
				mContactsInLocalFrame,
				this->inContacts()->getData(),
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData()
			);
		}
		int contact_size = this->inContacts()->size();

		//Velocity Solver
		if (!this->inContacts()->isEmpty() || topo->totalJointSize() > 0)
		{
			initializeJacobian(dt);
			int constraint_size = mVelocityConstraints.size();
			for (int i = 0; i < this->varIterationNumberForVelocitySolver()->getValue(); i++)
			{
				JacobiIteration(
					mLambda,
					mImpulseC,
					mJ,
					mB,
					mEta,
					mVelocityConstraints,
					mContactNumber,
					mK_1,
					mK_2,
					mK_3,
					this->inMass()->getData(),
					this->varFrictionCoefficient()->getData(),
					this->varGravityValue()->getData(),
					dt
				);
			}
		}

		/*updateVelocity(
			this->inFixedTag()->getData(),
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			mImpulseC,
			this->varLinearDamping()->getValue(),
			this->varAngularDamping()->getValue(),
			dt
		);
		*/

		// Position Solver
		if (!this->inContacts()->isEmpty() || topo->totalJointSize() > 0)
		{
			for (int i = 0; i < this->varIterationNumberForPositionSolver()->getValue(); i++)
			{
				mImpulseC.reset();
				mLambda.reset();
				initializeJacobianForNJS(i);
				int constraint_size = mPositionConstraints.size();
				for (int j = 0; j < 1; j++)
				{
						JacobiIterationForNJS(
							mLambda,
							mImpulseC,
							mJ_p,
							mB_p,
							mEta_p,
							mPositionConstraints,
							mContactNumber,
							mK_1,
							mK_2,
							mK_3
						);
					
				}

				updatePositionAndRotation(
					this->inCenter()->getData(),
					this->inQuaternion()->getData(),
					this->inRotationMatrix()->getData(),
					this->inInertia()->getData(),
					this->inInitialInertia()->getData(),
					mImpulseC,
					this->inVelocity()->getData(),
					this->inAngularVelocity()->getData(),
					dt
				);
			}

			initializeJacobianForNJS(this->varIterationNumberForPositionSolver()->getValue());
		}
		std::cout << initNum << " " << vel_solve << " " << pos_solve << std::endl;
		if (cnt == 1000)
		{
			for (int i = 0; i < velocity_solve.size(); i++)
			{
				std::cout << velocity_solve[i] << " " << position_solve[i] << std::endl;
			}
			std::cin.get();
		}
	}
	DEFINE_CLASS(PJSNJSConstraintSolver);
}