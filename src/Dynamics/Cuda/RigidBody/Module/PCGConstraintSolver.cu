#include "PCGConstraintSolver.h"
#include "SharedFuncsForRigidBody.h"

namespace dyno
{
	IMPLEMENT_TCLASS(PCGConstraintSolver, TDataType)

	template<typename TDataType>
	PCGConstraintSolver<TDataType>::PCGConstraintSolver()
		:ConstraintModule()
	{
		this->inContacts()->tagOptional(true);
	}

	template<typename TDataType>
	PCGConstraintSolver<TDataType>::~PCGConstraintSolver()
	{

	}

	template<typename TDataType>
	void PCGConstraintSolver<TDataType>::initializeJacobian(Real dt)
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
			if (mContactsInLocalFrame.size() != this->inContacts()->size()) {
				mContactsInLocalFrame.resize(this->inContacts()->size());
			}

			setUpContactsInLocalFrame(
				mContactsInLocalFrame,
				this->inContacts()->getData(),
				this->inCenter()->getData(),
				this->inRotationMatrix()->getData()
			);

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
		mContactNumber.reset();

		mJ.resize(4 * constraint_size);
		mB.resize(4 * constraint_size);
		mEta.resize(constraint_size);
		mLambda.resize(constraint_size);
		mCFM.resize(constraint_size);
		mERP.resize(constraint_size);
		mD.resize(constraint_size);
		mD_inv.resize(constraint_size);
		
		
		
		mJ.reset();
		mB.reset();
		mEta.reset();
		mLambda.reset();
		mCFM.reset();
		mERP.reset();
		mD.reset();
		mD_inv.reset();


		gradient.resize(constraint_size);
		freeGradient.resize(constraint_size);
		reducedGradient.resize(constraint_size);
		choppedGradient.resize(constraint_size);
		mP.resize(constraint_size);
		mAp.resize(constraint_size);
		mAg.resize(constraint_size);
		deltaArray.resize(constraint_size);

		gradient.reset();
		freeGradient.reset();
		reducedGradient.reset();
		choppedGradient.reset();
		mP.reset();
		mAp.reset();
		mAg.reset();
		deltaArray.reset();

		projectionGradient.resize(constraint_size);
		projectionGradient.reset();


		calculateJacobianMatrix(
			mJ,
			mB,
			this->inCenter()->getData(),
			this->inInertia()->getData(),
			this->inMass()->getData(),
			this->inRotationMatrix()->getData(),
			mVelocityConstraints
		);



		buildCFMAndERP(
			mJ,
			mB,
			mVelocityConstraints,
			mCFM,
			mERP,
			this->varFrequency()->getValue(),
			this->varDampingRatio()->getValue(),
			dt
		);


		calculateEtaVectorWithERP(
			mEta,
			mJ,
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			this->inCenter()->getData(),
			this->inQuaternion()->getData(),
			mVelocityConstraints,
			mERP,
			this->varSlop()->getValue(),
			dt
		);

		if (contact_size != 0)
		{
			calculateContactPoints(
				this->inContacts()->getData(),
				mContactNumber);
		}

		calculateDiagnalsInv(
			mD_inv,
			mJ,
			mB,
			mCFM,
			mContactNumber,
			mVelocityConstraints
		);
	}


	template<typename TDataType>
	void PCGConstraintSolver<TDataType>::constrain()
	{
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
	

		if (!this->inContacts()->isEmpty() || topo->totalJointSize() > 0)
		{
			initializeJacobian(dt);

			int constraint_size = mVelocityConstraints.size();
			int contact_size = this->inContacts()->size();

			initLambda(0, mLambda);
			// MPRGP
			calculateGradient(
				freeGradient,
				choppedGradient,
				gradient,
				mImpulseC,
				mJ,
				mB,
				mD_inv,
				mLambda,
				mEta,
				mCFM,
				mVelocityConstraints
			);

			mP.assign(freeGradient);
			Real init_norm;

			for (int i = 0; i < this->varIterationNumberForVelocitySolverCG()->getValue(); i++)
			{
				vectorAdd(projectionGradient, freeGradient, choppedGradient);
				Real norm = sqrt(innerDotOfVector(projectionGradient, projectionGradient));
				if (i == 0)
				{
					init_norm = norm;
				}

				if (norm <= 1e-4 * init_norm)
					break;
				Real gc_norm = innerDotOfVector(choppedGradient, choppedGradient);
				Real gf_norm = innerDotOfVector(freeGradient, freeGradient);
				

				if (gc_norm <= gf_norm)
				{

					mImpulseC.reset();
					calculateAx(
						mAp,
						mImpulseC,
						mJ,
						mB,
						mD_inv,
						mP,
						mCFM,
						mVelocityConstraints
					);

					Real alpha_cg = innerDotOfVector(gradient, mP) / innerDotOfVector(mP, mAp);

					Real alpha_f = calculateMaxStep(
						mP,
						mLambda,
						mVelocityConstraints,
						alpha_cg
					);

					vectorMultiplyScale(deltaArray, mP, -alpha_cg);
					vectorAdd(mLambda, mLambda, deltaArray);

					if (alpha_cg <= alpha_f)
					{
						calculateGradient(
							freeGradient,
							choppedGradient,
							gradient,
							mImpulseC,
							mJ,
							mB,
							mD_inv,
							mLambda,
							mEta,
							mCFM,
							mVelocityConstraints
						);


						mImpulseC.reset();
						calculateAx(
							mAg,
							mImpulseC,
							mJ,
							mB,
							mD_inv,
							freeGradient,
							mCFM,
							mVelocityConstraints
						); 
						Real beta = innerDotOfVector(mP, mAg) / innerDotOfVector(mP, mAp);
						vectorMultiplyScale(deltaArray, mP, -beta);
						vectorAdd(mP, deltaArray, freeGradient);
					}
					else
					{
						projectionLambda(
							mLambda,
							mVelocityConstraints
						);

						calculateGradient(
							freeGradient,
							choppedGradient,
							gradient,
							mImpulseC,
							mJ,
							mB,
							mD_inv,
							mLambda,
							mEta,
							mCFM,
							mVelocityConstraints
						);
						mP.assign(freeGradient);
					}
				}
				else {
					mImpulseC.reset();
					calculateAx(
						mAg,
						mImpulseC,
						mJ,
						mB,
						mD_inv,
						choppedGradient,
						mCFM,
						mVelocityConstraints
					);

					Real alpha_cg = innerDotOfVector(gradient, choppedGradient) / innerDotOfVector(choppedGradient, mAg);
					vectorMultiplyScale(deltaArray, choppedGradient, -alpha_cg);
					vectorAdd(mLambda, deltaArray, mLambda);

					calculateGradient(
						freeGradient,
						choppedGradient,
						gradient,
						mImpulseC,
						mJ,
						mB,
						mD_inv,
						mLambda,
						mEta,
						mCFM,
						mVelocityConstraints
					);
					mP.assign(freeGradient);
				}
			}
			
			mImpulseC.reset();
			calculateImpulse(
				mImpulseC,
				mLambda,
				mD_inv,
				mB,
				mVelocityConstraints
			);
			
			
 
			updateVelocity(
				this->inFixedTag()->getData(),
				this->inVelocity()->getData(),
				this->inAngularVelocity()->getData(),
				mImpulseC,
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

		}
		else
		{
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
		}
	}

	DEFINE_CLASS(PCGConstraintSolver);
}