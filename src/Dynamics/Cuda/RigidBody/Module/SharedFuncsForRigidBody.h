#pragma once
#include "Array/ArrayList.h"

#include "STL/Pair.h"

#include "Matrix/Transform3x3.h"

#include "Collision/CollisionData.h"

#include "Topology/DiscreteElements.h"

#include "Algorithm/Reduction.h"

#include "Collision/Attribute.h"

#include <algorithm>
#include <random>


namespace dyno 
{
	void ApplyTransform(
		DArrayList<Transform3f>& instanceTransform,
		const DArray<Vec3f>& diff,
		const DArray<Vec3f>& translate,
		const DArray<Mat3f>& rotation,
		const DArray<Mat3f>& rotationInit,
		const DArray<Pair<uint, uint>>& binding,
		const DArray<int>& bindingtag);

	void updateVelocity(
		DArray<int> tags,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray<Vec3f> impulse,
		float linearDamping,
		float angularDamping,
		float dt
	);

	void updateGesture(
		DArray<Vec3f> pos,
		DArray<Quat1f> rotQuat,
		DArray<Mat3f> rotMat,
		DArray<Mat3f> inertia,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray<Mat3f> inertia_init,
		float dt
	);

	void updatePositionAndRotation(
		DArray<Vec3f> pos,
		DArray<Quat1f> rotQuat,
		DArray<Mat3f> rotMat,
		DArray<Mat3f> inertia,
		DArray<Mat3f> inertia_init,
		DArray<Vec3f> impulse_constrain
	);

	void calculateContactPoints(
		DArray<TContactPair<float>> contacts,
		DArray<int> contactCnt
	);


	void calculateJacobianMatrix(
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<Vec3f> pos,
		DArray<Mat3f> inertia,
		DArray<float> mass,
		DArray<Mat3f> rotMat,
		DArray<TConstraintPair<float>> constraints
	);
	void calculateJacobianMatrixQuat(
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<Vec3f> pos,
		DArray<Mat3f> inertia,
		DArray<float> mass,
		DArray<Mat3f> rotMat,
		DArray<Quat1f> rotQuat,
		DArray<TConstraintPair<float>> constraints
	);

	void calculateJacobianMatrixForNJS(
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<Vec3f> pos,
		DArray<Mat3f> inertia,
		DArray<float> mass,
		DArray<Mat3f> rotMat,
		DArray<TConstraintPair<float>> constraints
	);


	void calculateEtaVectorForPJS(
		DArray<float> eta,
		DArray<Vec3f> J,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray<TConstraintPair<float>> constraints
	);

	void calculateEtaVectorForPJSBaumgarte(
		DArray<float> eta,
		DArray<Vec3f> J,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray<Vec3f> pos,
		DArray<Quat1f> rotation_q,
		DArray <TConstraintPair<float>> constraints,
		DArray<float> errors,
		float slop,
		float beta,
		float dt
	);

	void calculateEtaVectorForPJSBaumgarteQuat(
		DArray<float> eta,
		DArray<Vec3f> J,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray<Vec3f> pos,
		DArray<Quat1f> rotation_q,
		DArray<TConstraintPair<float>> constraints,
		DArray<float> errors,
		float slop,
		float beta,
		float dt
	);

	void calculateEtaVectorWithERP(
		DArray<float> eta,
		DArray<Vec3f> J,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray<Vec3f> pos,
		DArray<Quat1f> rotation_q,
		DArray <TConstraintPair<float>> constraints,
		DArray<float> ERP,
		float slop,
		float dt
	);

	void calculateEtaVectorForPJSoft(
		DArray<float> eta,
		DArray<Vec3f> J,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray<Vec3f> pos,
		DArray<Quat1f> rotation_q,
		DArray <TConstraintPair<float>> constraints,
		float slop,
		float zeta,
		float hertz,
		float substepping,
		float dt
	);
	
	void calculateEtaVectorForNJS(
		DArray<float> eta,
		DArray<Vec3f> J,
		DArray<Vec3f> pos,
		DArray<Quat1f> rotation_q,
		DArray <TConstraintPair<float>> constraints,
		float slop,
		float beta
	);
	
	void setUpContactsInLocalFrame(
		DArray<TContactPair<float>> contactsInLocalFrame,
		DArray<TContactPair<float>> contactsInGlobalFrame,
		DArray<Vec3f> pos,
		DArray<Mat3f> rotMat
	);
	
	void setUpContactAndFrictionConstraints(
		DArray<TConstraintPair<float>> constraints,
		DArray<TContactPair<float>> contactsInLocalFrame,
		DArray<Vec3f> pos,
		DArray<Mat3f> rotMat,
		bool hasFriction
	);

	void setUpContactAndFrictionConstraintsShuffle(
		DArray<TConstraintPair<float>> constraints,
		DArray<TContactPair<float>> contactsInLocalFrame,
		DArray<Vec3f> pos,
		DArray<Mat3f> rotMat,
		DArray<int> arr,
		bool hasFriction
	);
	
	void setUpContactConstraints(
		DArray<TConstraintPair<float>> constraints,
		DArray<TContactPair<float>> contactsInLocalFrame,
		DArray<Vec3f> pos,
		DArray<Mat3f> rotMat
	);

	void setUpBallAndSocketJointConstraints(
		DArray<TConstraintPair<float>> constraints,
		DArray<BallAndSocketJoint<float>> joints,
		DArray<Vec3f> pos,
		DArray<Mat3f> rotMat,
		int begin_index
	);
	
	void setUpSliderJointConstraints(
		DArray<TConstraintPair<float>> constraints,
		DArray<SliderJoint<float>> joints,
		DArray<Vec3f> pos,
		DArray<Mat3f> rotMat,
		int begin_index
	);

	void setUpHingeJointConstraints(
		DArray<TConstraintPair<float>> constraints,
		DArray<HingeJoint<float>> joints,
		DArray<Vec3f> pos,
		DArray<Mat3f> rotMat,
		DArray<Quat1f> rotation_q,
		int begin_index
	);

	void damgedHingeJointConstraints(
		DArray<HingeJoint<float>>& joints,
		DArray<float>& lambda,
		DArray<Vec3f>& B,
		DArray<TConstraintPair<float>>& constraints,
		DArray<float>& mass,
		int begin_index,
		Real dt
	);

	void damgedFixedJointConstraints(
		DArray<FixedJoint<float>>& joints,
		DArray<float>& lambda,
		DArray<Vec3f>& B,
		DArray<TConstraintPair<float>>& constraints,
		DArray<float>& mass,
		int begin_index,
		Real dt
	);

	void setUpFixedJointConstraints(
		DArray<TConstraintPair<float>> constraints,
		DArray<FixedJoint<float>> joints,
		DArray<Mat3f> rotMat,
		DArray<Quat1f> rotQuat,
		int begin_index
	);

	void setUpFixedJointConstraintsQuat(
		DArray<TConstraintPair<float>> constraints,
		DArray<FixedJoint<float>> joints,
		DArray<Mat3f> rotMat,
		DArray<Quat1f> rotQuat,
		int begin_index
	);

	void setUpPointJointConstraints(
		DArray<TConstraintPair<float>> constraints,
		DArray<PointJoint<float>> joints,
		DArray<Vec3f> pos,
		int begin_index
	);

	void calculateK(
		DArray<TConstraintPair<float>> constraints,
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<Vec3f> pos,
		DArray<Mat3f> inertia,
		DArray<float> mass,
		DArray<float> K_1,
		DArray<Mat2f> K_2,
		DArray<Mat3f> K_3
	);

	void calculateKWithCFM(
		DArray<TConstraintPair<float>> constraints,
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<Vec3f> pos,
		DArray<Mat3f> inertia,
		DArray<float> mass,
		DArray<float> K_1,
		DArray<Mat2f> K_2,
		DArray<Mat3f> K_3,
		DArray<float> CFM
	);


	void JacobiIteration(
		DArray<float> lambda,
		DArray<Vec3f> impulse,
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<float> eta,
		DArray<TConstraintPair<float>> constraints,
		DArray<int> nbq,
		DArray<float> K_1,
		DArray<Mat2f> K_2,
		DArray<Mat3f> K_3,
		DArray<float> mass,
		float mu,
		float g,
		float dt
	);

	void JacobiIterationShuffle(
		DArray<float> lambda,
		DArray<Vec3f> impulse,
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<float> eta,
		DArray<TConstraintPair<float>> constraints,
		DArray<int> nbq,
		DArray<float> K_1,
		DArray<Mat2f> K_2,
		DArray<Mat3f> K_3,
		DArray<float> mass,
		DArray<int> tag,
		float mu,
		float g,
		float dt
	);

	void JacobiIterationForCFM(
		DArray<float> lambda,
		DArray<Vec3f> impulse,
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<float> eta,
		DArray<TConstraintPair<float>> constraints,
		DArray<int> nbq,
		DArray<float> K_1,
		DArray<Mat2f> K_2,
		DArray<Mat3f> K_3,
		DArray<float> mass,
		DArray<float> CFM,
		float mu,
		float g,
		float dt
	);

	void JacobiIterationForSoft(
		DArray<float> lambda,
		DArray<Vec3f> impulse,
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<float> eta,
		DArray<TConstraintPair<float>> constraints,
		DArray<int> nbq,
		DArray<float> K_1,
		DArray<Mat2f> K_2,
		DArray<Mat3f> K_3,
		DArray<float> mass,
		float mu,
		float g,
		float dt,
		float zeta,
		float hertz
	);

	void JacobiIterationForNJS(
		DArray<float> lambda,
		DArray<Vec3f> impulse,
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<float> eta,
		DArray<TConstraintPair<float>> constraints,
		DArray<int> nbq,
		DArray<float> K_1,
		DArray<Mat2f> K_2,
		DArray<Mat3f> K_3
	);

	void setUpGravity(
		DArray<Vec3f> impulse_ext,
		float g,
		float dt
	);

	void calculateDiagnals(
		DArray<float> d,
		DArray<Vec3f> J,
		DArray<Vec3f> B
	);

	Real calculateDiagnalsMax(
		DArray<float> d,
		DArray<Vec3f> J,
		DArray<Vec3f> B
	);

	void calculateDiagnalsInv(
		DArray<float>& d_inv,
		DArray<Vec3f>& J,
		DArray<Vec3f>& B,
		DArray<float>& CFM,
		DArray<int>& nbq,
		DArray<TConstraintPair<float>>& constraints
	);

	void preConditionJ(
		DArray<Vec3f> J,
		DArray<float> d,
		DArray<float> eta
	);

	bool saveVectorToFile(
		const std::vector<float>& vec,
		const std::string& filename
	);


	void calculateEtaVectorForRelaxation(
		DArray<float> eta,
		DArray<Vec3f> J,
		DArray<Vec3f> velocity,
		DArray<Vec3f> angular_velocity,
		DArray <TConstraintPair<float>> constraints
	);
	

	bool saveMatrixToFile(
		DArray<float> &Matrix,
		int n,
		const std::string& filename
	);

	bool saveVectorToFile(
		DArray<float>& vec,
		const std::string& filename
	);

	void buildCFMAndERP(
		DArray<Vec3f> J,
		DArray<Vec3f> B,
		DArray<TConstraintPair<float>> constraints,
		DArray<float> CFM,
		DArray<float> ERP,
		float hertz,
		float zeta,
		float dt
	);

	void vectorAdd(
		DArray<float> &result,
		DArray<float> &arr1,
		DArray<float> &arr2
	);

	void vectorSub(
		DArray<float>& result,
		DArray<float>& arr1,
		DArray<float>& arr2
	);

	void preconditionedResidual(
		DArray<float>& residual,
		DArray<float>& result,
		DArray<float>& K_1,
		DArray<Mat2f>& K_2,
		DArray<Mat3f>& K_3,
		DArray<TConstraintPair<float>>& constraints
	);

	void vectorMultiplyScale(
		DArray<float> &result,
		DArray<float> &initArr,
		float scale
	);

	float innerDotOfVector(
		DArray<float> &arr1,
		DArray<float> &arr2
	);


	
	void calculateAx(
		DArray<float>& Ax,
		DArray<Vec3f>& impulse,
		DArray<Vec3f>& J,
		DArray<Vec3f>& B,
		DArray<float>& lambda,
		DArray<float>& CFM,
		DArray<TConstraintPair<float>>& constraints
	);

	void calculateAxWithoutCFM(
		DArray<float>& Ax,
		DArray<Vec3f>& impulse,
		DArray<Vec3f>& J,
		DArray<Vec3f>& B,
		DArray<float>& lambda,
		DArray<TConstraintPair<float>>& constraints
	);

	float calculateSpectralRadius(
		DArray<float>& Ax,
		DArray<Vec3f>& impulse,
		DArray<Vec3f>& J,
		DArray<Vec3f>& B,
		DArray<float>& lambda,
		DArray<TConstraintPair<float>>& constraints
	);


	int projectionLambda(
		DArray<float>& lambda,
		DArray<TConstraintPair<float>>& constraints,
		float mu,
		int contact_size
	);

	void calculateImpulse(
		DArray<Vec3f>& impulse,
		DArray<float>& lambda,
		DArray<float>& d_inv,
		DArray<Vec3f>& B,
		DArray<TConstraintPair<float>>& constraints
	);

	void initLambda(
		float initValue,
		DArray<float>& lambda
	);

	std::vector<int> generatePermutation(int n);

	void generatePermutationDArray(DArray<int> arr);

	void calculateImpulseByLambda(
		DArray<float> &lambda,
		DArray<TConstraintPair<float>> &constraints,
		DArray<Vec3f> &impulse,
		DArray<Vec3f> &B
	);
}
