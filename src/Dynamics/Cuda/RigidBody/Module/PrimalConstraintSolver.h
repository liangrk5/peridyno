/**
 * Copyright 2024 Liang Ruikai
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once
#include "Module/ConstraintModule.h"
#include "RigidBody/RigidBodyShared.h"

#include "Topology/DiscreteElements.h"
#include "Collision/Attribute.h"
#include "Collision/CollisionData.h"

namespace dyno
{
    template<typename TDataType>
    class PrimalConstraintSolver :public ConstraintModule
    {
        DECLARE_TCLASS(PrimalConstraintSolver, TDataType)
    public:
		typedef typename TDataType::Real Real;
		typedef typename TDataType::Coord Coord;
		typedef typename TDataType::Matrix Matrix;

		typedef typename ::dyno::Quat<Real> TQuat;
		typedef typename ::dyno::TContactPair<Real> ContactPair;
		typedef typename ::dyno::TConstraintPair<Real> Constraint;

		typedef typename BallAndSocketJoint<Real> BallAndSocketJoint;
		typedef typename SliderJoint<Real> SliderJoint;
		typedef typename HingeJoint<Real> HingeJoint;
		typedef typename FixedJoint<Real> FixedJoint;
		typedef typename PointJoint<Real> PointJoint;

		PrimalConstraintSolver();
		~PrimalConstraintSolver();

	public:
		DEF_VAR(bool, FractureEnabled, true, "");

		DEF_VAR(bool, FrictionEnabled, true, "");

		DEF_VAR(bool, GravityEnabled, true, "");

		DEF_VAR(uint, AutoStiffnessType, 1, "0 use constant stiff, 1 use max effectMass, 2 use own effectMass");

		DEF_VAR(Real, GravityValue, 9.8, "");

		DEF_VAR(Real, FrictionStiffness, 500, "");

		DEF_VAR(Real, Stiffness, 10000000, "");


		DEF_VAR(Real, Hertz, 30, "");

		DEF_VAR(Real, Slop, 0, "");

		DEF_VAR(uint, IterationNumberForVelocitySolver, 30, "");

		DEF_VAR(Real, LinearDamping, 0.0, "");

		DEF_VAR(Real, AngularDamping, 0.0, "");

		DEF_VAR(uint, SubStep, 10, "");

		DEF_VAR(Real, StepSize, 0.6, "");

	public:
		DEF_VAR_IN(Real, TimeStep, "Time step size");

		DEF_ARRAY_IN(Real, Mass, DeviceType::GPU, "Mass of rigid bodies");

		DEF_ARRAY_IN(Coord, Center, DeviceType::GPU, "Center of rigid bodies");

		DEF_ARRAY_IN(Coord, Velocity, DeviceType::GPU, "Velocity of rigid bodies");

		DEF_ARRAY_IN(Coord, AngularVelocity, DeviceType::GPU, "Angular velocity of rigid bodies");

		DEF_ARRAY_IN(Matrix, RotationMatrix, DeviceType::GPU, "Rotation matrix of rigid bodies");

		DEF_ARRAY_IN(Matrix, Inertia, DeviceType::GPU, "Interial matrix");

		DEF_ARRAY_IN(Matrix, InitialInertia, DeviceType::GPU, "Interial matrix");

		DEF_ARRAY_IN(TQuat, Quaternion, DeviceType::GPU, "Quaternion");

		DEF_ARRAY_IN(ContactPair, Contacts, DeviceType::GPU, "");

		DEF_INSTANCE_IN(DiscreteElements<TDataType>, DiscreteElements, "");

		DEF_ARRAY_IN(Attribute, Attribute, DeviceType::GPU, "Rigid body attributes");

		DEF_ARRAY_IN(Real, FrictionCoefficients, DeviceType::GPU, "FrictionCoefficients of rigid bodies");
    
	protected:
		void constrain() override;

	private:
		void IterationOneStep(Real dt);

	private:
		DArray<Coord> mJ;
		DArray<Coord> mB;
		DArray<Real> mD;
		DArray<Real> mEffectMass;

		DArray<Coord> mImpulseC;
		DArray<Coord> mImpulseExt;

		DArray<Real> normalForces;

		DArray<ContactPair> mContactsInLocalFrame;

		DArray<Constraint> mVelocityConstraints;

		DArray<Coord> mCurrentCenter;
		DArray<Matrix> mCurrentRotationMatrix;
		DArray<Matrix> mCurrentInertia;
		DArray<TQuat> mCurrentQuat;

		DArray<Coord> initialVelocity;
		DArray<Coord> initialAngularVelocity;

		DArray<Coord> mP;
		DArray<Matrix> mPreconditioner;

		DArray<Coord> mGradient;

	};
}