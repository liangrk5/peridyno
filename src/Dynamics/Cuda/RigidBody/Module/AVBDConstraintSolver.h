/**
 * Copyright 2026 PeriDyno contributors
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
 */
#pragma once
#include "Module/ConstraintModule.h"

#include "Topology/DiscreteElements.h"
#include "Collision/Attribute.h"
#include "Collision/CollisionData.h"
#include "Collision/RigidContactDetection.h"

#include "AVBDCore.h"

namespace dyno
{
	/**
	 * @brief Augmented Vertex Block Descent for rigid bodies (Giles et al., SIGGRAPH 2025).
	 *
	 *	Every iteration solves one 6x6 Newton system per body, bodies of the same color are processed in parallel
	 *	without atomics: each thread gathers the forces of the manifolds adjacent to its body through a CSR adjacency.
	 *	Colors are computed on the GPU and reused across frames, so that a coherent scene converges in a round or two.
	 *	Contacts are hard constraints handled by an augmented Lagrangian with a ramped penalty, lambda and penalty
	 *	are warm started across frames by matching manifolds on their shape ids.
	 */
	template<typename TDataType>
	class AVBDConstraintSolver : public ConstraintModule
	{
		DECLARE_TCLASS(AVBDConstraintSolver, TDataType)
	public:
		typedef typename TDataType::Real Real;
		typedef typename TDataType::Coord Coord;
		typedef typename TDataType::Matrix Matrix;

		typedef typename ::dyno::Quat<Real> TQuat;
		typedef typename ::dyno::TContactPair<Real> ContactPair;
		typedef typename ::dyno::TContactManifold<Real> ContactManifold;
		typedef typename ::dyno::AVBDManifold<Real> Manifold;

		AVBDConstraintSolver();
		~AVBDConstraintSolver() override;

	public:
		DEF_VAR(bool, FrictionEnabled, true, "");

		DEF_VAR(bool, GravityEnabled, true, "");

		DEF_VAR(Real, GravityValue, 9.8, "Gravity acceleration along -y");

		DEF_VAR(uint, IterationNumber, 10, "Number of AVBD iterations per substep");

		DEF_VAR(uint, SubStepping, 1, "");

		DEF_VAR(Real, Alpha, 0.95, "Stabilization, the fraction of the error at the beginning of a step that is not corrected within the step");

		DEF_VAR(Real, Gamma, 0.99, "Warm start decay of lambda and penalty");

		DEF_VAR(Real, BetaLinear, 100000, "Penalty ramping rate of linear constraints, per unit mass");

		DEF_VAR(Real, BetaAngular, 100, "Penalty ramping rate of angular constraints");

		DEF_VAR(Real, PenaltyMin, 1, "Per unit mass");

		DEF_VAR(Real, PenaltyMax, 1e10, "Per unit mass");

		DEF_VAR(Real, ContactMargin, 0.002, "Penetration allowed at rest, it keeps resting contacts alive between frames");

		DEF_VAR(Real, StickThreshold, 1e-5, "Tangential drift below which a contact is considered sticking and keeps its anchors");

		DEF_VAR(Real, WarmStartDistance, 0.02, "Contacts closer than this distance are matched with the ones of the previous frame");

	public:
		DEF_VAR_IN(Real, TimeStep, "Time step size");

		DEF_ARRAY_IN(Real, Mass, DeviceType::GPU, "Mass of rigid bodies");

		DEF_ARRAY_IN(Coord, Center, DeviceType::GPU, "Center of rigid bodies");

		DEF_ARRAY_IN(Coord, Velocity, DeviceType::GPU, "Velocity of rigid bodies");

		DEF_ARRAY_IN(Coord, AngularVelocity, DeviceType::GPU, "Angular velocity of rigid bodies");

		DEF_ARRAY_IN(Matrix, RotationMatrix, DeviceType::GPU, "Rotation matrix of rigid bodies");

		DEF_ARRAY_IN(Matrix, Inertia, DeviceType::GPU, "Inertia matrix in world space");

		DEF_ARRAY_IN(Matrix, InitialInertia, DeviceType::GPU, "Inertia matrix in the local frame");

		DEF_ARRAY_IN(TQuat, Quaternion, DeviceType::GPU, "Quaternion");

		DEF_ARRAY_IN(ContactManifold, Manifolds, DeviceType::GPU, "Contact manifolds from RigidContactDetection");

		DEF_ARRAY_IN(ContactPair, Contacts, DeviceType::GPU, "Additional contacts in the legacy format, each becomes a single point manifold");

		DEF_INSTANCE_IN(DiscreteElements<TDataType>, DiscreteElements, "");

		DEF_ARRAY_IN(Attribute, Attribute, DeviceType::GPU, "Rigid body attributes");

		DEF_ARRAY_IN(Real, FrictionCoefficients, DeviceType::GPU, "FrictionCoefficients of rigid bodies");

	protected:
		void constrain() override;

	private:
		void resizeBodyBuffers(uint bodyNum);
		void setupManifolds();
		void warmStart();
		void buildAdjacency(uint bodyNum);
		void colorBodies(uint bodyNum);
		void solveSubstep(Real h);
		void saveManifolds();

		// Body states, x0/q0 hold the pose at the beginning of the step, xHat/qHat the inertial target
		DArray<Coord> mX0;
		DArray<TQuat> mQ0;
		DArray<Coord> mXHat;
		DArray<TQuat> mQHat;
		DArray<Coord> mPrevVelocity;

		// Manifolds of the current and the previous frame
		DArray<Manifold> mManifolds;
		DArray<Manifold> mOldManifolds;
		DArray<unsigned long long> mOldKeys;
		DArray<unsigned long long> mSortKeys;
		DArray<uint> mSortIds;

		// Body to manifold adjacency in the CSR format, each entry is (manifold id << 1 | side)
		DArray<unsigned long long> mAdjKeys;
		DArray<uint> mAdjStart;
		DArray<uint> mAdj;

		// Graph coloring
		DArray<int> mColor;			// final color per body, kept across frames
		DArray<int> mTentative;
		DArray<int> mResolved;
		DArray<uint> mCounter;
		DArray<unsigned long long> mColorKeys;
		DArray<uint> mColorBodies;	// dynamic bodies sorted by color
		DArray<uint> mColorStart;
		std::vector<uint> mColorStartHost;

		uint mManifoldNum = 0;
	};
}
