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
#include "CollisionData.h"
#include "Attribute.h"

#include "Module/ComputeModule.h"

#include "Topology/DiscreteElements.h"
#include "Topology/TriangleSet.h"
#include "Topology/LinearBVH.h"

namespace dyno
{
	/**
	 * @brief A contact manifold between two shapes, carrying the shape ids so that contacts can be matched across frames.
	 */
	template<typename Real>
	struct TContactManifold
	{
		static const int MAX_POINTS = 4;

		/**
		 * Shape ids in a unified index space: [0, DiscreteElements::totalSize()) are elements of DiscreteElements,
		 * ids starting from DiscreteElements::totalSize() are triangles of the environment mesh.
		 */
		int shapeId1 = INVALID;
		int shapeId2 = INVALID;

		/**
		 * Rigid body ids, INVALID for the environment mesh. bodyId1 is never INVALID.
		 */
		int bodyId1 = INVALID;
		int bodyId2 = INVALID;

		/**
		 * Unit normal pointing from shape 1 to shape 2
		 */
		Vector<Real, 3> normal;

		/**
		 * World-space contact points, located on the surface of shape 2
		 */
		Vector<Real, 3> points[MAX_POINTS];

		/**
		 * Signed separation along the normal, negative when penetrating
		 */
		Real separation[MAX_POINTS];

		int count = 0;

		DYN_FUNC inline unsigned long long key() const {
			return ((unsigned long long)(uint)shapeId1 << 32) | (unsigned long long)(uint)shapeId2;
		}
	};

	/**
	 * @brief Contact generation for rigid bodies.
	 *
	 *	Broad phase: two linear BVHs, one for static shapes (fixed rigid bodies and the environment mesh) and one for
	 *	dynamic shapes. The dynamic tree is refitted every frame and rebuilt every RebuildInterval frames,
	 *	the static tree is rebuilt only when its content changes. Same-body, collision-mask and static-static pairs
	 *	are culled during traversal.
	 *
	 *	Narrow phase: pairs are sorted by their primitive type pair and every bucket is processed by its own kernel,
	 *	so that no kernel pays for the registers and divergence of all other pair types. Pairs without a dedicated
	 *	routine fall back to GJK/EPA on the convex hull of spheres.
	 *
	 *	Output: one manifold with up to four points per shape pair, plus the legacy per-point TContactPair array
	 *	so that the existing constraint solvers can consume the result unchanged.
	 */
	template<typename TDataType>
	class RigidContactDetection : public ComputeModule
	{
		DECLARE_TCLASS(RigidContactDetection, TDataType)

	public:
		typedef typename TDataType::Real Real;
		typedef typename TDataType::Coord Coord;
		typedef typename ::dyno::TAlignedBox3D<Real> AABB;
		typedef typename ::dyno::TContactManifold<Real> Manifold;

		RigidContactDetection();
		~RigidContactDetection() override;

	public:
		DEF_VAR(bool, SelfCollision, true, "Whether rigid bodies sharing the same object id collide with each other");

		DEF_VAR(Real, DHead, 0.0f, "Contact margin, contacts are generated when two shapes are closer than twice this value");

		DEF_VAR(uint, RebuildInterval, 8, "Rebuild the dynamic BVH every n frames and refit it in between");

		DEF_INSTANCE_IN(DiscreteElements<TDataType>, DiscreteElements, "");

		DEF_ARRAY_IN(CollisionMask, CollisionMask, DeviceType::GPU, "");

		DEF_ARRAY_IN(Attribute, Attribute, DeviceType::GPU, "");

		DEF_INSTANCE_IN(TriangleSet<TDataType>, TriangleSet, "Static environment mesh");

		DEF_ARRAY_OUT(Manifold, Manifolds, DeviceType::GPU, "Contact manifolds");

		DEF_ARRAY_OUT(TContactPair<Real>, Contacts, DeviceType::GPU, "Per-point contacts in the legacy format");

	protected:
		void compute() override;

	private:
		void updateBroadPhase(uint elementNum, uint meshTriNum);
		void findPairs(uint elementNum);
		void narrowPhase(uint elementNum);
		void writeLegacyContacts();
		void updateMeshEdges(DArray<Topology::Triangle>& indices);

		// Shape data
		DArray<AABB> mShapeAABB;		// AABBs of all shapes, elements first, then environment triangles
		DArray<int> mShapeBody;			// rigid body id of each shape, INVALID for the environment
		DArray<int> mShapeIsStatic;		// 1 for shapes of fixed rigid bodies and the environment

		DArray<int> mDynamicShapes;		// shape ids indexed by leaf id in the dynamic BVH
		DArray<int> mStaticShapes;		// shape ids indexed by leaf id in the static BVH
		DArray<AABB> mDynamicAABB;
		DArray<AABB> mStaticAABB;

		LinearBVH<TDataType> mDynamicBVH;
		LinearBVH<TDataType> mStaticBVH;
		uint mFrameSinceRebuild = 0;
		bool mStaticDirty = true;

		// Environment mesh in world space
		DArray<Triangle3D> mMeshTriangles;
		DArray<int> mMeshEdgeActive;		// per triangle edge, 0 for internal edges

		// Pairs
		DArray<unsigned long long> mPairKeys;
		DArray<uint> mPairCounter;
		uint mPairCapacity = 0;

		DArray<Manifold> mManifoldBuffer;
		DArray<uint> mBucketStart;

		DArray<uint> mPointCount;
		Scan<uint> mScan;
		Reduction<uint> mReduce;
	};
}
