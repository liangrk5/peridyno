#include "RigidContactDetection.h"

#include "CollisionDetectionAlgorithm.h"
#include "ConvexCollision.h"

#include "STL/Stack.h"

#include <thrust/device_ptr.h>
#include <thrust/transform_reduce.h>
#include <thrust/functional.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/count.h>
#include <thrust/reduce.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/counting_iterator.h>

namespace dyno
{
	IMPLEMENT_TCLASS(RigidContactDetection, TDataType)

	// Narrow phase type order, pairs are canonicalized so that the first shape has the smaller type index
	enum NarrowType
	{
		NT_BOX = 0,
		NT_TET,
		NT_CAPSULE,
		NT_SPHERE,
		NT_TRIANGLE,
		NT_MEDIALCONE,
		NT_MEDIALSLAB,
		NT_COUNT,
		NT_INVALID
	};

	DYN_FUNC inline int narrowType(ElementType type)
	{
		switch (type)
		{
		case ET_BOX: return NT_BOX;
		case ET_TET: return NT_TET;
		case ET_CAPSULE: return NT_CAPSULE;
		case ET_SPHERE: return NT_SPHERE;
		case ET_TRI: return NT_TRIANGLE;
		case ET_MEDIALCONE: return NT_MEDIALCONE;
		case ET_MEDIALSLAB: return NT_MEDIALSLAB;
		default: return NT_INVALID;
		}
	}

	DYN_FUNC inline ElementType shapeElementType(ElementOffset& offset, uint shapeId, uint elementNum)
	{
		return shapeId < elementNum ? offset.checkElementType(shapeId) : ET_TRI;
	}

	// Pair keys: [63:58] type bucket, [57:29] first shape id, [28:0] second shape id
	const int PAIR_ID_BITS = 29;
	const unsigned long long PAIR_ID_MASK = (1ull << PAIR_ID_BITS) - 1;
	const int PAIR_BUCKET_SHIFT = 2 * PAIR_ID_BITS;
	const int PAIR_BUCKET_NUM = NT_COUNT * NT_COUNT;

	DYN_FUNC inline unsigned long long encodePair(uint bucket, uint a, uint b)
	{
		return ((unsigned long long)bucket << PAIR_BUCKET_SHIFT) | ((unsigned long long)a << PAIR_ID_BITS) | (unsigned long long)b;
	}

	DYN_FUNC inline uint pairBucket(unsigned long long key) { return (uint)(key >> PAIR_BUCKET_SHIFT); }
	DYN_FUNC inline uint pairFirst(unsigned long long key) { return (uint)((key >> PAIR_ID_BITS) & PAIR_ID_MASK); }
	DYN_FUNC inline uint pairSecond(unsigned long long key) { return (uint)(key & PAIR_ID_MASK); }

	template<typename TDataType>
	RigidContactDetection<TDataType>::RigidContactDetection()
		: ComputeModule()
	{
		this->inCollisionMask()->tagOptional(true);
		this->inAttribute()->tagOptional(true);
		this->inTriangleSet()->tagOptional(true);

		mPairCounter.resize(1);
	}

	template<typename TDataType>
	RigidContactDetection<TDataType>::~RigidContactDetection()
	{
		mDynamicBVH.release();
		mStaticBVH.release();
	}

	//---------------------------------------------------------------------------------------------------------------
	// Shape setup
	//---------------------------------------------------------------------------------------------------------------

	template<typename AABB, typename Box3D>
	__global__ void RCD_SetupShapes(
		DArray<AABB> shapeAABB,
		DArray<int> shapeBody,
		DArray<int> shapeIsStatic,
		DArray<Box3D> boxes,
		DArray<Sphere3D> spheres,
		DArray<Tet3D> tets,
		DArray<Capsule3D> caps,
		DArray<Triangle3D> tris,
		DArray<MedialCone3D> cones,
		DArray<MedialSlab3D> slabs,
		DArray<Triangle3D> meshTris,
		DArray<Pair<uint, uint>> shape2RigidBody,
		DArray<Attribute> attribute,
		ElementOffset offset,
		uint elementNum,
		bool hasAttribute,
		Real margin)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= shapeAABB.size()) return;

		AABB box;
		int body = INVALID;
		int isStatic = 1;

		if (tId < elementNum)
		{
			switch (offset.checkElementType(tId))
			{
			case ET_SPHERE: box = spheres[tId - offset.sphereIndex()].aabb(); break;
			case ET_BOX: box = boxes[tId - offset.boxIndex()].aabb(); break;
			case ET_TET: box = tets[tId - offset.tetIndex()].aabb(); break;
			case ET_CAPSULE: box = caps[tId - offset.capsuleIndex()].aabb(); break;
			case ET_TRI: box = tris[tId - offset.triangleIndex()].aabb(); break;
			case ET_MEDIALCONE: box = cones[tId - offset.medialConeIndex()].aabb(); break;
			case ET_MEDIALSLAB: box = slabs[tId - offset.medialSlabIndex()].aabb(); break;
			default: break;
			}

			body = shape2RigidBody[tId].second;
			isStatic = hasAttribute && attribute[body].isFixed() ? 1 : 0;
		}
		else
		{
			box = meshTris[tId - elementNum].aabb();
		}

		box.v0 -= margin;
		box.v1 += margin;

		shapeAABB[tId] = box;
		shapeBody[tId] = body;
		shapeIsStatic[tId] = isStatic;
	}

	template<typename Coord>
	__global__ void RCD_SetupMeshTriangles(
		DArray<Triangle3D> meshTris,
		DArray<Coord> vertices,
		DArray<Topology::Triangle> indices)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= meshTris.size()) return;

		auto t = indices[tId];
		meshTris[tId] = Triangle3D(vertices[t[0]], vertices[t[1]], vertices[t[2]]);
	}

	template<typename AABB>
	__global__ void RCD_GatherAABB(
		DArray<AABB> dst,
		DArray<AABB> src,
		DArray<int> ids)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= ids.size()) return;

		dst[tId] = src[ids[tId]];
	}

	struct RCD_IsStatic
	{
		const int* flags;
		DYN_FUNC bool operator()(const int id) const { return flags[id] != 0; }
	};

	struct RCD_IsDynamic
	{
		const int* flags;
		DYN_FUNC bool operator()(const int id) const { return flags[id] == 0; }
	};

	/**
	 * @brief Order-independent signature of an id list, used to detect membership changes between frames
	 */
	struct RCD_IdHash
	{
		DYN_FUNC unsigned long long operator()(const int id) const { return (unsigned long long)id * 2654435761ull + 1ull; }
	};

	static unsigned long long listSignature(DArray<int>& ids)
	{
		if (ids.size() == 0) return 0;

		thrust::device_ptr<int> p(ids.begin());
		unsigned long long sum = thrust::transform_reduce(thrust::device, p, p + ids.size(),
			RCD_IdHash(),
			0ull, thrust::plus<unsigned long long>());
		return sum ^ ((unsigned long long)ids.size() << 48);
	}

	template<typename TDataType>
	void RigidContactDetection<TDataType>::updateBroadPhase(uint elementNum, uint meshTriNum)
	{
		uint total = elementNum + meshTriNum;

		// Split shapes into dynamic and static sets
		uint staticNum = thrust::count(thrust::device, thrust::device_ptr<int>(mShapeIsStatic.begin()), thrust::device_ptr<int>(mShapeIsStatic.begin()) + total, 1);
		uint dynamicNum = total - staticNum;

		unsigned long long oldDynamicSig = listSignature(mDynamicShapes);
		unsigned long long oldStaticSig = listSignature(mStaticShapes);

		mDynamicShapes.resize(dynamicNum);
		mStaticShapes.resize(staticNum);

		thrust::counting_iterator<int> first(0);
		thrust::copy_if(thrust::device, first, first + total, thrust::device_ptr<int>(mDynamicShapes.begin()), RCD_IsDynamic{ mShapeIsStatic.begin() });
		thrust::copy_if(thrust::device, first, first + total, thrust::device_ptr<int>(mStaticShapes.begin()), RCD_IsStatic{ mShapeIsStatic.begin() });

		mDynamicAABB.resize(dynamicNum);
		mStaticAABB.resize(staticNum);

		if (dynamicNum > 0)
		{
			cuExecute(dynamicNum, RCD_GatherAABB, mDynamicAABB, mShapeAABB, mDynamicShapes);

			bool membershipChanged = oldDynamicSig != listSignature(mDynamicShapes) || mDynamicBVH.size() != dynamicNum;
			uint interval = this->varRebuildInterval()->getValue();
			if (membershipChanged || mFrameSinceRebuild + 1 >= interval)
			{
				mDynamicBVH.construct(mDynamicAABB);
				mFrameSinceRebuild = 0;
			}
			else
			{
				mDynamicBVH.refit(mDynamicAABB);
				mFrameSinceRebuild++;
			}
		}

		if (staticNum > 0)
		{
			cuExecute(staticNum, RCD_GatherAABB, mStaticAABB, mShapeAABB, mStaticShapes);

			bool membershipChanged = oldStaticSig != listSignature(mStaticShapes) || mStaticBVH.size() != staticNum;
			if (membershipChanged || mStaticDirty)
			{
				mStaticBVH.construct(mStaticAABB);
				mStaticDirty = false;
			}
			else
			{
				// Fixed bodies may still be moved by the user, a refit is cheap compared with a rebuild
				mStaticBVH.refit(mStaticAABB);
			}
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	// Pair finding
	//---------------------------------------------------------------------------------------------------------------

	DYN_FUNC inline bool RCD_CheckMask(CollisionMask m0, CollisionMask m1, ElementType t0, ElementType t1)
	{
		return (m0 & t1) != 0 && (m1 & t0) != 0;
	}

	struct RCD_PairFilter
	{
		DArray<int> shapeBody;
		DArray<CollisionMask> mask;
		DArray<Attribute> attribute;
		ElementOffset offset;
		uint elementNum;
		bool hasMask;
		bool hasAttribute;
		bool selfCollision;

		GPU_FUNC bool accept(uint a, uint b)
		{
			int bodyA = shapeBody[a];
			int bodyB = shapeBody[b];

			if (bodyA == bodyB) return false;

			if (hasMask)
			{
				CollisionMask mA = bodyA == INVALID ? CT_AllObjects : mask[bodyA];
				CollisionMask mB = bodyB == INVALID ? CT_AllObjects : mask[bodyB];
				if (!RCD_CheckMask(mA, mB, shapeElementType(offset, a, elementNum), shapeElementType(offset, b, elementNum)))
					return false;
			}

			if (!selfCollision && hasAttribute && bodyA != INVALID && bodyB != INVALID)
			{
				if (attribute[bodyA].objectId() == attribute[bodyB].objectId())
					return false;
			}

			return true;
		}

		GPU_FUNC unsigned long long makeKey(uint a, uint b)
		{
			int ta = narrowType(shapeElementType(offset, a, elementNum));
			int tb = narrowType(shapeElementType(offset, b, elementNum));

			if (ta > tb || (ta == tb && a > b))
			{
				uint tmp = a; a = b; b = tmp;
				int t = ta; ta = tb; tb = t;
			}

			return encodePair(ta * NT_COUNT + tb, a, b);
		}
	};

	/**
	 * @brief Visit all leaves of the BVH overlapping the query box. visit(leafId) is called with the leaf index
	 *	in [0, bvh.leafNumber())
	 */
	template<typename BVH, typename AABB, typename Visitor>
	GPU_FUNC void RCD_Traverse(const BVH& bvh, const AABB& query, Visitor& visit)
	{
		uint N = bvh.leafNumber();
		if (N == 0) return;

		if (N == 1)
		{
			if (query.checkOverlap(bvh.getAABB(0))) visit(bvh.getObjectIdx(0));
			return;
		}

		int buffer[64];
		Stack<int> stack;
		stack.reserve(buffer, 64);

		int idx = 0;
		do
		{
			BVHNode node = bvh.getNode(idx);

			int children[2] = { node.left, node.right };
			bool traverse[2] = { false, false };

			for (int c = 0; c < 2; c++)
			{
				int child = children[c];
				if (child == EMPTY || !query.checkOverlap(bvh.getAABB(child))) continue;

				if (child >= (int)N - 1)
					visit(bvh.getObjectIdx(child - (N - 1)));
				else
					traverse[c] = true;
			}

			if (!traverse[0] && !traverse[1]) {
				idx = !stack.empty() ? stack.top() : EMPTY;
				stack.pop();
			}
			else
			{
				idx = traverse[0] ? children[0] : children[1];
				if (traverse[0] && traverse[1])
					stack.push(children[1]);
			}
		} while (idx != EMPTY);
	}

	template<typename AABB, typename BVH>
	__global__ void RCD_FindPairs(
		DArray<unsigned long long> pairKeys,
		DArray<uint> counter,
		BVH dynamicBVH,
		BVH staticBVH,
		DArray<int> dynamicShapes,
		DArray<int> staticShapes,
		DArray<AABB> dynamicAABB,
		RCD_PairFilter filter)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= dynamicShapes.size()) return;

		uint shapeA = dynamicShapes[tId];
		AABB box = dynamicAABB[tId];
		uint capacity = pairKeys.size();

		auto emit = [&](uint shapeB) {
			if (!filter.accept(shapeA, shapeB)) return;

			uint slot = atomicAdd(&counter[0], 1u);
			if (slot < capacity)
				pairKeys[slot] = filter.makeKey(shapeA, shapeB);
		};

		// Dynamic vs dynamic: report each pair once, from the shape with the smaller id
		auto visitDynamic = [&](uint leaf) {
			uint shapeB = dynamicShapes[leaf];
			if (shapeB > shapeA) emit(shapeB);
		};
		RCD_Traverse(dynamicBVH, box, visitDynamic);

		// Dynamic vs static
		auto visitStatic = [&](uint leaf) {
			emit(staticShapes[leaf]);
		};
		RCD_Traverse(staticBVH, box, visitStatic);
	}

	template<typename TDataType>
	void RigidContactDetection<TDataType>::findPairs(uint elementNum)
	{
		auto topo = this->inDiscreteElements()->constDataPtr();

		RCD_PairFilter filter;
		filter.shapeBody = mShapeBody;
		filter.offset = topo->calculateElementOffset();
		filter.elementNum = elementNum;
		filter.hasMask = !this->inCollisionMask()->isEmpty();
		filter.hasAttribute = !this->inAttribute()->isEmpty();
		filter.selfCollision = this->varSelfCollision()->getValue();
		if (filter.hasMask) filter.mask = this->inCollisionMask()->getData();
		if (filter.hasAttribute) filter.attribute = this->inAttribute()->getData();

		uint dynamicNum = mDynamicShapes.size();
		if (dynamicNum == 0) {
			mPairKeys.resize(0);
			return;
		}

		if (mPairCapacity == 0)
			mPairCapacity = 8 * dynamicNum;

		// The pair count is unknown in advance: write with an atomic counter and rerun once if the buffer overflowed
		for (int attempt = 0; attempt < 2; attempt++)
		{
			mPairKeys.resize(mPairCapacity);
			mPairCounter.reset();

			cuExecute(dynamicNum,
				RCD_FindPairs,
				mPairKeys,
				mPairCounter,
				mDynamicBVH,
				mStaticBVH,
				mDynamicShapes,
				mStaticShapes,
				mDynamicAABB,
				filter);

			CArray<uint> hCounter;
			hCounter.assign(mPairCounter);
			uint pairNum = hCounter[0];

			if (pairNum <= mPairCapacity)
			{
				mPairKeys.resize(pairNum);
				break;
			}

			mPairCapacity = pairNum + pairNum / 2;
		}

		// Sorting makes the result independent of the atomic write order and groups pairs by type
		if (mPairKeys.size() > 0)
		{
			thrust::device_ptr<unsigned long long> keys(mPairKeys.begin());
			thrust::sort(thrust::device, keys, keys + mPairKeys.size());
		}
	}

	//---------------------------------------------------------------------------------------------------------------
	// Narrow phase
	//---------------------------------------------------------------------------------------------------------------

	struct RCD_Shapes
	{
		DArray<OrientedBox3D> boxes;
		DArray<Sphere3D> spheres;
		DArray<Tet3D> tets;
		DArray<Capsule3D> caps;
		DArray<Triangle3D> tris;
		DArray<MedialCone3D> cones;
		DArray<MedialSlab3D> slabs;
		DArray<Triangle3D> meshTris;
		DArray<int> meshEdgeActive;		// per triangle edge, 0 for internal edges of the environment mesh
		DArray<int> shapeBody;
		ElementOffset offset;
		uint elementNum;
	};

	template<int T> struct RCD_ShapeOf;
	template<> struct RCD_ShapeOf<NT_BOX> { typedef OrientedBox3D Type; GPU_FUNC static Type get(RCD_Shapes& s, uint id) { return s.boxes[id - s.offset.boxIndex()]; } };
	template<> struct RCD_ShapeOf<NT_TET> { typedef Tet3D Type; GPU_FUNC static Type get(RCD_Shapes& s, uint id) { return s.tets[id - s.offset.tetIndex()]; } };
	template<> struct RCD_ShapeOf<NT_CAPSULE> { typedef Capsule3D Type; GPU_FUNC static Type get(RCD_Shapes& s, uint id) { return s.caps[id - s.offset.capsuleIndex()]; } };
	template<> struct RCD_ShapeOf<NT_SPHERE> { typedef Sphere3D Type; GPU_FUNC static Type get(RCD_Shapes& s, uint id) { return s.spheres[id - s.offset.sphereIndex()]; } };
	template<> struct RCD_ShapeOf<NT_TRIANGLE> { typedef Triangle3D Type; GPU_FUNC static Type get(RCD_Shapes& s, uint id) { return id < s.elementNum ? s.tris[id - s.offset.triangleIndex()] : s.meshTris[id - s.elementNum]; } };
	template<> struct RCD_ShapeOf<NT_MEDIALCONE> { typedef MedialCone3D Type; GPU_FUNC static Type get(RCD_Shapes& s, uint id) { return s.cones[id - s.offset.medialConeIndex()]; } };
	template<> struct RCD_ShapeOf<NT_MEDIALSLAB> { typedef MedialSlab3D Type; GPU_FUNC static Type get(RCD_Shapes& s, uint id) { return s.slabs[id - s.offset.medialSlabIndex()]; } };

	typedef CollisionDetection<Real> CD;

	/**
	 * @brief Swap the roles of A and B in a manifold: the normal flips and the points move from B's surface to A's
	 */
	DYN_FUNC inline void RCD_FlipManifold(TManifold<Real>& m)
	{
		for (int i = 0; i < m.contactCount; i++)
			m.contacts[i].position -= m.normal * m.contacts[i].penetration;
		m.normal = -m.normal;
	}

	/**
	 * @brief Generic convex fallback. Shapes are treated as inflated by dHat to match the dedicated routines.
	 */
	template<typename ShapeA, typename ShapeB>
	DYN_FUNC inline void RCD_Convex(TManifold<Real>& m, const ShapeA& a, const ShapeB& b, Real dHat)
	{
		TConvexContact<Real> c;
		if (ConvexCollision<Real>::query(c, makeSphereHull(a), makeSphereHull(b), 2 * dHat))
		{
			m.normal = c.normal;
			m.pushContact(c.pointB - c.normal * dHat, c.distance - 2 * dHat);
		}
	}

	// Dispatch for canonical pairs (type index of A <= type index of B). Returns the inflation applied to the pair,
	// i.e. separation = penetration + inflation.
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, OrientedBox3D a, OrientedBox3D b, Real d) { a.extent += d; b.extent += d; CD::request(m, a, b); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const OrientedBox3D& a, const Tet3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const OrientedBox3D& a, const Capsule3D& b, Real d) { CD::request(m, a, b.centerline(), d, b.radius + d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const OrientedBox3D& a, const Sphere3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const OrientedBox3D& a, const Triangle3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const OrientedBox3D& a, const MedialCone3D& b, Real d) { CD::request(m, a, b); return 0; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const OrientedBox3D& a, const MedialSlab3D& b, Real d) { CD::request(m, a, b); return 0; }

	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Tet3D& a, const Tet3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Tet3D& a, const Capsule3D& b, Real d) { CD::request(m, a, b.centerline(), d, b.radius + d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Tet3D& a, const Sphere3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Tet3D& a, const Triangle3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Tet3D& a, const MedialCone3D& b, Real d) { RCD_Convex(m, a, b, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Tet3D& a, const MedialSlab3D& b, Real d) { RCD_Convex(m, a, b, d); return 2 * d; }

	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Capsule3D& a, const Capsule3D& b, Real d) { CD::request(m, a.centerline(), b.centerline(), a.radius + d, b.radius + d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Capsule3D& a, const Sphere3D& b, Real d) { CD::request(m, a.centerline(), b, a.radius + d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Capsule3D& a, const Triangle3D& b, Real d) { CD::request(m, a.centerline(), b, a.radius + d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Capsule3D& a, const MedialCone3D& b, Real d)
	{
		// A capsule is a medial cone with equal radii
		MedialCone3D cone(a.startPoint(), a.endPoint(), a.radius, a.radius);
		CD::request(m, cone, b);
		return 0;
	}
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Capsule3D& a, const MedialSlab3D& b, Real d)
	{
		MedialCone3D cone(a.startPoint(), a.endPoint(), a.radius, a.radius);
		CD::request(m, cone, b);
		return 0;
	}

	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Sphere3D& a, const Sphere3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Sphere3D& a, const Triangle3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Sphere3D& a, const MedialCone3D& b, Real d) { RCD_Convex(m, a, b, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Sphere3D& a, const MedialSlab3D& b, Real d) { RCD_Convex(m, a, b, d); return 2 * d; }

	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Triangle3D& a, const Triangle3D& b, Real d) { CD::request(m, a, b, d, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Triangle3D& a, const MedialCone3D& b, Real d) { RCD_Convex(m, a, b, d); return 2 * d; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const Triangle3D& a, const MedialSlab3D& b, Real d) { RCD_Convex(m, a, b, d); return 2 * d; }

	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const MedialCone3D& a, const MedialCone3D& b, Real d) { CD::request(m, a, b); return 0; }
	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const MedialCone3D& a, const MedialSlab3D& b, Real d) { CD::request(m, a, b); return 0; }

	DYN_FUNC inline Real RCD_Collide(TManifold<Real>& m, const MedialSlab3D& a, const MedialSlab3D& b, Real d) { CD::request(m, a, b); return 0; }

	/**
	 * @brief Keep at most four points: the deepest one, the one farthest from it, then the points maximizing the area
	 */
	DYN_FUNC inline void RCD_ReducePoints(TManifold<Real>& m, int* selected, int& count)
	{
		int n = m.contactCount;
		if (n <= TContactManifold<Real>::MAX_POINTS)
		{
			for (int i = 0; i < n; i++) selected[i] = i;
			count = n;
			return;
		}

		int i0 = 0;
		for (int i = 1; i < n; i++)
			if (m.contacts[i].penetration < m.contacts[i0].penetration) i0 = i;

		Vec3f p0 = m.contacts[i0].position;

		int i1 = i0 == 0 ? 1 : 0;
		Real best = -1;
		for (int i = 0; i < n; i++)
		{
			Real d = (m.contacts[i].position - p0).normSquared();
			if (i != i0 && d > best) { best = d; i1 = i; }
		}
		Vec3f p1 = m.contacts[i1].position;

		int i2 = -1;
		best = -1;
		for (int i = 0; i < n; i++)
		{
			if (i == i0 || i == i1) continue;
			Real a = (p1 - p0).cross(m.contacts[i].position - p0).dot(m.normal);
			if (fabs(a) > best) { best = fabs(a); i2 = i; }
		}
		Vec3f p2 = m.contacts[i2].position;
		Real sign = (p1 - p0).cross(p2 - p0).dot(m.normal) >= 0 ? Real(1) : Real(-1);

		// The fourth point maximizes the area added on the side opposite to p2
		int i3 = -1;
		best = -REAL_MAX;
		for (int i = 0; i < n; i++)
		{
			if (i == i0 || i == i1 || i == i2) continue;
			Vec3f q = m.contacts[i].position;
			Real a01 = -sign * (p1 - p0).cross(q - p0).dot(m.normal);
			Real a12 = -sign * (p2 - p1).cross(q - p1).dot(m.normal);
			Real a20 = -sign * (p0 - p2).cross(q - p2).dot(m.normal);
			Real a = maximum(a01, maximum(a12, a20));
			if (a > best) { best = a; i3 = i; }
		}

		selected[0] = i0; selected[1] = i1; selected[2] = i2; selected[3] = i3;
		count = 4;
	}

	/**
	 * Contacts against an internal edge or vertex of the environment mesh take the face normal, otherwise a shape
	 * sliding over a flat tessellated floor would bump into the edges between triangles.
	 */
	GPU_FUNC inline void RCD_FilterInternalEdges(
		TManifold<Real>& m,
		const Triangle3D& tri,
		const DArray<int>& edgeActive,
		uint triId)
	{
		Vec3f e0 = tri.v[1] - tri.v[0];
		Vec3f e1 = tri.v[2] - tri.v[0];
		Vec3f f = e0.cross(e1);
		Real area2 = f.norm();
		if (area2 < REAL_EPSILON) return;
		f /= area2;

		// Already a face contact
		if (abs(f.dot(m.normal)) > Real(0.999)) return;

		// Feature of the deepest point
		int deepest = 0;
		for (int i = 1; i < m.contactCount; i++)
			if (m.contacts[i].penetration < m.contacts[deepest].penetration) deepest = i;

		Vec3f p = m.contacts[deepest].position - tri.v[0];
		Real d00 = e0.dot(e0), d01 = e0.dot(e1), d11 = e1.dot(e1);
		Real d20 = p.dot(e0), d21 = p.dot(e1);
		Real denom = d00 * d11 - d01 * d01;
		if (abs(denom) < REAL_EPSILON) return;
		Real b1 = (d11 * d20 - d01 * d21) / denom;
		Real b2 = (d00 * d21 - d01 * d20) / denom;
		Real bary[3] = { Real(1) - b1 - b2, b1, b2 };

		const Real eps = Real(1e-3);
		int zeros = 0;
		int zeroId[3];
		for (int k = 0; k < 3; k++)
			if (bary[k] < eps) zeroId[zeros++] = k;

		// Edge e runs from v[e] to v[(e + 1) % 3] and is opposite to vertex (e + 2) % 3
		bool internal = false;
		if (zeros == 1)
		{
			int e = (zeroId[0] + 1) % 3;
			internal = edgeActive[3 * triId + e] == 0;
		}
		else if (zeros == 2)
		{
			// vertex opposite to the two zero coordinates, shared by edges (k + 2) % 3 and k
			int k = 3 - zeroId[0] - zeroId[1];
			internal = edgeActive[3 * triId + k] == 0 && edgeActive[3 * triId + (k + 2) % 3] == 0;
		}

		if (!internal) return;

		Vec3f n = f.dot(m.normal) > 0 ? f : -f;
		Real c = n.dot(m.normal);
		m.normal = n;
		for (int i = 0; i < m.contactCount; i++)
			m.contacts[i].penetration *= c;
	}

	/**
	 * An edge of the environment mesh is active, i.e. may produce edge normals, when it is a boundary or
	 * non-manifold edge, or a convex edge sharper than a few degrees. Flat and concave edges are internal.
	 */
	__global__ void RCD_MeshEdgeKeys(
		DArray<unsigned long long> keys,
		DArray<uint> ids,
		DArray<Topology::Triangle> indices)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= indices.size()) return;

		auto t = indices[tId];
		for (int e = 0; e < 3; e++)
		{
			uint a = t[e];
			uint b = t[(e + 1) % 3];
			uint lo = a < b ? a : b;
			uint hi = a < b ? b : a;
			keys[3 * tId + e] = ((unsigned long long)lo << 32) | hi;
			ids[3 * tId + e] = 3 * tId + e;
		}
	}

	__global__ void RCD_MeshEdgeFlags(
		DArray<int> active,
		DArray<unsigned long long> keys,
		DArray<uint> ids,
		DArray<Triangle3D> tris)
	{
		uint i = threadIdx.x + (blockIdx.x * blockDim.x);
		if (i >= keys.size()) return;

		unsigned long long k = keys[i];
		uint first = i;
		while (first > 0 && keys[first - 1] == k) first--;
		uint last = i + 1;
		while (last < keys.size() && keys[last] == k) last++;

		uint self = ids[i];
		if (last - first != 2)
		{
			active[self] = 1;
			return;
		}

		uint other = ids[first == i ? i + 1 : first];
		Triangle3D t = tris[self / 3];
		Triangle3D u = tris[other / 3];

		Vec3f nt = (t.v[1] - t.v[0]).cross(t.v[2] - t.v[0]);
		Vec3f nu = (u.v[1] - u.v[0]).cross(u.v[2] - u.v[0]);
		Real lt = nt.norm(), lu = nu.norm();
		if (lt < REAL_EPSILON || lu < REAL_EPSILON)
		{
			active[self] = 1;
			return;
		}
		nt /= lt;
		nu /= lu;

		// vertex of u that is not on the shared edge
		Vec3f w = u.v[(other % 3 + 2) % 3];
		Vec3f a = t.v[self % 3];
		Real height = nt.dot(w - a);

		bool flat = abs(nt.dot(nu)) > Real(0.9986);	// within about 3 degrees
		bool concave = height > Real(0);
		active[self] = (flat || concave) ? 0 : 1;
	}

	template<int TA, int TB>
	__global__ void RCD_NarrowPhase(
		DArray<TContactManifold<Real>> manifolds,
		DArray<unsigned long long> pairKeys,
		uint begin,
		uint end,
		RCD_Shapes shapes,
		Real dHat)
	{
		uint tId = begin + threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= end) return;

		unsigned long long key = pairKeys[tId];
		uint idA = pairFirst(key);
		uint idB = pairSecond(key);

		auto shapeA = RCD_ShapeOf<TA>::get(shapes, idA);
		auto shapeB = RCD_ShapeOf<TB>::get(shapes, idB);

		TManifold<Real> m;
		m.contactCount = 0;
		Real inflation = RCD_Collide(m, shapeA, shapeB, dHat);

		TContactManifold<Real> out;
		out.count = 0;

		if (m.contactCount > 0)
		{
			int bodyA = shapes.shapeBody[idA];
			int bodyB = shapes.shapeBody[idB];

			// The environment mesh is always the second shape
			if (bodyA == INVALID)
			{
				RCD_FlipManifold(m);
				uint tmp = idA; idA = idB; idB = tmp;
				int t = bodyA; bodyA = bodyB; bodyB = t;
			}

			if (idB >= shapes.elementNum)
				RCD_FilterInternalEdges(m, shapes.meshTris[idB - shapes.elementNum], shapes.meshEdgeActive, idB - shapes.elementNum);

			int selected[4];
			int count = 0;
			RCD_ReducePoints(m, selected, count);

			out.shapeId1 = idA;
			out.shapeId2 = idB;
			out.bodyId1 = bodyA;
			out.bodyId2 = bodyB;
			out.normal = m.normal;
			out.count = count;
			for (int i = 0; i < count; i++)
			{
				out.points[i] = m.contacts[selected[i]].position;
				out.separation[i] = m.contacts[selected[i]].penetration + inflation;
			}
		}

		manifolds[tId] = out;
	}

	__global__ void RCD_FindBucketStart(
		DArray<uint> bucketStart,
		DArray<unsigned long long> pairKeys)
	{
		uint b = threadIdx.x + (blockIdx.x * blockDim.x);
		if (b >= bucketStart.size()) return;

		// First pair whose bucket is not smaller than b
		uint lo = 0, hi = pairKeys.size();
		while (lo < hi)
		{
			uint mid = (lo + hi) / 2;
			if (pairBucket(pairKeys[mid]) < b) lo = mid + 1;
			else hi = mid;
		}
		bucketStart[b] = lo;
	}

	template<int TA, int TB>
	static void RCD_LaunchNarrow(DArray<TContactManifold<Real>>& manifolds, DArray<unsigned long long>& keys, uint begin, uint end, RCD_Shapes& shapes, Real dHat)
	{
		if (end <= begin) return;
		cuExecute(end - begin, (RCD_NarrowPhase<TA, TB>), manifolds, keys, begin, end, shapes, dHat);
	}

	template<typename TDataType>
	void RigidContactDetection<TDataType>::narrowPhase(uint elementNum)
	{
		auto topo = this->inDiscreteElements()->getDataPtr();

		uint pairNum = mPairKeys.size();
		mManifoldBuffer.resize(pairNum);
		if (pairNum == 0) return;

		mBucketStart.resize(PAIR_BUCKET_NUM + 1);
		cuExecute(PAIR_BUCKET_NUM + 1, RCD_FindBucketStart, mBucketStart, mPairKeys);

		CArray<uint> start;
		start.assign(mBucketStart);

		RCD_Shapes shapes;
		shapes.boxes = topo->boxesInGlobal();
		shapes.spheres = topo->spheresInGlobal();
		shapes.tets = topo->tetsInGlobal();
		shapes.caps = topo->capsulesInGlobal();
		shapes.tris = topo->trianglesInGlobal();
		shapes.cones = topo->medialConesInGlobal();
		shapes.slabs = topo->medialSlabsInGlobal();
		shapes.meshTris = mMeshTriangles;
		shapes.meshEdgeActive = mMeshEdgeActive;
		shapes.shapeBody = mShapeBody;
		shapes.offset = topo->calculateElementOffset();
		shapes.elementNum = elementNum;

		Real dHat = this->varDHead()->getValue();

#define RCD_NARROW(A, B) RCD_LaunchNarrow<A, B>(mManifoldBuffer, mPairKeys, start[A * NT_COUNT + B], start[A * NT_COUNT + B + 1], shapes, dHat)
		RCD_NARROW(NT_BOX, NT_BOX);
		RCD_NARROW(NT_BOX, NT_TET);
		RCD_NARROW(NT_BOX, NT_CAPSULE);
		RCD_NARROW(NT_BOX, NT_SPHERE);
		RCD_NARROW(NT_BOX, NT_TRIANGLE);
		RCD_NARROW(NT_BOX, NT_MEDIALCONE);
		RCD_NARROW(NT_BOX, NT_MEDIALSLAB);
		RCD_NARROW(NT_TET, NT_TET);
		RCD_NARROW(NT_TET, NT_CAPSULE);
		RCD_NARROW(NT_TET, NT_SPHERE);
		RCD_NARROW(NT_TET, NT_TRIANGLE);
		RCD_NARROW(NT_TET, NT_MEDIALCONE);
		RCD_NARROW(NT_TET, NT_MEDIALSLAB);
		RCD_NARROW(NT_CAPSULE, NT_CAPSULE);
		RCD_NARROW(NT_CAPSULE, NT_SPHERE);
		RCD_NARROW(NT_CAPSULE, NT_TRIANGLE);
		RCD_NARROW(NT_CAPSULE, NT_MEDIALCONE);
		RCD_NARROW(NT_CAPSULE, NT_MEDIALSLAB);
		RCD_NARROW(NT_SPHERE, NT_SPHERE);
		RCD_NARROW(NT_SPHERE, NT_TRIANGLE);
		RCD_NARROW(NT_SPHERE, NT_MEDIALCONE);
		RCD_NARROW(NT_SPHERE, NT_MEDIALSLAB);
		RCD_NARROW(NT_TRIANGLE, NT_TRIANGLE);
		RCD_NARROW(NT_TRIANGLE, NT_MEDIALCONE);
		RCD_NARROW(NT_TRIANGLE, NT_MEDIALSLAB);
		RCD_NARROW(NT_MEDIALCONE, NT_MEDIALCONE);
		RCD_NARROW(NT_MEDIALCONE, NT_MEDIALSLAB);
		RCD_NARROW(NT_MEDIALSLAB, NT_MEDIALSLAB);
#undef RCD_NARROW
	}

	//---------------------------------------------------------------------------------------------------------------
	// Output
	//---------------------------------------------------------------------------------------------------------------

	struct RCD_HasContact
	{
		DYN_FUNC bool operator()(const TContactManifold<Real>& m) const { return m.count > 0; }
	};

	__global__ void RCD_CountPoints(
		DArray<uint> count,
		DArray<TContactManifold<Real>> manifolds)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= manifolds.size()) return;

		count[tId] = manifolds[tId].count;
	}

	__global__ void RCD_WriteLegacyContacts(
		DArray<TContactPair<Real>> contacts,
		DArray<uint> offset,
		DArray<TContactManifold<Real>> manifolds,
		Real dHat)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= manifolds.size()) return;

		TContactManifold<Real> m = manifolds[tId];
		uint base = offset[tId];

		for (int i = 0; i < m.count; i++)
		{
			TContactPair<Real> cp;
			cp.bodyId1 = m.bodyId1;
			cp.bodyId2 = m.bodyId2;
			cp.localId1 = m.shapeId1;
			cp.localId2 = m.shapeId2;
			cp.pos1 = m.points[i];
			cp.pos2 = m.points[i];
			cp.normal1 = -m.normal;
			cp.normal2 = m.normal;
			cp.contactType = m.bodyId2 == INVALID ? ContactType::CT_BOUDNARY : ContactType::CT_NONPENETRATION;
			// Same convention as NeighborElementQuery: shapes are inflated by dHat each
			cp.interpenetration = 2 * dHat - m.separation[i];
			contacts[base + i] = cp;
		}
	}

	template<typename TDataType>
	void RigidContactDetection<TDataType>::writeLegacyContacts()
	{
		auto& manifolds = this->outManifolds()->getData();

		if (this->outContacts()->isEmpty())
			this->outContacts()->allocate();
		auto& contacts = this->outContacts()->getData();

		uint num = manifolds.size();
		if (num == 0) {
			contacts.resize(0);
			return;
		}

		mPointCount.resize(num);
		cuExecute(num, RCD_CountPoints, mPointCount, manifolds);

		uint total = mReduce.accumulate(mPointCount.begin(), mPointCount.size());
		mScan.exclusive(mPointCount, true);

		contacts.resize(total);
		cuExecute(num, RCD_WriteLegacyContacts, contacts, mPointCount, manifolds, this->varDHead()->getValue());
	}

	template<typename TDataType>
	void RigidContactDetection<TDataType>::updateMeshEdges(DArray<Topology::Triangle>& indices)
	{
		uint edgeNum = 3 * indices.size();

		DArray<unsigned long long> keys;
		DArray<uint> ids;
		keys.resize(edgeNum);
		ids.resize(edgeNum);
		cuExecute(indices.size(), RCD_MeshEdgeKeys, keys, ids, indices);

		thrust::sort_by_key(thrust::device,
			thrust::device_ptr<unsigned long long>(keys.begin()),
			thrust::device_ptr<unsigned long long>(keys.begin()) + edgeNum,
			thrust::device_ptr<uint>(ids.begin()));

		mMeshEdgeActive.resize(edgeNum);
		cuExecute(edgeNum, RCD_MeshEdgeFlags, mMeshEdgeActive, keys, ids, mMeshTriangles);

		keys.clear();
		ids.clear();
	}

	template<typename TDataType>
	void RigidContactDetection<TDataType>::compute()
	{
		auto topo = this->inDiscreteElements()->getDataPtr();

		if (this->outManifolds()->isEmpty())
			this->outManifolds()->allocate();
		auto& manifolds = this->outManifolds()->getData();

		uint elementNum = topo->totalSize();

		// Environment mesh
		uint meshTriNum = 0;
		if (!this->inTriangleSet()->isEmpty())
		{
			auto ts = this->inTriangleSet()->constDataPtr();
			auto& indices = ts->triangleIndices();
			meshTriNum = indices.size();

			if (this->inTriangleSet()->isModified() || mMeshTriangles.size() != meshTriNum)
			{
				mMeshTriangles.resize(meshTriNum);
				if (meshTriNum > 0)
				{
					cuExecute(meshTriNum, RCD_SetupMeshTriangles, mMeshTriangles, ts->getPoints(), indices);
					updateMeshEdges(indices);
				}
				mStaticDirty = true;
			}
		}
		else if (mMeshTriangles.size() > 0)
		{
			mMeshTriangles.resize(0);
			mStaticDirty = true;
		}

		uint total = elementNum + meshTriNum;
		if (total == 0 || total > (1u << PAIR_ID_BITS))
		{
			manifolds.resize(0);
			writeLegacyContacts();
			return;
		}

		mShapeAABB.resize(total);
		mShapeBody.resize(total);
		mShapeIsStatic.resize(total);

		bool hasAttribute = !this->inAttribute()->isEmpty();
		DArray<Attribute> attribute;
		if (hasAttribute) attribute = this->inAttribute()->getData();

		cuExecute(total,
			RCD_SetupShapes,
			mShapeAABB,
			mShapeBody,
			mShapeIsStatic,
			topo->boxesInGlobal(),
			topo->spheresInGlobal(),
			topo->tetsInGlobal(),
			topo->capsulesInGlobal(),
			topo->trianglesInGlobal(),
			topo->medialConesInGlobal(),
			topo->medialSlabsInGlobal(),
			mMeshTriangles,
			topo->shape2RigidBodyMapping(),
			attribute,
			topo->calculateElementOffset(),
			elementNum,
			hasAttribute,
			this->varDHead()->getValue());

		updateBroadPhase(elementNum, meshTriNum);
		findPairs(elementNum);
		narrowPhase(elementNum);

		// Keep the manifolds with at least one point, in pair order
		uint pairNum = mManifoldBuffer.size();
		uint num = pairNum == 0 ? 0 : thrust::count_if(thrust::device,
			thrust::device_ptr<Manifold>(mManifoldBuffer.begin()),
			thrust::device_ptr<Manifold>(mManifoldBuffer.begin()) + pairNum,
			RCD_HasContact());

		manifolds.resize(num);
		if (num > 0)
		{
			thrust::copy_if(thrust::device,
				thrust::device_ptr<Manifold>(mManifoldBuffer.begin()),
				thrust::device_ptr<Manifold>(mManifoldBuffer.begin()) + pairNum,
				thrust::device_ptr<Manifold>(manifolds.begin()),
				RCD_HasContact());
		}

		writeLegacyContacts();
	}

	DEFINE_CLASS(RigidContactDetection);
}
