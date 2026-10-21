#include "AVBDConstraintSolver.h"

#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/binary_search.h>
#include <thrust/execution_policy.h>

namespace dyno
{
	IMPLEMENT_TCLASS(AVBDConstraintSolver, TDataType)

	// Colors [0, AVBD_MAX_COLORS) are conflict free, bodies that cannot get one are put into the overflow color
	// and are processed together in a final Jacobi style pass.
	#define AVBD_MAX_COLORS 32
	#define AVBD_OVERFLOW_COLOR AVBD_MAX_COLORS
	#define AVBD_UNCOLORED -1
	#define AVBD_NOT_DYNAMIC -2
	#define AVBD_MAX_COLORING_ROUNDS 64

	template<typename TDataType>
	AVBDConstraintSolver<TDataType>::AVBDConstraintSolver()
		: ConstraintModule()
	{
		this->inManifolds()->tagOptional(true);
		this->inContacts()->tagOptional(true);
		this->inFrictionCoefficients()->tagOptional(true);
	}

	template<typename TDataType>
	AVBDConstraintSolver<TDataType>::~AVBDConstraintSolver()
	{
	}

	/******************************************************************************************************************
	 * Helpers
	 ******************************************************************************************************************/

	DYN_FUNC inline uint AVBD_Hash(uint x)
	{
		x = (x ^ 61) ^ (x >> 16);
		x *= 9;
		x = x ^ (x >> 4);
		x *= 0x27d4eb2d;
		x = x ^ (x >> 15);
		return x;
	}

	DYN_FUNC inline bool AVBD_HigherPriority(uint u, uint v)
	{
		uint hu = AVBD_Hash(u);
		uint hv = AVBD_Hash(v);
		return hu > hv || (hu == hv && u > v);
	}

	GPU_FUNC inline int AVBD_Type(const Attribute& a)
	{
		return a.isDynamic() ? AVBD_DYNAMIC : (a.isPassive() ? AVBD_KINEMATIC : AVBD_FIXED);
	}

	/******************************************************************************************************************
	 * Manifold setup
	 ******************************************************************************************************************/

	template<typename Real>
	GPU_FUNC inline bool AVBD_IsDynamic(const DArray<Attribute>& att, int id)
	{
		return id != INVALID && att[id].isDynamic();
	}

	template<typename Real, typename Coord>
	GPU_FUNC inline void AVBD_InitManifold(
		AVBDManifold<Real>& m,
		int bodyA,
		int bodyB,
		Coord nBA,
		const Coord* pA,
		const Coord* pB,
		int count,
		DArray<Coord>& pos,
		DArray<Quat<Real>>& quat,
		DArray<Real>& mass,
		DArray<Attribute>& att,
		DArray<Real>& fricCoeffs,
		bool hasFriction,
		Real penaltyMin)
	{
		m.bodyA = bodyA;
		m.bodyB = bodyB;
		m.n = nBA;
		AVBD_Basis(m.n, m.t1, m.t2);

		Real muA = fricCoeffs.size() > 0 ? fricCoeffs[bodyA] : Real(0);
		Real muB = (fricCoeffs.size() > 0 && bodyB != INVALID) ? fricCoeffs[bodyB] : muA;
		m.friction = hasFriction ? (muA + muB) / 2 : Real(0);

		Real mA = mass[bodyA];
		m.mass = AVBD_IsDynamic<Real>(att, bodyB) ? mA * mass[bodyB] / (mA + mass[bodyB]) : mA;
		if (!(m.mass > Real(0))) m.mass = Real(1);

		Quat<Real> qAinv = quat[bodyA].conjugate();
		Quat<Real> qBinv = bodyB != INVALID ? quat[bodyB].conjugate() : Quat<Real>::identity();

		m.count = count;
		for (int i = 0; i < count; i++)
		{
			AVBDContact<Real> c;
			c.rA = qAinv.rotate(pA[i] - pos[bodyA]);
			c.rB = bodyB != INVALID ? qBinv.rotate(pB[i] - pos[bodyB]) : pB[i];
			c.C0 = Coord(0);
			c.penalty = Coord(penaltyMin * m.mass);
			c.lambda = Coord(0);
			c.stick = 0;
			m.c[i] = c;
		}
	}

	template<typename Real, typename Coord>
	__global__ void AVBD_SetupFromManifolds(
		DArray<AVBDManifold<Real>> manifolds,
		DArray<TContactManifold<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Attribute> att,
		DArray<Real> fricCoeffs,
		bool hasFriction,
		Real penaltyMin)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		TContactManifold<Real> in = input[tId];
		AVBDManifold<Real> m;

		int a = in.bodyId1;
		int b = in.bodyId2;
		int count = in.count < AVBDManifold<Real>::MAX_POINTS ? in.count : AVBDManifold<Real>::MAX_POINTS;

		// The input normal points from shape 1 to shape 2, points lie on shape 2
		Coord p1[AVBDManifold<Real>::MAX_POINTS];
		Coord p2[AVBDManifold<Real>::MAX_POINTS];
		for (int i = 0; i < count; i++)
		{
			p2[i] = in.points[i];
			p1[i] = in.points[i] - in.normal * in.separation[i];
		}

		bool dynA = AVBD_IsDynamic<Real>(att, a);
		bool dynB = AVBD_IsDynamic<Real>(att, b);

		if (dynA)
		{
			AVBD_InitManifold(m, a, b, -in.normal, p1, p2, count, pos, quat, mass, att, fricCoeffs, hasFriction, penaltyMin);
			m.key = ((unsigned long long)(uint)in.shapeId1 << 32) | (unsigned long long)(uint)in.shapeId2;
		}
		else if (dynB)
		{
			AVBD_InitManifold(m, b, a, in.normal, p2, p1, count, pos, quat, mass, att, fricCoeffs, hasFriction, penaltyMin);
			m.key = ((unsigned long long)(uint)in.shapeId2 << 32) | (unsigned long long)(uint)in.shapeId1;
		}
		else
		{
			m.count = 0;
			m.key = ~0ull;
		}

		manifolds[tId] = m;
	}

	template<typename Real, typename Coord>
	__global__ void AVBD_SetupFromContacts(
		DArray<AVBDManifold<Real>> manifolds,
		DArray<TContactPair<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Attribute> att,
		DArray<Real> fricCoeffs,
		bool hasFriction,
		Real penaltyMin,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		TContactPair<Real> in = input[tId];
		AVBDManifold<Real> m;

		int a = in.bodyId1;
		int b = in.bodyId2;

		// normal1 pushes body 1 out of body 2. Boundary contacts store the point on body 1,
		// all others store the point on body 2.
		Coord n = in.normal1;
		Real d = in.interpenetration;
		Coord p1, p2;
		if (in.contactType == ContactType::CT_BOUDNARY)
		{
			p1 = in.pos1;
			p2 = in.pos1 + n * d;
		}
		else
		{
			p2 = in.pos1;
			p1 = in.pos1 - n * d;
		}

		bool dynA = AVBD_IsDynamic<Real>(att, a);
		bool dynB = AVBD_IsDynamic<Real>(att, b);

		// Legacy contacts carry no shape ids, they are matched across frames by their body pair and position
		const unsigned long long legacy = 1ull << 63;
		if (dynA)
		{
			AVBD_InitManifold(m, a, b, n, &p1, &p2, 1, pos, quat, mass, att, fricCoeffs, hasFriction, penaltyMin);
			m.key = legacy | ((unsigned long long)(uint)a << 31) | (unsigned long long)((uint)(b + 1) & 0x7FFFFFFF);
		}
		else if (dynB)
		{
			AVBD_InitManifold(m, b, a, -n, &p2, &p1, 1, pos, quat, mass, att, fricCoeffs, hasFriction, penaltyMin);
			m.key = legacy | ((unsigned long long)(uint)b << 31) | (unsigned long long)((uint)(a + 1) & 0x7FFFFFFF);
		}
		else
		{
			m.count = 0;
			m.key = ~0ull;
		}

		manifolds[offset + tId] = m;
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::setupManifolds()
	{
		uint numM = this->inManifolds()->isEmpty() ? 0 : this->inManifolds()->size();
		uint numC = this->inContacts()->isEmpty() ? 0 : this->inContacts()->size();

		mManifoldNum = numM + numC;
		mManifolds.resize(mManifoldNum);
		if (mManifoldNum == 0)
			return;

		bool hasFriction = this->varFrictionEnabled()->getValue();
		Real kMin = this->varPenaltyMin()->getValue();

		DArray<Real> emptyFriction;
		auto& friction = this->inFrictionCoefficients()->isEmpty() ? emptyFriction : this->inFrictionCoefficients()->getData();

		if (numM > 0)
		{
			cuExecute(numM, AVBD_SetupFromManifolds,
				mManifolds,
				this->inManifolds()->getData(),
				this->inCenter()->getData(),
				this->inQuaternion()->getData(),
				this->inMass()->getData(),
				this->inAttribute()->getData(),
				friction,
				hasFriction,
				kMin);
		}

		if (numC > 0)
		{
			cuExecute(numC, AVBD_SetupFromContacts,
				mManifolds,
				this->inContacts()->getData(),
				this->inCenter()->getData(),
				this->inQuaternion()->getData(),
				this->inMass()->getData(),
				this->inAttribute()->getData(),
				friction,
				hasFriction,
				kMin,
				numM);
		}
	}

	/******************************************************************************************************************
	 * Joints
	 ******************************************************************************************************************/

	template<typename Real, typename Coord, typename Matrix>
	GPU_FUNC inline void AVBD_FinishJoint(
		AVBDJoint<Real>& j,
		DArray<Real>& mass,
		DArray<Matrix>& initialInertia,
		DArray<Attribute>& att,
		Real penaltyMin)
	{
		bool dynA = AVBD_IsDynamic<Real>(att, j.bodyA);
		bool dynB = AVBD_IsDynamic<Real>(att, j.bodyB);

		Real mA = dynA ? mass[j.bodyA] : Real(0);
		Real mB = dynB ? mass[j.bodyB] : Real(0);
		if (dynA && dynB) j.mass = mA * mB / (mA + mB);
		else j.mass = dynA ? mA : mB;
		if (!(j.mass > Real(0))) j.mass = Real(1);

		// Radius of gyration of the lighter dynamic body turns rotation errors into lengths
		int light = (dynA && (!dynB || mA <= mB)) ? j.bodyA : (dynB ? j.bodyB : INVALID);
		j.L = Real(1);
		if (light != INVALID && mass[light] > Real(0))
		{
			Matrix I = initialInertia[light];
			Real r2 = (I(0, 0) + I(1, 1) + I(2, 2)) / mass[light];
			if (r2 > Real(0)) j.L = sqrt(r2);
		}

		for (int k = 0; k < AVBDJoint<Real>::MAX_ROWS; k++)
		{
			j.C0[k] = Real(0);
			j.lambda[k] = Real(0);
			j.penalty[k] = penaltyMin * j.mass;
		}
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_SetupBallJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<BallAndSocketJoint<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Matrix> initialInertia,
		DArray<Attribute> att,
		Real penaltyMin,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		BallAndSocketJoint<Real> in = input[tId];
		AVBDJoint<Real> j;
		j.type = AVBD_JOINT_BALL;
		j.bodyA = in.bodyId1;
		j.bodyB = in.bodyId2;
		j.rA = in.r1;
		j.rB = in.r2;
		AVBD_FinishJoint<Real, Coord, Matrix>(j, mass, initialInertia, att, penaltyMin);
		joints[offset + tId] = j;
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_SetupHingeJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<HingeJoint<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Matrix> initialInertia,
		DArray<Attribute> att,
		Real penaltyMin,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		HingeJoint<Real> in = input[tId];
		AVBDJoint<Real> j;
		j.type = AVBD_JOINT_HINGE;
		j.bodyA = in.bodyId1;
		j.bodyB = in.bodyId2;
		j.rA = in.r1;
		j.axisA = in.hingeAxisBody1;
		if (j.bodyB != INVALID)
		{
			j.rB = in.r2;
			j.axisB = in.hingeAxisBody2;
		}
		else
		{
			// Anchored to the world at the current pose
			j.rB = pos[j.bodyA] + quat[j.bodyA].rotate(in.r1);
			j.axisB = quat[j.bodyA].rotate(in.hingeAxisBody1);
		}
		AVBD_FinishJoint<Real, Coord, Matrix>(j, mass, initialInertia, att, penaltyMin);
		joints[offset + tId] = j;
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_SetupSliderJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<SliderJoint<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Matrix> initialInertia,
		DArray<Attribute> att,
		Real penaltyMin,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		SliderJoint<Real> in = input[tId];
		AVBDJoint<Real> j;
		j.type = AVBD_JOINT_SLIDER;
		j.bodyA = in.bodyId1;
		j.bodyB = in.bodyId2;
		j.rA = in.r1;
		j.axisA = in.sliderAxis;
		j.qRel = Quat<Real>(in.q_init.x, in.q_init.y, in.q_init.z, in.q_init.w);
		j.rB = j.bodyB != INVALID ? in.r2 : pos[j.bodyA] + quat[j.bodyA].rotate(in.r1);
		AVBD_FinishJoint<Real, Coord, Matrix>(j, mass, initialInertia, att, penaltyMin);
		joints[offset + tId] = j;
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_SetupFixedJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<FixedJoint<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Matrix> initialInertia,
		DArray<Attribute> att,
		Real penaltyMin,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		FixedJoint<Real> in = input[tId];
		AVBDJoint<Real> j;
		j.type = AVBD_JOINT_FIXED;
		j.bodyA = in.bodyId1;
		j.bodyB = in.bodyId2;
		j.rA = in.r1;
		if (j.bodyB != INVALID)
		{
			j.rB = in.r2;
			j.qRel = in.q_init;
		}
		else
		{
			// The world anchor and orientation can be animated through setAnchorPoint / setAnchorAngle
			j.rB = in.w;
			j.qRel = in.q;
		}
		AVBD_FinishJoint<Real, Coord, Matrix>(j, mass, initialInertia, att, penaltyMin);
		joints[offset + tId] = j;
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_SetupPointJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<PointJoint<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Matrix> initialInertia,
		DArray<Attribute> att,
		Real penaltyMin,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		PointJoint<Real> in = input[tId];
		AVBDJoint<Real> j;
		j.type = AVBD_JOINT_POINT;
		j.bodyA = in.bodyId1;
		j.bodyB = INVALID;
		j.rA = Coord(0);
		j.rB = in.anchorPoint;
		AVBD_FinishJoint<Real, Coord, Matrix>(j, mass, initialInertia, att, penaltyMin);
		joints[offset + tId] = j;
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_SetupDistanceJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<DistanceJoint<Real>> input,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Real> mass,
		DArray<Matrix> initialInertia,
		DArray<Attribute> att,
		Real penaltyMin,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= input.size()) return;

		DistanceJoint<Real> in = input[tId];
		AVBDJoint<Real> j;
		j.type = AVBD_JOINT_DISTANCE;
		j.bodyA = in.bodyId1;
		j.bodyB = in.bodyId2;
		j.rA = in.r1;
		j.rB = in.r2;
		j.distance = in.distance;
		AVBD_FinishJoint<Real, Coord, Matrix>(j, mass, initialInertia, att, penaltyMin);
		joints[offset + tId] = j;
	}

	/**
	 * Joints are rebuilt every frame so that animated anchors are picked up, the solver state is carried over
	 * when the joint set did not change.
	 */
	template<typename Real>
	__global__ void AVBD_KeepJointState(
		DArray<AVBDJoint<Real>> joints,
		DArray<AVBDJoint<Real>> old)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= joints.size()) return;

		AVBDJoint<Real> j = joints[tId];
		const AVBDJoint<Real>& o = old[tId];
		if (o.type != j.type || o.bodyA != j.bodyA || o.bodyB != j.bodyB) return;

		for (int k = 0; k < AVBDJoint<Real>::MAX_ROWS; k++)
		{
			j.lambda[k] = o.lambda[k];
			j.penalty[k] = o.penalty[k];
		}

		// World anchors derived from the pose must stay where they were first placed
		if (j.bodyB < 0 && (j.type == AVBD_JOINT_HINGE || j.type == AVBD_JOINT_SLIDER))
		{
			j.rB = o.rB;
			j.axisB = o.axisB;
		}

		joints[tId] = j;
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::setupJoints()
	{
		auto topo = this->inDiscreteElements()->constDataPtr();

		auto& balls = topo->ballAndSocketJoints();
		auto& hinges = topo->hingeJoints();
		auto& sliders = topo->sliderJoints();
		auto& fixeds = topo->fixedJoints();
		auto& points = topo->pointJoints();
		auto& distances = topo->distanceJoints();

		uint total = balls.size() + hinges.size() + sliders.size() + fixeds.size() + points.size() + distances.size();

		bool keep = total > 0 && mJoints.size() == total;
		if (keep)
			mOldJoints.assign(mJoints);

		mJoints.resize(total);
		if (total == 0)
			return;

		if (hinges.size() + sliders.size() > 0)
		{
			static bool warned = false;
			if (!warned)
			{
				printf("AVBDConstraintSolver: ranges and motors of hinge and slider joints are not supported yet\n");
				warned = true;
			}
		}

		auto& pos = this->inCenter()->getData();
		auto& quat = this->inQuaternion()->getData();
		auto& mass = this->inMass()->getData();
		auto& inertia0 = this->inInitialInertia()->getData();
		auto& att = this->inAttribute()->getData();
		Real kMin = this->varPenaltyMin()->getValue();

		uint offset = 0;
		if (balls.size() > 0)
			cuExecute(balls.size(), AVBD_SetupBallJoints, mJoints, balls, pos, quat, mass, inertia0, att, kMin, offset);
		offset += balls.size();

		if (hinges.size() > 0)
			cuExecute(hinges.size(), AVBD_SetupHingeJoints, mJoints, hinges, pos, quat, mass, inertia0, att, kMin, offset);
		offset += hinges.size();

		if (sliders.size() > 0)
			cuExecute(sliders.size(), AVBD_SetupSliderJoints, mJoints, sliders, pos, quat, mass, inertia0, att, kMin, offset);
		offset += sliders.size();

		if (fixeds.size() > 0)
			cuExecute(fixeds.size(), AVBD_SetupFixedJoints, mJoints, fixeds, pos, quat, mass, inertia0, att, kMin, offset);
		offset += fixeds.size();

		if (points.size() > 0)
			cuExecute(points.size(), AVBD_SetupPointJoints, mJoints, points, pos, quat, mass, inertia0, att, kMin, offset);
		offset += points.size();

		if (distances.size() > 0)
			cuExecute(distances.size(), AVBD_SetupDistanceJoints, mJoints, distances, pos, quat, mass, inertia0, att, kMin, offset);

		if (keep)
			cuExecute(total, AVBD_KeepJointState, mJoints, mOldJoints);
	}

	/******************************************************************************************************************
	 * Warm start across frames
	 ******************************************************************************************************************/

	template<typename Real>
	__global__ void AVBD_WarmStart(
		DArray<AVBDManifold<Real>> manifolds,
		DArray<AVBDManifold<Real>> oldManifolds,
		DArray<unsigned long long> oldKeys,
		Real maxDist)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= manifolds.size()) return;

		AVBDManifold<Real> m = manifolds[tId];
		if (m.count == 0) return;

		// lower bound of the key
		int lo = 0;
		int hi = oldKeys.size();
		while (lo < hi)
		{
			int mid = (lo + hi) >> 1;
			if (oldKeys[mid] < m.key) lo = mid + 1;
			else hi = mid;
		}

		Real maxDist2 = maxDist * maxDist;
		for (int i = 0; i < m.count; i++)
		{
			AVBDContact<Real>& c = m.c[i];

			Real best = maxDist2;
			int bestM = -1;
			int bestC = -1;
			for (int j = lo; j < oldKeys.size() && oldKeys[j] == m.key; j++)
			{
				const AVBDManifold<Real>& o = oldManifolds[j];
				if (o.bodyA != m.bodyA || o.bodyB != m.bodyB) continue;

				for (int k = 0; k < o.count; k++)
				{
					Real d2 = (o.c[k].rA - c.rA).normSquared();
					if (d2 < best)
					{
						best = d2;
						bestM = j;
						bestC = k;
					}
				}
			}

			if (bestM >= 0)
			{
				const AVBDManifold<Real>& o = oldManifolds[bestM];
				const AVBDContact<Real>& oc = o.c[bestC];

				// Express the old force in the new basis since tangents are rebuilt every frame
				Vector<Real, 3> f = o.n * oc.lambda[0] + o.t1 * oc.lambda[1] + o.t2 * oc.lambda[2];
				c.lambda = Vector<Real, 3>(f.dot(m.n), f.dot(m.t1), f.dot(m.t2));
				c.penalty = oc.penalty;
				c.stick = oc.stick;

				// Static friction keeps the old anchors
				if (oc.stick)
				{
					c.rA = oc.rA;
					c.rB = oc.rB;
				}
			}
		}

		manifolds[tId] = m;
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::warmStart()
	{
		if (mManifoldNum == 0 || mOldKeys.size() == 0)
			return;

		cuExecute(mManifoldNum, AVBD_WarmStart,
			mManifolds,
			mOldManifolds,
			mOldKeys,
			this->varWarmStartDistance()->getValue());
	}

	template<typename Real>
	__global__ void AVBD_ExtractKeys(
		DArray<unsigned long long> keys,
		DArray<uint> ids,
		DArray<AVBDManifold<Real>> manifolds)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= manifolds.size()) return;

		keys[tId] = manifolds[tId].count > 0 ? manifolds[tId].key : ~0ull;
		ids[tId] = tId;
	}

	template<typename Real>
	__global__ void AVBD_GatherManifolds(
		DArray<AVBDManifold<Real>> dst,
		DArray<AVBDManifold<Real>> src,
		DArray<uint> ids)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= dst.size()) return;

		dst[tId] = src[ids[tId]];
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::saveManifolds()
	{
		if (mManifoldNum == 0)
		{
			mOldKeys.resize(0);
			mOldManifolds.resize(0);
			return;
		}

		mSortKeys.resize(mManifoldNum);
		mSortIds.resize(mManifoldNum);
		cuExecute(mManifoldNum, AVBD_ExtractKeys, mSortKeys, mSortIds, mManifolds);

		thrust::sort_by_key(thrust::device,
			thrust::device_ptr<unsigned long long>(mSortKeys.begin()),
			thrust::device_ptr<unsigned long long>(mSortKeys.begin()) + mManifoldNum,
			thrust::device_ptr<uint>(mSortIds.begin()));

		// Empty manifolds were given the largest key and sit at the end
		uint valid = thrust::lower_bound(thrust::device,
			thrust::device_ptr<unsigned long long>(mSortKeys.begin()),
			thrust::device_ptr<unsigned long long>(mSortKeys.begin()) + mManifoldNum,
			~0ull) - thrust::device_ptr<unsigned long long>(mSortKeys.begin());

		mOldKeys.resize(valid);
		mOldManifolds.resize(valid);
		if (valid == 0)
			return;

		mSortIds.resize(valid);
		mOldKeys.assign(mSortKeys, valid);
		cuExecute(valid, AVBD_GatherManifolds, mOldManifolds, mManifolds, mSortIds);
	}

	/******************************************************************************************************************
	 * Adjacency
	 ******************************************************************************************************************/

	template<typename Real>
	__global__ void AVBD_AdjacencyKeys(
		DArray<unsigned long long> keys,
		DArray<AVBDManifold<Real>> manifolds,
		DArray<Attribute> att)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= manifolds.size()) return;

		const AVBDManifold<Real>& m = manifolds[tId];

		unsigned long long kA = ~0ull;
		unsigned long long kB = ~0ull;
		if (m.count > 0)
		{
			kA = ((unsigned long long)(uint)m.bodyA << 32) | AVBD_Entry(tId, 0, 0);
			if (AVBD_IsDynamic<Real>(att, m.bodyB))
				kB = ((unsigned long long)(uint)m.bodyB << 32) | AVBD_Entry(tId, 0, 1);
		}

		keys[2 * tId] = kA;
		keys[2 * tId + 1] = kB;
	}

	template<typename Real>
	__global__ void AVBD_JointAdjacencyKeys(
		DArray<unsigned long long> keys,
		DArray<AVBDJoint<Real>> joints,
		DArray<Attribute> att,
		uint offset)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= joints.size()) return;

		const AVBDJoint<Real>& j = joints[tId];

		unsigned long long kA = ~0ull;
		unsigned long long kB = ~0ull;
		if (AVBD_IsDynamic<Real>(att, j.bodyA))
			kA = ((unsigned long long)(uint)j.bodyA << 32) | AVBD_Entry(tId, 1, 0);
		if (AVBD_IsDynamic<Real>(att, j.bodyB))
			kB = ((unsigned long long)(uint)j.bodyB << 32) | AVBD_Entry(tId, 1, 1);

		keys[offset + 2 * tId] = kA;
		keys[offset + 2 * tId + 1] = kB;
	}

	__global__ void AVBD_AdjacencyStart(
		DArray<uint> start,
		DArray<uint> adj,
		DArray<unsigned long long> keys,
		uint entryNum)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);

		if (tId < entryNum)
			adj[tId] = (uint)(keys[tId] & 0xFFFFFFFFull);

		if (tId >= start.size()) return;

		// lower bound of the first entry of body tId
		unsigned long long target = (unsigned long long)tId << 32;
		uint lo = 0;
		uint hi = entryNum;
		while (lo < hi)
		{
			uint mid = (lo + hi) >> 1;
			if (keys[mid] < target) lo = mid + 1;
			else hi = mid;
		}
		start[tId] = lo;
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::buildAdjacency(uint bodyNum)
	{
		uint jointNum = mJoints.size();
		uint slots = 2 * (mManifoldNum + jointNum);

		mAdjStart.resize(bodyNum + 1);
		if (slots == 0)
		{
			mAdj.resize(0);
			mAdjStart.reset();
			return;
		}

		mAdjKeys.resize(slots);
		if (mManifoldNum > 0)
			cuExecute(mManifoldNum, AVBD_AdjacencyKeys, mAdjKeys, mManifolds, this->inAttribute()->getData());
		if (jointNum > 0)
			cuExecute(jointNum, AVBD_JointAdjacencyKeys, mAdjKeys, mJoints, this->inAttribute()->getData(), 2 * mManifoldNum);

		// Sorting makes the gather order, hence the floating point sums, deterministic
		thrust::sort(thrust::device,
			thrust::device_ptr<unsigned long long>(mAdjKeys.begin()),
			thrust::device_ptr<unsigned long long>(mAdjKeys.begin()) + slots);

		uint entryNum = thrust::lower_bound(thrust::device,
			thrust::device_ptr<unsigned long long>(mAdjKeys.begin()),
			thrust::device_ptr<unsigned long long>(mAdjKeys.begin()) + slots,
			~0ull) - thrust::device_ptr<unsigned long long>(mAdjKeys.begin());

		mAdj.resize(entryNum);
		cuExecute(std::max(entryNum, bodyNum + 1), AVBD_AdjacencyStart, mAdjStart, mAdj, mAdjKeys, entryNum);
	}

	/******************************************************************************************************************
	 * Graph coloring
	 ******************************************************************************************************************/

	template<typename Real>
	GPU_FUNC inline int AVBD_Neighbor(
		const DArray<AVBDManifold<Real>>& manifolds,
		const DArray<AVBDJoint<Real>>& joints,
		const DArray<Attribute>& att,
		uint entry)
	{
		uint idx = entry >> 2;
		bool isB = (entry & 1) != 0;

		int other;
		if (entry & 2)
			other = isB ? joints[idx].bodyA : joints[idx].bodyB;
		else
			other = isB ? manifolds[idx].bodyA : manifolds[idx].bodyB;

		return AVBD_IsDynamic<Real>(att, other) ? other : INVALID;
	}

	__global__ void AVBD_ColorInit(
		DArray<int> color,
		DArray<int> tentative,
		DArray<Attribute> att)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= color.size()) return;

		if (!att[tId].isDynamic())
		{
			color[tId] = AVBD_NOT_DYNAMIC;
			tentative[tId] = AVBD_NOT_DYNAMIC;
			return;
		}

		// Try last frame's color first, most of them stay valid in a coherent scene
		int c = color[tId];
		tentative[tId] = (c >= 0 && c < AVBD_MAX_COLORS) ? c : AVBD_UNCOLORED;
		color[tId] = AVBD_UNCOLORED;
	}

	template<typename Real>
	__global__ void AVBD_ColorTentative(
		DArray<int> tentative,
		DArray<int> color,
		DArray<uint> adjStart,
		DArray<uint> adj,
		DArray<AVBDManifold<Real>> manifolds,
		DArray<AVBDJoint<Real>> joints,
		DArray<Attribute> att,
		bool keepPrevious)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= color.size()) return;
		if (color[tId] != AVBD_UNCOLORED) return;
		if (keepPrevious && tentative[tId] >= 0) return;

		unsigned int forbidden = 0;
		for (uint e = adjStart[tId]; e < adjStart[tId + 1]; e++)
		{
			int u = AVBD_Neighbor(manifolds, joints, att, adj[e]);
			if (u == INVALID) continue;

			int cu = color[u];
			if (cu >= 0 && cu < AVBD_MAX_COLORS)
				forbidden |= 1u << cu;
		}

		tentative[tId] = forbidden == 0xFFFFFFFFu ? AVBD_OVERFLOW_COLOR : __ffs(~forbidden) - 1;
	}

	template<typename Real>
	__global__ void AVBD_ColorResolve(
		DArray<int> resolved,
		DArray<int> tentative,
		DArray<int> color,
		DArray<uint> adjStart,
		DArray<uint> adj,
		DArray<AVBDManifold<Real>> manifolds,
		DArray<AVBDJoint<Real>> joints,
		DArray<Attribute> att)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= color.size()) return;

		resolved[tId] = color[tId];
		if (color[tId] != AVBD_UNCOLORED) return;

		int t = tentative[tId];
		if (t == AVBD_OVERFLOW_COLOR)
		{
			resolved[tId] = t;
			return;
		}

		bool conflict = false;
		for (uint e = adjStart[tId]; e < adjStart[tId + 1] && !conflict; e++)
		{
			int u = AVBD_Neighbor(manifolds, joints, att, adj[e]);
			if (u == INVALID || u == (int)tId) continue;

			if (color[u] == t)
				conflict = true;
			else if (color[u] == AVBD_UNCOLORED && tentative[u] == t && AVBD_HigherPriority(u, tId))
				conflict = true;
		}

		if (!conflict)
			resolved[tId] = t;
	}

	__global__ void AVBD_ColorCommit(
		DArray<int> color,
		DArray<int> resolved,
		DArray<uint> counter)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= color.size()) return;

		color[tId] = resolved[tId];
		if (resolved[tId] == AVBD_UNCOLORED)
			atomicAdd(&counter[0], 1u);
	}

	__global__ void AVBD_ColorKeys(
		DArray<unsigned long long> keys,
		DArray<int> color)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= color.size()) return;

		// Bodies left uncolored after the round limit are solved in the overflow pass
		int c = color[tId];
		if (c == AVBD_UNCOLORED) c = AVBD_OVERFLOW_COLOR;
		keys[tId] = c >= 0 ? (((unsigned long long)c << 32) | tId) : ~0ull;
	}

	__global__ void AVBD_ColorStart(
		DArray<uint> start,
		DArray<uint> bodies,
		DArray<unsigned long long> keys)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);

		if (tId < keys.size())
			bodies[tId] = (uint)(keys[tId] & 0xFFFFFFFFull);

		if (tId >= start.size()) return;

		unsigned long long target = (unsigned long long)tId << 32;
		uint lo = 0;
		uint hi = keys.size();
		while (lo < hi)
		{
			uint mid = (lo + hi) >> 1;
			if (keys[mid] < target) lo = mid + 1;
			else hi = mid;
		}
		start[tId] = lo;
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::colorBodies(uint bodyNum)
	{
		auto& att = this->inAttribute()->getData();

		mTentative.resize(bodyNum);
		mResolved.resize(bodyNum);
		mCounter.resize(1);

		cuExecute(bodyNum, AVBD_ColorInit, mColor, mTentative, att);

		CArray<uint> hCounter;
		for (uint round = 0; round < AVBD_MAX_COLORING_ROUNDS; round++)
		{
			cuExecute(bodyNum, AVBD_ColorTentative, mTentative, mColor, mAdjStart, mAdj, mManifolds, mJoints, att, round == 0);
			cuExecute(bodyNum, AVBD_ColorResolve, mResolved, mTentative, mColor, mAdjStart, mAdj, mManifolds, mJoints, att);

			mCounter.reset();
			cuExecute(bodyNum, AVBD_ColorCommit, mColor, mResolved, mCounter);

			hCounter.assign(mCounter);
			if (hCounter[0] == 0)
				break;
		}

		// Sort dynamic bodies by color
		mColorKeys.resize(bodyNum);
		cuExecute(bodyNum, AVBD_ColorKeys, mColorKeys, mColor);
		thrust::sort(thrust::device,
			thrust::device_ptr<unsigned long long>(mColorKeys.begin()),
			thrust::device_ptr<unsigned long long>(mColorKeys.begin()) + bodyNum);

		mColorBodies.resize(bodyNum);
		mColorStart.resize(AVBD_MAX_COLORS + 2);
		cuExecute(std::max(bodyNum, (uint)(AVBD_MAX_COLORS + 2)), AVBD_ColorStart, mColorStart, mColorBodies, mColorKeys);

		CArray<uint> hStart;
		hStart.assign(mColorStart);
		mColorStartHost.resize(hStart.size());
		for (uint i = 0; i < hStart.size(); i++)
			mColorStartHost[i] = hStart[i];
	}

	/******************************************************************************************************************
	 * Solver
	 ******************************************************************************************************************/

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_InitBodies(
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Coord> x0,
		DArray<Quat<Real>> q0,
		DArray<Coord> xHat,
		DArray<Quat<Real>> qHat,
		DArray<Coord> velocity,
		DArray<Coord> angularVelocity,
		DArray<Coord> prevVelocity,
		DArray<Matrix> rotation,
		DArray<Attribute> att,
		Real gravity,
		Real h)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= pos.size()) return;

		int type = AVBD_Type(att[tId]);
		AVBD_InitBody(tId, type, pos.begin(), quat.begin(), x0.begin(), q0.begin(), xHat.begin(), qHat.begin(),
			velocity.begin(), angularVelocity.begin(), prevVelocity.begin(), gravity, h);

		if (type == AVBD_KINEMATIC)
			rotation[tId] = quat[tId].toMatrix3x3();
	}

	template<typename Real, typename Coord>
	__global__ void AVBD_BeginManifolds(
		DArray<AVBDManifold<Real>> manifolds,
		DArray<Coord> x0,
		DArray<Quat<Real>> q0,
		AVBDParams params)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= manifolds.size()) return;

		AVBDManifold<Real> m = manifolds[tId];
		if (m.count == 0) return;

		AVBD_BeginManifold(m, x0.begin(), q0.begin(), params);
		manifolds[tId] = m;
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_Primal(
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Coord> x0,
		DArray<Quat<Real>> q0,
		DArray<Coord> xHat,
		DArray<Quat<Real>> qHat,
		DArray<Real> mass,
		DArray<Matrix> inertia,
		DArray<uint> colorBodies,
		DArray<uint> adjStart,
		DArray<uint> adj,
		DArray<AVBDManifold<Real>> manifolds,
		DArray<AVBDJoint<Real>> joints,
		uint first,
		uint last,
		Real alpha,
		Real h)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x) + first;
		if (tId >= last) return;

		AVBD_PrimalBody(colorBodies[tId], pos.begin(), quat.begin(), x0.begin(), q0.begin(), xHat.begin(), qHat.begin(),
			mass.begin(), inertia.begin(), adjStart.begin(), adj.begin(), manifolds.begin(), joints.begin(), alpha, h);
	}

	template<typename Real, typename Coord>
	__global__ void AVBD_Dual(
		DArray<AVBDManifold<Real>> manifolds,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Coord> x0,
		DArray<Quat<Real>> q0,
		AVBDParams params)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= manifolds.size()) return;

		AVBDManifold<Real> m = manifolds[tId];
		if (m.count == 0) return;

		AVBD_DualManifold(m, pos.begin(), quat.begin(), x0.begin(), q0.begin(), params);
		manifolds[tId] = m;
	}

	template<typename Real, typename Coord>
	__global__ void AVBD_BeginJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<Coord> x0,
		DArray<Quat<Real>> q0,
		AVBDParams params)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= joints.size()) return;

		AVBDJoint<Real> j = joints[tId];
		AVBD_BeginJoint(j, x0.begin(), q0.begin(), params);
		joints[tId] = j;
	}

	template<typename Real, typename Coord>
	__global__ void AVBD_DualJoints(
		DArray<AVBDJoint<Real>> joints,
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		AVBDParams params)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= joints.size()) return;

		AVBDJoint<Real> j = joints[tId];
		AVBD_DualJoint(j, pos.begin(), quat.begin(), params);
		joints[tId] = j;
	}

	template<typename Real, typename Coord, typename Matrix>
	__global__ void AVBD_FinishBodies(
		DArray<Coord> pos,
		DArray<Quat<Real>> quat,
		DArray<Coord> x0,
		DArray<Quat<Real>> q0,
		DArray<Coord> velocity,
		DArray<Coord> angularVelocity,
		DArray<Matrix> rotation,
		DArray<Matrix> inertia,
		DArray<Matrix> initialInertia,
		DArray<Attribute> att,
		Real h)
	{
		uint tId = threadIdx.x + (blockIdx.x * blockDim.x);
		if (tId >= pos.size()) return;

		AVBD_FinishBody(tId, AVBD_Type(att[tId]), pos.begin(), quat.begin(), x0.begin(), q0.begin(),
			velocity.begin(), angularVelocity.begin(), rotation.begin(), inertia.begin(), initialInertia.begin(), h);
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::resizeBodyBuffers(uint bodyNum)
	{
		if (mX0.size() == bodyNum)
			return;

		mX0.resize(bodyNum);
		mQ0.resize(bodyNum);
		mXHat.resize(bodyNum);
		mQHat.resize(bodyNum);

		// The velocity of the previous step drives the adaptive initial guess, start from the current one
		mPrevVelocity.assign(this->inVelocity()->getData());

		CArray<int> uncolored;
		uncolored.assign(bodyNum, AVBD_UNCOLORED);
		mColor.assign(uncolored);

		// Body ids are no longer meaningful, drop the warm start data
		mOldKeys.resize(0);
		mOldManifolds.resize(0);
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::solveSubstep(Real h)
	{
		auto& pos = this->inCenter()->getData();
		auto& quat = this->inQuaternion()->getData();
		auto& att = this->inAttribute()->getData();

		Real gravity = this->varGravityEnabled()->getValue() ? this->varGravityValue()->getValue() : Real(0);
		Real alpha = this->varAlpha()->getValue();

		AVBDParams params;
		params.alpha = alpha;
		params.gamma = this->varGamma()->getValue();
		params.betaLin = this->varBetaLinear()->getValue();
		params.penaltyMin = this->varPenaltyMin()->getValue();
		params.penaltyMax = this->varPenaltyMax()->getValue();
		params.margin = this->varContactMargin()->getValue();
		params.stickThreshold = this->varStickThreshold()->getValue();

		uint bodyNum = pos.size();

		cuExecute(bodyNum, AVBD_InitBodies,
			pos,
			quat,
			mX0,
			mQ0,
			mXHat,
			mQHat,
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			mPrevVelocity,
			this->inRotationMatrix()->getData(),
			att,
			gravity,
			h);

		if (mManifoldNum > 0)
		{
			cuExecute(mManifoldNum, AVBD_BeginManifolds,
				mManifolds,
				mX0,
				mQ0,
				params);
		}

		if (mJoints.size() > 0)
			cuExecute(mJoints.size(), AVBD_BeginJoints, mJoints, mX0, mQ0, params);

		uint iterNum = this->varIterationNumber()->getValue();
		for (uint it = 0; it < iterNum; it++)
		{
			// Colors are processed one after another, the overflow color last
			for (uint c = 0; c + 1 < mColorStartHost.size(); c++)
			{
				uint first = mColorStartHost[c];
				uint last = mColorStartHost[c + 1];
				if (last <= first) continue;

				cuExecute(last - first, AVBD_Primal,
					pos,
					quat,
					mX0,
					mQ0,
					mXHat,
					mQHat,
					this->inMass()->getData(),
					this->inInertia()->getData(),
					mColorBodies,
					mAdjStart,
					mAdj,
					mManifolds,
					mJoints,
					first,
					last,
					alpha,
					h);
			}

			if (mManifoldNum > 0)
			{
				cuExecute(mManifoldNum, AVBD_Dual,
					mManifolds,
					pos,
					quat,
					mX0,
					mQ0,
					params);
			}

			if (mJoints.size() > 0)
			{
				cuExecute(mJoints.size(), AVBD_DualJoints, mJoints, pos, quat, params);
			}
		}

		cuExecute(bodyNum, AVBD_FinishBodies,
			pos,
			quat,
			mX0,
			mQ0,
			this->inVelocity()->getData(),
			this->inAngularVelocity()->getData(),
			this->inRotationMatrix()->getData(),
			this->inInertia()->getData(),
			this->inInitialInertia()->getData(),
			att,
			h);
	}

	template<typename TDataType>
	void AVBDConstraintSolver<TDataType>::constrain()
	{
		uint bodyNum = this->inCenter()->size();
		if (bodyNum == 0)
			return;

		resizeBodyBuffers(bodyNum);

		setupManifolds();
		warmStart();
		setupJoints();
		buildAdjacency(bodyNum);
		colorBodies(bodyNum);

		uint substeps = std::max(this->varSubStepping()->getValue(), 1u);
		Real h = this->inTimeStep()->getValue() / substeps;
		for (uint s = 0; s < substeps; s++)
			solveSubstep(h);

		saveManifolds();
	}

	DEFINE_CLASS(AVBDConstraintSolver);
}
