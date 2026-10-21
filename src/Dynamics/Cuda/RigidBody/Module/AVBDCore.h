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
#include "Platform.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quat.h"

/**
 * Per-body and per-constraint routines of the AVBD rigid body solver. They work on raw pointers so that the same
 * code runs in the CUDA kernels and in CPU tests.
 */
namespace dyno
{
	/**
	 * @brief One contact point of an AVBD contact manifold, all three rows (normal and two tangents) share the same anchors.
	 */
	template<typename Real>
	struct AVBDContact
	{
		Vector<Real, 3> rA;			// anchor in the local frame of body A
		Vector<Real, 3> rB;			// anchor in the local frame of body B, or in world space when B is the static world
		Vector<Real, 3> C0;			// constraint error at the beginning of the step
		Vector<Real, 3> penalty;
		Vector<Real, 3> lambda;
		int stick = 0;
	};

	/**
	 * @brief A contact manifold as seen by the AVBD solver. Body A is always dynamic.
	 */
	template<typename Real>
	struct AVBDManifold
	{
		static const int MAX_POINTS = 4;

		unsigned long long key = 0;	// identifies the same manifold across frames

		int bodyA = -1;
		int bodyB = -1;				// -1 for the static world

		Real friction = 0;

		// Effective mass of the body pair, penalties are stored per unit mass so that the ramping behaves the same
		// for light and heavy bodies
		Real mass = 1;

		int count = 0;

		// Contact basis, the normal points from B to A
		Vector<Real, 3> n;
		Vector<Real, 3> t1;
		Vector<Real, 3> t2;

		AVBDContact<Real> c[MAX_POINTS];
	};

	enum AVBDBodyType
	{
		AVBD_FIXED = 0,
		AVBD_KINEMATIC = 1,
		AVBD_DYNAMIC = 2
	};

	struct AVBDParams
	{
		float alpha = 0.95f;		// stabilization
		float gamma = 0.99f;		// warm start decay
		float betaLin = 1e5f;		// penalty ramping, per unit mass
		float penaltyMin = 1.0f;	// per unit mass
		float penaltyMax = 1e10f;	// per unit mass
		float margin = 0.002f;		// contact margin
		float stickThreshold = 1e-5f;
	};

	// q + w, where w is a small rotation vector in world space
	template<typename Real>
	DYN_FUNC inline Quat<Real> AVBD_Add(const Quat<Real>& q, const Vector<Real, 3>& w)
	{
		Quat<Real> dq = Quat<Real>(w[0], w[1], w[2], Real(0)) * q;
		Quat<Real> r(q.x + Real(0.5) * dq.x, q.y + Real(0.5) * dq.y, q.z + Real(0.5) * dq.z, q.w + Real(0.5) * dq.w);
		return r.normalize();
	}

	// a - b as a rotation vector in world space
	template<typename Real>
	DYN_FUNC inline Vector<Real, 3> AVBD_Sub(const Quat<Real>& a, const Quat<Real>& b)
	{
		Quat<Real> d = a * b.conjugate();
		Real s = d.w < 0 ? Real(-2) : Real(2);
		return Vector<Real, 3>(d.x * s, d.y * s, d.z * s);
	}

	template<typename Real>
	DYN_FUNC inline void AVBD_Basis(const Vector<Real, 3>& n, Vector<Real, 3>& t1, Vector<Real, 3>& t2)
	{
		// Branchless orthonormal basis, Duff et al. 2017
		Real s = n[2] >= 0 ? Real(1) : Real(-1);
		Real a = Real(-1) / (s + n[2]);
		Real b = n[0] * n[1] * a;
		t1 = Vector<Real, 3>(Real(1) + s * n[0] * n[0] * a, s * b, -s * n[0]);
		t2 = Vector<Real, 3>(b, s + n[1] * n[1] * a, -n[1]);
	}

	/**
	 * Solve the symmetric positive definite system [L C^T; C A] [x; y] = [b; c] with an LDL^T factorization,
	 * L is the linear block, A the angular block and C the angular-linear coupling block.
	 */
	template<typename Real>
	DYN_FUNC inline void AVBD_Solve6(
		const SquareMatrix<Real, 3>& Lin,
		const SquareMatrix<Real, 3>& Ang,
		const SquareMatrix<Real, 3>& Cross,
		const Vector<Real, 3>& bLin,
		const Vector<Real, 3>& bAng,
		Vector<Real, 3>& xLin,
		Vector<Real, 3>& xAng)
	{
		Real A[6][6];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				A[i][j] = Lin(i, j);
				A[i + 3][j + 3] = Ang(i, j);
				A[i + 3][j] = Cross(i, j);
				A[j][i + 3] = Cross(i, j);
			}
		}

		Real L[6][6];
		Real D[6];
		for (int j = 0; j < 6; j++)
		{
			Real d = A[j][j];
			for (int k = 0; k < j; k++)
				d -= L[j][k] * L[j][k] * D[k];
			D[j] = d;

			Real invD = d != Real(0) ? Real(1) / d : Real(0);
			for (int i = j + 1; i < 6; i++)
			{
				Real v = A[i][j];
				for (int k = 0; k < j; k++)
					v -= L[i][k] * L[j][k] * D[k];
				L[i][j] = v * invD;
			}
		}

		Real y[6] = { bLin[0], bLin[1], bLin[2], bAng[0], bAng[1], bAng[2] };
		for (int i = 0; i < 6; i++)
			for (int k = 0; k < i; k++)
				y[i] -= L[i][k] * y[k];

		for (int i = 0; i < 6; i++)
			y[i] = D[i] != Real(0) ? y[i] / D[i] : Real(0);

		for (int i = 5; i >= 0; i--)
			for (int k = i + 1; k < 6; k++)
				y[i] -= L[k][i] * y[k];

		xLin = Vector<Real, 3>(y[0], y[1], y[2]);
		xAng = Vector<Real, 3>(y[3], y[4], y[5]);
	}

	/**
	 * Evaluate the linearized constraint and the clamped force of one contact point
	 */
	template<typename Real>
	DYN_FUNC inline void AVBD_EvalContact(
		const AVBDManifold<Real>& m,
		const AVBDContact<Real>& c,
		const Vector<Real, 3>& rAw,
		const Vector<Real, 3>& rBw,
		const Vector<Real, 3>& dxA,
		const Vector<Real, 3>& dwA,
		const Vector<Real, 3>& dxB,
		const Vector<Real, 3>& dwB,
		Real alpha,
		Vector<Real, 3>& C,
		Vector<Real, 3>& F,
		Real& frictionScale,
		Real& bounds)
	{
		const Vector<Real, 3> b[3] = { m.n, m.t1, m.t2 };

		for (int r = 0; r < 3; r++)
		{
			// Only penetration is stabilized, a positive gap may be closed within the step
			Real C0 = (r == 0 && c.C0[0] > Real(0)) ? c.C0[0] : c.C0[r] * (Real(1) - alpha);
			C[r] = C0
				+ b[r].dot(dxA) + rAw.cross(b[r]).dot(dwA)
				- b[r].dot(dxB) - rBw.cross(b[r]).dot(dwB);
			F[r] = c.penalty[r] * C[r] + c.lambda[r];
		}

		// The normal force can only push
		F[0] = F[0] < Real(0) ? F[0] : Real(0);

		// Friction cone
		bounds = -F[0] * m.friction;
		frictionScale = sqrt(F[1] * F[1] + F[2] * F[2]);
		if (frictionScale > bounds && frictionScale > Real(0))
		{
			Real s = bounds / frictionScale;
			F[1] *= s;
			F[2] *= s;
		}
	}

	/**
	 * Save the pose at the beginning of the step, compute the inertial target and move the body to the initial guess
	 */
	template<typename Real>
	DYN_FUNC inline void AVBD_InitBody(
		uint i,
		int type,
		Vector<Real, 3>* pos,
		Quat<Real>* quat,
		Vector<Real, 3>* x0,
		Quat<Real>* q0,
		Vector<Real, 3>* xHat,
		Quat<Real>* qHat,
		const Vector<Real, 3>* velocity,
		const Vector<Real, 3>* angularVelocity,
		Vector<Real, 3>* prevVelocity,
		Real gravity,
		Real h)
	{
		typedef Vector<Real, 3> Coord;

		Coord x = pos[i];
		Quat<Real> q = quat[i];
		x0[i] = x;
		q0[i] = q;

		if (type == AVBD_FIXED)
		{
			xHat[i] = x;
			qHat[i] = q;
			return;
		}

		Coord v = velocity[i];
		Coord w = angularVelocity[i];

		if (type == AVBD_KINEMATIC)
		{
			// Kinematic bodies follow their velocity and are treated as infinitely heavy
			pos[i] = x + v * h;
			quat[i] = AVBD_Add(q, w * h);
			xHat[i] = pos[i];
			qHat[i] = quat[i];
			return;
		}

		Coord g(0, -gravity, 0);

		// Inertial target
		xHat[i] = x + v * h + g * (h * h);
		qHat[i] = AVBD_Add(q, w * h);

		// Adaptive initial guess (Chen et al. 2024): only add the part of gravity the body was actually subject to
		Real weight = 0;
		if (gravity > 0)
		{
			Coord accel = (v - prevVelocity[i]) / h;
			weight = -accel[1] / gravity;
			weight = weight < 0 ? 0 : (weight > 1 ? 1 : weight);
			if (!(weight == weight)) weight = 0;
		}

		pos[i] = x + v * h + g * (weight * h * h);
		quat[i] = qHat[i];
		prevVelocity[i] = v;
	}

	/**
	 * Compute the constraint error at the beginning of the step and decay lambda and penalty (Eq. 19)
	 */
	template<typename Real>
	DYN_FUNC inline void AVBD_BeginManifold(
		AVBDManifold<Real>& m,
		const Vector<Real, 3>* x0,
		const Quat<Real>* q0,
		const AVBDParams& p)
	{
		typedef Vector<Real, 3> Coord;

		if (m.count == 0) return;

		Coord xA = x0[m.bodyA];
		Quat<Real> qA = q0[m.bodyA];
		bool hasB = m.bodyB >= 0;
		Coord xB = hasB ? x0[m.bodyB] : Coord(0);
		Quat<Real> qB = hasB ? q0[m.bodyB] : Quat<Real>::identity();

		for (int i = 0; i < m.count; i++)
		{
			AVBDContact<Real>& c = m.c[i];

			Coord pA = xA + qA.rotate(c.rA);
			Coord pB = hasB ? xB + qB.rotate(c.rB) : c.rB;
			Coord d = pA - pB;

			c.C0 = Coord(m.n.dot(d) + Real(p.margin), m.t1.dot(d), m.t2.dot(d));

			c.lambda = c.lambda * Real(p.alpha * p.gamma);

			Real kMin = Real(p.penaltyMin) * m.mass;
			Real kMax = Real(p.penaltyMax) * m.mass;
			for (int r = 0; r < 3; r++)
			{
				Real k = c.penalty[r] * Real(p.gamma);
				c.penalty[r] = k < kMin ? kMin : (k > kMax ? kMax : k);
			}
		}
	}

	/******************************************************************************************************************
	 * Joints
	 ******************************************************************************************************************/

	enum AVBDJointType
	{
		AVBD_JOINT_BALL = 0,		// point lock
		AVBD_JOINT_HINGE,			// point lock + two angular rows keeping the axes aligned
		AVBD_JOINT_SLIDER,			// two linear rows across the axis + full angular lock
		AVBD_JOINT_FIXED,			// point lock + full angular lock
		AVBD_JOINT_POINT,			// the center of body A is pinned to a world point
		AVBD_JOINT_DISTANCE			// distance between two anchors
	};

	/**
	 * @brief A joint as seen by the AVBD solver. Rows 0-2 are linear, rows 3-5 angular, rows 6-7 the lower and upper
	 * limits of a hinge or slider and row 8 its motor.
	 */
	template<typename Real>
	struct AVBDJoint
	{
		static const int MAX_ROWS = 9;
		static const int ROW_MIN = 6;
		static const int ROW_MAX = 7;
		static const int ROW_MOTOR = 8;

		int type = AVBD_JOINT_BALL;

		int bodyA = -1;
		int bodyB = -1;				// -1 for the static world

		Vector<Real, 3> rA;			// anchor in the local frame of A
		Vector<Real, 3> rB;			// anchor in the local frame of B, or in world space when B is the world
		Vector<Real, 3> axisA;		// hinge and slider axis in the local frame of A
		Vector<Real, 3> axisB;		// hinge axis in the local frame of B, or in world space when B is the world
		Quat<Real> qRel;			// target rotation: qA = qB * qRel, or qA = qRel when B is the world

		Real distance = 0;

		// Hinge angle (radians) or slider displacement (length) limits and motor speed. As in the TJS solver,
		// the hinge angle is the rotation of B relative to A around the axis of A and the slider displacement is
		// (pB - pA) along the axis of A. The motor drives the hinge angle at -speed and the displacement at +speed.
		int useRange = 0;
		int useMotor = 0;
		Real lo = 0;
		Real hi = 0;
		Real motorSpeed = 0;

		Real base = 0;				// hinge angle or slider displacement at the beginning of the step
		Real motorStep = 0;			// motor travel within one step

		Real L = 1;					// length scale turning angular errors into lengths
		Real mass = 1;				// effective mass, penalties are stored per unit mass

		Real C0[MAX_ROWS];
		Real penalty[MAX_ROWS];
		Real lambda[MAX_ROWS];
	};

	/**
	 * Constraint values and world-space Jacobians of all rows of a joint at the given poses
	 */
	template<typename Real>
	struct AVBDJointRows
	{
		uint mask = 0;			// active rows
		bool pointLock = false;	// rows 0-2 lock a point, geometric stiffness applies

		Real C[AVBDJoint<Real>::MAX_ROWS];
		Vector<Real, 3> jlA[AVBDJoint<Real>::MAX_ROWS];
		Vector<Real, 3> jaA[AVBDJoint<Real>::MAX_ROWS];
		Vector<Real, 3> jlB[AVBDJoint<Real>::MAX_ROWS];
		Vector<Real, 3> jaB[AVBDJoint<Real>::MAX_ROWS];

		Real value = 0;			// hinge angle or slider displacement

		Vector<Real, 3> rAw;
		Vector<Real, 3> rBw;
	};

	template<typename Real>
	DYN_FUNC inline void AVBD_EvalJoint(
		const AVBDJoint<Real>& j,
		const Vector<Real, 3>& xA,
		const Quat<Real>& qA,
		const Vector<Real, 3>& xB,
		const Quat<Real>& qB,
		AVBDJointRows<Real>& rows)
	{
		typedef Vector<Real, 3> Coord;

		bool hasB = j.bodyB >= 0;

		rows.rAw = j.type == AVBD_JOINT_POINT ? Coord(0) : qA.rotate(j.rA);
		rows.rBw = hasB ? qB.rotate(j.rB) : Coord(0);

		Coord pA = xA + rows.rAw;
		Coord pB = hasB ? xB + rows.rBw : j.rB;
		Coord d = pA - pB;

		rows.mask = 0;
		rows.pointLock = false;

		auto linearRow = [&](int k, const Coord& n, Real c) {
			rows.C[k] = c;
			rows.jlA[k] = n;
			rows.jaA[k] = rows.rAw.cross(n);
			rows.jlB[k] = -n;
			rows.jaB[k] = -rows.rBw.cross(n);
			rows.mask |= 1u << k;
		};

		auto angularRow = [&](int k, const Coord& a, Real c) {
			rows.C[k] = c;
			rows.jlA[k] = Coord(0);
			rows.jaA[k] = a;
			rows.jlB[k] = Coord(0);
			rows.jaB[k] = -a;
			rows.mask |= 1u << k;
		};

		switch (j.type)
		{
		case AVBD_JOINT_BALL:
		case AVBD_JOINT_HINGE:
		case AVBD_JOINT_FIXED:
		case AVBD_JOINT_POINT:
			for (int k = 0; k < 3; k++)
			{
				Coord e(0);
				e[k] = Real(1);
				linearRow(k, e, d[k]);
			}
			rows.pointLock = true;
			break;
		case AVBD_JOINT_SLIDER:
		{
			Coord a = qA.rotate(j.axisA);
			Coord t1, t2;
			AVBD_Basis(a, t1, t2);
			linearRow(0, t1, t1.dot(d));
			linearRow(1, t2, t2.dot(d));
			break;
		}
		case AVBD_JOINT_DISTANCE:
		{
			Real len = d.norm();
			Coord n = len > Real(1e-9) ? d / len : Coord(0, 1, 0);
			linearRow(0, n, len - j.distance);
			break;
		}
		default:
			break;
		}

		if (j.type == AVBD_JOINT_FIXED || j.type == AVBD_JOINT_SLIDER)
		{
			Quat<Real> target = hasB ? qB * j.qRel : j.qRel;
			Coord e = AVBD_Sub(qA, target);
			for (int k = 0; k < 3; k++)
			{
				Coord u(0);
				u[k] = j.L;
				angularRow(3 + k, u, j.L * e[k]);
			}
		}
		else if (j.type == AVBD_JOINT_HINGE)
		{
			Coord aA = qA.rotate(j.axisA);
			Coord aB = hasB ? qB.rotate(j.axisB) : j.axisB;
			Coord b1, b2;
			AVBD_Basis(aA, b1, b2);
			angularRow(3, b1.cross(aB) * j.L, j.L * b1.dot(aB));
			angularRow(4, b2.cross(aB) * j.L, j.L * b2.dot(aB));
		}

		if ((j.type == AVBD_JOINT_HINGE || j.type == AVBD_JOINT_SLIDER) && (j.useRange || j.useMotor))
		{
			// Gradient of the joint coordinate
			Coord gLA, gAA, gLB, gAB;
			Coord a = qA.rotate(j.axisA);
			if (j.type == AVBD_JOINT_HINGE)
			{
				Quat<Real> r = qB * qA.conjugate();
				if (r.w < 0) r = Quat<Real>(-r.x, -r.y, -r.z, -r.w);
				rows.value = Real(2) * atan2(Coord(r.x, r.y, r.z).dot(a), r.w);

				// angles are turned into lengths like the other angular rows
				gLA = Coord(0);
				gAA = -a * j.L;
				gLB = Coord(0);
				gAB = a * j.L;
			}
			else
			{
				rows.value = -d.dot(a);

				gLA = -a;
				gAA = -(pB - xA).cross(a);
				gLB = a;
				gAB = rows.rBw.cross(a);
			}

			Real scale = j.type == AVBD_JOINT_HINGE ? j.L : Real(1);

			auto gradientRow = [&](int k, Real sign, Real c) {
				rows.C[k] = c;
				rows.jlA[k] = gLA * sign;
				rows.jaA[k] = gAA * sign;
				rows.jlB[k] = gLB * sign;
				rows.jaB[k] = gAB * sign;
				rows.mask |= 1u << k;
			};

			if (j.useRange)
			{
				gradientRow(AVBDJoint<Real>::ROW_MIN, Real(1), scale * (rows.value - j.lo));
				gradientRow(AVBDJoint<Real>::ROW_MAX, Real(-1), scale * (j.hi - rows.value));
			}

			if (j.useMotor)
			{
				Real travel = rows.value - j.base;
				if (j.type == AVBD_JOINT_HINGE)
				{
					// wrap into (-pi, pi]
					const Real pi = Real(3.14159265358979323846);
					while (travel > pi) travel -= 2 * pi;
					while (travel <= -pi) travel += 2 * pi;
					gradientRow(AVBDJoint<Real>::ROW_MOTOR, Real(1), scale * (travel + j.motorStep));
				}
				else
				{
					gradientRow(AVBDJoint<Real>::ROW_MOTOR, Real(1), travel - j.motorStep);
				}
			}
		}
	}

	/**
	 * Force of one joint row. Limit rows are inequalities (C >= 0) and only push, they are inactive while inside
	 * the range. Only equality rows and violated limits are stabilized.
	 */
	template<typename Real>
	DYN_FUNC inline bool AVBD_JointRowForce(
		const AVBDJoint<Real>& j,
		const AVBDJointRows<Real>& rows,
		int k,
		Real alpha,
		Real& C,
		Real& F)
	{
		bool limit = k == AVBDJoint<Real>::ROW_MIN || k == AVBDJoint<Real>::ROW_MAX;
		Real C0 = j.C0[k];
		if (limit && C0 > Real(0)) C0 = Real(0);

		C = rows.C[k] - alpha * C0;
		F = j.penalty[k] * C + j.lambda[k];

		if (limit && F > Real(0))
		{
			F = Real(0);
			return false;
		}
		return true;
	}

	template<typename Real>
	DYN_FUNC inline void AVBD_BeginJoint(
		AVBDJoint<Real>& j,
		const Vector<Real, 3>* x0,
		const Quat<Real>* q0,
		const AVBDParams& p,
		Real h)
	{
		typedef Vector<Real, 3> Coord;

		bool hasB = j.bodyB >= 0;

		AVBDJointRows<Real> rows;
		AVBD_EvalJoint(j, x0[j.bodyA], q0[j.bodyA], hasB ? x0[j.bodyB] : Coord(0), hasB ? q0[j.bodyB] : Quat<Real>::identity(), rows);

		j.base = rows.value;
		j.motorStep = j.motorSpeed * h;

		Real kMin = Real(p.penaltyMin) * j.mass;
		Real kMax = Real(p.penaltyMax) * j.mass;
		for (int k = 0; k < AVBDJoint<Real>::MAX_ROWS; k++)
		{
			// The motor row tracks a velocity and is never stabilized
			j.C0[k] = ((rows.mask & (1u << k)) && k != AVBDJoint<Real>::ROW_MOTOR) ? rows.C[k] : Real(0);
			j.lambda[k] *= Real(p.alpha * p.gamma);

			Real pk = j.penalty[k] * Real(p.gamma);
			j.penalty[k] = pk < kMin ? kMin : (pk > kMax ? kMax : pk);
		}
	}

	// Diagonal approximation of the geometric stiffness of a point lock, see the reference implementation of AVBD
	template<typename Real>
	DYN_FUNC inline Vector<Real, 3> AVBD_PointGeometricStiffness(const Vector<Real, 3>& r, const Vector<Real, 3>& F)
	{
		SquareMatrix<Real, 3> H(0);
		for (int k = 0; k < 3; k++)
		{
			for (int i = 0; i < 3; i++)
			{
				H(i, i) -= r[k] * F[k];
				H(i, k) += r[i] * F[k];
			}
		}
		return Vector<Real, 3>(H.col(0).norm(), H.col(1).norm(), H.col(2).norm());
	}

	template<typename Real>
	DYN_FUNC inline void AVBD_StampJoint(
		const AVBDJoint<Real>& j,
		bool isA,
		const Vector<Real, 3>* pos,
		const Quat<Real>* quat,
		Real alpha,
		SquareMatrix<Real, 3>& lhsLin,
		SquareMatrix<Real, 3>& lhsAng,
		SquareMatrix<Real, 3>& lhsCross,
		Vector<Real, 3>& rhsLin,
		Vector<Real, 3>& rhsAng)
	{
		typedef Vector<Real, 3> Coord;

		bool hasB = j.bodyB >= 0;

		AVBDJointRows<Real> rows;
		AVBD_EvalJoint(j, pos[j.bodyA], quat[j.bodyA], hasB ? pos[j.bodyB] : Coord(0), hasB ? quat[j.bodyB] : Quat<Real>::identity(), rows);

		Coord Fpoint(0);
		for (int k = 0; k < AVBDJoint<Real>::MAX_ROWS; k++)
		{
			if (!(rows.mask & (1u << k))) continue;

			Real C, F;
			if (!AVBD_JointRowForce(j, rows, k, alpha, C, F))
				continue;

			Real K = j.penalty[k];
			if (k < 3) Fpoint[k] = F;

			Coord jl = isA ? rows.jlA[k] : rows.jlB[k];
			Coord ja = isA ? rows.jaA[k] : rows.jaB[k];

			for (int u = 0; u < 3; u++)
			{
				for (int v = 0; v < 3; v++)
				{
					lhsLin(u, v) += K * jl[u] * jl[v];
					lhsAng(u, v) += K * ja[u] * ja[v];
					lhsCross(u, v) += K * ja[u] * jl[v];
				}
			}

			rhsLin += jl * F;
			rhsAng += ja * F;
		}

		if (rows.pointLock)
		{
			Coord g = AVBD_PointGeometricStiffness(isA ? rows.rAw : -rows.rBw, Fpoint);
			for (int k = 0; k < 3; k++)
				lhsAng(k, k) += g[k];
		}
	}

	template<typename Real>
	DYN_FUNC inline void AVBD_DualJoint(
		AVBDJoint<Real>& j,
		const Vector<Real, 3>* pos,
		const Quat<Real>* quat,
		const AVBDParams& p)
	{
		typedef Vector<Real, 3> Coord;

		bool hasB = j.bodyB >= 0;

		AVBDJointRows<Real> rows;
		AVBD_EvalJoint(j, pos[j.bodyA], quat[j.bodyA], hasB ? pos[j.bodyB] : Coord(0), hasB ? quat[j.bodyB] : Quat<Real>::identity(), rows);

		Real beta = Real(p.betaLin) * j.mass;
		Real kMax = Real(p.penaltyMax) * j.mass;
		for (int k = 0; k < AVBDJoint<Real>::MAX_ROWS; k++)
		{
			if (!(rows.mask & (1u << k))) continue;

			Real C, F;
			bool active = AVBD_JointRowForce(j, rows, k, Real(p.alpha), C, F);
			j.lambda[k] = F;

			// Inactive limits keep their penalty
			if (active)
			{
				Real pk = j.penalty[k] + beta * (C < 0 ? -C : C);
				j.penalty[k] = pk < kMax ? pk : kMax;
			}
		}
	}

	/******************************************************************************************************************
	 * Primal
	 ******************************************************************************************************************/

	template<typename Real>
	DYN_FUNC inline void AVBD_StampManifold(
		const AVBDManifold<Real>& mf,
		bool isA,
		const Vector<Real, 3>* pos,
		const Quat<Real>* quat,
		const Vector<Real, 3>* x0,
		const Quat<Real>* q0,
		Real alpha,
		SquareMatrix<Real, 3>& lhsLin,
		SquareMatrix<Real, 3>& lhsAng,
		SquareMatrix<Real, 3>& lhsCross,
		Vector<Real, 3>& rhsLin,
		Vector<Real, 3>& rhsAng)
	{
		typedef Vector<Real, 3> Coord;

		int a = mf.bodyA;
		int b = mf.bodyB;
		bool hasB = b >= 0;

		Quat<Real> qA = quat[a];
		Coord dxA = pos[a] - x0[a];
		Coord dwA = AVBD_Sub(qA, q0[a]);

		Quat<Real> qB = hasB ? quat[b] : Quat<Real>::identity();
		Coord dxB = hasB ? pos[b] - x0[b] : Coord(0);
		Coord dwB = hasB ? AVBD_Sub(qB, q0[b]) : Coord(0);

		const Coord basis[3] = { mf.n, mf.t1, mf.t2 };
		Real sign = isA ? Real(1) : Real(-1);

		for (int i = 0; i < mf.count; i++)
		{
			const AVBDContact<Real>& c = mf.c[i];

			Coord rAw = qA.rotate(c.rA);
			Coord rBw = hasB ? qB.rotate(c.rB) : Coord(0);

			Coord C, F;
			Real frictionScale, bounds;
			AVBD_EvalContact(mf, c, rAw, rBw, dxA, dwA, dxB, dwB, alpha, C, F, frictionScale, bounds);

			Coord r = isA ? rAw : rBw;
			for (int k = 0; k < 3; k++)
			{
				Coord jl = basis[k] * sign;
				Coord ja = r.cross(jl);
				Real K = c.penalty[k];

				for (int u = 0; u < 3; u++)
				{
					for (int v = 0; v < 3; v++)
					{
						lhsLin(u, v) += K * jl[u] * jl[v];
						lhsAng(u, v) += K * ja[u] * ja[v];
						lhsCross(u, v) += K * ja[u] * jl[v];
					}
				}

				rhsLin += jl * F[k];
				rhsAng += ja * F[k];
			}
		}
	}

	/**
	 * Adjacency entries are (index << 2 | kind << 1 | side), kind 0 is a contact manifold, 1 a joint,
	 * side 0 means the body is body A of that constraint.
	 */
	DYN_FUNC inline uint AVBD_Entry(uint index, uint kind, uint side) { return (index << 2) | (kind << 1) | side; }

	/**
	 * One primal step of a dynamic body: assemble and solve its 6x6 Newton system (Eqs. 4-6)
	 */
	template<typename Real>
	DYN_FUNC inline void AVBD_PrimalBody(
		uint body,
		Vector<Real, 3>* pos,
		Quat<Real>* quat,
		const Vector<Real, 3>* x0,
		const Quat<Real>* q0,
		const Vector<Real, 3>* xHat,
		const Quat<Real>* qHat,
		const Real* mass,
		const SquareMatrix<Real, 3>* inertia,
		const uint* adjStart,
		const uint* adj,
		const AVBDManifold<Real>* manifolds,
		const AVBDJoint<Real>* joints,
		Real alpha,
		Real h)
	{
		typedef Vector<Real, 3> Coord;
		typedef SquareMatrix<Real, 3> Matrix;

		Real ih2 = Real(1) / (h * h);
		Real m = mass[body];
		Matrix I = inertia[body];

		// A body without mass, e.g. one made of medial primitives only, has no well defined Newton step
		if (!(m > Real(0)))
			return;

		Coord x = pos[body];
		Quat<Real> q = quat[body];

		Matrix lhsLin = Matrix::identityMatrix() * (m * ih2);
		Matrix lhsAng = I * ih2;
		Matrix lhsCross(0);

		Coord rhsLin = (x - xHat[body]) * (m * ih2);
		Coord rhsAng = I * AVBD_Sub(q, qHat[body]) * ih2;

		for (uint e = adjStart[body]; e < adjStart[body + 1]; e++)
		{
			uint entry = adj[e];
			bool isA = (entry & 1) == 0;

			if (entry & 2)
				AVBD_StampJoint(joints[entry >> 2], isA, pos, quat, alpha, lhsLin, lhsAng, lhsCross, rhsLin, rhsAng);
			else
				AVBD_StampManifold(manifolds[entry >> 2], isA, pos, quat, x0, q0, alpha, lhsLin, lhsAng, lhsCross, rhsLin, rhsAng);
		}

		Coord dx, dw;
		AVBD_Solve6(lhsLin, lhsAng, lhsCross, -rhsLin, -rhsAng, dx, dw);

		pos[body] = x + dx;
		quat[body] = AVBD_Add(q, dw);
	}

	/**
	 * Dual step of a manifold: update lambda, ramp the penalty and detect sticking contacts (Eqs. 11, 16)
	 */
	template<typename Real>
	DYN_FUNC inline void AVBD_DualManifold(
		AVBDManifold<Real>& m,
		const Vector<Real, 3>* pos,
		const Quat<Real>* quat,
		const Vector<Real, 3>* x0,
		const Quat<Real>* q0,
		const AVBDParams& p)
	{
		typedef Vector<Real, 3> Coord;

		if (m.count == 0) return;

		int a = m.bodyA;
		int b = m.bodyB;
		bool hasB = b >= 0;

		Quat<Real> qA = quat[a];
		Coord dxA = pos[a] - x0[a];
		Coord dwA = AVBD_Sub(qA, q0[a]);

		Quat<Real> qB = hasB ? quat[b] : Quat<Real>::identity();
		Coord dxB = hasB ? pos[b] - x0[b] : Coord(0);
		Coord dwB = hasB ? AVBD_Sub(qB, q0[b]) : Coord(0);

		Real beta = Real(p.betaLin) * m.mass;
		Real kMax = Real(p.penaltyMax) * m.mass;

		for (int i = 0; i < m.count; i++)
		{
			AVBDContact<Real>& c = m.c[i];

			Coord rAw = qA.rotate(c.rA);
			Coord rBw = hasB ? qB.rotate(c.rB) : Coord(0);

			Coord C, F;
			Real frictionScale, bounds;
			AVBD_EvalContact(m, c, rAw, rBw, dxA, dwA, dxB, dwB, Real(p.alpha), C, F, frictionScale, bounds);

			c.lambda = F;

			// Ramp the penalty only while the force is inside its bounds
			if (F[0] < 0)
			{
				Real k = c.penalty[0] + beta * (C[0] < 0 ? -C[0] : C[0]);
				c.penalty[0] = k < kMax ? k : kMax;
			}

			if (frictionScale <= bounds)
			{
				for (int r = 1; r < 3; r++)
				{
					Real k = c.penalty[r] + beta * (C[r] < 0 ? -C[r] : C[r]);
					c.penalty[r] = k < kMax ? k : kMax;
				}
				c.stick = sqrt(C[1] * C[1] + C[2] * C[2]) < Real(p.stickThreshold) ? 1 : 0;
			}
		}
	}

	/**
	 * BDF1 velocities and the derived rotation and world inertia
	 */
	template<typename Real>
	DYN_FUNC inline void AVBD_FinishBody(
		uint i,
		int type,
		const Vector<Real, 3>* pos,
		const Quat<Real>* quat,
		const Vector<Real, 3>* x0,
		const Quat<Real>* q0,
		Vector<Real, 3>* velocity,
		Vector<Real, 3>* angularVelocity,
		SquareMatrix<Real, 3>* rotation,
		SquareMatrix<Real, 3>* inertia,
		const SquareMatrix<Real, 3>* initialInertia,
		Real h)
	{
		if (type == AVBD_FIXED) return;

		if (type == AVBD_DYNAMIC)
		{
			velocity[i] = (pos[i] - x0[i]) / h;
			angularVelocity[i] = AVBD_Sub(quat[i], q0[i]) / h;
		}

		SquareMatrix<Real, 3> R = quat[i].toMatrix3x3();
		rotation[i] = R;
		inertia[i] = R * initialInertia[i] * R.transpose();
	}
}
