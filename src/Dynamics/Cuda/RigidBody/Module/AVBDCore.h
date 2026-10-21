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
			C[r] = c.C0[r] * (Real(1) - alpha)
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
		Real alpha,
		Real h)
	{
		typedef Vector<Real, 3> Coord;
		typedef SquareMatrix<Real, 3> Matrix;

		Real ih2 = Real(1) / (h * h);
		Real m = mass[body];
		Matrix I = inertia[body];

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
			const AVBDManifold<Real>& mf = manifolds[entry >> 1];
			bool isA = (entry & 1) == 0;

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
