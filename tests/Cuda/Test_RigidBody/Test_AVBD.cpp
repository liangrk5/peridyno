#include "gtest/gtest.h"

#include "RigidBody/Module/AVBDCore.h"

#include <vector>
#include <cmath>

using namespace dyno;

using Coord = Vector<float, 3>;
using Matrix = SquareMatrix<float, 3>;
using TQuat = Quat<float>;
using Manifold = AVBDManifold<float>;

/**
 * A minimal CPU driver of the AVBD routines: boxes on a static plane and on each other, contacts are generated
 * from the box corners and warm started by corner index, colors are given by the caller.
 */
class AVBDWorld
{
public:
	struct Plane
	{
		Coord n;		// unit normal
		float d = 0;	// n.x = d on the plane
	};

	void addBox(Coord center, Coord halfExtent, float density, TQuat q = TQuat::identity(), int type = AVBD_DYNAMIC)
	{
		float m = density * 8 * halfExtent[0] * halfExtent[1] * halfExtent[2];
		Coord s = halfExtent * 2;
		Matrix I0(0);
		I0(0, 0) = m / 12 * (s[1] * s[1] + s[2] * s[2]);
		I0(1, 1) = m / 12 * (s[0] * s[0] + s[2] * s[2]);
		I0(2, 2) = m / 12 * (s[0] * s[0] + s[1] * s[1]);

		pos.push_back(center);
		quat.push_back(q);
		half.push_back(halfExtent);
		mass.push_back(m);
		inertia0.push_back(I0);
		Matrix R = q.toMatrix3x3();
		inertia.push_back(R * I0 * R.transpose());
		rotation.push_back(R);
		velocity.push_back(Coord(0));
		angularVelocity.push_back(Coord(0));
		prevVelocity.push_back(Coord(0));
		type_.push_back(type);
		color.push_back(0);
		friction.push_back(0.5f);
	}

	void step(float h, int iterations)
	{
		uint N = (uint)pos.size();
		x0.resize(N); q0.resize(N); xHat.resize(N); qHat.resize(N);

		collide();

		// Adjacency
		std::vector<std::vector<uint>> lists(N);
		for (uint j = 0; j < manifolds.size(); j++)
		{
			lists[manifolds[j].bodyA].push_back(j << 1);
			if (manifolds[j].bodyB >= 0 && type_[manifolds[j].bodyB] == AVBD_DYNAMIC)
				lists[manifolds[j].bodyB].push_back((j << 1) | 1);
		}
		adjStart.assign(N + 1, 0);
		adj.clear();
		for (uint i = 0; i < N; i++)
		{
			adjStart[i] = (uint)adj.size();
			adj.insert(adj.end(), lists[i].begin(), lists[i].end());
		}
		adjStart[N] = (uint)adj.size();

		for (uint i = 0; i < N; i++)
			AVBD_InitBody(i, type_[i], pos.data(), quat.data(), x0.data(), q0.data(), xHat.data(), qHat.data(),
				velocity.data(), angularVelocity.data(), prevVelocity.data(), gravity, h);

		for (auto& m : manifolds)
			AVBD_BeginManifold(m, x0.data(), q0.data(), params);

		int maxColor = 0;
		for (uint i = 0; i < N; i++) maxColor = std::max(maxColor, color[i]);

		for (int it = 0; it < iterations; it++)
		{
			for (int c = 0; c <= maxColor; c++)
				for (uint i = 0; i < N; i++)
					if (type_[i] == AVBD_DYNAMIC && color[i] == c)
						AVBD_PrimalBody(i, pos.data(), quat.data(), x0.data(), q0.data(), xHat.data(), qHat.data(),
							mass.data(), inertia.data(), adjStart.data(), adj.data(), manifolds.data(), params.alpha, h);

			for (auto& m : manifolds)
				AVBD_DualManifold(m, pos.data(), quat.data(), x0.data(), q0.data(), params);
		}

		for (uint i = 0; i < N; i++)
			AVBD_FinishBody(i, type_[i], pos.data(), quat.data(), x0.data(), q0.data(), velocity.data(), angularVelocity.data(),
				rotation.data(), inertia.data(), inertia0.data(), h);
	}

	Coord corner(uint i, int k) const
	{
		Coord local((k & 1) ? half[i][0] : -half[i][0], (k & 2) ? half[i][1] : -half[i][1], (k & 4) ? half[i][2] : -half[i][2]);
		return pos[i] + quat[i].rotate(local);
	}

	std::vector<Coord> pos;
	std::vector<TQuat> quat;
	std::vector<Coord> velocity;
	std::vector<Coord> angularVelocity;
	std::vector<int> color;
	std::vector<float> friction;
	std::vector<Plane> planes;

	AVBDParams params;
	float gravity = 9.8f;

private:
	// key: (plane or body below) * 8 + corner, -1 - p for planes
	struct Feature { int a; int b; int corner; };

	void addContact(std::vector<Manifold>& out, std::vector<Feature>& features, int a, int b, int k, Coord n, Coord pA, Coord pB)
	{
		Manifold m;
		m.bodyA = a;
		m.bodyB = b;
		m.n = n;
		AVBD_Basis(m.n, m.t1, m.t2);
		m.friction = b >= 0 ? (friction[a] + friction[b]) / 2 : friction[a];
		m.mass = (b >= 0 && type_[b] == AVBD_DYNAMIC) ? mass[a] * mass[b] / (mass[a] + mass[b]) : mass[a];
		m.count = 1;

		AVBDContact<float>& c = m.c[0];
		c.rA = quat[a].conjugate().rotate(pA - pos[a]);
		c.rB = b >= 0 ? quat[b].conjugate().rotate(pB - pos[b]) : pB;
		c.penalty = Coord(params.penaltyMin * m.mass);
		c.lambda = Coord(0);
		c.stick = 0;

		// Warm start from the same feature
		for (size_t j = 0; j < oldFeatures.size(); j++)
		{
			const Feature& f = oldFeatures[j];
			if (f.a == a && f.b == b && f.corner == k)
			{
				const AVBDContact<float>& o = manifolds[j].c[0];
				c.lambda = o.lambda;
				c.penalty = o.penalty;
				c.stick = o.stick;
				if (o.stick)
				{
					c.rA = o.rA;
					c.rB = o.rB;
				}
				break;
			}
		}

		out.push_back(m);
		features.push_back({ a, b, k });
	}

	void collide()
	{
		std::vector<Manifold> out;
		std::vector<Feature> features;

		const float speculative = 0.05f;
		uint N = (uint)pos.size();
		for (uint i = 0; i < N; i++)
		{
			if (type_[i] != AVBD_DYNAMIC) continue;

			for (size_t p = 0; p < planes.size(); p++)
			{
				for (int k = 0; k < 8; k++)
				{
					Coord c = corner(i, k);
					float d = planes[p].n.dot(c) - planes[p].d;
					if (d < speculative)
						addContact(out, features, i, -1 - (int)p, k, planes[p].n, c, c - planes[p].n * d);
				}
			}

			// Axis aligned stacking: the bottom corners of box i against the top face of any box right below it
			for (uint j = 0; j < N; j++)
			{
				if (j == i) continue;
				float top = pos[j][1] + half[j][1];
				if (std::abs(pos[i][0] - pos[j][0]) > half[j][0] || std::abs(pos[i][2] - pos[j][2]) > half[j][2]) continue;
				if (pos[i][1] < pos[j][1]) continue;

				for (int k = 0; k < 8; k++)
				{
					if (k & 2) continue;
					Coord c = corner(i, k);
					float d = c[1] - top;
					if (d < speculative)
						addContact(out, features, i, (int)j, k, Coord(0, 1, 0), c, Coord(c[0], top, c[2]));
				}
			}
		}

		// Plane contacts use bodyB = -1, the plane index only lives in the feature
		for (auto& m : out)
			if (m.bodyB < 0) m.bodyB = -1;

		manifolds = out;
		oldFeatures = features;
	}

	std::vector<Coord> half;
	std::vector<float> mass;
	std::vector<Matrix> inertia;
	std::vector<Matrix> inertia0;
	std::vector<Matrix> rotation;
	std::vector<Coord> prevVelocity;
	std::vector<int> type_;

	std::vector<Coord> x0, xHat;
	std::vector<TQuat> q0, qHat;

	std::vector<Manifold> manifolds;
	std::vector<Feature> oldFeatures;
	std::vector<uint> adjStart;
	std::vector<uint> adj;
};

TEST(AVBD, Solve6)
{
	// A random SPD system, compare against a direct product
	Matrix L(4, 1, 0.5, 1, 5, 0.2, 0.5, 0.2, 3);
	Matrix A(2, 0.3, 0.1, 0.3, 2.5, 0.4, 0.1, 0.4, 1.8);
	Matrix C(0.1, 0.2, 0.3, -0.2, 0.1, 0.05, 0.3, -0.1, 0.2);
	Coord xl(1, -2, 0.5), xa(0.3, 0.7, -1.1);

	Coord bl = L * xl + C.transpose() * xa;
	Coord ba = C * xl + A * xa;

	Coord rl, ra;
	AVBD_Solve6(L, A, C, bl, ba, rl, ra);
	for (int i = 0; i < 3; i++)
	{
		EXPECT_NEAR(rl[i], xl[i], 1e-4f);
		EXPECT_NEAR(ra[i], xa[i], 1e-4f);
	}
}

TEST(AVBD, QuaternionDifference)
{
	TQuat q = TQuat(0.3f, Coord(0, 1, 0));
	Coord w(0.01f, -0.02f, 0.015f);
	Coord r = AVBD_Sub(AVBD_Add(q, w), q);
	for (int i = 0; i < 3; i++)
		EXPECT_NEAR(r[i], w[i], 1e-5f);
}

TEST(AVBD, BoxRestsOnGround)
{
	AVBDWorld w;
	w.planes.push_back({ Coord(0, 1, 0), 0 });
	w.addBox(Coord(0, 0.5f, 0), Coord(0.5f), 1000);

	for (int f = 0; f < 300; f++)
		w.step(1.0f / 60, 10);

	EXPECT_NEAR(w.pos[0][1], 0.5f - w.params.margin, 2e-3f);
	EXPECT_NEAR(w.pos[0][0], 0.0f, 1e-4f);
	EXPECT_NEAR(w.pos[0][2], 0.0f, 1e-4f);
	EXPECT_LT(w.velocity[0].norm(), 1e-2f);
	EXPECT_LT(w.angularVelocity[0].norm(), 1e-2f);
}

TEST(AVBD, BoxDropsAndSettles)
{
	AVBDWorld w;
	w.planes.push_back({ Coord(0, 1, 0), 0 });
	w.addBox(Coord(0, 1.0f, 0), Coord(0.5f), 1000);

	for (int f = 0; f < 300; f++)
		w.step(1.0f / 60, 10);

	EXPECT_NEAR(w.pos[0][1], 0.5f - w.params.margin, 5e-3f);
	EXPECT_LT(w.velocity[0].norm(), 1e-2f);
}

class AVBDStack : public ::testing::TestWithParam<float> {};

TEST_P(AVBDStack, TenBoxes)
{
	// The penalty is stored per unit mass, a stack must behave the same whatever the density
	const int num = 10;
	AVBDWorld w;
	w.planes.push_back({ Coord(0, 1, 0), 0 });
	for (int i = 0; i < num; i++)
	{
		w.addBox(Coord(0, 0.5f + i, 0), Coord(0.5f), GetParam());
		w.color[i] = i % 2;
	}

	for (int f = 0; f < 300; f++)
		w.step(1.0f / 60, 10);

	for (int i = 0; i < num; i++)
	{
		// every contact sinks by at most the margin plus a small residual
		EXPECT_NEAR(w.pos[i][1], 0.5f + i, (i + 1) * (w.params.margin + 2e-3f)) << "box " << i;
		EXPECT_NEAR(w.pos[i][0], 0.0f, 1e-3f) << "box " << i;
		EXPECT_LT(w.velocity[i].norm(), 1e-2f) << "box " << i;
	}
}

INSTANTIATE_TEST_SUITE_P(AVBD, AVBDStack, ::testing::Values(1.0f, 1000.0f, 7800.0f));

TEST(AVBD, FrictionOnSlope)
{
	const float theta = 20.0f * float(M_PI) / 180.0f;
	Coord n(-std::sin(theta), std::cos(theta), 0);
	Coord downhill(-std::cos(theta), -std::sin(theta), 0);
	TQuat q(theta, Coord(0, 0, 1));

	// Sticks when mu > tan(theta)
	{
		AVBDWorld w;
		w.planes.push_back({ n, 0 });
		w.addBox(n * 0.5f, Coord(0.5f), 1000, q);
		w.friction[0] = 0.6f;

		Coord start = w.pos[0];
		for (int f = 0; f < 120; f++)
			w.step(1.0f / 60, 10);

		EXPECT_LT((w.pos[0] - start).dot(downhill), 5e-3f);
	}

	// Slides with a = g (sin - mu cos) otherwise
	{
		AVBDWorld w;
		w.planes.push_back({ n, 0 });
		w.addBox(n * 0.5f, Coord(0.5f), 1000, q);
		w.friction[0] = 0.1f;

		Coord start = w.pos[0];
		const int frames = 60;
		for (int f = 0; f < frames; f++)
			w.step(1.0f / 60, 10);

		float t = frames / 60.0f;
		float expected = 0.5f * 9.8f * (std::sin(theta) - 0.1f * std::cos(theta)) * t * t;
		float travelled = (w.pos[0] - start).dot(downhill);
		EXPECT_NEAR(travelled, expected, 0.15f * expected);
	}
}
