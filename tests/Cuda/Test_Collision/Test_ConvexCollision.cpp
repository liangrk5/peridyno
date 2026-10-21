#include "gtest/gtest.h"

#include "Primitive/Primitive3D.h"
#include "Collision/ConvexCollision.h"

using namespace dyno;

using Coord3D = Vector<float, 3>;
using Convex = ConvexCollision<float>;

TEST(ConvexCollision, SphereSphere)
{
	TConvexContact<float> c;
	auto A = makeSphereHull(TSphere3D<float>(Coord3D(0, 0, 0), 1));

	EXPECT_TRUE(Convex::query(c, A, makeSphereHull(TSphere3D<float>(Coord3D(2.5, 0, 0), 1)), 1));
	EXPECT_NEAR(c.distance, 0.5f, 2e-3f);
	EXPECT_NEAR(c.normal[0], 1.0f, 2e-3f);

	EXPECT_TRUE(Convex::query(c, A, makeSphereHull(TSphere3D<float>(Coord3D(1.5, 0, 0), 1)), 1));
	EXPECT_NEAR(c.distance, -0.5f, 2e-2f);
	EXPECT_NEAR(c.normal[0], 1.0f, 2e-2f);
}

TEST(ConvexCollision, BoxBox)
{
	TConvexContact<float> c;
	TOrientedBox3D<float> b0(Coord3D(0, 0, 0), Coord3D(1, 0, 0), Coord3D(0, 1, 0), Coord3D(0, 0, 1), Coord3D(1, 1, 1));
	TOrientedBox3D<float> b1(Coord3D(0.3, 1.4, 0.2), Coord3D(1, 0, 0), Coord3D(0, 1, 0), Coord3D(0, 0, 1), Coord3D(0.5, 0.5, 0.5));

	EXPECT_TRUE(Convex::query(c, makeSphereHull(b0), makeSphereHull(b1), 1));
	EXPECT_NEAR(c.distance, -0.1f, 1e-2f);
	EXPECT_NEAR(c.normal[1], 1.0f, 1e-2f);

	b1.center = Coord3D(0.3, 1.2, 0.2);
	EXPECT_TRUE(Convex::query(c, makeSphereHull(b1), makeSphereHull(b0), 1));
	EXPECT_NEAR(c.distance, -0.3f, 1e-2f);
	EXPECT_NEAR(c.normal[1], -1.0f, 1e-2f);
}

TEST(ConvexCollision, BoxCapsuleAndSphere)
{
	TConvexContact<float> c;
	TOrientedBox3D<float> box(Coord3D(0, 0, 0), Coord3D(1, 0, 0), Coord3D(0, 1, 0), Coord3D(0, 0, 1), Coord3D(1, 1, 1));

	TCapsule3D<float> cap(Coord3D(-0.5, 1.3, 0), Coord3D(0.5, 1.3, 0), 0.2f);
	EXPECT_TRUE(Convex::query(c, makeSphereHull(box), makeSphereHull(cap), 1));
	EXPECT_NEAR(c.distance, 0.1f, 2e-3f);
	EXPECT_NEAR(c.normal[1], 1.0f, 2e-3f);

	EXPECT_TRUE(Convex::query(c, makeSphereHull(box), makeSphereHull(TSphere3D<float>(Coord3D(0.9, 0, 0), 0.5f)), 1));
	EXPECT_NEAR(c.distance, -0.6f, 2e-2f);
	EXPECT_NEAR(c.normal[0], 1.0f, 2e-2f);
}

TEST(ConvexCollision, MedialPrimitives)
{
	TConvexContact<float> c;

	TMedialCone3D<float> cone(Coord3D(0, 0.1, 0), Coord3D(1, 0.3, 0), 0.2f, 0.4f);
	TTriangle3D<float> tri(Coord3D(-2, 0, -2), Coord3D(2, 0, -2), Coord3D(0, 0, 3));
	EXPECT_TRUE(Convex::query(c, makeSphereHull(tri), makeSphereHull(cone), 1));
	EXPECT_NEAR(c.distance, -0.1f, 2e-2f);
	EXPECT_NEAR(std::abs(c.normal[1]), 1.0f, 2e-2f);

	TMedialSlab3D<float> slab(Coord3D(0, 0, 0), Coord3D(1, 0, 0), Coord3D(0, 0, 1), 0.1f, 0.1f, 0.1f);
	TCapsule3D<float> cap(Coord3D(0.2, 0.25, 0.2), Coord3D(0.2, 1.0, 0.2), 0.1f);
	EXPECT_TRUE(Convex::query(c, makeSphereHull(slab), makeSphereHull(cap), 1));
	EXPECT_NEAR(c.distance, 0.05f, 2e-3f);
}

TEST(ConvexCollision, FarApart)
{
	TConvexContact<float> c;
	TTet3D<float> tet(Coord3D(0, 0, 0), Coord3D(1, 0, 0), Coord3D(0, 1, 0), Coord3D(0, 0, 1));
	EXPECT_FALSE(Convex::query(c, makeSphereHull(tet), makeSphereHull(TSphere3D<float>(Coord3D(5, 5, 5), 1)), 1));
}
