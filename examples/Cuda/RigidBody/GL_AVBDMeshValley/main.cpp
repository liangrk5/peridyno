#include <GlfwApp.h>

#include <SceneGraph.h>

#include <RigidBody/RigidBodySystem.h>
#include <RigidBody/Module/TJSConstraintSolver.h>
#include <RigidBody/Module/ContactsUnion.h>

#include <Collision/NeighborElementQuery.h>
#include <Collision/CollistionDetectionBoundingBox.h>
#include <Collision/CollistionDetectionTriangleSet.h>
#include <Collision/RigidContactDetection.h>

#include <Topology/TriangleSet.h>

#include <GLRenderEngine.h>
#include <GLSurfaceVisualModule.h>
#include <GLWireframeVisualModule.h>

#include <Mapping/DiscreteElementsToTriangleSet.h>

#include <cmath>
#include <cstring>

using namespace std;
using namespace dyno;

/**
 * Several hundred boxes, spheres and capsules pouring down a chute into a rippled valley with pegs and a staircase,
 * all static geometry is a single triangle mesh. Run with "tjs" as the first argument to compare against the TJS
 * solver on the same scene.
 */

/**
 * A static triangle mesh with per-vertex colors, built procedurally so that the example needs no asset.
 */
class ValleyMesh : public Node
{
	DECLARE_CLASS(ValleyMesh);
public:
	ValleyMesh();

	DEF_INSTANCE_STATE(TriangleSet<DataType3f>, TriangleSet, "Static environment");

	DEF_ARRAY_STATE(Vec3f, VertexColor, DeviceType::GPU, "");

private:
	struct Builder
	{
		std::vector<Vec3f> points;
		std::vector<Vec3f> colors;
		std::vector<Topology::Triangle> triangles;

		uint vertex(Vec3f p, Vec3f c)
		{
			points.push_back(p);
			colors.push_back(c);
			return uint(points.size() - 1);
		}

		// Counter-clockwise seen from outside
		void quad(uint a, uint b, uint c, uint d)
		{
			triangles.push_back(Topology::Triangle(a, b, c));
			triangles.push_back(Topology::Triangle(a, c, d));
		}

		// An oriented box with shared corners, so that the diagonals of its faces are recognized as flat edges
		void box(Vec3f center, Vec3f half, Quat<float> q, Vec3f color)
		{
			uint v[8];
			for (int i = 0; i < 8; i++)
			{
				Vec3f local((i & 1 ? 1 : -1) * half.x, (i & 2 ? 1 : -1) * half.y, (i & 4 ? 1 : -1) * half.z);
				v[i] = vertex(center + q.rotate(local), color);
			}
			quad(v[0], v[4], v[6], v[2]);	// -x
			quad(v[1], v[3], v[7], v[5]);	// +x
			quad(v[0], v[1], v[5], v[4]);	// -y
			quad(v[2], v[6], v[7], v[3]);	// +y
			quad(v[0], v[2], v[3], v[1]);	// -z
			quad(v[4], v[5], v[7], v[6]);	// +z
		}

		// A closed vertical prism with n sides
		void prism(Vec3f base, float radius, float height, int n, Vec3f color)
		{
			uint bottom = uint(points.size());
			for (int i = 0; i < n; i++)
			{
				float a = 2.0f * float(M_PI) * i / n;
				Vec3f r(radius * cos(a), 0, radius * sin(a));
				vertex(base + r, color);
				vertex(base + r + Vec3f(0, height, 0), color);
			}
			uint cb = vertex(base, color);
			uint ct = vertex(base + Vec3f(0, height, 0), color);
			for (int i = 0; i < n; i++)
			{
				uint b0 = bottom + 2 * i, t0 = b0 + 1;
				uint b1 = bottom + 2 * ((i + 1) % n), t1 = b1 + 1;
				quad(b0, t0, t1, b1);
				triangles.push_back(Topology::Triangle(ct, t1, t0));
				triangles.push_back(Topology::Triangle(cb, b0, b1));
			}
		}
	};
};

IMPLEMENT_CLASS(ValleyMesh);

namespace
{
	const float PI_F = float(M_PI);

	float smoothstep(float e0, float e1, float x)
	{
		float t = std::min(std::max((x - e0) / (e1 - e0), 0.0f), 1.0f);
		return t * t * (3.0f - 2.0f * t);
	}

	const float EXTENT = 2.5f;

	// A flat basin with a central mound and ripples, rising to a rim
	float terrainHeight(float x, float z)
	{
		float r = sqrt(x * x + z * z);
		float rim = 0.6f * smoothstep(1.2f, EXTENT, r);
		float mound = 0.15f * exp(-r * r / 0.15f);
		float ripple = 0.03f * sin(4.0f * x + 1.0f) * cos(3.5f * z) + 0.015f * sin(9.0f * x - 7.0f * z);
		return 0.1f + rim + mound + ripple;
	}

	Vec3f lerp(Vec3f a, Vec3f b, float t) { return a * (1.0f - t) + b * t; }

	Vec3f terrainColor(float y, float slope)
	{
		Vec3f sand(0.86f, 0.78f, 0.58f);
		Vec3f grass(0.42f, 0.62f, 0.34f);
		Vec3f rock(0.52f, 0.48f, 0.45f);
		Vec3f c = lerp(sand, grass, smoothstep(0.12f, 0.35f, y));
		return lerp(c, rock, smoothstep(0.35f, 0.65f, slope));
	}

	// The chute runs from its top end along the xz diagonal towards the center of the valley
	const Vec3f CHUTE_TOP(-2.2f, 1.5f, -2.2f);
	const float CHUTE_LENGTH = 2.2f;
	const float CHUTE_DROP = 0.95f;
	const float CHUTE_WIDTH = 0.42f;
	const float CHUTE_WALL = 0.16f;
	const float THICKNESS = 0.02f;
}

ValleyMesh::ValleyMesh()
	: Node()
{
	Builder b;

	// Terrain as a height field with shared vertices
	const int N = 100;
	uint first = uint(b.points.size());
	for (int j = 0; j <= N; j++)
	{
		for (int i = 0; i <= N; i++)
		{
			float x = -EXTENT + 2.0f * EXTENT * i / N;
			float z = -EXTENT + 2.0f * EXTENT * j / N;
			float y = terrainHeight(x, z);
			const float e = 0.01f;
			float gx = (terrainHeight(x + e, z) - terrainHeight(x - e, z)) / (2 * e);
			float gz = (terrainHeight(x, z + e) - terrainHeight(x, z - e)) / (2 * e);
			b.vertex(Vec3f(x, y, z), terrainColor(y, sqrt(gx * gx + gz * gz)));
		}
	}
	for (int j = 0; j < N; j++)
	{
		for (int i = 0; i < N; i++)
		{
			uint v00 = first + j * (N + 1) + i;
			uint v10 = v00 + 1;
			uint v01 = v00 + (N + 1);
			uint v11 = v01 + 1;
			b.quad(v00, v01, v11, v10);
		}
	}

	// Chute: a floor and two side walls tilted along the diagonal, closed at the top
	Vec3f wood(0.62f, 0.40f, 0.24f);
	Vec3f dir = Vec3f(1, 0, 1).normalize();
	float run = CHUTE_LENGTH;
	float slope = atan2(CHUTE_DROP, run);
	float length = sqrt(run * run + CHUTE_DROP * CHUTE_DROP);
	Vec3f mid = CHUTE_TOP + dir * (0.5f * run) - Vec3f(0, 0.5f * CHUTE_DROP, 0);

	// Local frame of the chute: x along the chute (downhill), y up, z across
	Quat<float> yaw(-0.25f * PI_F, Vec3f(0, 1, 0));
	Quat<float> pitch(-slope, Vec3f(0, 0, 1));
	Quat<float> frame = yaw * pitch;

	b.box(mid, Vec3f(0.5f * length, 0.5f * THICKNESS, 0.5f * CHUTE_WIDTH + THICKNESS), frame, wood);
	for (int s = -1; s <= 1; s += 2)
	{
		Vec3f offset = frame.rotate(Vec3f(0, 0.5f * CHUTE_WALL, s * (0.5f * CHUTE_WIDTH + 0.5f * THICKNESS)));
		b.box(mid + offset, Vec3f(0.5f * length, 0.5f * CHUTE_WALL, 0.5f * THICKNESS), frame, wood);
	}
	{
		Vec3f offset = frame.rotate(Vec3f(-0.5f * length, 0.5f * CHUTE_WALL, 0));
		b.box(mid + offset, Vec3f(0.5f * THICKNESS, 0.5f * CHUTE_WALL, 0.5f * CHUTE_WIDTH + THICKNESS), frame, wood);
	}

	// Supports of the chute
	Vec3f steel(0.35f, 0.37f, 0.42f);
	for (int k = 0; k < 3; k++)
	{
		float t = 0.15f + 0.3f * k;
		Vec3f top = CHUTE_TOP + dir * (t * run) - Vec3f(0, t * CHUTE_DROP + 0.012f, 0);
		float ground = terrainHeight(top.x, top.z) - 0.05f;
		float h = top.y - ground;
		b.prism(Vec3f(top.x, ground, top.z), 0.035f, h, 8, steel);
	}

	// A ring of pegs around the central mound
	Vec3f stone(0.78f, 0.76f, 0.72f);
	for (int k = 0; k < 7; k++)
	{
		float a = 2.0f * PI_F * k / 7 + 0.3f;
		float x = 0.65f * cos(a), z = 0.65f * sin(a);
		float ground = terrainHeight(x, z) - 0.05f;
		b.prism(Vec3f(x, ground, z), 0.07f, 0.4f, 12, stone);
	}

	// A staircase climbing out of the basin on the +x side
	Vec3f brick(0.70f, 0.36f, 0.30f);
	for (int k = 0; k < 7; k++)
	{
		float x = 0.95f + 0.13f * k;
		float top = 0.22f + 0.07f * k;
		float bottom = 0.0f;
		b.box(Vec3f(x, 0.5f * (top + bottom), 0.2f), Vec3f(0.065f, 0.5f * (top - bottom), 0.35f), Quat<float>(), brick);
	}

	auto ts = this->stateTriangleSet()->allocate();
	ts->setPoints(b.points);
	ts->setTriangles(b.triangles);
	ts->update();

	this->stateVertexColor()->resize(b.colors.size());
	this->stateVertexColor()->getData().assign(b.colors);

	auto render = std::make_shared<GLSurfaceVisualModule>();
	render->varColorMode()->setCurrentKey(GLSurfaceVisualModule::CM_Vertex);
	render->varRoughness()->setValue(0.8f);
	this->stateTriangleSet()->connect(render->inTriangleSet());
	this->stateVertexColor()->connect(render->inColor());
	this->graphicsPipeline()->pushModule(render);
}

/**
 * A rigid body system that collides against an environment mesh with either solver.
 */
class MeshRigidBodySystem : public RigidBodySystem<DataType3f>
{
	DECLARE_CLASS(MeshRigidBodySystem);
public:
	MeshRigidBodySystem()
		: RigidBodySystem<DataType3f>()
	{
		this->inTriangleSet()->tagOptional(true);

		// The base constructor built the default TJS pipeline, rebuild it with the mesh
		for (auto m : mSolverModules)
			this->animationPipeline()->popModule(m);
		mSolverModules.clear();
		MeshRigidBodySystem::setupTJSPipeline();
	}

	DEF_INSTANCE_IN(TriangleSet<DataType3f>, TriangleSet, "Static environment mesh");

protected:
	void setupTJSPipeline() override
	{
		auto elementQuery = std::make_shared<NeighborElementQuery<DataType3f>>();
		this->stateTopology()->connect(elementQuery->inDiscreteElements());
		this->stateCollisionMask()->connect(elementQuery->inCollisionMask());
		this->stateAttribute()->connect(elementQuery->inAttribute());
		this->animationPipeline()->pushModule(elementQuery);

		auto cdMesh = std::make_shared<CollistionDetectionTriangleSet<DataType3f>>();
		this->stateTopology()->connect(cdMesh->inDiscreteElements());
		this->inTriangleSet()->connect(cdMesh->inTriangleSet());
		this->animationPipeline()->pushModule(cdMesh);

		auto cdBV = std::make_shared<CollistionDetectionBoundingBox<DataType3f>>();
		this->stateTopology()->connect(cdBV->inDiscreteElements());
		this->animationPipeline()->pushModule(cdBV);

		auto merge = std::make_shared<ContactsUnion<DataType3f>>();
		elementQuery->outContacts()->connect(merge->inContactsA());
		cdMesh->outContacts()->connect(merge->inContactsB());
		this->animationPipeline()->pushModule(merge);

		auto mergeAll = std::make_shared<ContactsUnion<DataType3f>>();
		merge->outContacts()->connect(mergeAll->inContactsA());
		cdBV->outContacts()->connect(mergeAll->inContactsB());
		this->animationPipeline()->pushModule(mergeAll);

		auto solver = std::make_shared<TJSConstraintSolver<DataType3f>>();
		this->stateTimeStep()->connect(solver->inTimeStep());
		this->varFrictionEnabled()->quote(solver->varFrictionEnabled());
		this->varGravityEnabled()->quote(solver->varGravityEnabled());
		this->varGravityValue()->quote(solver->varGravityValue());
		this->varSlop()->quote(solver->varSlop());
		this->stateMass()->connect(solver->inMass());
		this->stateCenter()->connect(solver->inCenter());
		this->stateVelocity()->connect(solver->inVelocity());
		this->stateAngularVelocity()->connect(solver->inAngularVelocity());
		this->stateRotationMatrix()->connect(solver->inRotationMatrix());
		this->stateInertia()->connect(solver->inInertia());
		this->stateQuaternion()->connect(solver->inQuaternion());
		this->stateInitialInertia()->connect(solver->inInitialInertia());
		this->stateAttribute()->connect(solver->inAttribute());
		this->stateFrictionCoefficients()->connect(solver->inFrictionCoefficients());
		this->stateTopology()->connect(solver->inDiscreteElements());
		mergeAll->outContacts()->connect(solver->inContacts());
		this->animationPipeline()->pushModule(solver);

		mSolverModules = { elementQuery, cdMesh, cdBV, merge, mergeAll, solver };
		mActiveSolver = TJS;
	}

	void setupAVBDPipeline() override
	{
		mSolverModules.clear();
		auto contactDetection = std::dynamic_pointer_cast<RigidContactDetection<DataType3f>>(this->pushAVBDModules(true, true));
		this->inTriangleSet()->connect(contactDetection->inTriangleSet());
	}
};

IMPLEMENT_CLASS(MeshRigidBodySystem);

std::shared_ptr<SceneGraph> createScene(bool useAVBD)
{
	std::shared_ptr<SceneGraph> scn = std::make_shared<SceneGraph>();
	scn->setLowerBound(Vec3f(-5.0f, -1.0f, -5.0f));
	scn->setUpperBound(Vec3f(5.0f, 6.0f, 5.0f));

	auto valley = scn->addNode(std::make_shared<ValleyMesh>());

	auto rigid = scn->addNode(std::make_shared<MeshRigidBodySystem>());
	rigid->varConstraintSolver()->setCurrentKey(useAVBD ?
		RigidBodySystem<DataType3f>::AVBD : RigidBodySystem<DataType3f>::TJS);
	valley->stateTriangleSet()->connect(rigid->inTriangleSet());

	const float h = 0.035f;

	BoxInfo box;
	box.halfLength = Vec3f(h);

	BoxInfo plank;
	plank.halfLength = Vec3f(2.0f * h, 0.6f * h, h);

	SphereInfo sphere;
	sphere.radius = h;

	CapsuleInfo capsule;
	capsule.radius = 0.6f * h;
	capsule.halfLength = 1.2f * h;

	RigidBodyInfo rigidBody;
	rigidBody.friction = 0.4f;

	uint seed = 1;
	auto random = [&seed]() { seed = seed * 1664525u + 1013904223u; return float(seed >> 8) / float(1 << 24); };

	auto addBody = [&](Vec3f p, int type) {
		rigidBody.position = p;
		Vec3f axis(random() - 0.5f, random() - 0.5f, random() - 0.5f);
		rigidBody.angle = axis.norm() > 1e-3f ? Quat<float>(2.0f * PI_F * random(), axis.normalize()) : Quat<float>();
		switch (type % 4)
		{
		case 0: rigid->addBox(box, rigidBody); break;
		case 1: rigid->addSphere(sphere, rigidBody); break;
		case 2: rigid->addCapsule(capsule, rigidBody); break;
		default: rigid->addBox(plank, rigidBody); break;
		}
	};

	// A column of bodies dropped into the top of the chute, they stream down one after another
	Vec3f dir = Vec3f(1, 0, 1).normalize();
	Vec3f across = Vec3f(1, 0, -1).normalize();
	Vec3f chuteStart = CHUTE_TOP + dir * 0.25f;
	for (int j = 0; j < 16; j++)
		for (int i = 0; i < 3; i++)
			for (int k = 0; k < 3; k++)
			{
				Vec3f p = chuteStart + dir * (0.11f * i) + across * (0.11f * (k - 1)) + Vec3f(0, 0.12f + 0.11f * j, 0);
				addBody(p, i + j + k);
			}

	// Rain over the valley
	const int dim = 12;
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < dim; i++)
			for (int k = 0; k < dim; k++)
			{
				float x = (i - 0.5f * (dim - 1)) * 0.15f + 0.2f;
				float z = (k - 0.5f * (dim - 1)) * 0.15f + 0.2f;
				Vec3f p(x + 0.02f * (random() - 0.5f), 1.1f + 0.15f * j, z + 0.02f * (random() - 0.5f));
				addBody(p, i + 2 * j + 3 * k);
			}

	auto mapper = std::make_shared<DiscreteElementsToTriangleSet<DataType3f>>();
	rigid->stateTopology()->connect(mapper->inDiscreteElements());
	rigid->graphicsPipeline()->pushModule(mapper);

	auto sRender = std::make_shared<GLSurfaceVisualModule>();
	sRender->varBaseColor()->setValue(Color(0.95f, 0.55f, 0.15f));
	sRender->varRoughness()->setValue(0.4f);
	mapper->outTriangleSet()->connect(sRender->inTriangleSet());
	rigid->graphicsPipeline()->pushModule(sRender);

	auto wireRender = std::make_shared<GLWireframeVisualModule>();
	wireRender->varBaseColor()->setValue(Color(0.15f, 0.08f, 0.02f));
	mapper->outTriangleSet()->connect(wireRender->inEdgeSet());
	rigid->graphicsPipeline()->pushModule(wireRender);

	return scn;
}

int main(int argc, char** argv)
{
	bool useAVBD = !(argc > 1 && strcmp(argv[1], "tjs") == 0);

	GlfwApp app;
	app.setSceneGraph(createScene(useAVBD));
	app.initialize(1280, 768);
	app.mainLoop();

	return 0;
}
