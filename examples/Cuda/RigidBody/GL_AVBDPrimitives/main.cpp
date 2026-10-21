#include <GlfwApp.h>

#include <SceneGraph.h>

#include <RigidBody/RigidBodySystem.h>

#include <GLRenderEngine.h>
#include <GLSurfaceVisualModule.h>
#include <GLWireframeVisualModule.h>

#include <Mapping/DiscreteElementsToTriangleSet.h>

#include <cstring>

using namespace std;
using namespace dyno;

/**
 * A pile of boxes, spheres and capsules solved with AVBD. Run with "tjs" as the first argument to compare
 * against the TJS solver on the same scene.
 */
std::shared_ptr<SceneGraph> createScene(bool useAVBD)
{
	std::shared_ptr<SceneGraph> scn = std::make_shared<SceneGraph>();

	auto rigid = scn->addNode(std::make_shared<RigidBodySystem<DataType3f>>());
	rigid->varConstraintSolver()->setCurrentKey(useAVBD ?
		RigidBodySystem<DataType3f>::AVBD : RigidBodySystem<DataType3f>::TJS);

	const float h = 0.05f;
	const int dim = 6;
	const int layers = 8;

	RigidBodyInfo rigidBody;
	rigidBody.friction = 0.5f;

	BoxInfo box;
	box.halfLength = Vec3f(h);

	SphereInfo sphere;
	sphere.radius = h;

	CapsuleInfo capsule;
	capsule.radius = 0.6f * h;
	capsule.halfLength = h;

	for (int j = 0; j < layers; j++)
	{
		for (int i = 0; i < dim; i++)
		{
			for (int k = 0; k < dim; k++)
			{
				rigidBody.position = Vec3f((2.2f * i - dim) * h, h + 2.5f * j * h, (2.2f * k - dim) * h);

				switch ((i + j + k) % 3)
				{
				case 0:
					rigid->addBox(box, rigidBody);
					break;
				case 1:
					rigid->addSphere(sphere, rigidBody);
					break;
				default:
					rigid->addCapsule(capsule, rigidBody);
					break;
				}
			}
		}
	}

	auto mapper = std::make_shared<DiscreteElementsToTriangleSet<DataType3f>>();
	rigid->stateTopology()->connect(mapper->inDiscreteElements());
	rigid->graphicsPipeline()->pushModule(mapper);

	auto sRender = std::make_shared<GLSurfaceVisualModule>();
	sRender->varBaseColor()->setValue(Color::SteelBlue2());
	sRender->varAlpha()->setValue(1.0f);
	mapper->outTriangleSet()->connect(sRender->inTriangleSet());
	rigid->graphicsPipeline()->pushModule(sRender);

	auto wireRender = std::make_shared<GLWireframeVisualModule>();
	wireRender->varBaseColor()->setValue(Color(0, 0, 0));
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
