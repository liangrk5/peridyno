#include <GlfwApp.h>

#include <SceneGraph.h>

#include <RigidBody/RigidBodySystem.h>

#include <GLRenderEngine.h>
#include <GLPointVisualModule.h>
#include <GLSurfaceVisualModule.h>
#include <GLWireframeVisualModule.h>

#include <Mapping/DiscreteElementsToTriangleSet.h>
#include <Mapping/ContactsToEdgeSet.h>
#include <Mapping/ContactsToPointSet.h>

#include "Collision/NeighborElementQuery.h"


using namespace std;
using namespace dyno;

std::shared_ptr<SceneGraph> creatBricks()
{
	std::shared_ptr<SceneGraph> scn = std::make_shared<SceneGraph>();

	auto rigid = scn->addNode(std::make_shared<RigidBodySystem<DataType3f>>());

	BoxInfo newBox, oldBox;
	SphereInfo sphere;
	RigidBodyInfo rigidBody;
	sphere.center = Vec3f(0.07, 0.12, 0.03);
	sphere.radius = 0.03;

	rigidBody.linearVelocity = Vec3f(0, 0, -2.0);

	auto body3 = rigid->addSphere(sphere, rigidBody, 100);

	rigidBody.linearVelocity = Vec3f(0, 0, 0);

	for (int i = 0; i < 10; i++)
	{
		newBox.center = Vec3f(0, 0.12, 0 - 0.1 * i);
		newBox.halfLength = Vec3f(0.01, 0.1, 0.01);

		oldBox.center = Vec3f(0.07, 0.12, 0 - 0.1 * i);
		oldBox.halfLength = Vec3f(0.05, 0.1, 0.01);
		auto body1 = rigid->addBox(newBox, rigidBody);
		auto body2 = rigid->addBox(oldBox, rigidBody, 10);

		auto& joint = rigid->createHingeJoint(body1, body2);
		joint.setAnchorPoint(Vec3f(0.015, 0.12, 0 - 0.1 * i));
		joint.setAxis(Vec3f(0, 1, 0));
		joint.setRange(-M_PI / 6, M_PI / 6);
		joint.setMaxForceAndTorque(100, 100);
		rigid->createUnilateralFixedJointStable(body1);
	}

	/*auto& joint2 = rigid->*/
	//joint2.setAnchorPoint(body1->center);

	auto mapper = std::make_shared<DiscreteElementsToTriangleSet<DataType3f>>();
	rigid->stateTopology()->connect(mapper->inDiscreteElements());
	rigid->graphicsPipeline()->pushModule(mapper);

	auto sRender = std::make_shared<GLSurfaceVisualModule>();
	sRender->setColor(Color(1, 1, 0));
	sRender->setAlpha(1.0f);
	mapper->outTriangleSet()->connect(sRender->inTriangleSet());
	rigid->graphicsPipeline()->pushModule(sRender);

	//TODO: to enable using internal modules inside a node
	//Visualize contact normals
	auto elementQuery = std::make_shared<NeighborElementQuery<DataType3f>>();
	rigid->stateTopology()->connect(elementQuery->inDiscreteElements());
	rigid->stateCollisionMask()->connect(elementQuery->inCollisionMask());
	rigid->graphicsPipeline()->pushModule(elementQuery);

	auto contactMapper = std::make_shared<ContactsToEdgeSet<DataType3f>>();
	elementQuery->outContacts()->connect(contactMapper->inContacts());
	contactMapper->varScale()->setValue(0.02);
	rigid->graphicsPipeline()->pushModule(contactMapper);

	auto wireRender = std::make_shared<GLWireframeVisualModule>();
	wireRender->setColor(Color(0, 0, 1));
	contactMapper->outEdgeSet()->connect(wireRender->inEdgeSet());
	rigid->graphicsPipeline()->pushModule(wireRender);

	//Visualize contact points
	auto contactPointMapper = std::make_shared<ContactsToPointSet<DataType3f>>();
	elementQuery->outContacts()->connect(contactPointMapper->inContacts());
	rigid->graphicsPipeline()->pushModule(contactPointMapper);

	auto pointRender = std::make_shared<GLPointVisualModule>();
	pointRender->setColor(Color(1, 0, 0));
	pointRender->varPointSize()->setValue(0.003f);
	contactPointMapper->outPointSet()->connect(pointRender->inPointSet());
	rigid->graphicsPipeline()->pushModule(pointRender);

	return scn;
}

int main()
{
	GlfwApp app;
	app.setSceneGraph(creatBricks());
	app.initialize(1280, 768);
	app.mainLoop();

	return 0;
}


