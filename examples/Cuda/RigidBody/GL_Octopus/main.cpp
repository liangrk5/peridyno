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
#include <Mapping/AnchorPointToPointSet.h>

#include "Collision/NeighborElementQuery.h"
#include <random>

using namespace std;
using namespace dyno;

std::shared_ptr<SceneGraph> creatCar()
{
	std::shared_ptr<SceneGraph> scn = std::make_shared<SceneGraph>();


	auto rigid = scn->addNode(std::make_shared<RigidBodySystem<DataType3f>>());

	uint N = 100;
	Vec3f offset(0);
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.0, 1.0);
	float range = M_PI / 3;
	for (uint i = 0; i < N; i++)
	{
		double random_x = dis(gen);
		double random_z = dis(gen);
		offset = Vec3f(random_x, i * 0.4, random_z);
		RigidBodyInfo rigidBody;
		rigidBody.bodyId = i;
		BoxInfo sphere;
		sphere.center = Vec3f(0, 2, 0) + offset;
		sphere.halfLength = Vec3f(0.15, 0.15, 0.15);
		auto BodyActor = rigid->addBox(sphere, rigidBody);

		BoxInfo box1, box2, box3, box4, box5;
		box1.center = Vec3f(0.15, 1.95, 0.15) + offset;
		box1.halfLength = Vec3f(0.015, 0.015, 0.05);
		box1.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor11 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(0.23, 1.95, 0.23) + offset;
		box2.halfLength = Vec3f(0.015, 0.015, 0.05);
		box2.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor12 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(0.31, 1.95, 0.31) + offset;
		box3.halfLength = Vec3f(0.015, 0.015, 0.05);
		box3.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor13 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(0.39, 1.95, 0.39) + offset;
		box4.halfLength = Vec3f(0.015, 0.015, 0.05);
		box4.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor14 = rigid->addBox(box4, rigidBody);


		box1.center = Vec3f(-0.15, 1.95, -0.15) + offset;
		box1.halfLength = Vec3f(0.015, 0.015, 0.05);
		box1.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor21 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(-0.23, 1.95, -0.23) + offset;
		box2.halfLength = Vec3f(0.015, 0.015, 0.05);
		box2.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor22 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(-0.31, 1.95, -0.31) + offset;
		box3.halfLength = Vec3f(0.015, 0.015, 0.05);
		box3.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor23 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(-0.39, 1.95, -0.39) + offset;
		box4.halfLength = Vec3f(0.015, 0.015, 0.05);
		box4.rot = Quat1f(M_PI / 4, Vec3f(0, 1, 0));
		auto legActor24 = rigid->addBox(box4, rigidBody);

		box1.center = Vec3f(0.15, 1.95, -0.15) + offset;
		box1.halfLength = Vec3f(0.015, 0.015, 0.05);
		box1.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor31 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(0.23, 1.95, -0.23) + offset;
		box2.halfLength = Vec3f(0.015, 0.015, 0.05);
		box2.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor32 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(0.31, 1.95, -0.31) + offset;
		box3.halfLength = Vec3f(0.015, 0.015, 0.05);
		box3.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor33 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(0.39, 1.95, -0.39) + offset;
		box4.halfLength = Vec3f(0.015, 0.015, 0.05);
		box4.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor34 = rigid->addBox(box4, rigidBody);

		box1.center = Vec3f(-0.15, 1.95, 0.15) + offset;
		box1.halfLength = Vec3f(0.015, 0.015, 0.05);
		box1.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor41 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(-0.23, 1.95, 0.23) + offset;
		box2.halfLength = Vec3f(0.015, 0.015, 0.05);
		box2.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor42 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(-0.31, 1.95, 0.31) + offset;
		box3.halfLength = Vec3f(0.015, 0.015, 0.05);
		box3.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor43 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(-0.39, 1.95, 0.39) + offset;
		box4.halfLength = Vec3f(0.015, 0.015, 0.05);
		box4.rot = Quat1f(-M_PI / 4, Vec3f(0, 1, 0));
		auto legActor44 = rigid->addBox(box4, rigidBody);

		box1.center = Vec3f(0, 1.95, 0.21) + offset;
		box1.halfLength = Vec3f(0.015, 0.015, 0.05);
		box1.rot = Quat1f(0, 0, 0, 1);
		auto legActor51 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(0, 1.95, 0.32) + offset;
		box2.halfLength = Vec3f(0.015, 0.015, 0.05);
		box2.rot = Quat1f(0, 0, 0, 1);
		auto legActor52 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(0, 1.95, 0.43) + offset;
		box3.halfLength = Vec3f(0.015, 0.015, 0.05);
		box3.rot = Quat1f(0, 0, 0, 1);
		auto legActor53 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(0, 1.95, 0.54) + offset;
		box4.halfLength = Vec3f(0.015, 0.015, 0.05);
		box4.rot = Quat1f(0, 0, 0, 1);
		auto legActor54 = rigid->addBox(box4, rigidBody);

		box1.center = Vec3f(0, 1.95, -0.21) + offset;
		box1.halfLength = Vec3f(0.015, 0.015, 0.05);
		box1.rot = Quat1f(0, 0, 0, 1);
		auto legActor61 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(0, 1.95, -0.32) + offset;
		box2.halfLength = Vec3f(0.015, 0.015, 0.05);
		box2.rot = Quat1f(0, 0, 0, 1);
		auto legActor62 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(0, 1.95, -0.43) + offset;
		box3.halfLength = Vec3f(0.015, 0.015, 0.05);
		box3.rot = Quat1f(0, 0, 0, 1);
		auto legActor63 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(0, 1.95, -0.54) + offset;
		box4.halfLength = Vec3f(0.015, 0.015, 0.05);
		box4.rot = Quat1f(0, 0, 0, 1);
		auto legActor64 = rigid->addBox(box4, rigidBody);

		box1.center = Vec3f(0.21, 1.95, 0) + offset;
		box1.halfLength = Vec3f(0.05, 0.015, 0.015);
		box1.rot = Quat1f(0, 0, 0, 1);
		auto legActor71 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(0.32, 1.95, 0) + offset;
		box2.halfLength = Vec3f(0.05, 0.015, 0.015);
		box2.rot = Quat1f(0, 0, 0, 1);
		auto legActor72 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(0.43, 1.95, 0) + offset;
		box3.halfLength = Vec3f(0.05, 0.015, 0.015);
		box3.rot = Quat1f(0, 0, 0, 1);
		auto legActor73 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(0.54, 1.95, 0) + offset;
		box4.halfLength = Vec3f(0.05, 0.015, 0.015);
		box4.rot = Quat1f(0, 0, 0, 1);
		auto legActor74 = rigid->addBox(box4, rigidBody);

		box1.center = Vec3f(-0.21, 1.95, 0) + offset;
		box1.halfLength = Vec3f(0.05, 0.015, 0.015);
		box1.rot = Quat1f(0, 0, 0, 1);
		auto legActor81 = rigid->addBox(box1, rigidBody);
		box2.center = Vec3f(-0.32, 1.95, 0) + offset;
		box2.halfLength = Vec3f(0.05, 0.015, 0.015);
		box2.rot = Quat1f(0, 0, 0, 1);
		auto legActor82 = rigid->addBox(box2, rigidBody);
		box3.center = Vec3f(-0.43, 1.95, 0) + offset;
		box3.halfLength = Vec3f(0.05, 0.015, 0.015);
		box3.rot = Quat1f(0, 0, 0, 1);
		auto legActor83 = rigid->addBox(box3, rigidBody);
		box4.center = Vec3f(-0.54, 1.95, 0) + offset;
		box4.halfLength = Vec3f(0.05, 0.015, 0.015);
		box4.rot = Quat1f(0, 0, 0, 1);
		auto legActor84 = rigid->addBox(box4, rigidBody);

		auto& hingeJoint1 = rigid->createHingeJoint(legActor81, BodyActor);
		hingeJoint1.setAnchorPoint((legActor81->center + BodyActor->center) / 2);
		hingeJoint1.setAxis(Vec3f(0, 0, 1));
		hingeJoint1.setRange(-range, range);

		auto& hingeJoint2 = rigid->createHingeJoint(legActor81, legActor82);
		hingeJoint2.setAnchorPoint((legActor81->center + legActor82->center) / 2);
		hingeJoint2.setAxis(Vec3f(0, 0, 1));
		hingeJoint2.setRange(-range, range);

		auto& hingeJoint3 = rigid->createHingeJoint(legActor82, legActor83);
		hingeJoint3.setAnchorPoint((legActor82->center + legActor83->center) / 2);
		hingeJoint3.setAxis(Vec3f(0, 0, 1));
		hingeJoint3.setRange(-range, range);

		auto& hingeJoint4 = rigid->createHingeJoint(legActor83, legActor84);
		hingeJoint4.setAnchorPoint((legActor83->center + legActor84->center) / 2);
		hingeJoint4.setAxis(Vec3f(0, 0, 1));
		hingeJoint4.setRange(-range, range);

		auto& hingeJoint5 = rigid->createHingeJoint(legActor71, BodyActor);
		hingeJoint5.setAnchorPoint((legActor71->center + BodyActor->center) / 2);
		hingeJoint5.setAxis(Vec3f(0, 0, 1));
		hingeJoint5.setRange(-range, range);

		auto& hingeJoint6 = rigid->createHingeJoint(legActor71, legActor72);
		hingeJoint6.setAnchorPoint((legActor71->center + legActor72->center) / 2);
		hingeJoint6.setAxis(Vec3f(0, 0, 1));
		hingeJoint6.setRange(-range, range);

		auto& hingeJoint7 = rigid->createHingeJoint(legActor72, legActor73);
		hingeJoint7.setAnchorPoint((legActor72->center + legActor73->center) / 2);
		hingeJoint7.setAxis(Vec3f(0, 0, 1));
		hingeJoint7.setRange(-range, range);

		auto& hingeJoint8 = rigid->createHingeJoint(legActor73, legActor74);
		hingeJoint8.setAnchorPoint((legActor73->center + legActor74->center) / 2);
		hingeJoint8.setAxis(Vec3f(0, 0, 1));
		hingeJoint8.setRange(-range, range);

		auto& hingeJoint9 = rigid->createHingeJoint(legActor61, BodyActor);
		hingeJoint9.setAnchorPoint((legActor61->center + BodyActor->center) / 2);
		hingeJoint9.setAxis(Vec3f(1, 0, 0));
		hingeJoint9.setRange(-range, range);

		auto& hingeJoint10 = rigid->createHingeJoint(legActor61, legActor62);
		hingeJoint10.setAnchorPoint((legActor61->center + legActor62->center) / 2);
		hingeJoint10.setAxis(Vec3f(1, 0, 0));
		hingeJoint10.setRange(-range, range);

		auto& hingeJoint11 = rigid->createHingeJoint(legActor62, legActor63);
		hingeJoint11.setAnchorPoint((legActor62->center + legActor63->center) / 2);
		hingeJoint11.setAxis(Vec3f(1, 0, 0));
		hingeJoint11.setRange(-range, range);

		auto& hingeJoint12 = rigid->createHingeJoint(legActor63, legActor64);
		hingeJoint12.setAnchorPoint((legActor63->center + legActor64->center) / 2);
		hingeJoint12.setAxis(Vec3f(1, 0, 0));
		hingeJoint12.setRange(-range, range);

		auto& hingeJoint13 = rigid->createHingeJoint(legActor51, BodyActor);
		hingeJoint13.setAnchorPoint((legActor51->center + BodyActor->center) / 2);
		hingeJoint13.setAxis(Vec3f(1, 0, 0));
		hingeJoint13.setRange(-range, range);

		auto& hingeJoint14 = rigid->createHingeJoint(legActor51, legActor52);
		hingeJoint14.setAnchorPoint((legActor51->center + legActor52->center) / 2);
		hingeJoint14.setAxis(Vec3f(1, 0, 0));
		hingeJoint14.setRange(-range, range);

		auto& hingeJoint15 = rigid->createHingeJoint(legActor52, legActor53);
		hingeJoint15.setAnchorPoint((legActor52->center + legActor53->center) / 2);
		hingeJoint15.setAxis(Vec3f(1, 0, 0));
		hingeJoint15.setRange(-range, range);

		auto& hingeJoint16 = rigid->createHingeJoint(legActor53, legActor54);
		hingeJoint16.setAnchorPoint((legActor53->center + legActor54->center) / 2);
		hingeJoint16.setAxis(Vec3f(1, 0, 0));
		hingeJoint16.setRange(-range, range);

		auto& hingeJoint17 = rigid->createHingeJoint(legActor11, BodyActor);
		hingeJoint17.setAnchorPoint((legActor11->center + BodyActor->center) / 2);
		hingeJoint17.setAxis(Vec3f(1, 0, -1));
		hingeJoint17.setRange(-range, range);

		auto& hingeJoint18 = rigid->createHingeJoint(legActor11, legActor12);
		hingeJoint18.setAnchorPoint((legActor11->center + legActor12->center) / 2);
		hingeJoint18.setAxis(Vec3f(1, 0, -1));
		hingeJoint18.setRange(-range, range);

		auto& hingeJoint19 = rigid->createHingeJoint(legActor12, legActor13);
		hingeJoint19.setAnchorPoint((legActor12->center + legActor13->center) / 2);
		hingeJoint19.setAxis(Vec3f(1, 0, -1));
		hingeJoint19.setRange(-range, range);

		auto& hingeJoint20 = rigid->createHingeJoint(legActor13, legActor14);
		hingeJoint20.setAnchorPoint((legActor13->center + legActor14->center) / 2);
		hingeJoint20.setAxis(Vec3f(1, 0, -1));
		hingeJoint20.setRange(-range, range);

		auto& hingeJoint21 = rigid->createHingeJoint(legActor21, BodyActor);
		hingeJoint21.setAnchorPoint((legActor21->center + BodyActor->center) / 2);
		hingeJoint21.setAxis(Vec3f(1, 0, -1));
		hingeJoint21.setRange(-range, range);

		auto& hingeJoint22 = rigid->createHingeJoint(legActor21, legActor22);
		hingeJoint22.setAnchorPoint((legActor21->center + legActor22->center) / 2);
		hingeJoint22.setAxis(Vec3f(1, 0, -1));
		hingeJoint22.setRange(-range, range);

		auto& hingeJoint23 = rigid->createHingeJoint(legActor22, legActor23);
		hingeJoint23.setAnchorPoint((legActor22->center + legActor23->center) / 2);
		hingeJoint23.setAxis(Vec3f(1, 0, -1));
		hingeJoint23.setRange(-range, range);

		auto& hingeJoint24 = rigid->createHingeJoint(legActor23, legActor24);
		hingeJoint24.setAnchorPoint((legActor23->center + legActor24->center) / 2);
		hingeJoint24.setAxis(Vec3f(1, 0, -1));
		hingeJoint24.setRange(-range, range);

		auto& hingeJoint25 = rigid->createHingeJoint(legActor31, BodyActor);
		hingeJoint25.setAnchorPoint((legActor31->center + BodyActor->center) / 2);
		hingeJoint25.setAxis(Vec3f(1, 0, 1));
		hingeJoint25.setRange(-range, range);

		auto& hingeJoint26 = rigid->createHingeJoint(legActor31, legActor32);
		hingeJoint26.setAnchorPoint((legActor31->center + legActor32->center) / 2);
		hingeJoint26.setAxis(Vec3f(1, 0, 1));
		hingeJoint26.setRange(-range, range);

		auto& hingeJoint27 = rigid->createHingeJoint(legActor32, legActor33);
		hingeJoint27.setAnchorPoint((legActor32->center + legActor33->center) / 2);
		hingeJoint27.setAxis(Vec3f(1, 0, 1));
		hingeJoint27.setRange(-range, range);

		auto& hingeJoint28 = rigid->createHingeJoint(legActor33, legActor34);
		hingeJoint28.setAnchorPoint((legActor33->center + legActor34->center) / 2);
		hingeJoint28.setAxis(Vec3f(1, 0, 1));
		hingeJoint28.setRange(-range, range);

		auto& hingeJoint29 = rigid->createHingeJoint(legActor41, BodyActor);
		hingeJoint29.setAnchorPoint((legActor41->center + BodyActor->center) / 2);
		hingeJoint29.setAxis(Vec3f(1, 0, 1));
		hingeJoint29.setRange(-range, range);

		auto& hingeJoint30 = rigid->createHingeJoint(legActor41, legActor42);
		hingeJoint30.setAnchorPoint((legActor41->center + legActor42->center) / 2);
		hingeJoint30.setAxis(Vec3f(1, 0, 1));
		hingeJoint30.setRange(-range, range);

		auto& hingeJoint31 = rigid->createHingeJoint(legActor42, legActor43);
		hingeJoint31.setAnchorPoint((legActor42->center + legActor43->center) / 2);
		hingeJoint31.setAxis(Vec3f(1, 0, 1));
		hingeJoint31.setRange(-range, range);

		auto& hingeJoint32 = rigid->createHingeJoint(legActor43, legActor44);
		hingeJoint32.setAnchorPoint((legActor43->center + legActor44->center) / 2);
		hingeJoint32.setAxis(Vec3f(1, 0, 1));
		hingeJoint32.setRange(-range, range);


	}
	
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
	app.setSceneGraph(creatCar());
	app.initialize(1280, 768);
	app.mainLoop();

	return 0;
}


