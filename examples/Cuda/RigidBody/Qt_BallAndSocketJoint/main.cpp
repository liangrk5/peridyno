 #include <QtApp.h>

#include <SceneGraph.h>
#include <RigidBody/Vechicle.h>

#include <GLRenderEngine.h>
#include <GLPointVisualModule.h>
#include <GLSurfaceVisualModule.h>
#include <GLWireframeVisualModule.h>

#include <Mapping/DiscreteElementsToTriangleSet.h>
#include <Mapping/ContactsToEdgeSet.h>
#include <Mapping/ContactsToPointSet.h>
#include <Mapping/AnchorPointToPointSet.h>

#include "Collision/NeighborElementQuery.h"
#include <Module/GLPhotorealisticInstanceRender.h>


#include <map>

#include "GltfLoader.h"
using namespace std;
using namespace dyno;

std::shared_ptr<SceneGraph> creatBricks()
{
	std::shared_ptr<SceneGraph> scn = std::make_shared<SceneGraph>();


	BoxInfo box;
	RigidBodyInfo rA;
	rA.bodyId = 1;
	rA.linearVelocity = Vec3f(1, 0.0, 0.0);
	box.center = Vec3f(0, 0.0, 0);
	box.halfLength = Vec3f(0.05, 0.05, 0.05);
	auto oldBoxActor = rigid->addBox(box, rA);

	JointBody->inTextureMesh()->connect(prRender->inTextureMesh());
	JointBody->stateInstanceTransform()->connect(prRender->inTransform());
	JointBody->graphicsPipeline()->pushModule(prRender);
	JointBody->varGravityEnabled()->setValue(false);
	auto texMesh = JointBody->inTextureMesh()->constDataPtr();
	std::vector<int> joint_Id = { 0, 1};
	std::map<int, std::shared_ptr<PdActor>> Actors;
	RigidBodyInfo rigidbody;
	rigidbody.angularVelocity = Vec3f(1, 0, 0);
	for (auto it : joint_Id)
	{
		RigidBodyInfo rB;
		rB.position = rA.position + Vec3f(0.0, 0.12f, 0.0);
		rB.linearVelocity = Vec3f(0, 0, 0);
		
		auto newBoxActor = rigid->addBox(box, rB);
		auto& ballAndSocketJoint = rigid->createBallAndSocketJoint(oldBoxActor, newBoxActor);
		ballAndSocketJoint.setAnchorPoint((rA.position + rB.position) / 2);

		rA = rB;
		oldBoxActor = newBoxActor;
	}

	auto mapper = std::make_shared<DiscreteElementsToTriangleSet<DataType3f>>();
	JointBody->stateTopology()->connect(mapper->inDiscreteElements());
	JointBody->graphicsPipeline()->pushModule(mapper);

	auto sRender = std::make_shared<GLSurfaceVisualModule>();
	sRender->setColor(Color(0.3f, 0.5f, 0.9f));
	sRender->setAlpha(0.8f);
	sRender->setRoughness(0.7f);
	sRender->setMetallic(3.0f);
	mapper->outTriangleSet()->connect(sRender->inTriangleSet());
	JointBody->graphicsPipeline()->pushModule(sRender);
	return scn;
}

int main()
{
	QtApp app;
	app.setSceneGraph(creatBricks());
	app.initialize(1280, 768);
	app.mainLoop();

	return 0;
}


