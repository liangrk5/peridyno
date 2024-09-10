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

#include <PlaneModel.h>
#include <map>

#include "GltfLoader.h"
using namespace std;
using namespace dyno;

std::shared_ptr<SceneGraph> creatBricks()
{
	std::shared_ptr<SceneGraph> scn = std::make_shared<SceneGraph>();


	auto prRender = std::make_shared<GLPhotorealisticInstanceRender>();
	auto JointBody = scn->addNode(std::make_shared<Vechicle<DataType3f>>());

	auto gltf = scn->addNode(std::make_shared<GltfLoader<DataType3f>>());
	gltf->varFileName()->setValue(getAssetPath() + "joint/Joint_gltf_Tex/Spherical.gltf");
	gltf->setVisible(false);
	gltf->stateTextureMesh()->connect(JointBody->inTextureMesh());

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
		auto up = texMesh->shapes()[it]->boundingBox.v1;
		auto down = texMesh->shapes()[it]->boundingBox.v0;
		BoxInfo box;
		if (it == 1)
			rigidbody.angularVelocity = Vec3f(0);
			
		box.center = texMesh->shapes()[it]->boundingTransform.translation();
		Vec3f tmp = (texMesh->shapes()[it]->boundingBox.v1 - texMesh->shapes()[it]->boundingBox.v0) / 2;
		box.halfLength = Vec3f(abs(tmp.x), abs(tmp.y), abs(tmp.z));
		if(it == 0)
			Actors[it] = JointBody->addBox(box, rigidbody, 100);
		else
			Actors[it] = JointBody->addBox(box, rigidbody, 100);
		JointBody->bind(Actors[it], Pair<uint, uint>(it, 0));
	}

	auto& joint = JointBody->createBallAndSocketJoint(Actors[0], Actors[1]);
	joint.setAnchorPoint(Vec3f(0, 0, 0));

	auto& joint2 =JointBody->createUnilateralFixedJoint(Actors[1]);
	joint2.setAnchorPoint(Actors[1]->center);
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


