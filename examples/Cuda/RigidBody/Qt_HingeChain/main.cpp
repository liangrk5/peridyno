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


	auto prRender = std::make_shared<GLPhotorealisticInstanceRender>();
	auto JointBody = scn->addNode(std::make_shared<Vechicle<DataType3f>>());

	auto gltf = scn->addNode(std::make_shared<GltfLoader<DataType3f>>());
	gltf->varFileName()->setValue(getAssetPath() + "joint/Joint_gltf_Tex/Chain.gltf");
	gltf->setVisible(false);
	gltf->stateTextureMesh()->connect(JointBody->inTextureMesh());
	gltf->varScale()->setValue(Vec3f(50));

	JointBody->inTextureMesh()->connect(prRender->inTextureMesh());
	JointBody->stateInstanceTransform()->connect(prRender->inTransform());
	JointBody->graphicsPipeline()->pushModule(prRender);
	//JointBody->varGravityEnabled()->setValue(false);
	auto texMesh = JointBody->inTextureMesh()->constDataPtr();
	std::map<int, std::shared_ptr<PdActor>> Actors;
	RigidBodyInfo rigidbody;
	for (int it = 0;it < 50;it++)
	{
		auto up = texMesh->shapes()[it]->boundingBox.v1;
		auto down = texMesh->shapes()[it]->boundingBox.v0;
		BoxInfo box;

		box.center = texMesh->shapes()[it]->boundingTransform.translation() * 50;
		Vec3f tmp = (texMesh->shapes()[it]->boundingBox.v1 - texMesh->shapes()[it]->boundingBox.v0) / 2;
		box.halfLength = Vec3f(abs(tmp.x), abs(tmp.y), abs(tmp.z)) * 50;
		Actors[it] = JointBody->addBox(box, rigidbody, 1000);

		JointBody->bind(Actors[it], Pair<uint, uint>(it, 0));
	}

	auto up = texMesh->shapes()[50]->boundingBox.v1;
	auto down = texMesh->shapes()[50]->boundingBox.v0;
	SphereInfo sphere;

	sphere.center = texMesh->shapes()[50]->boundingTransform.translation() * 50;
	Vec3f tmp = (texMesh->shapes()[50]->boundingBox.v1 - texMesh->shapes()[50]->boundingBox.v0) / 2 * 50;
	sphere.radius = tmp.x;
	Actors[50] = JointBody->addSphere(sphere, rigidbody, 660);

	JointBody->bind(Actors[50], Pair<uint, uint>(50, 0));
	
	for (int i = 0; i < 50; i++)
	{
		auto& joint = JointBody->createHingeJoint(Actors[i], Actors[i + 1]);
		joint.setAnchorPoint((Actors[i]->center + Actors[i + 1]->center) / 2);
		joint.setAxis(Vec3f(0, 0, 1));
		joint.setRange(-M_PI / 2, M_PI / 2);
	}

	auto& joint = JointBody->createPointJoint(Actors[0]);
	joint.setAnchorPoint(Actors[0]->center);

	
	/*auto mapper = std::make_shared<DiscreteElementsToTriangleSet<DataType3f>>();
	JointBody->stateTopology()->connect(mapper->inDiscreteElements());
	JointBody->graphicsPipeline()->pushModule(mapper);

	auto sRender = std::make_shared<GLSurfaceVisualModule>();
	sRender->setColor(Color(0.3f, 0.5f, 0.9f));
	sRender->setAlpha(0.8f);
	sRender->setRoughness(0.7f);
	sRender->setMetallic(3.0f);
	mapper->outTriangleSet()->connect(sRender->inTriangleSet());
	JointBody->graphicsPipeline()->pushModule(sRender);*/
	return scn;
}

int main()
{
	QtApp app;
	app.setSceneGraph(creatBricks());
	app.initialize(1280, 768);
	//Set the eye position for the camera
	app.renderWindow()->getCamera()->setEyePos(Vec3f(1.38f, 0.10f, 6.83));

	//Set the target position for the camera
	app.renderWindow()->getCamera()->setTargetPos(Vec3f(0, 0, 0));

	//Set the distance unit for the camera, the fault unit is meter
	app.renderWindow()->getCamera()->setUnitScale(44.648f);
	app.mainLoop();
	return 0;
}


