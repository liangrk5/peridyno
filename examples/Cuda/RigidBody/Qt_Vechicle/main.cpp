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

#include <BasicShapes/PlaneModel.h>
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
	gltf->varFileName()->setValue(getAssetPath() + "joint/SliderCrankLinkage/SliderCrankLinkage.gltf");
	gltf->setVisible(false);
	gltf->stateTextureMesh()->connect(JointBody->inTextureMesh());

	JointBody->inTextureMesh()->connect(prRender->inTextureMesh());
	JointBody->stateInstanceTransform()->connect(prRender->inTransform());
	JointBody->graphicsPipeline()->pushModule(prRender);
	JointBody->varGravityEnabled()->setValue(false);
	auto texMesh = JointBody->inTextureMesh()->constDataPtr();
	std::map<int, std::shared_ptr<PdActor>> Actors;
	RigidBodyInfo rigidbody;
	BoxInfo fixedBox;
	float tmp_length;
	for (int it=0;it <4;it++)
	{
		auto up = texMesh->shapes()[it]->boundingBox.v1;
		auto down = texMesh->shapes()[it]->boundingBox.v0;
		BoxInfo box;

		box.center = texMesh->shapes()[it]->boundingTransform.translation();
		Vec3f tmp = (texMesh->shapes()[it]->boundingBox.v1 - texMesh->shapes()[it]->boundingBox.v0) / 2;
		box.halfLength = Vec3f(abs(tmp.x), abs(tmp.y), abs(tmp.z));
		if(it != 3)
			Actors[it] = JointBody->addBox(box, rigidbody, 100);
		else
			Actors[it] = JointBody->addBox(box, rigidbody, 100000000);
		if (it == 0)
			tmp_length = tmp.x;
		JointBody->bind(Actors[it], Pair<uint, uint>(it, 0));
	}

	fixedBox.center = Actors[2]->center;
	fixedBox.halfLength = Vec3f(0.1, 0.1, 0.1);
	
	auto fixedActor = JointBody->addBox(fixedBox, rigidbody, 10000000);
	
	JointBody->createUnilateralFixedJointStable(fixedActor);

	JointBody->createUnilateralFixedJointStable(Actors[3]);

	auto& hingeJoint = JointBody->createHingeJoint(Actors[2], fixedActor);
	hingeJoint.setAnchorPoint(Actors[2]->center);
	hingeJoint.setAxis(Vec3f(0, 0, 1));
	hingeJoint.setMoter(3);

	auto& hingeJoint2 = JointBody->createHingeJoint(Actors[2], Actors[0]);
	hingeJoint2.setAnchorPoint(Actors[0]->center-Vec3f(tmp_length*0.85, 0, 0));
	hingeJoint2.setAxis(Vec3f(0, 0, 1));

	auto& hingeJoint3 = JointBody->createHingeJoint(Actors[0], Actors[1]);
	hingeJoint3.setAnchorPoint(Actors[0]->center + Vec3f(tmp_length * 0.85, 0, 0));
	hingeJoint3.setAxis(Vec3f(0, 0, 1));

	auto& sliderJoint = JointBody->createSliderJoint(Actors[1], Actors[3]);
	sliderJoint.setAnchorPoint(Actors[1]->center);
	sliderJoint.setAxis(Vec3f(1, 0, 0));


	
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
	app.renderWindow()->getCamera()->setEyePos(Vec3f(0.30f, 0.26f, 1.75));

	//Set the target position for the camera
	app.renderWindow()->getCamera()->setTargetPos(Vec3f(0, 0, 0));

	//Set the distance unit for the camera, the fault unit is meter
	app.renderWindow()->getCamera()->setUnitScale(3.326f);
	app.mainLoop();

	return 0;
}


