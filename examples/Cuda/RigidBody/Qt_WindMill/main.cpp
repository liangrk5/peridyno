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
	gltf->varFileName()->setValue(getAssetPath() + "Windmill/Windmill.gltf");
	gltf->setVisible(false);
	gltf->stateTextureMesh()->connect(JointBody->inTextureMesh());

	JointBody->inTextureMesh()->connect(prRender->inTextureMesh());
	JointBody->stateInstanceTransform()->connect(prRender->inTransform());
	JointBody->graphicsPipeline()->pushModule(prRender);
	//JointBody->varGravityEnabled()->setValue(false);
	auto texMesh = JointBody->inTextureMesh()->constDataPtr();
	std::map<int, std::shared_ptr<PdActor>> Actors;
	RigidBodyInfo rigidbody;
	for (int it = 0;it < 518;it++)
	{
		auto up = texMesh->shapes()[it]->boundingBox.v1;
		auto down = texMesh->shapes()[it]->boundingBox.v0;
		BoxInfo box;

		box.center = texMesh->shapes()[it]->boundingTransform.translation();
		Vec3f tmp = (texMesh->shapes()[it]->boundingBox.v1 - texMesh->shapes()[it]->boundingBox.v0) / 2;
		box.halfLength = Vec3f(abs(tmp.x), abs(tmp.y), abs(tmp.z));
		if (it == 5)
		{
			box.halfLength = box.halfLength * 0.3;
		}
		if (it != 4)
		{
			if(it < 6)
				Actors[it] = JointBody->addBox(box, rigidbody, 1000);
			else
				Actors[it] = JointBody->addBox(box, rigidbody, 50);
		}
		else
			Actors[it] = JointBody->addBox(box, rigidbody, 1000000);

		JointBody->bind(Actors[it], Pair<uint, uint>(it, 0));
	}
	
	for (int i = 0; i < 4; i++)
	{
		auto& joint = JointBody->createFixedJoint(Actors[i], Actors[5]);
		joint.setAnchorPoint((Actors[i]->center + Actors[5]->center) / 2);
	}
	
	auto& joint = JointBody->createHingeJoint(Actors[4], Actors[5]);
	joint.setAnchorPoint(Actors[5]->center);
	joint.setAxis(Vec3f(0, 0, 1));
	joint.setMoter(2);


	auto& joint2 = JointBody->createUnilateralFixedJoint(Actors[4]);
	joint2.setAnchorPoint(Actors[4]->center);
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
	app.renderWindow()->getCamera()->setEyePos(Vec3f(0.98f, 1.37f, 3.18));

	//Set the target position for the camera
	app.renderWindow()->getCamera()->setTargetPos(Vec3f(0, 0, 0));

	//Set the distance unit for the camera, the fault unit is meter
	app.renderWindow()->getCamera()->setUnitScale(7.152f);
	app.mainLoop();
	return 0;
}


