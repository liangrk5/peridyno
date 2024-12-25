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
	jeep->inTextureMesh()->connect(prRender->inTextureMesh());
	jeep->stateInstanceTransform()->connect(prRender->inTransform());
	jeep->graphicsPipeline()->pushModule(prRender);

	uint N = 1;

	for (uint i = 0; i < N; i++)
	{
		Vec3f tr = i * Vec3f(3.0f, 0.0f, 0.0f);

		BoxInfo box1, box2, box3, box4, box5, box6;
		//car body
		//box1.center = Vec3f(0, 1.171, -0.011) + tr;
		box1.halfLength = Vec3f(1.011, 0.793, 2.4);

		//box2.center = Vec3f(0, 1.044, -2.254) + tr;
		box2.halfLength = Vec3f(0.447, 0.447, 0.15);

		box3.center = Vec3f(0.812, 0.450, 1.722) + tr;
		box3.halfLength = Vec3f(0.2f);
		box4.center = Vec3f(-0.812, 0.450, 1.722) + tr;
		box4.halfLength = Vec3f(0.2f);
		/*CapsuleInfo capsule1, capsule2, capsule3, capsule4;

		capsule1.center = Vec3f(0.812, 0.450, 1.722) + tr;
		capsule1.rot = Quat1f(M_PI / 2, Vec3f(0, 0, 1));
		capsule1.halfLength = 0.1495;
		capsule1.radius = 0.450;
		capsule2.center = Vec3f(-0.812, 0.450, 1.722) + tr;
		capsule2.rot = Quat1f(M_PI / 2, Vec3f(0, 0, 1));
		capsule2.halfLength = 0.1495;
		capsule2.radius = 0.450;
		capsule3.center = Vec3f(-0.812, 0.450, -1.426) + tr;
		capsule3.rot = Quat1f(M_PI / 2, Vec3f(0, 0, 1));
		capsule3.halfLength = 0.1495;
		capsule3.radius = 0.450;
		capsule4.center = Vec3f(0.812, 0.450, -1.426) + tr;
		capsule4.rot = Quat1f(M_PI / 2, Vec3f(0, 0, 1));
		capsule4.halfLength = 0.1495;
		capsule4.radius = 0.450;*/
		SphereInfo sphere1, sphere2, sphere3, sphere4;
		//sphere1.center = Vec3f(0.812, 0.450, 1.722) + tr;
		sphere1.radius = 0.450;
		//sphere2.center = Vec3f(-0.812, 0.450, 1.722) + tr;
		sphere2.radius = 0.450;
		//sphere3.center = Vec3f(-0.812, 0.450, -1.426) + tr;
		sphere3.radius = 0.450;
		//sphere4.center = Vec3f(0.812, 0.450, -1.426) + tr;
		sphere4.radius = 0.450;

		//box5.center = Vec3f(0, 1.171, 1.722) + tr;
		box5.halfLength = Vec3f(0.8, 0.1, 0.1);

		RigidBodyInfo rigidbody;

		rigidbody.bodyId = i;

		rigidbody.position = Vec3f(0, 1.171, -0.011) + tr;
		auto bodyActor = jeep->addBox(box1, rigidbody, 100);

		box2.center = Vec3f(0.0f);
		rigidbody.position = Vec3f(0, 1.044, -2.254) + tr;
		auto spareTireActor = jeep->addBox(box2, rigidbody);

		Real wheel_velocity = 20;

		/*auto frontLeftTireActor = jeep->addCapsule(capsule1, rigidbody, 100);
		auto frontRightTireActor = jeep->addCapsule(capsule2, rigidbody, 100);
		auto rearLeftTireActor = jeep->addCapsule(capsule3, rigidbody, 100);
		auto rearRightTireActor = jeep->addCapsule(capsule4, rigidbody, 100);*/

		rigidbody.position = Vec3f(0.812, 0.450, 1.722) + tr;
		auto frontLeftTireActor = jeep->addSphere(sphere1, rigidbody, 50);

		rigidbody.position = Vec3f(-0.812, 0.450, 1.722) + tr;
		auto frontRightTireActor = jeep->addSphere(sphere2, rigidbody, 50);

		rigidbody.position = Vec3f(-0.812, 0.450, -1.426) + tr;
		auto rearLeftTireActor = jeep->addSphere(sphere3, rigidbody, 50);

		rigidbody.position = Vec3f(0.812, 0.450, -1.426) + tr;
		auto rearRightTireActor = jeep->addSphere(sphere4, rigidbody, 50);

		rigidbody.position = Vec3f(0, 1.171, 1.722) + tr;
		auto frontActor = jeep->addBox(box5, rigidbody, 25000);
		//auto rearActor = jeep->addBox(box6, rigidbody, 25000);

		//front rear
		auto& joint1 = jeep->createHingeJoint(frontLeftTireActor, frontActor);
		joint1.setAnchorPoint(frontLeftTireActor->center);
		//joint1.setMoter(wheel_velocity);
		joint1.setAxis(Vec3f(1, 0, 0));

		auto& joint2 = jeep->createHingeJoint(frontRightTireActor, frontActor);
		joint2.setAnchorPoint(frontRightTireActor->center);
		//joint2.setMoter(wheel_velocity);
		joint2.setAxis(Vec3f(1, 0, 0));

		//back rear
		auto& joint3 = jeep->createHingeJoint(rearLeftTireActor, bodyActor);
		joint3.setAnchorPoint(rearLeftTireActor->center);
		joint3.setMoter(wheel_velocity);
		joint3.setAxis(Vec3f(1, 0, 0));

		auto& joint4 = jeep->createHingeJoint(rearRightTireActor, bodyActor);
		joint4.setAnchorPoint(rearRightTireActor->center);
		joint4.setMoter(wheel_velocity);
		joint4.setAxis(Vec3f(1, 0, 0));


		auto& joint5 = jeep->createFixedJoint(bodyActor, spareTireActor);
		joint5.setAnchorPoint((bodyActor->center + spareTireActor->center) / 2);


		auto& joint6 = jeep->createHingeJoint(bodyActor, frontActor);
		joint6.setAnchorPoint(frontActor->center);
		joint6.setAxis(Vec3f(0, 1, 0));
		joint6.setRange(M_PI / 24, M_PI / 24);


		jeep->bind(bodyActor, Pair<uint, uint>(5, i));
		jeep->bind(spareTireActor, Pair<uint, uint>(4, i));
		jeep->bind(frontLeftTireActor, Pair<uint, uint>(0, i));
		jeep->bind(frontRightTireActor, Pair<uint, uint>(1, i));
		jeep->bind(rearLeftTireActor, Pair<uint, uint>(2, i));
		jeep->bind(rearRightTireActor, Pair<uint, uint>(3, i));
	}

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


