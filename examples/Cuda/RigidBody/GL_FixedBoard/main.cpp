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
#include <set>


using namespace std;
using namespace dyno;

std::shared_ptr<SceneGraph> creatBricks()
{
	std::shared_ptr<SceneGraph> scn = std::make_shared<SceneGraph>();

	auto rigid = scn->addNode(std::make_shared<RigidBodySystem<DataType3f>>());

	BoxInfo newBox, oldBox;
	SphereInfo sphere;
	RigidBodyInfo rigidBody;

	sphere.center = Vec3f(0.7, 0.1, 0.7);
	sphere.radius = 0.1;

	auto sphereActor = rigid->addSphere(sphere, rigidBody, 100);

    vector<vector<vector<std::shared_ptr<PdActor>>>> list;
    for (int i = 0; i < 10; i++)
    {
        vector<vector<std::shared_ptr<PdActor>>> res;
        for (int j = 0; j < 10; j++)
        {
            vector<std::shared_ptr<PdActor>> resInner;
            for (int k = 0; k < 10; k++)
            {
                newBox.center = Vec3f(0.11 * i, 1.5 + 0.11 * j, 0.11 * k);
                newBox.halfLength = Vec3f(0.05, 0.05, 0.05);
                auto newBoxActor = rigid->addBox(newBox, rigidBody);
                resInner.emplace_back(newBoxActor);
            }
            res.emplace_back(resInner);
        }
        list.emplace_back(res);
    }

    Real maxForce = 200;
    Real maxTorque = 200;

    std::set<std::pair<std::shared_ptr<PdActor>, std::shared_ptr<PdActor>>> createdJoints;

    for (int i = 0; i < list.size(); i++)
    {
        for (int j = 0; j < list[0].size(); j++)
        {
            for (int k = 0; k < list[0][0].size(); k++)
            {
                auto currentBox = list[i][j][k];

                // Check neighboring boxes (up, down, left, right, front, back)
                if (i > 0) // Up
                {
                    auto nearBox = list[i - 1][j][k];
                    if (createdJoints.find({ nearBox, currentBox }) == createdJoints.end() && createdJoints.find({ currentBox, nearBox }) == createdJoints.end())
                    {
                        auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                        joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                        joint.setMaxForceAndTorque(maxForce, maxTorque);
                        createdJoints.insert({ currentBox, nearBox });
                    }
                }
                if (i < list.size() - 1) // Down
                {
                    auto nearBox = list[i + 1][j][k];
                    if (createdJoints.find({ nearBox, currentBox }) == createdJoints.end() && createdJoints.find({ currentBox, nearBox }) == createdJoints.end())
                    {
                        auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                        joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                        joint.setMaxForceAndTorque(maxForce, maxTorque);
                        createdJoints.insert({ currentBox, nearBox });
                    }
                }
                if (j > 0) // Left
                {
                    auto nearBox = list[i][j - 1][k];
                    if (createdJoints.find({ nearBox, currentBox }) == createdJoints.end() && createdJoints.find({ currentBox, nearBox }) == createdJoints.end())
                    {
                        auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                        joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                        joint.setMaxForceAndTorque(maxForce, maxTorque);
                        createdJoints.insert({ currentBox, nearBox });
                    }
                }
                if (j < list[i].size() - 1) // Right
                {
                    auto nearBox = list[i][j + 1][k];
                    if (createdJoints.find({ nearBox, currentBox }) == createdJoints.end() && createdJoints.find({ currentBox, nearBox }) == createdJoints.end())
                    {
                        auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                        joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                        joint.setMaxForceAndTorque(maxForce, maxTorque);
                        createdJoints.insert({ currentBox, nearBox });
                    }
                }
                if (k > 0) // Front
                {
                    auto nearBox = list[i][j][k - 1];
                    if (createdJoints.find({ nearBox, currentBox }) == createdJoints.end() && createdJoints.find({ currentBox, nearBox }) == createdJoints.end())
                    {
                        auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                        joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                        joint.setMaxForceAndTorque(maxForce, maxTorque);
                        createdJoints.insert({ currentBox, nearBox });
                    }
                }
                if (k < list[i][j].size() - 1) // Back
                {
                    auto nearBox = list[i][j][k + 1];
                    if (createdJoints.find({ nearBox, currentBox }) == createdJoints.end() && createdJoints.find({ currentBox, nearBox }) == createdJoints.end())
                    {
                        auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                        joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                        joint.setMaxForceAndTorque(maxForce, maxTorque);
                        createdJoints.insert({ currentBox, nearBox });
                    }
                }
            }
        }
    }

    sphere.center = Vec3f(2.55, 0.1, 2.55);
    sphere.radius = 0.1;

    auto sphereActor1 = rigid->addSphere(sphere, rigidBody);

    vector<vector<std::shared_ptr<PdActor>>> list1;
    for (int i = 0; i < 10; i++)
    {
        vector<std::shared_ptr<PdActor>> res;
        for (int j = 0; j < 10; j++)
        {
            newBox.center = Vec3f(2.0 + 0.11 * i, 1.5, 2.0 + 0.11 * j);
            newBox.halfLength = Vec3f(0.05, 0.02, 0.05);
            auto newBoxActor = rigid->addBox(newBox, rigidBody);
            res.emplace_back(newBoxActor);
        }
        list1.emplace_back(res);
    }


    for (int i = 0; i < list1.size(); i++)
    {
        for (int j = 0; j < list1[0].size(); j++)
        {
            auto currentBox = list1[i][j];

            // 仅连接下方和右方的相邻 box

            if (i < list1.size() - 1) // 下
            {
                auto nearBox = list1[i + 1][j];
                auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                joint.setMaxForceAndTorque(maxForce, maxTorque);
            }
            if (j < list1[i].size() - 1) // 右
            {
                auto nearBox = list1[i][j + 1];
                auto& joint = rigid->createFixedJoint(currentBox, nearBox);
                joint.setAnchorPoint((currentBox->center + nearBox->center) / 2);
                joint.setMaxForceAndTorque(maxForce, maxTorque);
            }
        }
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
	app.setSceneGraph(creatBricks());
	app.initialize(1280, 768);
	app.mainLoop();

	return 0;
}


