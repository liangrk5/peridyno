#include "initializeRigidBody.h"

#include "NodeFactory.h"
#include "Vehicle.h"

namespace dyno
{
	std::atomic<RigidBodyInitializer*> RigidBodyInitializer::gInstance;
	std::mutex RigidBodyInitializer::gMutex;

	PluginEntry* RigidBodyInitializer::instance()
	{
		RigidBodyInitializer* ins = gInstance.load(std::memory_order_acquire);
		if (!ins) {
			std::lock_guard<std::mutex> tLock(gMutex);
			ins = gInstance.load(std::memory_order_relaxed);
			if (!ins) {
				ins = new RigidBodyInitializer();
				ins->setName("Rigid Body");
				ins->setVersion("1.0");
				ins->setDescription("A rigid body library");

				gInstance.store(ins, std::memory_order_release);
			}
		}

		return ins;
	}

	RigidBodyInitializer::RigidBodyInitializer()
		: PluginEntry()
	{
	}

	void RigidBodyInitializer::initializeActions()
	{
		NodeFactory* factory = NodeFactory::instance();

		auto page = factory->addPage(
			"Rigid Body",
			"ToolBarIco/RigidBody/RigidBody.png");

		auto group = page->addGroup("Rigid Body");

		group->addAction(
			"Rigid Body",
			"ToolBarIco/RigidBody/RigidBody.png",
			nullptr);

		auto articulated = page->addGroup("ArticulatedBody");

		articulated->addAction(
			"Jeep",
			"ToolBarIco/RigidBody/Jeep_45.png",
			[=]()->std::shared_ptr<Node> {
				return std::make_shared<Jeep<DataType3f>>();
			}
		);

		articulated->addAction(
			"Tank",
			"ToolBarIco/RigidBody/Tank_45.png",
			[=]()->std::shared_ptr<Node> {
				return std::make_shared<Tank<DataType3f>>();
			}
		);

		articulated->addAction(
			"UAV",
			"ToolBarIco/RigidBody/UAV_15.png",
			[=]()->std::shared_ptr<Node> {
				return std::make_shared<UAV<DataType3f>>();
			}
		);

		articulated->addAction(
			"UUV",
			"ToolBarIco/RigidBody/UUV_45.png",
			[=]()->std::shared_ptr<Node> {
				return std::make_shared<UUV<DataType3f>>();
			}
		);

	}
}

PERIDYNO_API dyno::PluginEntry* RigidBody::initDynoPlugin()
{
	if (dyno::RigidBodyInitializer::instance()->initialize())
		return dyno::RigidBodyInitializer::instance();

	return nullptr;
}

dyno::PluginEntry* RigidBody::initStaticPlugin()
{
	if (dyno::RigidBodyInitializer::instance()->initialize())
		return dyno::RigidBodyInitializer::instance();

	return nullptr;
}

