#include "QtModuleFlowScene.h"

#include "Object.h"
#include "SceneGraph.h"
#include "QtModuleWidget.h"
#include "SceneGraph.h"

#include "nodes/QNode"

#include "DirectedAcyclicGraph.h"
#include "AutoLayoutDAG.h"
#include "ModulePort.h"

#include <QtWidgets/QMenu>
#include <QLineEdit>
#include <QtWidgets>

namespace dyno
{
	class States : public Module
	{
	public:
		States() {};

		bool allowExported() override { return false; }
		bool allowImported() override { return false; }

		std::string caption() { return "States"; }
	};

	class Outputs : public Module
	{
	public:
		Outputs() {};

		bool allowExported() override { return false; }
		bool allowImported() override { return false; }

		std::string caption() { return "Outputs"; }
	};
}

namespace Qt
{
	QPointF SimStatePos = QPointF(0.0f, 0.0f);
	QPointF RenStatePos = QPointF(0.0f, 0.0f);

	QtModuleFlowScene::QtModuleFlowScene(std::shared_ptr<QtDataModelRegistry> registry,
		QObject* parent)
		: QtFlowScene(registry, parent)
	{
		connect(this, &QtFlowScene::nodeMoved, this, &QtModuleFlowScene::moveModule);
	}

	QtModuleFlowScene::QtModuleFlowScene(QObject* parent, QtNodeWidget* widget)
		: QtFlowScene(parent)
	{
		mNode = widget->getNode();

		auto classMap = dyno::Object::getClassMap();
		auto ret = std::make_shared<QtDataModelRegistry>();
		int id = 0;
		for (auto const c : *classMap)
		{
			id++;

			QString str = QString::fromStdString(c.first);
			dyno::Object* obj = dyno::Object::createObject(str.toStdString());
			std::shared_ptr<dyno::Module> module(dynamic_cast<dyno::Module*>(obj));

			if (module != nullptr)
			{
				QtDataModelRegistry::RegistryItemCreator creator = [str]() {
					auto new_obj = dyno::Object::createObject(str.toStdString());
					std::shared_ptr<dyno::Module> new_module(dynamic_cast<dyno::Module*>(new_obj));
					auto dat = std::make_unique<QtModuleWidget>(new_module);
					return dat; 
				};

				QString category = QString::fromStdString(module->getModuleType());
				ret->registerModel<QtModuleWidget>(category, creator);
			}
		}

		this->setRegistry(ret);

		if (mNode != nullptr)
			showModuleFlow(mNode.get());

		enableEditing();

		reorderAllModules();

		connect(this, &QtFlowScene::nodeMoved, this, &QtModuleFlowScene::moveModule);
		connect(this, &QtFlowScene::nodePlaced, this, &QtModuleFlowScene::addModule);
		connect(this, &QtFlowScene::nodeDeleted, this, &QtModuleFlowScene::deleteModule);

		connect(this, &QtFlowScene::outPortConextMenu, this, &QtModuleFlowScene::promoteOutput);
	}

	QtModuleFlowScene::~QtModuleFlowScene()
	{
		//To avoid editing the the module flow inside the node when the widget is closed.
		disableEditing();
	}

	void QtModuleFlowScene::enableEditing()
	{
		mEditingEnabled = true;

		auto allNodes = this->allNodes();

		for  (auto node : allNodes)
		{
			auto model = dynamic_cast<QtModuleWidget*>(node->nodeDataModel());
			if (model != nullptr)
			{
				model->enableEditing();
			}
		}
	}

	void QtModuleFlowScene::disableEditing()
	{
		mEditingEnabled = false;

		auto allNodes = this->allNodes();

		for  (auto node : allNodes)
		{
			auto model = dynamic_cast<QtModuleWidget*>(node->nodeDataModel());
			if (model != nullptr)
			{
				model->disableEditing();
			}
		}
	}

	void QtModuleFlowScene::showModuleFlow(Node* node)
	{
		clearScene();

		if (node == nullptr)
			return;

		auto& mlist = node->getModuleList();

		std::map<dyno::ObjectId, QtNode*> moduleMap;

		//To show the animation pipeline in default
		if (mActivePipeline == nullptr)
			mActivePipeline = node->animationPipeline();

		auto& modules = mActivePipeline->allModules();

		auto addModuleWidget = [&](std::shared_ptr<Module> m) -> void
		{
			auto mId = m->objectId();

			auto type = std::make_unique<QtModuleWidget>(m);

			auto& node = this->createNode(std::move(type));

			moduleMap[mId] = &node;

			QPointF posView(m->bx(), m->by());

			node.nodeGraphicsObject().setPos(posView);

			this->nodePlaced(node);
		};

		//Create a dummy module to store all state variables
		mStates = std::make_shared<dyno::States>();

		auto& fields = node->getAllFields();
		for (auto field : fields)
		{
			if ((field->getFieldType() == dyno::FieldTypeEnum::State
				|| field->getFieldType() == dyno::FieldTypeEnum::In) && field->inputPolicy() == FBase::One)
			{
				mStates->addOutputField(field);
			}
		}


		QPointF pos = mActivePipeline == node->animationPipeline() ? SimStatePos : RenStatePos;
		mStates->setBlockCoord(pos.x(), pos.y());

		addModuleWidget(mStates);

		for  (auto m : modules)
		{
			addModuleWidget(m.second);
		}

		auto createModuleConnections = [&](std::shared_ptr<Module> m) -> void
		{
			auto inId = m->objectId();

			if (moduleMap.find(inId) != moduleMap.end()) {
				auto inBlock = moduleMap[m->objectId()];

				auto imports = m->getImportModules();

				if (m->allowImported())
				{
					for (int i = 0; i < imports.size(); i++)
					{
						dyno::ModulePortType pType = imports[i]->getPortType();
						if (dyno::Single == pType)
						{
							auto moduleSrc = imports[i]->getModules()[0];
							if (moduleSrc != nullptr)
							{
								auto outId = moduleSrc->objectId();
								if (moduleMap.find(outId) != moduleMap.end())
								{
									auto outBlock = moduleMap[moduleSrc->objectId()];
									createConnection(*inBlock, i, *outBlock, 0);
								}
							}
						}
						else if (dyno::Multiple == pType)
						{
							//TODO: a weird problem exist here, if the expression "auto& nodes = ports[i]->getNodes()" is used,
							//we still have to call clear to avoid memory leak.
							auto& modules = imports[i]->getModules();
							//ports[i]->clear();
							for (int j = 0; j < modules.size(); j++)
							{
								if (modules[j] != nullptr)
								{
									auto outId = modules[j]->objectId();
									if (moduleMap.find(outId) != moduleMap.end())
									{
										auto outBlock = moduleMap[outId];
										createConnection(*inBlock, i, *outBlock, 0);
									}
								}
							}
							//nodes.clear();
						}
					}
				}


				auto fieldIn = m->getInputFields();

				for (int i = 0; i < fieldIn.size(); i++)
				{
					auto fieldSrc = fieldIn[i]->getSource();
					if (fieldSrc != nullptr) {
						auto parSrc = fieldSrc->parent();
						if (parSrc != nullptr)
						{
							Module* moduleSrc = dynamic_cast<Module*>(parSrc);
							if (moduleSrc == nullptr)
							{
								moduleSrc = mStates.get();
							}

							auto outId = moduleSrc->objectId();
							auto fieldsOut = moduleSrc->getOutputFields();

							uint outFieldIndex = 0;
							bool fieldFound = false;
							for (auto f : fieldsOut)
							{
								if (f == fieldSrc)
								{
									fieldFound = true;
									break;
								}
								outFieldIndex++;
							}

							if (moduleSrc->allowExported()) outFieldIndex++;

							uint inFieldIndex = m->allowImported() ? i + imports.size() : i;

							if (fieldFound && moduleMap.find(outId) != moduleMap.end())
							{
								auto outBlock = moduleMap[outId];
								createConnection(*inBlock, inFieldIndex, *outBlock, outFieldIndex);
							}
						}
					}
				}
			}
		};

		for  (auto m : modules)
		{
			createModuleConnections(m.second);
		}

		//Create a module for outputs
		mOutputs = std::make_shared<dyno::Outputs>();

		auto hasModuleConnected = [&](FBase* f) -> bool
			{
				bool fieldFound = false;
				auto fieldSrc = f->getSource();
				if (fieldSrc != nullptr) {
					auto parSrc = fieldSrc->parent();
					if (parSrc != nullptr)
					{
						Module* moduleSrc = dynamic_cast<Module*>(parSrc);
						if (moduleSrc == nullptr)
						{
							moduleSrc = mStates.get();
						}

						auto outId = moduleSrc->objectId();
						auto fieldsOut = moduleSrc->getOutputFields();

						if (moduleMap.find(outId) != moduleMap.end())
						{
							for (auto f : fieldsOut)
							{
								if (f == fieldSrc)
								{
									fieldFound = true;
									break;
								}
							}
						}
					}
				}
				return fieldFound;
			};

		auto& output_fields = node->getOutputFields();
		for (auto field : output_fields)
		{
			if (field->getFieldType() == dyno::FieldTypeEnum::Out && hasModuleConnected(field))
			{
				mOutputs->addInputField(field);
			}
		}

		if (mOutputs->getInputFields().size() > 0)
		{
			mOutputs->setBlockCoord(pos.x() + 500, pos.y());
			addModuleWidget(mOutputs);
			createModuleConnections(mOutputs);
		}
	}

	void QtModuleFlowScene::moveModule(QtNode& n, const QPointF& newLocation)
	{
		QtModuleWidget* mw = dynamic_cast<QtModuleWidget*>(n.nodeDataModel());

		auto m = mw == nullptr ? nullptr : mw->getModule();

		if (m != nullptr)
		{
			m->setBlockCoord(newLocation.x(), newLocation.y());
		}
	}

	void QtModuleFlowScene::showResetPipeline()
	{
		auto pipeline = mNode->resetPipeline();

		if (mNode == nullptr || mActivePipeline == pipeline)
			return;

		mActivePipeline = pipeline;

		updateModuleGraphView();

		if (mReorderResetPipeline) {
			reorderAllModules();
			mReorderResetPipeline = false;
		}
	}

	void QtModuleFlowScene::showAnimationPipeline()
	{
		auto pipeline = mNode->animationPipeline();

		if (mNode == nullptr || mActivePipeline == pipeline)
			return;

		mActivePipeline = mNode->animationPipeline();

		updateModuleGraphView();
	}

	void QtModuleFlowScene::showGraphicsPipeline()
	{
		auto pipeline = mNode->graphicsPipeline();

		if (mNode == nullptr || mActivePipeline == pipeline)
			return;

		mActivePipeline = mNode->graphicsPipeline();

		updateModuleGraphView();

		if (mReorderGraphicsPipeline) {
			reorderAllModules();
			mReorderGraphicsPipeline = false;
		}
	}

	void QtModuleFlowScene::reconstructActivePipeline()
	{
		mActivePipeline->forceUpdate();
	}

	void QtModuleFlowScene::promoteOutput(QtNode& n, const PortIndex index, const QPointF& pos)
	{
		//Index 0 is used for module connection
		if (index == 0)
			return;

		QMenu portMenu;

		portMenu.setObjectName("PortMenu");

		
		auto exportAction = portMenu.addAction("Export");

		connect(exportAction, &QAction::triggered, [&]()
			{
				auto dataModel = dynamic_cast<QtModuleWidget*>(n.nodeDataModel());

				if (dataModel != nullptr)
				{
					auto m = dataModel->getModule();
					if (m != nullptr)
					{
						auto fieldOut = m->getOutputFields();

						mActivePipeline->promoteOutputToNode(fieldOut[index - 1]);
					}

					emit nodeExportChanged();
				}
			});

		QPoint p(pos.x(), pos.y());

		portMenu.exec(p);
	}

	void QtModuleFlowScene::updateModuleGraphView()
	{
		disableEditing();

		clearScene();

		if (mNode != nullptr)
			showModuleFlow(mNode.get());

		enableEditing();
	}

	void QtModuleFlowScene::reorderAllModules()
	{
		if (mActivePipeline == nullptr)
			return;

		dyno::DirectedAcyclicGraph graph;

		auto constructDAG = [&](std::shared_ptr<Module> m) -> void
		{
			if (m->allowImported())
			{
				auto imports = m->getImportModules();
				for (int i = 0; i < imports.size(); i++)
				{
					auto inId = m->objectId();
					dyno::ModulePortType pType = imports[i]->getPortType();
					if (dyno::Single == pType)
					{
						auto module = imports[i]->getModules()[0];
						if (module != nullptr)
						{
							auto outId = module->objectId();

							graph.addEdge(outId, inId);
						}
					}
					else if (dyno::Multiple == pType)
					{
						auto& modules = imports[i]->getModules();
						for (int j = 0; j < modules.size(); j++)
						{
							if (modules[j] != nullptr)
							{
								auto outId = modules[j]->objectId();

								graph.addEdge(outId, inId);
							}
						}
						//nodes.clear();
					}
				}
			}

			auto outId = m->objectId();
			auto fieldOut = m->getOutputFields();
			for (int i = 0; i < fieldOut.size(); i++)
			{
				auto& sinks = fieldOut[i]->getSinks();
				for  (auto sink : sinks)
				{
					if (sink != nullptr) {
						auto parSrc = sink->parent();
						if (parSrc != nullptr)
						{
							Module* nodeSrc = dynamic_cast<Module*>(parSrc);

							if (nodeSrc != nullptr)
							{
								auto inId = nodeSrc->objectId();
								graph.addEdge(outId, inId);
							}
						}
					}
				}
			}
		};

		constructDAG(mStates);

		auto& mlists = mActivePipeline->activeModules();
		for (auto it = mlists.begin(); it != mlists.end(); it++)
		{
			constructDAG(*it);
		}

		dyno::AutoLayoutDAG layout(&graph);
		layout.update();

		//Set up the mapping from ObjectId to QtNode
		auto& _nodes = this->nodes();
		std::map<dyno::ObjectId, QtNode*> qtNodeMapper;
		std::map<dyno::ObjectId, Module*> moduleMapper;
		for (auto const& _node : _nodes)
		{
			auto const& qtNode = _node.second;
			auto model = qtNode->nodeDataModel();

			auto nodeData = dynamic_cast<QtModuleWidget*>(model);

			if (model != nullptr)
			{
				auto m = nodeData->getModule();
				if (m != nullptr)
				{
					qtNodeMapper[m->objectId()] = qtNode.get();
					moduleMapper[m->objectId()] = m.get();
				}
			}
		}

		float offsetX = 0.0f;
		for (size_t l = 0; l < layout.layerNumber(); l++)
		{
			auto& xc = layout.layer(l);

			float offsetY = 0.0f;
			float xMax = 0.0f;
			for (size_t index = 0; index < xc.size(); index++)
			{
				dyno::ObjectId id = xc[index];
				if (qtNodeMapper.find(id) != qtNodeMapper.end())
				{
					QtNode* qtNode = qtNodeMapper[id];
					NodeGeometry& geo = qtNode->nodeGeometry();

					float w = geo.width();
					float h = geo.height();

					xMax = std::max(xMax, w);

					Module* node = moduleMapper[id];

					node->setBlockCoord(offsetX, offsetY);

					offsetY += (h + mDy);
				}
			}

			offsetX += (xMax + mDx);
		}

		if (mActivePipeline == mNode->animationPipeline()) {
			SimStatePos = QPointF(mStates->bx(), mStates->by());
		}
		else {
			RenStatePos = QPointF(mStates->bx(), mStates->by());
		}
			

		qtNodeMapper.clear();
		moduleMapper.clear();

		updateModuleGraphView();
	}

	void QtModuleFlowScene::addModule(QtNode& n)
	{
		if (mActivePipeline == nullptr)
			return;

		auto nodeData = dynamic_cast<QtModuleWidget*>(n.nodeDataModel());

		if (mEditingEnabled && nodeData != nullptr) {
			mActivePipeline->pushModule(nodeData->getModule());
		}
	}

	void QtModuleFlowScene::deleteModule(QtNode& n)
	{
		if (mActivePipeline == nullptr)
			return;

		auto nodeData = dynamic_cast<QtModuleWidget*>(n.nodeDataModel());

		if (mEditingEnabled && nodeData != nullptr) {
			mActivePipeline->popModule(nodeData->getModule());

			emit this->nodeDeselected();
		}
	}
}
