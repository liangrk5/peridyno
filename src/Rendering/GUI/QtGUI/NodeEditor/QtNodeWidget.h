#pragma once

#include <QtCore/QObject>
#include <QtCore/QJsonObject>
#include <QtWidgets/QLabel>

#include "nodes/QNodeDataModel"

#include "Node.h"
#include "QtNodeData.h"
#include "QtFieldData.h"

#include <iostream>


using dyno::Node;

namespace Qt
{
	/// The model dictates the number of inputs and outputs for the Node.
	class QtNodeWidget : public QtNodeDataModel
	{
		Q_OBJECT

	public:
		QtNodeWidget(std::shared_ptr<Node> base = nullptr);

		virtual	~QtNodeWidget();

	public:

		QString caption() const override;

		QString name() const override;

		QString	nodeTips() const override;

		QString	portCaption(PortType portType, PortIndex portIndex) const override;

		QString	portTips(PortType portType, PortIndex portIndex) const override;

		QString	validationMessage() const override;

		unsigned int nPorts(PortType portType) const override;

		bool portCaptionVisible(PortType portType, PortIndex portIndex) const override;

		bool allowImported() const override { return false; }
		bool allowExported() const override { return true; }

		bool hotKeyRenderChecked() const override { return !mNode->isVisible(); }

		bool hotKeySimChecked() const override { return !mNode->isActive(); }

		bool hotKeyAutoSyncChecked() const override { return mNode->isAutoSync(); }


		/**
		 * @brief To test whether nodaData can be set as the input data for portIndex
		 * 
		 * @param portIndex Input index
		 * @param nodeData Input data
		 * @return true 
		 * @return false 
		 */
		bool tryInData(PortIndex portIndex, std::shared_ptr<QtNodeData> nodeData) override;

		void setInData(std::shared_ptr<QtNodeData> data, PortIndex portIndex) override;

		NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

		QWidget* embeddedWidget() override { return nullptr; }

		NodeValidationState validationState() const override;

		QtNodeDataModel::ConnectionPolicy portInConnectionPolicy(PortIndex portIndex) const override;

		std::shared_ptr<Node> getNode();

		std::shared_ptr<QtNodeData> outData(PortIndex port) override;

		std::vector<FBase*>& getOutputFields() const;
		std::vector<FBase*>& getInputFields() const;

		/**
		 * @brief When enabled, the scenegraph can be updated as long as the corresponding GUI is updated.
		 */
		void enableEditing();
		
		/**
		 * @brief When disabled, the scenegraph can not be affected by the corresponding GUI.
		 */
		void disableEditing();

	protected:
		virtual void updateModule();

	protected:
		using ExportNodePtr = std::shared_ptr<QtExportNode>;
		using ImportNodePtr = std::vector<std::shared_ptr<QtImportNode>>;

		ImportNodePtr mNodeInport;
		ExportNodePtr mNodeExport;

		using OutFieldPtr = std::vector<std::shared_ptr<QtFieldData>>;
		using InFieldPtr = std::vector<std::shared_ptr<QtFieldData>>;

		InFieldPtr mFieldInport;
		OutFieldPtr mFieldExport;

		std::shared_ptr<Node> mNode = nullptr;

		NodeValidationState modelValidationState = NodeValidationState::Valid;
		QString modelValidationError = QString("Missing or incorrect inputs");

	private:
		bool mEditingEnabled = true;
	};


}
