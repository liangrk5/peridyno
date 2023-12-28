/**
 * Program:   Parent class for all field widgets
 * Module:    QFieldWidget.h
 *
 * Copyright 2023 Xiaowei He
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
//Qt
#include <QGroupBox>
#include <QPushButton>
#include <QSpinBox>
#include <QDialog>
#include <QLineEdit>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QLabel>
#include <QVBoxLayout>

//PeriDyno
#include "Format.h"
#include "FCallBackFunc.h"
#include "QtGUI/Common.h"

//C++
#include <memory>

namespace dyno
{
	class Node;
	class Module;
	class FBase;

	class QDoubleSpinner;
	class QDoubleSlider;
	class mDoubleSpinBox;
	class ValueDialog;
	class toggleLabel;


	class PERIDYNO_QTGUI_API QFieldWidget : public QGroupBox
	{
		Q_OBJECT
	public:
		QFieldWidget(FBase* field);
		virtual ~QFieldWidget();

		inline FBase* field() { return mField; }

	signals:
		void fieldChanged();

	public slots:

	private:
		FBase* mField = nullptr;

	private:
		void syncValueFromField();

		std::shared_ptr<FCallBackFunc> callback = nullptr;
	};




}
