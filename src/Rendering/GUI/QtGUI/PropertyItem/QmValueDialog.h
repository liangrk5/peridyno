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
	class ValueButton;

	class ValueDialog : public QDialog
	{
		Q_OBJECT

	public:

		ValueDialog(QAbstractSpinBox* parent = nullptr);

		void mouseMoveEvent(QMouseEvent* event) override;
		void mouseReleaseEvent(QMouseEvent* event) override;

		void keyPressEvent(QKeyEvent* event) override;
		void keyReleaseEvent(QKeyEvent* event) override;

		void updateDialogPosition();

		ValueButton* button[5];

		QAbstractSpinBox* SBox1;
		mDoubleSpinBox* mDSpinBox;
		QSpinBox* mISpinBox;

	Q_SIGNALS:
		void DiaValueChange(double);

		void DiaValueChange(QString);

	public slots:
		//void ModifyValue(double);

		void initData(double);

	private:

	};


	class ValueButton : public QPushButton
	{
		Q_OBJECT

	public:
		explicit ValueButton(QWidget* parent = nullptr);

		void mouseMoveEvent(QMouseEvent* event) override;

		void mousePressEvent(QMouseEvent* event) override;

		void mouseReleaseEvent(QMouseEvent* event) override;

		void setRealText(QString t) { text = t; }

		double SpinBoxData = 0;

		double Data1 = 0;


		double defaultValue = 0;
		double finalValue = 0;
		int StartX = 0;
		int EndX = 0;
		QDialog* parentDialog;
		QAbstractSpinBox* DSB1;

		bool shiftPress = 0;
		
	Q_SIGNALS:
		void ValueChange(double);
	Q_SIGNALS:
		void Release(double);

	private:
		double sub = 0;
		int temp = 0;
		std::string str;
		QString text;
		bool displayRealValue = true;
	};


}
