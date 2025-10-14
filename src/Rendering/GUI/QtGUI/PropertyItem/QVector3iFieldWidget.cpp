#include "QVector3iFieldWidget.h"

#include <QGridLayout>

#include "Field.h"
#include "QPiecewiseSpinBox.h"

namespace dyno
{
	IMPL_FIELD_WIDGET(Vec3i, QVector3iFieldWidget)

	QVector3iFieldWidget::QVector3iFieldWidget(FBase* field)
		: QFieldWidget(field)
	{
		QGridLayout* layout = new QGridLayout;
		layout->setContentsMargins(0, 0, 0, 0);
		layout->setSpacing(0);

		this->setLayout(layout);

		//Label
		QLabel* name = new QLabel();
		QString str = FormatFieldWidgetName(field->getObjectName());
		name->setFixedSize(100, 18);
		QFontMetrics fontMetrics(name->font());
		QString elide = fontMetrics.elidedText(str, Qt::ElideRight, 100);
		name->setText(elide);
		//Set label tips
		name->setToolTip(str);

		spinner1 = new QPiecewiseSpinBox;
		spinner1->setMinimumWidth(30);
		spinner1->setRange(castMinimum<int>(field->getMin()), castMaximum<int>(field->getMax()));

		spinner2 = new QPiecewiseSpinBox;
		spinner2->setMinimumWidth(30);
		spinner2->setRange(castMinimum<int>(field->getMin()), castMaximum<int>(field->getMax()));

		spinner3 = new QPiecewiseSpinBox;
		spinner3->setMinimumWidth(30);
		spinner3->setRange(castMinimum<int>(field->getMin()), castMaximum<int>(field->getMax()));




		layout->addWidget(name, 0, 0);
		layout->addWidget(spinner1, 0, 1);
		layout->addWidget(spinner2, 0, 2);
		layout->addWidget(spinner3, 0, 3);
		layout->setSpacing(3);

		std::string template_name = field->getTemplateName();
		int v1 = 0;
		int v2 = 0;
		int v3 = 0;

		if (template_name == std::string(typeid(Vec3i).name()))
		{
			FVar<Vec3i>* f = TypeInfo::cast<FVar<Vec3i>>(field);
			auto v = f->getValue();
			v1 = v[0];
			v2 = v[1];
			v3 = v[2];
		}
		else if (template_name == std::string(typeid(Vec3u).name()))
		{
			FVar<Vec3u>* f = TypeInfo::cast<FVar<Vec3u>>(field);
			auto v = f->getValue();
			v1 = v[0];
			v2 = v[1];
			v3 = v[2];
		}

		spinner1->setValue(v1);
		spinner2->setValue(v2);
		spinner3->setValue(v3);

		QObject::connect(spinner1, SIGNAL(valueChanged(int)), this, SLOT(updateField(int)));
		QObject::connect(spinner2, SIGNAL(valueChanged(int)), this, SLOT(updateField(int)));
		QObject::connect(spinner3, SIGNAL(valueChanged(int)), this, SLOT(updateField(int)));

		QObject::connect(this, SIGNAL(fieldChanged()), this, SLOT(updateWidget()));
	}

	QVector3iFieldWidget::~QVector3iFieldWidget()
	{
	}

	void QVector3iFieldWidget::updateField(int)
	{
		int v1 = spinner1->value();
		int v2 = spinner2->value();
		int v3 = spinner3->value();

		std::string template_name = field()->getTemplateName();

		if (template_name == std::string(typeid(Vec3i).name()))
		{
			FVar<Vec3i>* f = TypeInfo::cast<FVar<Vec3i>>(field());
			f->setValue(Vec3i(v1, v2, v3),false);
		}
		else if (template_name == std::string(typeid(Vec3u).name()))
		{
			FVar<Vec3u>* f = TypeInfo::cast<FVar<Vec3u>>(field());
			f->setValue(Vec3u(v1, v2, v3),false);
		}

		emit fieldChanged();
	}

	void QVector3iFieldWidget::updateWidget()
	{
		std::string template_name = field()->getTemplateName();

		int v1 = 0;
		int v2 = 0;
		int v3 = 0;

		if (template_name == std::string(typeid(Vec3i).name()))
		{
			FVar<Vec3i>* f = TypeInfo::cast<FVar<Vec3i>>(field());
			auto v = f->getValue();
			v1 = v[0];
			v2 = v[1];
			v3 = v[2];
		}
		else if (template_name == std::string(typeid(Vec3u).name()))
		{
			FVar<Vec3u>* f = TypeInfo::cast<FVar<Vec3u>>(field());
			auto v = f->getValue();
			v1 = v[0];
			v2 = v[1];
			v3 = v[2];
		}

		spinner1->blockSignals(true);
		spinner2->blockSignals(true);
		spinner3->blockSignals(true);

		spinner1->setValue(v1);
		spinner2->setValue(v2);
		spinner3->setValue(v3);

		spinner1->blockSignals(false);
		spinner2->blockSignals(false);
		spinner3->blockSignals(false);
	}
}

