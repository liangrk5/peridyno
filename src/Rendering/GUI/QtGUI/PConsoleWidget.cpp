#include "PConsoleWidget.h"
#include "Platform.h"
#include <QListWidget>
#include <QDir>
#include <QStringList>
#include <QHBoxLayout>
#include <QListView>
#include <QPixmap>
#include "NodeFactory.h"

#include "PSimulationThread.h"

namespace dyno
{
	class CustomFileSystemModel : public QFileSystemModel
	{
	public:
		explicit CustomFileSystemModel(QObject* parent = nullptr) {};
		~CustomFileSystemModel() {};
	private:
		QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const
		{
			if (index.isValid())
			{
				if (role == Qt::DecorationRole)
				{
					QFileInfo info = CustomFileSystemModel::fileInfo(index);
					if (info.isFile())
					{
						if (info.suffix() == "png" || info.suffix() == "jpg" || info.suffix() == "bmp")
						{
							std::string iconPath = getAssetPath() + "/icon/ContentBrowser/image.png";
							return QPixmap(iconPath.c_str());
						}
						else if (info.suffix() == "obj" || info.suffix() == "gltf" || info.suffix() == "glb" || info.suffix() == "fbx" || info.suffix() == "STL" || info.suffix() == "stl")
						{
							std::string iconPath = getAssetPath() + "/icon/ContentBrowser/3dModel.png";
							return QPixmap(iconPath.c_str());
						}
					}
				}
			}

			return QFileSystemModel::data(index, role);
		}
	};

#ifdef PERIDYNO_QT_PYTHON_CONSOLE

	PConsoleWidget::PConsoleWidget(QWidget* parent) :
		QWidget(parent)
	{

		QVBoxLayout* layout = new QVBoxLayout(this);
		this->setLayout(layout);

		mCodeEditor = new QsciScintilla(this);
		mCodeEditor->setObjectName("Python Code");

		mCodeEditor->setMinimumSize(100, 100);

		mPythonLexer = new QsciLexerPython(mCodeEditor);
		applyDarkTheme(mPythonLexer);
		mCodeEditor->setLexer(mPythonLexer);

		QsciAPIs* apis = new QsciAPIs(mPythonLexer);
		apis->add(QString("import"));
		apis->add(QString("QtPathHelper"));
		apis->add(QString("PyPeridyno"));
		apis->add(QString("scn"));
		apis->add(QString("dyno"));
		apis->add(QString("addNode"));
		apis->add(QString("SceneGraph"));
		apis->add(QString("connect"));
		apis->add(QString("pushModule"));
		apis->add(QString("setValue"));
		apis->add(QString("Vector3f"));
		apis->prepare();

		mCodeEditor->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		mCodeEditor->setCaretForegroundColor(QColor("#d4d4d4"));
		
		// line numbers
		mCodeEditor->setMarginsBackgroundColor(QColor("#454545"));
		mCodeEditor->setMarginsForegroundColor(QColor("#d4d4d4"));
		mCodeEditor->setMarginLineNumbers(1, true);
		mCodeEditor->setMarginWidth(1, "0000");

		// auto completion
		mCodeEditor->setAutoCompletionSource(QsciScintilla::AcsAll);
		mCodeEditor->setAutoCompletionCaseSensitivity(true);
		mCodeEditor->setAutoCompletionThreshold(1);

		// indentation
		mCodeEditor->setIndentationsUseTabs(false);
		mCodeEditor->setIndentationWidth(4);
		mCodeEditor->setTabWidth(4);
		mCodeEditor->setAutoIndent(true);

		// font
		mCodeEditor->SendScintilla(QsciScintilla::SCI_SETCODEPAGE, QsciScintilla::SC_CP_UTF8);

		mCodeEditor->setMouseTracking(true);

		layout->addWidget(mCodeEditor);

		updateButton = new QPushButton("update", this);
		layout->addWidget(updateButton);


		connect(updateButton, &QPushButton::clicked, this, [this]() {
			const std::string& code = mCodeEditor->text().toStdString();
			execute(code);
			});

	}

	PConsoleWidget::~PConsoleWidget()
	{
	}


	void PConsoleWidget::execute(const std::string& src)
	{
		py::scoped_interpreter guard{};

		try {
			auto locals = py::dict();
			py::exec(src, py::globals(), locals);

			if (locals.contains("scn"))
			{
				auto scene = locals["scn"].cast<std::shared_ptr<dyno::SceneGraph>>();
				if (scene)
				{
					PSimulationThread::instance()->createNewScene(scene);
				}
			}
			else
			{
				std::cout << getPythonErrorDetails() << std::endl;
			}
		}
		catch (const std::exception& e) {
			QMessageBox::critical(nullptr, "Error", e.what());
		}
	}

	std::string PConsoleWidget::getPythonErrorDetails()
	{
		if (!PyErr_Occurred()) {
			return "No Python error occurred";
		}

		PyObject* type, * value, * traceback;
		PyErr_Fetch(&type, &value, &traceback);
		PyErr_NormalizeException(&type, &value, &traceback);

		std::string errorMsg;

		if (value) {
			py::object py_value = py::reinterpret_borrow<py::object>(value);
			errorMsg = py::str(py_value).cast<std::string>();
		}
		else {
			errorMsg = "Unknown Python error";
		}

		// ����� traceback����ȡ��ջ��Ϣ
		if (traceback) {
			py::object py_traceback = py::reinterpret_borrow<py::object>(traceback);

			// ���� traceback ģ������ʽ��������Ϣ
			try {
				py::module traceback_module = py::module::import("traceback");
				py::object format_tb = traceback_module.attr("format_tb");
				py::object tb_list = format_tb(py_traceback);
				py::object formatted = traceback_module.attr("format_exception_only")(type, value);

				// ��������Ĵ�����Ϣ
				std::string traceback_str;
				for (auto item : tb_list) {
					traceback_str += item.cast<std::string>();
				}
				for (auto item : formatted) {
					traceback_str += item.cast<std::string>();
				}

				if (!traceback_str.empty()) {
					errorMsg = traceback_str;
				}
			}
			catch (...) {
				// �����ʽ��ʧ�ܣ�ʹ�û���������Ϣ
			}
		}

		PyErr_Restore(type, value, traceback);
		PyErr_Clear();

		return errorMsg;
	}

	void PConsoleWidget::applyDarkTheme(QsciLexerPython* lexer)
	{
		if (!lexer)
			return;

		// set font
		QFont font("Courier New");
		lexer->setFont(font);

		// set color
		QColor backgroundColor("#1c1c1c");
		QColor defaultColor("#d4d4d4");
		QColor commentColor(87, 166, 74);
		QColor numberColor(181, 206, 168);
		QColor stringColor(206, 145, 120);
		QColor keywordColor(86, 156, 214);
		QColor classNameColor(78, 201, 176);
		QColor functionColor(220, 220, 170);
		QColor operatorColor(180, 180, 180);
		QColor identifierColor(220, 220, 220);
		QColor decoratorColor(197, 134, 192);
		QColor fstringColor(214, 157, 133);
		QColor highlightedIdColor(255, 215, 0);
		QColor unclosedStringColor(255, 100, 100);

		// set background color
		lexer->setPaper(backgroundColor);
		lexer->setDefaultPaper(backgroundColor);

		// set font color
		lexer->setColor(defaultColor);
		lexer->setDefaultColor(defaultColor);

		// comment
		lexer->setColor(commentColor, 1);                    // Comment
		lexer->setColor(commentColor, 12);                   // CommentBlock

		// number
		lexer->setColor(numberColor, 2);                     // Number

		// string
		lexer->setColor(stringColor, 3);                     // DoubleQuotedString
		lexer->setColor(stringColor, 4);                     // SingleQuotedString
		lexer->setColor(stringColor, 6);                     // TripleSingleQuotedString
		lexer->setColor(stringColor, 7);                     // TripleDoubleQuotedString

		// f-string
		lexer->setColor(fstringColor, 16);                   // DoubleQuotedFString
		lexer->setColor(fstringColor, 17);                   // SingleQuotedFString
		lexer->setColor(fstringColor, 18);                   // TripleSingleQuotedFString
		lexer->setColor(fstringColor, 19);                   // TripleDoubleQuotedFString

		// keyword
		lexer->setColor(keywordColor, 5);                    // Keyword
		lexer->setColor(classNameColor, 8);                  // ClassName
		lexer->setColor(functionColor, 9);                   // FunctionMethodName
		lexer->setColor(operatorColor, 10);                  // Operator
		lexer->setColor(identifierColor, 11);                // Identifier
		lexer->setColor(highlightedIdColor, 14);             // HighlightedIdentifier
		lexer->setColor(decoratorColor, 15);                 // Decorator

		// error word
		lexer->setColor(unclosedStringColor, 13);            // UnclosedString

		// set font
		lexer->setFont(font, 8);                             // ClassName
		lexer->setFont(font, 9);                             // FunctionMethodName

		//// keyword bold font
		//QFont boldFont("Courier New", 10);
		//boldFont.setBold(true);
		//lexer->setFont(boldFont, 5);

	}

#endif // PERIDYNO_QT_PYTHON_CONSOLE

	QContentBrowser::QContentBrowser(QWidget* parent /*= nullptr*/)
		: QWidget()
	{
		QHBoxLayout* layout = new QHBoxLayout(this);
		this->setLayout(layout);

		std::string path = getAssetPath();
		QDir root(path.c_str());

		//Add file browser
		model = new QFileSystemModel(this);
		model->setRootPath(path.c_str());
		model->setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
		model->sort(0, Qt::AscendingOrder);

//		QObject::connect(model, SIGNAL(directoryLoaded(QString)), SLOT(findDirectory(QString)));

		// File system
		treeView = new QTreeView(this);
		treeView->setModel(model);
		treeView->setHeaderHidden(true);	//Show tree header
		treeView->setFixedWidth(520);
		treeView->hideColumn(1);
		treeView->hideColumn(2);			//Hide second column
		treeView->hideColumn(3);
		treeView->setRootIndex(model->index(path.c_str()));
		layout->addWidget(treeView);



		QStringList filter;
		filter <<"*.png" << "*.jpg" << "*.bmp" << "*.obj" << "*.gltf" << "*.glb" << "*.fbx" << "*.STL" << "*.stl" << "*.xml";
		auto* listModel = new CustomFileSystemModel(this);
		listModel->setRootPath(path.c_str());
		listModel->setFilter(QDir::Files | QDir::NoDotAndDotDot);
		listModel->setNameFilters(filter);
		listModel->setNameFilterDisables(false);
		listModel->sort(0, Qt::AscendingOrder);

		listView = new QListView;
		listView->setModel(listModel);
		listView->setViewMode(QListView::IconMode);
		listView->setIconSize(QSize(80, 80));
		listView->setGridSize(QSize(120, 120));
		listView->setUniformItemSizes(true);
		listView->setResizeMode(QListWidget::Adjust);
		listView->setTextElideMode(Qt::ElideRight);
		listView->setRootIndex(listModel->index(path.c_str()));
		layout->addWidget(listView);

		connect(treeView, SIGNAL(clicked(const QModelIndex&)),
			this, SLOT(treeItemSelected(const QModelIndex&)));

		connect(listView, SIGNAL(clicked(const QModelIndex&)),
			this, SLOT(assetItemSelected(const QModelIndex&)));

		connect(listView, SIGNAL(doubleClicked(const QModelIndex&)),
			this, SLOT(assetDoubleClicked(const QModelIndex&)));
	}

	void QContentBrowser::treeItemSelected(const QModelIndex& index)
	{
		QString name = model->fileName(index);
		QString path = model->fileInfo(index).absolutePath() + "/" + name;

		//A hack
		QStringList filter;
		filter << "*.png" << "*.jpg" << "*.bmp" << "*.obj" << "*.gltf" << "*.glb" << "*.fbx" << "*.STL" << "*.stl" << "*.xml";
		auto* newListModel = new CustomFileSystemModel(this);
		newListModel->setRootPath(path);
		newListModel->setFilter(QDir::Files | QDir::NoDotAndDotDot);
		newListModel->setNameFilters(filter);
		newListModel->setNameFilterDisables(false);
		newListModel->sort(0, Qt::AscendingOrder);

 		listView->setModel(newListModel);
		listView->setRootIndex(newListModel->index(path));
	}

	void QContentBrowser::assetItemSelected(const QModelIndex& index)
	{
		QString name = model->fileName(index);
		QString path = model->fileInfo(index).absolutePath() + "/" + name;


	}

	void QContentBrowser::assetDoubleClicked(const QModelIndex& index)
	{
		QString name = model->fileName(index);
		QString path = model->fileInfo(index).absolutePath() + "/" + name;

		std::cout << path.toStdString() << "\n";

		auto ext = model->fileInfo(index).suffix().toStdString();
		auto ext2Act = NodeFactory::instance()->nodeContentActions();
		if (ext2Act.find(ext) != ext2Act.end())
		{
			auto func = ext2Act[ext];
			if (func != nullptr) {
				auto node = func(path.toStdString());

				emit nodeCreated(node);
			}
		}
	}
}