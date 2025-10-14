#include "GLWireframeVisualModule.h"

// opengl
#include <glad/glad.h>
#include "GLRenderEngine.h"
#include "Utility.h"

#include "line.vert.h"
#include "surface.frag.h"
#include "line.geom.h"

namespace dyno
{
	IMPLEMENT_CLASS(GLWireframeVisualModule)

	GLWireframeVisualModule::GLWireframeVisualModule()
	{
		this->setName("wireframe_renderer");
		this->varBaseColor()->setValue(Color::Grey21());
		this->varRadius()->setRange(0.001, 0.01);
	}

	GLWireframeVisualModule::~GLWireframeVisualModule()
	{
// 		edges.clear();
// 		vertices.clear();

// 		mVertexBuffer.release();
// 		mIndexBuffer.release();
	}


	std::string GLWireframeVisualModule::caption()
	{
		return "Wireframe Visual Module";
	}

	bool GLWireframeVisualModule::initializeGL()
	{
		mVAO.create();

		mIndexBuffer.create(GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW);
		mVertexBuffer.create(GL_ARRAY_BUFFER, GL_STATIC_DRAW);

		mVAO.bindIndexBuffer(&mIndexBuffer);
		mVAO.bindVertexBuffer(&mVertexBuffer, 0, 3, GL_FLOAT, 0, 0, 0);

		// create shader program
		mShaderProgram = Program::createProgramSPIRV(
			LINE_VERT, sizeof(LINE_VERT),
			SURFACE_FRAG, sizeof(SURFACE_FRAG),
			LINE_GEOM, sizeof(LINE_GEOM));

		// create shader uniform buffer
		mUniformBlock.create(GL_UNIFORM_BUFFER, GL_DYNAMIC_DRAW);

		return true;
	}

	void GLWireframeVisualModule::releaseGL()
	{
		mShaderProgram->release();
		delete mShaderProgram;

		mVAO.release();
		mVertexBuffer.release();
		mIndexBuffer.release();

		mUniformBlock.release();
	}


	void GLWireframeVisualModule::updateGL()
	{
		mNumEdges = mIndexBuffer.count();
		if (mNumEdges == 0) return;

		mVertexBuffer.updateGL();
		mIndexBuffer.updateGL();
	}

	void GLWireframeVisualModule::updateImpl()
	{
		// copy data
		auto edgeSet = this->inEdgeSet()->getDataPtr();
		auto edges = edgeSet->edgeIndices();
		auto vertices = edgeSet->getPoints();

		mVertexBuffer.load(vertices);
		mIndexBuffer.load(edges);
	}


	void GLWireframeVisualModule::paintGL(const RenderParams& rparams)
	{
		if (mNumEdges == 0)
			return;

		// setup uniform buffer
		mUniformBlock.load((void*)&rparams, sizeof(RenderParams));
		mUniformBlock.bindBufferBase(0);

		mShaderProgram->use();

		if (rparams.mode == GLRenderMode::COLOR)
		{
			Color c = this->varBaseColor()->getValue();
			mShaderProgram->setVec3("uBaseColor", Vec3f(c.r, c.g, c.b));
			mShaderProgram->setFloat("uMetallic", this->varMetallic()->getValue());
			mShaderProgram->setFloat("uRoughness", this->varRoughness()->getValue());
			mShaderProgram->setFloat("uAlpha", this->varAlpha()->getValue());
		}
		else if (rparams.mode == GLRenderMode::SHADOW)
		{
			// cast shadow?
		}
		else
		{
			printf("Unknown render pass!\n");
			return;
		}

		// preserve previous polygon mode
		int mode;
		glGetIntegerv(GL_POLYGON_MODE, &mode);

		if (this->varRenderMode()->getDataPtr()->currentKey() == EEdgeMode::LINE)
		{
			mShaderProgram->setInt("uEdgeMode", 0);
			// draw as lines
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glLineWidth(this->varLineWidth()->getData());
		}
		else
		{
			// draw as cylinders
			mShaderProgram->setInt("uEdgeMode", 1);
			mShaderProgram->setFloat("uRadius", this->varRadius()->getValue());
		}

		mVAO.bindIndexBuffer(&mIndexBuffer);
		mVAO.bindVertexBuffer(&mVertexBuffer, 0, 3, GL_FLOAT, 0, 0, 0);
		mVAO.bind();

		glDrawElements(GL_LINES, mNumEdges * 2, GL_UNSIGNED_INT, 0);

		// restore polygon mode
		glPolygonMode(GL_FRONT_AND_BACK, mode);

		glCheckError();
	}
}