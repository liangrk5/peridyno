#pragma once

#include "Node/ParametricModel.h"

#include "Field/FilePath.h"

#include "Topology/TriangleSet.h"
#include "Topology/TextureMesh.h"

namespace dyno
{

	class TextureMeshLoader : public ParametricModel<DataType3f>
	{
		DECLARE_CLASS(TextureMeshLoader)
	public:
		TextureMeshLoader();
		~TextureMeshLoader() override;
		std::string getNodeType() override { return "IO"; }

	public:

		DEF_VAR(FilePath, FileName, "", "The full obj file name");

		DEF_VAR(bool, UseInstanceTransform, true, "");

		DEF_INSTANCE_STATE(TextureMesh, TextureMesh, "");

	protected:
		void resetStates() override;

	private:
		void callbackLoadFile();
		void callbackTransform();

		CArray<Vec3f> initialShapeCenter;
		DArray<Vec3f> mInitialVertex;
		DArray<Vec3f> mInitialNormal;
		DArray<Vec2f> mInitialTexCoord;
	};

}