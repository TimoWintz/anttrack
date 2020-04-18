#pragma once

#include "btBulletDynamicsCommon.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/DebugStl.h>
#include <Magnum/ImageView.h>
#include <Magnum/Mesh.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Timeline.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/Trade/CameraData.h>

#define COLLIDER_OBJECT_ID "Collider"


namespace TrackSim {
    using namespace Magnum;
    typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
    typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;

    void loadScene(Trade::AbstractImporter& importer, Containers::Array<Containers::Optional<GL::Mesh>>& meshes, Object3D* parent, Shaders::Flat3D& shader, SceneGraph::DrawableGroup3D& drawables);
    void loadCollider(Trade::AbstractImporter& importer, btBvhTriangleMeshShape* colliderMesh, btTriangleIndexVertexArray* triangleIndexVertexArray, btDiscreteDynamicsWorld* bWorld, Object3D* parent);
    void loadCamera(Trade::AbstractImporter& importer, Object3D& cameraObject, SceneGraph::Camera3D* camera);
    void addObject(Trade::AbstractImporter& importer, Containers::Array<Containers::Optional<GL::Mesh>>& meshes,
                                Containers::ArrayView<const Containers::Optional<Trade::PhongMaterialData>> materials, Object3D* parent, UnsignedInt i,  Shaders::Flat3D& shader, SceneGraph::DrawableGroup3D& drawables);

    class ColoredDrawable: public SceneGraph::Drawable3D {
    public:
        explicit ColoredDrawable(Object3D& object, Shaders::Flat3D& shader, GL::Mesh& mesh, const Color4& color, SceneGraph::DrawableGroup3D& group): SceneGraph::Drawable3D{object, &group}, _shader(shader), _mesh(mesh), _color{color} {}

    private:
        void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override;

        Shaders::Flat3D& _shader;
        GL::Mesh& _mesh;
        Color4 _color;
    };
}