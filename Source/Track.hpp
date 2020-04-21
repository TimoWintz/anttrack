#pragma once
#include "Common.hpp"
#include "RigidBody.hpp"
#include "LogicComponent.hpp"



namespace TrackSim {
    using namespace Magnum;
    class Track: public RigidBody {
        public:
            Track(Object3D* parent, btDiscreteDynamicsWorld* bWorld, SceneGraph::DrawableGroup3D& drawables) : RigidBody{parent, bWorld}, _drawables(drawables) {};
            void load(Trade::AbstractImporter& importer);

        private:
            btBvhTriangleMeshShape* _colliderMesh;
            btTriangleIndexVertexArray* _triangleIndexVertexArray;
            Containers::Array<Containers::Optional<GL::Mesh>> _meshes;
            Shaders::Flat3D _shader;
            SceneGraph::DrawableGroup3D& _drawables;
            
    };
}
