#pragma once
#include "Common.hpp"
#include "RigidBody.hpp"
#include "Track.hpp"
#include "LogicComponent.hpp"
#include "Rider.hpp"

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
#include <Magnum/SceneGraph/FeatureGroup.h> 

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


namespace TrackSim {
    using namespace Magnum;

    class TrackSimApp: public Platform::Application {
        public:
            explicit TrackSimApp(const Arguments& arguments);

        private:
            void drawEvent() override;
            void viewportEvent(ViewportEvent& event) override;
            void mousePressEvent(MouseEvent& event) override;
            void mouseReleaseEvent(MouseEvent& event) override;
            void mouseMoveEvent(MouseMoveEvent& event) override;
            void mouseScrollEvent(MouseScrollEvent& event) override;
            void keyPressEvent(KeyEvent& event) override;

            Vector3 positionOnSphere(const Vector2i& position) const;
            Vector3 _previousPosition;

            void loadTrack();
            void loadRider();
            void initPhysics();

            
            

            SceneGraph::Camera3D* _camera;
            SceneGraph::DrawableGroup3D _drawables;
            LogicComponentGroup _logicComponents;

            GL::Mesh _box;
            Shaders::Flat3D _shader;

            // Physics
            btDbvtBroadphase _bBroadphase;
            btDefaultCollisionConfiguration _bCollisionConfig;
            btCollisionDispatcher _bDispatcher{&_bCollisionConfig};
            btSequentialImpulseConstraintSolver _bSolver;

            bool _drawDebug{true};

            /* The world has to live longer than the scene because RigidBody
            instances have to remove themselves from it on destruction */
            btDiscreteDynamicsWorld _bWorld{&_bDispatcher, &_bBroadphase, &_bSolver, &_bCollisionConfig};
            BulletIntegration::DebugDraw _debugDraw{NoCreate};

            Scene3D _scene;
            Object3D _cameraObject;
            Object3D _manipulator; // Parent of all objects in the scene
            Track _track{&_manipulator, &_bWorld, _drawables};
            Rider _rider{&_manipulator, &_bWorld, _drawables, _logicComponents};
            //Containers::Array<Object3D> _riders;
            

            Timeline _timeline;
    };

    
}