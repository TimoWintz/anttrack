#pragma once
#include "Common.hpp"
#include "RigidBody.hpp"
#include "LogicComponent.hpp"

#include <btBulletDynamicsCommon.h>
#include <Bullet/BulletDynamics/Vehicle/btRaycastVehicle.h>


namespace TrackSim {
    using namespace Magnum;
    class Rider: public RigidBody, LogicComponent {
        public:
            Rider(Object3D* parent, btDiscreteDynamicsWorld* bWorld, SceneGraph::DrawableGroup3D& drawables, LogicComponentGroup& logicComponents);

            btRigidBody& rigidBody() { return *_bRigidBody; }

            void load(Trade::AbstractImporter& importer);

            void start() override {};
            void update(Float dt) override {};
            void sendInputEvent(Application::InputEvent& e) override {};
            void stop() override {};

        private:
            //Collision
            Containers::Pointer<btBoxShape> _bBoxShape;

            // raycast vehicle
            btRaycastVehicle::btVehicleTuning _tuning;
            Containers::Pointer<btDefaultVehicleRaycaster> _vehicleRayCaster;
            Containers::Pointer<btRaycastVehicle> _vehicle;

            /// Current left/right steering amount (-1 to 1.)
            float _steering;

            float	_fEngineForce;
            float	_fBreakingForce;

            float	_fmaxEngineForce;
            float	_fmaxBreakingForce;

            float	_fVehicleSteering;
            float	_fsteeringIncrement;
            float	_fsteeringClamp;
            float	_fwheelRadius;
            float	_fwheelWidth;
            float	_fwheelFriction;
            float	_fsuspensionStiffness;
            float	_fsuspensionDamping;
            float	_fsuspensionCompression;
            float	_frollInfluence;
            float   _fsuspensionRestLength;

            float _mass;
            float _collisionLength;
            float _collisionWidth;
            float _collisionHeight;
           

            //Drawable group
            Containers::Array<Containers::Optional<GL::Mesh>> _meshes;
            Shaders::Flat3D _shader;
            SceneGraph::DrawableGroup3D& _drawables;
    };
}

