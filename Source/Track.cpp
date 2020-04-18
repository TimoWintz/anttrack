#include "Track.hpp"

namespace TrackSim {
    void Track::load(Trade::AbstractImporter& importer) {
        loadScene(importer, _meshes, this, _shader, _drawables);
        loadCollider(importer, _colliderMesh, _triangleIndexVertexArray, _bWorld, this);
        /* Bullet rigid body setup */
        btVector3 bInertia(0.0f, 0.0f, 0.0f);
        auto* motionState = new BulletIntegration::MotionState{*this};
        _bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{
                    0.0f, &motionState->btMotionState(), _colliderMesh, bInertia});
                _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
                _bWorld->addRigidBody(_bRigidBody.get());
    }
}
