#include "Rider.hpp"

namespace TrackSim {
    using namespace Magnum;
    using namespace Math::Literals;
    Rider::Rider(Object3D* parent, btDiscreteDynamicsWorld* bWorld, SceneGraph::DrawableGroup3D& drawables, LogicComponentGroup& logicComponents): RigidBody{parent, bWorld}, LogicComponent{*((Object3D*) this), logicComponents}, _drawables(drawables)
    {
        _fEngineForce = 0.0f;                                                 
        _fBreakingForce = 0.0f;                                               
                                                                            
        _fmaxEngineForce = 2500.f;//this should be engine/velocity dependent  
        _fmaxBreakingForce = 100.f;                                           
                                                                            
        _fVehicleSteering = 0.0f;                                             
        _fsteeringIncrement = 0.04f;                                          
        _fsteeringClamp = 0.3f;                                               
        _fwheelRadius = 0.35f;                                                 
        _fwheelWidth = 0.1f;                                                  
        _fwheelFriction = 1000;//BT_LARGE_FLOAT;                              
        _fsuspensionStiffness = 14.0f;//20.f;                                 
        _fsuspensionDamping = 2.0f;//2.3f;                                    
        _fsuspensionCompression = 4.0f;//4.4f;                                
        _frollInfluence = 0.01f;//1.0f;                                       
        _fsuspensionRestLength = 0.3f;//0.6

        _mass = 73.0f;
        _collisionLength = 2.0f;
        _collisionWidth = 0.5f;
        _collisionHeight = 1.0f;

        _vehicleRayCaster = NULL;
        _vehicle = NULL;
    }
    void Rider::load(Trade::AbstractImporter& importer) {
        loadScene(importer, _meshes, this, _shader, _drawables);
        /* Calculate inertia so the object reacts as it should with
        rotation and everything */
        btVector3 bInertia(0.0f, 0.0f, 0.0f);
        
        _bBoxShape.emplace(btBoxShape({0.5f * _collisionLength, 0.5f * _collisionWidth, 0.5f * _collisionHeight}));
        _bBoxShape->calculateLocalInertia(_mass, bInertia);

        /* Bullet rigid body setup */
        auto* motionState = new BulletIntegration::MotionState{*this};
        _bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{
            _mass, &motionState->btMotionState(), _bBoxShape.get(), bInertia});
        _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
        _bWorld->addRigidBody(_bRigidBody.get());
        rotateZ(90.0_degf);
        translate({0.0f, 0.0f, 3.0f * _collisionHeight});
        syncPose();
        //_bRigidBody->setLinearVelocity({0.0f, 0.0f, 5.0f * _collisionHeight});

        _vehicleRayCaster.emplace(btDefaultVehicleRaycaster(_bWorld));
        _vehicle.emplace(btRaycastVehicle(_tuning, _bRigidBody.get(), _vehicleRayCaster.get()));
        _vehicle->setCoordinateSystem( 1, 2, 0 );
        

        btVector3 wheelDirectionCS0(0,0,-1);
        btVector3 wheelAxleCS(0,1,0);

        btVector3 connectionPointCS0(0.5, 0.0f, -0.25f);
        _vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,_fsuspensionRestLength,_fwheelRadius,_tuning,true);
        

        btVector3 connectionPointCS1(-0.5, -_collisionWidth/2, 0.0f);
        _vehicle->addWheel(connectionPointCS1,wheelDirectionCS0,wheelAxleCS,_fsuspensionRestLength,_fwheelRadius,_tuning,false);
        btVector3 connectionPointCS2(-0.5, _collisionWidth/2, 0.0f);
        _vehicle->addWheel(connectionPointCS2,wheelDirectionCS0,wheelAxleCS,_fsuspensionRestLength,_fwheelRadius,_tuning,false);

        _vehicle->applyEngineForce(-100, 1);
        _vehicle->applyEngineForce(-100, 2);
        _vehicle->setBrake(0,0);

        _bWorld->addVehicle(_vehicle.get());

        for ( int i = 0; i < _vehicle->getNumWheels(); i++ )
        {
            btWheelInfo& wheel = _vehicle->getWheelInfo( i );
            wheel.m_suspensionStiffness = _fsuspensionStiffness;
            wheel.m_wheelsDampingRelaxation = _fsuspensionDamping;
            wheel.m_wheelsDampingCompression = _fsuspensionCompression;
            wheel.m_frictionSlip = _fwheelFriction;
            wheel.m_rollInfluence = _frollInfluence;
        }
    }
}

