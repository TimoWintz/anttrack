#pragma once
#include "Common.hpp"
#include <Magnum/SceneGraph/FeatureGroup.h> 

namespace TrackSim {
    using namespace Magnum;
    using namespace Platform;
    
    class LogicComponent : public SceneGraph::AbstractGroupedFeature<3, LogicComponent, Float> {
        public:
            explicit LogicComponent(Object3D& object, SceneGraph::FeatureGroup3D<LogicComponent>& group):
                SceneGraph::AbstractGroupedFeature<3, LogicComponent, Float>(object, &group) {};
            virtual void start() = 0;
            virtual void update(Float dt) = 0;
            virtual void sendInputEvent(Application::InputEvent& e) = 0;
            virtual void stop() = 0;
    };
    typedef SceneGraph::FeatureGroup3D<LogicComponent> LogicComponentGroup;
    
}