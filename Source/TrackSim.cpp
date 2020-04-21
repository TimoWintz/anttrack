#include "TrackSim.hpp"

#include "Common.hpp"
#include "Track.hpp"



namespace TrackSim {
    using namespace Magnum;
    using namespace Math::Literals;

    TrackSimApp::TrackSimApp(const Arguments& arguments): Platform::Application{arguments, Configuration{}
        .setTitle("Track Simulator")
        .setWindowFlags(Configuration::WindowFlag::Resizable), GLConfiguration{}.setSampleCount(8) } {

        _manipulator.setParent(&_scene);
       /* Every scene needs a camera */
        _cameraObject
            .setParent(&_scene);

        (*(_camera = new SceneGraph::Camera3D{_cameraObject}))
            .setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
            .setProjectionMatrix(Matrix4::perspectiveProjection(110.0_degf, 1.0f, 0.01f, 10000.0f))
            .setViewport(GL::defaultFramebuffer.viewport().size());

        /* Setup renderer and shader defaults */
        GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
        GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
        GL::Renderer::enable(GL::Renderer::Feature::PolygonOffsetFill);
        GL::Renderer::setPolygonOffset(2.0f, 0.5f);
        _shader = Shaders::Flat3D{};

        /* Init Physics Engine */
        initPhysics();

        /* Dummy box object */
        /*
        _box = MeshTools::compile(Primitives::uvSphereSolid(10, 10));
        auto* o = new RigidBody{&_track, 1.0f, &_bSphereShape, _bWorld};
        o->translate({0.,0.,20.f});
        o->syncPose();
        o->rigidBody().setLinearVelocity({0.f,10.f,0.f});
        new ColoredDrawable{*o, _shader, _box, 0xffffff_rgbf, _drawables};
        */


        /* Load Track object */
        loadTrack();

        /* Load Rider */
        loadRider();

        /* Start timeline */
        _timeline.start();
    }
    void TrackSimApp::loadTrack() {
        PluginManager::Manager<Trade::AbstractImporter> manager;
        Containers::Pointer<Trade::AbstractImporter> importer = manager.loadAndInstantiate("AnySceneImporter");
        if(!importer) std::exit(1);

        /* Import track obj */
        if(!importer->openFile("Data/track.dae"))
            std::exit(4);

        _track.load(*importer);
        loadCamera(*importer, _cameraObject, _camera);
    }

    void TrackSimApp::loadRider() {
        PluginManager::Manager<Trade::AbstractImporter> manager;
        Containers::Pointer<Trade::AbstractImporter> importer = manager.loadAndInstantiate("AnySceneImporter");
        if(!importer) std::exit(1);

        /* Import track obj */
        if(!importer->openFile("Data/bike.dae"))
            std::exit(4);

        _rider.load(*importer);
    }

    void TrackSimApp::initPhysics() {
        /* Bullet setup */
        _debugDraw = BulletIntegration::DebugDraw{};
        _debugDraw.setMode(BulletIntegration::DebugDraw::Mode::DrawWireframe);
        _bWorld.setGravity({0.0f, 0.0f, -9.81f});
        _bWorld.setDebugDrawer(&_debugDraw);

        /* Loop at 60 Hz max */
        setSwapInterval(1);
        setMinimalLoopPeriod(16);
    }

    void TrackSimApp::drawEvent() {
        GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);
        /* Step bullet simulation */
        auto dt = _timeline.previousFrameDuration();
        _bWorld.stepSimulation(dt, 5);

        _camera->draw(_drawables);

        /* Draw colliders */
        if (_drawDebug) {
            _debugDraw.setTransformationProjectionMatrix(
            _camera->projectionMatrix()*_camera->cameraMatrix()*_manipulator.transformation());
            _bWorld.debugDrawWorld();
        }


        swapBuffers();
        _timeline.nextFrame();
        redraw();
    }

    void  TrackSimApp::viewportEvent(ViewportEvent& event) {
        GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
        _camera->setViewport(event.windowSize());
    }

    void TrackSimApp::mousePressEvent(MouseEvent& event) {
    if(event.button() == MouseEvent::Button::Left)
        _previousPosition = positionOnSphere(event.position());
}

    void TrackSimApp::mouseReleaseEvent(MouseEvent& event) {
        if(event.button() == MouseEvent::Button::Left)
            _previousPosition = Vector3();
    }

    void TrackSimApp::mouseScrollEvent(MouseScrollEvent& event) {
        if(!event.offset().y()) return;

        /* Distance to origin */
        const Float distance = _cameraObject.transformation().translation().z();

        /* Move 15% of the distance back or forward */
        _cameraObject.translate(Vector3::zAxis(
            distance*(1.0f - (event.offset().y() > 0 ? 1/0.85f : 0.85f))));

        redraw();
    }

    Vector3 TrackSimApp::positionOnSphere(const Vector2i& position) const {
        const Vector2 positionNormalized = Vector2{position}/Vector2{_camera->viewport()} - Vector2{0.5f};
        const Float length = positionNormalized.length();
        const Vector3 result(length > 1.0f ? Vector3(positionNormalized, 0.0f) : Vector3(positionNormalized, 1.0f - length));
        return (result*Vector3::yScale(-1.0f)).normalized();
    }

    void TrackSimApp::mouseMoveEvent(MouseMoveEvent& event) {
        if(!(event.buttons() & MouseMoveEvent::Button::Left)) return;

        const Vector3 currentPosition = positionOnSphere(event.position());
        const Vector3 axis = Math::cross(_previousPosition, currentPosition);

        if(_previousPosition.length() < 0.001f || axis.length() < 0.001f) return;

        _manipulator.rotate(Math::angle(_previousPosition, currentPosition), axis.normalized());
        _previousPosition = currentPosition;

        redraw();
    }

    void TrackSimApp::keyPressEvent(KeyEvent& event){
        /*Vector3 pos = positionOnSphere({0, 0});
        if (event.key() == KeyEvent::Key::Up) {
            _manipulator.translate(-pos);
        }
        if (event.key() == KeyEvent::Key::Down) {
            _manipulator.translate(pos);
        }
        redraw();*/
    }


    
}
MAGNUM_APPLICATION_MAIN(TrackSim::TrackSimApp)
