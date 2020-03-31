#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <toml.hpp> // that's all! now you can use it.
#include <stdlib.h>     /* strtol */
#include <fstream>
#include <mio/mio.hpp>
#include <chrono>
#include <thread>
#include <codecvt>

#include <Magnum/Image.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Grid.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/VertexColor.h>
#include <Magnum/Timeline.h>
// #include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Trade/MeshData3D.h>


#include "track.h"
#include "rider.h"
#include "utils.h"
#include "race.h"
#include "gamelogic.h"

using namespace Magnum;
using namespace Math::Literals;

using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;

struct VertexColorData {
    Vector3 pos;
    Color3 color;
};
static const Color3 track_color(0x0000ff_rgbf);
static const Color3 track_color_up(0x000000_rgbf);
static const Color3 track_color_finish(0xffffff_rgbf);

/* class Rider3D : public Object3D {
    const Rider& _rider;
    std::string _window_title;
    std::stringstream _status;

    public:
        Rider3D(Scene3D* scene, const Rider& rider) : Object3D{scene}, _rider(rider) {
                std::stringstream ss;
                // ss << "Rider " << rider->id();
                _window_title = ss.str();
            };
        void update() {
            auto pos_xyz = _rider.pos_xyz();
            auto transformation = Math::Matrix4<float>::translation({float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2]) + 1}) *
                    Math::Matrix4<float>::scaling({0.5f, 0.5f, 0.5f});
            setTransformation(transformation);

            DescMap map = _rider.desc();
            _status.str("");
            for (auto x: map) {
                _status << x.first << ": \n\t" << x.second << std::endl;
            }
        }

        const Rider& rider() const {
            return _rider;
        }
}; */

class TrackApp: public Platform::Application {
    public:
        explicit TrackApp(const Arguments& arguments);

    private:
        void drawEvent() override;
        void reset();
        void receiveInput();

        void loadMeshes();

        Timeline timeline;
        double _time_scaling;

        toml::value _config;

        Shaders::VertexColor3D _vertexColorShader{NoCreate};
        Shaders::Flat3D _flatShader{NoCreate};
        GL::Mesh _track_mesh{NoCreate}, _grid{NoCreate};
        std::map<int, GL::Mesh> _rider_meshes;
        std::map<int, Object3D*> _rider_objects;

        Scene3D _scene;
        SceneGraph::DrawableGroup3D _drawables;

        Object3D* _cameraObject;
        SceneGraph::Camera3D* _camera;

        Vector2i _lastPosition{-1};
        Vector3 _rotationPoint, _translationPoint;

        mio::mmap_source commands_mmap;
        mio::mmap_sink status_mmap;

        char _buffer[4096];
        char _buffer_index;
        std::string _line;
        std::stringstream _line_stream;

        void outputStatus();

        bool _autostart;
};

class VertexColorDrawable: public SceneGraph::Drawable3D {
    public:
        explicit VertexColorDrawable(Object3D& object, Shaders::VertexColor3D& shader, GL::Mesh& mesh, SceneGraph::DrawableGroup3D& drawables): SceneGraph::Drawable3D{object, &drawables}, _shader(shader), _track_mesh(mesh) {}

        void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
            _shader.setTransformationProjectionMatrix(camera.projectionMatrix()*transformation);
            _track_mesh.draw(_shader);
        }

    private:
        Shaders::VertexColor3D& _shader;
        GL::Mesh& _track_mesh;
};

class FlatDrawable: public SceneGraph::Drawable3D {
    public:
        explicit FlatDrawable(Object3D& object, Shaders::Flat3D& shader, GL::Mesh& mesh, SceneGraph::DrawableGroup3D& drawables): SceneGraph::Drawable3D{object, &drawables}, _shader(shader), _track_mesh(mesh) {}

        void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
            _shader.setColor(0x747474_rgbf)
                .setTransformationProjectionMatrix(camera.projectionMatrix()*transformation);
            _track_mesh.draw(_shader);
        }

    private:
        Shaders::Flat3D& _shader;
        GL::Mesh& _track_mesh;
};

TrackApp::TrackApp(const Arguments& arguments): Platform::Application{arguments, NoCreate} {
    /* Load TOML config */
    std::cout << "Loading config...";
    _config = toml::parse("config.toml");
    std::cout << "Done." << std::endl;

    
    std::cout << "Initializing game logic...";
    /* Setup Game Logic */
    GameLogic::get().init_from_config(_config);
    std::cout << "Done." << std::endl;

    std::cout << "Init memory maps...";
    int page_size = 1024;
    std::error_code error;
    allocate_file("status", page_size);
    commands_mmap = mio::make_mmap_source("commands", error);
    status_mmap = mio::make_mmap_sink("status", error);

    if (error) {
        throw std::runtime_error("error");
    }
    std::cout << "Done." << std::endl;

    /* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
       MSAA if we have enough DPI. */
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("Track")
            .setSize({1600, 960}, dpiScaling);
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if(!tryCreate(conf, glConf))
            create(conf, glConf.setSampleCount(0));
    }


    /* Shaders, renderer setup */
    _vertexColorShader = Shaders::VertexColor3D{};
    _flatShader = Shaders::Flat3D{};
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

    loadMeshes();

    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add,
                            GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                            GL::Renderer::BlendFunction::OneMinusSourceAlpha);

    timeline.start();
    _buffer_index = 0;

    _autostart = toml::find<bool>(_config.at("General"), "autostart");
    _time_scaling = 5.0;
}


void TrackApp::drawEvent() {
    receiveInput();
    outputStatus();
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);
    double dt = _time_scaling*double(timeline.previousFrameDuration());
    GameLogic::get().update(dt);
    for (auto id : GameLogic::get().rider_ids()) {
        Vec3f xyz = GameLogic::get().rider_position_xyz(id);
        auto transformation = Math::Matrix4<float>::translation({float(xyz[0]), float(xyz[1]), float(xyz[2]) + 1}) *
                    Math::Matrix4<float>::scaling({0.5f, 0.5f, 0.5f});
        _rider_objects[id]->setTransformation(transformation);

    }
    _camera->draw(_drawables);
    swapBuffers();
    timeline.nextFrame();
    redraw();
}

void TrackApp::loadMeshes() {

    std::cout << "Loading meshes...";
      /* Grid */
    _grid = MeshTools::compile(Primitives::grid3DWireframe({15, 15}));
    auto grid = new Object3D{&_scene};
    (*grid)
        .scale(Vector3{100.0f});
    new FlatDrawable{*grid, _flatShader, _grid, _drawables};

    /* Set up the camera */
    _cameraObject = new Object3D{&_scene};
    (*_cameraObject)
        .translate(Vector3::zAxis(200.0f))
        .rotateX(50.0_degf);
    _camera = new SceneGraph::Camera3D{*_cameraObject};
    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(
        45.0_degf, Vector2{windowSize()}.aspectRatio(), 0.01f, 10000.0f));

    // TRACK MESH
    const auto track_config = toml::find(_config, "Track");
    int n_points = toml::find<int>(track_config, "n_points");
    std::vector<Vec3f> track_points = GameLogic::get().track_points(n_points);
    std::vector<VertexColorData> track_mesh_vertices(2*n_points);
    std::vector<UnsignedInt> track_mesh_indices(6*n_points);

    int finish_line_index = int(n_points * GameLogic::get().finish_line_position() / GameLogic::get().track_length());
    for (int i = 0; i < n_points; i++) {
        track_mesh_vertices[2*i].pos = Vector3{track_points[2*i][0], track_points[2*i][1], track_points[2*i][2]};
        track_mesh_vertices[2*i].color = track_color;

        track_mesh_vertices[2*i+1].pos = Vector3{track_points[2*i+1][0], track_points[2*i+1][1], track_points[2*i+1][2]};
        track_mesh_vertices[2*i+1].color = track_color_up;

        if (i == finish_line_index) {
            track_mesh_vertices[2*i].color = track_color_finish;
            track_mesh_vertices[2*i+1].color = track_color_finish;
        }
        if (i == 0) {
            track_mesh_vertices[2*i].color = 0xff0000_rgbf;
            track_mesh_vertices[2*i+1].color = 0xff0000_rgbf;
        }
        if (i == n_points/2) {
            track_mesh_vertices[2*i].color = 0xff0000_rgbf;
            track_mesh_vertices[2*i+1].color = 0xff0000_rgbf;
        }

        if (i < n_points-1) {
            track_mesh_indices[6*i] = 2*i;
            track_mesh_indices[6*i+1] = 2*i+2;
            track_mesh_indices[6*i+2] = 2*i+1;
            track_mesh_indices[6*i+3] = 2*i+1;
            track_mesh_indices[6*i+4] = 2*i+2;
            track_mesh_indices[6*i+5] = 2*i+3;
        }
        else {
            track_mesh_indices[6*i] = 2*i;
            track_mesh_indices[6*i+1] = 0;
            track_mesh_indices[6*i+2] = 2*i+1;
            track_mesh_indices[6*i+3] = 2*i+1;
            track_mesh_indices[6*i+4] = 0;
            track_mesh_indices[6*i+5] = 1;
        }
    }
    /* Track mesh */
    GL::Buffer vertex_buffer;
    vertex_buffer.setData(track_mesh_vertices);

    GL::Buffer index_buffer;
    index_buffer.setData(track_mesh_indices);

    _track_mesh = GL::Mesh{};
    _track_mesh.setCount(6*n_points)
        .addVertexBuffer(vertex_buffer, 0,
         Shaders::VertexColor3D::Position{},
         Shaders::VertexColor3D::Color3{})
        .setIndexBuffer(index_buffer, 0, GL::MeshIndexType::UnsignedInt, 0, 6*n_points);

    /* Track object */
    auto track = new Object3D{&_scene};
    new VertexColorDrawable{*track, _vertexColorShader, _track_mesh, _drawables};
    
    /* Riders */
    Color3 colors[36]; 
    const auto riders_config = toml::find(_config, "Riders");
    auto rider_map = toml::find<std::unordered_map<std::string , toml::value>>(riders_config, "riders");    auto riders_idx = toml::find<std::unordered_map<std::string , toml::value>>(riders_config, "riders");
    for (auto id: GameLogic::get().rider_ids()) {
         GL::Buffer colorBuffer;
        auto rider_3d = new Object3D(&_scene);
        _rider_objects[id] = rider_3d;
        auto cube = Primitives::cubeSolid();
        _rider_meshes[id] = MeshTools::compile(cube);
        auto color = hex_to_color(GameLogic::get().rider_color(id));
        for (int i = 0; i < 36; i++) {
            colors[i] = color;
        }
        colorBuffer.setData(colors, GL::BufferUsage::StaticDraw);
        _rider_meshes[id].addVertexBuffer(colorBuffer, 0, Shaders::VertexColor3D::Color3{});
        (*rider_3d).scale(Vector3{0.1f});
        new VertexColorDrawable{*rider_3d, _vertexColorShader, _rider_meshes[id], _drawables};
    }
    std::cout << "Done." << std::endl;

}

void TrackApp::receiveInput() {  
    int id, power, angle;
    using namespace std::chrono_literals;
    while (commands_mmap[24*GameLogic::get().n_riders()] != 0x0) {
        std::this_thread::sleep_for(10ms);
    }
    for (int i = 0; i < GameLogic::get().n_riders(); i++)
    {
        std::stringstream ss;
        for (int j = 0; j < 24; j++) {
            ss << commands_mmap[i * 24 + j];
            if (j % 8 == 0)
                ss << " ";

        }
        ss >> id >> power >> angle;
        GameLogic::get().set_power(id, power);
        GameLogic::get().set_steering(id, deg_to_rad(angle));
    }
}

void TrackApp::outputStatus() {
    std::wstring_convert<std::codecvt_utf8<char>, char> utf8conv;
    auto output = utf8conv.to_bytes(GameLogic::get().status());
    size_t pos = 0;
    for (auto& x : status_mmap) {
        if (pos == output.size())
            break;
        x = output[pos++];
    }
 }

 void TrackApp::reset() {
     return;
 }
MAGNUM_APPLICATION_MAIN(TrackApp)
