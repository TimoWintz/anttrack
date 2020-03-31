#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <toml.hpp> // that's all! now you can use it.
#include <stdlib.h>     /* strtol */
#include <fstream>

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
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Trade/MeshData3D.h>


#include "track.h"
#include "rider.h"
#include "utils.h"
#include "race.h"

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

class Rider3D : public Object3D {
    Rider* _rider;
    std::string _window_title;
    std::stringstream _status;

    public:
        Rider3D(Scene3D* scene, Rider* rider) : Object3D{scene}, _rider(rider) {
                std::stringstream ss;
                ss << "Rider " << rider->id();
                _window_title = ss.str();
            };
        void update() {
            auto pos_xyz = _rider->pos_xyz();
            auto transformation = Math::Matrix4<float>::translation({float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2]) + 1}) *
                    Math::Matrix4<float>::scaling({0.5f, 0.5f, 0.5f});
            setTransformation(transformation);

            DescMap map = _rider->desc();
            _status.str("");
            for (auto x: map) {
                _status << x.first << ": \n\t" << x.second << std::endl;
            }
        }

        void draw_gui() {
            ImGui::Begin(_window_title.c_str());
            ImGui::Text("%s", _status.str().c_str());
            ImGui::End();
        }

        Rider* rider() {
            return _rider;
        }
};

class TrackApp: public Platform::Application {
    public:
        explicit TrackApp(const Arguments& arguments);

    private:
        Rider* riderFactory();
        void loadTrack();
        void loadRiders();
        void loadRace();

        void updateRiders(double dt);
        void drawEvent() override;

        void reset();
        void receiveInput();

        Timeline timeline;
        double _time_scaling;

        toml::value _config;

        Track* _track;
        std::vector<int> _rider_ids;

        std::map<int, Rider*> _riders;

        RiderInteraction* _rider_interaction;

        Race* _race;

        std::vector<Rider3D*> _riders_3d;

        Shaders::VertexColor3D _vertexColorShader{NoCreate};
        Shaders::Flat3D _flatShader{NoCreate};
        GL::Mesh _track_mesh{NoCreate}, _grid{NoCreate};
        std::map<int, GL::Mesh> _rider_meshes;

        Scene3D _scene;
        SceneGraph::DrawableGroup3D _drawables;

        Object3D* _cameraObject;
        SceneGraph::Camera3D* _camera;

        Vector2i _lastPosition{-1};
        Vector3 _rotationPoint, _translationPoint;

        std::ofstream _out_pipe;
        std::ifstream _in_pipe;

        char _buffer[100];
        char _buffer_index;
        std::string _line;
        std::stringstream _line_stream;

        void outputStatus(int rider_id);

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
    _config = toml::parse("config.toml");

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

    /* Grid */
    _grid = MeshTools::compile(Primitives::grid3DWireframe({15, 15}));
    auto grid = new Object3D{&_scene};
    (*grid)
        .scale(Vector3{100.0f});
    new FlatDrawable{*grid, _flatShader, _grid, _drawables};

    loadTrack();
    loadRiders();

    /* Set up the camera */
    _cameraObject = new Object3D{&_scene};
    (*_cameraObject)
        .translate(Vector3::zAxis(200.0f))
        .rotateX(50.0_degf);
    _camera = new SceneGraph::Camera3D{*_cameraObject};
    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(
        45.0_degf, Vector2{windowSize()}.aspectRatio(), 0.01f, 10000.0f));

    GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add,
                            GL::Renderer::BlendEquation::Add);
    GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                            GL::Renderer::BlendFunction::OneMinusSourceAlpha);
    loadRace();

    timeline.start();
    _out_pipe.open("status", std::ios::out);
    _in_pipe.open("commands", std::ios::in);
    _buffer_index = 0;

    _autostart = toml::find<bool>(_config.at("General"), "autostart");
    _time_scaling = 5.0;

    if (_autostart) {
        _race->start();
    }
}


void TrackApp::drawEvent() {
    receiveInput();
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);
    double dt = _time_scaling*double(timeline.previousFrameDuration());
    updateRiders(dt);
    for (auto r : _riders_3d) {
        r->update();
    }
    _camera->draw(_drawables);

    swapBuffers();
    timeline.nextFrame();
    redraw();
}

void TrackApp::loadTrack() {
    const auto track_config = toml::find(_config, "Track");
    double min_angle = toml::find<double>(track_config, "min_angle");
    double max_angle = toml::find<double>(track_config, "max_angle");
    double track_width = toml::find<double>(track_config, "track_width");
    double inner_radius = toml::find<double>(track_config, "inner_radius");
    double track_length = toml::find<double>(track_config, "track_length");
    double finish_line_position = toml::find<double>(track_config, "finish_line_position");
    size_t n_points = toml::find<size_t>(track_config, "n_points");

    std::vector<double> track_incline = sine_track_incline(deg_to_rad(min_angle), deg_to_rad(max_angle), n_points); 
    _track = new Track(inner_radius, track_length, track_width, track_incline, finish_line_position);

    std::vector<VertexColorData> track_mesh_vertices(2*n_points);
    std::vector<UnsignedInt> track_mesh_indices(6*n_points);
    Vec2d dh;
    Vec3d xyz;
    bool flag_finish = false;
    for (size_t i = 0; i < n_points; i++) {
        dh[0] = double(i) / n_points * track_length;

        dh[1] = 0;
        _track->coord_dh_to_xyz(dh, xyz);
        track_mesh_vertices[2*i].pos = Vector3{float(xyz[0]), float(xyz[1]), float(xyz[2])};
        track_mesh_vertices[2*i].color = track_color;

        dh[1] = float(track_width);
        _track->coord_dh_to_xyz(dh, xyz);
        track_mesh_vertices[2*i+1].pos = Vector3{float(xyz[0]), float(xyz[1]), float(xyz[2])};
        track_mesh_vertices[2*i+1].color = track_color_up;
        if (!flag_finish && dh[0] > finish_line_position) {
            track_mesh_vertices[2*i].color = track_color_finish;
            track_mesh_vertices[2*i+1].color = track_color_finish;
            flag_finish = true;
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
}

void TrackApp::loadRiders() {

    const auto drafting_model_config = toml::find(_config, "DraftingModel");
    const auto drafting_model_type = toml::find<std::string>(drafting_model_config, "type");
    DraftingModel* drafting_model;
    if (drafting_model_type.compare("simple_cone_drafting") == 0) {
        auto slipstream_duration = toml::find<double>(drafting_model_config, "slipstream_duration");
        auto slipstream_angle = toml::find<double>(drafting_model_config, "slipstream_angle");
        drafting_model = new SimpleConeDrafting(slipstream_duration, deg_to_rad(slipstream_angle));
    }
    else {
        throw std::runtime_error("Unknown drafting type");
    }
    _rider_interaction = new RiderInteraction(drafting_model);

    const auto riders_config = toml::find(_config, "Riders");
    auto riders_idx = toml::find<std::unordered_map<std::string , toml::value>>(riders_config, "riders");
    _rider_ids = {};
    int id  = 0;
    double CdA, Cxx, mass, air_density;
    air_density = toml::find<double>(_config.at("Physics"), "air_density");
    Rider3D* rider_3d;
    Color3 colors[36]; 
    for (auto &x: riders_idx) {
        CdA = toml::find<double>(x.second, "CdA");
        Cxx = toml::find<double>(x.second, "Cxx");
        mass = toml::find<double>(x.second, "mass");
        _rider_ids.push_back(id);
        _riders[id] = new Rider(_track, new SimpleRiderPhysics(CdA, Cxx, air_density, mass));
        auto cube = Primitives::cubeSolid();
        _rider_meshes[id] = MeshTools::compile(cube);
        auto color = hex_to_color(toml::find<long>(x.second, "color"));
        for (int i = 0; i < 36; i++) {
            colors[i] = color;
        }
        GL::Buffer colorBuffer;
        colorBuffer.setData(colors, GL::BufferUsage::StaticDraw);
        _rider_meshes[id].addVertexBuffer(colorBuffer, 0, Shaders::VertexColor3D::Color3{});
        _rider_interaction->add_rider(_riders[id]);
        _riders[id]->set_pos_dh({3*id, 3});

        rider_3d = new Rider3D(&_scene, _riders[id]);
        (*rider_3d).scale(Vector3{0.1f});
        (*rider_3d).update();
        _riders_3d.push_back(rider_3d);
        new VertexColorDrawable{*rider_3d, _vertexColorShader, _rider_meshes[id], _drawables};
        id++;
    }
}

void TrackApp::loadRace() {
    std::vector<Rider*> riders_vec;
    for (auto x : _riders) {
        riders_vec.push_back(x.second);
    }
    _race = new Sprint(riders_vec, _track, 4);
    _race->position_riders();
}

void TrackApp::updateRiders(double dt) {
    if (_race->lock_riders()) {
        return;
    }
    for (auto id : _rider_ids) {
        _riders[id]->update(dt);
    }
    _rider_interaction->update();
    _race->update(dt);
}

void TrackApp::reset() {
    _race->position_riders();
}

void TrackApp::receiveInput() {
    std::string control;
    int rider_id;
    float value;
    int n = _in_pipe.readsome(_buffer + _buffer_index, 100-_buffer_index);
    for (int i = _buffer_index; i < _buffer_index + n; i++) {
        if (_buffer[i] == '\n') {
            _line_stream >> control >> rider_id >> value;
            _line_stream.clear();
            if (control.compare("reset") == 0) {
                reset();
            } 
            else if (control.compare("power") == 0) {
                _riders.at(rider_id)->set_power(value);
            }
            else if (control.compare("steering") == 0) {
                _riders.at(rider_id)->set_steering(deg_to_rad(value));
            }
            else if (control.compare("status") == 0) {
                outputStatus(rider_id);
            }
        }
        _line_stream << _buffer[i];
    }
    _buffer_index += n;
    if (_buffer_index >= 100) {
        _buffer_index = 0;
    }
}

void TrackApp::outputStatus(int rider_id) {
    _out_pipe << "time " << float(_time_scaling) * timeline.previousFrameTime() 
              << " velocity " << _riders[rider_id]->velocity()
              << " position " << _riders[rider_id]->pos_dh()[0]
              << " height " << _riders[rider_id]->pos_dh()[1]
              << " distance_to_finish " << _race->distance_to_finish()[rider_id];
    for (auto id : _rider_ids) {
        if (id == rider_id)
            continue;
        _out_pipe << " distance_to_rider_" << id << " " << _race->cumulative_distance()[id] - _race->cumulative_distance()[rider_id];
        _out_pipe << " relative_height_" << id << " " << _riders[id]->pos_dh()[1] - _riders[rider_id]->pos_dh()[1];
    }
    _out_pipe << " finished " << _race->is_finished()
              << " win " << (_race->rankings()[0] == rider_id) << std::endl; 
}

MAGNUM_APPLICATION_MAIN(TrackApp)
