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
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/VertexColor.h>
#include <Magnum/Trade/MeshData3D.h>

#include "track.h"
#include "constants.h"

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


class TrackApp: public Platform::Application {
    public:
        explicit TrackApp(const Arguments& arguments);

    private:
        void loadTrack();
        void drawEvent() override;
        std::unique_ptr<Track> _track;

        Shaders::VertexColor3D _vertexColorShader{NoCreate};
        Shaders::Flat3D _flatShader{NoCreate};
        GL::Mesh _mesh{NoCreate}, _grid{NoCreate};

        Scene3D _scene;
        SceneGraph::DrawableGroup3D _drawables;
        Object3D* _cameraObject;
        SceneGraph::Camera3D* _camera;

        Vector2i _lastPosition{-1};
        Vector3 _rotationPoint, _translationPoint;
};

class VertexColorDrawable: public SceneGraph::Drawable3D {
    public:
        explicit VertexColorDrawable(Object3D& object, Shaders::VertexColor3D& shader, GL::Mesh& mesh, SceneGraph::DrawableGroup3D& drawables): SceneGraph::Drawable3D{object, &drawables}, _shader(shader), _mesh(mesh) {}

        void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
            _shader.setTransformationProjectionMatrix(camera.projectionMatrix()*transformation);
            _mesh.draw(_shader);
        }

    private:
        Shaders::VertexColor3D& _shader;
        GL::Mesh& _mesh;
};

class FlatDrawable: public SceneGraph::Drawable3D {
    public:
        explicit FlatDrawable(Object3D& object, Shaders::Flat3D& shader, GL::Mesh& mesh, SceneGraph::DrawableGroup3D& drawables): SceneGraph::Drawable3D{object, &drawables}, _shader(shader), _mesh(mesh) {}

        void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
            _shader.setColor(0x747474_rgbf)
                .setTransformationProjectionMatrix(camera.projectionMatrix()*transformation);
            _mesh.draw(_shader);
        }

    private:
        Shaders::Flat3D& _shader;
        GL::Mesh& _mesh;
};

TrackApp::TrackApp(const Arguments& arguments): Platform::Application{arguments, NoCreate} {
    /* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
       MSAA if we have enough DPI. */
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("Track")
            .setSize(conf.size(), dpiScaling);
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

    /* Set up the camera */
    _cameraObject = new Object3D{&_scene};
    (*_cameraObject)
        .translate(Vector3::zAxis(200.0f))
        // .translate(Vector3::yAxis(-20.0f))
        .rotateX(50.0_degf);
        // .rotateZ(30.0_degf);
    _camera = new SceneGraph::Camera3D{*_cameraObject};
    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(
        45.0_degf, Vector2{windowSize()}.aspectRatio(), 0.01f, 10000.0f));
}


void TrackApp::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);

    _camera->draw(_drawables);

    swapBuffers();
}

void TrackApp::loadTrack() {
    double min_angle = 12 * constants::pi / 180.0; //12 deg
    double max_angle = 45 * constants::pi / 180.0; //45 deg
    double track_width = 8; //m
    double inner_radius = 27; //m
    double track_length = 250; //m

    size_t n_points = 100;
    std::vector<double> track_incline = sine_track_incline(min_angle, max_angle, n_points); 
    _track = std::unique_ptr<Track>(new Track(inner_radius, track_length, track_width, track_incline));

    std::vector<VertexColorData> track_mesh_vertices(2*n_points);
    std::vector<UnsignedByte> track_mesh_indices(6*n_points);
    Vec2d dh;
    Vec3d xyz;
    for (size_t i = 0; i < n_points; i++) {
        dh[0] = double(i) / n_points * track_length;

        dh[1] = double(0);
        _track->coord_dh_to_xyz(dh, xyz);
        track_mesh_vertices[2*i].pos = Vector3{float(xyz[0]), float(xyz[1]), float(xyz[2])};
        track_mesh_vertices[2*i].color = track_color;

        dh[1] = double(track_width);
        _track->coord_dh_to_xyz(dh, xyz);
        track_mesh_vertices[2*i+1].pos = Vector3{float(xyz[0]), float(xyz[1]), float(xyz[2])};
        track_mesh_vertices[2*i+1].color = track_color_up;

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

    _mesh = GL::Mesh{};
    _mesh.setCount(6*n_points)
        .addVertexBuffer(vertex_buffer, 0,
         Shaders::VertexColor3D::Position{},
         Shaders::VertexColor3D::Color3{})
        .setIndexBuffer(index_buffer, 0, GL::MeshIndexType::UnsignedByte, 0, 6*n_points);

    /* Track object */
    auto track = new Object3D{&_scene};
    new VertexColorDrawable{*track, _vertexColorShader, _mesh, _drawables};
}



MAGNUM_APPLICATION_MAIN(TrackApp)
