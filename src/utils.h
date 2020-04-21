#ifndef UTILS_H
#define UTILS_H
#include <cmath>
#include <iostream>
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

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector2f Vec2f;

static const float pi;
inline static float deg_to_rad(float angle_deg);
static Magnum::Color3 hex_to_color(long x);
inline void allocate_file(char* fname, int page_size);

#endif