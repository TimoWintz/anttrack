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

#include "utils.h"

int pi = (float) std::atan(1)*4;

inline static float deg_to_rad(float angle_deg) {
    return angle_deg / 180.0f * pi;
}
static Magnum::Color3 hexToColor(long x) {
    float red = float((x / 256 / 256) % 256) / 256;
    float green = float((x / 256) % 256) / 256;
    float blue = float(x % 256) / 256;
    return Magnum::Color3{red, green, blue};
}

inline void allocateFile(char* fname, int page_size) {
    // Fill buffer, then write it to file.
    const int file_size = 4 * page_size - 250; // 16134, if page size is 4KiB
    std::string buffer(file_size, 0);
    std::ofstream file(fname);
    file << buffer;
    file.close();
}

#endif
