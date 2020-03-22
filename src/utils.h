#ifndef UTILS_H
#define UTILS_H
#include <cmath>
#include <Magnum/Shaders/VertexColor.h>
const double pi = std::atan(1)*4;
static double deg_to_rad(double angle_deg) {
    return angle_deg / 180.0 * pi;
}
static Magnum::Color3 hex_to_color(long x) {
    float red = float((x / 256 / 256) % 256) / 256;
    float green = float((x / 256) % 256) / 256;
    float blue = float(x % 256) / 256;
    return Magnum::Color3{red, green, blue};
}
#endif
