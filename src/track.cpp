#include <cassert>
#include <iostream>

#include "track.h"
#include "utils.h"

void Track::configure(float inner_radius, float length, float width, std::vector<float> incline, float finish_line_position) {
    _inner_radius = inner_radius;
    _length = length;
    _width = width;
    _incline = incline;
    _finish_line_position = finish_line_position;
    _straight_length = 0.5*(length - 2*pi*inner_radius);
    assert(_straight_length >= 0);
}

TrackSegment Track::track_segment(const Vec3f& xyz) const {
    auto x = xyz[0];
    auto y = xyz[1];
    if (x > _straight_length/2) {
        return TrackSegment::Bank1;
    } 
    else if (x >= -_straight_length/2) {
        if (y < 0) {
            return TrackSegment::Straight1;
        }
        else {
            return TrackSegment::Straight2;
        }
    }
    else {
        return TrackSegment::Bank2;
    }
}

TrackSegment Track::track_segment(const Vec2f& dh) const {
    auto d = dh[0];
    if (d <= _straight_length/2) {
        return TrackSegment::Straight1;
    } 
    else if (d < _inner_radius * pi + _straight_length/2) {
        return TrackSegment::Bank1;
    }
    else if (d <= _inner_radius * pi + 1.5*_straight_length) {
        return TrackSegment::Straight2;
    }
    else if (d < 2 * _inner_radius * pi + 1.5*_straight_length) {
        return TrackSegment::Bank2;
    }
    else {
        return TrackSegment::Straight1;
    }
}

void Track::coord_xyz_to_dh(const Vec3f& xyz, Vec2f& res) const {
    float theta;
    float incline;
    auto x = xyz[0];
    auto y = xyz[1];
    switch ( track_segment(xyz) ) {
        case TrackSegment::Straight1:
            if (x >= 0)
                res[0] = x;
            else
                res[0] = _length + x;
            incline = interpolate_incline(res[0]);
            res[1] = (- y - _inner_radius) / std::cos(incline);
            break;
        case TrackSegment::Straight2:
            res[0] = 0.5*_straight_length + pi * _inner_radius + (0.5*_straight_length - x);
            incline = interpolate_incline(res[0]);
            res[1] = (y - _inner_radius) / std::cos(incline);
            break;
        case TrackSegment::Bank1:
            theta = std::atan(y / (x - _straight_length/2)) + pi / 2;
            res[0] = 0.5*_straight_length + _inner_radius*theta;
            incline = interpolate_incline(res[0]);
            res[1] = std::sqrt(std::pow(y, 2) + std::pow(x - _straight_length/2, 2)) - _inner_radius;
            res[1] /= std::cos(incline);
            break;
        case TrackSegment::Bank2:
            theta = std::atan(y / (x + _straight_length/2)) + pi / 2;
            res[0] = 1.5*_straight_length + _inner_radius*(pi + theta);
            incline = interpolate_incline(res[0]);
            res[1] = std::sqrt(std::pow(y, 2) + std::pow(x + _straight_length/2, 2)) - _inner_radius;
            res[1] /= std::cos(incline);
            break;
        default:
            break;
    }
}

void Track::coord_dh_to_xyz(const Vec2f& dh, Vec3f& xyz) const {
    float incline = interpolate_incline(dh[0]);
    float theta;
    xyz[2] = std::sin(incline)*dh[1];
    switch ( track_segment(dh) ) {
        case TrackSegment::Straight1:
            if (dh[0] < _straight_length/2)
                xyz[0] = dh[0];
            else
                xyz[0] = dh[0] - _length;
            xyz[1] = -_inner_radius - dh[1] * std::cos(incline);
            break;
        case TrackSegment::Straight2:
            xyz[0] = -(dh[0] - pi * _inner_radius - _straight_length);
            xyz[1] = dh[1] * std::cos(incline) + _inner_radius;
            break;
        case TrackSegment::Bank1:
            theta = (dh[0] - _straight_length / 2) / _inner_radius;
            xyz[0] = _straight_length/2 + std::sin(theta) * (_inner_radius + dh[1] * std::cos(incline));
            xyz[1] = -std::cos(theta) * (_inner_radius + dh[1] * std::cos(incline));
            break;
        case TrackSegment::Bank2:
            theta = (dh[0] - 1.5 * _straight_length - _inner_radius * pi) / _inner_radius;
            xyz[0] = -_straight_length/2 - std::sin(theta) * (_inner_radius + dh[1] * std::cos(incline));
            xyz[1] = std::cos(theta) * (_inner_radius + dh[1] * std::cos(incline));
            break;
        default:
            break;
    }
}

float Track::velocity_scaling(const Vec2f& dh) const {
    Vec3f xyz;
    coord_dh_to_xyz(dh, xyz);
    float x = xyz[0];
    float y = xyz[1];
    float radius;
    switch ( track_segment(dh) ) {
        case TrackSegment::Straight1:
        case TrackSegment::Straight2:
            return 1.0;
        case TrackSegment::Bank1:
            radius = std::sqrt(std::pow(y, 2) + std::pow(x - _straight_length/2, 2));
            return radius/_inner_radius;
        default:
            radius = std::sqrt(std::pow(y, 2) + std::pow(x + _straight_length/2, 2));
            return radius/_inner_radius;
    }
}

float Track::interpolate_incline(float d) const {
    while(d >= _length)
        d -= _length;
    float id = d / _length * _incline.size();
    size_t ib = size_t(id);
    size_t it = ib + 1;
    if (it == _incline.size())
        it = 0;
    float k = id - ib;
    while (k > 1)
        k -= 1;
    while (k < 0)
        k += 1;
    return (1-k) * _incline[ib] + k * _incline[it]; //[o] of incline
}


void Track::update_position(Vec2f& dh, float dl, float angle) {
    dh[0] += 1.0 / velocity_scaling(dh) * dl * std::cos(angle);
    dh[1] += dl * std::sin(angle);
    if (dh[1] < 0)
        dh[1] = 0;
    if (dh[1] > _width)
        dh[1] = _width;
    while (dh[0] > _length)
        dh[0] -= _length;
}

std::vector<float> sine_track_incline(float min_incline, float max_incline, int n_points) {
    std::vector<float> res(n_points);
    for (int i = 0; i < n_points; i++) {
        res[i] = min_incline + 0.5 * (1 - std::cos(4 * pi * float(i) / n_points)) * (max_incline - min_incline);
    }
    return res;
}

void Track::direction_vector(const Vec2f& dh, Vec3f& direction) const {
    // TODO : back to 3D velocity vector..
    // float incline = interp_track_incline(dh[0] + 1) - interp_track_incline(dh[0]);
    // float d_incline = interp_track_incline(dh[0] + 1) - interp_track_incline(dh[0]);
    float theta, d_theta;
    // direction[2] = std::cos(d_incline)*dh[1];
    direction[2] = 0.0;
    switch ( track_segment(dh) ) {
        case TrackSegment::Straight1:
            direction[0] = 1;
            // direction[1] = dh[1] * std::sin(d_incline);
            direction[1] = 0;
            break;
        case TrackSegment::Straight2:
            direction[0] = -1;
            direction[1] = 0;//-dh[1] * std::sin(d_incline);
            break;
        case TrackSegment::Bank1:
            theta = (dh[0] - _straight_length / 2) / _inner_radius;
            d_theta = 1 / _inner_radius;
            direction[0] = std::cos(theta); //* (-dh[1] * std::cos(d_incline));
            direction[1] = std::sin(theta); //* (-dh[1] * std::cos(d_incline));
            break;
        case TrackSegment::Bank2:
            theta = (dh[0] - 1.5 * _straight_length - _inner_radius * pi) / _inner_radius;
            d_theta = 1 / _inner_radius;
            direction[0] = - std::cos(theta); //* (_inner_radius + dh[1] * std::cos(incline)) - std::sin(theta) * (-dh[1] * std::sin(d_incline));
            direction[1] = - std::sin(theta); //* (inner_radius + dh[1] * std::cos(incline)) + std::cos(theta) * (-dh[1] * std::sin(d_incline));
            break;
        default:
            break;
        direction /= direction.norm();
    }
    float norm = 0;
    for (size_t i = 0; i < 3; i++)
        norm += direction[i] * direction[i];
    norm = std::sqrt(norm);
    for (size_t i = 0; i < 3; i++)
        direction[i] /= norm;
}
