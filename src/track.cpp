#include <cassert>
#include <iostream>

#include "track.h"
#include "constants.h"

Track::Track(double inner_radius, double track_length, double track_width, const std::vector<double>& track_incline) :
        inner_radius(inner_radius), track_length(track_length), track_width(track_width), track_incline(track_incline) {
    straight_length= 0.5*(track_length - 2*constants::pi*inner_radius);
    assert(straight_length >= 0);
}

TrackSegment Track::track_segment(const Vec3d& xyz) {
    auto x = xyz[0];
    auto y = xyz[1];
    if (x > straight_length/2) {
        return TrackSegment::Bank1;
    } 
    else if (x >= -straight_length/2) {
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

TrackSegment Track::track_segment(const Vec2d& dh) {
    auto d = dh[0];
    if (d <= straight_length/2) {
        return TrackSegment::Straight1;
    } 
    else if (d < inner_radius * constants::pi + straight_length/2) {
        return TrackSegment::Bank1;
    }
    else if (d <= inner_radius * constants::pi + 1.5*straight_length) {
        return TrackSegment::Straight2;
    }
    else if (d < 2 * inner_radius * constants::pi + 1.5*straight_length) {
        return TrackSegment::Bank2;
    }
    else {
        return TrackSegment::Straight1;
    }
}

void Track::coord_xyz_to_dh(const Vec3d& xyz, Vec2d& res) {
    double theta;
    double incline;
    auto x = xyz[0];
    auto y = xyz[1];
    switch ( track_segment(xyz) ) {
        case TrackSegment::Straight1:
            if (x >= 0)
                res[0] = x;
            else
                res[0] = track_length + x;
            incline = interp_track_incline(res[0]);
            res[1] = (- y - inner_radius) / std::cos(incline);
            break;
        case TrackSegment::Straight2:
            res[0] = 0.5*straight_length + constants::pi * inner_radius + (0.5*straight_length - x);
            incline = interp_track_incline(res[0]);
            res[1] = (y - inner_radius) / std::cos(incline);
            break;
        case TrackSegment::Bank1:
            theta = std::atan(y / (x - straight_length/2)) + constants::pi / 2;
            res[0] = 0.5*straight_length + inner_radius*theta;
            incline = interp_track_incline(res[0]);
            res[1] = std::sqrt(std::pow(y, 2) + std::pow(x - straight_length/2, 2)) - inner_radius;
            res[1] /= std::cos(incline);
            break;
        case TrackSegment::Bank2:
            theta = std::atan(y / (x + straight_length/2)) + constants::pi / 2;
            res[0] = 1.5*straight_length + inner_radius*(constants::pi + theta);
            incline = interp_track_incline(res[0]);
            res[1] = std::sqrt(std::pow(y, 2) + std::pow(x + straight_length/2, 2)) - inner_radius;
            res[1] /= std::cos(incline);
            break;
        default:
            break;
    }
}

void Track::coord_dh_to_xyz(const Vec2d& dh, Vec3d& xyz) {
    double incline = interp_track_incline(dh[0]);
    double theta;
    xyz[2] = std::sin(incline)*dh[1];
    switch ( track_segment(dh) ) {
        case TrackSegment::Straight1:
            if (dh[0] < straight_length/2)
                xyz[0] = dh[0];
            else
                xyz[0] = dh[0] - track_length;
            xyz[1] = -inner_radius - dh[1] * std::cos(incline);
            break;
        case TrackSegment::Straight2:
            xyz[0] = -(dh[0] - constants::pi * inner_radius - straight_length);
            xyz[1] = dh[1] * std::cos(incline) + inner_radius;
            break;
        case TrackSegment::Bank1:
            theta = (dh[0] - straight_length / 2) / inner_radius;
            xyz[0] = straight_length/2 + std::sin(theta) * (inner_radius + dh[1] * std::cos(incline));
            xyz[1] = -std::cos(theta) * (inner_radius + dh[1] * std::cos(incline));
            break;
        case TrackSegment::Bank2:
            theta = (dh[0] - 1.5 * straight_length - inner_radius * constants::pi) / inner_radius;
            xyz[0] = -straight_length/2 - std::sin(theta) * (inner_radius + dh[1] * std::cos(incline));
            xyz[1] = std::cos(theta) * (inner_radius + dh[1] * std::cos(incline));
            break;
        default:
            break;
    }
}

double Track::velocity_scaling(const Vec2d& dh) {
    Vec3d xyz;
    coord_dh_to_xyz(dh, xyz);
    double x = xyz[0];
    double y = xyz[1];
    double radius;
    switch ( track_segment(dh) ) {
        case TrackSegment::Straight1:
        case TrackSegment::Straight2:
            return 1.0;
        case TrackSegment::Bank1:
            radius = std::sqrt(std::pow(y, 2) + std::pow(x - straight_length/2, 2));
            return radius/inner_radius;
        default:
            radius = std::sqrt(std::pow(y, 2) + std::pow(x + straight_length/2, 2));
            return radius/inner_radius;
    }
}

double Track::interp_track_incline(double d) {
    while(d >= track_length)
        d -= track_length;
    double id = d / track_length * track_incline.size();
    size_t ib = size_t(id);
    size_t it = ib + 1;
    if (it == track_incline.size())
        it = 0;
    double k = id - ib;
    return (1-k) * track_incline[ib] + k * track_incline[it]; //[o] of incline
}


static Vec3d tmp_xyz;
static Vec2d tmp_dh;

double Track::update_position(const Vec3d& xyz, Vec3d& new_xyz, double dl, double angle) {
    coord_xyz_to_dh(xyz, tmp_dh);
    tmp_dh[0] += velocity_scaling(tmp_dh) * dl * std::cos(angle);
    tmp_dh[1] += dl * std::sin(angle);

    if (tmp_dh[1] > track_width)
        tmp_dh[1] -= track_width;
    else if (tmp_dh[1] < 0)
        tmp_dh[1] += 0;

    coord_dh_to_xyz(tmp_dh, new_xyz);
    return (new_xyz[2] - xyz[2]) / dl;
}

double Track::update_position(Vec3d& xyz, double dl, double angle) {
    double x = update_position(xyz, tmp_xyz, dl, angle);
    xyz[0] = tmp_xyz[0];
    xyz[1] = tmp_xyz[1];
    xyz[2] = tmp_xyz[2];
    return x;
}

std::vector<double> sine_track_incline(double min_incline, double max_incline, int n_points) {
    std::vector<double> res(n_points);
    for (int i = 0; i < n_points; i++) {
        res[i] = min_incline + 0.5 * (1 - std::cos(4 * constants::pi * double(i) / n_points)) * (max_incline - min_incline);
    }
    return res;
}
