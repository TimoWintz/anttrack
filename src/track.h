#ifndef TRACK_H_
#define TRACK_H_

#include <array>
#include <vector>
#include <Eigen/Dense>

#include "utils.h"

enum class TrackSegment { Straight1, Bank1, Straight2, Bank2 };

class Track
{
    protected:
        float _inner_radius;
        float _length;
        float _width;
        std::vector<float> _incline;

        float _straight_length;
        float _finish_line_position;
    public:
        float length() const { return _length; };
        float width() const { return _width; };
        float finish_line_position() const {return _finish_line_position;} 

        Track() { };
        void configure(float inner_radius, float track_length, float track_width, std::vector<float> track_incline, float finish_line_position);
        
        float interpolate_incline(float d) const;
        void coord_xyz_to_dh(const Vec3f& xyz, Vec2f& dh) const;
        void coord_dh_to_xyz(const Vec2f& dh, Vec3f& xyz) const;
        void direction_vector(const Vec2f& dh, Vec3f& direction) const;
        float velocity_scaling(const Vec2f& dh) const;

        TrackSegment track_segment(const Vec3f& xyz) const;
        TrackSegment track_segment(const Vec2f& dh) const;

        void update_position(Vec2f& dh, float dl, float angle);
};

std::vector<float> sine_track_incline(float min_incline, float max_incline, int n_points);

#endif
