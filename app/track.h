#ifndef TRACK_H_
#define TRACK_H_

#include <array>
#include <vector>
typedef std::array<double, 3> Vec3d;
typedef std::array<double, 2> Vec2d;

enum class TrackSegment { Straight1, Bank1, Straight2, Bank2 };

class Track
{
    protected:
        double inner_radius;
        double track_length;
        double track_width;
        const std::vector<double>& track_incline;

        double straight_length;
    public:
        double get_length() { return track_length; };
        double get_width() { return track_width; };

        Track(double inner_radius, double track_length, double track_width, const std::vector<double>& track_incline);
        double interp_track_incline(double d);
        void coord_xyz_to_dh(const Vec3d& xyz, Vec2d& dh);
        void coord_dh_to_xyz(const Vec2d& dh, Vec3d& xyz);
        double velocity_scaling(const Vec2d& dh);

        TrackSegment track_segment(const Vec3d& xyz);
        TrackSegment track_segment(const Vec2d& dh);

        double update_position(const Vec3d& xyz, Vec3d& new_xyz, double dl, double angle=0.0);
        double update_position(Vec3d& xyz, double dl, double angle=0.0);
};

std::vector<double> sine_track_incline(double min_incline, double max_incline, int n_points);

#endif
