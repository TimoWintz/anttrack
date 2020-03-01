#ifndef TRACK_H_
#define TRACK_H_

#include <array>
#include <vector>
#include <Eigen/Dense>

typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector2d Vec2d;
typedef std::pair<Vec3d, Vec3d> PosVel;

enum class TrackSegment { Straight1, Bank1, Straight2, Bank2 };

class Track
{
    protected:
        double inner_radius;
        double track_length;
        double track_width;
        std::vector<double> track_incline;

        double straight_length;
    public:
        double get_length() { return track_length; };
        double get_width() { return track_width; };

        Track(double inner_radius, double track_length, double track_width, std::vector<double> track_incline);
        double interp_track_incline(double d);
        void coord_xyz_to_dh(const Vec3d& xyz, Vec2d& dh);
        void coord_dh_to_xyz(const Vec2d& dh, Vec3d& xyz);
        void direction_vector(const Vec2d& dh, Vec3d& direction);
        double velocity_scaling(const Vec2d& dh);

        TrackSegment track_segment(const Vec3d& xyz);
        TrackSegment track_segment(const Vec2d& dh);

        void update_position(Vec2d& dh, double dl, double angle);
};

std::vector<double> sine_track_incline(double min_incline, double max_incline, int n_points);

#endif
