#ifndef RIDER_H_
#define RIDER_H_

#include <memory>

#include "track.h"

class RiderPhysics
{
    public:
        virtual double acceleration(double velocity, double gradient, double power) = 0;
};

class SimpleRiderPhysics : public RiderPhysics
{
    double CdA; // Air drag coefficient
    double Cxx; // Rolling drag coefficient
    double rho; // Air density
    double mass; // Cyclist mass

    public:
        RiderPhysics(double CdA, double Cxx, double rho, double mass) : CdA(CdA), Cxx(Cxx), rho(rho), mass(mass) {};

        double moving_resistance(double velocity);
        double gravity_acceleration(double gradient);
        double acceleration(double velocity, double gradient, double power);
};

class RiderController
{
    public:
        virtual double power() = 0;
};

class ConstantPower : public RiderController
{
    double _power;
    public:
        ConstantPower(double power) : _power(power) {};
        double power() {return _power;}
};


class RiderModel 
{
    public:
        virtual double maximum_power(double dt) = 0;
        void update(double P, double dt);
};

class WPModel : public RiderModel
{
    double _Wp;
    double _Wpbal;
    double _CP;

    public:
        WPModel(double Wp, double CP) : _Wp(Wp), _Wpbal(Wp), _CP(CP) {};
        double maximum_power(double dt);
        void update(double P, double dt);
};

class Rider
{
    std::shared_ptr<Track> _track;
    std::shared_ptr<RiderPhysics> _rider_physics;
    std::shared_ptr<RiderModel> _rider_model;
    std::shared_ptr<RiderController> _rider_controller;

    Vec3d _pos_xyz;
    Vec2d _pos_dh;

    double _velocity;

    public:
        TrackRider(std::shared_ptr<Track> track, std::shared_ptr<RiderPhysics> rider_physics);
        void update(double dt);
};

#endif
