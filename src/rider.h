#ifndef RIDER_H_
#define RIDER_H_

#include <memory>
#include <map>
#include <vector>

#include "track.h"

typedef std::map<std::string, float> DescMap;

class RiderPhysics
{
    public:
        virtual double acceleration(double velocity, double gradient, double power, double slipstream_velocity=0) = 0;
};

class SimpleRiderPhysics : public RiderPhysics
{
    double CdA; // Air drag coefficient
    double Cxx; // Rolling drag coefficient
    double rho; // Air density
    double mass; // Cyclist mass

    public:
        SimpleRiderPhysics(double CdA, double Cxx, double rho, double mass) : CdA(CdA), Cxx(Cxx), rho(rho), mass(mass) {};

        double moving_resistance(double velocity);
        double gravity_acceleration(double gradient);
        double acceleration(double velocity, double gradient, double power, double slipstream_velocity=0);
};

class RiderController
{
    public:
        virtual double power() = 0;
        virtual double steering() = 0;
};

class ConstantPower : public RiderController
{
    double _power;
    public:
        ConstantPower(double power) : _power(power) {};
        double power() {return _power;}
        double steering() {return 0;}
};


class RiderModel 
{
    public:
        virtual double maximum_power(double dt) = 0;
        virtual void update(double P, double dt) = 0;
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
    Track* _track;
    RiderPhysics* _rider_physics;
    RiderModel* _rider_model;
    RiderController* _rider_controller;

    static size_t _next_id;

    Vec3d _pos_xyz;
    Vec2d _pos_dh;
    Vec3d _direction;
    size_t _id;

    double _velocity;
    double _power;
    double _slipstream_velocity; // drafting speed


    public:
        Rider(Track* track, RiderPhysics* rider_physics, RiderModel* rider_model, RiderController* rider_controller);
        void update(double dt);
        size_t id() {return _id;};

        const Vec3d& pos_xyz() {return _pos_xyz;};
        const Vec2d& pos_dh() {return _pos_dh;};
        const Vec3d& direction_vector() {return _direction;};
        Vec3d velocity_vector() {return _velocity * _direction;};
        double velocity() {return _velocity;};
        void set_pos_dh(const Vec2d& dh);
        void set_pos_xyz(const Vec3d& xyz);
        void set_velocity(double vel) { _velocity = vel; };
        void set_slipstream_velocity(double slipstream_velocity) {_slipstream_velocity = slipstream_velocity;};

        DescMap desc();
};

class DraftingModel
{
    public:
            virtual double slipstream_velocity(const std::vector<PosVel> positions_and_velocities,
                                           const Vec3d& position) = 0;
};

class SimpleConeDrafting : public DraftingModel {
    double _duration;
    double _angle;
    double _max_multiplier;
    public:
        SimpleConeDrafting(double duration, double angle, double max_multiplier=0.5) :
            _duration(duration), _angle(angle), _max_multiplier(max_multiplier) {};
            double slipstream_velocity(const std::vector<PosVel> positions_and_velocities,
                                   const Vec3d& position) override;
};

class RiderInteraction
{
    DraftingModel* _drafting_model;
    std::vector<Rider*> _riders;
    std::vector<PosVel> _positions_and_velocities;
    public:
        RiderInteraction(DraftingModel* drafting_model) : _drafting_model(drafting_model) {};
        void add_rider(Rider* rider) { _riders.push_back(rider); _positions_and_velocities.push_back(PosVel());};
        void update();
};

#endif
