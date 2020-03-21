#ifndef RIDER_H_
#define RIDER_H_

#include <memory>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>

#include "track.h"

typedef std::map<std::string, float> DescMap;

class RiderPhysics
{
    public:
        virtual double delta_v(double velocity, double gradient, double power, double dt, double slipstream_velocity=0) = 0;
        virtual double current_power(double velocity, double gradient, double acceleration=0, double slipstream_velocity=0) = 0;
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
        double delta_v(double velocity, double gradient, double power, double dt, double slipstream_velocity=0);
        double current_power(double velocity, double gradient, double acceleration=0, double slipstream_velocity=0);
};

class RiderController
{
    public:
        virtual void update() = 0;
        virtual double power() = 0;
        virtual double steering() = 0;
};

class ConstantPower : public RiderController
{
    double _power;
    public:
        ConstantPower(double power) : _power(power) {};
        void update() {};
        double power() {return _power;}
        double steering() {return 0;}
};

class FifoController : public RiderController
{
    std::ifstream _fifo;
    char _buffer[100];
    char _buffer_index;
    double _power;
    double _steering;
    std::string _line;
    std::stringstream _line_stream;
    char _control;
    float _value;

    bool _done;

    public:
        void update();
        FifoController(const char* fname) : _power(0), _steering(0), _done(false) {
            std::cout << "opening fifo..." << std::endl;
            _fifo.open(fname, std::ios::in);
            std::cout << "done." << std::endl;
        }
        double power() {return _power;}
        double steering() {return _steering;}
};

class Rider
{
    Track* _track;
    RiderPhysics* _rider_physics;

    static size_t _next_id;

    Vec3d _pos_xyz;
    Vec2d _pos_dh;
    Vec3d _direction;
    size_t _id;
    double _dl;

    double _velocity;
    double _max_velocity;
    double _power;
    double _slipstream_velocity; // drafting speed
    double _steering;
    double _gradient;


    public:
        Rider(Track* track, RiderPhysics* rider_physics);
        void update(double dt);
        size_t id() {return _id;};

        const Vec3d& direction_vector() {return _direction;};
        Vec3d velocity_vector() {return _velocity * _direction;};

        const Vec2d& pos_dh() {return _pos_dh;};
        void set_pos_dh(const Vec2d& dh);

        const Vec3d& pos_xyz() {return _pos_xyz;};
        void set_pos_xyz(const Vec3d& xyz);

        double velocity() {return _velocity;};
        void set_velocity(double vel) { _velocity = vel; };

        double max_velocity() {return _max_velocity;};
        void set_max_velocity(double max_vel) { _max_velocity = max_vel; };

        double power() {return _power;};
        void set_power(double p) {_power = p;};

        double slipstream_velocity() {return _slipstream_velocity;};
        void set_slipstream_velocity(double slipstream_velocity) {_slipstream_velocity = slipstream_velocity;};

        double steering() {return _steering;};
        void set_steering(double steering) {_steering= steering;};

        double gradient() {return _gradient;};
        void set_gradient(double gradient) {_gradient= gradient;};

        double dl() {return _dl;}
        double length() {return 2.0;}
        double width() {return 0.5;}

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
