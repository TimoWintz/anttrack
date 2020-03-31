#ifndef RIDER_H_
#define RIDER_H_

#include <memory>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>

#include "track.h"

class RiderPhysics
{
    float CdA; // Air drag coefficient
    float Cxx; // Rolling drag coefficient
    float rho; // Air density
    float mass; // Cyclist mass

    public:
        RiderPhysics() {};
        RiderPhysics(float CdA, float Cxx, float rho, float mass) : CdA(CdA), Cxx(Cxx), rho(rho), mass(mass) {};

        float moving_resistance(float velocity) const;
        float gravity_acceleration(float gradient) const;
        float delta_v(float velocity, float gradient, float power, float dt, float slipstream_velocity=0) const;
        float current_power(float velocity, float gradient, float acceleration=0, float slipstream_velocity=0) const;
};

class RiderController
{
    std::ifstream _fifo;
    char _buffer[100];
    char _buffer_index;
    float _power;
    float _steering;
    std::string _line;
    std::stringstream _line_stream;
    char _control;
    float _value;

    bool _done;

    public:
        void update();
        RiderController(const char* fname) : _power(0), _steering(0), _done(false) {
            std::cout << "opening fifo..." << std::endl;
            _fifo.open(fname, std::ios::in);
            std::cout << "done." << std::endl;
        }
        float power() {return _power;}
        float steering() {return _steering;}
};


class Rider {
    float _velocity;
    Vec2f _position;
    public:
        Rider() : _velocity(0), _position({0,0}) {};
        Vec2f position() const {return _position;};
        float position(int i) const {return _position[i];};
        void set_position(const Vec2f& pos) {_position[0] = pos[0]; _position[1] = pos[1];};
        float velocity() const {return _velocity;};
        void set_velocity(float vel) {_velocity = vel;};
        void update_velocity(const RiderPhysics& physics, float dt, float power, float gradient, float drafting_coefficient);
};

class DraftingModel {
    float _duration;
    float _angle;
    float _max_multiplier;
    public:
        DraftingModel(float duration=0, float angle=0, float max_multiplier=0.5) :
            _duration(duration), _angle(angle), _max_multiplier(max_multiplier) {};
        std::map<int, float> drafting_coefficients(const std::map<int, Rider>& riders);
        void set_duration(float duration) {
            _duration = duration;
        }
        void set_angle(float angle) {
            _angle = angle;
        }
        void set_max_multiplier(float max_multiplier) {
            _max_multiplier = max_multiplier;
        }
};

#endif
