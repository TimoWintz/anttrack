#include "rider.h"
#include "constants.h"
#include <iostream>

static const double max_acceleration = 5*constants::g;
size_t Rider::_next_id = 0;

double SimpleRiderPhysics::moving_resistance(double velocity) {
    return 0.5 * rho * CdA * velocity * velocity + mass* Cxx * constants::g;
}

double SimpleRiderPhysics::gravity_acceleration(double gradient) {
    return gradient * constants::g;
}
double SimpleRiderPhysics::acceleration(double velocity, double gradient, double power) {
    double acceleration = power / velocity / mass - moving_resistance(velocity) / mass - gravity_acceleration(gradient);
    if (acceleration > max_acceleration)
        return max_acceleration;
    return acceleration;
}  

Rider::Rider(Track* track, RiderPhysics* rider_physics, RiderModel* rider_model, RiderController* rider_controller) : 
    _track(track), _rider_physics(rider_physics), _rider_model(rider_model), _rider_controller(rider_controller) {
        _pos_dh[0] = 0;
        _pos_dh[1] = 0;
        _track->coord_dh_to_xyz(_pos_dh, _pos_xyz);
        _velocity = 0;
        _id = _next_id++;
}

void Rider::update(double dt) {
    double steering = _rider_controller->steering();
    _power = _rider_controller->power();

    double dl = dt * _velocity;
    double gradient = _track->update_position(_pos_xyz, dl, steering);

    double max_power = _rider_model->maximum_power(dt);
    if (_power > max_power)
        _power = max_power;

    _rider_model->update(_power, dt);

    double acceleration = _rider_physics->acceleration(_velocity, gradient, _power);
    _velocity += dt * acceleration;
    if (_velocity < 0)
        _velocity = 0;
}

DescMap Rider::desc() {
    DescMap map = DescMap();
    map.insert(DescMap::value_type("speed (km/h)", _velocity * 3.6));
    map.insert(DescMap::value_type("power (W)", _power));
    return map;
}

double WPModel::maximum_power(double dt) {
    double P = _Wpbal / dt + _CP;
    if (P < 0)
        return 0.;
    return P;
}

void WPModel::update(double P, double dt) {
    if (P > _CP) 
        _Wpbal += (_CP - P) * dt; 
    else
        _Wpbal += (_CP - P) * (_Wp - _Wpbal) / _Wp * dt;
    if (_Wpbal > _Wp)
        _Wpbal = _Wp;
}
