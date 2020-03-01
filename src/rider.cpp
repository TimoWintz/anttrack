#include "rider.h"
#include "constants.h"
#include <iostream>
#include <numeric>

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

static double eps = 0.0001;

void Rider::update(double dt) {
    double steering = _rider_controller->steering();
    _power = _rider_controller->power();

    double dl = dt * _velocity;
    double old_z = _pos_xyz[2];
    _track->update_position(_pos_dh, dl, steering);
    _track->coord_dh_to_xyz(_pos_dh, _pos_xyz);
    _track->direction_vector(_pos_dh, _direction);

    double gradient;
    if (_velocity < eps)
        gradient = 0;
    else
        gradient = (_pos_xyz[2] - old_z) / _velocity / dt;

    double max_power = _rider_model->maximum_power(dt);
    if (_power > max_power)
        _power = max_power;

    _rider_model->update(_power, dt);

    double acceleration = _rider_physics->acceleration(_velocity, gradient, _power);
    _velocity += dt * acceleration;
    if (_velocity < 0)
        _velocity = 0;
}

void Rider::set_pos_dh(const Vec2d& dh){
        _pos_dh[0] = dh[0];
        _pos_dh[1] = dh[1];
        _track->coord_dh_to_xyz(_pos_dh, _pos_xyz);
}
void Rider::set_pos_xyz(const Vec3d& xyz){
        _pos_xyz[0] = xyz[0];
        _pos_xyz[1] = xyz[1];
        _pos_xyz[2] = xyz[2];
        _track->coord_xyz_to_dh(_pos_xyz, _pos_dh);
}

DescMap Rider::desc() {
    DescMap map = DescMap();
    map.insert(DescMap::value_type("speed (km/h)", _velocity * 3.6));
    map.insert(DescMap::value_type("power (W)", _power));
    map.insert(DescMap::value_type("distance (m)", _pos_dh[0]));
    map.insert(DescMap::value_type("height (m)", _pos_dh[1]));
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

double SimpleConeDrafting::slipstream_velocity(std::vector<PosVel> positions_and_velocities,
                                   const Vec3d& position) {
    auto cmp = [position](const PosVel& a, const PosVel& b) -> bool {
        return (a.first - position).dot(a.second / a.second.norm()) < (b.first - position).dot(b.second / b.second.norm());
    };
    std::sort(positions_and_velocities.begin(), positions_and_velocities.end(), cmp);
    double slipstream_velocity = 0.0;
    Vec3d relpos;
    double angle, dist, ratio, vel;
    for (auto pv: positions_and_velocities) {
        relpos = pv.first - position;
        dist = relpos.dot(pv.second);
        if (dist < 0)
            continue;
        angle = std::acos(dist / relpos.norm());
        ratio = _max_multiplier * (_angle - angle);
        if (ratio < 0)
            continue;
        vel = pv.second.norm();
        ratio *= (_duration - dist / vel) / _duration;
        if (ratio < 0)
            continue;
        slipstream_velocity += (pv.second.norm() - slipstream_velocity) * ratio;
    }
    return slipstream_velocity;
}
