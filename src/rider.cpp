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
double SimpleRiderPhysics::acceleration(double velocity, double gradient, double power, double slipstream_velocity) {
    double acceleration = power / velocity / mass - moving_resistance(velocity - slipstream_velocity) / mass - gravity_acceleration(gradient);
    if (acceleration > max_acceleration)
        return max_acceleration;
    return acceleration;
}  

double SimpleRiderPhysics::current_power(double velocity, double gradient, double acceleration, double slipstream_velocity) {
    return acceleration * velocity * mass + moving_resistance(velocity - slipstream_velocity) * velocity + 
        gravity_acceleration(gradient) * velocity * mass;
}  

Rider::Rider(Track* track, RiderPhysics* rider_physics) : 
    _track(track), _rider_physics(rider_physics),  _slipstream_velocity(0.0) {
        _pos_dh[0] = 0;
        _pos_dh[1] = 0;
        _track->coord_dh_to_xyz(_pos_dh, _pos_xyz);
        _velocity = 0;
        _power = 0;
        _max_velocity = 1000;
        _id = _next_id++;
}

static double eps = 0.0001;

void Rider::update(double dt) {
    if (dt == 0)
        return;
    double dl = dt * _velocity;
    double old_z = _pos_xyz[2];

    _track->update_position(_pos_dh, dl, _steering);
    _track->coord_dh_to_xyz(_pos_dh, _pos_xyz);
    _track->direction_vector(_pos_dh, _direction);

    if (_velocity < eps)
        _gradient = 0;
    else
        _gradient = (_pos_xyz[2] - old_z) / _velocity / dt;

    double acceleration = _rider_physics->acceleration(_velocity, _gradient, _power, _slipstream_velocity);
    double tmp_velocity = _velocity + dt * acceleration;
    if (tmp_velocity < 0)
        tmp_velocity = 0;
    if (tmp_velocity > _max_velocity) {
        tmp_velocity = _max_velocity;
    }
    acceleration = (tmp_velocity - _velocity) / dt;
    _velocity = tmp_velocity;
    if (_velocity > 0)
        _power = _rider_physics->current_power(_velocity, _gradient, acceleration, _slipstream_velocity);
    if (_power < 0)
        _power = 0;
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
    map.insert(DescMap::value_type("slipstream vel (km/h)", _slipstream_velocity * 3.6));
    map.insert(DescMap::value_type("vel x (km/h)", velocity_vector()[0] * 3.6));
    map.insert(DescMap::value_type("vel y (km/h)", velocity_vector()[1] * 3.6));
    map.insert(DescMap::value_type("pos x (m)", pos_xyz()[0]));
    map.insert(DescMap::value_type("pos y (m)", pos_xyz()[1]));
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
        if (dist <= 0)
            continue;
        // angle = std::acos(dist / relpos.norm());
        // ratio = _max_multiplier * (_angle - angle);
        // if (ratio < 0)
        //     continue;
        vel = pv.second.norm();
        ratio = _max_multiplier;
        ratio *= (_duration - dist / vel / vel) / _duration;
        // slipstream_velocity = dist;
        if (ratio < 0)
            continue;
        slipstream_velocity += (pv.second.norm() - slipstream_velocity) * ratio;
        // slipstream_velocity = dist;
    }
    // std::cout << "vel  = " << vel << std::endl;
    // std::cout << slipstream_velocity << std::endl;
    return slipstream_velocity;
}

void RiderInteraction::update() {
    for (size_t i = 0; i < _riders.size(); i++) {
        _positions_and_velocities[i].first = _riders[i]->pos_xyz();
        _positions_and_velocities[i].second = _riders[i]->velocity_vector();
        for (size_t j=0; j < _riders.size(); j++) {
            if (i == j)
                continue;
            auto dir_i = _riders[i]->direction_vector();
            auto signed_dist = dir_i.dot(_riders[j]->pos_xyz() - _riders[i]->pos_xyz());
            if (signed_dist > 0 && signed_dist < constants::rider_length) 
                _riders[i]->set_max_velocity(_riders[j]->velocity());
            else
                _riders[i]->set_max_velocity(std::numeric_limits<double>::max());
        }
    }
    for (auto rider : _riders) {
        double slipstream_velocity = _drafting_model->slipstream_velocity(_positions_and_velocities, rider->pos_xyz());
        rider->set_slipstream_velocity(slipstream_velocity);
    }
}