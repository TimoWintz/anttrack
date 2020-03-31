#include <iostream>
#include <numeric>
#include <istream>
#include <chrono>
#include <math.h>       /* isnan, sqrt */

#include "rider.h"
#include "gamelogic.h"

float RiderPhysics::moving_resistance(float velocity) const {
    return 0.5f * rho * CdA * velocity * velocity + mass* Cxx * 9.81f;
}

float RiderPhysics::gravity_acceleration(float gradient) const {
    return gradient * 9.81f;
}
float RiderPhysics::delta_v(float velocity, float gradient, float power, float dt, float slipstream_velocity) const {
    if (slipstream_velocity > velocity)
        slipstream_velocity = velocity;
    float backwards_acceleration = moving_resistance(velocity - slipstream_velocity) / mass + gravity_acceleration(gradient);
    float a = 1;
    float b = velocity - backwards_acceleration*dt;
    float c = - power*dt / mass + backwards_acceleration*velocity*dt;
    float delta = b*b - 4*a*c;
    if (delta < 0)
        return -velocity;
    float res = (-b + std::sqrt(delta)) / 2;
    return res;
}  

float RiderPhysics::current_power(float velocity, float gradient, float acceleration, float slipstream_velocity) const {
    if (slipstream_velocity > velocity)
        slipstream_velocity = velocity;
    return acceleration * velocity * mass + moving_resistance(velocity - slipstream_velocity) * velocity + 
        gravity_acceleration(gradient) * velocity * mass;
}  


static float eps = 0.0001;

void Rider::update_velocity(const RiderPhysics& physics, float dt, float power, float gradient, float drafting_coefficient) 
{
    if (dt == 0)
        return;
    float delta_v = physics.delta_v(_velocity, gradient, power, dt, drafting_coefficient);
    _velocity += delta_v;
    if (_velocity < 0)
        _velocity = 0;
    
}
/* void Rider::update(float dt) {
    if (dt == 0)
        return;
    float old_z = _pos_xyz[2];

    _track->update_position(_pos_dh,  dt * _velocity, _steering);
    _track->coord_dh_to_xyz(_pos_dh, _pos_xyz);
    _track->direction_vector(_pos_dh, _direction);

    if (_velocity < eps)
        _gradient = 0;
    else
        _gradient = (_pos_xyz[2] - old_z) / _velocity / dt;

    float delta_v = _rider_physics->delta_v(_velocity, _gradient, _power, dt, _slipstream_velocity);
    float tmp_velocity = _velocity + delta_v;
    if (tmp_velocity < 0)
        tmp_velocity = 0;
    if (tmp_velocity > _max_velocity) {
        tmp_velocity = _max_velocity;
    }
    _velocity = tmp_velocity;
}
 */
std::map<int, float> DraftingModel::drafting_coefficients(const std::map<int, Rider>& riders)
{
    std::map<int, float> res;
    for (auto id : GameLogic::get().rider_ids()) {
        res[id] = 0;
    }
    return res;
}

/* float DraftingModel::slipstream_velocity(std::vector<PosVel> positions_and_velocities,
                                   const Vec3f& position) {
    auto cmp = [position](const PosVel& a, const PosVel& b) -> bool {
        return (a.first - position).dot(a.second / a.second.norm()) < (b.first - position).dot(b.second / b.second.norm());
    };
    std::sort(positions_and_velocities.begin(), positions_and_velocities.end(), cmp);
    float slipstream_velocity = 0.0;
    Vec3f relpos;
    float angle, dist, ratio, vel;
    for (auto pv: positions_and_velocities) {
        relpos = pv.first - position;
        dist = relpos.dot(pv.second);
        if (dist <= 0)
            continue;
        angle = std::acos(dist / relpos.norm() / pv.second.norm());
        ratio = _max_multiplier * (_angle - angle) / _angle;
        if (ratio < 0)
            continue;
        vel = pv.second.norm();
        if (vel == 0) {
            continue;
        }
        // ratio = _max_multiplier;
        ratio *= (_duration - dist / vel / vel) / _duration;
        // slipstream_velocity = dist;
        if (ratio < 0)
            continue;
        slipstream_velocity += (pv.second.norm() - slipstream_velocity) * ratio;
        // slipstream_velocity = dist;
    }
    return slipstream_velocity;
}
 */
/* void RiderInteraction::update() {
    for (size_t i = 0; i < _riders.size(); i++) {
        _positions_and_velocities[i].first = _riders[i]->pos_xyz();
        _positions_and_velocities[i].second = _riders[i]->velocity_vector();
        _riders[i]->set_max_velocity(std::numeric_limits<float>::max());
        for (size_t j=0; j < _riders.size(); j++) {
            if (i == j)
                continue;
            auto dir_i = _riders[i]->direction_vector();
            auto signed_dist = dir_i.dot(_riders[j]->pos_xyz() - _riders[i]->pos_xyz());
            auto side_dist = std::abs(_riders[j]->pos_dh()[1] - _riders[i]->pos_dh()[1]);
            if (signed_dist > 0 && signed_dist < _riders[j]->length() && 
                    _riders[i]->max_velocity() > _riders[j]->velocity() && side_dist < _riders[j]->width()) 
                _riders[i]->set_max_velocity(_riders[j]->velocity());
        }
    }
    for (auto rider : _riders) {
        float slipstream_velocity = _drafting_model->slipstream_velocity(_positions_and_velocities, rider->pos_xyz());
        rider->set_slipstream_velocity(slipstream_velocity);
    }
} */
