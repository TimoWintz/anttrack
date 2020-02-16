#include "rider.h"
#include "constants.h"

double SimpleRiderPhysics::moving_resistance(double velocity) {
    return 0.5 * rho * CdA * velocity * velocity + Cxx * mass * constants::g;
}

double SimpleRiderPhysics::gravity_acceleration(double gradient) {
    return gradient * constants::g;
}
double SimpleRiderPhysics::acceleration(double velocity, double gradient, double power) {
    return power / velocity / mass - moving_resistance(velocity) - gravity_acceleration(gradient);
}  

Rider::Rider(std::shared_ptr<Track> track, std::shared_ptr<RiderPhysics> rider_physics) :
    _track(track), _rider_physics(rider_physics) {
        _pos_dh[0] = 0;
        _pos_dh[1] = 0;
        _track->coord_dh_to_xyz(_pos_dh, _pos_xyz);
}

void Rider::update(double dt) {
    double steering = _rider_controller->steering();
    double power = _rider_controller->power();

    double dl = dt * _velocity;
    double gradient = _track->update_position(_pos_xyz, dl, steering);

    double max_power = _rider_model->maximum_power(dt);
    if (power > max_power)
        power = max_power;

    double acceleration = _rider_physics->acceleration(_velocity, gradient, power);
    _velocity -= dt * acceleration;
}

double WPModel::maximum_power(double dt) {
    if (_Wpbal < 0)
        return 0.;
    return _Wpbal / dt + _CP;
}

void WPModel::update(double P, double dt) {
    if (P > _CP) 
        _Wpbal += (_CP - P) * dt; 
    else
        _Wpbal += (_CP - P) * (_Wp - _Wpbal) / _Wp * dt;
    if (_Wpbal > _Wp)
        _Wpbal = _Wp;
}
