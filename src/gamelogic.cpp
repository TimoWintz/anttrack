#include <sstream>

#include "gamelogic.h"
#include "utils.h"

GameLogic GameLogic::instance;

void GameLogic::init_from_config(toml::value& config) {
    try {

    // LOAD TRACK
    std::cout << "Setting up track...";
    const auto track_config = toml::find(config, "Track");
    float min_angle = toml::find<float>(track_config, "min_angle");
    float max_angle = toml::find<float>(track_config, "max_angle");
    float track_width = toml::find<float>(track_config, "track_width");
    float inner_radius = toml::find<float>(track_config, "inner_radius");
    float track_length = toml::find<float>(track_config, "track_length");
    float finish_line_position = toml::find<float>(track_config, "finish_line_position");
    size_t n_points = toml::find<size_t>(track_config, "n_points");
    std::vector<float> track_incline = sine_track_incline(deg_to_rad(min_angle), deg_to_rad(max_angle), n_points);
    _track.configure(inner_radius, track_length, track_width, track_incline, finish_line_position);

    // DRAFTING MODEL
    std::cout << "Setting up drafting model...";
    const auto drafting_model_config = toml::find(config, "DraftingModel");
    auto duration = toml::find<float>(drafting_model_config, "duration");
    auto angle = toml::find<float>(drafting_model_config, "angle");
    _drafting.set_duration(duration);
    _drafting.set_angle(angle);

    //RIDERS
    std::cout << "Setting up riders...";
    const auto riders_config = toml::find(config, "Riders");
    _bike_length = toml::find<float>(riders_config, "bike_length");
    _bike_width = toml::find<float>(riders_config, "bike_width");
    auto rider_map = toml::find<std::unordered_map<std::string , toml::value>>(riders_config, "riders");    auto riders_idx = toml::find<std::unordered_map<std::string , toml::value>>(riders_config, "riders");
    _rider_ids.clear();
    float rho = toml::find<float>(toml::find(config, "Physics"), "air_density");
    int i = 0;
    for (auto &x: riders_idx) {
        _rider_ids.push_back(i);

        float CdA = toml::find<float>(x.second, "CdA");
        float Cxx = toml::find<float>(x.second, "Cxx");
        float mass = toml::find<float>(x.second, "mass");
        _rider_physics[i] = RiderPhysics(CdA, Cxx, rho, mass);
        _rider_colors[i] = toml::find<long>(x.second, "color");
        _rider_power[i] = 0;
        _riders[i] = Rider();
        i++;
    }
    // RACE
    std::cout << "Setting up race...";
    const auto race_config = toml::find(config, "Race");
    int n_laps = toml::find<int>(race_config, "n_laps");
    _race.setup(n_laps, _riders);
    _race.start();

    } catch(std::exception e) {
        std::cout << "Failed to init game." << e.what() << std::endl;
        throw e;
    }
}

std::vector<Vec3f> GameLogic::track_points(int n_points) const {
    std::vector<Vec3f> res(2*n_points);
    Vec2f dh;
    Vec3f xyz;
    for (int i = 0; i < n_points; i++) {
        dh[0] = float(i) / n_points * track_length();
        dh[1] = 0;
        _track.coord_dh_to_xyz(dh, xyz);
        res[2*i] = xyz;
        dh[1] = track_width();
        _track.coord_dh_to_xyz(dh, xyz);
        res[2*i + 1] = xyz;
    }
    return res;
}

void GameLogic::update(float dt) {
    auto x = _drafting.drafting_coefficients(_riders);
    Vec3f xyz;
    Vec2f dh;

    for (auto id : rider_ids())
    {
        auto pos = _riders[id].position();
        dh = pos;

        _track.coord_dh_to_xyz(dh, xyz);
        float z = xyz[2];
        _track.update_position(dh, 1, _rider_steering[id]);
        _track.coord_dh_to_xyz(dh, xyz);
        float gradient = xyz[2] - z;
        
        _track.update_position(pos, _riders[id].velocity() * dt, _rider_steering[id]);
        _riders[id].set_position(pos);
        _riders[id].update_velocity(_rider_physics[id], dt, _rider_power[id], gradient, x[id]);
    }
    _race.update(dt, _riders);
}
Vec3f GameLogic::rider_position_xyz(int id) {
    Vec3f xyz;
    _track.coord_dh_to_xyz(_riders[id].position(), xyz);
    return xyz;
}

std::string GameLogic::status() {
    std::stringstream ss;
    for (auto id : _rider_ids)
    {
        ss << "id " << id;
        ss << " position " << _riders[id].position(0) << " " << _riders[id].position(1);
        ss << " velocity " << _riders[id].velocity();
        ss << " elapsed_dist " << _race.cumulative_distance(id);
        ss << " distance_to_finish " << _race.distance_to_finish(id) << std::endl;
    }
    ss << std::endl;
    return ss.str();
}