#include "race.h"
#include "gamelogic.h"

void Race::setup(int n_laps, std::map<int, Rider>& riders) {
    _n_laps = n_laps;
    _total_distance = GameLogic::get().track_length() * n_laps;
    _is_started = false;
    _is_finished = false;
    auto pos = rider_positions();
    for (auto id : GameLogic::get().rider_ids())
    {
        riders[id].set_position(pos[id]);
        riders[id].set_velocity(0);
    }
}

std::map<int, Vec2f> Race::rider_positions()
{
    std::map<int, Vec2f> result;
    std::vector<int> rider_ids = GameLogic::get().rider_ids();
    int i = 0;
    int j = 1;
    for (auto x : rider_ids) 
    {
        float position_h = i * GameLogic::get().bike_width();
        if (position_h > GameLogic::get().track_width()) {
            i = 0;
            position_h = 0;
            j += 1;
        }
        float position_d = GameLogic::get().track_length() - j * GameLogic::get().bike_length() ;
        result[x] = Vec2f(position_d, position_h);
        i++;
    } 
    return result; 
}

void Race::start() {
    _current_time = 0;
    for (int i = 0; i < 2; i++) {
        _current_n_laps[i] = 0;
        _cumulative_time[i] = 0.0;
        _cumulative_distance[i] = 0.0;
    }
    _is_started = true;
}

inline void adjust_diff(float& val) {
    while (val < -GameLogic::get().track_length() / 2) {
        val += GameLogic::get().track_length();
    }
    while (val > GameLogic::get().track_length() / 2) {
        val -= GameLogic::get().track_length();
    }
}

void Race::update(double dt, std::map<int, Rider>& riders) {
    _current_time += dt;
    if (!_is_started) {
        return;
    }
    float finish_line = GameLogic::get().finish_line_position();
    bool flag = true;
    for (auto i : GameLogic::get().rider_ids()) {
        if (_rider_positions.find(i) == _rider_positions.end()) {
            _rider_positions[i] = riders[i].position(0);
            continue;
        }
        if (_current_n_laps[i] == _n_laps)
            continue;
        float dl = riders[i].position(0) - _rider_positions[i];
        adjust_diff(dl);
        _cumulative_distance[i] += dl;
        flag = false;
        float to_finish_line = riders[i].position(0) - finish_line;
        adjust_diff(to_finish_line);
        float prev_to_finish_line = _rider_positions[i] - finish_line;
        adjust_diff(prev_to_finish_line);

        if (prev_to_finish_line < 0 && to_finish_line >= 0) {
            _current_n_laps[i] ++;
            _cumulative_time[i] = _current_time - to_finish_line / riders[i].velocity() / dt;
        }
        _rider_positions[i] = riders[i].position(0);
    }
    if (_is_finished)
        return;
    if ((_current_n_laps[0] > _current_n_laps[1]) || ((_current_n_laps[0] == _current_n_laps[1]) && _cumulative_time[0] < _cumulative_time[1]))
        _rankings = {0, 1};
    else
        _rankings = {1,0};
    _is_finished = (_current_n_laps[0] == _n_laps) || (_current_n_laps[1] == _n_laps);
    return;
}
