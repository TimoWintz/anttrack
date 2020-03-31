#include "race.h"

void Sprint::position_riders() {
    _riders[0]->set_pos_dh({_track->finish_line_position() + 1 , 0});
    _riders[1]->set_pos_dh({_track->finish_line_position(), 1});
    _is_started = false;
    _is_finished = false;
}

void Sprint::start() {
    _current_time = 0;
    for (int i = 0; i < 2; i++) {
        _current_n_laps[i] = 0;
        _cumulative_time[i] = 0.0;
        _cumulative_distance[i] = 0.0;
    }
    _is_started = true;
}

DescMap Sprint::desc() {
    DescMap map = DescMap();
    map.insert(DescMap::value_type("laps (rider 0)", _current_n_laps[0] ));
    map.insert(DescMap::value_type("laps (rider 1)", _current_n_laps[1] ));
    map.insert(DescMap::value_type("last lap timing (rider 0)", _cumulative_time[0] ));
    map.insert(DescMap::value_type("last lap timing (rider 1)", _cumulative_time[1] ));
    return map;
}

void Sprint::update(double dt) {
    _current_time += dt;
    if (!_is_started) {
        return;
    }
    double finish_line, rider_pos;
    finish_line = _track->finish_line_position();
    bool flag = true;
    for (int i = 0; i < 2; i++) {
        _cumulative_distance[i] += _riders[i]->dl();
        if (_current_n_laps[i] == _n_laps)
            continue;
        flag = false;
        rider_pos = _riders[i]->pos_dh()[0];
        if (rider_pos >= finish_line && rider_pos - _riders[i]->dl() < finish_line) {
            _current_n_laps[i] ++;
            _cumulative_time[i] = _current_time - (rider_pos - finish_line) / _riders[i]->velocity() / dt;
        }
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
