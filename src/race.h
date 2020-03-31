#ifndef race_h_INCLUDED
#define race_h_INCLUDED
#include <vector>
#include <array>

#include "rider.h"
#include "track.h"

class Race {
    int _n_laps;
    float _total_distance;
    
    bool _is_finished;
    bool _is_started;
    float _current_time;
    

    std::vector<int> _rankings;
    std::map<int, float> _rider_positions;
    std::map<int, int> _current_n_laps;
    std::map<int, float> _cumulative_time;
    std::map<int, float> _cumulative_distance;

    public:
        Race() {};
        void setup(int n_laps, std::map<int, Rider>& riders);
        bool is_finished() const {return _is_finished; };
        bool is_started() const {return _is_started; };
        bool lock_riders() const {return !_is_started;};
        std::map<int, Vec2f> rider_positions();
        void update(double dt, std::map<int, Rider> &riders) ;
        void start();
        std::vector<int> rankings() {return _rankings;};
        std::vector<float> timings() {return std::vector<float>({_cumulative_time[0], _cumulative_time[1]});};
        std::vector<float> cumulative_distance() {return {_cumulative_distance[0], _cumulative_distance[1]};};
        std::vector<float> distance_to_finish() {return {_total_distance - _cumulative_distance[0], 
                                                        _total_distance - _cumulative_distance[1]};};
};

#endif // src/race_h_INCLUDED

