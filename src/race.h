#ifndef race_h_INCLUDED
#define race_h_INCLUDED
#include <vector>
#include <array>

#include "rider.h"
#include "track.h"
#include "utils.h"

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
        float timings(int id) {return _cumulative_time[id];};
        float cumulative_distance(int id) {return _cumulative_distance[id];};
        float distance_to_finish(int id) {return _total_distance - _cumulative_distance[id];};
};

#endif // src/race_h_INCLUDED

