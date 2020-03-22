#ifndef race_h_INCLUDED
#define race_h_INCLUDED
#include <vector>
#include <array>

#include "rider.h"
#include "track.h"

class Race {
    public:
        virtual void update(double dt) = 0;
        virtual bool is_finished() = 0;
        virtual void position_riders() = 0;
        virtual bool is_started() = 0;
        virtual void start() = 0;
        virtual bool lock_riders() = 0;
        virtual std::vector<int> rankings() = 0; 
        virtual std::vector<float> timings() = 0;
        virtual std::vector<float> cumulative_distance() = 0;
        virtual DescMap desc() = 0;
};

class Sprint: public Race {
    std::vector<Rider*> _riders;
    Track* _track;
    int _n_laps;
    bool _is_finished;
    bool _is_started;
    float _current_time;

    std::vector<int> _rankings;

    std::array<int, 2> _current_n_laps;
    std::array<float, 2> _cumulative_time;
    std::array<float, 2> _cumulative_distance;

    public:
        Sprint(std::vector<Rider*> riders, Track* track, int n_laps) : _riders(riders), _track(track), _n_laps(n_laps),
            _is_finished(false), _is_started(false) {
            if (riders.size() != 2) {
                throw std::runtime_error("Sprint is for 2 riders only.");
            }
        };
        bool is_finished() {return _is_finished; };
        bool is_started() {return _is_started; };
        bool lock_riders() {return !_is_started;}
        void position_riders();
        void update(double time);
        void start();
        std::vector<int> rankings() {return _rankings;};
        std::vector<float> timings() {return std::vector<float>({_cumulative_time[0], _cumulative_time[1]});};
        DescMap desc();
        std::vector<float> cumulative_distance() {return {_cumulative_distance[0], _cumulative_distance[1]};};
};

#endif // src/race_h_INCLUDED

