#ifndef gamelogic_h_INCLUDED
#define gamelogic_h_INCLUDED

#include <toml.hpp> // that's all! now you can use it.

#include "track.h"
#include "rider.h"
#include "race.h"
#include "utils.h"

class GameLogic {
    private:
        Track _track;
        std::vector<int> _rider_ids;
        std::map<int, RiderPhysics> _rider_physics;
        std::map<int, float> _rider_power;
        std::map<int, Rider> _riders;
        std::map<int, float> _rider_steering;
        std::map<int, long> _rider_colors;
        Race _race;
        DraftingModel _drafting;

        float _bike_length;
        float _bike_width;
        
        int _n_laps;

        static GameLogic instance;
        GameLogic() {

        };
    public:
        static GameLogic& get() {
            return instance;
        };
        void init_from_config(toml::value& config);
        std::vector<int> rider_ids() const {return _rider_ids;};
        std::vector<Vec3f> track_points(int n_points) const;
        int n_riders() const {return _rider_ids.size();};
        float track_width() const {return _track.width();};
        float track_length() const {return _track.length();};
        float finish_line_position() const {return _track.finish_line_position();};
        float bike_length() const {return _bike_length; };
        float bike_width() const {return _bike_width; };
        float n_laps() const {return _n_laps; };
        long rider_color(int id) {return _rider_colors[id];};
        void update(float dt);
        void set_power(int id, float value) {
            _rider_power[id] = value;
        }
        void set_steering(int id, float value) {
            _rider_steering[id] = value;
        }
        Vec3f rider_position_xyz(int id);
        std::string GameLogic::status();
    };
#endif