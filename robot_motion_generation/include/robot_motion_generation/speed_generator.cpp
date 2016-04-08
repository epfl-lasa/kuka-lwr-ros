#include "kuka_common_action_server/speed_generator.h"
#include <cmath>


namespace asrv{



Speed_generator::Speed_generator(double max_speed_ms, double min_speed_ms, double beta):
    max_speed_ms(max_speed_ms),
    min_speed_ms(min_speed_ms),
    beta(beta)
{

}

void Speed_generator::set_min_speed_ms(const double min_speed_ms){
    this->min_speed_ms = min_speed_ms;
}

void Speed_generator::set_max_speed_ms(const double max_speed_ms){
    this->max_speed_ms = max_speed_ms;
}

double Speed_generator::bel_shape_curve(double distance_target_m){
    distance_target_cm = 100 * distance_target_m;
    return (min_speed_ms + (1.0 - std::exp(-beta * (distance_target_cm * distance_target_cm))) * max_speed_ms);
}


}
