#ifndef ACTION_SERVER_SPEED_GENERATOR_H_
#define ACTION_SERVER_SPEED_GENERATOR_H_

#include "kuka_common_action_server/si_units.h"
#include <boost/units/is_unit.hpp>


namespace asrv{



class Speed_generator{



public:

    Speed_generator(double max_speed_ms=0.1, double min_speed_ms=0.0, double beta=10.0);

    /**
     * @brief bel_shape_curve: speed is modulated as a function of the distance to the
     *                         target.
     * @param distance_target: distance to target point (meters).
     * @param beta           : stifness (1/var), the higher the stiffness,
     *                         the higher the velocity
     * @return               : speed amplitude to be multiplied to direction vector
     */
    double bel_shape_curve(double distance_target_m);

    void set_min_speed_ms(const double min_speed_ms);

    void set_max_speed_ms(const double max_speed_ms);

private:

    double max_speed_ms;
    double min_speed_ms;
    double beta;                /// parameter of bel shape curve (high value sharp slop, low value slow slop)
    double distance_target_cm;

};


}
#endif
