#include "beam_utils/angles.hpp"

namespace beam {

double wrapToPi(double angle) {
    double wrapped_angle = wrapToTwoPi(angle + M_PI) - M_PI;
    return wrapped_angle;
}

double wrapToTwoPi(double angle) {
    double wrapped_angle = fmod(angle, 2 * M_PI);

    if (wrapped_angle < 0) {
        wrapped_angle += 2 * M_PI;
    }

    return wrapped_angle;
}

double deg2rad(double d) {
    return d * (M_PI / 180);
}

double rad2deg(double r) {
    return r * (180 / M_PI);
}

double wrapTo180(double euler_angle) {
    return fmod((euler_angle + 180.0), 360.0) - 180.0;
}

double wrapTo360(double euler_angle) {
    if (euler_angle > 0) {
        return fmod(euler_angle, 360.0);
    } else {
        euler_angle += 360.0;
        return fmod(euler_angle, 360.0);
    }
}

}  // namespace beam
