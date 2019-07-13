#ifndef BEAM_UTILS_ANGLES_HPP
#define BEAM_UTILS_ANGLES_HPP

#include <cmath>

namespace beam {

  #define DEG_TO_RAD 0.0174532925199433
  #define RAD_TO_DEG 57.2957795130823209  

/** Wraps input angle to the interval [-PI, PI).
 *
 * @param[in] angle the original angle.
 * @return the wrapped angle.
 */
double wrapToPi(double angle);

/** Wraps input angle to the interval [0, 2*PI).
 *
 * @param[in] angle the original angle.
 * @return the wrapped angle.
 */
double wrapToTwoPi(double angle);

/** Converts degrees to radians. */
double deg2rad(double d);

/** Converts radians to degrees. */
double rad2deg(double r);

/** Wraps `euler_angle` to 180 degrees **/
double wrapTo180(double euler_angle);

/** Wraps `euler_angle` to 360 degrees **/
double wrapTo360(double euler_angle);

}  // namespace beam
#endif  // BEAM_UTILS_ANGLES_HPP
