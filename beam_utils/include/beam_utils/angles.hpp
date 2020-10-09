#pragma once

#include <cmath>

namespace beam {

/**
 * @Brief Wraps input angle to the interval [-PI, PI).
 * @param angle the original angle.
 * @return the wrapped angle.
 */
double WrapToPi(double angle);

/**
 * @Brief Wraps input angle to the interval [0, 2*PI).
 * @param angle the original angle.
 * @return the wrapped angle.
 */
double WrapToTwoPi(double angle);

/**
 * @Brief Return the smallest difference between two angles. This takes into
 * account the case where one or both angles are outside (0, 360). By smallest
 * error, we mean for example: GetSmallestAngleErrorDeg(10, 350) = 20, not 340
 * @param angle 1 in degrees
 * @param angle 2 in degrees
 * @return error in degrees
 */
double GetSmallestAngleErrorDeg(double angle1, double angle2);

/**
 * @Brief Return the smallest difference between two angles. This takes into
 * account the case where one or both angles are outside (0, 2PI). By smallest
 * error, we mean for example: GetSmallestAngleErrorDeg(0.1PI, 1.9PI) = 0.2PI,
 * not 1.8PI
 * @param angle 1 in radians
 * @param angle 2 in radians
 * @return error in radians
 */
double GetSmallestAngleErrorRad(double angle1, double angle2);

/** Converts degrees to radians. */
double Deg2Rad(double d);

/** Converts radians to degrees. */
double Rad2Deg(double r);

/** Wraps `euler_angle` to 180 degrees **/
double wrapTo180(double euler_angle);

/** Wraps `euler_angle` to 360 degrees **/
double wrapTo360(double euler_angle);

} // namespace beam
