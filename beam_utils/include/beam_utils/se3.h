/** @file
 * @ingroup utils
 *
 * bspline fitting from: https://docs.openvins.com/
 *
 */

#pragma once

#include <Eigen/Eigen>

namespace beam {
/** @addtogroup utils
 *  @{ */

// *************************************************
// THE  FOLLOWING FUNCTIONS WERE TAKEN FROM OPENVINS
// *************************************************

/*
 * ADAPTED FOR BEAM FROM:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * @section Summary
 * This file contains the common utility functions for operating on JPL
 * quaternions. We need to take special care to handle edge cases when
 * converting to and from other rotation formats. All equations are based on the
 * following tech report: > Trawny, Nikolas, and Stergios I. Roumeliotis.
 * "Indirect Kalman filter for 3D attitude estimation." > University of
 * Minnesota, Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005. >
 * http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
 *
 * @section JPL Quaternion Definition
 *
 * We define the quaternion as the following linear combination:
 * @f[
 *  \bar{q} = q_4+q_1\mathbf{i}+q_2\mathbf{j}+q_3\mathbf{k}
 * @f]
 * Where i,j,k are defined as the following:
 * @f[
 *  \mathbf{i}^2=-1~,~\mathbf{j}^2=-1~,~\mathbf{k}^2=-1
 * @f]
 * @f[
 *  -\mathbf{i}\mathbf{j}=\mathbf{j}\mathbf{i}=\mathbf{k}
 *  ~,~
 *  -\mathbf{j}\mathbf{k}=\mathbf{k}\mathbf{j}=\mathbf{i}
 *  ~,~
 *  -\mathbf{k}\mathbf{i}=\mathbf{i}\mathbf{k}=\mathbf{j}
 * @f]
 * As noted in [Trawny2005] this does not correspond to the Hamilton notation,
 * and follows the "JPL Proposed Standard Conventions". The q_4 quantity is the
 * "scalar" portion of the quaternion, while q_1,q_2,q_3 are part of the
 * "vector" portion. We split the 4x1 vector into the following convention:
 * @f[
 *  \bar{q} = \begin{bmatrix}q_1\\q_2\\q_3\\q_4\end{bmatrix} =
 * \begin{bmatrix}\mathbf{q}\\q_4\end{bmatrix}
 * @f]
 * It is also important to note that the quaternion is constrainted to the unit
 * circle:
 * @f[
 *  |\bar{q}| = \sqrt{\bar{q}^\top\bar{q}} = \sqrt{|\mathbf{q}|^2+q_4^2} = 1
 * @f]
 *
 */

/**
 * @brief Returns a JPL quaternion from a rotation matrix
 *
 * This is based on the equation 74 in [Indirect Kalman Filter for 3D Attitude
 * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf). In the
 * implementation, we have 4 statements so that we avoid a division by zero and
 * instead always divide by the largest diagonal element. This all comes from
 * the definition of a rotation matrix, using the diagonal elements and an
 * off-diagonal. \f{align*}{ \mathbf{R}(\bar{q})= \begin{bmatrix}
 *  q_1^2-q_2^2-q_3^2+q_4^2 & 2(q_1q_2+q_3q_4) & 2(q_1q_3-q_2q_4) \\
 *  2(q_1q_2-q_3q_4) & -q_2^2+q_2^2-q_3^2+q_4^2 & 2(q_2q_3+q_1q_4) \\
 *  2(q_1q_3+q_2q_4) & 2(q_2q_3-q_1q_4) & -q_1^2-q_2^2+q_3^2+q_4^2
 *  \end{bmatrix}
 * \f}
 *
 * @param[in] rot 3x3 rotation matrix
 * @return 4x1 quaternion
 */
Eigen::Matrix<double, 4, 1> Rot2Quat(const Eigen::Matrix<double, 3, 3>& rot);

/**
 * @brief Skew-symmetric matrix from a given 3x1 vector
 *
 * This is based on equation 6 in [Indirect Kalman Filter for 3D Attitude
 * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf): \f{align*}{
 *  \lfloor\mathbf{v}\times\rfloor =
 *  \begin{bmatrix}
 *  0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0
 *  \end{bmatrix}
 * @f}
 *
 * @param[in] w 3x1 vector to be made a skew-symmetric
 * @return 3x3 skew-symmetric matrix
 */
Eigen::Matrix<double, 3, 3> SkewX(const Eigen::Matrix<double, 3, 1>& w);

/**
 * @brief Converts JPL quaterion to SO(3) rotation matrix
 *
 * This is based on equation 62 in [Indirect Kalman Filter for 3D Attitude
 * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf): \f{align*}{
 *  \mathbf{R} =
 * (2q_4^2-1)\mathbf{I}_3-2q_4\lfloor\mathbf{q}\times\rfloor+2\mathbf{q}\mathbf{q}^\top
 * @f}
 *
 * @param[in] q JPL quaternion
 * @return 3x3 SO(3) rotation matrix
 */
Eigen::Matrix<double, 3, 3> Quat2Rot(const Eigen::Matrix<double, 4, 1>& q);

/**
 * @brief Multiply two JPL quaternions
 *
 * This is based on equation 9 in [Indirect Kalman Filter for 3D Attitude
 * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf). We also enforce
 * that the quaternion is unique by having q_4 be greater than zero. \f{align*}{
 *  \bar{q}\otimes\bar{p}=
 *  \mathcal{L}(\bar{q})\bar{p}=
 *  \begin{bmatrix}
 *  q_4\mathbf{I}_3+\lfloor\mathbf{q}\times\rfloor & \mathbf{q} \\
 *  -\mathbf{q}^\top & q_4
 *  \end{bmatrix}
 *  \begin{bmatrix}
 *  \mathbf{p} \\ p_4
 *  \end{bmatrix}
 * @f}
 *
 * @param[in] q First JPL quaternion
 * @param[in] p Second JPL quaternion
 * @return 4x1 resulting q*p quaternion
 */
Eigen::Matrix<double, 4, 1> QuatMultiply(const Eigen::Matrix<double, 4, 1>& q,
                                         const Eigen::Matrix<double, 4, 1>& p);

/**
 * @brief Returns vector portion of skew-symmetric
 *
 * See skew_x() for details.
 *
 * @param[in] w_x skew-symmetric matrix
 * @return 3x1 vector portion of skew
 */
inline Eigen::Matrix<double, 3, 1> Vee(const Eigen::Matrix<double, 3, 3>& w_x);

/**
 * @brief SO(3) matrix exponential
 *
 * SO(3) matrix exponential mapping from the vector to SO(3) lie group.
 * This formula ends up being the [Rodrigues
 * formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula). This
 * definition was taken from "Lie Groups for 2D and 3D Transformations" by Ethan
 * Eade equation 15. http://ethaneade.com/lie.pdf
 *
 * \f{align*}{
 * \exp\colon\mathfrak{so}(3)&\to SO(3) \\
 * \exp(\mathbf{v}) &=
 * \mathbf{I}
 * +\frac{\sin{\theta}}{\theta}\lfloor\mathbf{v}\times\rfloor
 * +\frac{1-\cos{\theta}}{\theta^2}\lfloor\mathbf{v}\times\rfloor^2 \\
 * \mathrm{where}&\quad \theta^2 = \mathbf{v}^\top\mathbf{v}
 * @f}
 *
 * @param[in] w 3x1 vector in R(3) we will take the exponential of
 * @return SO(3) rotation matrix
 */
Eigen::Matrix<double, 3, 3> ExpSo3(const Eigen::Matrix<double, 3, 1>& w);

/**
 * @brief SO(3) matrix logarithm
 *
 * This definition was taken from "Lie Groups for 2D and 3D Transformations" by
 * Ethan Eade equation 17 & 18. http://ethaneade.com/lie.pdf \f{align*}{
 * \theta &= \textrm{arccos}(0.5(\textrm{trace}(\mathbf{R})-1)) \\
 * \lfloor\mathbf{v}\times\rfloor &=
 * \frac{\theta}{2\sin{\theta}}(\mathbf{R}-\mathbf{R}^\top)
 * @f}
 *
 * This function is based on the GTSAM one as the original implementation was a
 * bit unstable. See the following:
 * - https://github.com/borglab/gtsam/
 * - https://github.com/borglab/gtsam/issues/746
 * - https://github.com/borglab/gtsam/pull/780
 *
 * @param[in] R 3x3 SO(3) rotation matrix
 * @return 3x1 in the R(3) space [omegax, omegay, omegaz]
 */
Eigen::Matrix<double, 3, 1> LogSo3(const Eigen::Matrix<double, 3, 3>& R);

/**
 * @brief SE(3) matrix exponential function
 *
 * Equation is from Ethan Eade's reference: http://ethaneade.com/lie.pdf
 * \f{align*}{
 * \exp([\boldsymbol\omega,\mathbf u])&=\begin{bmatrix} \mathbf R & \mathbf V
 * \mathbf u \\ \mathbf 0 & 1 \end{bmatrix} \\[1em]
 * \mathbf R &= \mathbf I + A \lfloor \boldsymbol\omega \times\rfloor + B
 * \lfloor \boldsymbol\omega \times\rfloor^2 \\ \mathbf V &= \mathbf I + B
 * \lfloor \boldsymbol\omega \times\rfloor + C \lfloor \boldsymbol\omega
 * \times\rfloor^2 \f} where we have the following definitions \f{align*}{
 * \theta &= \sqrt{\boldsymbol\omega^\top\boldsymbol\omega} \\
 * A &= \sin\theta/\theta \\
 * B &= (1-\cos\theta)/\theta^2 \\
 * C &= (1-A)/\theta^2
 * \f}
 *
 * @param vec 6x1 in the R(6) space [omega, u]
 * @return 4x4 SE(3) matrix
 */
Eigen::Matrix4d ExpSe3(Eigen::Matrix<double, 6, 1> vec);

/**
 * @brief SE(3) matrix logarithm
 *
 * Equation is from Ethan Eade's reference: http://ethaneade.com/lie.pdf
 * \f{align*}{
 * \boldsymbol\omega
 * &=\mathrm{skew\_offdiags}\Big(\frac{\theta}{2\sin\theta}(\mathbf R - \mathbf
 * R^\top)\Big) \\ \mathbf u &= \mathbf V^{-1}\mathbf t \f} where we have the
 * following definitions \f{align*}{
 * \theta &= \mathrm{arccos}((\mathrm{tr}(\mathbf R)-1)/2) \\
 * \mathbf V^{-1} &= \mathbf I - \frac{1}{2} \lfloor \boldsymbol\omega
 * \times\rfloor + \frac{1}{\theta^2}\Big(1-\frac{A}{2B}\Big)\lfloor
 * \boldsymbol\omega \times\rfloor^2 \f}
 *
 * This function is based on the GTSAM one as the original implementation was a
 * bit unstable. See the following:
 * - https://github.com/borglab/gtsam/
 * - https://github.com/borglab/gtsam/issues/746
 * - https://github.com/borglab/gtsam/pull/780
 *
 * @param mat 4x4 SE(3) matrix
 * @return 6x1 in the R(6) space [omega, u]
 */
Eigen::Matrix<double, 6, 1> LogSe3(Eigen::Matrix4d mat);

/**
 * @brief Hat operator for R^6 -> Lie Algebra se(3)
 *
 * \f{align*}{
 * \boldsymbol\Omega^{\wedge} = \begin{bmatrix} \lfloor \boldsymbol\omega
 * \times\rfloor & \mathbf u \\ \mathbf 0 & 0 \end{bmatrix} \f}
 *
 * @param vec 6x1 in the R(6) space [omega, u]
 * @return Lie algebra se(3) 4x4 matrix
 */
Eigen::Matrix4d HatSe3(const Eigen::Matrix<double, 6, 1>& vec);

/**
 * @brief SE(3) matrix analytical inverse
 *
 * It seems that using the .inverse() function is not a good way.
 * This should be used in all cases we need the inverse instead of numerical
 * inverse. https://github.com/rpng/open_vins/issues/12 \f{align*}{
 * \mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^\top &
 * -\mathbf{R}^\top\mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix} \f}
 *
 * @param[in] T SE(3) matrix
 * @return inversed SE(3) matrix
 */
Eigen::Matrix4d InvSe3(const Eigen::Matrix4d& T);

/**
 * @brief JPL Quaternion inverse
 *
 * See equation 21 in [Indirect Kalman Filter for 3D Attitude
 * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf). \f{align*}{
 *  \bar{q}^{-1} = \begin{bmatrix} -\mathbf{q} \\ q_4 \end{bmatrix}
 * \f}
 *
 * @param[in] q quaternion we want to change
 * @return inversed quaternion
 */
Eigen::Matrix<double, 4, 1> Inv(Eigen::Matrix<double, 4, 1> q);

/**
 * @brief Integrated quaternion from angular velocity
 *
 * See equation (48) of trawny tech report [Indirect Kalman Filter for 3D
 * Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
 *
 */
Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w);

/**
 * @brief Normalizes a quaternion to make sure it is unit norm
 * @param q_t Quaternion to normalized
 * @return Normalized quaterion
 */
Eigen::Matrix<double, 4, 1> QuatNorm(Eigen::Matrix<double, 4, 1> q_t);

/**
 * @brief Computes left Jacobian of SO(3)
 *
 * The left Jacobian of SO(3) is defined equation (7.77b) in [State Estimation
 * for Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) by
 * Timothy D. Barfoot. Specifically it is the following (with
 * \f$\theta=|\boldsymbol\theta|\f$ and \f$\mathbf
 * a=\boldsymbol\theta/|\boldsymbol\theta|\f$): \f{align*}{
 * J_l(\boldsymbol\theta) = \frac{\sin\theta}{\theta}\mathbf I +
 * \Big(1-\frac{\sin\theta}{\theta}\Big)\mathbf a \mathbf a^\top +
 * \frac{1-\cos\theta}{\theta}\lfloor \mathbf a \times\rfloor \f}
 *
 * @param w axis-angle
 * @return The left Jacobian of SO(3)
 */
Eigen::Matrix<double, 3, 3> JlSo3(const Eigen::Matrix<double, 3, 1>& w);

/**
 * @brief Computes right Jacobian of SO(3)
 *
 * The right Jacobian of SO(3) is related to the left by Jl(-w)=Jr(w).
 * See equation (7.87) in [State Estimation for
 * Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) by
 * Timothy D. Barfoot. See @ref Jl_so3() for the definition of the left Jacobian
 * of SO(3).
 *
 * @param w axis-angle
 * @return The right Jacobian of SO(3)
 */
Eigen::Matrix<double, 3, 3> JrSo3(const Eigen::Matrix<double, 3, 1>& w);

/**
 * @brief Gets roll, pitch, yaw of argument rotation (in that order).
 *
 * To recover the matrix: R_input = R_z(yaw)*R_y(pitch)*R_x(roll)
 * If you are interested in how to compute Jacobians checkout this report:
 * http://mars.cs.umn.edu/tr/reports/Trawny05c.pdf
 *
 * @param rot Rotation matrix
 * @return roll, pitch, yaw values (in that order)
 */
Eigen::Matrix<double, 3, 1> Rot2Rpy(const Eigen::Matrix<double, 3, 3>& rot);

/**
 * @brief Construct rotation matrix from given roll
 * @param t roll angle
 */
Eigen::Matrix<double, 3, 3> RotX(double t);

/**
 * @brief Construct rotation matrix from given pitch
 * @param t pitch angle
 */
Eigen::Matrix<double, 3, 3> RotY(double t);

/**
 * @brief Construct rotation matrix from given yaw
 * @param t yaw angle
 */
Eigen::Matrix<double, 3, 3> RotZ(double t);


// *************************************************
// OPENVINS FUNCTION DONE
// *************************************************


/** @} group utils */
} // namespace beam
