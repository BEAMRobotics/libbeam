/** @file
 * @ingroup utils
 *
 * Utility for solving roots for polynomials
 */

#pragma once

#include <Eigen/Core>

namespace beam {

Eigen::VectorXd RemoveLeadingZeros(const Eigen::VectorXd& polynomial_in);

template <typename T>
inline T EvaluatePolynomial(const Eigen::VectorXd& polynomial, const T& x) {
  T v = 0.0;
  for (int i = 0; i < polynomial.size(); ++i) { v = v * x + polynomial(i); }
  return v;
}

Eigen::VectorXd DifferentiatePolynomial(const Eigen::VectorXd& polynomial);

Eigen::VectorXd MultiplyPolynomials(const Eigen::VectorXd& poly1,
                                    const Eigen::VectorXd& poly2);

Eigen::VectorXd AddPolynomials(const Eigen::VectorXd& poly1,
                               const Eigen::VectorXd& poly2);

double FindRootIterativeNewton(const Eigen::VectorXd& polynomial,
                               const double x0, const double epsilon,
                               const int max_iterations);

} // namespace beam
