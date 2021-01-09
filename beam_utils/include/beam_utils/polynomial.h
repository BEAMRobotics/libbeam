/** @file
 * @ingroup utils
 *
 * Utility for solving roots for polynomials
 */

#pragma once

#include <Eigen/Core>

namespace beam {

/**
 * @brief Remove leading terms with zero coefficients.
 */
Eigen::VectorXd RemoveLeadingZeros(const Eigen::VectorXd& polynomial_in);

/**
 * @brief Evaluate the polynomial at x using the Horner scheme.
 */
template <typename T>
inline T EvaluatePolynomial(const Eigen::VectorXd& polynomial, const T& x) {
  T v = 0.0;
  for (int i = 0; i < polynomial.size(); ++i) { v = v * x + polynomial(i); }
  return v;
}

/**
 * @brief  Return the derivative of the given polynomial. It is assumed that the
 * input polynomial is at least of degree zero.
 */
Eigen::VectorXd DifferentiatePolynomial(const Eigen::VectorXd& polynomial);

/**
 * @brief Multiplies the two polynoimals together.
 */
Eigen::VectorXd MultiplyPolynomials(const Eigen::VectorXd& poly1,
                                    const Eigen::VectorXd& poly2);

/**
 * @brief Adds two polynomials together.
 */
Eigen::VectorXd AddPolynomials(const Eigen::VectorXd& poly1,
                               const Eigen::VectorXd& poly2);

/**
 * @briefFind a root from the starting guess using Newton's method.
 */
double FindRootIterativeNewton(const Eigen::VectorXd& polynomial,
                               const double x0, const double epsilon,
                               const int max_iterations);

} // namespace beam
