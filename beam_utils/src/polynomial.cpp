#include "beam_utils/polynomial.h"

#include <cmath>

namespace beam {

using Eigen::MatrixXd;
using Eigen::VectorXcd;
using Eigen::VectorXd;

// Remove leading terms with zero coefficients.
VectorXd RemoveLeadingZeros(const VectorXd& polynomial_in) {
  int i = 0;
  while (i < (polynomial_in.size() - 1) && polynomial_in(i) == 0) { ++i; }
  return polynomial_in.tail(polynomial_in.size() - i);
}

VectorXd DifferentiatePolynomial(const VectorXd& polynomial) {
  const int degree = polynomial.rows() - 1;
  // Degree zero polynomials are constants, and their derivative does
  // not result in a smaller degree polynomial, just a degree zero
  // polynomial with value zero.
  if (degree == 0) { return VectorXd::Zero(1); }

  VectorXd derivative(degree);
  for (int i = 0; i < degree; ++i) {
    derivative(i) = (degree - i) * polynomial(i);
  }

  return derivative;
}

VectorXd MultiplyPolynomials(const VectorXd& poly1, const VectorXd& poly2) {
  VectorXd multiplied_poly = VectorXd::Zero(poly1.size() + poly2.size() - 1);
  for (int i = 0; i < poly1.size(); i++) {
    for (int j = 0; j < poly2.size(); j++) {
      multiplied_poly.reverse()(i + j) +=
          poly1.reverse()(i) * poly2.reverse()(j);
    }
  }
  return multiplied_poly;
}

VectorXd AddPolynomials(const VectorXd& poly1, const VectorXd& poly2) {
  if (poly1.size() > poly2.size()) {
    VectorXd sum = poly1;
    sum.tail(poly2.size()) += poly2;
    return sum;
  } else {
    VectorXd sum = poly2;
    sum.tail(poly1.size()) += poly1;
    return sum;
  }
}

double FindRootIterativeNewton(const Eigen::VectorXd& polynomial,
                               const double x0, const double epsilon,
                               const int max_iterations) {
  double root = x0;
  const Eigen::VectorXd derivative = DifferentiatePolynomial(polynomial);
  double prev;
  for (int i = 0; i < max_iterations; i++) {
    prev = root;
    root -= EvaluatePolynomial(polynomial, root) /
            EvaluatePolynomial(derivative, root);
    if (std::abs(prev - root) < epsilon) { break; }
  }
  return root;
}

} // namespace beam
