/** @file
 * @ingroup utils
 *
 * Utility for solving roots for polynomials
 */

#pragma once

#include <Eigen/Core>

namespace beam {

enum ConvergenceType {
  NO_CONVERGENCE = 0,
  LINEAR_CONVERGENCE = 1,
  QUADRATIC_CONVERGENCE = 2
};

class JenkinsTraubSolver {
public:
  JenkinsTraubSolver(const Eigen::VectorXd& coeffs, Eigen::VectorXd* real_roots,
                     Eigen::VectorXd* complex_roots)
      : polynomial_(coeffs),
        real_roots_(real_roots),
        complex_roots_(complex_roots),
        num_solved_roots_(0) {}

  // Extracts the roots using the Jenkins Traub method.
  bool ExtractRoots();

private:
  /**
   * @brief Removes any zero roots and divides polynomial by z
   */
  void RemoveZeroRoots();

  /**
   * @brief Computes the magnitude of the roots to provide and initial search
   * radius for the iterative solver.
   */
  double ComputeRootRadius();

  /**
   * @brief Computes the zero-shift applied to the K-Polynomial.
   */
  void ComputeZeroShiftKPolynomial();

  /**
   * @brief Stage 1 of the Jenkins-Traub method. This stage is not technically
   * necessary, but helps separate roots that are close to zero.
   */
  void ApplyZeroShiftToKPolynomial(const int num_iterations);

  /**
   * @brief Computes and returns the update of sigma(z) based on the current
   * K-polynomial.
   * NOTE: This function is used by the fixed shift iterations (which hold
   * sigma constant) so sigma is *not* modified internally by this function. If
   * you want to change sigma, simply call sigma = ComputeNextSigma();
   */
  Eigen::VectorXd ComputeNextSigma();

  /**
   * @brief Updates the K-polynomial based on the current value of sigma for the
   * fixed or variable shift stage.
   */
  void UpdateKPolynomialWithQuadraticShift(
      const Eigen::VectorXd& polynomial_quotient,
      const Eigen::VectorXd& k_polynomial_quotient);

  /**
   * @brief Apply fixed-shift iterations to the K-polynomial to separate the
   * roots. Based on the convergence of the K-polynomial, we apply a
   * variable-shift linear or quadratic iteration to determine a real root or
   * complex conjugate pair of roots respectively.
   */
  ConvergenceType ApplyFixedShiftToKPolynomial(const std::complex<double>& root,
                                               const int max_iterations);

  /**
   * @brief Applies one of the variable shifts to the K-Polynomial. Returns
   * true upon successful convergence to a good root, and false otherwise.
   */
  bool ApplyVariableShiftToKPolynomial(
      const ConvergenceType& fixed_shift_convergence,
      const std::complex<double>& root);

  /**
   * @brief Applies a quadratic shift to the K-polynomial to determine a pair
   * of roots that are complex conjugates. Return true if a root was
   * successfully found.
   */
  bool ApplyQuadraticShiftToKPolynomial(const std::complex<double>& root,
                                        const int max_iterations);

  /**
   * @brief Applies a linear shift to the K-polynomial to determine a single
   * real root. Return true if a root was successfully found.
   */
  bool ApplyLinearShiftToKPolynomial(const std::complex<double>& root,
                                     const int max_iterations);

  /**
   * @brief Adds the root to the output variables.
   */
  void AddRootToOutput(const double real, const double imag);

  /**
   * @brief Solves polynomials of degree <= 2.
   */
  bool SolveClosedFormPolynomial();

  /**
   * @brief Determines if the root has converged by measuring the relative and
   * absolute change in the root value. This stopping criterion is a simple
   * measurement that proves to work well.
   */
  template <typename T>
  bool HasRootConverged(const std::vector<T>& roots) {
    static constexpr double kRootMagnitudeTolerance = 1e-8;
    if (roots.size() != 3) { return false; }

    const double e_i = std::abs(roots[2] - roots[1]);
    const double e_i_minus_1 = std::abs(roots[1] - roots[0]);
    const double mag_root = std::abs(roots[1]);
    if (e_i <= e_i_minus_1) {
      if (mag_root < kRootMagnitudeTolerance) {
        return e_i < kAbsoluteTolerance;
      } else {
        return e_i / mag_root <= kRelativeTolerance;
      }
    }

    return false;
  }

  /**
   * @brief Determines whether the iteration has converged by examining the
   * three most recent values for convergence.
   */
  template <typename T>
  bool HasConverged(const T& sequence) {
    const bool convergence_condition_1 =
        std::abs(sequence(1) - sequence(0)) < std::abs(sequence(0)) / 2.0;
    const bool convergence_condition_2 =
        std::abs(sequence(2) - sequence(1)) < std::abs(sequence(1)) / 2.0;

    // If the sequence has converged then return true.
    return convergence_condition_1 && convergence_condition_2;
  }

  /**
   * @brief Perform division of a polynomial by a quadratic factor. The
   * quadratic divisor should have leading 1s.
   */
  void QuadraticSyntheticDivision(const Eigen::VectorXd& polynomial,
                                  const Eigen::VectorXd& quadratic_divisor,
                                  Eigen::VectorXd* quotient,
                                  Eigen::VectorXd* remainder);

  /**
   * @brief Perform division by a linear term of the form (z - x) and evaluate P
   * at x.
   */
  void SyntheticDivisionAndEvaluate(const Eigen::VectorXd& polynomial,
                                    const double x, Eigen::VectorXd* quotient,
                                    double* eval);

  /**
   * @brief Solves for the root of the equation ax + b = 0.
   */
  double FindLinearPolynomialRoots(const double a, const double b);

  /**
   * @brief Stable quadratic roots according to BKP Horn.
   * http://people.csail.mit.edu/bkph/articles/Quadratics.pdf
   */
  void FindQuadraticPolynomialRoots(const double a, const double b,
                                    const double c,
                                    std::complex<double>* roots);

  /////////////////////////// Variables ////////////////////////////

  // Helper variables to manage the polynomials as they are being manipulated
  // and deflated.
  Eigen::VectorXd polynomial_;
  Eigen::VectorXd k_polynomial_;
  // Sigma is the quadratic factor the divides the K-polynomial.
  Eigen::Vector3d sigma_;

  // Let us define a, b, c, and d such that:
  //   P(z) = Q_P * sigma(z) + b * (z + u) + a
  //   K(z) = Q_K * sigma(z) + d * (z + u ) + c
  //
  // where Q_P and Q_K are the quotients from polynomial division of
  // sigma(z). Note that this means for a given a root s of sigma:
  //
  //   P(s)      = a - b * s_conj
  //   P(s_conj) = a - b * s
  //   K(s)      = c - d * s_conj
  //   K(s_conj) = c - d * s
  double a_, b_, c_, d_;

  // Output reference variables.
  Eigen::VectorXd* real_roots_;
  Eigen::VectorXd* complex_roots_;
  int num_solved_roots_;

  // Keeps track of whether the linear and quadratic shifts have been attempted
  // yet so that we do not attempt the same shift twice.
  bool attempted_linear_shift_;
  bool attempted_quadratic_shift_;

  // Number of zero-shift iterations to perform.
  static constexpr int kNumZeroShiftIterations = 20;

  // The number of fixed shift iterations is computed as
  //   # roots found * this multiplier.
  static constexpr int kFixedShiftIterationMultiplier = 20;

  // If the fixed shift iterations fail to converge, we restart this many times
  // before considering the solve attempt as a failure.
  static constexpr int kMaxFixedShiftRestarts = 20;

  // The maximum number of linear shift iterations to perform before considering
  // the shift as a failure.
  static constexpr int kMaxLinearShiftIterations = 20;

  // The maximum number of quadratic shift iterations to perform before
  // considering the shift as a failure.
  static constexpr int kMaxQuadraticShiftIterations = 20;

  // When quadratic shift iterations are stalling, we attempt a few fixed shift
  // iterations to help convergence.
  static constexpr int kInnerFixedShiftIterations = 5;

  // During quadratic iterations, the real values of the root pairs should be
  // nearly equal since the root pairs are complex conjugates. This tolerance
  // measures how much the real values may diverge before consider the quadratic
  // shift to be failed.
  static constexpr double kRootPairTolerance = 0.01;

  // Machine precision constants.
  static constexpr double mult_eps = std::numeric_limits<double>::epsilon();
  static constexpr double sum_eps = std::numeric_limits<double>::epsilon();
  static constexpr double kAbsoluteTolerance = 1e-14;
  static constexpr double kRelativeTolerance = 1e-10;
};

} // namespace beam
