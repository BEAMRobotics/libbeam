#include <beam_utils/se3.h>

namespace beam {

Eigen::Matrix<double, 4, 1> Rot2Quat(const Eigen::Matrix<double, 3, 3>& rot) {
  Eigen::Matrix<double, 4, 1> q;
  double T = rot.trace();
  if ((rot(0, 0) >= T) && (rot(0, 0) >= rot(1, 1)) &&
      (rot(0, 0) >= rot(2, 2))) {
    q(0) = sqrt((1 + (2 * rot(0, 0)) - T) / 4);
    q(1) = (1 / (4 * q(0))) * (rot(0, 1) + rot(1, 0));
    q(2) = (1 / (4 * q(0))) * (rot(0, 2) + rot(2, 0));
    q(3) = (1 / (4 * q(0))) * (rot(1, 2) - rot(2, 1));

  } else if ((rot(1, 1) >= T) && (rot(1, 1) >= rot(0, 0)) &&
             (rot(1, 1) >= rot(2, 2))) {
    q(1) = sqrt((1 + (2 * rot(1, 1)) - T) / 4);
    q(0) = (1 / (4 * q(1))) * (rot(0, 1) + rot(1, 0));
    q(2) = (1 / (4 * q(1))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(1))) * (rot(2, 0) - rot(0, 2));
  } else if ((rot(2, 2) >= T) && (rot(2, 2) >= rot(0, 0)) &&
             (rot(2, 2) >= rot(1, 1))) {
    q(2) = sqrt((1 + (2 * rot(2, 2)) - T) / 4);
    q(0) = (1 / (4 * q(2))) * (rot(0, 2) + rot(2, 0));
    q(1) = (1 / (4 * q(2))) * (rot(1, 2) + rot(2, 1));
    q(3) = (1 / (4 * q(2))) * (rot(0, 1) - rot(1, 0));
  } else {
    q(3) = sqrt((1 + T) / 4);
    q(0) = (1 / (4 * q(3))) * (rot(1, 2) - rot(2, 1));
    q(1) = (1 / (4 * q(3))) * (rot(2, 0) - rot(0, 2));
    q(2) = (1 / (4 * q(3))) * (rot(0, 1) - rot(1, 0));
  }
  if (q(3) < 0) { q = -q; }
  // normalize and return
  q = q / (q.norm());
  return q;
}

Eigen::Matrix<double, 3, 3> SkewX(const Eigen::Matrix<double, 3, 1>& w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}

Eigen::Matrix<double, 3, 3> Quat2Rot(const Eigen::Matrix<double, 4, 1>& q) {
  Eigen::Matrix<double, 3, 3> q_x = SkewX(q.block(0, 0, 3, 1));
  Eigen::MatrixXd Rot =
      (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) -
      2 * q(3, 0) * q_x +
      2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
  return Rot;
}

Eigen::Matrix<double, 4, 1> QuatMultiply(const Eigen::Matrix<double, 4, 1>& q,
                                         const Eigen::Matrix<double, 4, 1>& p) {
  Eigen::Matrix<double, 4, 1> q_t;
  Eigen::Matrix<double, 4, 4> Qm;
  // create big L matrix
  Qm.block(0, 0, 3, 3) =
      q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - SkewX(q.block(0, 0, 3, 1));
  Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
  Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
  Qm(3, 3) = q(3, 0);
  q_t = Qm * p;
  // ensure unique by forcing q_4 to be >0
  if (q_t(3, 0) < 0) { q_t *= -1; }
  // normalize and return
  return q_t / q_t.norm();
}

Eigen::Matrix<double, 3, 1> Vee(const Eigen::Matrix<double, 3, 3>& w_x) {
  Eigen::Matrix<double, 3, 1> w;
  w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
  return w;
}

Eigen::Matrix<double, 3, 3> ExpSo3(const Eigen::Matrix<double, 3, 1>& w) {
  // get theta
  Eigen::Matrix<double, 3, 3> w_x = SkewX(w);
  double theta = w.norm();
  // Handle small angle values
  double A, B;
  if (theta < 1e-7) {
    A = 1;
    B = 0.5;
  } else {
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
  }
  // compute so(3) rotation
  Eigen::Matrix<double, 3, 3> R;
  if (theta == 0) {
    R = Eigen::MatrixXd::Identity(3, 3);
  } else {
    R = Eigen::MatrixXd::Identity(3, 3) + A * w_x + B * w_x * w_x;
  }
  return R;
}

Eigen::Matrix<double, 3, 1> LogSo3(const Eigen::Matrix<double, 3, 3>& R) {
  // note switch to base 1
  double R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  double R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  double R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const double tr = R.trace();
  Eigen::Vector3d omega;

  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (tr + 1.0 < 1e-10) {
    if (std::abs(R33 + 1.0) > 1e-5)
      omega =
          (M_PI / sqrt(2.0 + 2.0 * R33)) * Eigen::Vector3d(R13, R23, 1.0 + R33);
    else if (std::abs(R22 + 1.0) > 1e-5)
      omega =
          (M_PI / sqrt(2.0 + 2.0 * R22)) * Eigen::Vector3d(R12, 1.0 + R22, R32);
    else
      // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
      omega =
          (M_PI / sqrt(2.0 + 2.0 * R11)) * Eigen::Vector3d(1.0 + R11, R21, R31);
  } else {
    double magnitude;
    const double tr_3 = tr - 3.0; // always negative
    if (tr_3 < -1e-7) {
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      // see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0;
    }
    omega = magnitude * Eigen::Vector3d(R32 - R23, R13 - R31, R21 - R12);
  }

  return omega;
}

Eigen::Matrix4d ExpSe3(Eigen::Matrix<double, 6, 1> vec) {
  // Precompute our values
  Eigen::Vector3d w = vec.head(3);
  Eigen::Vector3d u = vec.tail(3);
  double theta = sqrt(w.dot(w));
  Eigen::Matrix3d wskew;
  wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  // Handle small angle values
  double A, B, C;
  if (theta < 1e-7) {
    A = 1;
    B = 0.5;
    C = 1.0 / 6.0;
  } else {
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
    C = (1 - A) / (theta * theta);
  }

  // Matrices we need V and Identity
  Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V = I_33 + B * wskew + C * wskew * wskew;

  // Get the final matrix to return
  Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
  mat.block(0, 0, 3, 3) = I_33 + A * wskew + B * wskew * wskew;
  mat.block(0, 3, 3, 1) = V * u;
  mat(3, 3) = 1;
  return mat;
}

Eigen::Matrix<double, 6, 1> LogSe3(Eigen::Matrix4d mat) {
  Eigen::Vector3d w = LogSo3(mat.block<3, 3>(0, 0));
  Eigen::Vector3d T = mat.block<3, 1>(0, 3);
  const double t = w.norm();
  if (t < 1e-10) {
    Eigen::Matrix<double, 6, 1> log;
    log << w, T;
    return log;
  } else {
    Eigen::Matrix3d W = SkewX(w / t);
    // Formula from Agrawal06iros, equation (14)
    // simplified with Mathematica, and multiplying in T to avoid matrix math
    double Tan = tan(0.5 * t);
    Eigen::Vector3d WT = W * T;
    Eigen::Vector3d u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
    Eigen::Matrix<double, 6, 1> log;
    log << w, u;
    return log;
  }
}

Eigen::Matrix4d HatSe3(const Eigen::Matrix<double, 6, 1>& vec) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
  mat.block(0, 0, 3, 3) = SkewX(vec.head(3));
  mat.block(0, 3, 3, 1) = vec.tail(3);
  return mat;
}

Eigen::Matrix4d InvSe3(const Eigen::Matrix4d& T) {
  Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
  Tinv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
  Tinv.block(0, 3, 3, 1) = -Tinv.block(0, 0, 3, 3) * T.block(0, 3, 3, 1);
  return Tinv;
}

Eigen::Matrix<double, 4, 1> Inv(Eigen::Matrix<double, 4, 1> q) {
  Eigen::Matrix<double, 4, 1> qinv;
  qinv.block(0, 0, 3, 1) = -q.block(0, 0, 3, 1);
  qinv(3, 0) = q(3, 0);
  return qinv;
}

Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w) {
  Eigen::Matrix<double, 4, 4> mat;
  mat.block(0, 0, 3, 3) = -SkewX(w);
  mat.block(3, 0, 1, 3) = -w.transpose();
  mat.block(0, 3, 3, 1) = w;
  mat(3, 3) = 0;
  return mat;
}

Eigen::Matrix<double, 4, 1> QuatNorm(Eigen::Matrix<double, 4, 1> q_t) {
  if (q_t(3, 0) < 0) { q_t *= -1; }
  return q_t / q_t.norm();
}

Eigen::Matrix<double, 3, 3> JlSo3(const Eigen::Matrix<double, 3, 1>& w) {
  double theta = w.norm();
  if (theta < 1e-6) {
    return Eigen::MatrixXd::Identity(3, 3);
  } else {
    Eigen::Matrix<double, 3, 1> a = w / theta;
    Eigen::Matrix<double, 3, 3> J =
        sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) +
        (1 - sin(theta) / theta) * a * a.transpose() +
        ((1 - cos(theta)) / theta) * SkewX(a);
    return J;
  }
}

Eigen::Matrix<double, 3, 3> JrSo3(const Eigen::Matrix<double, 3, 1>& w) {
  return JlSo3(-w);
}

Eigen::Matrix<double, 3, 1> Rot2Rpy(const Eigen::Matrix<double, 3, 3>& rot) {
  Eigen::Matrix<double, 3, 1> rpy;
  rpy(1, 0) =
      atan2(-rot(2, 0), sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
  if (std::abs(cos(rpy(1, 0))) > 1.0e-12) {
    rpy(2, 0) = atan2(rot(1, 0) / cos(rpy(1, 0)), rot(0, 0) / cos(rpy(1, 0)));
    rpy(0, 0) = atan2(rot(2, 1) / cos(rpy(1, 0)), rot(2, 2) / cos(rpy(1, 0)));
  } else {
    rpy(2, 0) = 0;
    rpy(0, 0) = atan2(rot(0, 1), rot(1, 1));
  }
  return rpy;
}

Eigen::Matrix<double, 3, 3> RotX(double t) {
  Eigen::Matrix<double, 3, 3> r;
  double ct = cos(t);
  double st = sin(t);
  r << 1.0, 0.0, 0.0, 0.0, ct, -st, 0.0, st, ct;
  return r;
}

Eigen::Matrix<double, 3, 3> RotY(double t) {
  Eigen::Matrix<double, 3, 3> r;
  double ct = cos(t);
  double st = sin(t);
  r << ct, 0.0, st, 0.0, 1.0, 0.0, -st, 0.0, ct;
  return r;
}

Eigen::Matrix<double, 3, 3> RotZ(double t) {
  Eigen::Matrix<double, 3, 3> r;
  double ct = cos(t);
  double st = sin(t);
  r << ct, -st, 0.0, st, ct, 0.0, 0.0, 0.0, 1.0;
  return r;
}

int euler2rot(const Eigen::Vector3d& euler, int euler_seq, Eigen::Matrix3d& R) {
  double R11, R12, R13, R21, R22, R23, R31, R32, R33;
  double phi, theta, psi;

  phi = euler(0);
  theta = euler(1);
  psi = euler(2);

  if (euler_seq == 321) {
    // euler 3-2-1
    R11 = cos(theta) * cos(psi);
    R12 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R13 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);

    R21 = cos(theta) * sin(psi);
    R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R23 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);

    R31 = -sin(theta);
    R32 = sin(phi) * cos(theta);
    R33 = cos(phi) * cos(theta);

  } else if (euler_seq == 123) {
    // euler 1-2-3
    R11 = cos(theta) * cos(psi);
    R12 = cos(theta) * sin(psi);
    R13 = -sin(theta);

    R21 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R23 = sin(phi) * cos(theta);

    R31 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    R32 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    R33 = cos(phi) * cos(theta);

  } else {
    return -1;
  }

  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;

  return 0;
}

int euler2quat(const Eigen::Vector3d& euler, int euler_seq,
               Eigen::Quaterniond& q) {
  double alpha, beta, gamma;
  double c1, c2, c3, s1, s2, s3;
  double w, x, y, z;

  alpha = euler(0);
  beta = euler(1);
  gamma = euler(2);

  c1 = cos(alpha / 2.0);
  c2 = cos(beta / 2.0);
  c3 = cos(gamma / 2.0);
  s1 = sin(alpha / 2.0);
  s2 = sin(beta / 2.0);
  s3 = sin(gamma / 2.0);

  switch (euler_seq) {
    case 123:
      // euler 1-2-3 to quaternion
      w = c1 * c2 * c3 - s1 * s2 * s3;
      x = s1 * c2 * c3 + c1 * s2 * s3;
      y = c1 * s2 * c3 - s1 * c2 * s3;
      z = c1 * c2 * s3 + s1 * s2 * c3;
      break;

    case 321:
      // euler 3-2-1 to quaternion
      w = c1 * c2 * c3 + s1 * s2 * s3;
      x = s1 * c2 * c3 - c1 * s2 * s3;
      y = c1 * s2 * c3 + s1 * c2 * s3;
      z = c1 * c2 * s3 - s1 * s2 * c3;
      break;

    default:
      printf("Error! Invalid euler sequence [%d]\n", euler_seq);
      return -1;
      break;
  }

  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;

  return 0;
}

int quat2euler(const Eigen::Quaterniond& q, int euler_seq,
               Eigen::Vector3d& euler) {
  double qw, qx, qy, qz;
  double qw2, qx2, qy2, qz2;
  double phi, theta, psi;

  qw = q.w();
  qx = q.x();
  qy = q.y();
  qz = q.z();

  qw2 = pow(qw, 2);
  qx2 = pow(qx, 2);
  qy2 = pow(qy, 2);
  qz2 = pow(qz, 2);

  if (euler_seq == 123) {
    // euler 1-2-3
    phi = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2));
    theta = asin(2 * (qx * qz + qy * qw));
    psi = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2));

  } else if (euler_seq == 321) {
    // euler 3-2-1
    phi = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
    theta = asin(2 * (qy * qw - qx * qz));
    psi = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  } else {
    return -1;
  }

  euler << phi, theta, psi;
  return 0;
}

int quat2rot(const Eigen::Quaterniond& q, Eigen::Matrix3d& R) {
  double qw = q.w();
  double qx = q.x();
  double qy = q.y();
  double qz = q.z();

  // double qw2 = pow(qw, 2);
  double qx2 = pow(qx, 2);
  double qy2 = pow(qy, 2);
  double qz2 = pow(qz, 2);

  // inhomogeneous form
  double R11 = 1 - 2 * qy2 - 2 * qz2;
  double R12 = 2 * qx * qy + 2 * qz * qw;
  double R13 = 2 * qx * qz - 2 * qy * qw;

  double R21 = 2 * qx * qy - 2 * qz * qw;
  double R22 = 1 - 2 * qx2 - 2 * qz2;
  double R23 = 2 * qy * qz + 2 * qx * qw;

  double R31 = 2 * qx * qz + 2 * qy * qw;
  double R32 = 2 * qy * qz - 2 * qx * qw;
  double R33 = 1 - 2 * qx2 - 2 * qy2;

  // // homogeneous form
  // R11 = qx2 + qx2 - qy2 - qz2;
  // R12 = 2 * (qx * qy - qw * qz);
  // R13 = 2 * (qw * qy + qx * qz);
  //
  // R21 = 2 * (qx * qy + qw * qz);
  // R22 = qw2 - qx2 + qy2 - qz2;
  // R23 = 2 * (qy * qz - qw * qx);
  //
  // R31 = 2 * (qx * qz - qw * qy);
  // R32 = 2 * (qw * qx + qy * qz);
  // R33 = qw2 - qx2 - qy2 + qz2;

  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;

  return 0;
}

Eigen::Matrix3d RightJacobianOfSO3(const Eigen::Vector3d& w) {
  static const double root2_eps = sqrt(std::numeric_limits<double>::epsilon());
  static const double root4_eps = sqrt(root2_eps);
  static const double qdrt720 = sqrt(sqrt(720.0));
  static const double qdrt5040 = sqrt(sqrt(5040.0));
  static const double sqrt24 = sqrt(24.0);
  static const double sqrt120 = sqrt(120.0);

  double angle = w.norm();
  double cangle = cos(angle);
  double sangle = sin(angle);
  double angle2 = angle * angle;

  double cos_term;
  // compute (1-cos(x))/x^2, its taylor expansion around 0 is
  // 1/2-x^2/24+x^4/720+o(x^6)
  if (angle > root4_eps * qdrt720) {
    cos_term = (1 - cangle) / angle2;
  } else { // use taylor expansion to avoid singularity
    cos_term = 0.5;
    if (angle > root2_eps * sqrt24) { // we have to include x^2 term
      cos_term -= angle2 / 24.0;
    }
  }

  double sin_term;
  // compute (x-sin(x))/x^3, its taylor expansion around 0 is
  // 1/6-x^2/120+x^4/5040+o(x^6)
  if (angle > root4_eps * qdrt5040) {
    sin_term = (angle - sangle) / (angle * angle2);
  } else {
    sin_term = 1.0 / 6.0;
    if (angle > root2_eps * sqrt120) { // we have to include x^2 term
      sin_term -= angle2 / 120.0;
    }
  }

  Eigen::Matrix3d hat_w = beam::SkewTransform(w);
  return Eigen::Matrix3d::Identity() - cos_term * hat_w +
         sin_term * hat_w * hat_w;
}

bool IsTransformationMatrix(const Eigen::Matrix4d& T, std::string& summary,
                            int precision) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);

  // check for infinity or nan
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      if (std::isinf(T(i, j))) {
        summary = "Inf value found at coord. (" + std::to_string(i) + "" +
                  std::to_string(j) + ").";
        return false;
      }
      if (std::isnan(T(i, j))) {
        summary = "nan value found at coord. (" + std::to_string(i) + "" +
                  std::to_string(j) + ").";
        return false;
      }
    }
  }

  // check that bottom row is [0 0 0 1]
  if (T(3, 0) != 0 || T(3, 1) != 0 || T(3, 2) != 0 || T(3, 3) != 1) {
    summary = "Bottom row not equal to [0 0 0 1]";
    return false;
  }

  return IsRotationMatrix(R, summary, precision);
}

bool IsRotationMatrix(const Eigen::Matrix3d& R, std::string& summary,
                      int precision) {
  Eigen::Matrix3d shouldBeIdentity = RoundMatrix(R * R.transpose(), precision);
  if (!shouldBeIdentity.isIdentity()) {
    summary = "R R^T != Identity";
    return false;
  }

  double detR = R.determinant();
  double detRRound = std::round(detR * precision) / precision;
  if (detRRound != 1) {
    summary = "Det(R) != 1";
    return false;
  }

  return true;
}

Eigen::Vector3d RToLieAlgebra(const Eigen::Matrix3d& R) {
  return InvSkewTransform(R.log());
}

Eigen::Vector3d QToLieAlgebra(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return RToLieAlgebra(R);
}

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps) {
  return SkewTransform(eps).exp();
}

Eigen::Quaterniond LieAlgebraToQ(const Eigen::Vector3d& eps) {
  Eigen::Quaterniond q(LieAlgebraToR(eps));
  return q.normalized();
}

Eigen::Matrix4d InterpolateTransform(const Eigen::Matrix4d& m1,
                                     const beam::TimePoint& t1,
                                     const Eigen::Matrix4d& m2,
                                     const beam::TimePoint& t2,
                                     const beam::TimePoint& t) {
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  Eigen::Matrix4d T1 = m1;
  Eigen::Matrix4d T2 = m2;
  Eigen::Matrix4d T;

  Eigen::Matrix3d R1 = T1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = T2.block<3, 3>(0, 0);
  Eigen::Matrix3d R = (R2 * R1.transpose()).pow(w2) * R1;

  Eigen::Vector4d tr1 = T1.rightCols<1>();
  Eigen::Vector4d tr2 = T2.rightCols<1>();
  Eigen::Vector4d tr = (1 - w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = tr;

  return T;
}

Eigen::Matrix4d InterpolateTransform(const Eigen::Matrix4d& m1,
                                     const double& t1,
                                     const Eigen::Matrix4d& m2,
                                     const double& t2, const double& t) {
  double w2 = 1.0 * (t - t1) / (t2 - t1);

  Eigen::Matrix4d T1 = m1;
  Eigen::Matrix4d T2 = m2;
  Eigen::Matrix4d T;

  Eigen::Matrix3d R1 = T1.block<3, 3>(0, 0);
  Eigen::Matrix3d R2 = T2.block<3, 3>(0, 0);
  Eigen::Matrix3d R = (R2 * R1.transpose()).pow(w2) * R1;

  Eigen::Vector4d tr1 = T1.rightCols<1>();
  Eigen::Vector4d tr2 = T2.rightCols<1>();
  Eigen::Vector4d tr = (1 - w2) * tr1 + w2 * tr2;

  T.setIdentity();
  T.block<3, 3>(0, 0) = R;
  T.rightCols<1>() = tr;

  return T;
}

Eigen::Vector3d InvSkewTransform(const Eigen::Matrix3d& M) {
  Eigen::Vector3d V;
  V(0) = M(2, 1);
  V(1) = M(0, 2);
  V(2) = M(1, 0);
  return V;
}

Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V) {
  Eigen::Matrix3d M;
  M(0, 0) = 0;
  M(0, 1) = -V(2, 0);
  M(0, 2) = V(1, 0);
  M(1, 0) = V(2, 0);
  M(1, 1) = 0;
  M(1, 2) = -V(0, 0);
  M(2, 0) = -V(1, 0);
  M(2, 1) = V(0, 0);
  M(2, 2) = 0;
  return M;
}

Eigen::Matrix4d InvertTransform(const Eigen::MatrixXd& T) {
  Eigen::Matrix4d T_inv;
  T_inv.setIdentity();
  T_inv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
  T_inv.block(0, 3, 3, 1) =
      -T.block(0, 0, 3, 3).transpose() * T.block(0, 3, 3, 1);
  return T_inv;
}

Eigen::Matrix4d RelativeTransform(const Eigen::Matrix4d& T_W_A,
                                  const Eigen::Matrix4d& T_W_B) {
  Eigen::Matrix4d W_T_A_B;
  Eigen::Matrix3d R_W_A = T_W_A.block<3, 3>(0, 0);
  Eigen::Matrix3d R_W_B = T_W_B.block<3, 3>(0, 0);
  Eigen::Matrix3d R_A_B = R_W_A.transpose() * R_W_B;
  Eigen::Vector3d t_A_B =
      T_W_A.block<3, 1>(0, 3).transpose() - T_W_B.block<3, 1>(0, 3).transpose();
  W_T_A_B.block<3, 3>(0, 0) = R_A_B;
  W_T_A_B.block<3, 1>(0, 3) = t_A_B.transpose();
  return W_T_A_B;
}

Eigen::Matrix4d AverageTransforms(
    const std::vector<Eigen::Matrix4d, AlignMat4d>& transforms) {
  if (transforms.size() == 1) { return transforms[0]; }

  std::vector<double> sum(6);
  for (const Eigen::Matrix4d T : transforms) {
    Eigen::Vector3d r = beam::RToLieAlgebra(T.block(0, 0, 3, 3));
    sum[0] += T(0, 3);
    sum[1] += T(1, 3);
    sum[2] += T(2, 3);
    sum[3] += r[0];
    sum[4] += r[1];
    sum[5] += r[2];
  }

  Eigen::Matrix4d T_AVG = Eigen::Matrix4d::Identity();
  Eigen::Vector3d r(sum[3] / transforms.size(), sum[4] / transforms.size(),
                    sum[5] / transforms.size());
  T_AVG.block(0, 0, 3, 3) = beam::LieAlgebraToR(r);
  T_AVG(0, 3) = sum[0] / transforms.size();
  T_AVG(1, 3) = sum[1] / transforms.size();
  T_AVG(2, 3) = sum[2] / transforms.size();

  return T_AVG;
}

Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations) {
  Eigen::Vector3d r_perturb = perturbations.block(0, 0, 3, 1);
  Eigen::Vector3d t_perturb = perturbations.block(3, 0, 3, 1);
  Eigen::Matrix3d R_in = T_in.block(0, 0, 3, 3);
  Eigen::Matrix3d R_out = LieAlgebraToR(r_perturb) * R_in;
  Eigen::Matrix4d T_out;
  T_out.setIdentity();
  T_out.block(0, 3, 3, 1) = T_in.block(0, 3, 3, 1) + t_perturb;
  T_out.block(0, 0, 3, 3) = R_out;
  return T_out;
}

Eigen::Matrix4d PerturbTransformDegM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations) {
  Eigen::VectorXd perturbations_rad(perturbations);
  perturbations_rad[0] = beam::Deg2Rad(perturbations_rad[0]);
  perturbations_rad[1] = beam::Deg2Rad(perturbations_rad[1]);
  perturbations_rad[2] = beam::Deg2Rad(perturbations_rad[2]);
  return PerturbTransformRadM(T_in, perturbations_rad);
}

void RPYtoQuaternion(double roll, double pitch, double yaw,
                     Eigen::Quaterniond& q) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  q = rollAngle * pitchAngle * yawAngle;
  q.normalize();
}

void RPYtoQuaternion(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q) {
  RPYtoQuaternion(rpy[0], rpy[1], rpy[2], q);
}

void RPYtoQuaternionDeg(double roll, double pitch, double yaw,
                        Eigen::Quaterniond& q) {
  RPYtoQuaternion(beam::Deg2Rad(roll), beam::Deg2Rad(pitch), beam::Deg2Rad(yaw),
                  q);
}

void RPYtoQuaternionDeg(const Eigen::Vector3d& rpy, Eigen::Quaterniond& q) {
  RPYtoQuaternion(beam::Deg2Rad(rpy[0]), beam::Deg2Rad(rpy[1]),
                  beam::Deg2Rad(rpy[2]), q);
}

void QuaterniontoRPY(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy) {
  rpy = q.toRotationMatrix().eulerAngles(0, 1, 2); // roll pitch yaw order
}

void QuaterniontoRPY(const Eigen::Quaterniond& q, double& roll, double& pitch,
                     double& yaw) {
  Eigen::Vector3d rpy;
  QuaterniontoRPY(q, rpy);
  roll = rpy[0];
  pitch = rpy[1];
  yaw = rpy[2];
}

void QuaterniontoRPYDeg(const Eigen::Quaterniond& q, Eigen::Vector3d& rpy) {
  QuaterniontoRPY(q, rpy);
  rpy[0] = beam::Rad2Deg(rpy[0]);
  rpy[1] = beam::Rad2Deg(rpy[1]);
  rpy[2] = beam::Rad2Deg(rpy[2]);
}

void QuaterniontoRPYDeg(const Eigen::Quaterniond& q, double& roll,
                        double& pitch, double& yaw) {
  QuaterniontoRPY(q, roll, pitch, yaw);
  roll = beam::Rad2Deg(roll);
  pitch = beam::Rad2Deg(pitch);
  yaw = beam::Rad2Deg(yaw);
}

Eigen::Matrix4d
    QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose) {
  Eigen::Quaterniond quaternion{pose[0], pose[1], pose[2], pose[3]};
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = quaternion.toRotationMatrix();
  T(0, 3) = pose[4];
  T(1, 3) = pose[5];
  T(2, 3) = pose[6];
  return T;
}

std::vector<double>
    TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaterniond q = Eigen::Quaterniond(R);
  std::vector<double> pose{q.w(),   q.x(),   q.y(),  q.z(),
                           T(0, 3), T(1, 3), T(2, 3)};
  return pose;
}

void QuaternionAndTranslationToTransformMatrix(const Eigen::Quaterniond& q,
                                               const Eigen::Vector3d& p,
                                               Eigen::Matrix4d& T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = p.transpose();
}

void TransformMatrixToQuaternionAndTranslation(const Eigen::Matrix4d& T,
                                               Eigen::Quaterniond& q,
                                               Eigen::Vector3d& p) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Quaterniond q_tmp(R);
  q = q_tmp;
  p = T.block<3, 1>(0, 3).transpose();
}

bool PassedMotionThreshold(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                           double angle_threshold_deg,
                           double translation_threshold_m,
                           bool is_threshold_minimum, bool must_pass_both,
                           bool output_error_cause) {
  Eigen::Matrix4d T_DIFF = beam::InvertTransform(T1) * T2;
  Eigen::Matrix3d R_DIFF = T_DIFF.block(0, 0, 3, 3);
  double angle =
      beam::Rad2Deg(std::abs(Eigen::AngleAxis<double>(R_DIFF).angle()));
  double translation = T_DIFF.block(0, 3, 3, 1).norm();

  if (is_threshold_minimum) {
    bool result1 = angle > angle_threshold_deg;
    bool result2 = translation > translation_threshold_m;
    if (!result1 && output_error_cause) {
      std::cout << "Angle: " << angle
                << " <= Threshold: " << angle_threshold_deg << "\n";
    }
    if (!result2 && output_error_cause) {
      std::cout << "Trans: " << translation
                << " <= Threshold: " << translation_threshold_m << "\n";
    }
    if (must_pass_both) {
      return result1 && result2;
    } else {
      return result1 || result2;
    }
  } else {
    bool result1 = angle < angle_threshold_deg;
    bool result2 = translation < translation_threshold_m;
    if (!result1 && output_error_cause) {
      std::cout << "Angle: " << angle
                << " >= Threshold: " << angle_threshold_deg << "\n";
    }
    if (!result2 && output_error_cause) {
      std::cout << "Trans: " << translation
                << " >= Threshold: " << translation_threshold_m << "\n";
    }
    if (must_pass_both) {
      return result1 && result2;
    } else {
      return result1 || result2;
    }
  }
}

bool ArePosesEqual(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                   double angle_threshold_deg, double translation_threshold_m,
                   bool output_error_cause) {
  return PassedMotionThreshold(T1, T2, angle_threshold_deg,
                               translation_threshold_m, false, true,
                               output_error_cause);
}

bool PassedMinMotion(const Eigen::Matrix4d& T1, const Eigen::Matrix4d& T2,
                     double angle_threshold_deg, double translation_threshold_m,
                     bool output_error_cause) {
  return PassedMotionThreshold(T1, T2, angle_threshold_deg,
                               translation_threshold_m, true, false,
                               output_error_cause);
}

Eigen::Matrix4f VectorToEigenTransform(const std::vector<float>& v) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 0) = v[0];
  T(0, 1) = v[1];
  T(0, 2) = v[2];
  T(0, 3) = v[3];
  T(1, 0) = v[4];
  T(1, 1) = v[5];
  T(1, 2) = v[6];
  T(1, 3) = v[7];
  T(2, 0) = v[8];
  T(2, 1) = v[9];
  T(2, 2) = v[10];
  T(2, 3) = v[11];
  return T;
}

Eigen::Matrix4d VectorToEigenTransform(const std::vector<double>& v) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = v[0];
  T(0, 1) = v[1];
  T(0, 2) = v[2];
  T(0, 3) = v[3];
  T(1, 0) = v[4];
  T(1, 1) = v[5];
  T(1, 2) = v[6];
  T(1, 3) = v[7];
  T(2, 0) = v[8];
  T(2, 1) = v[9];
  T(2, 2) = v[10];
  T(2, 3) = v[11];
  return T;
}

std::vector<double> EigenTransformToVector(const Eigen::Matrix4d& T) {
  std::vector<double> v;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) { v.push_back(T(i, j)); }
  }
  return v;
}

std::vector<float> EigenTransformToVector(const Eigen::Matrix4f& T) {
  std::vector<float> v;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) { v.push_back(T(i, j)); }
  }
  return v;
}

void OutputTransformInformation(const Eigen::Matrix4d& T,
                                std::ostream& stream) {
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
  TransformMatrixToQuaternionAndTranslation(T, q, t);
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  stream << "T: \n"
         << T << "\n"
         << "RPY (DEG): [" << Rad2Deg(rpy[0]) << ", " << Rad2Deg(rpy[1]) << ", "
         << Rad2Deg(rpy[3]) << "]\n"
         << "Quat (wxyz): [" << q.w() << ", " << q.x() << ", " << q.y() << ", "
         << q.z() << "]\n"
         << "Trans: [" << t[0] << ", " << t[1] << ", " << t[2] << "]\n";
}

std::string TransformationMatrixToString(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d p = T.block<3, 1>(0, 3).transpose();
  Eigen::Vector3d ea = R.eulerAngles(2, 1, 0);
  std::string output;
  output += "Roll: " + std::to_string(ea[0]);
  output += ", Pitch: " + std::to_string(ea[1]);
  output += ", Yaw: " + std::to_string(ea[2]);
  output += ", x: " + std::to_string(p[0]);
  output += ", y: " + std::to_string(p[1]);
  output += ", z: " + std::to_string(p[2]);
  return output;
}

} // namespace beam
