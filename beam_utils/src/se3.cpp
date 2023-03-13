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

Eigen::Matrix<double, 3, 3>
    JlSo3(const Eigen::Matrix<double, 3, 1>& w) {
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

Eigen::Matrix<double, 3, 3>
    JrSo3(const Eigen::Matrix<double, 3, 1>& w) {
  return JlSo3(-w);
}

Eigen::Matrix<double, 3, 1>
    Rot2Rpy(const Eigen::Matrix<double, 3, 3>& rot) {
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

} // namespace beam
