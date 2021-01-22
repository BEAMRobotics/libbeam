#include "test_util.hpp"
#include <X11/Xlib.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace beam_optimization { namespace util {

double time_now(void) {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec) / 1000000.0);
}

double WrapToPi(double angle) {
  double wrapped_angle = WrapToTwoPi(angle + M_PI) - M_PI;
  return wrapped_angle;
}

double WrapToTwoPi(double angle) {
  double wrapped_angle = fmod(angle, 2 * M_PI);
  if (wrapped_angle < 0) { wrapped_angle += 2 * M_PI; }
  return wrapped_angle;
}

double DegToRad(double d) {
  return d * (M_PI / 180);
}

double RadToDeg(double r) {
  return r * (180 / M_PI);
}

double WrapTo180(double euler_angle) {
  return RadToDeg(WrapToPi(DegToRad(euler_angle)));
}

double WrapTo360(double euler_angle) {
  return RadToDeg(WrapToTwoPi(DegToRad(euler_angle)));
}

double GetSmallestAngleErrorDeg(double angle1, double angle2) {
  return RadToDeg(GetSmallestAngleErrorRad(DegToRad(angle1), DegToRad(angle2)));
}

double GetSmallestAngleErrorRad(double angle1, double angle2) {
  double angle1_wrapped = WrapToTwoPi(angle1);
  double angle2_wrapped = WrapToTwoPi(angle2);
  if (std::min(angle1_wrapped, angle2_wrapped) < M_PI / 2 &&
      std::max(angle1_wrapped, angle2_wrapped) > 3 * M_PI / 2) {
    return 2 * M_PI - (std::max(angle1_wrapped, angle2_wrapped) -
                       std::min(angle1_wrapped, angle2_wrapped));
  } else {
    return std::max(angle1_wrapped, angle2_wrapped) -
           std::min(angle1_wrapped, angle2_wrapped);
  }
}

Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& M, const int& precision) {
  Eigen::MatrixXd Mround(M.rows(), M.cols());
  for (int i = 0; i < M.rows(); i++) {
    for (int j = 0; j < M.cols(); j++) {
      Mround(i, j) = std::round(M(i, j) * std::pow(10, precision)) /
                     std::pow(10, precision);
    }
  }
  return Mround;
}

bool IsRotationMatrix(const Eigen::Matrix3d& R) {
  int precision = 3;
  Eigen::Matrix3d shouldBeIdentity = RoundMatrix(R * R.transpose(), precision);
  double detR = R.determinant();
  double detRRound = std::round(detR * precision) / precision;
  if (shouldBeIdentity.isIdentity() && detRRound == 1) {
    return 1;
  } else {
    return 0;
    if (!shouldBeIdentity.isIdentity()) {
      LOG_ERROR("Rotation matrix invalid. R x R^T != I");
    } else {
      LOG_ERROR(
          "Rotation matrix invalid. Determinant not equal to 1. det = %.5f",
          detR);
    }
  }
}

bool IsTransformationMatrix(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  bool homoFormValid, tValid;

  // check translation for infinity or nan
  if (std::isinf(T(0, 3)) || std::isinf(T(1, 3)) || std::isinf(T(2, 3)) ||
      std::isnan(T(0, 3)) || std::isnan(T(1, 3)) || std::isnan(T(2, 3))) {
    tValid = 0;
    LOG_ERROR("Translation invalid.t = [%.5f, %.5f, %.5f]", T(0, 3), T(1, 3),
              T(2, 3));
  } else {
    tValid = 1;
  }

  // check that bottom row is [0 0 0 1]
  if (T(3, 0) == 0 && T(3, 1) == 0 && T(3, 2) == 0 && T(3, 3) == 1) {
    homoFormValid = 1;
  } else {
    homoFormValid = 0;
    LOG_ERROR(
        "Transform not homographic form. Last row: [%.5f, %.5f, %.5f, %.5f]",
        T(3, 0), T(3, 1), T(3, 2), T(3, 3));
  }

  if (homoFormValid && tValid && IsRotationMatrix(R)) {
    return 1;
  } else {
    return 0;
  }
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
  perturbations_rad[0] = util::DegToRad(perturbations_rad[0]);
  perturbations_rad[1] = util::DegToRad(perturbations_rad[1]);
  perturbations_rad[2] = util::DegToRad(perturbations_rad[2]);
  return PerturbTransformRadM(T_in, perturbations_rad);
}

Eigen::Matrix4d BuildTransformEulerDegM(double rollInDeg, double pitchInDeg,
                                        double yawInDeg, double tx, double ty,
                                        double tz) {
  Eigen::Vector3d t(tx, ty, tz);
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(DegToRad(rollInDeg), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(DegToRad(pitchInDeg), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DegToRad(yawInDeg), Eigen::Vector3d::UnitZ());
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = R;
  T.block(0, 3, 3, 1) = t;
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

Eigen::Vector3d RToLieAlgebra(const Eigen::Matrix3d& R) {
  return InvSkewTransform(R.log());
}

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps) {
  return SkewTransform(eps).exp();
}

Eigen::Matrix4d InvertTransform(const Eigen::MatrixXd& T) {
  Eigen::Matrix4d T_inv;
  T_inv.setIdentity();
  T_inv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
  T_inv.block(0, 3, 3, 1) =
      -T.block(0, 0, 3, 3).transpose() * T.block(0, 3, 3, 1);
  return T_inv;
}

Eigen::Matrix4d
    QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose) {
  Eigen::Quaternion<double> quaternion{pose[0], pose[1], pose[2], pose[3]};
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
  Eigen::Quaternion<double> q = Eigen::Quaternion<double>(R);
  std::vector<double> pose{q.w(),   q.x(),   q.y(),  q.z(),
                           T(0, 3), T(1, 3), T(2, 3)};
  return pose;
}

cv::Mat DrawCoordinateFrame(
    const cv::Mat& img_in, const Eigen::MatrixXd& T_cam_frame,
    const std::shared_ptr<beam_calibration::CameraModel>& camera_model,
    const double& scale) {
  cv::Mat img_out;
  img_out = img_in.clone();
  Eigen::Vector4d origin(0, 0, 0, 1), x_end(scale, 0, 0, 1),
      y_end(0, scale, 0, 1), z_end(0, 0, scale, 1);
  Eigen::Vector4d o_trans = T_cam_frame * origin, x_trans = T_cam_frame * x_end,
                  y_trans = T_cam_frame * y_end, z_trans = T_cam_frame * z_end;

  Eigen::Vector3d o(o_trans(0), o_trans(1), o_trans(2)),
      x(x_trans(0), x_trans(1), x_trans(2)),
      y(y_trans(0), y_trans(1), y_trans(2)),
      z(z_trans(0), z_trans(1), z_trans(2));

  opt<Eigen::Vector2i> start_pixel = camera_model->ProjectPoint(o);
  opt<Eigen::Vector2i> end_pixel_x = camera_model->ProjectPoint(x);
  opt<Eigen::Vector2i> end_pixel_y = camera_model->ProjectPoint(y);
  opt<Eigen::Vector2i> end_pixel_z = camera_model->ProjectPoint(z);

  if (!start_pixel.has_value() || !end_pixel_x.has_value() ||
      !end_pixel_y.has_value() || !end_pixel_z.has_value()) {
    printf("Unable to draw coordinate frame. Frame exceeds image dimensions");
    return img_out;
  }

  cv::Point start, end_x, end_y, end_z;
  start.x = start_pixel.value()(0);
  start.y = start_pixel.value()(1);
  end_x.x = end_pixel_x.value()(0);
  end_x.y = end_pixel_x.value()(1);
  end_y.x = end_pixel_y.value()(0);
  end_y.y = end_pixel_y.value()(1);
  end_z.x = end_pixel_z.value()(0);
  end_z.y = end_pixel_z.value()(1);

  cv::Scalar colourX(0, 0, 255); // BGR
  cv::Scalar colourY(0, 255, 0);
  cv::Scalar colourZ(255, 0, 0);
  int thickness = 3;

  cv::line(img_out, start, end_x, colourX, thickness);
  cv::line(img_out, start, end_y, colourY, thickness);
  cv::line(img_out, start, end_z, colourZ, thickness);

  return img_out;
}

cv::Mat ProjectPointsToImage(
    const cv::Mat& img, boost::shared_ptr<PointCloud>& cloud,
    const Eigen::MatrixXd& T_IMAGE_CLOUD,
    std::shared_ptr<beam_calibration::CameraModel>& camera_model) {
  cv::Mat img_out;
  img_out = img.clone();
  Eigen::Vector4d point(0, 0, 0, 1);
  Eigen::Vector4d point_transformed(0, 0, 0, 1);
  for (int i = 0; i < cloud->size(); i++) {
    point = util::PCLPointToEigen(cloud->at(i)).homogeneous();
    point_transformed = T_IMAGE_CLOUD * point;
    opt<Eigen::Vector2i> pixel =
        camera_model->ProjectPoint(point_transformed.hnormalized());
    if (!pixel.has_value()) { continue; }
    cv::circle(img_out, cv::Point(pixel.value()[0], pixel.value()[1]), 2,
               cv::Scalar(0, 255, 0));
  }
  return img_out;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
    ProjectPoints(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud,
                  std::shared_ptr<beam_calibration::CameraModel>& camera_model,
                  const Eigen::Matrix4d& T) {
  Eigen::Vector3d point(0, 0, 0);
  Eigen::Vector4d point_transformed(0, 0, 0, 1);
  pcl::PointXYZ point_projected;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> projected_points =
      boost::make_shared<PointCloud>();
  for (int i = 0; i < cloud->size(); i++) {
    point = util::PCLPointToEigen(cloud->at(i));
    point_transformed = T * point.homogeneous();
    opt<Eigen::Vector2d> pixel =
        camera_model->ProjectPointPrecise(point_transformed.hnormalized());
    if (!pixel.has_value()) { continue; }
    point_projected.x = pixel.value()[0];
    point_projected.y = pixel.value()[1];
    point_projected.z = 0;
    projected_points->push_back(point_projected);
  }
  return projected_points;
}

PointCloudColor::Ptr ColorPointCloud(const PointCloud::Ptr& cloud, const int& r,
                                     const int& g, const int& b) {
  PointCloudColor::Ptr coloured_cloud;
  coloured_cloud = boost::make_shared<PointCloudColor>();
  uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                  static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
  pcl::PointXYZRGB point;
  for (PointCloud::iterator it = cloud->begin(); it != cloud->end(); ++it) {
    point.x = it->x;
    point.y = it->y;
    point.z = it->z;
    point.rgb = *reinterpret_cast<float*>(&rgb);
    coloured_cloud->push_back(point);
  }
  return coloured_cloud;
}

void OutputTransformInformation(const Eigen::Affine3d& T,
                                const std::string& transform_name) {
  OutputTransformInformation(T.matrix(), transform_name);
}

void OutputTransformInformation(const Eigen::Matrix4d& T,
                                const std::string& transform_name) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  std::cout << transform_name << ":\n"
            << T << "\n"
            << "rpy (deg): [" << util::RadToDeg(util::WrapToPi(rpy[0]))
            << ", " << util::RadToDeg(util::WrapToPi(rpy[1])) << ", "
            << util::RadToDeg(util::WrapToPi(rpy[2])) << "]\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector4d, AlignVec4d> &points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i ++) {
    pcl::PointXYZ to_add = EigenPointToPCL(points.at(i).head(3));
    return_cloud->push_back(to_add);
  }

  return return_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector4d> &points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i ++) {
    pcl::PointXYZ to_add = EigenPointToPCL(points.at(i).head(3));
    return_cloud->push_back(to_add);
  }

  return return_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector2d, AlignVec2d> &points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i ++) {
    pcl::PointXYZ to_add;
    to_add.x = points.at(i).x();
    to_add.y = points.at(i).y();
    to_add.z = 0;
    return_cloud->push_back(to_add);
  }

  return return_cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(const std::vector<Eigen::Vector2d> &points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i ++) {
    pcl::PointXYZ to_add;
    to_add.x = points.at(i).x();
    to_add.y = points.at(i).y();
    to_add.z = 0;
    return_cloud->push_back(to_add);
  }

  return return_cloud;

}

}} // namespace beam_optimization::util