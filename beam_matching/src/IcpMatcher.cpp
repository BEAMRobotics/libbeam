#include <beam_matching/IcpMatcher.h>

#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <beam_utils/kdtree.h>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {

IcpMatcher::Params::Params(const std::string& param_config) {
  std::string read_file = param_config;
  if (param_config.empty()) {
    return;
  } else if (!boost::filesystem::exists(param_config)) {
    BEAM_WARN("Invalid matcher config path, file does not exist, using "
              "default. Input: {}",
              param_config);
    return;
  } else if (param_config == "DEFAULT_PATH") {
    std::string default_path = beam::LibbeamRoot();
    default_path += "beam_matching/config/icp.json";
    if (!boost::filesystem::exists(default_path)) {
      BEAM_WARN("Could not find default icp config at: {}. Using "
                "default params.",
                default_path);
      return;
    }
    read_file = default_path;
  }

  BEAM_INFO("Loading ICP matcher config file: {}", read_file);

  nlohmann::json J;
  std::ifstream file(read_file);
  file >> J;

  int covar_est_temp;
  this->max_corr = J["max_corr"];
  this->max_iter = J["max_iter"];
  this->t_eps = J["t_eps"];
  this->lidar_ang_covar = J["lidar_ang_covar"];
  this->lidar_lin_covar = J["lidar_lin_covar"];
  covar_est_temp = J["covar_estimator"];
  this->res = J["res"];
  this->multiscale_steps = J["multiscale_steps"];

  if ((covar_est_temp >= IcpMatcherParams::CovarMethod::LUM) &&
      (covar_est_temp <= IcpMatcherParams::CovarMethod::LUMold)) {
    this->covar_estimator =
        static_cast<IcpMatcherParams::CovarMethod>(covar_est_temp);
  } else {
    LOG_ERROR("Invalid covariance estimate method, using LUM");
    this->covar_estimator = IcpMatcherParams::CovarMethod::LUM;
  }
}

IcpMatcher::IcpMatcher(Params params) : params_(params) {
  SetIcpParams();
}

IcpMatcher::~IcpMatcher() {
  if (this->ref_) {
    this->ref_.reset();
    this->downsampled_ref_.reset();
  }
  if (this->target_) {
    this->target_.reset();
    this->downsampled_target_.reset();
  }
  if (this->final_) { this->final_.reset(); }
}

void IcpMatcher::SetParams(Params params) {
  this->params_ = params;
  SetIcpParams();
}

void IcpMatcher::SetIcpParams() {
  this->ref_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->target_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->final_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->downsampled_ref_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  this->downsampled_target_ =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (this->params_.res > 0) {
    this->filter_.setLeafSize(this->params_.res, this->params_.res,
                              this->params_.res);
  }
  this->resolution_ = this->params_.res;

  this->icp_.setMaxCorrespondenceDistance(this->params_.max_corr);
  this->icp_.setMaximumIterations(this->params_.max_iter);
  this->icp_.setTransformationEpsilon(this->params_.t_eps);
  this->icp_.setEuclideanFitnessEpsilon(this->params_.fit_eps);
}

void IcpMatcher::SetRef(const PointCloudPtr& ref) {
  this->ref_ = ref;
}

void IcpMatcher::SetTarget(const PointCloudPtr& target) {
  this->target_ = target;
}

bool IcpMatcher::Match() {
  if (this->params_.res > 0) {
    if (this->params_.multiscale_steps > 0) {
      Eigen::Affine3d running_transform = Eigen::Affine3d::Identity();
      for (int i = this->params_.multiscale_steps; i >= 0; i--) {
        float leaf_size = pow(2, i) * this->params_.res;
        this->filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
        this->filter_.setInputCloud(this->ref_);
        this->filter_.filter(*(this->downsampled_ref_));
        pcl::transformPointCloud(*(this->downsampled_ref_),
                                 *(this->downsampled_ref_), running_transform);
        this->icp_.setInputSource(this->downsampled_ref_);

        this->filter_.setInputCloud(this->target_);
        this->filter_.filter(*(this->downsampled_target_));
        this->icp_.setInputTarget(this->downsampled_target_);

        this->icp_.setMaxCorrespondenceDistance(pow(2, i) *
                                                this->params_.max_corr);
        this->icp_.align(*(this->final_));
        if (!icp_.hasConverged()) { return false; }
        running_transform.matrix() =
            icp_.getFinalTransformation().cast<double>() *
            running_transform.matrix();
      }
      this->result_ = running_transform;
      return true;
    } else {
      this->filter_.setLeafSize(this->params_.res, this->params_.res,
                                this->params_.res);
      this->filter_.setInputCloud(this->ref_);
      this->filter_.filter(*(this->downsampled_ref_));
      this->icp_.setInputSource(this->downsampled_ref_);

      this->filter_.setInputCloud(this->target_);
      this->filter_.filter(*(this->downsampled_target_));
      this->icp_.setInputTarget(this->downsampled_target_);

      this->icp_.align(*(this->final_));
      if (icp_.hasConverged()) {
        this->result_ = this->icp_.getFinalTransformation().cast<double>();
        return true;
      }
    }
  } else {
    this->icp_.setInputTarget(this->target_);
    this->icp_.setInputSource(this->ref_);
    this->icp_.align(*(this->final_));
    if (this->icp_.hasConverged()) {
      this->result_.matrix() = icp_.getFinalTransformation().cast<double>();
      return true;
    }
  }
  return false;
}

void IcpMatcher::CalculateCovariance() {
  switch (this->params_.covar_estimator) {
    case IcpMatcherParams::CovarMethod::LUM: this->EstimateLUM(); break;
    case IcpMatcherParams::CovarMethod::CENSI: this->EstimateCensi(); break;
    case IcpMatcherParams::CovarMethod::LUMold: this->EstimateLUMold(); break;
    default: return;
  }
}

/**
 * This is an implementation of the Haralick or Censi covariance approximation
 * for ICP. The core idea behind this is that the covariance of the cost f'n J
 * wrt optimization variable x is:
 *   cov(x) ~= (d2J/dx2)^-1*(d2J/dzdx)*cov(z)*(d2J/dzdx)'*(d2J/dx2)^-1
 *
 * Idea was taken from
 * https://censi.science/pub/research/2007-icra-icpcov.pdf
 *
 * This is an implementation for euler angles, what is below is a cleaned up
 * version of
 * http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7153246
 *
 * @INPROCEEDINGS{3d_icp_cov,
 * author={Prakhya, S.M. and Liu Bingbing and Yan Rui and Weisi Lin},
 *       booktitle={Machine Vision Applications (MVA), 2015 14th IAPR
 *       International Conference on},
 *       title={A closed-form estimate of 3D ICP covariance},
 *       year={2015},
 *       pages={526-529},
 *       doi={10.1109/MVA.2015.7153246},
 *       month={May},}
 */
void IcpMatcher::EstimateCensi() {
  auto& ref = this->ref_;
  auto& target = this->target_;
  if (this->params_.res > 0) {
    ref = this->downsampled_ref_;
    target = this->downsampled_target_;
  }
  if (this->icp_.hasConverged()) {
    const auto eulers = this->result_.rotation().eulerAngles(0, 1, 2);
    const auto translation = this->result_.translation();
    // set up aliases to shrink following lines
    const double &X1 = translation.x(), X2 = translation.y(),
                 X3 = translation.z();
    // precompute trig quantities
    double cr, sr, cp, sp, cy, sy;
    // r = roll, p = pitch, y = yaw, c = cos, s = sine
    cr = cos(eulers[0]);
    sr = sin(eulers[0]);
    cp = cos(eulers[1]);
    sp = sin(eulers[1]);
    cy = cos(eulers[2]);
    sy = sin(eulers[2]);

    Eigen::DiagonalMatrix<double, 6> sphere_cov;
    sphere_cov.diagonal() << this->params_.lidar_lin_covar,
        this->params_.lidar_ang_covar, this->params_.lidar_ang_covar,
        this->params_.lidar_lin_covar, this->params_.lidar_ang_covar,
        this->params_.lidar_ang_covar;
    Eigen::MatrixXd cov_Z(Eigen::MatrixXd::Zero(6, 6));
    Eigen::MatrixXd j(Eigen::MatrixXd::Zero(6, 6));
    double az, br, rg; // azimuth, bearing and range

    // The ordering for partials is x, y, z, rotx, roty, rotz
    // This is a symmetric matrix, so only need to fill out the upper
    // triangular portion
    Eigen::MatrixXd d2J_dX2(Eigen::MatrixXd::Zero(6, 6));

    // To hold running total of d2J_dZdX*cov(z)*d2J_dZdX'
    Eigen::MatrixXd middle(Eigen::MatrixXd::Zero(6, 6));

    // Gets overwritten each loop
    Eigen::MatrixXd d2J_dZdX(Eigen::MatrixXd::Zero(6, 6));
    d2J_dZdX(3, 0) = -2;
    d2J_dZdX(4, 1) = -2;
    d2J_dZdX(5, 2) = -2;

    auto list = this->icp_.correspondences_.get();
    for (auto it = list->begin(); it != list->end(); ++it) {
      if (it->index_match > -1) {
        // it is -1 if there is no match in the target cloud
        // set up some aliases to make following lines more compact
        const float &Z1 = (target->points[it->index_match].x),
                    Z2 = (target->points[it->index_match].y),
                    Z3 = (target->points[it->index_match].z),
                    Z4 = (ref->points[it->index_query].x),
                    Z5 = (ref->points[it->index_query].y),
                    Z6 = (ref->points[it->index_query].z);

        rg = std::sqrt(Z1 * Z1 + Z2 * Z2 + Z3 * Z3);
        br = std::atan2(Z2, Z1);
        az = std::atan(Z3 / std::sqrt(Z1 * Z1 + Z2 * Z2));
        j(0, 0) = cos(br) * sin(az);
        j(1, 0) = sin(br) * sin(az);
        j(2, 0) = cos(az);
        j(0, 1) = -rg * sin(br) * sin(az);
        j(1, 1) = rg * cos(br) * sin(az);
        j(0, 2) = rg * cos(br) * cos(az);
        j(1, 2) = rg * cos(az) * sin(br);
        j(2, 2) = -rg * sin(az);
        rg = std::sqrt(Z4 * Z4 + Z5 * Z5 + Z6 * Z6);
        br = std::atan2(Z5, Z4);
        az = std::atan(Z6 / std::sqrt(Z4 * Z4 + Z5 * Z5));
        j(3, 3) = cos(br) * sin(az);
        j(4, 3) = sin(br) * sin(az);
        j(5, 3) = cos(az);
        j(3, 4) = -rg * sin(br) * sin(az);
        j(4, 4) = rg * cos(br) * sin(az);
        j(3, 5) = rg * cos(br) * cos(az);
        j(4, 5) = rg * cos(az) * sin(br);
        j(5, 5) = -rg * sin(az);
        cov_Z = j * sphere_cov.derived() * j.transpose();

        // clang-format off

                // coordinate transform jacobian. Order is range, bearing,
                // azimuth for first and 2nd point
                // [ cos(S2)*sin(S3), -S1*sin(S2)*sin(S3),   S1*cos(S2)*cos(S3),               0,                0,                  0]
                // [ sin(S2)*sin(S3),  S1*cos(S2)*sin(S3),   S1*cos(S3)*sin(S2),               0,                0,                  0]
                // [         cos(S3),                   0,          -S1*sin(S3),               0,                0,                  0]
                // [               0,                   0,                    0, cos(S5)*sin(S6), -S4*sin(S5)*sin(S6), S4*cos(S5)*cos(S6)]
                // [               0,                   0,                   0, sin(S5)*sin(S6),  S4*cos(S5)*sin(S6),  S4*cos(S6)*sin(S5)]
                // [               0,                   0,                  0,         cos(S6),                   0,       -S4*sin(S6)]

                // d2J_dx2

                d2J_dX2(0, 0) += 2;
                d2J_dX2(1, 1) += 2;
                d2J_dX2(2, 2) += 2;

                d2J_dX2(0, 3) += 2 * Z2 * (sr * sy + cr * cy * sp) + 2 * Z3 * (cr * sy - cy * sr * sp);
                d2J_dX2(1, 3) += -2 * Z2 * (cy * sr - cr * sp * sy) - 2 * Z3 * (cr * cy + sr * sp * sy);
                d2J_dX2(2, 3) += 2 * cp * (Z2 * cr - Z3 * sr);
                d2J_dX2(3, 3) += (2 * Z2 * (cr * sy - cy * sr * sp) - 2 * Z3 * (sr * sy + cr * cy * sp)) * (X1 - Z4 - Z2 * (cr * sy - cy * sr * sp) +
                     Z3 * (sr * sy + cr * cy * sp) + Z1 * cp * cy) - (2 * Z2 * (cr * cy + sr * sp * sy) - 2 * Z3 * (cy * sr - cr * sp * sy)) *
                    (X2 - Z5 + Z2 * (cr * cy + sr * sp * sy) - Z3 * (cy * sr - cr * sp * sy) + Z1 * cp * sy) - (2 * Z3 * cr * cp + 2 * Z2 * cp * sr) *
                    (X3 - Z6 - Z1 * sp + Z3 * cr * cp + Z2 * cp * sr) + (Z2 * (sr * sy + cr * cy * sp) + Z3 * (cr * sy - cy * sr * sp)) *
                    (2 * Z2 * (sr * sy + cr * cy * sp) + 2 * Z3 * (cr * sy - cy * sr * sp)) + (Z2 * (cy * sr - cr * sp * sy) +
                    Z3 * (cr * cy + sr * sp * sy)) * (2 * Z2 * (cy * sr - cr * sp * sy) + 2 * Z3 * (cr * cy + sr * sp * sy)) +
                    (Z2 * cr * cp - Z3 * cp * sr) * (2 * Z2 * cr * cp - 2 * Z3 * cp * sr);

                d2J_dX2(0, 4) += 2 * cy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dX2(1, 4) += 2 * sy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dX2(2, 4) += -2 * Z1 * cp - 2 * Z3 * cr * sp - 2 * Z2 * sr * sp;
                d2J_dX2(3, 4) += -2 * (Z2 * cr - Z3 * sr) * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy - X2 * cp * sy + Z5 * cp * sy);
                d2J_dX2(4, 4) += (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) * (2 * Z1 * cp + 2 * Z3 * cr * sp + 2 * Z2 * sr * sp) -
                  (2 * Z3 * cr * cp - 2 * Z1 * sp + 2 * Z2 * cp * sr) *(X3 - Z6 - Z1 * sp + Z3 * cr * cp + Z2 * cp * sr) +
                  2 * cy * cy * pow((Z3 * cr * cp - Z1 * sp + Z2 * cp * sr), 2) +2 * sy * sy *
                    pow((Z3 * cr * cp - Z1 * sp + Z2 * cp * sr), 2) - 2 * cy * (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) *
                    (X1 - Z4 + Z1 * cp * cy - Z2 * cr * sy + Z3 * sr * sy + Z2 * cy * sr * sp + Z3 * cr * cy * sp) -
                  2 * sy * (Z1 * cp + Z3 * cr * sp + Z2 * sr * sp) * (X2 - Z5 + Z2 * cr * cy + Z1 * cp * sy - Z3 * cy * sr +
                     Z3 * cr * sp * sy + Z2 * sr * sp * sy);

                d2J_dX2(0, 5) += 2 * Z3 * (cy * sr - cr * sp * sy) - 2 * Z2 * (cr * cy + sr * sp * sy) - 2 * Z1 * cp * sy;
                d2J_dX2(1, 5) += 2 * Z3 * (sr * sy + cr * cy * sp) - 2 * Z2 * (cr * sy - cy * sr * sp) + 2 * Z1 * cp * cy;
                // d2J_dX2(2,5) += 0;  This quantity is zero
                d2J_dX2(3, 5) +=
                  2 * X1 * Z3 * cr * cy - 2 * Z3 * Z4 * cr * cy +
                  2 * X1 * Z2 * cy * sr + 2 * X2 * Z3 * cr * sy -
                  2 * Z2 * Z4 * cy * sr - 2 * Z3 * Z5 * cr * sy +
                  2 * X2 * Z2 * sr * sy - 2 * Z2 * Z5 * sr * sy +
                  2 * X2 * Z2 * cr * cy * sp - 2 * Z2 * Z5 * cr * cy * sp -
                  2 * X1 * Z2 * cr * sp * sy - 2 * X2 * Z3 * cy * sr * sp +
                  2 * Z2 * Z4 * cr * sp * sy + 2 * Z3 * Z5 * cy * sr * sp +
                  2 * X1 * Z3 * sr * sp * sy - 2 * Z3 * Z4 * sr * sp * sy;
                d2J_dX2(4, 5) += 2 * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr) *
                                     (X2 * cy - Z5 * cy - X1 * sy + Z4 * sy);
                d2J_dX2(5, 5) +=
                  2 * Z1 * Z4 * cp * cy - 2 * X2 * Z2 * cr * cy -
                  2 * X1 * Z1 * cp * cy + 2 * Z2 * Z5 * cr * cy +
                  2 * X1 * Z2 * cr * sy - 2 * X2 * Z1 * cp * sy +
                  2 * X2 * Z3 * cy * sr - 2 * Z2 * Z4 * cr * sy +
                  2 * Z1 * Z5 * cp * sy - 2 * Z3 * Z5 * cy * sr -
                  2 * X1 * Z3 * sr * sy + 2 * Z3 * Z4 * sr * sy -
                  2 * X1 * Z3 * cr * cy * sp + 2 * Z3 * Z4 * cr * cy * sp -
                  2 * X1 * Z2 * cy * sr * sp - 2 * X2 * Z3 * cr * sp * sy +
                  2 * Z2 * Z4 * cy * sr * sp + 2 * Z3 * Z5 * cr * sp * sy -
                  2 * X2 * Z2 * sr * sp * sy + 2 * Z2 * Z5 * sr * sp * sy;

                // d2J_dZdX
                // Instead of Filling out this quantity directly,
                // d2J_dZdX*cov(z)*d2J_dZdX' will be calculated for the
                // current correspondence. This is then added elementwise to a
                // running total matrix. This is because
                // the number of columns d2J_dZdX grows linearly with the number
                // of correspondences. This approach can be
                // done because each measurement is assumed independent
                d2J_dZdX(0, 0) = 2 * cp * cy;
                d2J_dZdX(1, 0) = 2 * cy * sr * sp - 2 * cr * sy;
                d2J_dZdX(2, 0) = 2 * sr * sy + 2 * cr * cy * sp;
                // d2J_dZdX(3,0) = -2;
                // d2J_dZdX(4,0) = 0;
                // d2J_dZdX(5,0) = 0;

                d2J_dZdX(0, 1) = 2 * cp * sy;
                d2J_dZdX(1, 1) = 2 * cr * cy + 2 * sr * sp * sy;
                d2J_dZdX(2, 1) = 2 * cr * sp * sy - 2 * cy * sr;
                // d2J_dZdX(3,1) = 0;
                // d2J_dZdX(4,1) = -2;
                // d2J_dZdX(5,1) = 0;

                d2J_dZdX(0, 2) = -2 * sp;
                d2J_dZdX(1, 2) = 2 * cp * sr;
                d2J_dZdX(2, 2) = 2 * cr * cp;
                // d2J_dZdX(3,2) = 0;
                // d2J_dZdX(4,2) = 0;
                // d2J_dZdX(5,2) = -2;

                // d2J_dZdX(0,3) = 0;
                d2J_dZdX(1, 3) = 2 * X3 * cr * cp - 2 * Z6 * cr * cp -
                                 2 * X2 * cy * sr + 2 * Z5 * cy * sr +
                                 2 * X1 * sr * sy - 2 * Z4 * sr * sy +
                                 2 * X2 * cr * sp * sy - 2 * Z5 * cr * sp * sy +
                                 2 * X1 * cr * cy * sp - 2 * Z4 * cr * cy * sp;
                d2J_dZdX(2, 3) = 2 * Z5 * cr * cy - 2 * X2 * cr * cy +
                                 2 * X1 * cr * sy - 2 * X3 * cp * sr -
                                 2 * Z4 * cr * sy + 2 * Z6 * cp * sr -
                                 2 * X1 * cy * sr * sp + 2 * Z4 * cy * sr * sp -
                                 2 * X2 * sr * sp * sy + 2 * Z5 * sr * sp * sy;
                d2J_dZdX(3, 3) = -2 * Z2 * (sr * sy + cr * cy * sp) -
                                 2 * Z3 * (cr * sy - cy * sr * sp);
                d2J_dZdX(4, 3) = 2 * Z2 * (cy * sr - cr * sp * sy) +
                                 2 * Z3 * (cr * cy + sr * sp * sy);
                d2J_dZdX(5, 3) = -2 * cp * (Z2 * cr - Z3 * sr);

                d2J_dZdX(0, 4) = 2 * Z6 * cp - 2 * X3 * cp - 2 * X1 * cy * sp +
                                 2 * Z4 * cy * sp - 2 * X2 * sp * sy +
                                 2 * Z5 * sp * sy;
                d2J_dZdX(1, 4) = -2 * sr * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy -
                             X2 * cp * sy + Z5 * cp * sy);
                d2J_dZdX(2, 4) = -2 * cr * (X3 * sp - Z6 * sp - X1 * cp * cy + Z4 * cp * cy -
                             X2 * cp * sy + Z5 * cp * sy);
                d2J_dZdX(3, 4) = -2 * cy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dZdX(4, 4) = -2 * sy * (Z3 * cr * cp - Z1 * sp + Z2 * cp * sr);
                d2J_dZdX(5, 4) = 2 * Z1 * cp + 2 * Z3 * cr * sp + 2 * Z2 * sr * sp;

                d2J_dZdX(0, 5) = 2 * cp * (X2 * cy - Z5 * cy - X1 * sy + Z4 * sy);
                d2J_dZdX(1, 5) = 2 * Z4 * cr * cy - 2 * X1 * cr * cy -
                                 2 * X2 * cr * sy + 2 * Z5 * cr * sy +
                                 2 * X2 * cy * sr * sp - 2 * Z5 * cy * sr * sp -
                                 2 * X1 * sr * sp * sy + 2 * Z4 * sr * sp * sy;
                d2J_dZdX(2, 5) = 2 * X1 * cy * sr - 2 * Z4 * cy * sr +
                                 2 * X2 * sr * sy - 2 * Z5 * sr * sy -
                                 2 * X1 * cr * sp * sy + 2 * Z4 * cr * sp * sy +
                                 2 * X2 * cr * cy * sp - 2 * Z5 * cr * cy * sp;
                d2J_dZdX(3, 5) = 2 * Z2 * (cr * cy + sr * sp * sy) -
                                 2 * Z3 * (cy * sr - cr * sp * sy) +
                                 2 * Z1 * cp * sy;
                d2J_dZdX(4, 5) = 2 * Z2 * (cr * sy - cy * sr * sp) -
                                 2 * Z3 * (sr * sy + cr * cy * sp) -
                                 2 * Z1 * cp * cy;
                // d2J_dZdX(5,5) = 0;

                middle.noalias() += d2J_dZdX * cov_Z * (d2J_dZdX.transpose());

        // clang-format on
      }
    }

    // The covariance is approximately: (Prakhya eqn. 3)
    // d2J_dX2^-1 * d2J_dZdX*cov(z)*d2J_dZdX' *  d2J_dX2^-1
    // = d2J_dX2^-1 * middle * d2J_dX2^-1
    this->covariance_ = (d2J_dX2.inverse() * middle * d2J_dX2.inverse());
  }
}

/**
 * 3D formulation of the approach by Lu & Milios
 * http://www-robotics.usc.edu/~gaurav/CS547/milios_map.pdf
 * Implementation from PCL
 */
void IcpMatcher::EstimateLUMold() {
  auto& source_trans = this->final_;
  PointCloudPtr targetc;
  if (this->params_.res > 0) {
    targetc = this->downsampled_target_;
  } else {
    targetc = this->target_;
  }
  uint64_t numSourcePts = source_trans->size();
  std::vector<Eigen::Vector3f> corrs_aver{numSourcePts};
  std::vector<Eigen::Vector3f> corrs_diff{numSourcePts};
  int numCorr = 0;

  Eigen::Matrix<double, 6, 6> edgeCov = Eigen::Matrix<double, 6, 6>::Identity();

  // build kd tree for source points
  beam::KdTree<pcl::PointXYZ> kdtree(*targetc);

  // iterate through the source cloud and compute match covariance

  for (uint64_t i = 0; i < numSourcePts; i++) {
    pcl::PointXYZ qpt = source_trans->points[i];
    std::vector<uint32_t> nn_idx;
    std::vector<float> nn_sqr_dist;
    // returns the index of the nn point in the targetc
    kdtree.nearestKSearch(qpt, 1, nn_idx, nn_sqr_dist);

    if (nn_sqr_dist[0] <
        this->params_.max_corr * this->params_.max_corr) // if the distance to
    // point is less than max correspondence distance, use it to calculate
    {
      Eigen::Vector3f source_pt = qpt.getVector3fMap();
      Eigen::Vector3f target_pt = targetc->points[nn_idx[0]].getVector3fMap();

      // Compute the point pair average and difference and store for later
      // use
      corrs_aver[numCorr] = 0.5 * (source_pt + target_pt);
      corrs_diff[numCorr] = source_pt - target_pt;
      numCorr++;
    } else {
      continue;
    }
  }
  corrs_aver.resize(numCorr);
  corrs_diff.resize(numCorr);

  // now compute the M matrix
  Eigen::Matrix<double, 6, 6> MM = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> MZ = Eigen::Matrix<double, 6, 1>::Zero();
  for (int ci = 0; ci != numCorr; ++ci) // ci = correspondence iterator
  {
    // Fast computation of summation elements of M'M
    MM(0, 4) -= corrs_aver[ci](1);
    MM(0, 5) += corrs_aver[ci](2);
    MM(1, 3) -= corrs_aver[ci](2);
    MM(1, 4) += corrs_aver[ci](0);
    MM(2, 3) += corrs_aver[ci](1);
    MM(2, 5) -= corrs_aver[ci](0);
    MM(3, 4) -= corrs_aver[ci](0) * corrs_aver[ci](2);
    MM(3, 5) -= corrs_aver[ci](0) * corrs_aver[ci](1);
    MM(4, 5) -= corrs_aver[ci](1) * corrs_aver[ci](2);
    MM(3, 3) += corrs_aver[ci](1) * corrs_aver[ci](1) +
                corrs_aver[ci](2) * corrs_aver[ci](2);
    MM(4, 4) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                corrs_aver[ci](1) * corrs_aver[ci](1);
    MM(5, 5) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                corrs_aver[ci](2) * corrs_aver[ci](2);

    // Fast computation of M'Z
    MZ(0) += corrs_diff[ci](0);
    MZ(1) += corrs_diff[ci](1);
    MZ(2) += corrs_diff[ci](2);
    MZ(3) += corrs_aver[ci](1) * corrs_diff[ci](2) -
             corrs_aver[ci](2) * corrs_diff[ci](1);
    MZ(4) += corrs_aver[ci](0) * corrs_diff[ci](1) -
             corrs_aver[ci](1) * corrs_diff[ci](0);
    MZ(5) += corrs_aver[ci](2) * corrs_diff[ci](0) -
             corrs_aver[ci](0) * corrs_diff[ci](2);
  }
  // Remaining elements of M'M
  MM(0, 0) = MM(1, 1) = MM(2, 2) = static_cast<float>(numCorr);
  MM(4, 0) = MM(0, 4);
  MM(5, 0) = MM(0, 5);
  MM(3, 1) = MM(1, 3);
  MM(4, 1) = MM(1, 4);
  MM(3, 2) = MM(2, 3);
  MM(5, 2) = MM(2, 5);
  MM(4, 3) = MM(3, 4);
  MM(5, 3) = MM(3, 5);
  MM(5, 4) = MM(4, 5);

  // Compute pose difference estimation
  Eigen::Matrix<double, 6, 1> D =
      static_cast<Eigen::Matrix<double, 6, 1>>(MM.inverse() * MZ);

  // Compute s^2
  float ss = 0.0f;
  for (int ci = 0; ci != numCorr; ++ci) // ci = correspondence iterator
  {
    ss += static_cast<float>(
        pow(corrs_diff[ci](0) -
                (D(0) + corrs_aver[ci](2) * D(5) - corrs_aver[ci](1) * D(4)),
            2.0f) +
        pow(corrs_diff[ci](1) -
                (D(1) + corrs_aver[ci](0) * D(4) - corrs_aver[ci](2) * D(3)),
            2.0f) +
        pow(corrs_diff[ci](2) -
                (D(2) + corrs_aver[ci](1) * D(3) - corrs_aver[ci](0) * D(5)),
            2.0f));
  }

  // When reaching the limitations of computation due to linearization
  if (ss < 0.0000000000001 || !std::isfinite(ss)) {
    LOG_ERROR("Covariance matrix calculation was unsuccessful");
  }

  // Store the results in the slam graph
  edgeCov = MM * (1.0f / ss);

  this->covariance_ = edgeCov.inverse();
}

/** Taken from the Lu and Milios matcher in PCL */
void IcpMatcher::EstimateLUM() {
  auto& ref = this->final_;
  PointCloudPtr targetc;
  if (this->params_.res > 0) {
    targetc = this->downsampled_target_;
  } else {
    targetc = this->target_;
  }
  if (this->icp_.hasConverged()) {
    auto list = this->icp_.correspondences_.get();
    Eigen::Matrix<double, 6, 6> MM = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> MZ = Eigen::Matrix<double, 6, 1>::Zero();
    std::vector<Eigen::Vector3f> corrs_aver;
    std::vector<Eigen::Vector3f> corrs_diff;

    int numCorr = 0;
    for (auto it = list->begin(); it != list->end(); ++it) {
      if (it->index_match > -1) {
        corrs_aver.push_back(
            Eigen::Vector3f(0.5f * (ref->points[it->index_query].x +
                                    targetc->points[it->index_match].x),
                            0.5f * (ref->points[it->index_query].y +
                                    targetc->points[it->index_match].y),
                            0.5f * (ref->points[it->index_query].z +
                                    targetc->points[it->index_match].z)));
        corrs_diff.push_back(Eigen::Vector3f(
            ref->points[it->index_query].x - targetc->points[it->index_match].x,
            ref->points[it->index_query].y - targetc->points[it->index_match].y,
            ref->points[it->index_query].z -
                targetc->points[it->index_match].z));
        numCorr++;
      }
    }

    for (int ci = 0; ci != numCorr; ++ci) // ci = correspondence iterator
    {
      // Fast computation of summation elements of M'M
      MM(0, 4) -= corrs_aver[ci](1);
      MM(0, 5) += corrs_aver[ci](2);
      MM(1, 3) -= corrs_aver[ci](2);
      MM(1, 4) += corrs_aver[ci](0);
      MM(2, 3) += corrs_aver[ci](1);
      MM(2, 5) -= corrs_aver[ci](0);
      MM(3, 4) -= corrs_aver[ci](0) * corrs_aver[ci](2);
      MM(3, 5) -= corrs_aver[ci](0) * corrs_aver[ci](1);
      MM(4, 5) -= corrs_aver[ci](1) * corrs_aver[ci](2);
      MM(3, 3) += corrs_aver[ci](1) * corrs_aver[ci](1) +
                  corrs_aver[ci](2) * corrs_aver[ci](2);
      MM(4, 4) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                  corrs_aver[ci](1) * corrs_aver[ci](1);
      MM(5, 5) += corrs_aver[ci](0) * corrs_aver[ci](0) +
                  corrs_aver[ci](2) * corrs_aver[ci](2);

      // Fast computation of M'Z
      MZ(0) += corrs_diff[ci](0);
      MZ(1) += corrs_diff[ci](1);
      MZ(2) += corrs_diff[ci](2);
      MZ(3) += corrs_aver[ci](1) * corrs_diff[ci](2) -
               corrs_aver[ci](2) * corrs_diff[ci](1);
      MZ(4) += corrs_aver[ci](0) * corrs_diff[ci](1) -
               corrs_aver[ci](1) * corrs_diff[ci](0);
      MZ(5) += corrs_aver[ci](2) * corrs_diff[ci](0) -
               corrs_aver[ci](0) * corrs_diff[ci](2);
    }
    // Remaining elements of M'M
    MM(0, 0) = MM(1, 1) = MM(2, 2) = static_cast<float>(numCorr);
    MM(4, 0) = MM(0, 4);
    MM(5, 0) = MM(0, 5);
    MM(3, 1) = MM(1, 3);
    MM(4, 1) = MM(1, 4);
    MM(3, 2) = MM(2, 3);
    MM(5, 2) = MM(2, 5);
    MM(4, 3) = MM(3, 4);
    MM(5, 3) = MM(3, 5);
    MM(5, 4) = MM(4, 5);

    // Compute pose difference estimation
    Eigen::Matrix<double, 6, 1> D =
        static_cast<Eigen::Matrix<double, 6, 1>>(MM.inverse() * MZ);

    // Compute s^2
    float ss = 0.0f;
    for (int ci = 0; ci != numCorr; ++ci) // ci = correspondence iterator
    {
      ss += static_cast<float>(
          pow(corrs_diff[ci](0) -
                  (D(0) + corrs_aver[ci](2) * D(5) - corrs_aver[ci](1) * D(4)),
              2.0f) +
          pow(corrs_diff[ci](1) -
                  (D(1) + corrs_aver[ci](0) * D(4) - corrs_aver[ci](2) * D(3)),
              2.0f) +
          pow(corrs_diff[ci](2) -
                  (D(2) + corrs_aver[ci](1) * D(3) - corrs_aver[ci](0) * D(5)),
              2.0f));
    }

    // When reaching the limitations of computation due to linearization
    if (ss < 0.0000000000001 || !std::isfinite(ss)) {
      LOG_ERROR("Covariance matrix calculation was unsuccessful");
      return;
    }

    auto information = MM * (1.0f / ss);
    this->covariance_ = information.inverse();
  }
}

void IcpMatcher::SaveResults(const std::string& output_dir,
                             const std::string& prefix) {
  SaveResultsPCLXYZ(output_dir, prefix, ref_, target_);
}

} // namespace beam_matching
