#define CATCH_CONFIG_MAIN
#include <fstream>
#include <iostream>

#include <catch2/catch.hpp>

#include <beam_cv/Utils.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/RelativePoseEstimator.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_utils/angles.h>

#include <beam_utils/time.h>

#include <beam_calibration/Radtan.h>

std::shared_ptr<beam_cv::Matcher> matcher =
    std::make_shared<beam_cv::FLANNMatcher>(beam_cv::FLANN::KDTree, 0.8, true,
                                            true, cv::FM_RANSAC, 5);

std::shared_ptr<beam_calibration::CameraModel> LoadCam0() {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 23, cam_loc.end());
  std::string cam0_loc = cam_loc + "tests/test_data/cam0.json";
  return beam_calibration::CameraModel::Create(cam0_loc);
}

std::shared_ptr<beam_calibration::CameraModel> LoadCam1() {
  std::string cam_loc = __FILE__;
  cam_loc.erase(cam_loc.end() - 23, cam_loc.end());
  std::string cam1_loc = cam_loc + "tests/test_data/cam1.json";
  return beam_calibration::CameraModel::Create(cam1_loc);
}

cv::Mat LoadIm1() {
  std::string cur_loc = __FILE__;
  std::string im_loc = cur_loc;
  im_loc.erase(im_loc.end() - 23, im_loc.end());
  im_loc += "tests/test_data/im1.png";
  return cv::imread(im_loc, cv::IMREAD_COLOR);
}

cv::Mat LoadIm0() {
  std::string cur_loc = __FILE__;
  std::string im_loc = cur_loc;
  im_loc.erase(im_loc.end() - 23, im_loc.end());
  im_loc += "tests/test_data/im0.png";
  return cv::imread(im_loc, cv::IMREAD_COLOR);
}

TEST_CASE("Test feature matching: ORB") {
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::ORBDetector>(1000);

  std::shared_ptr<beam_calibration::CameraModel> cam0 = LoadCam0();
  std::shared_ptr<beam_calibration::CameraModel> cam1 = LoadCam1();

  cv::Mat imL = LoadIm1();
  cv::Mat imR = LoadIm0();

  std::vector<Eigen::Vector2i> pL_v;
  std::vector<Eigen::Vector2i> pR_v;
  beam_cv::DetectComputeAndMatch(imL, imR, descriptor, detector, matcher,
  pL_v,
                                 pR_v);

  beam::opt<Eigen::Matrix4d> T =
  beam_cv::RelativePoseEstimator::RANSACEstimator(
      cam1, cam0, pL_v, pR_v, beam_cv::EstimatorMethod::SEVENPOINT, 20, 5.0,
      12);

  Eigen::Quaterniond q(T.value().block<3, 3>(0, 0));
  Eigen::Quaterniond identity(Eigen::Matrix3d::Identity());
  double theta_rad = 2 * acos(abs(q.dot(identity)));
  double theta_deg = beam::Rad2Deg(theta_rad);
  if (T.has_value()) {
    std::stringstream ss;
    ss << T.value();
    BEAM_INFO("Angle error: {}", theta_deg);
    BEAM_INFO("Estimated pose:\n {}", ss.str());
  }

  REQUIRE(theta_deg < 10.0);
}

TEST_CASE("Test feature matching: SIFT") {
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::SIFTDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::SIFTDetector>(200);

  std::shared_ptr<beam_calibration::CameraModel> cam0 = LoadCam0();
  std::shared_ptr<beam_calibration::CameraModel> cam1 = LoadCam1();

  cv::Mat imL = LoadIm1();
  cv::Mat imR = LoadIm0();

  std::vector<Eigen::Vector2i> pL_v;
  std::vector<Eigen::Vector2i> pR_v;
  beam_cv::DetectComputeAndMatch(imL, imR, descriptor, detector, matcher,
  pL_v,
                                 pR_v);

  beam::opt<Eigen::Matrix4d> T =
  beam_cv::RelativePoseEstimator::RANSACEstimator(
      cam1, cam0, pL_v, pR_v, beam_cv::EstimatorMethod::SEVENPOINT, 20, 5.0,
      1);

  Eigen::Quaterniond q(T.value().block<3, 3>(0, 0));
  Eigen::Quaterniond identity(Eigen::Matrix3d::Identity());
  double theta_rad = 2 * acos(abs(q.dot(identity)));
  double theta_deg = beam::Rad2Deg(theta_rad);
  if (T.has_value()) {
    std::stringstream ss;
    ss << T.value();
    BEAM_INFO("Angle error: {}", theta_deg);
    BEAM_INFO("Estimated pose:\n {}", ss.str());
  }
  REQUIRE(theta_deg < 10.0);
}

TEST_CASE("Test feature matching: BRISK") {
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::BRISKDescriptor>();
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::SIFTDetector>(200);

  std::shared_ptr<beam_calibration::CameraModel> cam0 = LoadCam0();
  std::shared_ptr<beam_calibration::CameraModel> cam1 = LoadCam1();

  cv::Mat imL = LoadIm1();
  cv::Mat imR = LoadIm0();

  std::vector<Eigen::Vector2i> pL_v;
  std::vector<Eigen::Vector2i> pR_v;
  beam_cv::DetectComputeAndMatch(imL, imR, descriptor, detector, matcher,
  pL_v,
                                 pR_v);

  beam::opt<Eigen::Matrix4d> T =
  beam_cv::RelativePoseEstimator::RANSACEstimator(
      cam1, cam0, pL_v, pR_v, beam_cv::EstimatorMethod::SEVENPOINT, 20, 5.0,
      1);

  Eigen::Quaterniond q(T.value().block<3, 3>(0, 0));
  Eigen::Quaterniond identity(Eigen::Matrix3d::Identity());
  double theta_rad = 2 * acos(abs(q.dot(identity)));
  double theta_deg = beam::Rad2Deg(theta_rad);
  if (T.has_value()) {
    std::stringstream ss;
    ss << T.value();
    BEAM_INFO("Angle error: {}", theta_deg);
    BEAM_INFO("Estimated pose:\n {}", ss.str());
  }

  REQUIRE(theta_deg < 10.0);
}

// TEST_CASE("Test feature extraction and relative pose") {
//   std::shared_ptr<beam_cv::Descriptor> descriptor =
//       std::make_shared<beam_cv::ORBDescriptor>();
//   std::shared_ptr<beam_cv::Detector> detector =
//       std::make_shared<beam_cv::ORBDetector>(5000);

//   std::string cam1_loc = "/home/jake/sample.json";
//   std::shared_ptr<beam_calibration::CameraModel> cam0 =
//       beam_calibration::CameraModel::Create(cam1_loc);

//   cv::Mat imL = cv::imread("/home/jake/first.jpg", cv::IMREAD_COLOR);
//   cv::Mat imR = cv::imread("/home/jake/second.jpg", cv::IMREAD_COLOR);

//   std::vector<Eigen::Vector2i> pL_v;
//   std::vector<Eigen::Vector2i> pR_v;
//   beam_cv::DetectComputeAndMatch(imL, imR, descriptor, detector, matcher, pL_v,
//                                  pR_v);

//   beam::opt<Eigen::Matrix4d> T =
//       beam_cv::RelativePoseEstimator::RANSACEstimator(
//           cam0, cam0, pL_v, pR_v, beam_cv::EstimatorMethod::EIGHTPOINT, 20,
//           10.0);
//   std::cout << T.value() << std::endl;
//   for (auto& p : pL_v) {
//     imL.at<cv::Point3_<uchar>>(p[1], p[0]).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1], p[0]).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1], p[0]).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0]).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0]).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0]).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1], p[0] - 1).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1], p[0] - 1).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1], p[0] - 1).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0] - 1).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0] - 1).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0] - 1).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0] + 1).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0] + 1).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1] - 1, p[0] + 1).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1], p[0] + 1).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1], p[0] + 1).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1], p[0] + 1).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0] + 1).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0] + 1).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0] + 1).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0]).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0]).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0]).y = 0;

//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0] - 1).z = 255;
//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0] - 1).x = 0;
//     imL.at<cv::Point3_<uchar>>(p[1] + 1, p[0] - 1).y = 0;
//   }

//   for (auto& p : pR_v) {
//     imR.at<cv::Point3_<uchar>>(p[1], p[0]).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1], p[0]).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1], p[0]).y = 0;

//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0]).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0]).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0]).y = 0;

//     imR.at<cv::Point3_<uchar>>(p[1], p[0] - 1).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1], p[0] - 1).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1], p[0] - 1).y = 0;

//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0] - 1).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0] - 1).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0] - 1).y = 0;

//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0] + 1).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0] + 1).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1] - 1, p[0] + 1).y = 0;
//     imR.at<cv::Point3_<uchar>>(p[1], p[0] + 1).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1], p[0] + 1).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1], p[0] + 1).y = 0;

//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0] + 1).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0] + 1).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0] + 1).y = 0;

//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0]).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0]).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0]).y = 0;

//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0] - 1).z = 255;
//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0] - 1).x = 0;
//     imR.at<cv::Point3_<uchar>>(p[1] + 1, p[0] - 1).y = 0;
//   }

//   cv::imwrite("/home/jake/1.jpg", imL);
//   cv::imwrite("/home/jake/2.jpg", imR);
// }
