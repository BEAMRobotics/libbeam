#include "beam_calibration/CameraModel.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <typeinfo>

using namespace cv;

int main() {
  // radtan
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 14, radtan_location.end());
  radtan_location += "tests/test_data/F1.json";
  std::shared_ptr<beam_calibration::CameraModel> radtan =
      beam_calibration::CameraModel::LoadJSON(radtan_location);

  // load equidistant
  std::string equidistant_location = __FILE__;
  equidistant_location.erase(equidistant_location.end() - 14,
                             equidistant_location.end());
  equidistant_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> equid =
      beam_calibration::CameraModel::LoadJSON(equidistant_location);

  // load conf file
  std::string ladybug_location = __FILE__;
  ladybug_location.erase(ladybug_location.end() - 14, ladybug_location.end());
  ladybug_location += "tests/test_data/ladybug.conf";
  beam_calibration::LadybugCamera ladybug(0, ladybug_location);

  Eigen::Matrix4d m;
  m << 1, 3.89313e-06, 6.3614e-06, -0.380727, -3.89309e-06, 1, -5.86562e-06,
      -4.39357e-06, -6.36143e-06, 5.8656e-06, 1, 0.634802, 0, 0, 0, 1;
  std::vector<Eigen::Vector4d> cylinder_points_;
  double radius_ = 0.0635;
  double height_ = 0.5;
  double threshold_ = 0.05;
  Eigen::Vector4d point(0, 0, 0, 1);
  double r = radius_ + threshold_;
  for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 12) {
    point(2) = r * std::cos(theta);

    point(1) = r * std::sin(theta);

    point(0) = -threshold_;
    cylinder_points_.push_back(point);

    point(0) = height_ + threshold_;

    cylinder_points_.push_back(point);
  }

  // load Image
  std::string image_location = "/home/jake/Pictures/vicon/vicon1.png";

  Mat image;
  image = imread(image_location, CV_LOAD_IMAGE_COLOR);
  Mat new_image = equid->UndistortImage(image);
  for (uint32_t i = 0; i < cylinder_points_.size(); i++) {
    Eigen::Vector4d p = m * cylinder_points_[i];
    beam::Vec3 new_p;
    new_p << p[0] / p[3], p[1] / p[3], p[2] / p[3];

    beam::Vec2 result = equid->ProjectUndistortedPoint(new_p);
    double x = result[0], y = result[1];
    if (x > 0 && x < 1536 && y > 0 && y < 2048) {
      new_image.at<Vec3b>(Point(x, y)).val[0] = 0;
      new_image.at<Vec3b>(Point(x, y)).val[1] = 0;
      new_image.at<Vec3b>(Point(x, y)).val[2] = 255;
      new_image.at<Vec3b>(Point(x + 1, y)).val[0] = 0;
      new_image.at<Vec3b>(Point(x + 1, y)).val[1] = 0;
      new_image.at<Vec3b>(Point(x + 1, y)).val[2] = 255;
      new_image.at<Vec3b>(Point(x, y + 1)).val[0] = 0;
      new_image.at<Vec3b>(Point(x, y + 1)).val[1] = 0;
      new_image.at<Vec3b>(Point(x, y + 1)).val[2] = 255;
      new_image.at<Vec3b>(Point(x - 1, y)).val[0] = 0;
      new_image.at<Vec3b>(Point(x - 1, y)).val[1] = 0;
      new_image.at<Vec3b>(Point(x - 1, y)).val[2] = 255;
      new_image.at<Vec3b>(Point(x, y - 1)).val[0] = 0;
      new_image.at<Vec3b>(Point(x, y - 1)).val[1] = 0;
      new_image.at<Vec3b>(Point(x, y - 1)).val[2] = 255;
      new_image.at<Vec3b>(Point(x + 1, y + 1)).val[0] = 0;
      new_image.at<Vec3b>(Point(x + 1, y + 1)).val[1] = 0;
      new_image.at<Vec3b>(Point(x + 1, y + 1)).val[2] = 255;
      new_image.at<Vec3b>(Point(x + 1, y - 1)).val[0] = 0;
      new_image.at<Vec3b>(Point(x + 1, y - 1)).val[1] = 0;
      new_image.at<Vec3b>(Point(x + 1, y - 1)).val[2] = 255;
      new_image.at<Vec3b>(Point(x - 1, y - 1)).val[0] = 0;
      new_image.at<Vec3b>(Point(x - 1, y - 1)).val[1] = 0;
      new_image.at<Vec3b>(Point(x - 1, y - 1)).val[2] = 255;
      new_image.at<Vec3b>(Point(x - 1, y + 1)).val[0] = 0;
      new_image.at<Vec3b>(Point(x - 1, y + 1)).val[1] = 0;
      new_image.at<Vec3b>(Point(x - 1, y + 1)).val[2] = 255;
    }
  }

  imwrite("/home/jake/Pictures/vicon/vicon1und.png", new_image);
}
