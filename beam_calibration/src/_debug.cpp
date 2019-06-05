#include "beam_calibration/CameraModel.h"
#include "beam_calibration/LadybugCamera.h"
#include "beam_calibration/TfTree.h"
#include "beam_utils/math.hpp"
#include <boost/filesystem.hpp>
#include <typeinfo>

int main() {
  // radtan
  std::string radtan_location = __FILE__;
  radtan_location.erase(radtan_location.end() - 14, radtan_location.end());
  radtan_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> radtan =
      beam_calibration::CameraModel::LoadJSON(radtan_location);
  // load equidistant
  std::string equidistant_location = __FILE__;
  equidistant_location.erase(equidistant_location.end() - 14,
                             equidistant_location.end());
  equidistant_location += "tests/test_data/F2.json";
  std::shared_ptr<beam_calibration::CameraModel> equid =
      beam_calibration::CameraModel::LoadJSON(equidistant_location);

  std::string image_location = "/home/jake/Pictures/img_06.png";

  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  cv::Mat undistorted_image = equid->UndistortImage(image);
  cv::imwrite("/home/jake/Pictures/cpp_undistorted.png", undistorted_image);

  beam::Vec3 p1(10, 10, 10);
  beam::Vec3 p2(20, 20, 20);
  std::cout << beam::distance(p1, p2) << std::endl;
  /*
  // load conf file
  std::string conf_location = __FILE__;
  conf_location.erase(conf_location.end() - 14, conf_location.end());
  conf_location += "tests/test_data/ladybug.conf";
  beam_calibration::LadybugCamera ladybug(0, conf_location);
  */
}
