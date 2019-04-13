#include "beam/colorize/Projection.h"
#include "beam/calibration/TfTree.h"
#include "beam/calibration/Pinhole.h"
#include "beam/utils/math.hpp"
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main() {

  // load intrinsics
  std::string intrinsics_name = "F1.json";
  std::string intrinsics_location = __FILE__;
  intrinsics_location.erase(intrinsics_location.end() - 13,
                            intrinsics_location.end());
  intrinsics_location += "tests/test_data/";
  intrinsics_location += intrinsics_name;

  beam_calibration::Pinhole F1;
  F1.LoadJSON(intrinsics_location);

  // load Image
  std::string image_name = "image18.jpg";
  std::string image_location = __FILE__;
  image_location.erase(image_location.end() - 13,
                            image_location.end());
  image_location += "tests/test_data/";
  image_location += image_name;

  cv::Mat image;
  image = cv::imread(image_location, CV_LOAD_IMAGE_COLOR);
  /*
  if(! image.data )
  {
      std::cout <<  "Could not open or find the image" << std::endl ;
      return -1;
  }

  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
  cv::imshow( "Display window", image );
  */

}
