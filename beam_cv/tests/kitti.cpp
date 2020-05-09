// beam
#include "beam_cv/DepthCompletion.h"
#include "beam_cv/DepthMap.h"
#include "beam_cv/LinearRegression.h"
#include "beam_cv/Utils.h"
#include "beam_utils/math.hpp"
// std
#include <chrono>
#include <cmath>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sys/types.h>
#include <vector>
// opencv
#include <opencv/cv.h>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
  string current_drive = string(argv[1]);
  string training_dir = current_drive + "/proj_depth/velodyne_raw/image_02/";
  string prediction_dir = current_drive + "/image_02/";
  char train_dir[training_dir.size() + 1];
  strcpy(train_dir, training_dir.c_str());
  struct dirent* entry;
  DIR* dir = opendir(train_dir);

  if (dir != NULL) {
    while ((entry = readdir(dir)) != NULL) {
      string depth_image_name(entry->d_name);
      string depth_image_path = training_dir + depth_image_name;
      if (depth_image_name.size() > 4) {
        Mat depth_img;
        Mat depth = imread(depth_image_path, IMREAD_GRAYSCALE);
        depth.convertTo(depth_img, CV_32F);
        beam_cv::MultiscaleInterpolation(depth_img);
        std::string output_location = prediction_dir + depth_image_name;
        imwrite(output_location, depth_img);
      }
    }
  }
  closedir(dir);
}
