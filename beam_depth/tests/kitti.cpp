#include <opencv2/core.hpp>

#include "beam_depth/DepthCompletion.h"
#include "beam_depth/DepthMap.h"
#include "beam_utils/math.h"

#include <dirent.h>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
  if (argc < 4 || argc > 4) {
    std::cout
        << "Usage: ./beam_depth_kitti [current drive] [kitti folder location] "
           "[image_02 or image_01]"
        << std::endl;
  } else {
    string current_drive = string(argv[1]);
    string kitti_location = string(argv[2]);
    string camera = string(argv[3]);

    string training_dir = kitti_location + "/train/" + current_drive +
                          "/proj_depth/velodyne_raw/" + camera;
    string prediction_dir =
        kitti_location + "/prediction/" + current_drive + camera;
    std::cout << "Output will be written to: " + prediction_dir << std::endl;

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
          beam_depth::MultiscaleInterpolation(depth_img);
          std::string output_location = prediction_dir + depth_image_name;
          imwrite(output_location, depth_img);
        }
      }
    }
    closedir(dir);
  }
}
