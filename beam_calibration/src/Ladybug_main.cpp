#include "beam_calibration/Ladybug.h"

int main(int argc, char* argv[]) {
  beam_calibration::Ladybug ladybug{0};
  std::string path = "/home/steve/ladybug.conf";
  ladybug.LoadJSON(path);
  std::cout << ladybug << std::endl;

  beam::Vec3 point{1000, 1000, 10};
  beam::Vec2 test = ladybug.ProjectDistortedPoint(point);
  std::cout << test << std::endl;
  if (ladybug.PixelInImage(test)) {
    std::cout << "Pixel in image" << std::endl;
  } else
    std::cout << "Pixel not in image" << std::endl;
}
