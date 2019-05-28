// format libraries
#include <nlohmann/json.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
/*
using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.01905f;
const float arucoMarkerDimension = 0.1016f;
const Size chessboardDimensions = Size(5, 6);

void createKnownBoardPositions(Size board_size, float square_edge_length,
                               vector<Point3f>& corners) {
  for (int i = 0; i < board_size.height; i++) {
    for (int j = 0; j < board_size.width; j++) {
      corners.push_back(
          Point3f(j * square_edge_length, i * square_edge_length, 0.0f));
    }
  }
}

void getChessboardCorners(vector<Mat> images,
                          vector<vector<Point2f>>& all_found_corners) {
  for (vector<Mat>::iterator iter = images.begin(); iter != images.end();
       iter++) {
    vector<Point2f> pointBuf;
    bool found = findChessBoardCorners(*iter, chessboardDimensions, pointBuf,
                                       CV_CALIB_CB_ADAPTIVE_THRESH |
                                           CV_CALIB_CB_NORMALIZE_IMAGE);
    if (found) { all_found_corners.push_back(pointBuf); }
    drawChessBoardCorners(*iter, chessboardDimensions, pointBuf, found);
    imshow("Looking for corners", *iter);
    waitKey(0);
  }
}
*/
int main(int argv, char** argc) {}