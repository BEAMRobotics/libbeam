// format libraries
#include <nlohmann/json.hpp>

// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.01905f;
const float aurcoSquareDimension = 0.1016f;
const Size chessboardDimensions = Size(6, 6);

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
    bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf,
                                       CV_CALIB_CB_ADAPTIVE_THRESH |
                                           CV_CALIB_CB_NORMALIZE_IMAGE);
    if (found) { all_found_corners.push_back(pointBuf); }
    /*
    drawChessBoardCorners(*iter, chessboardDimensions, pointBuf, found);
    imshow("Looking for corners", *iter);
    waitKey(0);*/
  }
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize,
                       float squareEdgeLength, Mat& cameraMatrix,
                       Mat& distanceCoefficients) {
  vector<vector<Point2f>> checkerboardImageSpacePoints;
  getChessboardCorners(calibrationImages, checkerboardImageSpacePoints);

  vector<vector<Point3f>> worldSpaceCornerPoints(1);
  createKnownBoardPositions(boardSize, squareEdgeLength,
                            worldSpaceCornerPoints[0]);
  worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(),
                                worldSpaceCornerPoints[0]);

  vector<Mat> rVectors, tVectors;
  distanceCoefficients = Mat::zeros(8, 1, CV_64F);
  calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints,
                  boardSize, cameraMatrix, distanceCoefficients, rVectors,
                  tVectors);
}

int main() {
  string path = "home/jake/fisheye_calib_pics/";
  vector<Mat> calibration_images;

  /*
  string image_name = "img_30.jpg";
  string image_location = __FILE__;
  image_location.erase(image_location.end() - 14, image_location.end());
  image_location += image_name;
  cout << image_location << endl;
  Mat image;
  image = imread(image_location, CV_LOAD_IMAGE_COLOR);
  Size patternsize(6, 6);  // number of centers
  vector<Point2f> centers; // this will be filled by the detected centers

  bool patternfound = findChessboardCorners(image, patternsize, centers);

  cout << patternfound << endl;
  drawChessboardCorners(image, patternsize, Mat(centers), patternfound);

  imwrite("/home/jake/Pictures/labelled_img.jpg", image);*/
}