#define CATCH_CONFIG_MAIN
#include "beam_cv/geometry/AbsolutePoseEstimator.h"
#include <catch2/catch.hpp>

TEST_CASE("testing p3p progress") {
    Eigen::Vector2i y1, y2, y3;
    Eigen::Vector3d X1, X2, X3;
    y1(0) = 0.0; y1(1) = 0.0;
    y2(0) = 1.0; y2(1) = 0.0;
    y3(0) = 2.0; y3(1) = 1.0;
    X1(0) = 0.0; X1(1) = 0.0; X1(2) = 2.0;
    X2(0) = 1.41421356237309; X2(1) = 0.0; X2(2) = 1.41421356237309;
    X3(0) = 1.63299316185545; X3(1) = 0.816496580927726; X3(2) =0.816496580927726;
    std::vector<Eigen::Vector2i> pixels{y1,y2};
    std::vector<Eigen::Vector3d> points{X1,X2,X3};
    beam_cv::AbsolutePoseEstimator test;
    test.P3PEstimator(pixels, points)
}