#include "beam_utils/math.h"
#include "beam_utils/se3.h"

#include <catch2/catch.hpp>
#include <iostream>

constexpr float PI = 3.14159;

TEST_CASE("Float comparator", "[Math.h]") {
  float f1 = 2.2345642;
  float f2 = 2.2345652;
  REQUIRE(beam::fltcmp(f1, f2, 0.000001) == 0);
  REQUIRE(beam::fltcmp(f1, f2, 0.0000001) == -1);
  REQUIRE(beam::fltcmp(f2, f1, 0.0000001) == 1);
}

TEST_CASE("Median", "[Math.h]") {
  std::vector<double> vec;
  for (int i = 1; i < 10; i++) { vec.push_back(i); }
  REQUIRE(beam::median(vec) == 5);
  vec.push_back(2);
  REQUIRE(beam::median(vec) == 4.5);
}

TEST_CASE("Distance", "[Math.h]") {
  Eigen::Vector3d point1(0, 0, 0);
  Eigen::Vector3d point2(100, 20, 30);
  Eigen::Vector3d point3(100, 10, 20);
  REQUIRE(beam::distance(point1, point2) == Approx(106.301).epsilon(0.01));
  REQUIRE(beam::distance(point1, point3) == Approx(102.47).epsilon(0.01));
  REQUIRE(beam::distance(point3, point2) == Approx(14.1421).epsilon(0.01));
}

TEST_CASE("RoundMatrix", "[Math.h]") {
  Eigen::MatrixXd matrix2x2(2, 2), matrix2x2round3(2, 2), matrix2x2round2(2, 2);
  matrix2x2 << 0.0041, 0.0045, 0.0061, 0.0077;
  matrix2x2round3 << 0.004, 0.005, 0.006, 0.008;
  matrix2x2round2 << 0.00, 0.00, 0.01, 0.01;
  REQUIRE(matrix2x2round3 == beam::RoundMatrix(matrix2x2, 3));
  REQUIRE(matrix2x2round2 == beam::RoundMatrix(matrix2x2, 2));
}

TEST_CASE("IsTransformationMatrix & IsRotationMatrix", "[Math.h]") {
  // Note, this also tests IsRotationMatrix
  Eigen::Matrix4d ValidT1, ValidT2, InvalidT1, InvalidT2, InvalidT3;
  ValidT1.setIdentity();
  ValidT2.setIdentity();
  ValidT2(0, 1) = 0.00001;
  InvalidT1.setIdentity();
  InvalidT1(0, 1) = 2;
  InvalidT2.setIdentity();
  InvalidT2(3, 1) = 1;
  InvalidT3.setIdentity();
  InvalidT3(0, 1) = 0.001;

  REQUIRE(beam::IsTransformationMatrix(ValidT1) == true);
  REQUIRE(beam::IsTransformationMatrix(ValidT2) == true);
  REQUIRE(beam::IsTransformationMatrix(InvalidT1) == false);
  REQUIRE(beam::IsTransformationMatrix(InvalidT2) == false);
  REQUIRE(beam::IsTransformationMatrix(InvalidT3) == false);
}

TEST_CASE("EigenTransformToVector and VectorToEigenTransform") {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) { T(i, j) = beam::randf(-1, 1); }
  }

  std::vector<double> v = beam::EigenTransformToVector(T);

  Eigen::Matrix4d T2 = beam::VectorToEigenTransform(v);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) { REQUIRE(T(i, j) == T2(i, j)); }
  }

  Eigen::Matrix4f Tf = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      T(i, j) = static_cast<float>(beam::randf(-1, 1));
    }
  }

  std::vector<float> vf = beam::EigenTransformToVector(Tf);

  Eigen::Matrix4f Tf2 = beam::VectorToEigenTransform(vf);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) { REQUIRE(Tf(i, j) == Tf2(i, j)); }
  }
}

TEST_CASE("logit and logit inv", "[Math.h]") {
  double l0 = beam::Logit(0.5);
  REQUIRE(beam::LogitInv(l0) == 0.5);
  double pk = 0.7;

  double p1 = beam::BayesianLogitUpdate(pk, l0, 0.5);
  REQUIRE(p1 == pk);

  double p_cur = p1;
  for (int i = 0; i < 20; i++) {
    double p_last = p_cur;
    p_cur = beam::BayesianLogitUpdate(pk, l0, p_last);
    REQUIRE(p_cur > p_last);
    REQUIRE(p_cur < 1);
  }
}

TEST_CASE("Random pose generation", "[se3.h]") {
  int N = 100;
  for (int i = 0; i < N; i++) {
    Eigen::Matrix4d T = beam::GenerateRandomPose(0.0, 1.0);
    REQUIRE(beam::IsTransformationMatrix(T));
  }
}
