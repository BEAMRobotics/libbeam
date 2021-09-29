#define CATCH_CONFIG_MAIN
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Geometry>
#include <catch2/catch.hpp>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <CamPoseReprojectionCost.h>
#include <beam_utils/angles.h>
#include <beam_utils/math.h>
#include <beam_utils/visualizer.h>
#include <test_util.h>

namespace beam_optimization {

const bool VISUALIZATION = false;

using namespace beam;

ceres::Solver::Options ceres_solver_options_;
std::unique_ptr<ceres::LossFunction> loss_function_;
std::unique_ptr<ceres::LocalParameterization> se3_parameterization_;
bool output_results_{false};

std::shared_ptr<ceres::Problem> SetupCeresProblem() {
  // set ceres solver params
  ceres_solver_options_.minimizer_progress_to_stdout = false;
  ceres_solver_options_.max_num_iterations = 50;
  ceres_solver_options_.max_solver_time_in_seconds = 1e6;
  ceres_solver_options_.function_tolerance = 1e-8;
  ceres_solver_options_.gradient_tolerance = 1e-10;
  ceres_solver_options_.parameter_tolerance = 1e-8;
  ceres_solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;

  // set ceres problem options
  ceres::Problem::Options ceres_problem_options;

  // if we want to manage our own data for these, we can set these flags:
  ceres_problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;

  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_problem_options);

  loss_function_ =
      std::unique_ptr<ceres::LossFunction>(new ceres::HuberLoss(1.0));

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));
  se3_parameterization_ = std::unique_ptr<ceres::LocalParameterization>(
      new ceres::ProductParameterization(quat_parameterization.release(),
                                         identity_parameterization.release()));

  return problem;
}

void SolveProblem(const std::shared_ptr<ceres::Problem>& problem,
                  bool output_results) {
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_solver_options_, problem.get(), &ceres_summary);
  if (output_results) {
    LOG_INFO("Done.");
    LOG_INFO("Outputting ceres summary:");
    std::string report = ceres_summary.FullReport();
    std::cout << report << "\n";
  }
}

/******************************************************************************************************************/
// TEST CASE 1
// radtan camera model projection without noise
// check convergence with unperturbed input
// check convergence with slightly perturbed input
/******************************************************************************************************************/

TEST_CASE("Test rt projection - no noise") {
  // create point clouds for results visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_p; // input cloud perturbed
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      input_cloud_p_proj; // input cloud perturbed projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      final_cloud_p_proj; // final solved cloud projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud; // target "detected" cloud
  beam::Visualizer test_vis("test_1_vis");

  // create keypoints
  std::vector<Eigen::Vector4d, beam::AlignVec4d> points;
  double max_distance_x = 2, max_distance_y = 2, max_distance_z = 4;
  for (int i = 0; i < 80; i++) {
    double x = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_x;
    double y = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_y;
    double z = ((double)std::rand() / (RAND_MAX)-0) * 1 * max_distance_z;
    Eigen::Vector4d point(x, y, z, 1);
    points.push_back(point);
  }

  // Create intrinsics
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 40, file_location.end());
  file_location += "/config/CamFactorIntrinsics.json";
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(file_location);
  // Create initial transform
  Eigen::Matrix4d T_CW = Eigen::Matrix4d::Identity();
  T_CW(2, 3) = 5; // simple z translation to start

  // create perturbed initial
  Eigen::VectorXd perturbation(6, 1);
  perturbation << 0.3, -0.3, 0.3, 0.5, -0.5, 0.3;
  Eigen::Matrix4d T_CW_pert = beam::PerturbTransformDegM(T_CW, perturbation);

  // create projected (detected) points - no noise
  std::vector<Eigen::Vector2d, beam::AlignVec2d> pixels(points.size());
  std::vector<bool> pixels_valid(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector4d point_transformed = T_CW * points[i];
    Eigen::Vector2d pixel;
    bool in_image = false;
    bool in_domain = camera_model->ProjectPoint(point_transformed.hnormalized(),
                                                pixel, in_image);
    if (in_domain && in_image) {
      pixels_valid[i] = true;
      pixels[i] = pixel;
    } else {
      pixels_valid[i] = false;
    }
  }

  // Visualization - create target, input_cloud_p, input_cloud_p_proj cloud
  if (VISUALIZATION) {
    target_cloud = test_util::MakePointCloud(pixels);

    std::vector<Eigen::Vector4d, beam::AlignVec4d> perturbed_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> perturbed_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      perturbed_points[i] = T_CW_pert * points[i];
      Eigen::Vector2d perturbed_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(
          perturbed_points[i].hnormalized(), perturbed_pixel, in_image);
      if (in_image && in_domain) {
        perturbed_pixels[i] = perturbed_pixel;
      } else {
        Eigen::Vector2d zero(0, 0);
        perturbed_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    input_cloud_p = test_util::MakePointCloud(perturbed_points);
    input_cloud_p_proj = test_util::MakePointCloud(perturbed_pixels);
  }

  // create values to optimize
  Eigen::Matrix3d R1 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
  std::vector<double> results_perfect_init{
      q1.w(), q1.x(), q1.y(), q1.z(), T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};
  Eigen::Matrix3d R2 = T_CW_pert.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q2 = Eigen::Quaternion<double>(R2);
  std::vector<double> results_perturbed_init{
      q2.w(),          q2.x(),          q2.y(),         q2.z(),
      T_CW_pert(0, 3), T_CW_pert(1, 3), T_CW_pert(2, 3)};

  // build problems
  std::shared_ptr<ceres::Problem> problem1 = SetupCeresProblem();
  std::shared_ptr<ceres::Problem> problem2 = SetupCeresProblem();

  problem1->AddParameterBlock(&(results_perfect_init[0]), 7,
                              se3_parameterization_.get());
  problem2->AddParameterBlock(&(results_perturbed_init[0]), 7,
                              se3_parameterization_.get());

  for (size_t i = 0; i < points.size(); i++) {
    if (pixels_valid[i]) {
      Eigen::Vector3d P_CAMERA = points[i].hnormalized();

      // add residuals for perfect init

      std::unique_ptr<ceres::CostFunction> cost_function1(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));

      problem1->AddResidualBlock(cost_function1.release(), loss_function_.get(),
                                 &(results_perfect_init[0]));

      // add residuals for perturbed init
      std::unique_ptr<ceres::CostFunction> cost_function2(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));
      problem2->AddResidualBlock(cost_function2.release(), loss_function_.get(),
                                 &(results_perturbed_init[0]));

      // Check that the inputs are correct:
      Eigen::Vector2d pixel_projected;
      bool in_image = false;
      REQUIRE(camera_model->ProjectPoint((T_CW * points[i]).hnormalized(),
                                         pixel_projected, in_image));
      REQUIRE(pixel_projected.isApprox(pixels[i], 1e-5));
    }
  }

  Eigen::Matrix3d R3 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q3 = Eigen::Quaternion<double>(R3);
  std::vector<double> initial{q3.w(),     q3.x(),     q3.y(),    q3.z(),
                              T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};

  LOG_INFO("TESTING WITH PERFECT INITIALIZATION");
  SolveProblem(problem1, output_results_);
  Eigen::Matrix4d T_CW_opt1 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perfect_init);

  LOG_INFO("TESTING WITH PERTURBED INITIALIZATION");
  SolveProblem(problem2, output_results_);
  Eigen::Matrix4d T_CW_opt2 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perturbed_init);

  if (VISUALIZATION) {
    std::vector<Eigen::Vector4d, beam::AlignVec4d> final_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> final_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      final_points[i] = T_CW_opt2 * points[i];

      Eigen::Vector2d final_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(final_points[i].hnormalized(),
                                                  final_pixel, in_image);
      if (in_image && in_domain) {
        final_pixels[i] = final_pixel;
      } else {
        Eigen::Vector2d zero(0, 0);
        final_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    final_cloud_p_proj = test_util::MakePointCloud(final_pixels);

    test_vis.startVis();
    // white, red, green, blue
    test_vis.displayClouds(target_cloud, input_cloud_p, input_cloud_p_proj,
                           final_cloud_p_proj, "target", "in_pert",
                           "in_pert_proj", "final");

    char end = ' ';

    while (end != 'r') { cin >> end; }

    test_vis.endVis();
  }

  REQUIRE(beam::RoundMatrix(T_CW, 5) == beam::RoundMatrix(T_CW_opt1, 5));
  REQUIRE(beam::RoundMatrix(T_CW, 5) == beam::RoundMatrix(T_CW_opt2, 5));
}

/******************************************************************************************************************/
// TEST CASE 2
// radtan camera model projection with noise in detected pixels
// check convergence with unperturbed input
// check convergence with slightly perturbed input
/******************************************************************************************************************/

TEST_CASE("Test rt projection - with noise") {
  // create point clouds for results visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_p; // input cloud perturbed
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      input_cloud_p_proj; // input cloud perturbed projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      final_cloud_p_proj; // final solved cloud projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud; // target "detected" cloud
  beam::Visualizer test2_vis("test_2_vis");

  // create keypoints
  std::vector<Eigen::Vector4d, beam::AlignVec4d> points;
  double max_distance_x = 2, max_distance_y = 2, max_distance_z = 4;
  for (int i = 0; i < 80; i++) {
    double x = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_x;
    double y = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_y;
    double z = ((double)std::rand() / (RAND_MAX)-0) * 1 * max_distance_z;
    Eigen::Vector4d point(x, y, z, 1);
    points.push_back(point);
  }

  // Create intrinsics
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 40, file_location.end());
  file_location += "/config/CamFactorIntrinsics.json";
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(file_location);

  // Create initial transform
  Eigen::Matrix4d T_CW = Eigen::Matrix4d::Identity();
  T_CW(2, 3) = 5; // simple z translation to start

  // create perturbed initial (different from first test case)
  Eigen::VectorXd perturbation(6, 1);
  perturbation << 0.2, -0.4, 0.1, 0.7, -0.3, 0.4;
  Eigen::Matrix4d T_CW_pert = beam::PerturbTransformDegM(T_CW, perturbation);

  // create projected (detected) points - with noise
  std::vector<Eigen::Vector2d, beam::AlignVec2d> pixels(points.size());
  std::vector<bool> pixels_valid(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector4d point_transformed = T_CW * points[i];

    Eigen::Vector2d pixel;
    bool in_image = false;
    bool in_domain = camera_model->ProjectPoint(point_transformed.hnormalized(),
                                                pixel, in_image);
    if (in_image && in_domain) {
      // generate noise in the projected pixel (+/- 2 pixels)
      auto rng_x = std::mt19937(std::random_device{}());
      auto rng_y = std::mt19937(std::random_device{}());
      std::uniform_real_distribution<double> dist(0, 4.0);
      double noise_x = dist(rng_x) - 2;
      double noise_y = dist(rng_y) - 2;
      pixel.x() += (int)noise_x;
      pixel.y() += (int)noise_y;
      pixels_valid[i] = true;
      pixels[i] = pixel;
    } else {
      pixels_valid[i] = false;
    }
  }

  // Visualization - create target, input_cloud_p, input_cloud_p_proj cloud
  if (VISUALIZATION) {
    target_cloud = test_util::MakePointCloud(pixels);

    std::vector<Eigen::Vector4d, beam::AlignVec4d> perturbed_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> perturbed_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      perturbed_points[i] = T_CW_pert * points[i];
      Eigen::Vector2d perturbed_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(
          perturbed_points[i].hnormalized(), perturbed_pixel, in_image);
      if (in_image && in_domain) {
        perturbed_pixels[i] = perturbed_pixel;
      } else {
        Eigen::Vector2d zero(0, 0);
        perturbed_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    input_cloud_p = test_util::MakePointCloud(perturbed_points);
    input_cloud_p_proj = test_util::MakePointCloud(perturbed_pixels);
  }

  // create values to optimize
  Eigen::Matrix3d R1 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
  std::vector<double> results_perfect_init{
      q1.w(), q1.x(), q1.y(), q1.z(), T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};
  Eigen::Matrix3d R2 = T_CW_pert.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q2 = Eigen::Quaternion<double>(R2);
  std::vector<double> results_perturbed_init{
      q2.w(),          q2.x(),          q2.y(),         q2.z(),
      T_CW_pert(0, 3), T_CW_pert(1, 3), T_CW_pert(2, 3)};

  // build problems
  std::shared_ptr<ceres::Problem> problem1 = SetupCeresProblem();
  std::shared_ptr<ceres::Problem> problem2 = SetupCeresProblem();

  problem1->AddParameterBlock(&(results_perfect_init[0]), 7,
                              se3_parameterization_.get());
  problem2->AddParameterBlock(&(results_perturbed_init[0]), 7,
                              se3_parameterization_.get());

  for (size_t i = 0; i < points.size(); i++) {
    if (pixels_valid[i]) {
      Eigen::Vector3d P_CAMERA = points[i].hnormalized();

      // add residuals for perfect init
      std::unique_ptr<ceres::CostFunction> cost_function1(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));

      problem1->AddResidualBlock(cost_function1.release(), loss_function_.get(),
                                 &(results_perfect_init[0]));

      // add residuals for perturbed init
      std::unique_ptr<ceres::CostFunction> cost_function2(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));
      problem2->AddResidualBlock(cost_function2.release(), loss_function_.get(),
                                 &(results_perturbed_init[0]));

      // Check that the inputs are correct (within the noise level):
      Eigen::Vector2d pixel_projected;
      bool in_image = false;
      REQUIRE(camera_model->ProjectPoint((T_CW * points[i]).hnormalized(),
                                         pixel_projected, in_image));
      REQUIRE(pixel_projected.isApprox(pixels[i], 2.1));
    }
  }

  Eigen::Matrix3d R3 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q3 = Eigen::Quaternion<double>(R3);
  std::vector<double> initial{q3.w(),     q3.x(),     q3.y(),    q3.z(),
                              T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};

  LOG_INFO("TESTING WITH NOISY PERFECT INITIALIZATION");
  SolveProblem(problem1, output_results_);
  Eigen::Matrix4d T_CW_opt1 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perfect_init);

  LOG_INFO("TESTING WITH NOISY PERTURBED INITIALIZATION");
  SolveProblem(problem2, output_results_);
  Eigen::Matrix4d T_CW_opt2 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perturbed_init);

  if (VISUALIZATION) {
    std::vector<Eigen::Vector4d, beam::AlignVec4d> final_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> final_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      final_points[i] = T_CW_opt2 * points[i];
      Eigen::Vector2d final_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(final_points[i].hnormalized(),
                                                  final_pixel, in_image);
      if (in_image && in_domain) {
        final_pixels[i] = final_pixel;
      } else {
        Eigen::Vector2d zero(0, 0);
        final_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    final_cloud_p_proj = test_util::MakePointCloud(final_pixels);

    test2_vis.startVis();
    // white, red, green, blue
    test2_vis.displayClouds(target_cloud, input_cloud_p, input_cloud_p_proj,
                            final_cloud_p_proj, "target", "in_pert",
                            "in_pert_proj", "final");

    char end = ' ';

    while (end != 'r') { cin >> end; }

    test2_vis.endVis();
  }

  // with noise, precision will be lower than perfect projection
  REQUIRE(beam::RoundMatrix(T_CW, 1) == beam::RoundMatrix(T_CW_opt1, 1));
  REQUIRE(beam::RoundMatrix(T_CW, 1) == beam::RoundMatrix(T_CW_opt2, 1));
}

/******************************************************************************************************************/
// TEST CASE 3
// radtan camera model projection with initial perturbed projection outside of
// camera frame check convergence with perturbed cloud projecting out of the
// camera frame (some points clipped)
/******************************************************************************************************************/

TEST_CASE("Test rt projection - with clipping") {
  // create point clouds for results visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_p; // input cloud perturbed
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      input_cloud_p_proj; // input cloud perturbed projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      final_cloud_p_proj; // final solved cloud projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud; // target "detected" cloud
  beam::Visualizer test3_vis("test_3_vis");

  // create keypoints (larger spread than other test cases so perturbation for
  // clipping doesn't have to be too great)
  std::vector<Eigen::Vector4d, beam::AlignVec4d> points;
  double max_distance_x = 4, max_distance_y = 4, max_distance_z = 4;
  for (int i = 0; i < 80; i++) {
    double x = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_x;
    double y = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_y;
    double z = ((double)std::rand() / (RAND_MAX)-0) * 1 * max_distance_z;
    Eigen::Vector4d point(x, y, z, 1);
    points.push_back(point);
  }

  // Create intrinsics
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 40, file_location.end());
  file_location += "/config/CamFactorIntrinsics.json";
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(file_location);
  // Create initial transform
  Eigen::Matrix4d T_CW = Eigen::Matrix4d::Identity();
  T_CW(2, 3) = 5; // simple z translation to start

  // create perturbed initial (different from first test case)
  Eigen::VectorXd perturbation(6, 1);
  perturbation << 0, 0, 0, 0.7, -5, 0.4;
  Eigen::Matrix4d T_CW_pert = beam::PerturbTransformDegM(T_CW, perturbation);

  // create projected (detected) points - no noise
  std::vector<Eigen::Vector2d, beam::AlignVec2d> pixels(points.size());
  std::vector<bool> pixels_valid(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector4d point_transformed = T_CW * points[i];
    Eigen::Vector2d pixel;
    bool in_image = false;
    bool in_domain = camera_model->ProjectPoint(point_transformed.hnormalized(),
                                                pixel, in_image);
    if (in_image && in_domain) {
      pixels_valid[i] = true;
      pixels[i] = pixel;
    } else {
      pixels_valid[i] = false;
    }
  }

  // Visualization - create target, input_cloud_p, input_cloud_p_proj cloud
  if (VISUALIZATION) {
    target_cloud = test_util::MakePointCloud(pixels);

    std::vector<Eigen::Vector4d, beam::AlignVec4d> perturbed_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> perturbed_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      perturbed_points[i] = T_CW_pert * points[i];
      Eigen::Vector2d perturbed_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(
          perturbed_points[i].hnormalized(), perturbed_pixel, in_image);

      if (in_image && in_domain) {
        perturbed_pixels[i] = perturbed_pixel;
        printf("projected in frame \n");
      } else {
        printf("initial perturbed point projected out-of-frame \n");
        Eigen::Vector2d zero(0, 0);
        perturbed_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    input_cloud_p = test_util::MakePointCloud(perturbed_points);
    input_cloud_p_proj = test_util::MakePointCloud(perturbed_pixels);
  }

  // create values to optimize
  Eigen::Matrix3d R1 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
  std::vector<double> results_perfect_init{
      q1.w(), q1.x(), q1.y(), q1.z(), T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};
  Eigen::Matrix3d R2 = T_CW_pert.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q2 = Eigen::Quaternion<double>(R2);
  std::vector<double> results_perturbed_init{
      q2.w(),          q2.x(),          q2.y(),         q2.z(),
      T_CW_pert(0, 3), T_CW_pert(1, 3), T_CW_pert(2, 3)};

  // build problems
  std::shared_ptr<ceres::Problem> problem1 = SetupCeresProblem();
  std::shared_ptr<ceres::Problem> problem2 = SetupCeresProblem();

  problem1->AddParameterBlock(&(results_perfect_init[0]), 7,
                              se3_parameterization_.get());
  problem2->AddParameterBlock(&(results_perturbed_init[0]), 7,
                              se3_parameterization_.get());

  for (size_t i = 0; i < points.size(); i++) {
    if (pixels_valid[i]) {
      Eigen::Vector3d P_CAMERA = points[i].hnormalized();

      // add residuals for perfect init
      std::unique_ptr<ceres::CostFunction> cost_function1(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));

      problem1->AddResidualBlock(cost_function1.release(), loss_function_.get(),
                                 &(results_perfect_init[0]));

      // add residuals for perturbed init
      std::unique_ptr<ceres::CostFunction> cost_function2(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));
      problem2->AddResidualBlock(cost_function2.release(), loss_function_.get(),
                                 &(results_perturbed_init[0]));

      // Check that the inputs are correct (within the noise level):
      Eigen::Vector2d pixel_projected;
      bool in_image = false;
      REQUIRE(camera_model->ProjectPoint((T_CW * points[i]).hnormalized(),
                                         pixel_projected, in_image));
      REQUIRE(pixel_projected.isApprox(pixels[i], 1e-5));
    }
  }

  Eigen::Matrix3d R3 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q3 = Eigen::Quaternion<double>(R3);
  std::vector<double> initial{q3.w(),     q3.x(),     q3.y(),    q3.z(),
                              T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};

  LOG_INFO("TESTING WITH PERFECT INITIALIZATION");
  SolveProblem(problem1, output_results_);
  Eigen::Matrix4d T_CW_opt1 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perfect_init);

  LOG_INFO("TESTING WITH CLIPPED PERTURBED INITIALIZATION");
  SolveProblem(problem2, output_results_);
  Eigen::Matrix4d T_CW_opt2 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perturbed_init);

  if (VISUALIZATION) {
    std::vector<Eigen::Vector4d, beam::AlignVec4d> final_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> final_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      final_points[i] = T_CW_opt2 * points[i];
      Eigen::Vector2d final_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(final_points[i].hnormalized(),
                                                  final_pixel, in_image);
      if (in_image && in_domain) {
        final_pixels[i] = final_pixel;
      } else {
        Eigen::Vector2d zero(0, 0);
        final_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    final_cloud_p_proj = test_util::MakePointCloud(final_pixels);

    test3_vis.startVis();
    // white, red, green, blue
    test3_vis.displayClouds(target_cloud, input_cloud_p, input_cloud_p_proj,
                            final_cloud_p_proj, "target", "in_pert",
                            "in_pert_proj", "final");

    char end = ' ';

    while (end != 'r') { cin >> end; }

    test3_vis.endVis();
  }

  // with noise, precision will be lower than perfect projection
  REQUIRE(beam::RoundMatrix(T_CW, 5) == beam::RoundMatrix(T_CW_opt1, 5));
  REQUIRE(beam::RoundMatrix(T_CW, 5) == beam::RoundMatrix(T_CW_opt2, 5));
}

/******************************************************************************************************************/
// TEST CASE 4
// radtan camera model projection with initial perturbed projection outside of
// camera model domain ceres solution should fail on first iteration with
// "Solution Failed, Residual and Jacobian evaluation failed"
/******************************************************************************************************************/

TEST_CASE("Test rt projection - with invalid initial pose") {
  // create point clouds for results visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_p; // input cloud perturbed
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      input_cloud_p_proj; // input cloud perturbed projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      final_cloud_p_proj; // final solved cloud projected
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud; // target "detected" cloud
  beam::Visualizer test3_vis("test_3_vis");

  // create keypoints (larger spread than other test cases so perturbation for
  // clipping doesn't have to be too great)
  std::vector<Eigen::Vector4d, beam::AlignVec4d> points;
  double max_distance_x = 4, max_distance_y = 4, max_distance_z = 4;
  for (int i = 0; i < 80; i++) {
    double x = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_x;
    double y = ((double)std::rand() / (RAND_MAX)-0.5) * 2 * max_distance_y;
    double z = ((double)std::rand() / (RAND_MAX)-0) * 1 * max_distance_z;
    Eigen::Vector4d point(x, y, z, 1);
    points.push_back(point);
  }

  // Create intrinsics
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 40, file_location.end());
  file_location += "/config/CamFactorIntrinsics.json";
  std::cout << file_location << std::endl;
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(file_location);

  // Create initial transform
  Eigen::Matrix4d T_CW = Eigen::Matrix4d::Identity();
  T_CW(2, 3) = 4; // simple z translation to start

  // create perturbed initial (different from first test case)
  Eigen::VectorXd perturbation(6, 1);
  perturbation << 0, 0, 0, 0, 0, -10;
  Eigen::Matrix4d T_CW_pert = beam::PerturbTransformDegM(T_CW, perturbation);

  // create projected (detected) points - no noise
  std::vector<Eigen::Vector2d, beam::AlignVec2d> pixels(points.size());
  std::vector<bool> pixels_valid(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector4d point_transformed = T_CW * points[i];
    Eigen::Vector2d pixel;
    bool in_image = false;
    bool in_domain = camera_model->ProjectPoint(point_transformed.hnormalized(),
                                                pixel, in_image);
    if (in_image && in_domain) {
      pixels_valid[i] = true;
      pixels[i] = pixel;
    } else {
      pixels_valid[i] = false;
    }
  }

  // Visualization - create target, input_cloud_p, input_cloud_p_proj cloud
  if (VISUALIZATION) {
    target_cloud = test_util::MakePointCloud(pixels);

    std::vector<Eigen::Vector4d, beam::AlignVec4d> perturbed_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> perturbed_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      perturbed_points[i] = T_CW_pert * points[i];
      Eigen::Vector2d perturbed_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(
          perturbed_points[i].hnormalized(), perturbed_pixel, in_image);
      if (in_image && in_domain) {
        perturbed_pixels[i] = perturbed_pixel;
      } else {
        Eigen::Vector2d zero(0, 0);
        perturbed_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    input_cloud_p = test_util::MakePointCloud(perturbed_points);
    input_cloud_p_proj = test_util::MakePointCloud(perturbed_pixels);
  }

  // create values to optimize
  Eigen::Matrix3d R1 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q1 = Eigen::Quaternion<double>(R1);
  std::vector<double> results_perfect_init{
      q1.w(), q1.x(), q1.y(), q1.z(), T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};
  Eigen::Matrix3d R2 = T_CW_pert.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q2 = Eigen::Quaternion<double>(R2);
  std::vector<double> results_perturbed_init{
      q2.w(),          q2.x(),          q2.y(),         q2.z(),
      T_CW_pert(0, 3), T_CW_pert(1, 3), T_CW_pert(2, 3)};
  std::vector<double> initial_perturbed_init = results_perturbed_init;

  // build problems
  std::shared_ptr<ceres::Problem> problem1 = SetupCeresProblem();
  std::shared_ptr<ceres::Problem> problem2 = SetupCeresProblem();

  problem1->AddParameterBlock(&(results_perfect_init[0]), 7,
                              se3_parameterization_.get());
  problem2->AddParameterBlock(&(results_perturbed_init[0]), 7,
                              se3_parameterization_.get());

  for (size_t i = 0; i < points.size(); i++) {
    if (pixels_valid[i]) {
      Eigen::Vector3d P_CAMERA = points[i].hnormalized();

      // add residuals for perfect init
      std::unique_ptr<ceres::CostFunction> cost_function1(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));

      problem1->AddResidualBlock(cost_function1.release(), loss_function_.get(),
                                 &(results_perfect_init[0]));

      // add residuals for perturbed init
      std::unique_ptr<ceres::CostFunction> cost_function2(
          CeresReprojectionCostFunction::Create(pixels[i], P_CAMERA,
                                                camera_model));
      problem2->AddResidualBlock(cost_function2.release(), loss_function_.get(),
                                 &(results_perturbed_init[0]));

      // Check that the inputs are correct (within the noise level):
      Eigen::Vector2d pixel_projected;
      bool in_image = false;
      REQUIRE(camera_model->ProjectPoint((T_CW * points[i]).hnormalized(),
                                         pixel_projected, in_image));
      REQUIRE(pixel_projected.isApprox(pixels[i], 1e-5));
    }
  }

  Eigen::Matrix3d R3 = T_CW.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q3 = Eigen::Quaternion<double>(R3);
  std::vector<double> initial{q3.w(),     q3.x(),     q3.y(),    q3.z(),
                              T_CW(0, 3), T_CW(1, 3), T_CW(2, 3)};

  LOG_INFO("TESTING WITH PERFECT INITIALIZATION");
  SolveProblem(problem1, output_results_);
  Eigen::Matrix4d T_CW_opt1 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perfect_init);

  LOG_INFO("TESTING WITH INVALID PERTURBED INITIALIZATION");
  SolveProblem(problem2, output_results_);
  Eigen::Matrix4d T_CW_opt2 =
      beam::QuaternionAndTranslationToTransformMatrix(results_perturbed_init);

  if (VISUALIZATION) {
    std::vector<Eigen::Vector4d, beam::AlignVec4d> final_points(points.size());
    std::vector<Eigen::Vector2d, beam::AlignVec2d> final_pixels(points.size());

    for (size_t i = 0; i < points.size(); i++) {
      final_points[i] = T_CW_opt2 * points[i];
      Eigen::Vector2d final_pixel;
      bool in_image = false;
      bool in_domain = camera_model->ProjectPoint(final_points[i].hnormalized(),
                                                  final_pixel, in_image);
      if (in_image && in_domain) {
        final_pixels[i] = final_pixel;
      } else {
        Eigen::Vector2d zero(0, 0);
        final_pixels[i] =
            zero; // just set missing pixels to zero for visualization
      }
    }

    final_cloud_p_proj = test_util::MakePointCloud(final_pixels);

    test3_vis.startVis();
    // white, red, green, blue
    test3_vis.displayClouds(target_cloud, input_cloud_p, input_cloud_p_proj,
                            final_cloud_p_proj, "target", "in_pert",
                            "in_pert_proj", "final");

    char end = ' ';

    while (end != 'r') { cin >> end; }

    test3_vis.endVis();
  }

  REQUIRE(beam::RoundMatrix(T_CW, 5) == beam::RoundMatrix(T_CW_opt1, 5));

  // require that solution with invalid inital pose fails without modifying pose
  REQUIRE(results_perturbed_init == initial_perturbed_init);
}

} // namespace beam_optimization