/** @file
 * @ingroup cv
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <torch/torch.h>

namespace beam_cv {

struct SuperPoint : torch::nn::Module {
  SuperPoint();

  std::vector<torch::Tensor> forward(torch::Tensor x);

  torch::nn::Conv2d conv1a;
  torch::nn::Conv2d conv1b;

  torch::nn::Conv2d conv2a;
  torch::nn::Conv2d conv2b;

  torch::nn::Conv2d conv3a;
  torch::nn::Conv2d conv3b;

  torch::nn::Conv2d conv4a;
  torch::nn::Conv2d conv4b;

  torch::nn::Conv2d convPa;
  torch::nn::Conv2d convPb;

  // descriptor
  torch::nn::Conv2d convDa;
  torch::nn::Conv2d convDb;

private:
  const int c1_ = 64;
  const int c2_ = 64;
  const int c3_ = 128;
  const int c4_ = 128;
  const int c5_ = 256;
  const int d1_ = 256;
};

/**
 * Add description
 */
class SuperPointModel {
public:
  /**
   * @brief Constructor
   */
  SuperPointModel(const std::string& model_file);

  /**
   * @brief Default destructor
   */
  ~SuperPointModel() = default;

  void Detect(const cv::Mat& img, bool cuda);

  void ComputeDescriptors(const std::vector<cv::KeyPoint>& keypoints,
                          cv::Mat& descriptors);

  void GetKeyPoints(float threshold, int iniX, int maxX, int iniY, int maxY,
                    std::vector<cv::KeyPoint>& keypoints, bool nms);

  // This is set to true once Detect has been called
  bool has_been_initialized{false};

private:
  void NMS(const std::vector<cv::KeyPoint>& det, const cv::Mat& conf,
           std::vector<cv::KeyPoint>& pts, int border, int dist_thresh,
           int img_width, int img_height);
  
  std::shared_ptr<SuperPoint> model_;
  torch::Tensor mProb_;
  torch::Tensor mDesc_;
};

} // namespace beam_cv
