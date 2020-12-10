/** @file
 * @ingroup cv
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <torch/torch.h>

namespace beam_cv {

/**
 * This is an implementation of the SuperPoint model from:
 * https://arxiv.org/abs/1712.07629
 *
 * The python implementation of the model can be found here, along with the
 * pretrained model: https://github.com/magicleap/SuperPointPretrainedNetwork
 *
 * The C++ implementation was also based on (but modified from) this:
 * https://github.com/KinglittleQ/SuperPoint_SLAM
 * Note that the c++ version of the pretrained model was also taken from
 * SuperPoint_SLAM
 */
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
};

/**
 * This class implements all basic functionality for the SuperPoint model. It
 * relies on the SuperPoint struct and implements the methods for computing
 * keypoints and descriptors based on the input parameters and input image.
 *
 * IMPORTANT NOTE: Unlike most other keypoints detectors/descriptors, the
 * features and descriptors are calculated at the same time for an input image
 * (in the Detect() function). Therefore, the order to run the functions must be
 * the following:
 *    1. Detect()
 *    2. GetKeyPoints()
 *    3. ComputeDescriptors()
 * If (1) and (2) are called, then (1) is called on a new image, the descriptors
 * will be overridden so you cannot call GetDescriptors from an old set of
 * keypoint from (2)
 */
class SuperPointModel {
public:
  /**
   * @brief Constructor
   * @param model_file pretrained model file in c++ format. An example is stored
   * in beam_cv/data/models/superpoint.pt which was retrived from
   * https://github.com/KinglittleQ/SuperPoint_SLAM. For information on
   * convreting from a python trained model to a c++ script, see:
   * https://pytorch.org/tutorials/advanced/cpp_export.html
   */
  SuperPointModel(const std::string& model_file);

  /**
   * @brief Default destructor
   */
  ~SuperPointModel() = default;

  /**
   * @brief Detect keypoints and descriptors
   * @param img input image
   * @param cuda whether or not to use cuda. You will need to have the cuda
   * version of libtorch installed. If cuda is unavailable and this bool is set
   * to true, it will not attempt to run on cuda.
   */
  void Detect(const cv::Mat& img, bool cuda);

  /**
   * @brief Compute the descriptors given a set of keypoints.
   * @param keypoints keypoint locations to extract
   * @param descriptors reference to the descriptors mat that will be filled in
   */
  void ComputeDescriptors(const std::vector<cv::KeyPoint>& keypoints,
                          cv::Mat& descriptors);

  /**
   * @brief Get the keypoints that have beeen detected in the Detect method,
   * after applying the following filtering, respectively:
   *    1. Keep only keypoints with confidence higher than conf_threshold
   *    2. Apply non maximum suppression which also sets a minimum distance
   * between keypoints and removes keypoints outside border
   *    3. Select top features (up to max_features) by taking the strongest
   * features. To ensure an even distribution in the selectd keypoints, this
   * separates the image into a grid (of size grid_size) and calculates the
   * maximum K number of features per grid and selects top K features in each
   * grid. This will ensure a maximum bound on the features but this will not
   * necessarily be achieved if too many features have been filtered in the
   * previous stage, or if there are grids with less than K features.
   * @param keypoints reference to keypoint vector to be filled in
   * @param conf_threshold confidence threshold. This need to be manually tuned
   * @param border number of border pixels to exclude features from
   * @param nms_dist_threshold minimum threshold (in pixels) separation between
   * selected keypoints
   * @param max_features maximum number of features to extract. If we to 0, it
   * will use all features that have passed the above three threshold parameters
   * @param grid_size when selecting maximum number of features to store, it
   * will divide the image into equal grid sizes and take K maximum confidence
   * features in each grid. If set to zero, then it will not break the image
   * into grids before extracting top features.
   */
  void GetKeyPoints(std::vector<cv::KeyPoint>& keypoints,
                    float conf_threshold = 0.1, int border = 0,
                    int nms_dist_threshold = 0, int max_features = 0,
                    int grid_size = 0);

  // This is set to true once Detect has been called
  bool has_been_initialized{false};

private:
  void NMS(const std::vector<cv::KeyPoint>& det, const cv::Mat& conf,
           std::vector<cv::KeyPoint>& pts, int border, int dist_thresh,
           int img_width, int img_height);

  void SelectKBestFromGrid(const std::vector<cv::KeyPoint>& keypoints_in,
                           std::vector<cv::KeyPoint>& keypoints_out,
                           int max_features, int grid_size, int border);

  std::shared_ptr<SuperPoint> model_;
  torch::Tensor mProb_;
  torch::Tensor mDesc_;
};

} // namespace beam_cv
