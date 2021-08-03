/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <beam_cv/descriptors/Descriptor.h>

namespace beam_cv {

/** Representation of a descriptor extractor using the SIFT algorithm.
 *
 * Internally, this class is wrapping OpenCV's SIFT descriptor module. More info
 * can be found here: http://docs.opencv.org/trunk/db/d95/classcv_1_1SIFT.html
 */
class SIFTDescriptor : public Descriptor {
public:
  struct Params {
    /** The number of best features to retain */
    int num_features = 0;

    /**  The number of layers in each octave */
    int num_octave_layers = 3;

    /** The contrast threshold used to filter out weak
     * features in semi-uniform (low-contrast) regions. The larger the
     * threshold, the less features are produced by the detector. */
    double contrast_threshold = 0.04;

    /** The threshold used to filter out edge-like features.
     * Note that its meaning is different from the contrast_threshold, i.e.
     * the larger the edgeThreshold, the less features are filtered out (more
     * features are retained). */
    double edge_threshold = 10;

    /** The sigma of the Gaussian applied to the input image at the
     * octave #0. If your image is captured with a weak camera with soft lenses,
     * you might want to reduce the number. */
    double sigma = 1.6;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  SIFTDescriptor(const Params& params);

  /**
   * @brief Constructor that takes individual params
   * @param num_features The number of best features to retain
   * @param num_octave_layers The number of layers in each octave
   * @param contrast_threshold The contrast threshold used to filter out weak
   * features in semi-uniform (low-contrast) regions. The larger the threshold,
   * the less features are produced by the detector.
   * @param edge_threshold The threshold used to filter out edge-like features.
   * Note that the its meaning is different from the contrastThreshold, i.e. the
   * larger the edgeThreshold, the less features are filtered out (more features
   * are retained).
   * @param sigma The sigma of the Gaussian applied to the input image at the
   * octave #0. If your image is captured with a weak camera with soft lenses,
   * you might want to reduce the number.
   */
  SIFTDescriptor(int num_features = 0, int num_octave_layers = 3,
                 double contrast_threshold = 0.04, double edge_threshold = 10,
                 double sigma = 1.6);

  /**
   * @brief Default destructor
   */
  ~SIFTDescriptor() override = default;

  /** Extracts descriptors from the keypoints in an image, using the SIFT
   *  descriptor extractor.
   *  @param image the image to detect features in.
   *  @param keypoints the keypoints from the detected image
   *  @return an array containing the computed descriptors.
   */
  cv::Mat ExtractDescriptors(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints);

private:
  // this gets called in each constructor
  void Setup();

  Params params_;

  /** The pointer to the wrapped cv::SIFT object. */
  cv::Ptr<cv::SIFT> sift_descriptor_;

  /** Checks whether the desired configuration is valid. */
  void CheckConfig();
};
} // namespace beam_cv
