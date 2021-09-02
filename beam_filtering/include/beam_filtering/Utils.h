/** @file
 * @ingroup filtering
 */

#pragma once

#include <beam_filtering/Filters.h>
#include <nlohmann/json.hpp>

namespace beam_filtering {
/// @addtogroup filtering

using FilterParamsType = std::pair<FilterType, std::vector<double>>;

/**
 * @brief print the options for filter params, and any requirements
 */
void PrintFilterParamsOptions();

/**
 * @brief function for loading params for DROR filter
 * @param J input json
 * @param params reference to vector of doubles representing the params
 * @return true if successful and passed all tests
 */
bool LoadDRORParams(const nlohmann::json& J, std::vector<double>& params);

/**
 * @brief function for loading params for ROR filter
 * @param J input json
 * @param params reference to vector of doubles representing the params
 * @return true if successful and passed all tests
 */
bool LoadRORParams(const nlohmann::json& J, std::vector<double>& params);

/**
 * @brief function for loading params for CropBox filter
 * @param J input json
 * @param params reference to vector of doubles representing the params
 * @return true if successful and passed all tests
 */
bool LoadCropBoxParams(const nlohmann::json& J, std::vector<double>& params);

/**
 * @brief function for loading params for Voxel filter
 * @param J input json
 * @param params reference to vector of doubles representing the params
 * @return true if successful and passed all tests
 */
bool LoadVoxelDownsamplingParams(const nlohmann::json& J,
                                 std::vector<double>& params);

/**
 * @brief Method for loading a filter params object from a json. See
 * filter_params.json in test_data for example of the format of each filter
 * @param J json
 * @return filter parameters
 */
FilterParamsType GetFilterParams(const nlohmann::json& J);

/**
 * @brief Method for loading a vector of filter params from a json. See
 * filter_params.json in test_data for example of the format of each filter
 * @param J json with list of filter params
 * @return vector of filter parameters
 */
std::vector<FilterParamsType> LoadFilterParamsVector(const nlohmann::json& J);

/**
 * @brief method for filtering a point cloud based on a list of filters with
 * their associated parameters
 * @param cloud point cloud to filter
 * @param filter_params
 * @return filtered_cloud
 */
template <class PointT>
inline pcl::PointCloud<PointT>
    FilterPointCloud(const pcl::PointCloud<PointT>& cloud,
                     const std::vector<FilterParamsType>& filter_params) {
  pcl::PointCloud<PointT> filtered_cloud = cloud;
  for (uint8_t i = 0; i < filter_params.size(); i++) {
    std::shared_ptr<pcl::PointCloud<PointT>> input_cloud =
        std::make_shared<pcl::PointCloud<PointT>>(filtered_cloud);
    FilterType filter_type = filter_params[i].first;
    std::vector<double> params = filter_params[i].second;
    if (filter_type == FilterType::DROR) {
      beam_filtering::DROR<PointT> outlier_removal;
      outlier_removal.SetRadiusMultiplier(params[0]);
      outlier_removal.SetAzimuthAngle(params[1]);
      outlier_removal.SetMinNeighbors(params[2]);
      outlier_removal.SetMinSearchRadius(params[3]);
      outlier_removal.SetInputCloud(input_cloud);
      outlier_removal.Filter();
      filtered_cloud = outlier_removal.GetFilteredCloud();
    } else if (filter_type == FilterType::ROR) {
      beam_filtering::ROR<PointT> outlier_removal;
      outlier_removal.SetRadiusSearch(params[0]);
      outlier_removal.SetMinNeighbors(params[2]);
      outlier_removal.SetInputCloud(input_cloud);
      outlier_removal.Filter();
      filtered_cloud = outlier_removal.GetFilteredCloud();
    } else if (filter_type == FilterType::VOXEL) {
      beam_filtering::VoxelDownsample<PointT> downsampler;
      downsampler.SetVoxelSize(
          Eigen::Vector3f(params[0], params[1], params[2]));
      downsampler.SetInputCloud(input_cloud);
      downsampler.Filter();
      filtered_cloud = downsampler.GetFilteredCloud();
    } else if (filter_type == FilterType::CROPBOX) {
      Eigen::Vector3d min_vec(params[0], params[1], params[2]);
      Eigen::Vector3d max_vec(params[3], params[4], params[5]);
      Eigen::Vector3f min_vecf = min_vec.cast<float>();
      Eigen::Vector3f max_vecf = max_vec.cast<float>();
      beam_filtering::CropBox<PointT> cropper;
      cropper.SetMinVector(min_vecf);
      cropper.SetMaxVector(max_vecf);
      if (params[6] == 1) {
        cropper.SetRemoveOutsidePoints(true);
      } else {
        cropper.SetRemoveOutsidePoints(false);
      }
      cropper.SetInputCloud(input_cloud);
      cropper.Filter();
      filtered_cloud = cropper.GetFilteredCloud();
    }
  }
  return filtered_cloud;
}

} // namespace beam_filtering
