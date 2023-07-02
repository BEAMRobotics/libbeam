/** @file
 * @ingroup utils
 *
 * Utility pointcloud functions that do not fit anywhere else.
 *
 */

#pragma once

#define PCL_NO_PRECOMPILE
#include <geometry_msgs/Vector3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/kdtree.h>
#include <beam_utils/pcl_conversions.h>

// Create point types of different lidars
// Velodyne
struct PointXYZIRT {
  PCL_ADD_POINT4D
  float intensity;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using PointCloudIRT = pcl::PointCloud<PointXYZIRT>;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time) // offset in seconds from cloud time 
    )
// clang-format on

// Ouster
struct PointXYZITRRNR {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t time; // time since beginning of scan in nanoseconds
  std::uint16_t reflectivity;
  std::uint8_t ring;
  std::uint16_t noise;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZITRRNR, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, time, time)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, noise, noise)
    (std::uint32_t, range, range))
// clang-format on

/**
 * @brief Enum class for different types of Lidars
 */
enum class LidarType { VELODYNE = 0, OUSTER };

// Map for storing lidar type string input
static std::map<std::string, LidarType> LidarTypeStringMap = {
    {"VELODYNE", LidarType::VELODYNE},
    {"OUSTER", LidarType::OUSTER}};

// function for listing types of lidars available
inline std::string GetLidarTypes() {
  std::string types;
  for (auto it = LidarTypeStringMap.begin(); it != LidarTypeStringMap.end();
       it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

// create a point type for storing poses
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
    (double, time, time)
    )
// clang-format on

#ifndef BEAM_PCL_TYPEDEF
#  define BEAM_PCL_TYPEDEF

/**
 * @brief typedefs for commonly used pcl clouds
 */
typedef pcl::PointXYZRGB PointTypeCol;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef std::shared_ptr<PointCloud> PointCloudPtr;
typedef pcl::PointCloud<PointTypeCol> PointCloudCol;
typedef std::shared_ptr<PointCloudCol> PointCloudColPtr;

#endif // BEAM_PCL_TYPEDEF

namespace beam {
/** @addtogroup utils
 *  @{ */

static ros::Time time_tmp = ros::Time(0);
static std::string _string_tmp = "";
static uint32_t seq_tmp = 0;

/**
 * @brief Convert from a pcl pointcloud to a ROS pointcloud
 * @param cloud pcl pointcloud
 * @param time stamp
 * @param frame_id frame associated with the lidar
 * @param seq scan number
 * @return ros pointcloud
 */
template <typename PointT>
sensor_msgs::PointCloud2 PCLToROS(const pcl::PointCloud<PointT>& cloud,
                                  const ros::Time& time = ros::Time(0),
                                  const std::string& frame_id = "",
                                  uint32_t seq = 0) {
  // Convert to pointcloud2 data type
  pcl::PCLPointCloud2 cloud2;
  pcl::toPCLPointCloud2(cloud, cloud2);

  // Convert to ros msg
  sensor_msgs::PointCloud2 ros_cloud;
  beam::pcl_conversions::fromPCL(cloud2, ros_cloud);

  // update header info
  ros_cloud.header.stamp = time;
  ros_cloud.header.seq = seq;
  ros_cloud.header.frame_id = frame_id;

  return ros_cloud;
}

/**
 * @brief Convert from a ROS pointcloud to a pcl pointcloud
 * @param msg ros pointcloud
 * @param time stamp
 * @param frame_id frame associated with the lidar
 * @param seq scan number
 * @return pcl point cloud
 */
PointCloud ROSToPCL(const sensor_msgs::PointCloud2& msg,
                    ros::Time& time = time_tmp,
                    std::string& frame_id = _string_tmp,
                    uint32_t& seq = seq_tmp);

/**
 * @brief Convert from a ROS pointcloud to a pcl pointcloud
 * @param cloud_out output cloud to fill in
 * @param cloud ros pointcloud
 * @param time stamp
 * @param frame_id frame associated with the lidar
 * @param seq scan number
 * @return pcl point cloud
 */
void ROSToPCL(PointCloud& cloud_out, const sensor_msgs::PointCloud2& msg,
              ros::Time& time = time_tmp, std::string& frame_id = _string_tmp,
              uint32_t& seq = seq_tmp);

/**
 * @brief Convert from a ROS pointcloud to a pcl pointcloud
 * @param cloud_out output cloud to fill in
 * @param cloud ros pointcloud
 * @param time stamp
 * @param frame_id frame associated with the lidar
 * @param seq scan number
 * @return pcl point cloud
 */
void ROSToPCL(pcl::PointCloud<PointXYZIRT>& cloud_out,
              const sensor_msgs::PointCloud2& msg, ros::Time& time = time_tmp,
              std::string& frame_id = _string_tmp, uint32_t& seq = seq_tmp);

/**
 * @brief Convert from a ROS pointcloud to a pcl pointcloud
 * @param cloud_out output cloud to fill in
 * @param cloud ros pointcloud
 * @param time stamp
 * @param frame_id frame associated with the lidar
 * @param seq scan number
 * @return pcl point cloud
 */
void ROSToPCL(pcl::PointCloud<PointXYZITRRNR>& cloud_out,
              const sensor_msgs::PointCloud2& msg, ros::Time& time = time_tmp,
              std::string& frame_id = _string_tmp, uint32_t& seq = seq_tmp);

/**
 * @brief Convert from a pcl pointcloud to a vector of ros vectors
 * @param cloud pcl pointcloud
 * @return ros vector
 */
std::vector<geometry_msgs::Vector3> PCLToROSVector(const PointCloud& cloud);

/**
 * @brief Convert from a pcl pointcloud to a vector of ros vectors. Note we only
 * copy xyz
 * @param cloud pcl pointcloud
 * @return ros vector
 */
std::vector<geometry_msgs::Vector3> PCLToROSVector(const PointCloudIRT& cloud);

/**
 * @brief Convert from a vector of ros vectors to a pcl pointcloud
 * @param vector ros vector
 * @return cloud
 */
PointCloud ROSVectorToPCL(const std::vector<geometry_msgs::Vector3>& vector);

/**
 * @brief Convert from a vector of ros vectors to a pcl pointcloud with IRT
 * fields
 * @param vector ros vector
 * @return cloud
 */
PointCloudIRT
    ROSVectorToPCLIRT(const std::vector<geometry_msgs::Vector3>& vector);

/**
 * @brief converts PointXYZIRPYT to pose + ROS stamp
 * @param point
 * @param time reference to output time
 * @param T reference to output transform
 */
void PCLPointToPose(const PointXYZIRPYT& point, ros::Time& time,
                    Eigen::Matrix4d& T);
/**
 * @brief Add RGB color to a pcl pointcloud
 * @param r red color intensity
 * @param g green color intensity
 * @param b blue color intensity
 * @return colored pointcloud
 */
PointCloudCol ColorPointCloud(const PointCloud& cloud, uint8_t r, uint8_t g,
                              uint8_t b);

/**
 * @brief Add RGB color to a pcl pointcloud
 * @param r red color intensity
 * @param g green color intensity
 * @param b blue color intensity
 * @return colored pointcloud
 */
PointCloudCol ColorPointCloud(const PointCloudIRT& cloud, uint8_t r, uint8_t g,
                              uint8_t b);

/**
 * @brief Add a coordinate frame (built a prebuilt pointcloud) to a color pcl
 * pointcloud
 * @param cloud original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
PointCloudCol
    AddFrameToCloud(const PointCloudCol& cloud, const PointCloudCol& frame,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

/**
 * @brief Add a coordinate frame (built a prebuilt pointcloud) to a pcl
 * pointcloud
 * @param cloud original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
PointCloud
    AddFrameToCloud(const PointCloud& cloud, const PointCloud& frame,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity());

/**
 * @brief Merge a coordinate frame (built a prebuilt pointcloud) to a pcl
 * pointcloud
 * @param cloud reference to original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
void MergeFrameToCloud(pcl::PointCloud<pcl::PointXYZRGBL>& cloud,
                       const pcl::PointCloud<pcl::PointXYZRGBL>& frame,
                       const Eigen::Matrix4d& T);

/**
 * @brief Merge a coordinate frame (built a prebuilt pointcloud) to a pcl
 * pointcloud
 * @param cloud reference to original cloud
 * @param frame poincloud frame to add
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @return pointcloud containing frame
 */
void MergeFrameToCloud(PointCloudCol& cloud, const PointCloudCol& frame,
                       const Eigen::Matrix4d& T);

/**
 * @brief Add a coordinate frame to a color pcl pointcloud. This function calls
 * CreateFrameCol to build the frame. The coordinate frame will be colored RGB
 * for frames XYZ, respectively.
 * @param cloud original cloud
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return color pointcloud containing frame
 */
PointCloudCol
    AddFrameToCloud(const PointCloudCol& cloud,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity(),
                    double increment = 0.01, double length = 0.3);

/**
 * @brief Add a coordinate frame to a pcl pointcloud. This function calls
 * CreateFrameCol to build the frame
 * @param cloud original cloud
 * @param T optional transform to apply to the frame to get it in the cloud
 * reference frame
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return pointcloud containing frame
 */
PointCloud
    AddFrameToCloud(const PointCloud& cloud,
                    const Eigen::Matrix4d& T = Eigen::Matrix4d::Identity(),
                    double increment = 0.01, double length = 0.3);

/**
 * @brief Build a coordinate frame of points.
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return pointcloud frame
 */
PointCloud CreateFrame(double increment = 0.01, double length = 0.3);

/**
 * @brief Build a coordinate frame of points. The coordinate frame will be
 * colored RGB for frames XYZ, respectively.
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return colored pointcloud frame
 */
PointCloudCol CreateFrameCol(double increment = 0.01, double length = 0.3);

/**
 * @brief Build a coordinate frame of points with timestamp as a label.
 * @param t timestamp to add to the point labels
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return pointcloud frame
 */
pcl::PointCloud<pcl::PointXYZL> CreateFrame(const ros::Time& t,
                                            double increment = 0.01,
                                            double length = 0.3);

/**
 * @brief Build a coordinate frame of points with timestamp as a label. The
 * coordinate frame will be colored RGB for frames XYZ, respectively.
 * @param t timestamp to add to the point
 * @param increment distance between points in the frame, in meters
 * @param length frame length in meters
 * @return colored pointcloud frame
 */
pcl::PointCloud<pcl::PointXYZRGBL> CreateFrameCol(const ros::Time& t,
                                                  double increment = 0.01,
                                                  double length = 0.3);

/**
 * @brief draw a line between two points and return as a pointcloud. The point
 * fields from pt1 will be copied to all other points
 *
 * @tparam PointT type of point to make line out of
 * @param pt1 start point of line
 * @param pt2 end point of line
 * @param increment distance between each point
 * @return pcl::PointCloud<PointT>
 */
template <typename PointT>
inline pcl::PointCloud<PointT> DrawLine(const PointT& pt1, const PointT& pt2,
                                        double increment = 0.01) {
  Eigen::Vector3f dir =
      Eigen::Vector3f(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
  double len = dir.norm();
  dir[0] = dir[0] / len;
  dir[1] = dir[1] / len;
  dir[2] = dir[2] / len;

  double cur_length = 0;
  PointT pt = pt1;
  pt.x = 0;
  pt.y = 0;
  pt.z = 0;
  Eigen::Vector3f pt1_eig(pt1.x, pt1.y, pt1.z);

  pcl::PointCloud<PointT> cloud;
  while (cur_length < len) {
    Eigen::Vector3f pt_eig = pt1_eig + cur_length * dir;
    pt.x = pt_eig[0];
    pt.y = pt_eig[1];
    pt.z = pt_eig[2];
    cloud.push_back(pt);
    cur_length += increment;
  }
  return cloud;
}

/**
 * @brief Calculate the absolute distance of the point to the origin.
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float PointDistance(const PointT& p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * @brief Calculate the squared distance of the point to the origin.
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float SquaredPointDistance(const PointT& p) {
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

/**
 * @brief Calculate the squared difference of the given two points.
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float SquaredDiff(const PointT& a, const PointT& b, const float& wb) {
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

/**
 * @brief Calculate the squared difference of the given two points.
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float SquaredDiff(const PointT& a, const PointT& b) {
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

template <typename PointT>
inline void getMinMax3D(const pcl::PointCloud<PointT>& cloud, PointT& min_pt,
                        PointT& max_pt) {
  Eigen::Array4f min_p, max_p;
  min_p.setConstant(FLT_MAX);
  max_p.setConstant(-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense) {
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap();
      min_p = min_p.min(pt);
      max_p = max_p.max(pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else {
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      // Check if the point is invalid
      if (!std::isfinite(cloud.points[i].x) ||
          !std::isfinite(cloud.points[i].y) ||
          !std::isfinite(cloud.points[i].z))
        continue;
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap();
      min_p = min_p.min(pt);
      max_p = max_p.max(pt);
    }
  }
  min_pt.x = min_p[0];
  min_pt.y = min_p[1];
  min_pt.z = min_p[2];
  max_pt.x = max_p[0];
  max_pt.y = max_p[1];
  max_pt.z = max_p[2];
}

/**
 * @brief add random noise to a pointcloud. This will add some perturbation
 * between -max_pert and +max_pert for each axis (x,y,z) of each point
 * @param cloud reference to cloud to update
 * @param max_pert max perturbation (in m) of each axis for a point
 * @param radom_seed if set to true, this will set the random seed based on the
 * current time
 */
void AddNoiseToCloud(PointCloud& cloud, double max_pert = 0.01,
                     bool random_seed = true);

enum PointCloudFileType { PCDBINARY, PCDASCII, PLYBINARY, PLYASCII };

/** Map for storing string input */
static std::map<std::string, PointCloudFileType> PointCloudFileTypeStringMap = {
    {"PCDBINARY", PointCloudFileType::PCDBINARY},
    {"PCDASCII", PointCloudFileType::PCDASCII},
    {"PLYBINARY", PointCloudFileType::PLYBINARY},
    {"PLYASCII", PointCloudFileType::PLYASCII}};

/** Map for storing file extension with each type of point cloud file */
static std::map<PointCloudFileType, std::string>
    PointCloudFileTypeExtensionMap = {{PointCloudFileType::PCDBINARY, ".pcd"},
                                      {PointCloudFileType::PCDASCII, ".pcd"},
                                      {PointCloudFileType::PLYBINARY, ".ply"},
                                      {PointCloudFileType::PLYASCII, ".ply"}};

/** function for listing types of PointCloud files */
inline std::string GetPointCloudFileTypes() {
  std::string types;
  for (auto it = PointCloudFileTypeStringMap.begin();
       it != PointCloudFileTypeStringMap.end(); it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

/**
 * @brief function for saving PCD point clouds. Using the regular pcl i/o will
 * throw exceptions if the point clouds are empty, but we don't always want
 * this. This is a wrapper around the pcl save functions to avoid that and save
 * different types of file
 * @param filename full path to file
 * @param input_cloud
 * @param file_type enum class for point cloud file type
 * @param error_type string with the resulting error if save was unsuccessful
 * @return true if successful
 */
template <class PointT>
inline bool
    SavePointCloud(const std::string& filename,
                   const pcl::PointCloud<PointT>& cloud,
                   PointCloudFileType file_type = PointCloudFileType::PCDBINARY,
                   std::string& error_type = _string_tmp) {
  // check extension
  std::string extension_should_be = PointCloudFileTypeExtensionMap[file_type];
  if (!HasExtension(filename, extension_should_be)) {
    error_type = "Invalid file extension. Input file: " + filename +
                 " . Extension should be: " + extension_should_be;
    return false;
  }

  // check path exists
  boost::filesystem::path path(filename);
  if (!boost::filesystem::exists(path.parent_path())) {
    error_type =
        "File path parent directory does not exist. Input file: " + filename;
    return false;
  }

  // check pointcloud isn't empty
  if (cloud.size() == 0) {
    error_type = "Empty point cloud.";
    return false;
  }

  // try to save cloud
  pcl::PLYWriter writer;
  try {
    switch (file_type) {
      case PointCloudFileType::PCDASCII:
        pcl::io::savePCDFileASCII(filename, cloud);
        break;
      case PointCloudFileType::PCDBINARY:
        pcl::io::savePCDFileBinary(filename, cloud);
        break;
      case PointCloudFileType::PLYASCII:
        writer.write<PointT>(filename, cloud, false);
        break;
      case PointCloudFileType::PLYBINARY:
        writer.write<PointT>(filename, cloud, true);
        break;
    }
  } catch (pcl::PCLException& e) {
    error_type = "Exception throw by pcl: " + std::string(e.detailedMessage());
    return false;
  }

  return true;
}

template <class PointT>
float CalculatePointNorm(PointT p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename PointT>
void ExtractEuclideanClusters(const pcl::PointCloud<PointT>& cloud,
                              const std::shared_ptr<beam::KdTree<PointT>>& tree,
                              float tolerance,
                              std::vector<pcl::PointIndices>& clusters,
                              int min_pts_per_cluster,
                              int max_pts_per_cluster) {
  if (tree->cloud.pts.size() != cloud.size()) {
    BEAM_ERROR("{}: Tree built for a different point cloud "
               "dataset ({}) than the input cloud ({})!",
               __func__, tree->cloud.pts.size(), cloud.size());
    return;
  }
  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed(cloud.size(), false);

  std::vector<uint32_t> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    if (processed[i]) continue;

    pcl::Indices seed_queue;
    int sq_idx = 0;
    seed_queue.push_back(i);

    processed[i] = true;

    while (sq_idx < static_cast<int>(seed_queue.size())) {
      // Search for sq_idx
      const auto point = cloud.points[seed_queue[sq_idx]];
      if (tree->radiusSearch(point, tolerance, nn_indices, nn_distances) > 1) {
        sq_idx++;
        continue;
      }

      for (std::size_t j = 0; j < nn_indices.size(); ++j) {
        if (processed[nn_indices[j]]) { continue; }
        // Perform euclidean clustering
        seed_queue.push_back(static_cast<int>(nn_indices[j]));
        processed[nn_indices[j]] = true;
      }
      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size() >= min_pts_per_cluster &&
        seed_queue.size() <= max_pts_per_cluster) {
      pcl::PointIndices r;
      r.indices.resize(seed_queue.size());
      for (std::size_t j = 0; j < seed_queue.size(); ++j) {
        r.indices[j] = seed_queue[j];
      }

      // These two lines should not be needed: (can anyone confirm?) -FF
      std::sort(r.indices.begin(), r.indices.end());
      r.indices.erase(std::unique(r.indices.begin(), r.indices.end()),
                      r.indices.end());

      r.header = cloud.header;
      clusters.push_back(r);
    } else {
      BEAM_DEBUG(
          "{}: This cluster has {} points, which "
          "is not between {} and {} points, so it is not a final cluster",
          __func__, seed_queue.size(), min_pts_per_cluster,
          max_pts_per_cluster);
    }
  }
}
/** @} group utils */
} // namespace beam
