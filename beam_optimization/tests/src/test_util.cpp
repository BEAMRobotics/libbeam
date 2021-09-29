#include <test_util.h>

#include <X11/Xlib.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace beam_optimization { namespace test_util {

void OutputTransformInformation(const Eigen::Affine3d& T,
                                const std::string& transform_name) {
  OutputTransformInformation(T.matrix(), transform_name);
}

void OutputTransformInformation(const Eigen::Matrix4d& T,
                                const std::string& transform_name) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  std::cout << transform_name << ":\n"
            << T << "\n"
            << "rpy (deg): [" << beam::Rad2Deg(beam::WrapToPi(rpy[0])) << ", "
            << beam::Rad2Deg(beam::WrapToPi(rpy[1])) << ", "
            << beam::Rad2Deg(beam::WrapToPi(rpy[2])) << "]\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(
    const std::vector<Eigen::Vector4d, beam::AlignVec4d>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i++) {
    pcl::PointXYZ to_add = EigenPointToPCL(points.at(i).head(3));
    return_cloud->push_back(to_add);
  }

  return return_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
    MakePointCloud(const std::vector<Eigen::Vector4d>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i++) {
    pcl::PointXYZ to_add = EigenPointToPCL(points.at(i).head(3));
    return_cloud->push_back(to_add);
  }

  return return_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakePointCloud(
    const std::vector<Eigen::Vector2d, beam::AlignVec2d>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i++) {
    pcl::PointXYZ to_add;
    to_add.x = points.at(i).x();
    to_add.y = points.at(i).y();
    to_add.z = 0;
    return_cloud->push_back(to_add);
  }

  return return_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
    MakePointCloud(const std::vector<Eigen::Vector2d>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (uint16_t i = 0; i < points.size(); i++) {
    pcl::PointXYZ to_add;
    to_add.x = points.at(i).x();
    to_add.y = points.at(i).y();
    to_add.z = 0;
    return_cloud->push_back(to_add);
  }

  return return_cloud;
}

}} // namespace beam_optimization::test_util