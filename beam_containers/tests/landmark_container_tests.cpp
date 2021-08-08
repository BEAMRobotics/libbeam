#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <boost/filesystem.hpp>

#include <beam_containers/LandmarkContainer.h>
#include <beam_containers/LandmarkMeasurement.h>
#include <beam_utils/math.h>
#include <beam_cv/descriptors/Descriptor.h>

std::string output_path = "/tmp/landmark_container_tests/";

using namespace beam_containers;

TEST(LandmarkContainer, ReadWrite) {
  LandmarkContainer<LandmarkMeasurement> landmarks;

  for (uint64_t i = 0; i < 5; i++) {
    // create landmark data
    ros::Time t = ros::Time(i) + ros::Duration(beam::randf(0, 1));
    uint8_t sensor_id = 3;
    uint64_t landmark_id = beam::randi(0, 100);
    uint64_t image = i;
    Eigen::Vector2d value;
    value[0] = beam::randi(0, 640);
    value[1] = beam::randi(0, 480);

    // create desriptor
    std::vector<uint8_t> data;
    for (int k = 0; k < 8; k++) {
      uint8_t new_value = static_cast<uint8_t>(beam::randf(0, 255));
      data.push_back(new_value);
    }

    cv::Mat descriptor(1, data.size(), CV_8U, data.data());

    // create measurement and add to the container
    LandmarkMeasurement m(t, sensor_id, landmark_id, image, value, descriptor);
    landmarks.Insert(m);
  }

  // save to json
  boost::filesystem::create_directory(output_path);
  landmarks.SaveToJson(output_path + "landmarks.json");

  // read json
  LandmarkContainer<LandmarkMeasurement> landmarks_read;
  landmarks_read.LoadFromJson(output_path + "landmarks.json");
  // landmarks_read.SaveToJson(output_path + "landmarks_read.json");
  boost::filesystem::remove_all(output_path);

  // compare
  EXPECT_TRUE(landmarks.size() == landmarks_read.size());

  auto all_landmarks = landmarks.GetTimeWindow(ros::TIME_MIN, ros::TIME_MAX);
  for (auto m_iter = all_landmarks.first; m_iter != all_landmarks.second;
       m_iter++) {
    LandmarkMeasurement m_match = landmarks_read.GetMeasurement(
        m_iter->time_point, m_iter->sensor_id, m_iter->landmark_id);
    EXPECT_TRUE(m_iter->image == m_match.image);
    EXPECT_TRUE(m_iter->value[0] == m_match.value[0]);
    EXPECT_TRUE(m_iter->value[1] == m_match.value[1]);
    for (uint8_t desc_iter = 0; desc_iter < m_iter->descriptor.cols;
         desc_iter++) {
      uint8_t value = m_iter->descriptor.at<uint8_t>(0, desc_iter);
      uint8_t value_read = m_match.descriptor.at<uint8_t>(0, desc_iter);
      EXPECT_TRUE(value == value_read);
    }
  }
}
