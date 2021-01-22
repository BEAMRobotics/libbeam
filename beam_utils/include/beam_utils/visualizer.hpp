#pragma once

#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/make_shared.hpp>
#include <atomic>

namespace beam { 

/**
 * @brief Interactive visualizer class to display point clouds and correspondences
 * Note: to use: 
 * 1. create visualizer instance
 * 2. call startVis()
 * 3. call desired displayClouds() version
 * 4. call endVis()
 */
class Visualizer{
public: 

    /**
     * @brief Constructor 
     * @param name_ display name
     */
    Visualizer(const std::string name_); 

    /**
     * @brief Empty destructor 
     */
    ~Visualizer(); 

    /**
     * @brief Starts visualizer in a new thread 
     * @param coord_size size of the coordinate axes to display 
     */
    void startVis(uint16_t coord_size = 100); 

    /**
     * @brief Ends visualizer thread
     * @todo get display window to close when called, currently hangs until end of program 
     */
    void endVis();

    /**
     * @brief Method to display one point cloud 
     * @param cloud_ point cloud to display
     * @param id_ unique cloud id for display
     */
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::string id_);

    /**
     * @brief Method to display n clouds, cloud and id vectors must be the same length, 
     *        ids must be unique
     * @param clouds_ point clouds to display
     * @param ids_ unique cloud ids for display
     */
    void displayClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_, 
                            std::vector<std::string> ids_);

    /**
     * @brief Method to display two point clouds
     * @param cloud1_ point cloud to display
     * @param cloud2_ point cloud to display
     * @param id1_ unique cloud id for display
     * @param id2_ unique cloud id for display
     */
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                            std::string id1_,
                            std::string id2_);

    /**
     * @brief Method to display three point clouds
     * @param cloud1_ point cloud to display
     * @param cloud2_ point cloud to display
     * @param cloud3_ point cloud to display
     * @param id1_ unique cloud id for display
     * @param id2_ unique cloud id for display
     * @param id3_ unique cloud id for display
     */
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_,
                       std::string id1_,
                       std::string id2_,
                       std::string id3_);

    /**
     * @brief Method to display four point clouds
     * @param cloud1_ point cloud to display
     * @param cloud2_ point cloud to display
     * @param cloud3_ point cloud to display
     * @param cloud4_ point cloud to display
     * @param id1_ unique cloud id for display
     * @param id2_ unique cloud id for display
     * @param id3_ unique cloud id for display
     * @param id4_ unique cloud id for display
     */
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4_,
                       std::string id1_,
                       std::string id2_,
                       std::string id3_,
                       std::string id4_);

    /**
     * @brief Method to display an image cloud, projected cloud and correspondences 
     * @param image_cloud_ labelled image point cloud
     * @param projected_cloud_ projected CAD point cloud
     * @param corrs_ correspondences between projected point cloud and labelled image cloud
     * @param id_image_ unique cloud id for display
     * @param id_projected_ unique cloud id for display
     */
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                            pcl::CorrespondencesConstPtr corrs_,
                            std::string id_image_,
                            std::string id_projected_);

    /**
     * @brief Method to display an image cloud, CAD cloud, projected cloud and correspondences 
     * @param image_cloud_ labelled image point cloud
     * @param CAD_cloud_ CAD cloud 
     * @param projected_cloud_ projected CAD point cloud
     * @param corrs_ correspondences between projected point cloud and labelled image cloud
     * @param id_image_ unique cloud id for display
     * @param id_CAD_ unique cloud id for display
     * @param id_projected_ unique cloud id for display
     */
    void displayClouds(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                            pcl::CorrespondencesConstPtr corrs_,
                            std::string id_image_,
                            std::string id_CAD_,
                            std::string id_projected_);

private: 
    pcl::visualization::PCLVisualizer::Ptr point_cloud_display;
    std::thread vis_thread;

    //mutex for the point_cloud_display object, held by the main thread when updating the visualization params
    std::mutex mtx;

    std::string display_name;

    std::atomic_flag continueFlag = ATOMIC_FLAG_INIT;
    bool display1_called, display2_called, display3_called, display4_called, 
        display5_called, display6_called, displayv_called;

    //vis thread method in which the visualizer spins
    void spin();

};


} // namespace beam
