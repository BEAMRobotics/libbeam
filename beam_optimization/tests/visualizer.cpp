#include "visualizer.hpp"

namespace beam_optimization {

Visualizer::Visualizer(const std::string name_) {
  display_name = name_;
} 

Visualizer::~Visualizer() {
  display1_called = false;
  display2_called = false;
  display3_called = false; 
  display4_called = false;

}

void Visualizer::startVis() {
  point_cloud_display = boost::make_shared<pcl::visualization::PCLVisualizer> (display_name);
  point_cloud_display->setBackgroundColor (0, 0, 0);
  point_cloud_display->addCoordinateSystem (100);
  point_cloud_display->initCameraParameters ();

  printf("Started Vis \n");

  this->continueFlag.test_and_set(std::memory_order_relaxed);

  //start the visualizer spinning in its own thread
  this->vis_thread = std::thread(&Visualizer::spin, this);
}

void Visualizer::endVis() {
  this->continueFlag.clear(std::memory_order_relaxed);
  vis_thread.join();
}

void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::string id_) {
  
  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //if the visualizer does not already contain the point cloud, add it
  if(!display1_called) {
    point_cloud_display->addPointCloud(cloud_, id_,0);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_);
    point_cloud_display->resetCamera();
    printf("Point cloud added \n");
    
  }
  //otherwise, update the existing cloud
  else 
    point_cloud_display->updatePointCloud(cloud_,id_);

  mtx.unlock();

  display1_called = true;

}

// display camera and projected points in 2D without correspondences
void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                                std::string id_image_,
                                std::string id_projected_) {
  
  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //if the visualizer does not already contain the image cloud, add it
  if(!display2_called) {
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_projected_);  
    point_cloud_display->resetCamera();  
  }
  //otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
  }

  mtx.unlock();

  display2_called = true;

}

void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_two_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_three_,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_four_,
                            std::string id_cloud_one_,
                            std::string id_cloud_two_,
                            std::string id_cloud_three_,
                            std::string id_cloud_four_) {

  //get mutex for visulalizer spinning in vis thread and either create a new cloud or update the existing one
  mtx.lock();

  //colorize clouds
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1 (cloud_one_, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb2 (cloud_two_, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb3 (cloud_three_, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb4 (cloud_four_, 0, 0, 255);

  //if the visualizer does not already contain the image cloud, add it
  if(!display3_called) {
    
    point_cloud_display->addPointCloud(cloud_one_, rgb1, id_cloud_one_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_cloud_one_);
    point_cloud_display->resetCamera();

    point_cloud_display->addPointCloud(cloud_two_, rgb2, id_cloud_two_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_cloud_two_);
    point_cloud_display->resetCamera();

    point_cloud_display->addPointCloud(cloud_three_, rgb3, id_cloud_three_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_cloud_three_);
    point_cloud_display->resetCamera();

    point_cloud_display->addPointCloud(cloud_four_, rgb4, id_cloud_four_);
    point_cloud_display->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_cloud_four_);
    point_cloud_display->resetCamera();

  }
  //otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(cloud_one_, rgb1, id_cloud_one_);
    point_cloud_display->updatePointCloud(cloud_two_, rgb2, id_cloud_two_);
    point_cloud_display->updatePointCloud(cloud_three_, rgb3, id_cloud_three_);
    point_cloud_display->updatePointCloud(cloud_four_, rgb4, id_cloud_four_);
  }

  mtx.unlock();

  display3_called = true;

}

//private threaded functions

void Visualizer::spin() {
  printf("in vis thread \n");
  while (this->continueFlag.test_and_set(std::memory_order_relaxed) &&
           !(this->point_cloud_display->wasStopped()))
  {

    mtx.lock();
    point_cloud_display->spinOnce (3);
    mtx.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}




}