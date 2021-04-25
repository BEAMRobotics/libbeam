#include <beam_utils/visualizer.h>
#include <stdio.h>

namespace beam {

Visualizer::Visualizer(const std::string name_) {
  display_name = name_;
  display1_called = false;
  display2_called = false;
  display3_called = false; 
  display4_called = false;
  display5_called = false;
  display6_called = false;
} 

Visualizer::~Visualizer() {

}

void Visualizer::startVis(uint16_t coord_size) {
  point_cloud_display = 
    std::make_shared<pcl::visualization::PCLVisualizer> (display_name);
  point_cloud_display->setBackgroundColor (0, 0, 0);
  point_cloud_display->addCoordinateSystem (coord_size);
  point_cloud_display->initCameraParameters ();

  this->continueFlag.test_and_set(std::memory_order_relaxed);

  // start the visualizer spinning in its own thread
  this->vis_thread = std::thread(&Visualizer::spin, this);
}

void Visualizer::endVis() {
  this->continueFlag.clear(std::memory_order_relaxed);
  vis_thread.join();
}

void Visualizer::displayClouds
  (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_, std::string id_) {
  
  // get mutex for visulalizer spinning in vis thread 
  // and either create a new cloud or update the existing one
  mtx.lock();

  // if the visualizer does not already contain the point cloud, add it
  if(!display1_called) {
    point_cloud_display->addPointCloud(cloud_, id_,0);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_);
    point_cloud_display->resetCamera();    
  }

  // otherwise, update the existing cloud
  else 
    point_cloud_display->updatePointCloud(cloud_,id_);

  mtx.unlock();

  display1_called = true;

}

void Visualizer::displayClouds
  (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_, 
   std::vector<std::string> ids_) {
  
  // get mutex for visulalizer spinning in vis thread 
  // and either create a new cloud or update the existing one
  mtx.lock();

  // if the visualizer does not already contain the point clouds, add it
  if(!displayv_called) {
    for (uint8_t i = 0; i < clouds_.size(); i ++) {
      point_cloud_display->addPointCloud(clouds_[i], ids_[i],0);
      point_cloud_display->setPointCloudRenderingProperties 
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ids_[i]);
      point_cloud_display->resetCamera();   
    }
 
  }

  // otherwise, update the existing clouds
  else {
    for (uint8_t i = 0; i < clouds_.size(); i ++) {
       point_cloud_display->updatePointCloud(clouds_[i],ids_[i]);
    }
  }

  mtx.unlock();

  displayv_called = true;

}

void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                                std::string id1_,
                                std::string id2_) {
  
  // get mutex for visulalizer spinning in vis thread 
  // and either create a new cloud or update the existing one
  mtx.lock();

  // if the visualizer does not already contain the image cloud, add it
  if(!display2_called) {
    point_cloud_display->addPointCloud(cloud1_, id1_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id1_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(cloud2_, id2_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id2_);  
    point_cloud_display->resetCamera();  
  }

  // otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(cloud1_, id1_);
    point_cloud_display->updatePointCloud(cloud2_, id2_);
  }

  mtx.unlock();

  display2_called = true;

}

// display three clouds with no correspondences
void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_,
                       std::string id1_,
                       std::string id2_,
                       std::string id3_) {

  // get mutex for visulalizer spinning in vis thread 
  // and either create a new cloud or update the existing one
  mtx.lock();

  // if the visualizer does not already contain the image cloud, add it
  if(!display3_called) {
    point_cloud_display->addPointCloud(cloud1_, id1_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id1_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(cloud2_, id2_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id2_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(cloud3_, id3_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id3_);
    point_cloud_display->resetCamera();
  }
  // otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(cloud1_, id1_);
    point_cloud_display->updatePointCloud(cloud2_, id2_);
    point_cloud_display->updatePointCloud(cloud3_, id3_);
  }

  mtx.unlock();

  display3_called = true;

}

// display three clouds with no correspondences
void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3_,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4_,
                       std::string id1_,
                       std::string id2_,
                       std::string id3_,
                       std::string id4_) {

  // get mutex for visulalizer spinning in vis thread and 
  // either create a new cloud or update the existing one
  mtx.lock();

  // if the visualizer does not already contain the image cloud, add it
  if(!display3_called) {
    point_cloud_display->addPointCloud(cloud1_, id1_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id1_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(cloud2_, id2_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id2_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(cloud3_, id3_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id3_);
    point_cloud_display->resetCamera();
    point_cloud_display->addPointCloud(cloud4_, id4_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id4_);
    point_cloud_display->resetCamera();
  }

  // otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(cloud1_, id1_);
    point_cloud_display->updatePointCloud(cloud2_, id2_);
    point_cloud_display->updatePointCloud(cloud3_, id3_);
    point_cloud_display->updatePointCloud(cloud3_, id4_);
  }

  mtx.unlock();

  display4_called = true;

}

void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud_,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                                pcl::CorrespondencesConstPtr corrs_,
                                std::string id_image_,
                                std::string id_projected_) {

  // get mutex for visulalizer spinning in vis thread and 
  // either create a new cloud or update the existing one
  mtx.lock();

  // if the visualizer does not already contain the image cloud, add it
  if(!display3_called) {
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_projected_); 
  }
  // otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
  }    

  // remove all correspondence lines and redraw
  point_cloud_display->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1; 
  uint16_t line_id = 0;

  // illustrate correspondences
  for (uint16_t i = 0; i < corrs_->size(); i++) {
    uint16_t proj_point_index = corrs_->at(i).index_query;
    uint16_t cam_point_index = corrs_->at(i).index_match;

    point_cloud_display->addLine(projected_cloud_->at(proj_point_index), 
                                 image_cloud_->at(cam_point_index),
                                 0, 255, 0, std::to_string(line_id));
    line_start_index += 2;
    line_end_index += 2;
    line_id ++;
  }

  mtx.unlock();

  display5_called = true;

}

void Visualizer::displayClouds(pcl::PointCloud<pcl::PointXYZ>::ConstPtr image_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_cloud_,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_,
                        pcl::CorrespondencesConstPtr corrs_,
                        std::string id_image_,
                        std::string id_CAD_,
                        std::string id_projected_) {

  // get mutex for visulalizer spinning in vis thread and 
  // either create a new cloud or update the existing one
  mtx.lock();

  // if the visualizer does not already contain the image cloud, add it
  if(!display6_called) {
    point_cloud_display->addPointCloud(image_cloud_, id_image_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_image_);
    point_cloud_display->addPointCloud(CAD_cloud_, id_CAD_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_CAD_);
    point_cloud_display->addPointCloud(projected_cloud_, id_projected_);
    point_cloud_display->setPointCloudRenderingProperties 
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id_projected_); 
    point_cloud_display->resetCamera();
  }

  // otherwise, update the existing cloud
  else {
    point_cloud_display->updatePointCloud(image_cloud_, id_image_);
    point_cloud_display->updatePointCloud(CAD_cloud_, id_CAD_);
    point_cloud_display->updatePointCloud(projected_cloud_, id_projected_);
    point_cloud_display->resetCamera();
  }    

  // remove all correspondence lines and redraw
  point_cloud_display->removeAllShapes();

  uint16_t line_start_index = 0, line_end_index = 1; 
  uint16_t line_id = 0;

  // illustrate correspondences
  for (uint16_t i = 0; i < corrs_->size(); i++) {

    uint16_t proj_point_index = corrs_->at(i).index_query;
    uint16_t cam_point_index = corrs_->at(i).index_match;

    point_cloud_display->addLine(projected_cloud_->at(proj_point_index), 
                                 image_cloud_->at(cam_point_index),
                                 0, 255, 0, std::to_string(line_id));
    line_start_index += 2;
    line_end_index += 2;
    line_id ++;

  }

  mtx.unlock();

  display6_called = true;

}

void Visualizer::spin() {
  while (this->continueFlag.test_and_set(std::memory_order_relaxed) &&
           !(this->point_cloud_display->wasStopped()))
  {
    mtx.lock();
    point_cloud_display->spinOnce (3);
    mtx.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}




} // namespace beam