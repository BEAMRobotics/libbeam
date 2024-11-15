#define CATCH_CONFIG_MAIN
#include "beam_containers/ImageBridge.h"
#include <boost/filesystem.hpp>
#include <catch2/catch.hpp>

TEST_CASE("Testing loading different ImageBridge objects from json") {
  beam_containers::ImageBridge img_container1, img_container2;

  std::string file_location_root = __FILE__;
  file_location_root.erase(file_location_root.end() - 25,
                           file_location_root.end());
  file_location_root = file_location_root + "test_data/";
  std::string file_location1 = file_location_root + "ImageBridge1";
  std::string file_location2 = file_location_root + "ImageBridge2";
  REQUIRE_NOTHROW(img_container1.LoadFromJSON(file_location1));
  REQUIRE_NOTHROW(img_container2.LoadFromJSON(file_location2));
  REQUIRE(img_container1.GetBagName() ==
          "/home/nick/bag_files/ig_scans/2019_02_13_Structures_Lab/"
          "ig_scan_2019-02-13-19-44-24.bag");
  REQUIRE(img_container1.GetBGRFrameId() == "F1_link");
  REQUIRE(img_container1.GetBGRIsDistorted() == true);
  REQUIRE(img_container1.GetBGRMaskMethod() == "");
  REQUIRE(img_container1.GetImageSeq() == 1);
  REQUIRE(img_container1.GetIRFrameId() == "");
  REQUIRE(img_container1.GetIRIsDistorted() == false);
  REQUIRE(img_container1.GetIRMaskMethod() == "");
  REQUIRE(img_container1.IsBGRImageSet() == true);
  REQUIRE(img_container1.IsBGRMaskSet() == false);
  REQUIRE(img_container1.IsIRImageSet() == false);
  REQUIRE(img_container1.IsIRMaskSet() == false);
  REQUIRE(img_container1.GetTimePoint().time_since_epoch().count() ==
          1550105082718658048);

  REQUIRE(img_container2.GetBagName() ==
          "/home/nick/bag_files/ig_scans/2019_02_13_Structures_Lab/"
          "ig_scan_2019-02-13-19-44-24.bag");
  REQUIRE(img_container2.GetBGRFrameId() == "F1_link");
  REQUIRE(img_container2.GetBGRIsDistorted() == true);
  REQUIRE(img_container2.GetBGRMaskMethod() == "deep_learning");
  REQUIRE(img_container2.GetImageSeq() == 2);
  REQUIRE(img_container2.GetIRFrameId() == "F2_link");
  REQUIRE(img_container2.GetIRIsDistorted() == false);
  REQUIRE(img_container2.GetIRMaskMethod() == "deep_learning");
  REQUIRE(img_container2.IsBGRImageSet() == true);
  REQUIRE(img_container2.IsBGRMaskSet() == true);
  REQUIRE(img_container2.IsIRImageSet() == true);
  REQUIRE(img_container2.IsIRMaskSet() == true);
  REQUIRE(img_container2.GetTimePoint().time_since_epoch().count() ==
          1550105082718658048);
}

TEST_CASE("Testing building image from scratch") {
  beam_containers::ImageBridge img_container;
  std::string file_location_root = __FILE__;
  file_location_root.erase(file_location_root.end() - 25,
                           file_location_root.end());
  file_location_root = file_location_root + "test_data/ImageBridge2/";
  std::string bgr_path = file_location_root + "BGRImage.jpg";
  std::string bgr_mask_path = file_location_root + "BGRMask.png";
  std::string ir_path = file_location_root + "IRImage.jpg";
  std::string ir_mask_path = file_location_root + "IRMask.png";

  std::string bagname = "bagname", frame1 = "F1_link", frame2 = "F2_link",
              mask_method1 = "deep_learning1", mask_method2 = "deep_learning2";
  int seq = 2;
  bool bool_true = true, bool_false = false;
  img_container.SetBagName(bagname);
  img_container.SetBGRFrameId(frame1);
  img_container.SetBGRIsDistorted(bool_true);
  img_container.SetBGRMaskMethod(mask_method1);
  img_container.SetImageSeq(seq);
  img_container.SetIRFrameId(frame2);
  img_container.SetIRIsDistorted(bool_false);
  img_container.SetIRMaskMethod(mask_method2);

  cv::Mat bgr_img = cv::imread(bgr_path);
  img_container.SetBGRImage(bgr_img);
  cv::Mat bgr_mask = cv::imread(bgr_mask_path);
  img_container.SetBGRMask(bgr_mask);
  cv::Mat ir_img = cv::imread(ir_path);
  img_container.SetIRImage(ir_img);
  cv::Mat ir_mask = cv::imread(ir_mask_path);
  img_container.SetIRMask(ir_mask);

  REQUIRE(img_container.GetBagName() == "bagname");
  REQUIRE(img_container.GetBGRFrameId() == "F1_link");
  REQUIRE(img_container.GetBGRIsDistorted() == true);
  REQUIRE(img_container.GetBGRMaskMethod() == "deep_learning1");
  REQUIRE(img_container.GetImageSeq() == 2);
  REQUIRE(img_container.GetIRFrameId() == "F2_link");
  REQUIRE(img_container.GetIRIsDistorted() == false);
  REQUIRE(img_container.GetIRMaskMethod() == "deep_learning2");

  REQUIRE(img_container.IsBGRImageSet() == true);
  REQUIRE(img_container.IsBGRMaskSet() == true);
  REQUIRE(img_container.IsIRImageSet() == true);
  REQUIRE(img_container.IsIRMaskSet() == true);
}
