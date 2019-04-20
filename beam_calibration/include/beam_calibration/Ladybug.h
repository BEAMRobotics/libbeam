#pragma once
#include "beam_calibration/Intrinsics.h"

#include <ladybug/ladybug.h>
#include <ladybug/ladybuggeom.h>
#include <ladybug/ladybugrenderer.h>

namespace beam_calibration {

class Ladybug : public Intrinsics {
public:
  Ladybug(unsigned int id);

  Ladybug() = default;
  ~Ladybug() = default;

  IntrinsicsType GetType() const { return IntrinsicsType::LADYBUG; }

  void LoadJSON(std::string& file) override;

  std::string GetFrameId() override;

  void SetFrameId(std::string& frame_id) override;

  beam::Vec2 GetImgDims() override;

  void SetK(beam::Mat3 K) override;

  beam::Mat3 GetK() override;

  void SetImgDims(beam::Vec2& img_dims) override;

  beam::Vec2 ProjectPoint(beam::Vec3& X) override;

  beam::Vec2 ProjectPoint(beam::Vec4& X) override;

  beam::Vec2 ProjectDistortedPoint(beam::Vec3& X) override;

  beam::Vec2 UndistortPixel(beam::Vec2 pixel_in);
  beam::Vec2 DistortPixel(beam::Vec2 pixel_in);

  beam::Vec2 ApplyProjection(beam::Vec3& X);

  beam::Vec2 ApplyDistortedProjection(beam::Vec3& X);

  bool PixelInImage(beam::Vec2 pixel_in);

  /**
   * @brief Overload for pushing to output streams
   */
  friend std::ostream& operator<<(std::ostream& out, Ladybug& Ladybug);

private:
  void ReadCameraCalibration(std::string calibration_path);

  void LadybugCheckError();

  LadybugContext lb_context_;
  LadybugError lb_error_;

  // Variables for storing intrinsic information
  std::string frame_id_, calibration_date_;
  bool is_K_full_ = false, is_rad_distortion_valid_ = false,
       is_tan_distortion_valid_ = false, is_calibration_date_set_ = false;
  beam::Mat3 K_ = Eigen::Matrix3d::Zero();
  beam::Vec2 tan_coeffs_;
  beam::VecX rad_coeffs_ = Eigen::VectorXd::Random(3);

  unsigned int cam_id_ = 0;
  const unsigned int LB_FULL_WIDTH = 2048;
  const unsigned int LB_FULL_HEIGHT = 2464;
  beam::Vec2 img_dims_ = {LB_FULL_WIDTH, LB_FULL_HEIGHT};
};

}