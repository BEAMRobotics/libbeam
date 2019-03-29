#pragma once

namespace beam_calibration {

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class IntrinsicsType { PINHOLE = 0, FISHEYE, LADYBUG };

/**
 * @brief Abstract class for calibrations
 */
class Intrinsics {
public:
  /**
   * @brief Default constructor
   */
  Intrinsics() = default;

  /**
   * @brief Default destructor
   */
  virtual ~Intrinsics() = default;

  /**
   * @brief Pure virtual method for returning type of intrinsics
   * @return
   */
  virtual IntrinsicsType GetType() const = 0;
};

} // namespace beam_calibration
