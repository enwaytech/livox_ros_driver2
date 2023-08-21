/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 */

#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <vector>

#include "livox_ros_driver2/livox_point.h"

namespace livox_ros 
{

/// \brief PointXYZI with extra information of the radial distance from the frame's origin in the XY plane
struct PCLLivoxPointXyzrtlt
{
  PCLLivoxPointXyzrtlt() = default;

  inline explicit PCLLivoxPointXyzrtlt(const livox_ros::LivoxPointXyzrtlt& pt)
    : x {pt.x}
    , y {pt.y}
    , z {pt.z}
    , reflectivity {pt.reflectivity}
    , tag {pt.tag}
    , line {pt.line}
    , timestamp {pt.timestamp}
  {
  }

  PCL_ADD_POINT4D; // add the base PointXYZ (which has data[4])
  float reflectivity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


} // namespace livox_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::PCLLivoxPointXyzrtlt,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, reflectivity, intensity)
  (std::uint8_t, tag, tag)
  (std::uint8_t, line, line)
  (double, timestamp, timestamp)
);
// clang-format on
