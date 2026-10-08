#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pandarGeneral/point_types.h"

namespace hesai_lidar {
// The SDK callback supplies the first point's acquisition time. Never replace
// it with publication time: queued scans must retain their measurement time.
inline sensor_msgs::msg::PointCloud2 measurementPointCloud(
    const PPointCloud & cloud, double timestamp)
{
  if (!std::isfinite(timestamp) || timestamp <= 0.0 ||
      timestamp >= static_cast<double>(std::numeric_limits<int32_t>::max())) {
    throw std::invalid_argument("measurement timestamp is outside ROS time range");
  }
  const int64_t seconds = static_cast<int64_t>(timestamp);
  const int64_t ns = seconds * 1000000000LL +
    static_cast<int64_t>(std::llround((timestamp - seconds) * 1e9));
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  output.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return output;
}
}  // namespace hesai_lidar
