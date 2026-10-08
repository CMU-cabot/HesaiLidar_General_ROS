#include <gtest/gtest.h>
#include <limits>
#include "pointcloud_message.hpp"

TEST(MeasurementPointCloud, DelayedPublicationPreservesAcquisitionTimeAndPoints)
{
  PPointCloud cloud;
  cloud.header.frame_id = "lidar";
  cloud.header.stamp = 1900000000000000ULL;  // Deliberately different publication time.
  PPoint point{};
  point.x = 1.25f;
  point.y = -2.0f;
  point.z = 0.5f;
  point.timestamp = 1700000000.125;
  point.ring = 3;
  cloud.push_back(point);
  const auto output = hesai_lidar::measurementPointCloud(cloud, point.timestamp);
  EXPECT_EQ(output.header.stamp.sec, 1700000000);
  EXPECT_EQ(output.header.stamp.nanosec, 125000000u);
  EXPECT_EQ(output.header.frame_id, "lidar");
  PPointCloud decoded;
  pcl::fromROSMsg(output, decoded);
  ASSERT_EQ(decoded.size(), 1u);
  EXPECT_EQ(decoded[0].timestamp, point.timestamp);
  EXPECT_EQ(decoded[0].x, point.x);
  EXPECT_EQ(decoded[0].ring, point.ring);
}

TEST(MeasurementPointCloud, RejectsInvalidClockRatherThanRestampingAsCurrent)
{
  PPointCloud cloud;
  for (double stamp : {0.0, -1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN(), 2147483648.0}) {
    EXPECT_THROW(hesai_lidar::measurementPointCloud(cloud, stamp), std::invalid_argument);
  }
}
