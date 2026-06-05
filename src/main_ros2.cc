// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <hesai_lidar/msg/pandar_scan.hpp>
#include <hesai_lidar/msg/pandar_packet.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include "std_msgs/msg/string.hpp"
// #define PRINT_FLAG 

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std;
namespace hesai_lidar
{
namespace
{
rclcpp::Time timeFromUnixSeconds(double timestamp)
{
  const auto seconds = static_cast<int64_t>(std::floor(timestamp));
  auto nanoseconds =
    static_cast<uint32_t>(std::round((timestamp - static_cast<double>(seconds)) * 1e9));
  if (nanoseconds >= 1000000000u) {
    nanoseconds -= 1000000000u;
    return rclcpp::Time(seconds + 1, nanoseconds, RCL_SYSTEM_TIME);
  }
  return rclcpp::Time(seconds, nanoseconds, RCL_SYSTEM_TIME);
}

double timeMsgToUnixSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
}  // namespace

class HesaiLidarClient: public rclcpp::Node
{
public:
  HesaiLidarClient():Node("hesai_lidar"),
  diagnostics_(this)
  { 
    this->declare_parameter<std::string>("pcap_file", "");
    this->declare_parameter<std::string>("server_ip", "");
    this->declare_parameter<int>("lidar_recv_port", 2368);
    this->declare_parameter<int>("gps_port", 10110);
    this->declare_parameter<double>("start_angle", 0.0);
    this->declare_parameter<std::string>("lidar_correction_file", "");
    this->declare_parameter<std::string>("lidar_type", "");
    this->declare_parameter<std::string>("frame_id", "");
    this->declare_parameter<int>("pcldata_type", 0);
    this->declare_parameter<std::string>("publish_type", "");
    this->declare_parameter<std::string>("timestamp_type", "");
    this->declare_parameter<std::string>("data_type", "");
    this->declare_parameter<std::string>("multicast_ip", "");
    this->declare_parameter<bool>("coordinate_correction_flag", false);
    this->declare_parameter<std::string>("target_frame", "");
    this->declare_parameter<std::string>("fixed_frame", "");
    this->declare_parameter<double>("target_fps", 10.0);
    this->declare_parameter<double>("diagnostics_log_period_sec", 5.0);
    this->declare_parameter<double>("diagnostics_warn_packet_age_sec", 0.5);
    rclcpp::QoS qos(rclcpp::KeepLast(7)); 
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    lidarPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar", sensor_qos);
    packetPublisher = this->create_publisher<hesai_lidar::msg::PandarScan>("pandar_packets", qos);
    this->timer_callback();
  }


private:

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::msg::PandarScan::SharedPtr scan) // the timestamp from first point cloud of cld
  {
    const auto callback_start = std::chrono::steady_clock::now();
    const rclcpp::Time publish_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const rclcpp::Time lidar_stamp = timeFromUnixSeconds(timestamp);
    const rclcpp::Time now = this->now();
    sensor_msgs::msg::PointCloud2 output;
    bool published_points = false;
    bool published_packets = false;
    if(m_sPublishType == "both" || m_sPublishType == "points"){
      pcl_conversions::toPCL(now, cld->header.stamp);
      pcl::toROSMsg(*cld, output);
      lidarPublisher->publish(output);
      diag_pointcloud_->tick(now);
      published_points = true;
#ifdef PRINT_FLAG
        std::cout.setf(ios::fixed);
        std::cout << "timestamp: " << std::setprecision(10) << timestamp << ", point size: " << cld->points.size() << std::endl;
#endif        
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher->publish(*scan);
      int64_t seconds = static_cast<int64_t>(timestamp);
      uint32_t nanoseconds = static_cast<uint32_t>((timestamp - seconds) * 1e9);
      diag_packet_->tick(rclcpp::Time(seconds, nanoseconds));
      published_packets = true;
#ifdef PRINT_FLAG
        std::cout << "raw size: "<< scan->packets.size() << std::endl;
#endif
    }
    logPipelineDiagnostics(
      publish_time, lidar_stamp, callback_start, cld->points.size(), output.data.size(),
      scan, published_points, published_packets);
  }

  void gpsCallback(int timestamp) {
#ifdef PRINT_FLAG
      std::cout << "gps: " << timestamp << std::endl;
#endif      
  }

  void scanCallback(const hesai_lidar::msg::PandarScan::SharedPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

  void timer_callback()
  {
    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string lidarType;
    string frameId;
    int pclDataType;
    string pcapFile;
    string dataType;
    string multicastIp;
    bool coordinateCorrectionFlag;
    string targetFrame;
    string fixedFrame;

    this->get_parameter("pcap_file", pcapFile);
    this->get_parameter("server_ip", serverIp);
    this->get_parameter("lidar_recv_port", lidarRecvPort);
    this->get_parameter("gps_port", gpsPort);
    this->get_parameter("start_angle", startAngle);
    this->get_parameter("lidar_correction_file", lidarCorrectionFile);
    this->get_parameter("lidar_type", lidarType);
    this->get_parameter("frame_id", frameId);
    this->get_parameter("pcldata_type", pclDataType);
    this->get_parameter("publish_type", m_sPublishType);
    this->get_parameter("timestamp_type", m_sTimestampType);
    this->get_parameter("data_type", dataType);
    this->get_parameter("multicast_ip", multicastIp);
    this->get_parameter("coordinate_correction_flag", coordinateCorrectionFlag);
    this->get_parameter("target_frame", targetFrame);
    this->get_parameter("fixed_frame", fixedFrame);
    this->get_parameter("background_b", targetFrame);
    this->get_parameter("diagnostics_log_period_sec", diagnostics_log_period_sec_);
    this->get_parameter("diagnostics_warn_packet_age_sec", diagnostics_warn_packet_age_sec_);
  
    // diagnostic updater
    this->get_parameter("target_fps", target_fps_);
    std::string deviceName = std::string("HesaiLidar ") + lidarType;
    diagnostics_.setHardwareID(deviceName);
    diag_pointcloud_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
      "hesai_pointcloud", diagnostics_, diagnostic_updater::FrequencyStatusParam(
        &target_fps_, &target_fps_, 0.1, 2),
        diagnostic_updater::TimeStampStatusParam());

    diag_packet_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
      "hesai_packets", diagnostics_, diagnostic_updater::FrequencyStatusParam(
        &target_fps_, &target_fps_, 0.1, 2),
        diagnostic_updater::TimeStampStatusParam());

    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, std::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile, \
      coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        std::ifstream fin(lidarCorrectionFile);
        if (fin.is_open()) {
          std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
          int length = 0;
          std::string strlidarCalibration;
          fin.seekg(0, std::ios::end);
          length = fin.tellg();
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[length];
          fin.read(buffer, length);
          fin.close();
          strlidarCalibration = buffer;
          int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
          if (ret != 0) {
            std::cout << "Load correction file from " << lidarCorrectionFile <<" failed" << std::endl;
          } else {
            std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
          }
        }
        else{
          std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
        }
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", std::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, \
      lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
      if (hsdk != NULL) {
        packetSubscriber = this->create_subscription<hesai_lidar::msg::PandarScan>("pandar_packets", 10, std::bind(&HesaiLidarClient::scanCallback, this, std::placeholders::_1));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        std::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        std::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId,\
         m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
    }
    
    if (hsdk != NULL) {
        hsdk->Start();
        // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
    } else {
        printf("create sdk fail\n");
    }
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPublisher;
  rclcpp::Publisher<hesai_lidar::msg::PandarScan>::SharedPtr packetPublisher;
  rclcpp::TimerBase::SharedPtr timer_;
  PandarGeneralSDK* hsdk;
  string m_sPublishType;
  string m_sTimestampType;
  rclcpp::Subscription<hesai_lidar::msg::PandarScan>::SharedPtr packetSubscriber;
  diagnostic_updater::Updater diagnostics_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> diag_pointcloud_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> diag_packet_;
  double target_fps_;
  double diagnostics_log_period_sec_;
  double diagnostics_warn_packet_age_sec_;

  void logPipelineDiagnostics(
    const rclcpp::Time & publish_time,
    const rclcpp::Time & lidar_stamp,
    const std::chrono::steady_clock::time_point & callback_start,
    const size_t point_count,
    const size_t pointcloud_bytes,
    const hesai_lidar::msg::PandarScan::SharedPtr & scan,
    const bool published_points,
    const bool published_packets)
  {
    const double publish_sec = publish_time.seconds();
    const double cloud_stamp_age_sec = (publish_time - lidar_stamp).seconds();
    double oldest_packet_stamp_sec = std::numeric_limits<double>::infinity();
    double newest_packet_stamp_sec = -std::numeric_limits<double>::infinity();
    size_t valid_packet_count = 0;

    for (const auto & packet : scan->packets) {
      const double packet_stamp_sec = timeMsgToUnixSeconds(packet.stamp);
      if (packet_stamp_sec <= 0.0 || !std::isfinite(packet_stamp_sec)) {
        continue;
      }
      oldest_packet_stamp_sec = std::min(oldest_packet_stamp_sec, packet_stamp_sec);
      newest_packet_stamp_sec = std::max(newest_packet_stamp_sec, packet_stamp_sec);
      ++valid_packet_count;
    }

    double oldest_packet_age_sec = -1.0;
    double newest_packet_age_sec = -1.0;
    double packet_span_sec = -1.0;
    if (valid_packet_count > 0) {
      oldest_packet_age_sec = publish_sec - oldest_packet_stamp_sec;
      newest_packet_age_sec = publish_sec - newest_packet_stamp_sec;
      packet_span_sec = newest_packet_stamp_sec - oldest_packet_stamp_sec;
    }

    const auto callback_end = std::chrono::steady_clock::now();
    const double callback_ms =
      std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
    const int throttle_ms =
      static_cast<int>(std::max(0.1, diagnostics_log_period_sec_) * 1000.0);

    if (oldest_packet_age_sec >= diagnostics_warn_packet_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), throttle_ms,
        "hesai pipeline lag: cloud_stamp_age=%.3fs oldest_packet_age=%.3fs "
        "newest_packet_age=%.3fs packet_span=%.3fs packets=%zu valid_packets=%zu "
        "points=%zu pointcloud_bytes=%zu callback=%.2fms published_points=%d "
        "published_packets=%d",
        cloud_stamp_age_sec, oldest_packet_age_sec, newest_packet_age_sec, packet_span_sec,
        scan->packets.size(), valid_packet_count, point_count, pointcloud_bytes, callback_ms,
        published_points, published_packets);
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), throttle_ms,
        "hesai pipeline: cloud_stamp_age=%.3fs oldest_packet_age=%.3fs "
        "newest_packet_age=%.3fs packet_span=%.3fs packets=%zu valid_packets=%zu "
        "points=%zu pointcloud_bytes=%zu callback=%.2fms published_points=%d "
        "published_packets=%d",
        cloud_stamp_age_sec, oldest_packet_age_sec, newest_packet_age_sec, packet_span_sec,
        scan->packets.size(), valid_packet_count, point_count, pointcloud_bytes, callback_ms,
        published_points, published_packets);
    }
  }
};
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hesai_lidar::HesaiLidarClient>());
  rclcpp::shutdown();
  return 0;
}
#include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(HesaiLidarClient)
