#ifndef LIZHYR_PROCESSOR_NODE_HPP
#define LIZHYR_PROCESSOR_NODE_HPP

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ProcessorNode : public rclcpp::Node
{
public:
  explicit ProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ProcessorNode() override;

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropRoi(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterGround(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input) const;

  std::vector<char> serializeChunk(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & chunk) const;

  void transmitThreadMain();
  int connectSocket() const;
  bool sendAll(int sock_fd, const uint8_t * data, size_t size) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  std::string lidar_topic_;
  uint32_t lidar_id_;
  std::array<double, 3> roi_min_;
  std::array<double, 3> roi_max_;
  int chunk_size_;
  std::string server_ip_;
  int server_port_;
  bool ground_filter_enabled_;
  double ground_z_threshold_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> current_chunk_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::vector<char>> tx_queue_;

  std::atomic<bool> running_;
  uint32_t sequence_;
  std::thread tx_thread_;
};

#endif  // LIZHYR_PROCESSOR_NODE_HPP
