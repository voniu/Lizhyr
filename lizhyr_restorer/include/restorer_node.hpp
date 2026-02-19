#ifndef LIZHYR_RESTORER_NODE_HPP
#define LIZHYR_RESTORER_NODE_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class RestorerNode : public rclcpp::Node
{
public:
  explicit RestorerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RestorerNode() override;

private:
  struct Header
  {
    uint32_t magic;
    uint32_t payload_size;
    uint32_t lidar_id;
    uint32_t sequence;
  };

  void serverThreadMain();
  bool readExact(int fd, uint8_t * buffer, size_t size) const;
  bool handleClient(int client_fd);
  void publishClouds(uint32_t lidar_id, uint32_t sequence, const std::vector<uint8_t> & payload);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr getPublisher(uint32_t lidar_id);

  int server_port_;
  std::string target_frame_id_;

  std::atomic<bool> running_;
  int server_fd_;
  std::thread server_thread_;

  std::mutex pub_mutex_;
  std::unordered_map<uint32_t, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers_;
  std::unordered_map<uint32_t, uint32_t> last_sequence_;
};

#endif  // LIZHYR_RESTORER_NODE_HPP
