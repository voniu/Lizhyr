#include "processor_node.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "pcl_conversions/pcl_conversions.h"

namespace
{
constexpr uint32_t kMagic = 0x4C444152;  // "LDAR"
constexpr size_t kHeaderSize = 16;
}

ProcessorNode::ProcessorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("processor_node", options),
  roi_min_({-30.0, -30.0, -2.0}),
  roi_max_({30.0, 30.0, 10.0}),
  chunk_size_(10),
  server_ip_("192.168.1.100"),
  server_port_(8080),
  ground_filter_enabled_(false),
  ground_z_threshold_(-1.5),
  running_(true),
  sequence_(0)
{
  lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/ouster/points");
  lidar_id_ = declare_parameter<int>("lidar_id", 1);
  const auto roi_min_param = declare_parameter<std::vector<double>>(
    "roi_min", {roi_min_[0], roi_min_[1], roi_min_[2]});
  const auto roi_max_param = declare_parameter<std::vector<double>>(
    "roi_max", {roi_max_[0], roi_max_[1], roi_max_[2]});
  chunk_size_ = declare_parameter<int>("chunk_size", chunk_size_);
  server_ip_ = declare_parameter<std::string>("server_ip", server_ip_);
  server_port_ = declare_parameter<int>("server_port", server_port_);
  ground_filter_enabled_ = declare_parameter<bool>(
    "ground_filter_enabled", ground_filter_enabled_);
  ground_z_threshold_ = declare_parameter<double>(
    "ground_z_threshold", ground_z_threshold_);

  if (roi_min_param.size() == 3 && roi_max_param.size() == 3) {
    roi_min_ = {roi_min_param[0], roi_min_param[1], roi_min_param[2]};
    roi_max_ = {roi_max_param[0], roi_max_param[1], roi_max_param[2]};
  } else {
    RCLCPP_WARN(get_logger(), "Invalid ROI params size, using defaults.");
  }

  if (chunk_size_ <= 0) {
    RCLCPP_WARN(get_logger(), "chunk_size must be > 0, using default 10.");
    chunk_size_ = 10;
  }

  if (lidar_id_ == 0) {
    RCLCPP_WARN(get_logger(), "lidar_id should be > 0, using 1.");
    lidar_id_ = 1;
  }

  current_chunk_.reserve(static_cast<size_t>(chunk_size_));

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ProcessorNode::pointCloudCallback, this, std::placeholders::_1));

  tx_thread_ = std::thread(&ProcessorNode::transmitThreadMain, this);
}

ProcessorNode::~ProcessorNode()
{
  running_.store(false);
  queue_cv_.notify_all();
  if (tx_thread_.joinable()) {
    tx_thread_.join();
  }
}

void ProcessorNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  auto roi_cloud = cropRoi(cloud);
  auto filtered_cloud = filterGround(roi_cloud);

  current_chunk_.push_back(filtered_cloud);
  if (static_cast<int>(current_chunk_.size()) < chunk_size_) {
    return;
  }

  auto payload = serializeChunk(current_chunk_);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    tx_queue_.push_back(std::move(payload));
  }
  queue_cv_.notify_one();
  current_chunk_.clear();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProcessorNode::cropRoi(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  output->points.reserve(input->points.size());

  for (const auto & pt : input->points) {
    // Filter out NaN and infinite values
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    if (pt.x < roi_min_[0] || pt.x > roi_max_[0]) {
      continue;
    }
    if (pt.y < roi_min_[1] || pt.y > roi_max_[1]) {
      continue;
    }
    if (pt.z < roi_min_[2] || pt.z > roi_max_[2]) {
      continue;
    }
    output->points.push_back(pt);
  }

  output->width = static_cast<uint32_t>(output->points.size());
  output->height = 1;
  output->is_dense = true;
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProcessorNode::filterGround(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input) const
{
  if (!ground_filter_enabled_) {
    return pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>(*input));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
  output->points.reserve(input->points.size());

  for (const auto & pt : input->points) {
    if (pt.z < ground_z_threshold_) {
      continue;
    }
    output->points.push_back(pt);
  }

  output->width = static_cast<uint32_t>(output->points.size());
  output->height = 1;
  output->is_dense = true;
  return output;
}

std::vector<char> ProcessorNode::serializeChunk(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & chunk) const
{
  size_t total_points = 0;
  for (const auto & cloud : chunk) {
    total_points += cloud->points.size();
  }

  const size_t header_bytes = sizeof(uint32_t);
  const size_t per_cloud_header = sizeof(uint32_t);
  const size_t per_point_bytes = sizeof(float) * 3;
  const size_t payload_size = header_bytes +
    chunk.size() * per_cloud_header +
    total_points * per_point_bytes;

  std::vector<char> buffer(payload_size);
  char * write_ptr = buffer.data();

  const uint32_t cloud_count = static_cast<uint32_t>(chunk.size());
  std::memcpy(write_ptr, &cloud_count, sizeof(uint32_t));
  write_ptr += sizeof(uint32_t);

  for (const auto & cloud : chunk) {
    const uint32_t point_count = static_cast<uint32_t>(cloud->points.size());
    std::memcpy(write_ptr, &point_count, sizeof(uint32_t));
    write_ptr += sizeof(uint32_t);

    for (const auto & pt : cloud->points) {
      std::memcpy(write_ptr, &pt.x, sizeof(float));
      write_ptr += sizeof(float);
      std::memcpy(write_ptr, &pt.y, sizeof(float));
      write_ptr += sizeof(float);
      std::memcpy(write_ptr, &pt.z, sizeof(float));
      write_ptr += sizeof(float);
    }
  }

  return buffer;
}

void ProcessorNode::transmitThreadMain()
{
  int sock_fd = -1;

  while (running_.load()) {
    std::vector<char> payload;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return !running_.load() || !tx_queue_.empty();
      });
      if (!running_.load()) {
        break;
      }
      payload = tx_queue_.front();
    }

    if (sock_fd < 0) {
      sock_fd = connectSocket();
      if (sock_fd < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
    }

    uint32_t net_magic = htonl(kMagic);
    uint32_t net_size = htonl(static_cast<uint32_t>(payload.size()));
    uint32_t net_lidar_id = htonl(lidar_id_);
    uint32_t net_sequence = htonl(sequence_);
    uint8_t header[kHeaderSize];
    std::memcpy(header, &net_magic, sizeof(uint32_t));
    std::memcpy(header + sizeof(uint32_t), &net_size, sizeof(uint32_t));
    std::memcpy(header + sizeof(uint32_t) * 2, &net_lidar_id, sizeof(uint32_t));
    std::memcpy(header + sizeof(uint32_t) * 3, &net_sequence, sizeof(uint32_t));

    bool ok = sendAll(sock_fd, header, kHeaderSize) &&
      sendAll(sock_fd, reinterpret_cast<const uint8_t *>(payload.data()), payload.size());

    if (!ok) {
      if (sock_fd >= 0) {
        close(sock_fd);
        sock_fd = -1;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!tx_queue_.empty()) {
        tx_queue_.pop_front();
      }
    }

    ++sequence_;
  }

  if (sock_fd >= 0) {
    close(sock_fd);
  }
}

int ProcessorNode::connectSocket() const
{
  int sock_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd < 0) {
    RCLCPP_WARN(get_logger(), "Socket creation failed: %s", std::strerror(errno));
    return -1;
  }

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(server_port_));

  if (::inet_pton(AF_INET, server_ip_.c_str(), &addr.sin_addr) <= 0) {
    RCLCPP_WARN(get_logger(), "Invalid server_ip: %s", server_ip_.c_str());
    ::close(sock_fd);
    return -1;
  }

  if (::connect(sock_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_WARN(get_logger(), "Connect failed: %s", std::strerror(errno));
    ::close(sock_fd);
    return -1;
  }

  return sock_fd;
}

bool ProcessorNode::sendAll(int sock_fd, const uint8_t * data, size_t size) const
{
  size_t total_sent = 0;
  while (total_sent < size) {
    ssize_t sent = ::send(sock_fd, data + total_sent, size - total_sent, 0);
    if (sent <= 0) {
      return false;
    }
    total_sent += static_cast<size_t>(sent);
  }
  return true;
}
