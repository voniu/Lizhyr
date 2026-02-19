#include "restorer_node.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr uint32_t kMagic = 0x4C444152;  // "LDAR"
constexpr size_t kHeaderSize = 16;
constexpr size_t kPayloadFixedBytes = sizeof(uint32_t) * 2;  // cloud_count + point_count
}

RestorerNode::RestorerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("restorer_node", options),
  server_port_(8080),
  target_frame_id_("map"),
  running_(true),
  server_fd_(-1)
{
  server_port_ = declare_parameter<int>("server_port", server_port_);
  target_frame_id_ = declare_parameter<std::string>("target_frame_id", target_frame_id_);

  server_thread_ = std::thread(&RestorerNode::serverThreadMain, this);
}

RestorerNode::~RestorerNode()
{
  running_.store(false);

  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }

  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

void RestorerNode::serverThreadMain()
{
  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Server socket creation failed: %s", std::strerror(errno));
    return;
  }

  int opt = 1;
  if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    RCLCPP_WARN(get_logger(), "setsockopt(SO_REUSEADDR) failed: %s", std::strerror(errno));
  }

  sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(static_cast<uint16_t>(server_port_));

  if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "Bind failed on port %d: %s", server_port_, std::strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  if (::listen(server_fd_, 4) < 0) {
    RCLCPP_ERROR(get_logger(), "Listen failed: %s", std::strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  RCLCPP_INFO(get_logger(), "Restorer server listening on port %d", server_port_);

  while (running_.load()) {
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
    if (client_fd < 0) {
      if (!running_.load()) {
        break;
      }
      RCLCPP_WARN(get_logger(), "Accept failed: %s", std::strerror(errno));
      continue;
    }

    RCLCPP_INFO(get_logger(), "Client connected");
    handleClient(client_fd);
    ::close(client_fd);
    RCLCPP_INFO(get_logger(), "Client disconnected");
  }
}

bool RestorerNode::readExact(int fd, uint8_t * buffer, size_t size) const
{
  size_t total = 0;
  while (total < size) {
    ssize_t read_bytes = ::recv(fd, buffer + total, size - total, 0);
    if (read_bytes <= 0) {
      return false;
    }
    total += static_cast<size_t>(read_bytes);
  }
  return true;
}

bool RestorerNode::handleClient(int client_fd)
{
  while (running_.load()) {
    uint8_t header_bytes[kHeaderSize];
    if (!readExact(client_fd, header_bytes, kHeaderSize)) {
      return false;
    }

    Header header;
    // Byte-swap from network order to host order for header fields.
    std::memcpy(&header.magic, header_bytes, sizeof(uint32_t));
    std::memcpy(&header.payload_size, header_bytes + 4, sizeof(uint32_t));
    std::memcpy(&header.lidar_id, header_bytes + 8, sizeof(uint32_t));
    std::memcpy(&header.sequence, header_bytes + 12, sizeof(uint32_t));

    header.magic = ntohl(header.magic);
    header.payload_size = ntohl(header.payload_size);
    header.lidar_id = ntohl(header.lidar_id);
    header.sequence = ntohl(header.sequence);

    if (header.magic != kMagic) {
      RCLCPP_WARN(get_logger(), "Invalid magic: 0x%08x", header.magic);
      return false;
    }

    if (header.payload_size < kPayloadFixedBytes) {
      RCLCPP_WARN(get_logger(), "Invalid payload size: %u", header.payload_size);
      return false;
    }

    std::vector<uint8_t> payload(header.payload_size);
    if (!readExact(client_fd, payload.data(), payload.size())) {
      return false;
    }

    publishClouds(header.lidar_id, header.sequence, payload);
  }

  return true;
}

void RestorerNode::publishClouds(
  uint32_t lidar_id, uint32_t sequence, const std::vector<uint8_t> & payload)
{
  const uint8_t * ptr = payload.data();
  const uint8_t * end = payload.data() + payload.size();

  uint32_t cloud_count = 0;
  uint32_t point_count = 0;
  std::memcpy(&cloud_count, ptr, sizeof(uint32_t));
  ptr += sizeof(uint32_t);
  std::memcpy(&point_count, ptr, sizeof(uint32_t));
  ptr += sizeof(uint32_t);

  const size_t points_per_cloud = static_cast<size_t>(point_count);
  const size_t bytes_per_cloud = points_per_cloud * sizeof(float) * 3;
  const size_t expected_payload = kPayloadFixedBytes + bytes_per_cloud * cloud_count;

  if (payload.size() < expected_payload || ptr > end) {
    RCLCPP_WARN(get_logger(), "Payload size mismatch (lidar_id=%u)", lidar_id);
    return;
  }

  auto publisher = getPublisher(lidar_id);

  {
    std::lock_guard<std::mutex> lock(pub_mutex_);
    auto it = last_sequence_.find(lidar_id);
    if (it != last_sequence_.end() && sequence != it->second + 1) {
      RCLCPP_WARN(get_logger(), "Sequence jump for lidar_id=%u: %u -> %u",
        lidar_id, it->second, sequence);
    }
    last_sequence_[lidar_id] = sequence;
  }

  RCLCPP_INFO(get_logger(), "lidar_id=%u sequence=%u", lidar_id, sequence);

  for (uint32_t cloud_idx = 0; cloud_idx < cloud_count; ++cloud_idx) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = target_frame_id_;
    msg.height = 1;
    msg.width = point_count;
    msg.is_bigendian = false;
    msg.is_dense = true;

    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step * msg.height);

    // Copy contiguous x,y,z float32 payload directly into the PointCloud2 data.
    std::memcpy(msg.data.data(), ptr, bytes_per_cloud);
    ptr += bytes_per_cloud;

    publisher->publish(msg);
  }
}

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr RestorerNode::getPublisher(
  uint32_t lidar_id)
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  auto it = publishers_.find(lidar_id);
  if (it != publishers_.end()) {
    return it->second;
  }

  const std::string topic = "/Lizhyr/points_" + std::to_string(lidar_id);
  auto publisher = create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());
  publishers_.emplace(lidar_id, publisher);
  return publisher;
}
