#ifndef SERIAL_DRIVER_MY_NODE_HPP_
#define SERIAL_DRIVER_MY_NODE_HPP_

#define READER_BUFFER_SIZE 64  // do not change this
#define MAX_BUFFER_SIZE 2048
#define DECODE_BUFFER_SIZE 128
#define TRANSMIT_BUFFER 128

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <visualization_msgs/msg/marker.hpp>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <chrono>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

// serial_driver_ch343 and auto_aim_interfaces
#include "auto_aim_interfaces/msg/cvmode.hpp"
#include "auto_aim_interfaces/msg/gimbal_command.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "serial_driver_ch343/crc.hpp"
#include "serial_driver_ch343/protocol.hpp"
#include "serial_driver_ch343/serialDriver.hpp"

namespace rm_serial_driver
{

class SerialDriverNode : public rclcpp::Node
{
public:
  SerialDriverNode(const rclcpp::NodeOptions & options);

private:
  //communicate with ch343
  void rx();
  int receive();
  int transmit();
  void classify(uint8_t * data);
  PkgState decode();

  //communicate with RV
  void GimbalCommand_CB(auto_aim_interfaces::msg::GimbalCommand::SharedPtr msg);
  void resetArmorTracker();
  void setParam(const rclcpp::Parameter & param);
  std::shared_ptr<SerialConfig> config;
  std::shared_ptr<Port> port;

  std::deque<uint8_t> receive_buffer;
  std::deque<uint8_t> transmit_buffer;
  uint8_t decodeBuffer[DECODE_BUFFER_SIZE];
  uint8_t receiveBuffer[READER_BUFFER_SIZE];
  std::mutex transmit_mutex;
  std::thread tx_thread;
  std::thread rx_thread;

  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

  //publish and subscribe
  rclcpp::Publisher<auto_aim_interfaces::msg::Cvmode>::SharedPtr cvmode_pub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::GimbalCommand>::SharedPtr gimbal_command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

  // Param client to set detect_color
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 1;  // 1 for BLUE, 0 for RED
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Broadcast tf from odom to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_transmit;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
  //protocol
  Header header;
  PkgState pkgState;
  //debug info
  bool crc_ok = false;
  bool crc_ok_header = false;

  int error_sum_payload = 0;
  int error_sum_header = 0;
  int read_sum = 0;
  int write_num = 0;
  int pkg_sum = 0;
  int classify_pkg_sum = 0;
  int trans_pkg_sum = 0;
  int state[5];
};
}  // namespace rm_serial_driver
#endif  // SERIAL_DRIVER_MY_NODE_HPP_