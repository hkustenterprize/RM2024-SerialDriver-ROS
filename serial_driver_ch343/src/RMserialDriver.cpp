#include "serial_driver_ch343/RMserialDriver.hpp"
#define DEBUG_SERIAL_DRIVER 0
namespace rm_serial_driver
{
SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("read_ch343", options),
  config(std::make_shared<SerialConfig>(2000000, 8, false, StopBit::TWO, Parity::NONE)),
  port(std::make_shared<Port>(config))
{
  RCLCPP_WARN(get_logger(), "Begin the Node read_ch343 !");

  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  gimbal_command_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCommand>(
    "/serial_driver/gimbal_command", rclcpp::SensorDataQoS(),
    std::bind(&SerialDriverNode::GimbalCommand_CB, this, std::placeholders::_1));

  cvmode_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::Cvmode>("/serial_driver/cv_mode", 10);
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/serial_driver/debug_latency", 10);

  while (true) {
    if (port->isPortOpen())
      break;
    else
      port->openPort();
  }

  timer_transmit = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&SerialDriverNode::transmit, this));
  rx_thread = std::thread(&SerialDriverNode::rx, this);
}
//rx
void SerialDriverNode::rx()
{
  while (true) {
    receive();
    pkgState = PkgState::COMPLETE;
    while (receive_buffer.size() > 0 && pkgState != PkgState::HEADER_INCOMPLETE &&
           pkgState != PkgState::PAYLOAD_INCOMPLETE) {
      pkgState = decode();
    }
  }
}

PkgState SerialDriverNode::decode()
{
  int size = receive_buffer.size();

  for (int i = 0; i < size; i++) {
    if (receive_buffer[i] == 0xAA) {
      if (i + int(sizeof(Header)) > size) {
        return PkgState::HEADER_INCOMPLETE;
      }

      std::copy(
        receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header), decodeBuffer);
      crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

      if (!crc_ok_header) {
        RCLCPP_ERROR(get_logger(), "CRC error in header !");
        error_sum_header++;
        try {
          receive_buffer.erase(
            receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header));
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
        }

        return PkgState::CRC_HEADER_ERRROR;
      }

      this->header = arrayToStruct<Header>(decodeBuffer);

      // pkg length = payload(dataLen) + header len (include header crc) + 2crc
      if (i + int(header.dataLen) + int(sizeof(Header)) + 2 > size) {
        return PkgState::PAYLOAD_INCOMPLETE;
      }

      std::copy(
        receive_buffer.begin() + i,
        receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
      crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer, header.dataLen + sizeof(Header) + 2);

      if (!crc_ok) {
        error_sum_payload++;
        //payload error
        try {
          //check if there is a coming pkg
          for (int j = i + 1; j < int(header.dataLen) + int(sizeof(Header)) + 2 + i; j++) {
            if (receive_buffer[j] == 0xAA) {
              if (j + sizeof(Header) > header.dataLen + sizeof(Header) + 2 + i) {
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + j);
                return PkgState::HEADER_INCOMPLETE;
              }

              std::copy(
                receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header),
                decodeBuffer);
              crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

              if (!crc_ok_header) {
                receive_buffer.erase(
                  receive_buffer.begin(), receive_buffer.begin() + j + sizeof(Header));
                j += sizeof(Header) - 1;
                continue;
              }

              this->header = arrayToStruct<Header>(decodeBuffer);

              if (j + sizeof(Header) + header.dataLen + 2) {
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + j);
                return PkgState::PAYLOAD_INCOMPLETE;
              }

              std::copy(
                receive_buffer.begin() + i,
                receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
              crc_ok =
                crc16::Verify_CRC16_Check_Sum(decodeBuffer, header.dataLen + sizeof(Header) + 2);

              if (!crc_ok) {
                RCLCPP_ERROR(get_logger(), "CRC error in payload !");
                receive_buffer.erase(
                  receive_buffer.begin(),
                  receive_buffer.begin() + j + sizeof(Header) + header.dataLen + 2);
                j += sizeof(Header) + header.dataLen + 2 - 1;
                continue;
              }
            }
          }
          receive_buffer.erase(
            receive_buffer.begin(),
            receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
        }

        return PkgState::CRC_PKG_ERROR;
      }

      //complete
      try {
        receive_buffer.erase(
          receive_buffer.begin(), receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
      }

      pkg_sum++;

      std::chrono::high_resolution_clock::time_point end =
        std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> time = end - start;

#if DEBUG_SERIAL_DRIVER
      RCLCPP_INFO(
        get_logger(),
        "crc error rate : %f pkg sum rate: %f hz classify_pkg_sum rate:%f hz,read_sum rate: %f "
        "transmit hz :%f  time : %f \n",
        float(error_sum_header + error_sum_payload) / float(pkg_sum),
        double(pkg_sum) / time.count(), double(classify_pkg_sum) / time.count(),
        float(read_sum) * 11 / time.count(), trans_pkg_sum / time.count(), time.count());
#endif

      classify(decodeBuffer);
      return PkgState::COMPLETE;
    }
  }
  receive_buffer.erase(receive_buffer.begin(), receive_buffer.end());
  return PkgState::OTHER;
}

void SerialDriverNode::classify(uint8_t * data)
{
  // Header header = arrayToStruct<Header>(data);
  GimbalMsg packet = arrayToStruct<GimbalMsg>(data);
  if (packet.header.protocolID != GIMBAL_MSG) return;


  try {
    classify_pkg_sum++;

    auto_aim_interfaces::msg::Cvmode cvmode_msg;

    if (packet.target_color == 2) {
      resetArmorTracker();
    } else if (packet.target_color == 1) {
      setParam(rclcpp::Parameter("detect_color", 1));
      cvmode_msg.bullet_speed = 1;
    } else if (packet.target_color == 0) {
      setParam(rclcpp::Parameter("detect_color", 0));
      cvmode_msg.bullet_speed = 0;
    }

    // cvmode_msg.bullet_speed = packet.bullet_speed;
    cvmode_msg.cur_cv_mode = packet.cur_cv_mode;
    cvmode_pub_->publish(cvmode_msg);

    geometry_msgs::msg::TransformStamped t;
    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
    t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
    t.header.frame_id = "odom";
    t.child_frame_id = "gimbal_link";
    tf2::Quaternion q(packet.q_x, packet.q_y, packet.q_z, packet.q_w);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
    port->closePort();
    port->openPort();
  }
}

int SerialDriverNode::receive()
{
  int read_num = 0;

  read_num = read(port->fd, receiveBuffer, 64);
  read_sum += read_num;

  if (read_num > 0) {
    receive_buffer.insert(receive_buffer.end(), receiveBuffer, receiveBuffer + read_num);
  } else {
    RCLCPP_ERROR(get_logger(), "Can not read from the ch343 !");
    port->closePort();
    port->openPort();
  }
  return read_num;
}

//tx
void SerialDriverNode::GimbalCommand_CB(auto_aim_interfaces::msg::GimbalCommand::SharedPtr msg)
{
  try {
    trans_pkg_sum++;
    uint8_t buffer[sizeof(GimbalCommand)];
    GimbalCommand gimbal_command_msg;

    gimbal_command_msg.header.dataLen = sizeof(GimbalCommand) - sizeof(Header) - 2;
    gimbal_command_msg.header.protocolID = GIMBAL_CMD;
    gimbal_command_msg.target_pitch = msg->gimbal_command.x;
    gimbal_command_msg.target_yaw = msg->gimbal_command.y;
    gimbal_command_msg.shoot_mode = msg->gimbal_command.z;

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&gimbal_command_msg), sizeof(Header));
    crc16::Append_CRC16_Check_Sum(
      reinterpret_cast<uint8_t *>(&gimbal_command_msg), sizeof(GimbalCommand));

    structToArray(gimbal_command_msg, buffer);
    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(), buffer, buffer + sizeof(GimbalCommand));

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds();
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "s");
    latency_pub_->publish(latency);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    port->closePort();
    port->openPort();
  }
}

int SerialDriverNode::transmit()
{
  uint8_t buffer[TRANSMIT_BUFFER];
  long size = transmit_buffer.size();
  if (size) {
    while (size > 2 * TRANSMIT_BUFFER && transmit_buffer.size() > 0) {
      size -= TRANSMIT_BUFFER;
      std::lock_guard<std::mutex> lockf(transmit_mutex);
      std::copy(transmit_buffer.begin(), transmit_buffer.begin() + TRANSMIT_BUFFER, buffer);
      transmit_buffer.erase(transmit_buffer.begin(), transmit_buffer.begin() + TRANSMIT_BUFFER);
      write_num = write(port->fd, buffer, TRANSMIT_BUFFER);

      if (write_num < 0) {
        RCLCPP_ERROR(get_logger(), "Can not transmit");
        port->closePort();
        port->openPort();
      }
    }
  }
  return write_num;
}

void SerialDriverNode::resetArmorTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void SerialDriverNode::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    // RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    // RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        // RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}
}  // namespace rm_serial_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SerialDriverNode)