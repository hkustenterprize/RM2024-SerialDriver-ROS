#include <sys/cdefs.h>

#include <algorithm>
#include <boost/cstdint.hpp>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <variant>
#include <vector>

// Protocol version: v3.1 (RM2024)
// note: CRC operations are defined in CRC.hpp
/*
 * CRC parameters:
 * width = 16, i.e. CRC-16
 * init = 0xFFFF
 * poly = 0x1189
 * final_xor_value = 0 (do nothing)
 */

// UART parameters:
/*
 * baud_rate = 2000000
 * stop bits = 2
 * parity = none
 * hardware_flow_ctrl = no
 * data_size = 8-bit
 */

namespace rm_serial_driver
{
// Ax stands for msg received (msg from mcu to nuc)
// Bx stands for msg sent (msg from nuc to mcu)
#define ID_NUM 8
enum CommunicationType : uint8_t {
  GIMBAL_MSG = 0xA2,
  CHASSIS_MSG = 0xA3,
  SENTRY_GIMBAL_MSG = 0xA7,
  FIELD_MSG = 0xA9,

  GIMBAL_CMD = 0xB2,
  CHASSIS_CMD = 0xB3,
  ACTION_CMD = 0xB4,
  SENTRY_GIMBAL_CMD = 0xB7,
};

// **************************** //
// * protocol for ring buffer * //
// **************************** //
struct Header
{
  uint8_t sof = 0xAAu;
  uint8_t dataLen = 0;
  uint8_t protocolID = 0;
  uint8_t crc_1;
  uint8_t crc_2;

  Header() = default;

} __attribute__((packed));

struct ChassisMsg
{
  Header header;

  float xVel;
  float yVel;
  float wVel;

  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));

struct FieldMsg
{
  Header header;

  uint8_t game_state;
  uint16_t remain_time;
  uint16_t bullet_remain;
  uint8_t team;  // 0 - red 1 - blue

  uint16_t red_hp[8];
  uint16_t blue_hp[8];
  bool in_combat;
  bool bullet_supply_available;
  bool shooter_locked;

  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));

struct GimbalMsg  // TWOCRC_GIMBAL_MSG, also 0xA2
{
  Header header;

  uint8_t cur_cv_mode;
  uint8_t target_color;
  float bullet_speed;
  float q_w;
  float q_x;
  float q_y;
  float q_z;

  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));

struct SentryGimbalMsg  // TWOCRC_GIMBALSTATUS_MSG, also 0xA3
{
  Header header;

  uint8_t cur_cv_mode;
  uint8_t target_color;
  float bullet_speed;

  // small gimbal q
  float small_q_w;
  float small_q_x;
  float small_q_y;
  float small_q_z;

  // main gimbal q
  float big_q_w;
  float big_q_x;
  float big_q_y;
  float big_q_z;

  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));

struct GimbalCommand  // TWOCRC_GIMBAL_CMD, also 0xB1
{
  Header header;

  float target_pitch;
  float target_yaw;
  uint8_t shoot_mode;

  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));

struct ChassisCommand  // TWOCRC_CHASSIS_CMD, also 0xB2
{
  Header header;

  float vel_x;
  float vel_y;
  float vel_w;

  uint8_t crc1;
  uint8_t crc2;

} __attribute__((packed));

struct ActionCommand  // TWOCRC_ACTION_CMD, also 0xB3
{
  Header header;

  bool scan;
  float spin;
  bool cv_enable;

  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));

struct SentryGimbalCommand
{
  Header header;

  float l_target_pitch;
  float l_target_yaw;
  uint8_t l_shoot_mode;

  float r_target_pitch;
  float r_target_yaw;
  uint8_t r_shoot_mode;

  float main_target_pitch;
  float main_target_yaw;

  uint8_t crc_3;
  uint8_t crc_4;

} __attribute__((packed));

using ProtocoType = std::variant<
  GimbalMsg, ChassisMsg, SentryGimbalMsg, FieldMsg, GimbalCommand, ChassisCommand, ActionCommand,
  SentryGimbalCommand>;
std::map<CommunicationType, ProtocoType> protocolMap;

template <typename T>
inline T arrayToStruct(const uint8_t * buffer)
{
  T result;
  std::memcpy(&result, buffer, sizeof(T));
  return result;
}

template <typename T>
void structToArray(const T & inputStruct, uint8_t * outputArray)
{
  std::memcpy(outputArray, reinterpret_cast<const uint8_t *>(&inputStruct), sizeof(T));
}

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T received_packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
  return received_packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> sent_packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    sent_packet.begin());
  return sent_packet;
}
}  // namespace rm_serial_driver
