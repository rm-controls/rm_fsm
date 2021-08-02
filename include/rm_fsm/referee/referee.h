//
// Created by peter on 2021/5/17.
//
#ifndef RM_FSM_REFEREE_H_
#define RM_FSM_REFEREE_H_

#include <cstdint>
#include <serial/serial.h>
#include <ros/ros.h>

#include <rm_common/referee/data.h>
#include <rm_msgs/Referee.h>
#include <rm_msgs/SuperCapacitor.h>

namespace rm_fsm {
class Referee {
 public:
  Referee() : last_get_(ros::Time::now()), serial_port_("/dev/usbReferee") {
    referee_data_.robot_hurt_.hurt_type_ = 0x09;
    referee_data_.bullet_remaining_.bullet_remaining_num_17_mm_ = 500;
  };
  void init();
  void read();
  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);

  ros::Publisher referee_pub_;
  rm_common::RefereeData referee_data_{};
 private:
  int unpack(uint8_t *rx_data);
  void pack(uint8_t *tx_buffer, uint8_t *data, int cmd_id, int len) const;
  void getRobotInfo();
  void publishData();

  serial::Serial serial_;
  ros::Time last_get_;
  rm_msgs::Referee referee_pub_data_;
  const std::string serial_port_;
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 256;
  uint8_t unpack_buffer_[256]{};
};

// CRC verification
uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char uc_crc_8);
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
void appendCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
uint16_t getCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc);
uint32_t verifyCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length);
void appendCRC16CheckSum(unsigned char *pch_message, unsigned int dw_length);
}

#endif //RM_FSM_REFEREE_H_