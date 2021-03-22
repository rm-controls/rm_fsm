//
// Created by luohx on 20-2-19.
//
#include <ros/ros.h>
#include "rm_fsm/referee.h"

namespace referee {
void Referee::init() {
  serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
  int count = 0;

  try {
    serial_.setPort("/dev/usbReferee");
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
  } catch (std::exception &e) {
    ROS_ERROR("Cannot set serial port of referee system, check whether the serial library is installed.");
    return;
  }

  while (!serial_.isOpen()) {
    try {
      serial_.open();
      this->flag = true;
    } catch (serial::IOException &e) {
      ROS_WARN("Referee system serial cannot open [%s]", e.what());
    }
    ros::Duration(0.5).sleep();
    if (count++ >= 1) {
      break;
    }
  }
  if (this->flag) {
    ROS_INFO("Referee serial open successfully.\n");
    referee_unpack_obj.index = 0;
    referee_unpack_obj.unpack_step = kStepHeaderSof;
  } else {
    ROS_INFO("Run fsm without referee system");
  }
}

void Referee::read() {
  if (serial_.waitReadable()) {
    std::vector<uint8_t> rx_buffer;
    try {
      serial_.read(rx_buffer, serial_.available());
    } catch (serial::IOException &e) {
      ROS_ERROR("Referee system disconnect.");
      ROS_WARN("Run robot without power limit and heat limit.");
      this->flag = false;
      return;
    }

    rx_len_ = rx_buffer.size();
    unpack(rx_buffer);
  }

  referee_pub_data_.chassis_volt = referee_data_.power_heat_data_.chassis_volt;
  referee_pub_data_.chassis_current = referee_data_.power_heat_data_.chassis_current;
  referee_pub_data_.chassis_power = referee_data_.power_heat_data_.chassis_power;
  referee_pub_data_.chassis_power_buffer = referee_data_.power_heat_data_.chassis_power_buffer;
  referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_heat0;
  referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_heat0_cooling_limit;
  referee_pub_data_.robot_hp = referee_data_.game_robot_status_.remain_HP;

  referee_pub_.publish(referee_pub_data_);
}

void Referee::unpack(const std::vector<uint8_t> &rx_buffer) {
  int num = 0;
  uint8_t byte = 0;
  while (rx_len_) {
    byte = rx_buffer[num];

    switch (referee_unpack_obj.unpack_step) {
      case kStepHeaderSof: {
        if (byte == 0xA5) {
          referee_unpack_obj.unpack_step = kStepLengthLow;
          referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        } else {
          referee_unpack_obj.index = 0;
        }
      }
        break;
      case kStepLengthLow: {
        referee_unpack_obj.data_len = byte;
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        referee_unpack_obj.unpack_step = kStepLengthHigh;
      }
        break;
      case kStepLengthHigh: {
        referee_unpack_obj.data_len |= (byte << 8);
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        if (referee_unpack_obj.data_len < kProtocolFrameLength - kProtocolCmdIdLength - kProtocolCmdIdLength) {
          referee_unpack_obj.unpack_step = kStepFrameSeq;
        } else {
          referee_unpack_obj.unpack_step = kStepHeaderSof;
          referee_unpack_obj.index = 0;
        }
      }
        break;
      case kStepFrameSeq: {
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        referee_unpack_obj.unpack_step = kStepHeaderCrc8;
      }
        break;
      case kStepHeaderCrc8: {
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        if (referee_unpack_obj.index == (kProtocolHeaderLength - 1)) {
          if (verifyCRC8CheckSum(referee_unpack_obj.protocol_packet, kProtocolHeaderLength)) {
            referee_unpack_obj.unpack_step = kStepDataCrc16;
          } else {
            referee_unpack_obj.unpack_step = kStepHeaderSof;
            referee_unpack_obj.index = 0;
          }
        }
      }
        break;
      case kStepDataCrc16: {
        if (referee_unpack_obj.index
            < (kProtocolHeaderLength + kProtocolTailLength + kProtocolCmdIdLength + referee_unpack_obj.data_len - 1)) {
          referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        }
        if (referee_unpack_obj.index
            >= (kProtocolHeaderLength + kProtocolTailLength + kProtocolCmdIdLength + referee_unpack_obj.data_len - 1)) {
          referee_unpack_obj.unpack_step = kStepHeaderSof;
          referee_unpack_obj.index = 0;
          if (verifyCRC16CheckSum(referee_unpack_obj.protocol_packet,
                                  kProtocolHeaderLength + kProtocolTailLength + kProtocolCmdIdLength
                                      + referee_unpack_obj.data_len)) {
            getData(referee_unpack_obj.protocol_packet);
            memset(referee_unpack_obj.protocol_packet, 0, sizeof(referee_unpack_obj.protocol_packet));
            referee_unpack_obj.unpack_step = kStepHeaderSof;
          }
        }
      }
        break;
      default: {
        referee_unpack_obj.unpack_step = kStepHeaderSof;
        memset(referee_unpack_obj.protocol_packet, 0, sizeof(referee_unpack_obj.protocol_packet));
        referee_unpack_obj.index = 0;
        num = 0;
      }
        break;
    }
    num++;
    rx_len_--;
  }
  referee_unpack_obj.unpack_step = kStepHeaderSof;
  memset(referee_unpack_obj.protocol_packet, 0, sizeof(referee_unpack_obj.protocol_packet));
}

void Referee::getData(uint8_t *frame) {
  uint16_t cmd_id = 0;
  uint8_t index = 0;
  index += (sizeof(FrameHeaderStruct) - 1);
  memcpy(&cmd_id, frame + index, sizeof(uint16_t));

  index += sizeof(uint16_t);

  switch (cmd_id) {
    case kGameStatusCmdId: {
      memcpy(&referee_data_.game_status_, frame + index, sizeof(GameStatus));
      break;
    }
    case kGameResultCmdId: {
      memcpy(&referee_data_.game_result_, frame + index, sizeof(GameResult));
      break;
    }
    case kGameRobotHpCmdId: {
      memcpy(&referee_data_.game_robot_hp_, frame + index, sizeof(GameRobotHp));
      break;
    }
    case kDartStatusCmdId: {
      memcpy(&referee_data_.dart_status_, frame + index, sizeof(DartStatus));
      break;
    }
    case kIcraZoneStatusCmdId: {
      memcpy(&referee_data_.icra_buff_debuff_zone_status, frame + index, sizeof(IcraBuffDebuffZoneStatus));
      break;
    }
    case kFieldEventsCmdId: {
      memcpy(&referee_data_.event_data_, frame + index, sizeof(EventData));
      break;
    }
    case kSupplyProjectileActionCmdId: {
      memcpy(&referee_data_.supply_projectile_action_, frame + index, sizeof(SupplyProjectileAction));
      break;
    }
    case kRefereeWarningCmdId: {
      memcpy(&referee_data_.referee_warning_, frame + index, sizeof(RefereeWarning));
      break;
    }
    case kDartRemainingCmdId: {
      memcpy(&referee_data_.dart_remaining_time_, frame + index, sizeof(DartRemainingTime));
      break;
    }
    case kRobotStatusCmdId: {
      memcpy(&referee_data_.game_robot_status_, frame + index, sizeof(GameRobotStatus));
      break;
    }
    case kPowerHeatDataCmdId: {
      memcpy(&referee_data_.power_heat_data_, frame + index, sizeof(PowerHeatData));
      this->referee_data_.power_heat_data_.chassis_volt =
          referee_data_.power_heat_data_.chassis_volt / 1000;       //mV->V
      this->referee_data_.power_heat_data_.chassis_current =
          referee_data_.power_heat_data_.chassis_current / 1000;    //mA->A
      break;
    }
    case kRobotPosCmdId: {
      memcpy(&referee_data_.game_robot_pos_, frame + index, sizeof(GameRobotPos));
      break;
    }
    case kBuffCmdId: {
      memcpy(&referee_data_.buff_, frame + index, sizeof(Buff));
      break;
    }
    case kAerialRobotEnergyCmdId: {
      memcpy(&referee_data_.aerial_robot_energy_, frame + index, sizeof(AerialRobotEnergy));
      break;
    }
    case kRobotHurtCmdId: {
      memcpy(&referee_data_.robot_hurt_, frame + index, sizeof(RobotHurt));
      break;
    }
    case kShootDataCmdId: {
      memcpy(&referee_data_.shoot_data_, frame + index, sizeof(ShootData));
      break;
    }
    case kBulletRemainingCmdId: {
      memcpy(&referee_data_.bullet_remaining_, frame + index, sizeof(BulletRemaining));
      break;
    }
    case kRobotRfidStatusCmdId: {
      memcpy(&referee_data_.rfid_status_, frame + index, sizeof(RfidStatus));
      break;
    }
    case kDartClientCmdId: {
      memcpy(&referee_data_.dart_client_cmd_, frame + index, sizeof(DartClientCmd));
      break;
    }
    case kRobotCommandCmdId: {
      memcpy(&referee_data_.robot_command_, frame + index, sizeof(RobotCommand));
      break;
    }
    default: {
      ROS_WARN("[Referee]Referee command ID not found.");
      break;
    }
  }
}

void Referee::drawGraphic(RobotId robot_id, ClientId client_id,
                          int side, GraphicOperateType operate_type) {
  uint8_t tx_buffer[128] = {0,};
  DrawClientGraphicData send_data;

  send_data.tx_frame_header_.sof = 0xA5;
  send_data.tx_frame_header_.seq = 0;
  send_data.tx_frame_header_.data_length = sizeof(StudentInteractiveHeaderData) + sizeof(GraphicDataStruct);

  memcpy(tx_buffer, &send_data.tx_frame_header_, sizeof(FrameHeaderStruct));
  appendCRC8CheckSum(tx_buffer, sizeof(FrameHeaderStruct));

  send_data.cmd_id_ = kStudentInteractiveDataCmdId;

  send_data.graphic_header_data_.data_cmd_id = kClientGraphicSingleCmdId;
  send_data.graphic_header_data_.send_ID = robot_id;
  send_data.graphic_header_data_.receiver_ID = client_id;

  if (side) {
    send_data.graphic_data_struct_.graphic_name[0] = 1;
    send_data.graphic_data_struct_.start_x = 100;
    send_data.graphic_data_struct_.start_y = 800;
    send_data.graphic_data_struct_.end_x = 200;
    send_data.graphic_data_struct_.end_y = 900;
  } else {
    send_data.graphic_data_struct_.graphic_name[0] = 0;
    send_data.graphic_data_struct_.start_x = 1720;
    send_data.graphic_data_struct_.start_y = 800;
    send_data.graphic_data_struct_.end_x = 1820; // 11 bit
    send_data.graphic_data_struct_.end_y = 900; // 11 bit
  }

  send_data.graphic_data_struct_.graphic_name[1] = 0;
  send_data.graphic_data_struct_.graphic_name[2] = 0;
  send_data.graphic_data_struct_.operate_type = operate_type;
  send_data.graphic_data_struct_.graphic_type = 1;
  send_data.graphic_data_struct_.layer = 0;
  send_data.graphic_data_struct_.color = 1; // yellow
  send_data.graphic_data_struct_.start_angle = 0; // take the lower 9 bit
  send_data.graphic_data_struct_.end_angle = 0; // 9 bit
  send_data.graphic_data_struct_.width = 10; // 10 bit
  send_data.graphic_data_struct_.radius = 0; // 11 bit


  memcpy(tx_buffer + sizeof(FrameHeaderStruct), (uint8_t *) &send_data.cmd_id_,
         sizeof(send_data.cmd_id_) + sizeof(send_data.graphic_header_data_)
             + sizeof(send_data.graphic_data_struct_));

  appendCRC16CheckSum(tx_buffer, sizeof(send_data));

  serial_.write(tx_buffer, sizeof(send_data));
}

void Referee::drawFloat(RobotId robot_id, ClientId client_id,
                        float data, GraphicOperateType operate_type) {
  uint8_t tx_buffer[128] = {0,};
  DrawClientGraphicData send_data;

  send_data.tx_frame_header_.sof = 0xA5;
  send_data.tx_frame_header_.seq = 0;
  send_data.tx_frame_header_.data_length = sizeof(StudentInteractiveHeaderData) + sizeof(GraphicDataStruct);

  memcpy(tx_buffer, &send_data.tx_frame_header_, sizeof(FrameHeaderStruct));
  appendCRC8CheckSum(tx_buffer, sizeof(FrameHeaderStruct));

  send_data.cmd_id_ = kStudentInteractiveDataCmdId;

  send_data.graphic_header_data_.data_cmd_id = kClientGraphicSingleCmdId;
  send_data.graphic_header_data_.send_ID = robot_id;
  send_data.graphic_header_data_.receiver_ID = client_id;

  send_data.graphic_data_struct_.graphic_name[0] = 0;
  send_data.graphic_data_struct_.graphic_name[1] = 0;
  send_data.graphic_data_struct_.graphic_name[2] = 1;

  send_data.graphic_data_struct_.operate_type = operate_type;
  send_data.graphic_data_struct_.graphic_type = 5;
  send_data.graphic_data_struct_.start_angle = 20;
  send_data.graphic_data_struct_.start_x = 910;
  send_data.graphic_data_struct_.start_y = 300;
  send_data.graphic_data_struct_.layer = 2;
  send_data.graphic_data_struct_.color = 1; // yellow
  send_data.graphic_data_struct_.end_angle = 2; // 9 bit
  send_data.graphic_data_struct_.width = 5; // 10 bit

  memcpy(tx_buffer + sizeof(FrameHeaderStruct), (uint8_t *) &send_data.cmd_id_,
         sizeof(send_data.cmd_id_) + sizeof(send_data.graphic_header_data_)
             + sizeof(send_data.graphic_data_struct_));
  memcpy(tx_buffer + sizeof(send_data) - 6, (uint8_t *) &data,
         sizeof(float));

  appendCRC16CheckSum(tx_buffer, sizeof(send_data));

  serial_.write(tx_buffer, sizeof(send_data));
}

void Referee::drawCharacter(RobotId robot_id, ClientId client_id, int side,
                            GraphicOperateType operate_type, std::string data) {
  uint8_t tx_buffer[128] = {0,};
  DrawClientCharData send_data;

  send_data.tx_frame_header_.sof = 0xA5;
  send_data.tx_frame_header_.seq = 0;
  send_data.tx_frame_header_.data_length =
      sizeof(StudentInteractiveHeaderData) + sizeof(GraphicDataStruct) + sizeof(send_data.data_);

  memcpy(tx_buffer, &send_data.tx_frame_header_, sizeof(FrameHeaderStruct));
  appendCRC8CheckSum(tx_buffer, sizeof(FrameHeaderStruct));

  send_data.cmd_id_ = kStudentInteractiveDataCmdId;

  send_data.graphic_header_data_.data_cmd_id = kClientCharacterCmdId;
  send_data.graphic_header_data_.send_ID = robot_id;
  send_data.graphic_header_data_.receiver_ID = client_id;

  if (side) {
    send_data.graphic_data_struct_.graphic_name[0] = 1;
    send_data.graphic_data_struct_.start_angle = 20;
    send_data.graphic_data_struct_.start_x = 100;
    send_data.graphic_data_struct_.start_y = 800;
  } else {
    send_data.graphic_data_struct_.graphic_name[0] = 0;
    send_data.graphic_data_struct_.start_angle = 20;
    send_data.graphic_data_struct_.start_x = 1720;
    send_data.graphic_data_struct_.start_y = 800;
  }

  send_data.graphic_data_struct_.graphic_name[1] = 1;
  send_data.graphic_data_struct_.graphic_name[2] = 0;
  send_data.graphic_data_struct_.operate_type = operate_type;
  send_data.graphic_data_struct_.graphic_type = 7;
  send_data.graphic_data_struct_.layer = 1;
  send_data.graphic_data_struct_.color = 3; // orange
  send_data.graphic_data_struct_.end_angle = (int) data.size(); // 9 bit
  send_data.graphic_data_struct_.width = 5; // 10 bit

  for (int i = 0; i < (int) data.size() && i < 30; ++i) {
    send_data.data_[i] = data[i];
  }

  memcpy(tx_buffer + sizeof(FrameHeaderStruct), (uint8_t *) &send_data.cmd_id_,
         sizeof(send_data.cmd_id_) + sizeof(send_data.graphic_header_data_)
             + sizeof(send_data.graphic_data_struct_) + sizeof(send_data.data_));

  appendCRC16CheckSum(tx_buffer, sizeof(send_data));

  serial_.write(tx_buffer, sizeof(send_data));
}

}

/******************* CRC Verify *************************/
uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8) {
  unsigned char uc_index;
  while (dw_length--) {
    uc_index = ucCRC8 ^ (*pch_message++);
    ucCRC8 = CRC8_table[uc_index];
  }
  return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char ucExpected = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) {
    return 0;
  }
  ucExpected = getCRC8CheckSum(pch_message, dw_length - 1, CRC8_INIT);
  return (ucExpected == pch_message[dw_length - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void appendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength) {
  unsigned char ucCRC = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) return;
  ucCRC = getCRC8CheckSum((unsigned char *) pchMessage, dwLength - 1, CRC8_INIT);
  pchMessage[dwLength - 1] = ucCRC;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
  uint8_t chData;
  if (pchMessage == nullptr) {
    return 0xFFFF;
  }
  while (dwLength--) {
    chData = *pchMessage++;
    (wCRC) = ((uint16_t) (wCRC) >> 8) ^ wCRC_table[((uint16_t) (wCRC) ^ (uint16_t) (chData)) & 0x00ff];
  }
  return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wExpected = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) {
    return 0;
  }
  wExpected = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wCRC = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) {
    return;
  }
  wCRC = getCRC16CheckSum((uint8_t *) pchMessage, dwLength - 2, CRC16_INIT);
  pchMessage[dwLength - 2] = (uint8_t) (wCRC & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t) ((wCRC >> 8) & 0x00ff);
}
