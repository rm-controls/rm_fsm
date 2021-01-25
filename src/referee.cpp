//
// Created by luohx on 20-2-19.
//
#include <ros/ros.h>
#include <rm_fsm/referee.h>

namespace referee {
void Referee::init() {
  serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
  serial_.setPort("/dev/ttyUSB0");
  serial_.setBaudrate(115200);
  serial_.setTimeout(timeout);
  int count = 0;

  while (!serial_.isOpen()) {
    try {
      serial_.open();
    } catch (serial::IOException &e) {
      ROS_WARN("Referee system serial cannot open [%s]", e.what());
    }
    ros::Duration(0.5).sleep();
    if (count++ >= 1) {
      break;
    }
  }
  if (this->flag) {
    ROS_INFO("serial open successfully.\n");
    this->flag = true;
    referee_unpack_obj.index = 0;
    referee_unpack_obj.unpack_step = kStepHeaderSof;
  } else {
    ROS_INFO("Run fsm without referee system");
  }
}

void Referee::read() {
  if (serial_.waitReadable()) {
    std::vector<uint8_t> rx_buffer;
    serial_.read(rx_buffer, serial_.available());
    rx_len_ = rx_buffer.size();

    unpack(rx_buffer);
  }
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
          if (verify_CRC8_check_sum(referee_unpack_obj.protocol_packet, kProtocolHeaderLength)) {
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
          if (verify_CRC16_check_sum(referee_unpack_obj.protocol_packet,
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
    case kStudentInteractiveDataCmdId: {
      memcpy(&referee_data_.student_interactive_header_data_, frame + index, sizeof(StudentInteractiveHeaderData));

      uint16_t data_cmd_id = 0;
      memcpy(&data_cmd_id, frame + index, sizeof(uint16_t));

      index += sizeof(StudentInteractiveHeaderData);

      if (data_cmd_id >= kRobotInteractiveCmdIdMin && data_cmd_id <= kRobotInteractiveCmdIdMax)
        memcpy(&referee_data_.robot_interactive_data_, frame + index, sizeof(RobotInteractiveData));
      else if (data_cmd_id == kClientGraphicDeleteCmdId)
        memcpy(&referee_data_.client_custom_graphic_delete_,
               frame + index,
               sizeof(ClientCustomGraphicDelete));
      else if (data_cmd_id == kClientGraphicSingleCmdId)
        memcpy(&referee_data_.client_custom_graphic_single_,
               frame + index,
               sizeof(ClientCustomGraphicSingle));
      else if (data_cmd_id == kClientGraphicDoubleCmdId)
        memcpy(&referee_data_.client_custom_graphic_double_,
               frame + index,
               sizeof(ClientCustomGraphicDouble));
      else if (data_cmd_id == kClientGraphicFiveCmdId)
        memcpy(&referee_data_.client_custom_graphic_five_,
               frame + index,
               sizeof(ClientCustomGraphicFive));
      else if (data_cmd_id == kClientGraphicSevenCmdId)
        memcpy(&referee_data_.client_custom_graphic_seven_,
               frame + index,
               sizeof(ClientCustomGraphicSeven));
      else if (data_cmd_id == kClientCharacterCmdId)
        memcpy(&referee_data_.client_custom_character_,
               frame + index,
               sizeof(ClientCustomCharacter));
      else {
        ROS_WARN("[Referee]Interactive data command ID not found.");
      }
    }
      break;
    default: {
      ROS_WARN("[Referee]Referee command ID not found.");
      break;
    }
  }
}
}

uint32_t verify_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char ucExpected = 0;
  if ((pch_message == 0) || (dw_length <= 2)) {
    return 0;
  }
  ucExpected = get_CRC8_check_sum(pch_message, dw_length - 1, CRC8_INIT);
  return (ucExpected == pch_message[dw_length - 1]);
}
uint8_t get_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8) {
  unsigned char uc_index;
  while (dw_length--) {
    uc_index = ucCRC8 ^ (*pch_message++);
    ucCRC8 = CRC8_table[uc_index];
  }
  return (ucCRC8);
}
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wExpected = 0;
  if ((pchMessage == NULL) || (dwLength <= 2)) {
    return 0;
  }
  wExpected = get_CRC16_check_sum(pchMessage, dwLength - 2, 0xffff);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
  //return wExpected;
}
uint16_t get_CRC16_check_sum(uint8_t *pch_message, uint32_t dw_length, uint16_t wCRC) {
  uint8_t chData;
  if (pch_message == NULL) {
    return 0xFFFF;
  }
  while (dw_length--) {
    chData = *pch_message++;
    (wCRC) = ((uint16_t) (wCRC) >> 8) ^ wCRC_table[((uint16_t) (wCRC) ^ (uint16_t) (chData)) & 0x00ff];
  }
  return wCRC;
}
