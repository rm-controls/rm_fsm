//
// Created by luohx on 20-2-19.
//
#ifndef SRC_RM_BRIDGE_INCLUDE_RT_RT_REFEREE_H_
#define SRC_RM_BRIDGE_INCLUDE_RT_RT_REFEREE_H_
//Referee System
#include <cstdint>
#include <serial/serial.h>
#include <rm_msgs/Referee.h>
#include "rm_fsm/protocol.h"

struct RefereeData {
  GameStatus game_status_;
  GameResult game_result_;
  GameRobotHp game_robot_hp_;
  DartStatus dart_status_;
  IcraBuffDebuffZoneStatus icra_buff_debuff_zone_status;
  EventData event_data_;
  SupplyProjectileAction supply_projectile_action_;
  RefereeWarning referee_warning_;
  DartRemainingTime dart_remaining_time_;
  GameRobotStatus game_robot_status_;
  PowerHeatData power_heat_data_;
  GameRobotPos game_robot_pos_;
  Buff buff_;
  AerialRobotEnergy aerial_robot_energy_;
  RobotHurt robot_hurt_;
  ShootData shoot_data_;
  BulletRemaining bullet_remaining_;
  RfidStatus rfid_status_;
  DartClientCmd dart_client_cmd_;
  StudentInteractiveData student_interactive_data_;
  GraphicDataStruct graphic_data_struct_;
  RobotInteractiveData robot_interactive_data_;
  RobotCommand robot_command_;
  int performance_system_; // Performance level system
};

class PowerManagerData {
 public:
  PowerManagerData() = default;
  ~PowerManagerData() = default;
  float parameters[4] = {0};
  void read(const std::vector<uint8_t> &rx_buffer);

 private:
  void DTP_Received_CallBack(unsigned char Receive_Byte);
  void Receive_CallBack(unsigned char PID, unsigned char Data[8]);

  unsigned char Receive_Buffer[128] = {0};
  unsigned char PingPong_Buffer[128] = {0};
  unsigned int Receive_BufCounter = 0;
  float Int16ToFloat(unsigned short data0);
};

class Referee {
 public:
  Referee() = default;
  ~Referee() = default;
  void init();
  void read();
  void drawGraphic(int robot_id, int client_id, int side, GraphicOperateType operate_type);
  void drawCharacter(int robot_id, int client_id, int side, GraphicOperateType operate_type, std::string data);
  void drawFloat(int robot_id, int client_id, float data, GraphicOperateType operate_type);
  void sendInteractiveData(int data_cmd_id, int sender_id, int receiver_id, const std::vector<uint8_t> &data);

  RefereeData referee_data_{};
  PowerManagerData power_manager_data_;
  bool flag = false;
  ros::Publisher referee_pub_;
  rm_msgs::Referee referee_pub_data_;

 private:
  void unpack(const std::vector<uint8_t> &rx_buffer);
  void getData(uint8_t *frame);

  serial::Serial serial_;
  std::vector<uint8_t> rx_data_;
  UnpackData referee_unpack_obj{};

  const int kUnpackLength = 160;
  const int kProtocolFrameLength = 128, kProtocolHeaderLength = sizeof(FrameHeaderStruct),
      kProtocolCmdIdLength = 2, kProtocolTailLength = 2;

  int data_len_ = 0;
  bool use_power_manager_ = false;
};

uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8);
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
void appendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
void appendCRC16CheckSum(unsigned char *pchMessage, unsigned int dwLength);

#endif //SRC_RM_BRIDGE_INCLUDE_RT_RT_REFEREE_H_