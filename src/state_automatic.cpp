//
// Created by astro on 2021/1/26.
//

#include "rm_fsm/state_automatic.h"
template<typename T>
StateAutomatic<T>::StateAutomatic(FsmData<T> *fsm_data,
                                  const std::string &state_string,
                                  ros::NodeHandle &nh,
                                  bool pc_control):State<T>(fsm_data, state_string, nh, pc_control) {
  this->tf_listener_ = new tf2_ros::TransformListener(this->tf_);
  point_side_ = 1;
  gimbal_position_ = 1;
  auto_move_chassis_speed_ = getParam(this->state_nh_, "auto_move_chassis_speed", 1.0);
  auto_move_chassis_accel_ = getParam(this->state_nh_, "auto_move_chassis_accel", 1.0);
  auto_move_pitch_speed_ = getParam(this->state_nh_, "auto_move_pitch_speed", 0.5);
  auto_move_yaw_speed_ = getParam(this->state_nh_, "auto_move_yaw_speed", 3.14);
  auto_move_distance_ = getParam(this->state_nh_, "auto_move_distance", 3.0);
}

template<typename T>
void StateAutomatic<T>::onEnter() {
  ROS_INFO("[fsm] Enter automatic mode");
}

template<typename T>
void StateAutomatic<T>::run() {
  geometry_msgs::TransformStamped gimbal_transformStamped;
  geometry_msgs::TransformStamped chassis_transformStamped;
  double move_end, move_start;
  double shoot_hz = 0;
  double roll{}, pitch{}, yaw{};

  move_end = auto_move_distance_ - 0.5 * auto_move_chassis_speed_ * auto_move_chassis_speed_ / auto_move_chassis_accel_;
  move_start = 0.5 * auto_move_chassis_speed_ * auto_move_chassis_speed_ / auto_move_chassis_accel_;
  ros::Time now = ros::Time::now();
  this->setShoot(this->data_->shoot_cmd_.PASSIVE, this->data_->shoot_cmd_.SPEED_10M_PER_SECOND, shoot_hz, now);

  try {
    gimbal_transformStamped = this->tf_.lookupTransform("odom", "link_pitch", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    //ROS_ERROR("%s",ex.what());
  }
  quatToRPY(gimbal_transformStamped.transform.rotation, roll, pitch, yaw);

  try {
    chassis_transformStamped = this->tf_.lookupTransform("odom", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    //ROS_WARN("%s", ex.what());
  }
  //chassis control
  if (chassis_transformStamped.transform.translation.x >= move_end - 0.1)
    point_side_ = 2;
  else if (chassis_transformStamped.transform.translation.x <= move_start + 0.1)
    point_side_ = 1;
  if (point_side_ == 1) {
    this->data_->chassis_cmd_.mode = this->data_->chassis_cmd_.RAW;
    this->data_->cmd_vel.linear.x = auto_move_chassis_speed_;
    this->data_->chassis_cmd_.accel.linear.x = auto_move_chassis_accel_;
  } else if (point_side_ == 2) {
    this->data_->chassis_cmd_.mode = this->data_->chassis_cmd_.RAW;
    this->data_->cmd_vel.linear.x = -auto_move_chassis_speed_;
    this->data_->chassis_cmd_.accel.linear.x = auto_move_chassis_accel_;
  }
  this->data_->chassis_cmd_.current_limit = 0.5;
  this->data_->vel_cmd_pub_.publish(this->data_->cmd_vel);
  this->data_->chassis_cmd_pub_.publish(this->data_->chassis_cmd_);
  //gimbal control
  if (pitch > (0.9))
    gimbal_position_ = 1;
  else if (pitch < (-0.1))
    gimbal_position_ = 2;
  if (gimbal_position_ == 1) {
    this->data_->gimbal_cmd_.mode = this->data_->gimbal_cmd_.RATE;
    this->data_->gimbal_cmd_.rate_yaw = auto_move_yaw_speed_;
    this->data_->gimbal_cmd_.rate_pitch = -auto_move_pitch_speed_;
  } else if (gimbal_position_ == 2) {
    this->data_->gimbal_cmd_.mode = this->data_->gimbal_cmd_.RATE;
    this->data_->gimbal_cmd_.rate_yaw = auto_move_yaw_speed_;
    this->data_->gimbal_cmd_.rate_pitch = auto_move_pitch_speed_;
  }
  this->data_->gimbal_cmd_pub_.publish(this->data_->gimbal_cmd_);
  std::cout << "pitch:" << pitch << std::endl;
}

template<typename T>
void StateAutomatic<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("[fsm] Exit automatic mode");
}

template
class StateAutomatic<double>;
template
class StateAutomatic<float>;