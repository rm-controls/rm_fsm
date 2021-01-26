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
  point_side_ =1;
  gimbal_position_ =1;
  auto_move_chassis_speed_ = getParam(this->state_nh_,"auto_move_chassis_speed",1);
  auto_move_chassis_accel_ = getParam(this->state_nh_,"auto_move_chassis_accel",1);
  auto_move_pitch_speed_ = getParam(this->state_nh_,"auto_move_pitch_speed",1);
  auto_move_yaw_speed_ = getParam(this->state_nh_,"auto_move_yaw_speed",3.14);
  auto_move_distance_ = getParam(this->state_nh_,"auto_move_distance",2.9);
}

template<typename T>
void StateAutomatic<T>::onEnter() {
  ROS_INFO("[fsm] Enter automatic mode");
}

template<typename T>
void StateAutomatic<T>::run() {

  geometry_msgs::TransformStamped chassis_transformStamped;
  double move_end,move_start;
  int shoot_speed = 0;
  double shoot_hz = 0;

  move_end = auto_move_distance_-0.5*auto_move_chassis_speed_*auto_move_chassis_speed_/auto_move_chassis_accel_;
  move_start = 0.5*auto_move_chassis_speed_*auto_move_chassis_speed_/auto_move_chassis_accel_;
  ros::Time now = ros::Time::now();
    if (this->data_->dbus_data_.s_l == this->data_->dbus_data_.MID) this->setShoot(this->data_->shoot_cmd_.PUSH, shoot_speed, shoot_hz, now);
    else this->setShoot(this->data_->shoot_cmd_.READY, shoot_speed, shoot_hz, now);


  //this->setChassis(this->data_->chassis_cmd_.RAW, linear_x, linear_y, angular_z);
  //this->setShoot(this->data_->shoot_cmd_.READY, shoot_speed, shoot_hz, now);

  try {
    chassis_transformStamped = this->tf_.lookupTransform("odom", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    //ROS_WARN("%s", ex.what());
  }
  std::cout << chassis_transformStamped.transform.translation.x <<std::endl;
  //chassis control
  if(chassis_transformStamped.transform.translation.x>=move_end-0.1)
    point_side_=2;
  else if(chassis_transformStamped.transform.translation.x<=move_start+0.1)
    point_side_=1;
  if(point_side_==1){
    this->data_->chassis_cmd_.mode=this->data_->chassis_cmd_.RAW;
    this->data_->cmd_vel.linear.x =auto_move_chassis_speed_;
    this->data_->chassis_cmd_.accel.linear.x = auto_move_chassis_accel_;
  }
  else if(point_side_==2){
    this->data_->chassis_cmd_.mode=this->data_->chassis_cmd_.RAW;
    this->data_->cmd_vel.linear.x =-auto_move_chassis_speed_;
    this->data_->chassis_cmd_.accel.linear.x = auto_move_chassis_accel_;
  }
  this->data_->chassis_cmd_.current_limit = 0.5;
  this->data_->vel_cmd_pub_.publish(this->data_->cmd_vel);
  this->data_->chassis_cmd_pub_.publish(this->data_->chassis_cmd_);
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