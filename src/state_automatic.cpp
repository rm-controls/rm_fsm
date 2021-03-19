//
// Created by astro on 2021/1/26.
//

#include "rm_fsm/state_automatic.h"
template<typename T>
StateAutomatic<T>::StateAutomatic(FsmData<T> *fsm_data,
                                  const std::string &state_string,
                                  ros::NodeHandle &nh):State<T>(fsm_data, state_string, nh) {
  this->tf_listener_ = new tf2_ros::TransformListener(this->tf_);
  point_side_ = 1;
  gimbal_position_ = 1;
  calibration_ = 0;
  speed_ = 0;
  last_position_ = 0;
  current_position_ = 0;
  auto_move_chassis_speed_ = getParam(nh, "auto_move/chassis_speed", 1.0);
  auto_move_chassis_accel_ = getParam(nh, "auto_move/chassis_accel", 1.0);
  auto_move_pitch_speed_ = getParam(nh, "auto_move/pitch_speed", 0.5);
  auto_move_yaw_speed_ = getParam(nh, "auto_move/yaw_speed", 3.14);
  start_ = getParam(nh, "auto_move/start", 0.3);
  end_ = getParam(nh, "auto_move/end", 1.5);

  map2odom_.header.stamp = ros::Time::now();
  map2odom_.header.frame_id = "map";
  map2odom_.child_frame_id = "odom";
  map2odom_.transform.translation.x = 0;
  map2odom_.transform.translation.y = 0;
  map2odom_.transform.translation.z = 0;
  map2odom_.transform.rotation.w = 1;
  tf_broadcaster_.sendTransform(map2odom_);
}

template<typename T>
void StateAutomatic<T>::onEnter() {
  ROS_INFO("Enter automatic mode");
}

template<typename T>
void StateAutomatic<T>::run() {
  geometry_msgs::TransformStamped gimbal_transformStamped;
  geometry_msgs::TransformStamped chassis_transformStamped;
  static int time_counter = 0;
  double roll{}, pitch{}, yaw{};
  ros::Time now = ros::Time::now();

  try{
    gimbal_transformStamped = this->tf_.lookupTransform("odom", "link_pitch",ros::Time(0));
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

  current_position_ = chassis_transformStamped.transform.translation.x;
  speed_ = (current_position_- last_position_)*100;
  last_position_ = current_position_;
  if(calibration_) {
    // set shooter
    this->setShoot(rm_msgs::ShootCmd::PASSIVE, rm_msgs::ShootCmd::SPEED_10M_PER_SECOND, this->shoot_hz_, now);

    // set chassis
    if ((current_position_ >= end_) && (point_side_ == 1))
      point_side_ = 2;
    else if ((current_position_ <= start_) && (point_side_ == 3))
      point_side_ = 1;

    if (point_side_ == 1) {
      this->accel_x_ = auto_move_chassis_accel_;
      this->setChassis(rm_msgs::ChassisCmd::RAW, auto_move_chassis_speed_, 0.0, 0.0);
    } else if (point_side_ == 2) {
      this->setChassis(rm_msgs::ChassisCmd::PASSIVE, 0.0, 0.0, 0.0);
      if(speed_ <= 0)
        point_side_ = 3;
    } else if (point_side_ == 3) {
      this->accel_x_ = auto_move_chassis_accel_;
      this->setChassis(rm_msgs::ChassisCmd::RAW, -auto_move_chassis_speed_, 0.0, 0.0);
    } else if (point_side_ == 4) {
      this->setChassis(rm_msgs::ChassisCmd::PASSIVE, 0.0, 0.0, 0.0);
      if (speed_ >= 0)
        point_side_ = 1;
    }

    // set gimbal
    if (pitch > (0.9))
      gimbal_position_ = 1;
    else if (pitch < (-0.1))
      gimbal_position_ = 2;

    if (gimbal_position_ == 1) {
      this->setGimbal(rm_msgs::GimbalCmd::RATE, auto_move_yaw_speed_, -auto_move_pitch_speed_, 0);
    } else if (gimbal_position_ == 2) {
      this->setGimbal(rm_msgs::GimbalCmd::RATE, auto_move_yaw_speed_, auto_move_pitch_speed_, 0);
    } else if (point_side_ == 3) {
      this->setGimbal(rm_msgs::GimbalCmd::PASSIVE, 0.0, 0.0, 0);
    }

  } else {
    this->setChassis(rm_msgs::ChassisCmd::RAW, -0.2, 0, 0);
    this->setGimbal(rm_msgs::GimbalCmd::PASSIVE, 0, 0, 0);
    time_counter++;
    if (time_counter > 35) {
      if ((speed_ > -0.04) && (speed_ < 0.04)) {
        calibration_ = 1;
        map2odom_.header.stamp = ros::Time::now();
        map2odom_.header.frame_id = "map";
        map2odom_.child_frame_id = "odom";
        map2odom_.transform.translation.x = current_position_;
        map2odom_.transform.translation.y = 0;
        map2odom_.transform.translation.z = 0;
        map2odom_.transform.rotation.w = 1;
        tf_broadcaster_.sendTransform(map2odom_);

        odom2baselink_.header.stamp = ros::Time::now();
        odom2baselink_.header.frame_id = "odom";
        odom2baselink_.child_frame_id = "base_link";
        odom2baselink_.transform.translation.x = 0;
        odom2baselink_.transform.translation.y = 0;
        odom2baselink_.transform.translation.z = 0;
        odom2baselink_.transform.rotation.w = 1;
        br.sendTransform(odom2baselink_);
      }
      time_counter = 0;
    }
  }
}

template<typename T>
void StateAutomatic<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("Exit automatic mode");
}

template
class StateAutomatic<double>;
template
class StateAutomatic<float>;