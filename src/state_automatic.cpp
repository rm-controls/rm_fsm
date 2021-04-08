//
// Created by astro on 2021/1/26.
//

#include "rm_fsm/state_automatic.h"
template<typename T>
StateAutomatic<T>::StateAutomatic(FsmData<T> *fsm_data,
                                  const std::string &state_string,
                                  ros::NodeHandle &nh):State<T>(nh, fsm_data, state_string) {
  this->tf_listener_ = new tf2_ros::TransformListener(this->tf_);
  point_side_ = 1;
  gimbal_position_ = 1;
  calibration_ = 0;
  speed_ = 0;
  current_position_ = 0;
  auto_move_chassis_speed_ = getParam(nh, "auto_move/chassis_speed", 1.0);
  auto_move_accel_x_ = getParam(nh, "control_param/accel_x", 2.0);
  auto_move_pitch_speed_ = getParam(nh, "auto_move/pitch_speed", 0.5);
  auto_move_yaw_speed_ = getParam(nh, "auto_move/yaw_speed", 3.14);
  collision_distance_ = getParam(nh, "auto_move/collision_distance", 0.3);
  start_ = getParam(nh, "auto_move/start", 0.3);
  end_ = getParam(nh, "auto_move/end", 2.5);
  end_ = end_ - 0.275 - 0.275;
  calibration_speed_ = getParam(nh, "auto_move/calibration_speed", 0.15);
  column_ = getParam(nh, "auto_move/column", 1);
  if (column_)
    std::cout << "use column_" << std::endl;
  else
    std::cout << "not use column_" << std::endl;
  map2odom_.header.stamp = ros::Time::now();
  map2odom_.header.frame_id = "map";
  map2odom_.child_frame_id = "odom";
  map2odom_.transform.translation.x = 0;
  map2odom_.transform.translation.y = 0;
  map2odom_.transform.translation.z = 0;
  map2odom_.transform.rotation.w = 1;
  tf_broadcaster_.sendTransform(map2odom_);
  effort_sub_ = nh.subscribe<sensor_msgs::JointState>(
      "/joint_states", 10, &StateAutomatic::effortDataCallback, this);
}

template<typename T>
void StateAutomatic<T>::onEnter() {
  ROS_INFO("Enter automatic mode");
}

template<typename T>
void StateAutomatic<T>::run() {

  geometry_msgs::TransformStamped gimbal_transformStamped;
  geometry_msgs::TransformStamped chassis_transformStamped;
  double now_effort = 0;
  static double sum_effort = 0;
  static int time_counter2 = 0;
  double roll{}, pitch{}, yaw{};
  ros::Time now = ros::Time::now();
  double stop_distance = 0.5 * auto_move_chassis_speed_ * auto_move_chassis_speed_ / auto_move_accel_x_;
  this->loadParam();

  try {
    gimbal_transformStamped = this->tf_.lookupTransform("yaw", "pitch", ros::Time(0));
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
  sum_effort+=effort_data_.effort[0];
  time_counter2++;
 if(time_counter2==10){
    current_position_ = chassis_transformStamped.transform.translation.x;
    speed_ = effort_data_.velocity[0];
    now_effort = sum_effort/10.0;
    time_counter2=0;
    sum_effort = 0;
  }
  if(calibration_) {
    this->data_->shooter_heat_limit_->input(this->data_->referee_, this->expect_shoot_hz_, this->safe_shoot_hz_);
    this->data_->target_cost_function_->input(this->data_->track_data_array_);
    attack_id_ = this->data_->target_cost_function_->output();
    //shooter control
    if (attack_id_ != 0 && this->data_->gimbal_des_error_.error_yaw < this->gimbal_error_limit_
        && this->data_->gimbal_des_error_.error_pitch < this->gimbal_error_limit_) {
      this->setShoot(rm_msgs::ShootCmd::PUSH,
                     30,
                     this->data_->shooter_heat_limit_->output(),
                     now);
    } else if (now - last_time_ > ros::Duration(0.5)) {
      this->setShoot(rm_msgs::ShootCmd::PUSH,
                     30,
                     this->data_->shooter_heat_limit_->output(),
                     now);
    } else {
      this->setShoot(rm_msgs::ShootCmd::READY, 30, 0, now);
    }

    //chassis control
    if (column_) {
      if ((current_position_ >= end_ - collision_distance_) && (point_side_ == 1))
        point_side_ = 2;
      else if ((current_position_ <= start_ + collision_distance_) && (point_side_ == 3))
        point_side_ = 4;
      if (point_side_ == 1) {
        this->setChassis(rm_msgs::ChassisCmd::RAW, auto_move_chassis_speed_, 0.0, 0.0);
      } else if (point_side_ == 2) {
        this->setChassis(rm_msgs::ChassisCmd::PASSIVE, 0.0, 0.0, 0.0);
        if (speed_ <= 0)
          point_side_ = 3;
      } else if (point_side_ == 3) {
        this->setChassis(rm_msgs::ChassisCmd::RAW, -auto_move_chassis_speed_, 0.0, 0.0);
      } else if (point_side_ == 4) {
        this->setChassis(rm_msgs::ChassisCmd::PASSIVE, 0.0, 0.0, 0.0);
        if (speed_ >= 0)
          point_side_ = 1;
      }
    } else {
      if (current_position_ >= end_ - stop_distance - 0.2)
        point_side_ = 2;
      else if (current_position_ <= start_ + stop_distance)
        point_side_ = 1;
      if (point_side_ == 1) {
        this->setChassis(rm_msgs::ChassisCmd::RAW, auto_move_chassis_speed_, 0.0, 0.0);
      } else {
        this->setChassis(rm_msgs::ChassisCmd::RAW, -auto_move_chassis_speed_, 0.0, 0.0);
      }
    }


    //gimbal control
    if (attack_id_ != 0) {
      this->setGimbal(rm_msgs::GimbalCmd::TRACK, 0, 0, attack_id_, 30);
      last_time_ = now;
    } else {
      if (now - last_time_ > ros::Duration(0.5)) {
        if (pitch > 0.750)
          gimbal_position_ = 1;
        else if (pitch < (0.459))
          gimbal_position_ = 2;
        if (gimbal_position_ == 1) {
          this->setGimbal(rm_msgs::GimbalCmd::RATE, auto_move_yaw_speed_, -auto_move_pitch_speed_, 0, 0);
        } else if (gimbal_position_ == 2) {
          this->setGimbal(rm_msgs::GimbalCmd::RATE, auto_move_yaw_speed_, auto_move_pitch_speed_, 0, 0);
        }
      }
    }

  } else {
    this->setChassis(rm_msgs::ChassisCmd::RAW, -calibration_speed_, 0, 0);
    this->setGimbal(rm_msgs::GimbalCmd::PASSIVE, 0, 0, 0, 0);
    if (now - calibration_time_ > ros::Duration(0.4)) {
      if (now_effort < -1.1) {
        std::cout << "calibration finish !" << std::endl;
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
      }
    }
}

template<typename T>
void StateAutomatic<T>::onExit() {
  // Nothing to clean up when exiting
  ROS_INFO("Exit automatic mode");
  calibration_ = 0;
}

template
class StateAutomatic<double>;
template
class StateAutomatic<float>;