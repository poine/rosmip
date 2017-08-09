#ifndef ROSMIP_CONTROL__ROSMIP_HARDWARE_INTERFACE_H
#define ROSMIP_CONTROL__ROSMIP_HARDWARE_INTERFACE_H

#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <rosmip_control/dsm_interface.h>

#define NB_JOINTS 2

#include "roboticscape.h"


class RosMipHardwareInterface : public hardware_interface::RobotHW
{

 public:
  RosMipHardwareInterface();
  virtual ~RosMipHardwareInterface();
  void write();
  void read();
  bool start();
  bool shutdown();
  void switch_motors_on()  { rc_enable_motors(); }
  void switch_motors_off() { rc_disable_motors(); }

  void DSMCallback(void);
  
 private:

  // Joints
  double joint_position_[NB_JOINTS];
  double joint_velocity_[NB_JOINTS];
  double joint_effort_[NB_JOINTS];
  double joint_effort_command_[NB_JOINTS];
  // IMU
  hardware_interface::ImuSensorHandle::Data imu_data_;
  double imu_orientation_[4]; // quat is in the order of geometry_msg, ie x, y, z, w
  double imu_angular_velocity_[3];
  double imu_linear_acceleration_[3];
  // DSM
  hardware_interface::DsmHandle::Data dsm_data_;
  bool dsm_ok_;
  float turn_stick_, drive_stick_;

  
  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::ImuSensorInterface     imu_sensor_interface_;
  hardware_interface::DsmInterface           dsm_interface_;
  
  rc_imu_data_t rc_imu_data_;

  
};


#endif // ROSMIP_CONTROL__ROSMIP_HARDWARE_INTERFACE_H
// example in here: https://github.com/ROBOTIS-OP/robotis_op_ros_control/blob/master/include/robotis_op_ros_control/robotis_op_hardware_interface.h
