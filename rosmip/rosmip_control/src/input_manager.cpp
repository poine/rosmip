#include "rosmip_control/input_manager.h"

// 1m/s for full stick deflection
#define DRIVE_RATE_ADVANCED		1.
// 
#define TURN_RATE_ADVANCED		10

namespace rosmip_controller {

  InputManager::InputManager() {
    
  }

  /*******************************************************************************
   *
   *
   *******************************************************************************/
  bool InputManager::init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh) {

    hardware_interface::DsmInterface* d = hw->get<hardware_interface::DsmInterface>();
    dsm_ = d->getHandle("dsm");

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &InputManager::cmdVelCallback, this);

    return true;
  }


  void InputManager::update(const ros::Time& now) {

    if (*dsm_.getOk() and *dsm_.getModeSwitch() > 0.5) {
      rt_commands_.lin = *dsm_.getDriveStick() * DRIVE_RATE_ADVANCED;
      rt_commands_.ang = *dsm_.getTurnStick() * TURN_RATE_ADVANCED;
    }
    else {
      rt_commands_ = *(command_.readFromRT());
      const double dt = (now - rt_commands_.stamp).toSec();
      if (dt > 0.5) {
	rt_commands_.lin = 0.;
	rt_commands_.ang = 0.;
      }
      //else
      //	std::cerr << "ros cmd" << std::endl;
    }
    
  }

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void InputManager::cmdVelCallback(const geometry_msgs::Twist& command) {
    //    if (isRunning()) {
    nrt_ros_command_struct_.ang   = command.angular.z;
    nrt_ros_command_struct_.lin   = command.linear.x;
    nrt_ros_command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (nrt_ros_command_struct_);
    //ROS_DEBUG_STREAM_NAMED(__NAME,
    //			   "Added values to command. "
    //			   << "Ang: "   << command_struct_.ang << ", "
    //			   << "Lin: "   << command_struct_.lin << ", "
    //			   << "Stamp: " << command_struct_.stamp);
    //    }
    //    else {
    //     ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    //   }
  }


  
}
