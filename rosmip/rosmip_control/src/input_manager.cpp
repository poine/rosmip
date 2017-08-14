#include "rosmip_control/input_manager.h"


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


  void InputManager::update() {

  }

  
  /*******************************************************************************
   *
   *
   *******************************************************************************/
  void InputManager::cmdVelCallback(const geometry_msgs::Twist& command) {
    //    if (isRunning()) {
    command_struct_.ang   = command.angular.z;
    command_struct_.lin   = command.linear.x;
    command_struct_.stamp = ros::Time::now();
    command_.writeFromNonRT (command_struct_);
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
