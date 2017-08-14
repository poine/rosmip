#ifndef ROSMIP_CONTROL_INPUT_MANAGER
#define ROSMIP_CONTROL_INPUT_MANAGER

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "rosmip_control/rosmip_hardware_interface.h"

namespace rosmip_controller {

  class InputManager {

  public:
    InputManager();
    bool init(hardware_interface::RobotHW* hw, ros::NodeHandle &controller_nh);
    void update();
   
    struct Commands
    {
      double lin;
      double ang;
      ros::Time stamp;
      
      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    hardware_interface::DsmHandle dsm_;
 private:
   void cmdVelCallback(const geometry_msgs::Twist& command);
   

  };

}

#endif // ROSMIP_CONTROL_INPUT_MANAGER
