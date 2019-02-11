#ifndef ROSMIP_CONTROLLER_H
#define ROSMIP_CONTROLLER_H

#include <dynamic_reconfigure/server.h>
#include <rosmip_control/balancerConfig.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/TwistStamped.h>
#include <rosmip_control/debug.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/LinearMath/Quaternion.h>

#ifdef USE_ROBOTICSCAPE
#include "roboticscape.h"
#else
#include <robotcontrol.h>
#endif

#include "rosmip_control/rosmip_hardware_interface.h"
#include "rosmip_control/state_estimation.h"
#include "rosmip_control/tipping_monitor.h"
#include "rosmip_control/input_manager.h"
#include "rosmip_control/publisher.h"
#include "rosmip_control/legacy_ctl_law.h"
#include "rosmip_control/sfb_ctl_law.h"

namespace rosmip_controller {

  class RosMipLegacyController :
    public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::ImuSensorInterface, hardware_interface::DsmInterface>
    {
  public:
    RosMipLegacyController();
    ~RosMipLegacyController();
    
    bool init(hardware_interface::RobotHW* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& , const ros::Duration&);
    void stopping(const ros::Time&);

    
  private:

    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;
    hardware_interface::ImuSensorHandle imu_;
    //hardware_interface::DsmHandle dsm_;
 
    /// Publishers
    std::shared_ptr<realtime_tools::RealtimePublisher<rosmip_control::debug> > debug_pub_;
    rosmip_controller::OdomPublisher    odom_publisher_;
    rosmip_controller::DebugIOPublisher debug_io_publisher_;
 
    /// Control Law
    rosmip_controller::LegacyCtlLaw ctl_law_;
    rosmip_controller::SFBCtlLaw sfb_ctl_law_;
    rosmip_controller::StateEstimator state_est_;
    rosmip_controller::TippingMonitor tip_mon_;
    rosmip_controller::InputManager   inp_mng_;

    // we keep a pointer on it for non standard stuff like radio control and motors on/off
    RosMipHardwareInterface* hw_;

    void publishDebug(const ros::Time& now);
    void resetControlLaw();
    
  };

}


#endif 
