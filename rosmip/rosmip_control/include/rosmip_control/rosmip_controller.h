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

//#include <geometry_msgs/TwistStamped.h>
#include <rosmip_control/debug.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/LinearMath/Quaternion.h>

#include "roboticscape.h"

#include "rosmip_control/rosmip_hardware_interface.h"
#include "rosmip_control/state_estimation.h"
#include "rosmip_control/tipping_monitor.h"

namespace rosmip_controller {

  class RosMipController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                                 hardware_interface::ImuSensorInterface>
  {
  public:
    RosMipController();
    ~RosMipController();
    
    bool init(hardware_interface::RobotHW* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& , const ros::Duration&);
    void stopping(const ros::Time&);

    void publishOdometry(const ros::Time& now);
    
  private:

    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;
    hardware_interface::ImuSensorHandle imu_;

    /// Publishers
    bool enable_odom_tf_;
    std::shared_ptr<realtime_tools::RealtimePublisher<rosmip_control::debug> > debug_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

    /// Dynamic reconfigure
    dynamic_reconfigure::Server<rosmip_control::balancerConfig>* cfg_server_;
    void cfg_callback(rosmip_control::balancerConfig &config, uint32_t level);

    /// State estimation
    tf::Quaternion q_imu_to_base_;   // constant
    tf::Quaternion q_odom_to_imu_;   // provided by DMP's AHRS
    tf::Quaternion q_odom_to_base_;  // combination of the above two

    /// Velocity command related:
    void cmdVelCallback(const geometry_msgs::Twist& command);
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

    
    /// control
    double kp_theta_, kd_theta_;
    
    rc_filter_t D1_, D2_, D3_;

    StateEstimator state_est_;
    TippingMonitor tip_mon_;
    RosMipHardwareInterface* hw_;
    
  };

}


#endif 
