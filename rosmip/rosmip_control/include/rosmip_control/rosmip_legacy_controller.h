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
#include "rosmip_control/input_manager.h"

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
    bool enable_odom_tf_;
    std::shared_ptr<realtime_tools::RealtimePublisher<rosmip_control::debug> > debug_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

    /// Control Law
    struct setpoint_t {
      //	arm_state_t arm_state;	  // see arm_state_t declaration
      //	drive_mode_t drive_mode;  // NOVICE or ADVANCED
      float theta;			  // body lean angle (rad)
      float phi;			  // wheel position (rad)
      float phi_dot;			  // rate at which phi reference updates (rad/s)
      float gamma;			  // body turn angle (rad)
      float gamma_dot;		          // rate at which gamma setpoint updates (rad/s)
    };
    struct setpoint_t setpoint_;
    struct core_state_t {
      float wheelAngleR;	// wheel rotation relative to body
      float wheelAngleL;
      float theta; 		// body angle radians
      float phi;		// average wheel angle in global frame
      float gamma;		// body turn (yaw) angle radians
      float vBatt; 		// battery voltage 
      float d1_u;		// output of balance controller D1 to motors
      float d2_u;		// output of position controller D2 (theta_ref)
      float d3_u;		// output of steering controller D3 to motors
      float dutyL;	        //
      float dutyR;	        //
    };
    struct core_state_t core_state_;
    
    rc_filter_t D1_, D2_, D3_;

    StateEstimator state_est_;
    TippingMonitor tip_mon_;
    InputManager   inp_mng_;
    // we keep a pointer on it for non standard stuff like radio control and motors on/off
    RosMipHardwareInterface* hw_;

    void publishOdometry(const ros::Time& now);
    void publishDebug(const ros::Time& now);
    void resetControlLaw();
    
  };

}


#endif 
