#include <cmath>
#include <rosmip_control/rosmip_controller.h>

namespace rosmip_controller {

  double get_pitch(const double* q) { return -asin(2.*q[1]*q[3]-2*q[0]*q[2]); } //  -asin(2.*x*z - 2.*w*y)
  
#define __NAME "rosmip_dummy_controller"

  RosMipController::RosMipController() {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipDummyController::RosMipDummyController...");
  }

  RosMipController:: ~RosMipController() {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipDummyController::~RosMipDummyController...");
  }


  bool RosMipController::init(hardware_interface::RobotHW* hw,
			      ros::NodeHandle& root_nh,
			      ros::NodeHandle& controller_nh) {

    ROS_INFO_STREAM_NAMED("RosMipController","in RosMipController::init...");
    hardware_interface::EffortJointInterface* e = hw->get<hardware_interface::EffortJointInterface>();
    left_wheel_joint_  = e->getHandle("left_wheel_joint");
    right_wheel_joint_ = e->getHandle("right_wheel_joint");

    hardware_interface::ImuSensorInterface* i = hw->get<hardware_interface::ImuSensorInterface>();
    imu_ = i->getHandle("imu");

    debug_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 100));

    const string base_frame_id_ = "base_link";
    const string base_frame_id_ = "odom";
    
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_
    
    ROS_INFO_STREAM_NAMED("RosMipController","leaving RosMipController::init...");
    return true;
  }

  void RosMipController::starting(const ros::Time& time) {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::starting...");
  }

  void RosMipController::update(const ros::Time& now, const ros::Duration& dt) {


    //ROS_INFO_STREAM_NAMED(__NAME, "  in RosMipController::update...");
    double phi_l = left_wheel_joint_.getPosition();
    double phid_l = left_wheel_joint_.getVelocity();
      
    const double* quat = imu_.getOrientation();
    double theta = get_pitch(quat);
    const double* om = imu_.getAngularVelocity();
    double q = om[1];

    double sp_theta = -0.05*phid_l + 0.005*sin(now.toSec());
    const double Kp = 0.8, Kd =  0.1;
    double tau = Kp*(theta-sp_theta) + Kd*q;

    left_wheel_joint_.setCommand(tau);
    right_wheel_joint_.setCommand(tau);

    if (debug_pub_->trylock()) {
      debug_pub_->msg_.header.stamp = now;
      debug_pub_->msg_.twist.linear.x = tau;
      //debug_pub_->msg_.twist.linear.y = foo;
      debug_pub_->unlockAndPublish();
    }
    
  }


  void RosMipController::publishOdometry(const ros::Time& now) {

  }


  
  void RosMipController::stopping(const ros::Time&) {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::stop...");
    left_wheel_joint_.setCommand(0);
    right_wheel_joint_.setCommand(0);
  }
  
  PLUGINLIB_EXPORT_CLASS(rosmip_controller::RosMipController, controller_interface::ControllerBase);
}
