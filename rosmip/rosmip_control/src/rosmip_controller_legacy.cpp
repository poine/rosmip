#include <cmath>
#include <boost/assign.hpp>
#include <tf/transform_datatypes.h>

#include <rosmip_control/rosmip_legacy_controller.h>

//#define DISABLE_MOTORS
namespace rosmip_controller {

 
  double saturate(double _v, double _min, double _max) {
    if (_v < _min) return _min;
    if (_v > _max) return _max;
    return _v;
  }

  
#define __NAME "rosmip_balance_controller"
#define THETA_0	          -0.2
#define ENCODER_CHANNEL_L  1
#define ENCODER_CHANNEL_R  2
#define WHEEL_RADIUS_M     0.03
//#define WHEEL_TRACK_M      (0.083+0.01)
#define WHEEL_TRACK_M      (0.083)


  
/*******************************************************************************
 *
 *
 *******************************************************************************/
RosMipLegacyController::RosMipLegacyController():
  state_est_(WHEEL_RADIUS_M, WHEEL_TRACK_M)
{
  ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::RosMipLegacyController...");
  
    
}
  
/*******************************************************************************
 *
 *
 *******************************************************************************/
RosMipLegacyController:: ~RosMipLegacyController() {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::~RosMipLegacyController...");
}


#define SQR(_a) (_a*_a)
/*******************************************************************************
 *
 *
 *******************************************************************************/
bool RosMipLegacyController::init(hardware_interface::RobotHW* hw,
				    ros::NodeHandle& root_nh,
				    ros::NodeHandle& controller_nh)
{

    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::init...");
    hw_ = static_cast<RosMipHardwareInterface*>(hw);
    hardware_interface::EffortJointInterface* e = hw->get<hardware_interface::EffortJointInterface>();
    left_wheel_joint_  = e->getHandle("left_wheel_joint");
    right_wheel_joint_ = e->getHandle("right_wheel_joint");

    hardware_interface::ImuSensorInterface* i = hw->get<hardware_interface::ImuSensorInterface>();
    imu_ = i->getHandle("imu");

    //hardware_interface::DsmInterface* d = hw->get<hardware_interface::DsmInterface>();
    //dsm_ = d->getHandle("dsm");
    
    state_est_.init();

    double odom_ws, odom_lr, odom_rr;
    controller_nh.getParam("odom_ws", odom_ws);
    controller_nh.getParam("odom_lr", odom_lr);
    controller_nh.getParam("odom_rr", odom_rr);
    state_est_.set_wheels_params(odom_lr, odom_ws);

    ctl_law_.init();
    sfb_ctl_law_.init();
    
    inp_mng_.init(hw, controller_nh);
    
    debug_pub_.reset(new realtime_tools::RealtimePublisher<rosmip_control::debug>(controller_nh, "debug", 100));
 
    odom_publisher_.init(root_nh, controller_nh);
    debug_io_publisher_.init(root_nh, controller_nh);
    
    ROS_INFO_STREAM_NAMED(__NAME, "leaving RosMipLegacyController::init...");
    return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipLegacyController::starting(const ros::Time& now) {
  ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::starting...");
  state_est_.starting(now, imu_.getOrientation(), left_wheel_joint_.getPosition(), right_wheel_joint_.getPosition());
  ctl_law_.starting();
  sfb_ctl_law_.starting();
  odom_publisher_.starting(now);
}
  
/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipLegacyController::update(const ros::Time& now, const ros::Duration& dt) {
  // state estimation
  state_est_.update(now, imu_.getAngularVelocity(), imu_.getOrientation(), left_wheel_joint_.getPosition(), right_wheel_joint_.getPosition());

  tip_mon_.update(state_est_.inertial_pitch_ + THETA_0);
 
  if (tip_mon_.prev_status_ == TIPPED and tip_mon_.status_ == UPRIGHT) {
    resetControlLaw();
    hw_->switch_motors_on();
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::update... switching motors on");
  }
  if (tip_mon_.prev_status_ == UPRIGHT and tip_mon_.status_ == TIPPED) {
    hw_->switch_motors_off();
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::update... switching motors off");
  }

  inp_mng_.update(now);

  //ctl_law_.update(state_est_, inp_mng_);
  ctl_law_.update(state_est_.inertial_pitch_, state_est_.left_wheel_phi, state_est_.right_wheel_phi, inp_mng_.rt_commands_.lin, inp_mng_.rt_commands_.ang);
  //_sfb_ctl_law_.update();
  
  
#ifdef DISABLE_MOTORS
  left_wheel_joint_.setCommand(0);
  right_wheel_joint_.setCommand(0);
#else
  //ROS_INFO("  in RosMipLegacyController::update setpoint: %.2f %.2f %.2f %.2f %.2f",
  //   	     setpoint_.theta, setpoint_.phi, setpoint_.phi_dot, setpoint_.gamma, setpoint_.gamma_dot);
  //ROS_INFO("  in RosMipLegacyController::update duties:%f %f", core_state_.dutyL, core_state_.dutyR);
  left_wheel_joint_.setCommand(ctl_law_.core_state_.dutyL);
  right_wheel_joint_.setCommand(ctl_law_.core_state_.dutyR);
#endif

  odom_publisher_.publish(state_est_.odom_yaw_, state_est_.x_, state_est_.y_,
			  state_est_.linear_, state_est_.angular_, now);
  publishDebug(now);
  debug_io_publisher_.publish(state_est_.left_wheel_phi, state_est_.right_wheel_phi,
			      0, 0,
			      state_est_.inertial_pitch_, state_est_.pitch_dot_,
			      ctl_law_.core_state_.dutyL*512, ctl_law_.core_state_.dutyR*512,
			      now);
}



/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipLegacyController::publishDebug(const ros::Time& now) {
  if (debug_pub_->trylock()) {
    //debug_pub_->msg_.header.stamp = now;
    debug_pub_->msg_.gamma = ctl_law_.core_state_.gamma;
    debug_pub_->msg_.theta = ctl_law_.core_state_.theta;
    //debug_pub_->msg_.thetad = thetad;
    debug_pub_->msg_.phiL = ctl_law_.core_state_.wheelAngleL;
    debug_pub_->msg_.phiR = ctl_law_.core_state_.wheelAngleR;
    //debug_pub_->msg_.phidL = phidL;
    //debug_pub_->msg_.phidR = phidR;
    debug_pub_->msg_.gamma_sp = ctl_law_.setpoint_.gamma;
    debug_pub_->msg_.theta_sp = ctl_law_.setpoint_.theta;
    debug_pub_->msg_.phiL_sp = ctl_law_.setpoint_.phi;
    debug_pub_->msg_.phiR_sp = ctl_law_.setpoint_.gamma;
    debug_pub_->msg_.tauL = ctl_law_.core_state_.dutyL;
    debug_pub_->msg_.tauR = ctl_law_.core_state_.dutyR;
    debug_pub_->unlockAndPublish();
  }
}


/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipLegacyController::resetControlLaw() {
  ctl_law_.reset();
#ifdef USE_ROBOTICSCAPE
  rc_set_motor_all(0.0f);
  rc_set_encoder_pos(ENCODER_CHANNEL_L,0);
  rc_set_encoder_pos(ENCODER_CHANNEL_R,0);
#else
  rc_motor_set(0,0.f);
  rc_encoder_write(ENCODER_CHANNEL_L,0);
  rc_encoder_write(ENCODER_CHANNEL_R,0);
#endif // TODO
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipLegacyController::stopping(const ros::Time&) {
  ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::stop...");
  left_wheel_joint_.setCommand(0);
  right_wheel_joint_.setCommand(0);
}
  
  
  PLUGINLIB_EXPORT_CLASS(rosmip_controller::RosMipLegacyController, controller_interface::ControllerBase);
}
