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
    ROS_INFO_STREAM_NAMED(__NAME, "    wheels param: " <<  odom_lr << " " << odom_rr << " " << odom_ws);
    ctl_law_.init(odom_lr, odom_ws);

    // Get params for control law
    XmlRpc::XmlRpcValue d1_mun_list, d1_den_list; double D1_gain;
    controller_nh.getParam("d1_num", d1_mun_list);
    controller_nh.getParam("d1_den", d1_den_list);
    double D1_num[] = {static_cast<double>(d1_mun_list[0]), static_cast<double>(d1_mun_list[1]), static_cast<double>(d1_mun_list[2])};
    double D1_den[] = {static_cast<double>(d1_den_list[0]), static_cast<double>(d1_den_list[1]), static_cast<double>(d1_den_list[2])};
    controller_nh.getParam("d1_gain", D1_gain);
    ROS_INFO_STREAM_NAMED(__NAME, "    d1 num: " << D1_num[0] << ", " << D1_num[1] << ", " << D1_num[2]);
    ROS_INFO_STREAM_NAMED(__NAME, "    d1 den: " << D1_den[0] << ", " << D1_den[1] << ", " << D1_den[2]);
    ROS_INFO_STREAM_NAMED(__NAME, "    d1 gain: " << D1_gain);
    ctl_law_.set_d1_params(D1_gain, D1_num, D1_den);

    XmlRpc::XmlRpcValue d2_mun_list, d2_den_list; double D2_gain;
    controller_nh.getParam("d2_num", d2_mun_list);
    controller_nh.getParam("d2_den", d2_den_list);
    double D2_num[] = {static_cast<double>(d2_mun_list[0]), static_cast<double>(d2_mun_list[1]), static_cast<double>(d2_mun_list[2])};
    double D2_den[] = {static_cast<double>(d2_den_list[0]), static_cast<double>(d2_den_list[1]), static_cast<double>(d2_den_list[2])};
    controller_nh.getParam("d2_gain", D2_gain);
    ROS_INFO_STREAM_NAMED(__NAME, "    d2 num: " << D2_num[0] << ", " << D2_num[1] << ", " << D2_num[2]);
    ROS_INFO_STREAM_NAMED(__NAME, "    d2 den: " << D2_den[0] << ", " << D2_den[1] << ", " << D2_den[2]);
    ROS_INFO_STREAM_NAMED(__NAME, "    d2 gain: " << D2_gain);
    ctl_law_.set_d2_params(D2_gain, D2_num, D2_den);

    double D3_kp, D3_kd, D3_ki, D3_sat;
    controller_nh.getParam("d3_kp", D3_kp);
    controller_nh.getParam("d3_kd", D3_kd);
    controller_nh.getParam("d3_ki", D3_ki);
    controller_nh.getParam("d3_sat", D3_sat);
    ROS_INFO_STREAM_NAMED(__NAME, "    d3 kp: " << D3_kp << " kd: " << D3_kd << " ki: " << D3_ki<< " sat: " << D3_sat);
    ctl_law_.set_d3_params(D3_kp, D3_kd, D3_ki, D3_sat);
    
    state_est_.set_wheels_params(odom_lr, odom_ws);
    sfb_ctl_law_.init();
    
    inp_mng_.init(hw, controller_nh);
    
    odom_publisher_.init(root_nh, controller_nh);
    debug_io_publisher_.init(root_nh, controller_nh);
    debug_ctl_publisher_.init(root_nh, controller_nh);

    
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
  debug_ctl_publisher_.starting(now);
}
  
/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipLegacyController::update(const ros::Time& now, const ros::Duration& dt) {
  // state estimation
  state_est_.update(now, imu_.getAngularVelocity(), imu_.getOrientation(),
		    left_wheel_joint_.getPosition(), right_wheel_joint_.getPosition());

  //tip_mon_.update(state_est_.inertial_pitch_ + THETA_0);
  tip_mon_.update(state_est_.vert_body_);
 
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
  ctl_law_.update(state_est_.inertial_pitch_, state_est_.left_wheel_phi, state_est_.right_wheel_phi,
		  inp_mng_.rt_commands_.lin, inp_mng_.rt_commands_.ang);
  //_sfb_ctl_law_.update();
  
  
#ifdef DISABLE_MOTORS
  left_wheel_joint_.setCommand(0);
  right_wheel_joint_.setCommand(0);
#else
  //ROS_INFO("  in RosMipLegacyController::update setpoint: %.2f %.2f %.2f %.2f %.2f",
  //   	     setpoint_.theta, setpoint_.phi, setpoint_.phi_dot, setpoint_.gamma, setpoint_.gamma_dot);
  //ROS_INFO("  in RosMipLegacyController::update duties:%f %f", core_state_.dutyL, core_state_.dutyR);
  if (tip_mon_.status_ == UPRIGHT) {
    left_wheel_joint_.setCommand(ctl_law_.core_state_.dutyL);
    right_wheel_joint_.setCommand(ctl_law_.core_state_.dutyR);
  }
  else {
    left_wheel_joint_.setCommand(0);
    right_wheel_joint_.setCommand(0);
  }
#endif

  odom_publisher_.publish(state_est_, now);
  debug_ctl_publisher_.publish(ctl_law_, tip_mon_, now);
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
void RosMipLegacyController::resetControlLaw() {
  ROS_INFO_STREAM_NAMED(__NAME, "in RosMipLegacyController::resetControlLaw...");
  ctl_law_.reset();
  rc_motor_set(0, 0.f);
  rc_encoder_write(ENCODER_CHANNEL_L, 0);
  rc_encoder_write(ENCODER_CHANNEL_R, 0);
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
