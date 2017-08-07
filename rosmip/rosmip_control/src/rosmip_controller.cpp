#include <cmath>
#include <rosmip_control/rosmip_controller.h>



namespace rosmip_controller {

  //double get_pitch(const double* q) { return -asin(2.*q[1]*q[3]-2*q[0]*q[2]); } //  -asin(2.*x*z - 2.*w*y)
  double get_pitch(const tf::Quaternion& q) { return -asin(2.*q.x()*q.z()-2*q.w()*q.y()); } //  -asin(2.*x*z - 2.*w*y)
  double saturate(double _v, double _min, double _max) {
    if (_v < _min) return _min;
    if (_v > _max) return _max;
    return _v;
  }

  
#define __NAME "rosmip_balance_controller"
#define SOFT_START_SEC	     0.7
// inner loop controller 100hz new
// D1_GAIN 1.05
#define 	D1_GAIN	     0.02
#define 	D1_ORDER     2
#define 	D1_NUM       {-4.945, 8.862, -3.967}
#define 	D1_DEN       { 1.000, -1.481, 0.4812}
#define         D1_DT        0.01
  
  RosMipController::RosMipController():
    enable_odom_tf_(true)
  {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::RosMipController...");
    // set up D1 Theta controller
    D1_ = rc_empty_filter();
    float D1_num[] = D1_NUM;
    float D1_den[] = D1_DEN;
    rc_alloc_filter_from_arrays(&D1_, D1_ORDER, D1_DT, D1_num, D1_den);
    D1_.gain = D1_GAIN;
    rc_enable_saturation(&D1_, -1.0, 1.0);
    rc_enable_soft_start(&D1_, SOFT_START_SEC);
  }

  RosMipController:: ~RosMipController() {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::~RosMipController...");
  }


  void RosMipController::cfg_callback(rosmip_control::balancerConfig &config, uint32_t level) {
    kp_theta_ = config.kp_theta;
    kd_theta_ = config.kd_theta;
    ROS_INFO("Reconfigure Request: level %d kp_theta %f kd_theta %f", level, kp_theta_, kd_theta_);
  }
  
  bool RosMipController::init(hardware_interface::RobotHW* hw,
			      ros::NodeHandle& root_nh,
			      ros::NodeHandle& controller_nh)
  {

    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::init...");
    hw_ = static_cast<RosMipHardwareInterface*>(hw);
    hardware_interface::EffortJointInterface* e = hw->get<hardware_interface::EffortJointInterface>();
    left_wheel_joint_  = e->getHandle("left_wheel_joint");
    right_wheel_joint_ = e->getHandle("right_wheel_joint");

    hardware_interface::ImuSensorInterface* i = hw->get<hardware_interface::ImuSensorInterface>();
    imu_ = i->getHandle("imu");

    state_est_.init();
    
    debug_pub_.reset(new realtime_tools::RealtimePublisher<rosmip_control::debug>(controller_nh, "debug", 100));
    const std::string base_frame_id_ = "base_link";
    const std::string odom_frame_id_ = "odom";
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;

    q_imu_to_base_.setRPY(0, 0, -M_PI/2); // this is not correct
                                          // this should be fetched from the robot description
                                          // and in theory this should be (math.pi/2, 0, math.pi/2)
                                          // with the current configuration
                                          // so, there is something fishy with the ORIENTATION_Y_BLAH in hardware interface
    //q_imu_to_base_.setRPY(M_PI/2, 0, M_PI/2);

#if 1
    dynamic_reconfigure::Server<rosmip_control::balancerConfig>::CallbackType f;
    f = boost::bind(&RosMipController::cfg_callback, this, _1, _2);
    cfg_server_ = new dynamic_reconfigure::Server<rosmip_control::balancerConfig>(controller_nh);
    cfg_server_->setCallback(f);
#endif
    rosmip_control::balancerConfig config;
    controller_nh.getParam("/rosmip_balance_controller/kp_theta", config.kp_theta);
    //controller_nh.getParameter<double>("/rosmip_balance_controller/kp_theta", config.kp_theta, 0.8);
    controller_nh.getParam("/rosmip_balance_controller/kd_theta", config.kd_theta);
    this->cfg_callback(config, 0);

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &RosMipController::cmdVelCallback, this);
    
    ROS_INFO_STREAM_NAMED(__NAME, "leaving RosMipController::init...");
    return true;
  }

  void RosMipController::starting(const ros::Time& time) {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::starting...");
    state_est_.starting(time);
  }

  void RosMipController::update(const ros::Time& now, const ros::Duration& dt) {
    // state estimation
    const double* odom_to_imu_q =  imu_.getOrientation(); 
    q_odom_to_imu_ = tf::Quaternion(odom_to_imu_q[0], odom_to_imu_q[1], odom_to_imu_q[2], odom_to_imu_q[3]); // x, y, z, w
    q_odom_to_base_ =  q_odom_to_imu_ * q_imu_to_base_;

    double theta = get_pitch(q_odom_to_base_);

    tip_mon_.update(theta);

    if (tip_mon_.prev_status_ == TIPPED and tip_mon_.status_ == UPRIGHT) {
      hw_->switch_motors_on();
      ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::update... switching motors on");
    }
    if (tip_mon_.prev_status_ == UPRIGHT and tip_mon_.status_ == TIPPED)
      hw_->switch_motors_off();
    
    //ROS_INFO_STREAM_NAMED(__NAME, "  in RosMipController::update... %f", dt.toSec());
    //ROS_INFO("  in RosMipController::update... %f", dt.toSec());
    double phiL = left_wheel_joint_.getPosition();
    double phiR = right_wheel_joint_.getPosition();

    state_est_.update(phiL, phiR, now);

    // this we'll soon get from state est
    double phidL = left_wheel_joint_.getVelocity();
    double phidR = right_wheel_joint_.getVelocity();
      
    double phi = (phiL+phiR)/2;
    double phid = (phidL+phidR)/2;
    double sp_phi = 0.5*sin(now.toSec());
    if (fmod(now.toSec(), 4) > 2)
      sp_phi = 0.5;
    else
      sp_phi = -0.5;
    //std::cerr << fmod(now.toSec(), 3) << std::endl;
    double theta0 = 0.;//4;
    double sp_theta = -0.05*(phi-sp_phi)-0.05*phid + theta0;
    //double sp_theta = theta0;//-0.05*phid + theta0;
    

    //const double* quat = imu_.getOrientation();
    //double theta = get_pitch(quat);
    const double* om = imu_.getAngularVelocity();
    double thetad = om[0];

    //const double Kp = 0.8, Kd =  0.1;
    //const double Kp = 0.9, Kd =  0.1;
    double tau = kp_theta_*(theta-sp_theta) + kd_theta_*thetad;

    float foo = rc_march_filter(&D1_, (sp_theta - theta));
    //std::cerr << "gain " << D1_.gain << " filter " << foo << " me " << tau << std::endl;


    left_wheel_joint_.setCommand(tau);
    right_wheel_joint_.setCommand(tau);

    if (debug_pub_->trylock()) {
      //debug_pub_->msg_.header.stamp = now;
      debug_pub_->msg_.theta = theta;
      debug_pub_->msg_.thetad = thetad;
      debug_pub_->msg_.phiL = phiL;
      debug_pub_->msg_.phiR = phiR;
      debug_pub_->msg_.phidL = phidL;
      debug_pub_->msg_.phidR = phidR;
      debug_pub_->msg_.theta_sp = sp_theta;
      debug_pub_->msg_.phiL_sp = sp_phi;
      debug_pub_->msg_.phiR_sp = sp_phi;
      debug_pub_->msg_.tauL = tau;
      debug_pub_->msg_.tauR = foo;
      debug_pub_->unlockAndPublish();
    }
    publishOdometry(now);
    
  }

  void RosMipController::publishOdometry(const ros::Time& now) {
    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = now;
        odom_frame.transform.translation.x = state_est_.x_;
        odom_frame.transform.translation.y = state_est_.y_;
        odom_frame.transform.rotation.x = q_odom_to_base_.x();
	odom_frame.transform.rotation.y = q_odom_to_base_.y();
	odom_frame.transform.rotation.z = q_odom_to_base_.z();
	odom_frame.transform.rotation.w = q_odom_to_base_.w();
        tf_odom_pub_->unlockAndPublish();
      }
  }
  
  void RosMipController::stopping(const ros::Time&) {
    ROS_INFO_STREAM_NAMED(__NAME, "in RosMipController::stop...");
    left_wheel_joint_.setCommand(0);
    right_wheel_joint_.setCommand(0);
  }

  void RosMipController::cmdVelCallback(const geometry_msgs::Twist& command) {
    //    if (isRunning()) {
      command_struct_.ang   = command.angular.z;
      command_struct_.lin   = command.linear.x;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT (command_struct_);
      ROS_DEBUG_STREAM_NAMED(__NAME,
                             "Added values to command. "
                             << "Ang: "   << command_struct_.ang << ", "
                             << "Lin: "   << command_struct_.lin << ", "
                             << "Stamp: " << command_struct_.stamp);
      //    }
      //    else {
      //     ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
      //   }
  }

  


  
  PLUGINLIB_EXPORT_CLASS(rosmip_controller::RosMipController, controller_interface::ControllerBase);
}
