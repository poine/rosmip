
#include "rosmip_control/state_estimation.h"

#include <tf/LinearMath/Transform.h>

namespace rosmip_controller {

  double get_yaw(const tf::Quaternion& q) {
    // atan2(2(qx qy+qw qz), 1 - 2 (qy^2+qz^2))
    return atan2(2.0*(q.x()*q.y() + q.w()*q.z()),
		 1.-2*(q.y()*q.y()+q.z()*q.z()));
  }
  double get_pitch(const tf::Quaternion& q) {
    // -asin(2.*x*z - 2.*w*y)
    return -asin(2.*q.x()*q.z()-2*q.w()*q.y());
  } 
  
  StateEstimator::StateEstimator(double wheel_r, double wheel_sep, size_t velocity_rolling_window_size):
      wheel_radius_(wheel_r)
    , wheel_separation_(wheel_sep)
    , velocity_rolling_window_size_(velocity_rolling_window_size)
    , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
    , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  {}


  void StateEstimator::set_wheels_params(double wheel_r, double wheel_sep) {
    std::cerr << "StateEstimator::set_wheels_params:" << wheel_r << " " << wheel_sep << std::endl;
    wheel_radius_ = wheel_r;
    wheel_separation_ = wheel_sep;
  }


  
  void  StateEstimator::init() {
    tf::Quaternion q_base_to_imu;
    q_base_to_imu.setRPY(M_PI/2, 0, M_PI/2);
    q_imu_to_base_ = q_base_to_imu.inverse();
  }

  
  void  StateEstimator::starting(const ros::Time& now, const double* odom_to_imu_q, const double lw_phi, const double rw_phi) {
    x_ = 0.;
    y_ = 0.;
    left_wheel_phi = lw_phi;
    right_wheel_phi = rw_phi;
      
    left_wheel_old_pos_ = lw_phi * wheel_radius_;
    right_wheel_old_pos_ = rw_phi * wheel_radius_;
    odom_yaw_ = (left_wheel_old_pos_+right_wheel_old_pos_) / wheel_separation_;

    resetAccumulators();
    timestamp_ = now;
    
  }
  
  void  StateEstimator::update(const ros::Time &now, const double* imu_rvel, const double* odom_to_imu_q, const double lw_phi, const double rw_phi) {

    q_odom_to_imu_ = tf::Quaternion(odom_to_imu_q[0], odom_to_imu_q[1], odom_to_imu_q[2], odom_to_imu_q[3]); // x, y, z, w
    q_odom_to_base_ =  q_odom_to_imu_ * q_imu_to_base_;
    tf::Matrix3x3(q_odom_to_base_).getRPY( inertial_roll_, inertial_pitch_, inertial_yaw_ );
    //inertial_yaw_ =  get_yaw(q_odom_to_base_);
    //pitch_ = get_pitch(q_odom_to_base_);
    pitch_dot_ = imu_rvel[2]; // FIXME, I get rz???
    //pitch_dot_ = imu_rvel[0];

    // Store current wheel angles
    left_wheel_phi = lw_phi;
    right_wheel_phi = rw_phi;
    
    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = lw_phi * wheel_radius_;
    const double right_wheel_cur_pos = rw_phi * wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;
    
    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;
    
    /// Compute linear and angular diff:
    const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
    const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

    /// Integrate odometry:
    integrate(linear, angular);

    const double dt = (now - timestamp_).toSec();
    if (dt < 0.0001)
      return; // Interval too small to differentiate
    
    timestamp_ = now;
    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    
  }

  void StateEstimator::resetAccumulators() {
  }
  
  void StateEstimator::integrate(double linear, double angular) {
    if (fabs(angular) < 1e-6) {  /// Runge-Kutta 2nd order integration:
       const double direction = odom_yaw_ + angular * 0.5;
       x_       += linear * cos(direction);
       y_       += linear * sin(direction);
       odom_yaw_ += angular;
    }
    else {                       /// Exact integration (should solve problems when angular is zero):
      const double yaw_old = odom_yaw_;
      const double r = linear/angular;
      odom_yaw_ += angular;
      x_        +=  r * (sin(odom_yaw_) - sin(yaw_old));
      y_        += -r * (cos(odom_yaw_) - cos(yaw_old));
    }
  }
  

}
