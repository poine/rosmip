#ifndef ROSMIP_CONTROL_STATE_ESTIMATION_H
#define ROSMIP_CONTROL_STATE_ESTIMATION_H

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>

namespace rosmip_controller {

  class StateEstimator {

  public:
    StateEstimator(double wheel_r, double wheel_sep);

    void init();
    void starting(const ros::Time& time);
    void update(const double* odom_to_imu_q, double left_pos, double right_pos, const ros::Time &time);

    /// Current timestamp:
    ros::Time timestamp_;
    /// Current pose:
    double x_, y_, yaw_, pitch_;         // m, m, rad, rad
    /// Current velocity:
    double linear_,  angular_;       // m/s, rad/s
    /// Orientation
    tf::Quaternion q_imu_to_base_;   // constant
    tf::Quaternion q_odom_to_imu_;   // provided by DMP's AHRS
    tf::Quaternion q_odom_to_base_;  // combination of the above two

    double left_wheel_phi, right_wheel_phi;
    double odom_yaw_, inertial_yaw_;
      
  private:
    /// Previou wheel position/state [rad]:
    double left_wheel_old_pos_,  right_wheel_old_pos_;

    /// Wheel kinematic parameters [m]:
    double wheel_radius_;
    double wheel_separation_;

    void integrate(double linear, double angular);
    
  };

}

#endif // ROSMIP_CONTROL_STATE_ESTIMATION_H
