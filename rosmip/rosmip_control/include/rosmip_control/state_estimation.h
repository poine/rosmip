#ifndef ROSMIP_CONTROL_STATE_ESTIMATION_H
#define ROSMIP_CONTROL_STATE_ESTIMATION_H

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

namespace rosmip_controller {

  namespace bacc = boost::accumulators;
 
  class StateEstimator {

  public:
    StateEstimator(double wheel_r, double wheel_sep, size_t velocity_rolling_window_size = 10);

    void init();
    void set_wheels_params(double wheel_r, double wheel_sep);
    void starting(const ros::Time& now, const double* odom_to_imu_q, const double lw_phi, const double rw_phi);
    void update(const ros::Time &now, const double* imu_rvel, const double* odom_to_imu_q, const double lw_phi, const double rw_phi);

    /// Current timestamp:
    ros::Time timestamp_;
    /// Current pose:
    double x_, y_;                   // m, m, rad, rad
    /// Current velocity:
    double linear_,  angular_;       // m/s, rad/s
    /// Orientation
    tf::Quaternion q_imu_to_base_;   // constant
    tf::Quaternion q_odom_to_imu_;   // provided by DMP's AHRS
    tf::Quaternion q_odom_to_base_;  // combination of the above two
    double inertial_roll_, inertial_pitch_, inertial_yaw_; // RPY from MPU
    tf::Vector3 vert_body_; // vertical in body frame
    double odom_yaw_;
    double pitch_dot_;
    tf::Vector3 body_rvel_;
    
    double left_wheel_phi, right_wheel_phi;
      
  private:
    /// Previou wheel position/state [rad]:
    double left_wheel_old_pos_,  right_wheel_old_pos_;

    /// Wheel kinematic parameters [m]:
    double wheel_radius_;
    double wheel_separation_;

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;
    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

    void integrate(double linear, double angular);
    void resetAccumulators();
    
  };

}

#endif // ROSMIP_CONTROL_STATE_ESTIMATION_H
