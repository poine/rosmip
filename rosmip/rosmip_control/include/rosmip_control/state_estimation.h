#ifndef ROSMIP_CONTROL_STATE_ESTIMATION_H
#define ROSMIP_CONTROL_STATE_ESTIMATION_H

#include <ros/ros.h>

namespace rosmip_controller {

  class StateEstimator {

  public:
    StateEstimator();

    void init();
    void starting(const ros::Time& time);
    void update(double left_pos, double right_pos, const ros::Time &time);

    /// Current timestamp:
    ros::Time timestamp_;
    /// Current pose:
    double x_, y_, heading_;   // m, m, rad
    /// Current velocity:
    double linear_,  angular_; // m/s, rad/s

    /// Previou wheel position/state [rad]:
    double left_wheel_old_pos_,  right_wheel_old_pos_;

    /// Wheel kinematic parameters [m]:
    double wheel_separation_;
    double wheel_radius_;

    void integrate(double linear, double angular);
    
  };

}

#endif // ROSMIP_CONTROL_STATE_ESTIMATION_H
