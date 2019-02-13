#ifndef ROSMIP_CONTROL__PUBLISHER_H_
#define ROSMIP_CONTROL__PUBLISHER_H_



#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <rosmip_control/msg_debug_io.h>
#include <rosmip_control/msg_debug_ctl.h>
#include <rosmip_control/state_estimation.h>
#include <rosmip_control/tipping_monitor.h>
#include <rosmip_control/legacy_ctl_law.h>

#define MAX_SENSOR_LEN 15
#define MIN_SENSOR_FOR_PUBLISH 10

namespace rosmip_controller {

  //
  // Publish input/output 
  //
  class DebugIOPublisher {

  public:
    DebugIOPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void publish(const double lw_angle, const double rw_angle,
		 const double lw_rvel, const double rw_rvel,
		 const double pitch, const double pitch_dot,
		 const int8_t lw_pwm, const int8_t rw_pwm,
		 const ros::Time& now);

  private:
    boost::shared_ptr<realtime_tools::RealtimePublisher<rosmip_control::msg_debug_io> > pub_;
    int nb_data_;
    ros::Time stamp_[MAX_SENSOR_LEN];
    double lw_angle_[MAX_SENSOR_LEN];
    double rw_angle_[MAX_SENSOR_LEN];
    double lw_rvel_[MAX_SENSOR_LEN];
    double pitch_[MAX_SENSOR_LEN];
    double pitch_dot_[MAX_SENSOR_LEN];
    double rw_rvel_[MAX_SENSOR_LEN];
    int8_t lw_pwm_[MAX_SENSOR_LEN];
    int8_t rw_pwm_[MAX_SENSOR_LEN];
  };

  //
  // Publish debug_ctl
  //
  class DebugCtlPublisher {
  public:
    DebugCtlPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& now);
    void publish(const LegacyCtlLaw& ctl, const TippingMonitor& tip_mon, const ros::Time& now);
  private:
    boost::shared_ptr<realtime_tools::RealtimePublisher<rosmip_control::msg_debug_ctl> > pub_;
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
  };
  
  //
  // Publish odometry
  //
  class OdomPublisher {

  public:
    OdomPublisher();
    void init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& now);
    //void publish(const double heading, const double x, const double y, const double linear, const double angular, const ros::Time& now);
    void publish(rosmip_controller::StateEstimator& se, const ros::Time& now);

  private:
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string base_link_;
    bool enable_odom_tf_;
 
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

  };

  
}

#endif // ROSMIP_CONTROL__PUBLISHER_H_
