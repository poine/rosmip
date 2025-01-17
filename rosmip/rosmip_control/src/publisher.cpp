#include "rosmip_control/publisher.h"

#include <boost/assign.hpp>
#include <tf/transform_datatypes.h>

namespace rosmip_controller {


  //********************************
  //  Send rosmip_control::msg_debug_io messages
  //  They contain high frequency encoders and imu measurements
  //  sent in chuncks.
  //********************************
  DebugIOPublisher::DebugIOPublisher() {}

  void DebugIOPublisher::init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    pub_.reset(new realtime_tools::RealtimePublisher<rosmip_control::msg_debug_io>(controller_nh, "debug_io", 100));
    nb_data_ = 0;
  }

  void DebugIOPublisher::publish(const double lw_angle, const double rw_angle,
				 const double lw_rvel, const double rw_rvel,
				 const double pitch, const double pitch_dot,
				 const int8_t lw_pwm, const int8_t rw_pwm,
				 const ros::Time& now) {
    lw_angle_[nb_data_] = lw_angle;
    rw_angle_[nb_data_] = rw_angle;
    lw_rvel_[nb_data_] = lw_rvel;
    rw_rvel_[nb_data_] = rw_rvel;
    lw_pwm_[nb_data_] = lw_pwm;
    rw_pwm_[nb_data_] = rw_pwm;
    pitch_[nb_data_] = pitch;
    pitch_dot_[nb_data_] = pitch_dot;
    stamp_[nb_data_] = now;
    nb_data_ += 1;

    if (nb_data_ >= MIN_SENSOR_FOR_PUBLISH) {
      if (pub_->trylock()) {
	memcpy(pub_->msg_.stamp.elems, stamp_, nb_data_*sizeof(ros::Time));

	memcpy(pub_->msg_.lw_angle.elems, lw_angle_, nb_data_*sizeof(double));
	memcpy(pub_->msg_.rw_angle.elems, rw_angle_, nb_data_*sizeof(double));

	memcpy(pub_->msg_.lw_rvel.elems, lw_rvel_, nb_data_*sizeof(double));
	memcpy(pub_->msg_.rw_rvel.elems, rw_rvel_, nb_data_*sizeof(double));

	memcpy(pub_->msg_.lw_pwm.elems, lw_pwm_, nb_data_*sizeof(int8_t));
	memcpy(pub_->msg_.rw_pwm.elems, rw_pwm_, nb_data_*sizeof(int8_t));

	memcpy(pub_->msg_.pitch.elems, pitch_, nb_data_*sizeof(double));
	memcpy(pub_->msg_.pitch_dot.elems, pitch_dot_, nb_data_*sizeof(double));

	pub_->msg_.nb_data = nb_data_;
	pub_->unlockAndPublish();
	nb_data_ = 0;
      }
      else {
	if (nb_data_ < MAX_SENSOR_LEN) {
	  ROS_INFO("#### DebugIOPublisher::publish() could no publish... will retry");
	}
	else {
	  ROS_INFO("#### DebugIOPublisher::publish() could no publish... discarding one");
	  nb_data_ -= 1;
	}
      }
    }
  }

  //********************************
  //  Send rosmip_control::msg_ctl_debug messages
  //  They contain high frequency encoders and imu measurements
  //  sent in chuncks.
  //********************************
  DebugCtlPublisher::DebugCtlPublisher():
    publish_period_(1.0 / 50.) {
  }

  void DebugCtlPublisher::init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    pub_.reset(new realtime_tools::RealtimePublisher<rosmip_control::msg_debug_ctl>(controller_nh, "debug_ctl", 100));
  }

  void DebugCtlPublisher::starting(const ros::Time& now) {
    last_state_publish_time_ = now;
   }

  
  void DebugCtlPublisher::publish(const LegacyCtlLaw& ctl, const TippingMonitor& tip_mon, const ros::Time& now) {
    if (now < last_state_publish_time_ + publish_period_)
      return;
    last_state_publish_time_ += publish_period_;
    if (pub_->trylock()) {
      pub_->msg_.upright = tip_mon. status_;
      pub_->msg_.sp_theta = ctl.setpoint_.theta;
      pub_->msg_.sp_phi = ctl.setpoint_.phi;
      pub_->msg_.sp_phi_dot = ctl.setpoint_.phi_dot;
      pub_->msg_.sp_gamma = ctl.setpoint_.gamma;
      pub_->msg_.sp_gamma_dot = ctl.setpoint_.gamma_dot;

      pub_->msg_.wheel_angle_l = ctl.core_state_.wheelAngleL;
      pub_->msg_.wheel_angle_r = ctl.core_state_.wheelAngleR;
      pub_->msg_.theta = ctl.core_state_.theta;
      pub_->msg_.phi = ctl.core_state_.phi;
      pub_->msg_.gamma = ctl.core_state_.gamma;
      pub_->msg_.d1_u = ctl.core_state_.d1_u;
      pub_->msg_.d2_u = ctl.core_state_.d2_u;
      pub_->msg_.d3_u = ctl.core_state_.d3_u;
      pub_->msg_.duty_l = ctl.core_state_.dutyL;
      pub_->msg_.duty_r = ctl.core_state_.dutyR;
      pub_->unlockAndPublish();
    }
  }

  

  //********************************
  //  Copied from Homere... I really need a common codebase for controllers
  //********************************
  OdomPublisher::OdomPublisher():
      odom_frame_id_("odom")
    , base_frame_id_("base_link")
    , base_link_("base_link")
    , enable_odom_tf_(true)
    , publish_period_(1.0 / 50.)
  {}
  

  void OdomPublisher::init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    controller_nh.getParam("enable_odom_tf", enable_odom_tf_);
    ROS_INFO_STREAM("enable odom tf " << enable_odom_tf_);
    controller_nh.getParam("base_frame_id",  base_frame_id_);
    controller_nh.getParam("odom_frame_id",  odom_frame_id_);
    ROS_INFO_STREAM("base_frame_id " << base_frame_id_);
    ROS_INFO_STREAM("odom_frame_id " << odom_frame_id_);

    
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
                                      (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                                      (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
                                      (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
                                      (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
                                      (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
                                      (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
                                       (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                                       (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
                                       (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
                                       (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
                                       (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
                                       (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;

  }

  void OdomPublisher::starting(const ros::Time& now) {
    last_state_publish_time_ = now;
   }
  
  // void OdomPublisher::publish(const double heading, const double x, const double y, const double linear, const double angular, const ros::Time& now) {
  //   if (last_state_publish_time_ + publish_period_ < now) {
  //     last_state_publish_time_ += publish_period_;
      
  //     const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(heading));

  //     if (odom_pub_->trylock())
  //       {
  //           odom_pub_->msg_.header.stamp = now;
  //           odom_pub_->msg_.pose.pose.position.x = x;
  //           odom_pub_->msg_.pose.pose.position.y = y;
  // 	    odom_pub_->msg_.pose.pose.position.z = 0.;
  //           odom_pub_->msg_.pose.pose.orientation = orientation;
  //           odom_pub_->msg_.twist.twist.linear.x  = linear;
  //           odom_pub_->msg_.twist.twist.angular.z = angular;
  //           odom_pub_->unlockAndPublish();
  //       }
      
  //     if (enable_odom_tf_ && tf_odom_pub_->trylock())
  //       {
  // 	  geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
  // 	  odom_frame.header.stamp = now;
  // 	  odom_frame.transform.translation.x = x;
  // 	  odom_frame.transform.translation.y = y;
  // 	  odom_frame.transform.rotation = orientation;
  // 	  tf_odom_pub_->unlockAndPublish();
  //       }
  //   }
    
  // }
  

  void OdomPublisher::publish(rosmip_controller::StateEstimator& se, const ros::Time& now) {
    if (last_state_publish_time_ + publish_period_ < now) {
      last_state_publish_time_ += publish_period_;

#if 1
      const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(se.odom_yaw_));
#else
      geometry_msgs::Quaternion orientation;
      quaternionTFToMsg(se.q_odom_to_base_ , orientation);
#endif
	
      if (odom_pub_->trylock())
        {
	  odom_pub_->msg_.header.stamp = now;
	  odom_pub_->msg_.pose.pose.position.x = se.x_;
	  odom_pub_->msg_.pose.pose.position.y = se.y_;
	  odom_pub_->msg_.pose.pose.position.z = 0.;
	  odom_pub_->msg_.pose.pose.orientation = orientation;
	  odom_pub_->msg_.twist.twist.linear.x  = se.linear_;
	  odom_pub_->msg_.twist.twist.angular.z = se.angular_;
	  odom_pub_->unlockAndPublish();
        }
      
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
	  geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
	  odom_frame.header.stamp = now;
	  odom_frame.transform.translation.x = se.x_;
	  odom_frame.transform.translation.y = se.y_;
	  odom_frame.transform.rotation = orientation;
	  tf_odom_pub_->unlockAndPublish();
        }
      
    }
  }
  
}

