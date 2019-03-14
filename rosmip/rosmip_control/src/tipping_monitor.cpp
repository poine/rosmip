#include <math.h>
#include <ros/ros.h>

#include "rosmip_control/tipping_monitor.h"

//#define MAX_THETA_FOR_TIPPING 0.85 // 48.7 degres
#define MAX_THETA_FOR_TIPPING (45/180.*M_PI)
#define MIN_TIME_FOR_UPRIGHT 200
#define MAX_TIME_FOR_TIPPING 50

namespace rosmip_controller {

  TippingMonitor::TippingMonitor():
    status_(UPRIGHT),
    prev_status_(TIPPED),
    counter_(0) {
    max_bank_ = cos(MAX_THETA_FOR_TIPPING);
  }


  void TippingMonitor::update(tf::Vector3 vert_body) {
    
    //std::cerr << vert_body.getX() << " " << vert_body.getY() << " " << vert_body.getZ() << " " << std::endl;
    ROS_INFO_STREAM_THROTTLE(0.1, "vert_body " << vert_body.getX() << " " << vert_body.getY() << " " << vert_body.getZ());
    prev_status_ = status_;
    ROS_INFO_STREAM_THROTTLE(0.1, "status:" << prev_status_ << "" <<  status_); // every 0.1 s
    switch (status_) {
    case TIPPED:
      if (vert_body.getZ() >= max_bank_) {
	counter_ += 1;
	if (counter_ > MIN_TIME_FOR_UPRIGHT) {
	  status_ = UPRIGHT;
	  ROS_INFO("switching to UPRIGHT");
	  fflush(stdout); 
	}
      }
      else
	counter_ = 0;
      break;
    case UPRIGHT:
      if (vert_body.getZ() <  max_bank_) {
	counter_ += 1;
    	if (counter_ > MAX_TIME_FOR_TIPPING) {
	  status_ = TIPPED;
	  ROS_INFO("switching to TIPPED");
	  fflush(stdout); 
	}
      }
      else
	counter_ = 0;
      break;
    }
  }

  
}

