#include <math.h>
#include <ros/ros.h>

#include "rosmip_control/tipping_monitor.h"

#define MAX_THETA_FOR_TIPPING 0.85
#define MIN_TIME_FOR_UPRIGHT 200
#define MAX_TIME_FOR_TIPPING 50

namespace rosmip_controller {

  TippingMonitor::TippingMonitor():
    status_(UPRIGHT),
    prev_status_(TIPPED),
    counter_(0) {
    
  }

  void TippingMonitor::update(double theta) {
    prev_status_ = status_;
    //ROS_INFO("status %d %d", prev_status_, status_);
    switch (status_) {
    case TIPPED:
      if (fabs(theta) <  MAX_THETA_FOR_TIPPING) {
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
      if (fabs(theta) >  MAX_THETA_FOR_TIPPING) {
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

