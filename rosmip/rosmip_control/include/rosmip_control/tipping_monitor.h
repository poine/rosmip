#ifndef ROSMIP_CONTROL_TIPPING_MONITOR_H
#define ROSMIP_CONTROL_TIPPING_MONITOR_H

#include <tf/LinearMath/Quaternion.h>

namespace rosmip_controller {
  enum TippingStatus { UPRIGHT, TIPPED };


 class TippingMonitor {

  public:
    TippingMonitor();

    void update(double theta);
    void update(tf::Vector3 vert_body);
 
    enum TippingStatus status_;
    enum TippingStatus prev_status_;
  
 private:
    unsigned int counter_;
    float max_bank_;
  };

}

#endif // ROSMIP_CONTROL_TIPPING_MONITOR_H
