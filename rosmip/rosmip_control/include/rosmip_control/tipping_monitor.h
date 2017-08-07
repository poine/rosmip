#ifndef ROSMIP_CONTROL_TIPPING_MONITOR_H
#define ROSMIP_CONTROL_TIPPING_MONITOR_H

namespace rosmip_controller {
  enum TippingStatus { UPRIGHT, TIPPED };


 class TippingMonitor {

  public:
    TippingMonitor();

    void update(double theta);

    enum TippingStatus status_;
    enum TippingStatus prev_status_;
  
 private:
    unsigned int counter_;
    
  };

}

#endif // ROSMIP_CONTROL_TIPPING_MONITOR_H
