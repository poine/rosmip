#include "robotcontrol.h"

namespace rosmip_controller {

  class SFBCtlLaw {
  public:
    SFBCtlLaw();
    void reset();
    void init();
    void starting();
    void update(double theta, double lwa, double rwa, double lin_sp, double ang_sp);
    void set_d1_params(const double gain, double* _num, double* _den);

    double get_pwm_left()  {return pwm_l_;}
    double get_pwm_right() {return pwm_r_;}


    double pwm_l_, pwm_r_; 
    
    
  };
}
    
