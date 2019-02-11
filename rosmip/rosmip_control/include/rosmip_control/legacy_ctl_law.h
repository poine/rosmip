
#ifdef USE_ROBOTICSCAPE
#include "roboticscape.h"
#else
#include "robotcontrol.h"
#endif
//#include "rosmip_control/state_estimation.h"
//#include "rosmip_control/input_manager.h"

namespace rosmip_controller {

  class LegacyCtlLaw {
  public:
    LegacyCtlLaw();
    void reset();
    void init();
    void starting();
    //StateEstimator state_est, InputManager inp_mng); let's try not to get ROS involved to ease cython interface
    void update(double theta, double lwa, double rwa, double lin_sp, double ang_sp);
    void set_d1_params(const double gain, double* _num, double* _den);

    
    double get_pwm_left()  {return core_state_.dutyL;}
    double get_pwm_right() {return core_state_.dutyR;}

    struct setpoint_t {
      //	arm_state_t arm_state;	  // see arm_state_t declaration
      //	drive_mode_t drive_mode;  // NOVICE or ADVANCED
      float theta;			  // body lean angle (rad)
      float phi;			  // wheel position (rad)
      float phi_dot;			  // rate at which phi reference updates (rad/s)
      float gamma;			  // body turn angle (rad)
      float gamma_dot;		          // rate at which gamma setpoint updates (rad/s)
    };
    struct setpoint_t setpoint_;

    struct core_state_t {
      float wheelAngleR;	// wheel rotation relative to body
      float wheelAngleL;
      float theta; 		// body angle radians
      float phi;		// average wheel angle in global frame
      float gamma;		// body turn (yaw) angle radians
      float vBatt; 		// battery voltage 
      float d1_u;		// output of balance controller D1 to motors
      float d2_u;		// output of position controller D2 (theta_ref)
      float d3_u;		// output of steering controller D3 to motors
      float dutyL;	        //
      float dutyR;	        //
    };
    struct core_state_t core_state_;

    rc_filter_t D1_, D2_, D3_;
 
  };
  
}
