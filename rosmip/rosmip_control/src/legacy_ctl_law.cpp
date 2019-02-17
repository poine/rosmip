
#include <rosmip_control/legacy_ctl_law.h>


#define THETA_0	          -0.2
#define WHEEL_RADIUS_M     0.03
#define WHEEL_TRACK_M      (0.083)

#define SOFT_START_SEC     0.7
#define DT                 0.01
// inner loop controller 100hz
#define D1_GAIN	           1.05
#define D1_ORDER           2
#define D1_NUM             {-4.945, 8.862, -3.967}
#define D1_DEN             { 1.000, -1.481, 0.4812}
// outer loop controller new 100hz
#define D2_GAIN            0.9
#define	D2_ORDER           2
#define D2_NUM             {0.18856,  -0.37209,  0.18354}
#define D2_DEN	           {1.00000,  -1.86046,   0.86046}
#define THETA_REF_MAX      0.33
// steering controller
#define D3_KP              1.0
#define D3_KI              0.3
#define D3_KD              0.05
#define STEERING_INPUT_MAX 0.5


namespace rosmip_controller {

  LegacyCtlLaw::LegacyCtlLaw():
    wr_(WHEEL_RADIUS_M),
    ws_(WHEEL_TRACK_M) {
    // set up D1 Theta controller (inner loop)
    double D1_num[] = D1_NUM, D1_den[] = D1_DEN;
    set_d1_params(D1_GAIN, D1_num, D1_den);

    // set up D2 Phi controller (outer loop)
    double D2_num[] = D2_NUM, D2_den[] = D2_DEN;
    set_d2_params(D2_GAIN, D2_num, D2_den);

    // set up D3 gamma (steering) controller
    set_d3_params(D3_KP, D3_KD, D3_KI, STEERING_INPUT_MAX);
  }

  void LegacyCtlLaw::set_d1_params(const double _gain, double* _num, double* _den) {
    D1_ = rc_filter_empty();
    rc_filter_alloc_from_arrays(&D1_, DT, _num, D1_ORDER+1, _den, D1_ORDER+1);
    D1_.gain = _gain;
    rc_filter_enable_saturation(&D1_, -1.0, 1.0);
    rc_filter_enable_soft_start(&D1_, SOFT_START_SEC);
  }

  void LegacyCtlLaw::set_d2_params(const double _gain, double* _num, double* _den) {
    D2_ = rc_filter_empty();
    rc_filter_alloc_from_arrays(&D2_, DT, _num, D2_ORDER+1, _den, D2_ORDER+1);
    D2_.gain = _gain;
    rc_filter_enable_saturation(&D2_, -THETA_REF_MAX, THETA_REF_MAX);
    rc_filter_enable_soft_start(&D2_, SOFT_START_SEC);
  }
  
  void LegacyCtlLaw::set_d3_params(const double kp, const double kd, const double ki, const double sat) {
    D3_ = rc_filter_empty();
    rc_filter_pid(&D3_, kp, ki, kd, 4*DT, DT);
    rc_filter_enable_saturation(&D3_, -sat, sat);
  }

  void LegacyCtlLaw::init(double wr, double ws) {
    wr_ = wr;
    ws_ = ws;
  }

  void LegacyCtlLaw::starting() {
    reset();
  }

  void LegacyCtlLaw::reset() {
    rc_filter_reset(&D1_);
    rc_filter_reset(&D2_);
    rc_filter_reset(&D3_);

    setpoint_.theta = 0.0f;
    setpoint_.phi   = 0.0f;
    setpoint_.gamma = 0.0f;
  }

  void LegacyCtlLaw::update(double theta, double lwa, double rwa, double lin_sp, double ang_sp) {//rvStateEstimator state_est, InputManager inp_mng) {
    //
    core_state_.theta = theta + THETA_0;//state_est.inertial_pitch_ + THETA_0;
    core_state_.wheelAngleL = lwa;//state_est.left_wheel_phi;
    core_state_.wheelAngleR = rwa;//state_est.right_wheel_phi;
    core_state_.phi = ((core_state_.wheelAngleL+core_state_.wheelAngleR)/2) + core_state_.theta;
    core_state_.gamma = (core_state_.wheelAngleR-core_state_.wheelAngleL) * (wr_/ws_);
    //
    setpoint_.phi_dot = lin_sp/wr_;//inp_mng.rt_commands_.lin/WHEEL_RADIUS_M;
    setpoint_.gamma_dot = ang_sp;//inp_mng.rt_commands_.ang;
    
    setpoint_.phi += setpoint_.phi_dot*DT;
    core_state_.d2_u = rc_filter_march(&D2_, setpoint_.phi-core_state_.phi);
    setpoint_.theta = core_state_.d2_u;
    
    //D1.gain = D1_GAIN * V_NOMINAL/cstate.vBatt;
    core_state_.d1_u = rc_filter_march(&D1_, (setpoint_.theta-core_state_.theta));
    setpoint_.gamma += setpoint_.gamma_dot * DT;
    core_state_.d3_u = rc_filter_march(&D3_, setpoint_.gamma - core_state_.gamma);

    core_state_.dutyL = core_state_.d1_u - core_state_.d3_u;
    core_state_.dutyR = core_state_.d1_u + core_state_.d3_u;

  }
  
  
}
