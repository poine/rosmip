#include <rosmip_control/rosmip_hardware_interface.h>

#include <thread>

#define __NAME "rosmip_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint"};



// mechanics
#define GEARBOX                         75.81
#define ENCODER_RES                     12

// electrical hookups
#define MOTOR_CHANNEL_L                 2
#define MOTOR_CHANNEL_R                 1
#define MOTOR_POLARITY_L               -1
#define MOTOR_POLARITY_R                1
#define ENCODER_CHANNEL_L               2
#define ENCODER_CHANNEL_R               1
#define ENCODER_POLARITY_L             -1
#define ENCODER_POLARITY_R              1

// IMU
#define IMU_SAMPLE_RATE_HZ 100
#define IMU_DT (1./IMU_SAMPLE_RATE_HZ)

// DSM channel config
#define DSM_DRIVE_POL			1
#define DSM_TURN_POL		        1
#define DSM_MODE_POL		        1
#define DSM_DRIVE_CH			3
#define DSM_TURN_CH			2
#define DSM_MODE_CH                     6
#define DSM_DEAD_ZONE			0.02


// Global variable because roboticscape guy refused to add a user supplied argument to their callback
// Welcome to the 70s....
static RosMipHardwareInterface* _foo_hw_interf = NULL;


void _imu_callback(void* data) { reinterpret_cast<RosMipHardwareInterface*>(data)->IMUCallback(); }
void _dsm_callback(void* data) { reinterpret_cast<RosMipHardwareInterface*>(data)->DSMCallback(); }


/*******************************************************************************
 *
 *
 *******************************************************************************/
RosMipHardwareInterface::RosMipHardwareInterface():
  dsm_ok_(false),
  turn_stick_(0.),
  drive_stick_(0.),
  mode_switch_(0.),
  gear_enc_res_(GEARBOX*ENCODER_RES)
{
  ROS_INFO_STREAM_NAMED(__NAME, "in RosMipHardwareInterface::RosMipHardwareInterface...");

  ROS_INFO_STREAM_NAMED(__NAME, "Registering interfaces");
  // register joints
  for (int i=0; i<NB_JOINTS; i++) {
    joint_position_[i] = 0.;
    joint_velocity_[i] = 0.;
    joint_effort_[i] = 0.;
    joint_effort_command_[i] = 0.;
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    ej_interface_.registerHandle(hardware_interface::JointHandle(
        js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
  }
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);

  // register IMU
  imu_data_.name = "imu";
  imu_data_.frame_id = "imu_link";
  imu_data_.orientation = imu_orientation_;
  imu_data_.angular_velocity = imu_angular_velocity_;
  imu_data_.linear_acceleration = imu_linear_acceleration_;
  hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
  imu_sensor_interface_.registerHandle(imu_sensor_handle);
  registerInterface(&imu_sensor_interface_);

  // register DSM
  dsm_data_.name = "dsm";
  dsm_data_.ok = &dsm_ok_;
  dsm_data_.drive_stick = &drive_stick_;
  dsm_data_.turn_stick = &turn_stick_;
  dsm_data_.mode_switch = &mode_switch_;
  hardware_interface::DsmHandle dsm_handle(dsm_data_);
  dsm_interface_.registerHandle(dsm_handle);
  registerInterface(&dsm_interface_);

  
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
RosMipHardwareInterface::~RosMipHardwareInterface() {
  ROS_INFO(" ~RosMipHardwareInterface");
  // TODO make sure this is called
}



#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

static void __mpu_cbk(void)
{
  //ros::Time now = ros::Time::now();
  //std::cerr << " __mpu_cbk " << now << std::endl;
  _foo_hw_interf->IMUCallback();
}


/*******************************************************************************
 *
 *
 *******************************************************************************/
bool RosMipHardwareInterface::start() {

  // encoders
  if(rc_encoder_eqep_init()){
    ROS_ERROR("in RosMipHardwareInterface::start: failed to initialize eqep");
    return -1;
  }
  // motors
  if (rc_motor_init_freq(RC_MOTOR_DEFAULT_PWM_FREQ)) {
    ROS_ERROR("in RosMipHardwareInterface::start: failed to initialize motors");
    return -1;
  }
  // IMU
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
  conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
  conf.dmp_sample_rate = IMU_SAMPLE_RATE_HZ;
  conf.dmp_fetch_accel_gyro = true;
  conf.orient = ORIENTATION_Z_UP;
  if(rc_mpu_initialize_dmp(&rc_mpu_data_, conf)){
    ROS_ERROR("in HomereHardwareInterface::start: can't talk to IMU, all hope is lost\n");
    return false;
  }
  _foo_hw_interf = this;
  rc_mpu_set_dmp_callback(&__mpu_cbk);

  // start dsm listener
  // rc_initialize_dsm();
  // rc_set_dsm_data_func(&_dsm_callback, reinterpret_cast<void*>(this));

  rc_set_state(RUNNING);
  return true;
}


/*******************************************************************************
 *
 *
 *******************************************************************************/
bool RosMipHardwareInterface::shutdown() {
  ROS_INFO("in RosMipHardwareInterface::shutdown");
  rc_encoder_eqep_cleanup();
  rc_mpu_power_off();
  return true;
}


/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipHardwareInterface::read() {
  //ROS_INFO(" read HW");
  double left_wheel_angle = rc_encoder_read(ENCODER_CHANNEL_L) * 2 * M_PI / (ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);
  double right_wheel_angle = rc_encoder_read(ENCODER_CHANNEL_R) * 2 * M_PI / (ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);

  joint_velocity_[0] = (left_wheel_angle - joint_position_[0]) / IMU_DT;
  joint_velocity_[1] = (right_wheel_angle - joint_position_[1]) / IMU_DT;
  joint_position_[0] = left_wheel_angle;
  joint_position_[1] = right_wheel_angle;

  
  //ROS_INFO(" read HW %f %f %f %f", imu_orientation_[0], imu_orientation_[1], imu_orientation_[2], imu_orientation_[3]);
  
  // FIXME... where is the mutex ?
//   if(rc_is_new_dsm_data()){
//     turn_stick_  = rc_get_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
//     drive_stick_ = rc_get_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;
//     mode_switch_ = rc_get_dsm_ch_normalized(DSM_MODE_CH) * DSM_MODE_POL;
//     if(fabs(drive_stick_)<DSM_DEAD_ZONE) drive_stick_ = 0.0;
//     if(fabs(turn_stick_)<DSM_DEAD_ZONE)  turn_stick_  = 0.0;
//     dsm_ok_ = true;
//     //ROS_INFO(" read HW DSM  new data %f %f %x", turn_stick_, drive_stick_, this);
//   }
//   else if (rc_nanos_since_last_dsm_packet() > 0.1*1e9) { //rc_is_dsm_active()==0) {
//     //ROS_INFO(" read HW DSM not active");
//     dsm_ok_ = false;
//   }
  //ROS_INFO(" DSM %d", dsm_ok_);

}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipHardwareInterface::write() {
  //ROS_INFO(" write HW");
  float dutyL =  joint_effort_command_[0];
  float dutyR =  joint_effort_command_[1];
  //ROS_INFO(" write HW %f %f %f", joint_effort_command_[0], dutyL, dutyR);
  rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
  rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
void RosMipHardwareInterface::DSMCallback(void) {
  //std::cerr << "in RosMipHardwareInterface::DSMCallback" << std::endl;

  
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
#define _DEG2RAD(_D) _D/180.*M_PI
void RosMipHardwareInterface::IMUCallback(void) {
  imu_orientation_[0] = rc_mpu_data_.dmp_quat[1];
  imu_orientation_[1] = rc_mpu_data_.dmp_quat[2];
  imu_orientation_[2] = rc_mpu_data_.dmp_quat[3];
  imu_orientation_[3] = rc_mpu_data_.dmp_quat[0];

  imu_angular_velocity_[0] = _DEG2RAD(rc_mpu_data_.gyro[0]); // WTF are those units !!!
  imu_angular_velocity_[1] = _DEG2RAD(rc_mpu_data_.gyro[1]);
  imu_angular_velocity_[2] = _DEG2RAD(rc_mpu_data_.gyro[2]); 

  imu_linear_acceleration_[0] = rc_mpu_data_.accel[0];
  imu_linear_acceleration_[1] = rc_mpu_data_.accel[1];
  imu_linear_acceleration_[2] = rc_mpu_data_.accel[2];
}

#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, __NAME);
  ROS_INFO_STREAM_NAMED(__NAME, "ROSMIP Hardware node starting...");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  // look here : https://github.com/ROBOTIS-OP/robotis_op_ros_control/blob/master/src/robotis_op_hardware_interface.cpp
  double enc_mult;
  if (nh.getParam("enc_mult", enc_mult)) {
    ROS_INFO_STREAM_NAMED(__NAME, "  enc_mult " << enc_mult);
  }
  else {
    ROS_INFO_STREAM_NAMED(__NAME, "  enc_mult not found " << enc_mult);
  }
  return 0;
  
  RosMipHardwareInterface hw;
  if (!hw.start()) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Failed to initialize hardware. bailling out...");
    return -1;
  }


  controller_manager::ControllerManager cm(&hw, nh);
  ros::Duration period(IMU_DT);
  while (ros::ok() and rc_get_state()!=EXITING)
  {
    rc_mpu_block_until_dmp_data();
    hw.read();
    cm.update(ros::Time::now(), period);
    hw.write();
  }
  hw.shutdown();
  ROS_INFO_STREAM_NAMED(__NAME, "ROSMIP Hardware node exiting...");
  return 0;
}
