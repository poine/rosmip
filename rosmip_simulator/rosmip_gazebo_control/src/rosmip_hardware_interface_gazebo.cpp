#include <rosmip_gazebo_control/rosmip_hardware_interface_gazebo.h>

#define __NAME "rosmip_hardware_interface_gazebo"

namespace rosmip_hardware_gazebo {

  const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint"};

  RosMipHardwareInterface::RosMipHardwareInterface():
    motors_on_(false)
  {
    this->registerInterface(static_cast<RosMipHardwareInterface *>(this));
    ROS_INFO_STREAM_NAMED("foo", "RosMipHardwareInterface::RosMipHardwareInterface");
    gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as state input for control" << std::endl;
    //ROS_INFO("bla");
  }

  bool RosMipHardwareInterface::initSim(
					const std::string& robot_namespace,
					ros::NodeHandle model_nh,
					gazebo::physics::ModelPtr parent_model,
					const urdf::Model *const urdf_model,
					std::vector<transmission_interface::TransmissionInfo> transmissions) {
    
    ROS_INFO_STREAM_NAMED( __NAME, "RosMipHardwareInterface::initSim");
    // store parent model pointer
    model_ = parent_model;
    link_ = model_->GetLink();
    for (int i=0; i<NB_JOINTS; i++) {
      gz_joints_[i] = parent_model->GetJoint(joint_name_[i]);
    }

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
    
    
    // TODO read that from urdf
    //tf::Quaternion q_imu_to_base;
    q_base_to_imu_.setRPY(M_PI/2, 0, M_PI/2); //q_imu_to_base.setRPY(0, 0, -M_PI/2);
    //q_base_to_imu_ = q_imu_to_base.inverse();
    
    return true;
  }


  void RosMipHardwareInterface::readSim(ros::Time time, ros::Duration period) {
    //ROS_INFO_STREAM_NAMED("foo", "RosMipHardwareInterface::readSim");
    gz_pose_             =  link_->GetWorldPose();
    //gz_acceleration_     = ((link_->GetWorldLinearVel() - gz_velocity_) + acceleration_time_constant * gz_acceleration_) / (period.toSec() + acceleration_time_constant);
    gz_velocity_         =  link_->GetWorldLinearVel();
    gz_angular_velocity_ =  link_->GetWorldAngularVel();
    //std::cerr << gz_pose_ << std::endl;

    for (int i=0; i<NB_JOINTS; i++) {
      joint_position_[i] = gz_joints_[i]->GetAngle(0).Radian();
      joint_velocity_[i] = gz_joints_[i]->GetVelocity(0);
    }
    //std::cerr <<  "read " << joint_position_[0] << " " <<  joint_position_[1] << std::endl;
    //
    tf::Quaternion q_world_to_base = tf::Quaternion(gz_pose_.rot.x, gz_pose_.rot.y, gz_pose_.rot.z, gz_pose_.rot.w); // x, y, z, w
    tf::Quaternion q_world_to_imu = q_world_to_base * q_base_to_imu_;
      
    imu_orientation_[0] = q_world_to_imu.x();
    imu_orientation_[1] = q_world_to_imu.y();
    imu_orientation_[2] = q_world_to_imu.z();
    imu_orientation_[3] = q_world_to_imu.w();
    //std::cerr <<  gz_pose_.rot.x << " " << gz_pose_.rot.y << " " << gz_pose_.rot.z << " " << gz_pose_.rot.w << std::endl;
    
    tf::Vector3 rvel_world(gz_angular_velocity_.x, gz_angular_velocity_.y, gz_angular_velocity_.z);
    tf::Vector3 rvel_imu = tf::quatRotate(q_world_to_imu, rvel_world);
    imu_angular_velocity_[0] = rvel_imu.x();
    imu_angular_velocity_[1] = rvel_imu.y();
    imu_angular_velocity_[2] = rvel_imu.z(); 
    
  }

  void RosMipHardwareInterface::writeSim(ros::Time time, ros::Duration period) {
    for (int i=0; i<NB_JOINTS; i++) {
      gz_joints_[i]->SetForce(0, joint_effort_command_[i]);
    }
    //std::cerr <<  "write " << joint_effort_command_[0] << " " <<  joint_effort_command_[1] << std::endl;
  }

  void RosMipHardwareInterface::switch_motors_on()  {
    ROS_INFO_STREAM_NAMED( __NAME, "RosMipHardwareInterface::witch_motors_on");
    motors_on_ = true;
  }

  void RosMipHardwareInterface::switch_motors_off() {
    ROS_INFO_STREAM_NAMED( __NAME, "RosMipHardwareInterface::witch_motors_off");
    motors_on_ = false;
  }

  
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rosmip_hardware_gazebo::RosMipHardwareInterface, gazebo_ros_control::RobotHWSim)

