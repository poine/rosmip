roslaunch rosmip_gazebo rosmip_world.launch config:=rosmip_2


roslaunch smocap_gazebo  single_cam.launch pos_x:=0. pos_y:=0. pos_z:=3. rot_P:=1.5707963267948966 camera_w:=1280 camera_h:=1024


rosrun smocap smocap_node.py _cameras:=camera_1 _detector_cfg_path:=`rospack find smocap`/params/gazebo_detector_cfg.yaml _run_mono_tracker:=tr _trap_losses:=false _height_above_floor:=0.09



rosrun smocap smocap_aruco_node.py _cameras:=camera_1 _marker_size:=0.05


roslaunch rosmip_navigation robot_localization_ekf.launch config:=ekf_smocap


## infrared in gazebo

  * gazebo and rosmip

roslaunch rosmip_gazebo rosmip_world.launch config:=rosmip_2

  * camera as in demo_z room

roslaunch smocap_gazebo demo_gazebo_cfg_z1.launch start_gazebo:=false start_hog_marker:=false

  * smocap

rosrun smocap smocap_node.py _cameras:=camera_1 _detector_cfg_path:=`rospack find smocap`/params/gazebo_detector_cfg.yaml _run_mono_tracker:=false

  * rviz -d /home/poine/work/rosmip/rosmip_desktop/rosmip_viz/rviz/pure_pursuit.rviz

  * roslaunch rosmip_navigation robot_localization_ekf.launch  config:=ekf_smocap
  * rosrun tf static_transform_publisher 0 0 0  0 0 0 world map 100