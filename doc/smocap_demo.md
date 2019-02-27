roslaunch rosmip_gazebo rosmip_world.launch config:=rosmip_4


roslaunch smocap_gazebo  single_cam.launch pos_x:=0. pos_y:=0. pos_z:=3. rot_P:=1.5707963267948966 camera_w:=1280 camera_h:=1024


rosrun smocap smocap_aruco_node.py _cameras:=camera_1 _marker_size:=0.05


roslaunch rosmip_navigation robot_localization_ekf.launch config:=ekf_smocap
