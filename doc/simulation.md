
### Robot in gazebo (loads balance control)

    roslaunch rosmip_gazebo rosmip_world.launch config:=rosmip_2
    roslaunch rosmip_gazebo one_robot_in_z_room.launch robot_name:=rosmip_2

### Real time plot of balancing control

    rqt_multiplot --multiplot-config /home/poine/work/rosmip/rosmip/rosmip_control/config/rqt_multiplot/debug_ctl.xml

### open loop inputs (for testing control)

    rosrun two_d_guidance send_cmd_vel.py _cmd_topic:=/rosmip_balance_controller/cmd_vel _signal_type:=step_lin _amp:=0.1 _period:=6.

### Joystick teleop

    roslaunch rosmip_control  teleop.launch


### Pure pursuit (path following)

    roslaunch rosmip_navigation demo_pure_pursuit.launch vel_setpoint:=0.1

    ( roslaunch rosmip_gazebo_demos  pure_pursuit_demo.launch
       vel_setpoint:=0.1
       config:=rosmip_2
       path_filename:=`rospack find two_d_guidance`/paths/test/track_ethz_dual_01.npz
       map_path:=`rospack find rosmip_worlds`/maps/enac_bench/track_test2.yaml	
    )

    roslaunch rosmip_gazebo_demos  pure_pursuit_demo.launch vel_setpoint:=0.3 path_filename:=`rospack find two_d_guidance`/paths/test/track_ethz_dual_01.npz
    
paths/demo_z/track_ethz_cam1_cw.npz

### Movebase

    roslaunch rosmip_gazebo_demos move_base_demo.launch
       world:=maze_1
       world:=empty
       map_path:=`rospack find rosmip_worlds`/maps/enac_bench/track_test2.yaml
       map_path:=`rospack find rosmip_worlds`/maps/enac_bench/track_test1.yaml


    roslaunch rosmip_navigation move_base_demo.launch
      world:=empty
      map_path:=`rospack find rosmip_worlds`/maps/enac_bench/track_test2.yaml




### demo pp in gazebo z world

roslaunch rosmip_gazebo_demos one_robot_in_z_room.launch robot_name:=rosmip_1

roslaunch rosmip_navigation demo_pure_pursuit_z.launch vel_setpoint:=0.35

roslaunch rosmip_gazebo_demos two_robots_in_z_room_pp_demo.launch



roslaunch rosmip_gazebo rosmip_world.launch config:=rosmip_2 world_path:=`rospack find common_simulations`/worlds/ethz_cam1.world

roslaunch smocap_gazebo demo_gazebo_cfg_z1.launch start_gazebo:=false start_hog_marker:=false

rosrun smocap smocap_node.py _cameras:=camera_1 _detector_cfg_path:=`rospack find smocap`/params/gazebo_detector_cfg.yaml _run_mono_tracker:=false

roslaunch rosmip_navigation demo_pure_pursuit_z.launch vel_setpoint:=0.3

## demo pp in real z world

rosrun smocap smocap_node.py _cameras:=ueye_enac_z_1  _detector_cfg_path:=`rospack find smocap`/params/enac_demo_z/expe_z_detector_default.yaml _run_mono_tracker:=false
