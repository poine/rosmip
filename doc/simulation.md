
### Robot in gazebo (loads balance control)

    roslaunch rosmip_gazebo rosmip_world.launch config:=rosmip_2

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

### Movebase

    roslaunch rosmip_gazebo_demos move_base_demo.launch
       world:=maze_1
       world:=empty
       map_path:=`rospack find rosmip_worlds`/maps/enac_bench/track_test2.yaml
       map_path:=`rospack find rosmip_worlds`/maps/enac_bench/track_test1.yaml


    roslaunch rosmip_navigation move_base_demo.launch
      world:=empty
      map_path:=`rospack find rosmip_worlds`/maps/enac_bench/track_test2.yaml