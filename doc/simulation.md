

roslaunch rosmip_gazebo rosmip_world.launch config:=rosmip_2

rqt_multiplot --multiplot-config /home/poine/work/rosmip/rosmip/rosmip_control/config/rqt_multiplot/debug_ctl.xml

roslaunch rosmip_control  teleop.launch

rosrun homere_control send_cmd_vel.py _cmd_topic:=/rosmip_balance_controller/cmd_vel _signal_type:=step_lin _amp:=0.1 _period:=6.



 ./pp_guidance_node.py _path_filename:=/home/poine/work/oscar/oscar/oscar_control/paths/track_ethz_dual_01.npz  _twist_cmd_topic:=/rosmip_balance_controller/cmd_vel