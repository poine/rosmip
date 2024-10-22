


# hardware

  rosmip1: 
     - motors: https://www.robotshop.com/en/6v-pololu-751-micro-metal-gearmotor-extended-motor-shaft.html
     - encoders: https://www.robotshop.com/en/12-cpr-magnetic-encoder-pair-kit-micro-metal-hpcb-compatible.html
     - wheels: https://www.robotshop.com/en/pololu-wheel-60-8mm-black-pair.html

  rosmip2: 
     motors+enc: https://www.robotshop.com/en/6v-1001-155rpm-micro-metal-gearmotor-encoder.html
  
  in box:
     -https://www.robotshop.com/eu/en/6v-pololu-1001-micro-metal-gearmotor-extended-motor-shaft.html
 



# 3D printed frame

   * motors are slidings in and out, block them
   * we could use conical head screws and have the inprint of the screw printed
   * make hooks for wires
   * secure battery better


## Robot Installation

### OS
  - install ubuntu (not debian) if you want ROS binary packages (you do, believe me, you do NOT want to build ROS from sources on the beaglebone)
    - [enable wifi] (https://www.digikey.com/en/maker/blogs/2017/how-to-setup-wifi-on-the-beaglebone-black-wireless)
    - Note: on february 12 2019, don't upgrade as it will break network and you'll need a serial console
    - enlarge parition to use all sdcard before installing ROS (sudo /opt/scripts/tools/grow_partition.sh)
    - [add swap](https://sheldondwill.wordpress.com/2013/12/14/beaglebone-black-ubuntu-adding-a-swapfile/)
    
    - when booting from sd card, led 2 blinks, whereas when booting from internal emmc, led3 blinks

### ROS

[wiki](http://wiki.ros.org/Installation/UbuntuARM)

  - sudo apt install ros-melodic-ros-base
  

### librobotcontrol

  - https://github.com/StrawsonDesign/librobotcontrol
  - clone / install script
  - test with rc_xxx

  -remove old bootloader in emmc: sudo dd if=/dev/zero of=/dev/mmcblk1 bs=1M count=10
  see https://github.com/StrawsonDesign/librobotcontrol/issues/123
  edit uEnv to change overlay for remote proc pru - pru stuff need to run root
  

### rosmip firmware

  - mkdir ~/work/overlay_ws
  - cd ~/work/overlay_ws
  - wstool init src
  - cd src
  - git clone https://github.com/poine/rosmip.git
  - git clone https://github.com/poine/smocap.git (FIXME, find a way to remove that dependency)
  - cd ..
  - catkin make

  - sudo apt-get install ros-melodic-gazebo-ros-control (FIXME, robot should not need that)
  - sudo apt-get install ros-melodic-xacro

#### testing:

  roslaunch rosmip_bringup robot.launch

  roslaunch rosmip_viz view_robot.launch


### Auto starting on boot
I use [robot upstart](http://docs.ros.org/jade/api/robot_upstart/html/index.html)

rosrun robot_upstart install --job rosmip --user poine rosmip_bringup/launch/robot.launch --logdir /home/poine/.ros/log

Fix permissions:

sudo vi /usr/sbin/rosmip-start

# Punch it.
#setuidgid poine roslaunch $LAUNCH_FILENAME &
sudo -u poine /home/poine/start_rosmip.sh &
PID=$!



## Ground Station Installation

### tensorflow/keras
  sudo apt install virtualenv
  virtualenv --system-site-packages -p python2 ./venv_tf_p2/
  source venv_tf_p2/bin/activate
  pip install --upgrade tensorflow
  pip install --upgrade keras
  pip install --upgrade control





 rqt_multiplot --multiplot-config /home/poine/work/rosmip/rosmip/rosmip_control/config/rqt_multiplot/debug_ctl.xml
