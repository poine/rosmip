# rosmip
ROS Mobile Inverted Pendulum

This project is an attempt to create a ROS version of EduMip (https://github.com/StrawsonDesign/EduMiP), a lightweight beaglebone based mobile inverted pendulum.

ROSMIP is able to navigate using movebase. In the repository are the hardware interface and the balancing controller (adapted from roboticscape example) used to integrate MIP with ROS. Included is also a gazebo simulation model. Short term/relative positioning is done using odometry. For long term/absolute positioning, I am developing a simple camera-based tracker, [smocap](https://github.com/poine/smocap).

Here are a couple of pictures:

<table>
  <tr>
  <td>
  <!--<img src="https://lh6.googleusercontent.com/DSEkQQiF0ypbTMQ7l5DrfpPLy2A4iirdhgKhLdmJ5v4Sb4Gr642pNe3WT934QLtm2h1JE3UVCw6S0bl6Y7Dc6QRR661DffLWgbTWvfaUr9jKF8p85dtRxuecCK9L6kRa0AnjA4cJ" alt="ROSMIP">-->
  
  <img src="https://lh3.googleusercontent.com/aKiPeTlPN6_mH47hJkXS9eg2ZHjkmYzT6VvRs1q96znt_SHkby7lzjOcq-Z36NCd8id6SC8xKZznDmWMCB7SP7sqAUWnpblttWwyR9K31irsyiNWwl3SP3X3jFYS3NZ9p9o4lK74" alt="ROSMIP v0">
  
  </td>
   <td>
  <img src="https://lh5.googleusercontent.com/lLsFaULNafq_xSZIHIQDaCgG7rAhe9uWF0cf5RGCBcooUE2UIZ0QqHcDq8Zh8BJPmvsD2PZwMgRDvGhYxY_VR02I9mmkC4ktasUDZbTyqDQm_iTEGx7BmfIj46t2xGfzdgWX_r2f" alt="Rosmip in Gazebo"/>
  
  </td>
  <td>
  <img src="https://lh4.googleusercontent.com/f6W5d5N863J8zcQVsqtcejeghsrM6z3qzT1zjxXYmP8rfHo7yntBpBpu2NSi2f-kCVZwLvHN75lxNmHhsRb7QwUMudNrRu5VhCiJMWxcwAFef1QoXZFLcdLRGtXxyHQZA0oRsH1g" alt="Rosmip in rviz"/>
  
  </td>
  <td>
 <img src="https://lh3.googleusercontent.com/V3b4-MgdXX1VZfGjAvKBkgMRZC0d-7yVZ-FB5fDj9Y4dZe7TmqMJFOoXXE3t725P8RZD8bCkBzeAExx1-VRe339YVwPDJgZpHTflf5viiD288bFhiC9yDOu8xRMrRCqXhdqYtzR5" alt="ROSMIP v1 with IR markers">
  </td>
  </tr>
  <tr>
  <td>ROSMIP v0 Hardware</td> <td>ROSMIP in Gazebo</td> <td>ROSMIP in rviz</td> <td> Rosmip v1 with IR marker </td>
  </tr>
  <tr> 
  <td> 
  <a href="https://youtu.be/CxEfidW_15c"><img src="https://lh5.googleusercontent.com/rYDiZ9pE-Kq6ll2QeSQS8jVsgAFPFzVDHDFqjsIwQXdOyfZ_K8Nb5sLM3Tk7jl208omNAKqZ-vUjqjbOrXNrYlpatG2JMKxpoxQcGsf6U3Xd_7WY1N9Xs5WruEdimSj53lcon-TP"/></a>
</td>
  </tr>
  <tr> 
  <td> ROSMIP following a curvy track under the authority of a pure pursuit controller</td>
  </tr>
  </table>


* [Getting started](doc/getting_started.md)
* [Hardware](doc/hardware.md)

