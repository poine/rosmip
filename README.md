# rosmip
ROS Mobile Inverted Pendulum

This project is an attempt to create a ROS version of EduMip (https://github.com/StrawsonDesign/EduMiP), a lightweight beaglebone based mobile inverted pendulum.

ROSMIP is able to navigate using movebase. In the repository are the hardware interface and the balancing controller (adapted from roboticscape example) used to integrate MIP with ROS. Included is also a gazebo simulation model. Short term/relative positioning is done using odometry. For long term/absolute positioning, I am developing a simple camera-based tracker, [smocap](https://github.com/poine/smocap).

Here are a couple of pictures:

<table>
  <tr>
  <td>
  <!--<img src="https://lh6.googleusercontent.com/DSEkQQiF0ypbTMQ7l5DrfpPLy2A4iirdhgKhLdmJ5v4Sb4Gr642pNe3WT934QLtm2h1JE3UVCw6S0bl6Y7Dc6QRR661DffLWgbTWvfaUr9jKF8p85dtRxuecCK9L6kRa0AnjA4cJ" alt="ROSMIP">-->
  
  <img src="https://lh3.googleusercontent.com/ojdLWzGg4OlEdW_GGai2b5M78x69r1Jr67Ac8sshQ6FOIVIj39bP5_IDQo6y7e8oJ4V0nF7Z_EXtg81voRG5y7bd6-dhk4Kq_tYacEXYx6jR_P5Kw-T3kx8H5xReItsqKE4OKsZ6h5XWLETG8-p8sBHcxdZNpsqWB4SRtLnGTzxXXp8bnyqL102RjNhj6fr457N-lLE11GI8ifCfZjNaw9d8yB2iuq9U5GFo56VzkEJrpQJkrVKnBG2oAXVV6nPQzgKNnoF_CaRQ5k1FPNYupcKYFtn47eQOoXZRXAzzYNgNFoBTmDUg9581iWnOYTpRLKa1_2IKlO-4RImS80X0Ne2svcV1IzTEdyVdyxApdu8usttFIzh83P-u2l2z2DRlJjbrxKkD-M1k1eaH5zwoMt6VH7Ui59JyYUQVc8Qm0MP-e4_6wsyjMPyaz0eI2k1H60sRG_eWSd_33Hlrjtrh8LoF4jliopmJdukHarJJyumb-CiOsUJ39dfyke8Y5Bky-JnPnKk5KwUpf9idDpVQidJhsm9IhGwlyGA4NkbwMWzcFa-aR4UATQaVam6utAMiVhdaT9KFxjZR5ZwGWOmsv93PYbqgsXp07luY050c-Kc=w939-h1199-no" alt="ROSMIP">
  
  </td>
   <td>
  <img src="https://lh5.googleusercontent.com/lLsFaULNafq_xSZIHIQDaCgG7rAhe9uWF0cf5RGCBcooUE2UIZ0QqHcDq8Zh8BJPmvsD2PZwMgRDvGhYxY_VR02I9mmkC4ktasUDZbTyqDQm_iTEGx7BmfIj46t2xGfzdgWX_r2f" alt="Rosmip in Gazebo"/>
  
  </td>
  <td>
  <img src="https://lh4.googleusercontent.com/f6W5d5N863J8zcQVsqtcejeghsrM6z3qzT1zjxXYmP8rfHo7yntBpBpu2NSi2f-kCVZwLvHN75lxNmHhsRb7QwUMudNrRu5VhCiJMWxcwAFef1QoXZFLcdLRGtXxyHQZA0oRsH1g" alt="Rosmip in rviz"/>
  
  </td>
  <td>
  <img src="https://lh3.googleusercontent.com/COWQ_6q6h7Rh0wXhdVP5C7y8bAPbKxnDrzeqR6t998mR4fqHHeUQ1q9st0E46-gQ6Y5W0FgNYn9_9FUtxP_6zjoz5G1PjyLnBJRNUw7zc1kSGatyzEqvB9QJ6m8KkFxVXRslHGkrmu1T_Zm46ljH44bJEhKUy-UPsecPunq1-1mTn1ljlK8tg3C2oCCPy2TE1R60ldl9s0CnM4tOOkrHtb5nHG0Ey8Fvy95aIbQ-nMoHpn1rSOHD_uCgGsGT3w89ce77sw5Uvcc5GxjOrioFE6IoNViJGDlDH8xyyKFI_lvvOqqcTfUnidWrun5fszTX8grSpEVim8DAIwXc10_RDKGZ-JzrM7eqYZLavmUzjfYA08c3YXxmJ1FHXMIYgD_XROaQhMgt32iSI47t8DvKsLwkCyAe75zqJh_kSD5ltCXhojI_dPlwEXljMi5_LROhr_zy7WJp7HppZkMcyAW4A3sw9chpoD7OenXRdmuiLUvmL7zv1PFM5KlgVGOqVoOKjYh4KM5YqTpJ_XH5Lg6zo6eLStqF6emL17AW8YMLpKdNlBHLKxPPuBhI_U2NvpBjoVHRNMuxM5iJJD36_i3UFeE2l7l1V7PZaHYdD9B2iBc=w587-h782-no" alt="Rosmip with IR marker" />
  </td>
  </tr>
  <tr>
  <td>ROSMIP Hardware</td> <td>ROSMIP in Gazebo</td> <td>ROSMIP in rviz</td> <td> Rosmip v1 with IR marker </td>
  </tr>
  </table>


* [Getting started](doc/getting_started.md)
* [Hardware](doc/hardware.md)

