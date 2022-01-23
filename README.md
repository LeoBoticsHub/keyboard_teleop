# ros_keyboard_teleop
This ros package allow to drive a robot through a /cmd_vel topic using keyboard keys. The commands can be given without any needs to be focused on the teleop terminal, i.e., you can use the commands directly on simulation or anywhere else.

## usage
change the ```config/keyboard_teleop_params.yaml``` file to set the following parameters based on your robot specs:
- twist velocity command topic (cmd_vel_topic_name)
- father frame id (cmd_vel_father_frame_id)
- max linear velocity (max_lin_vel)
- max angular velocity (max_ang_vel)


### omni-directional robot control
   ```
   roslaunch ros_keyboard_teleop keyboard_teleop.launch model_type:=omni
   ```
   <img src="https://user-images.githubusercontent.com/76060218/150688007-8ea3e5bf-0c3e-414a-be70-5a2039f66586.png" alt="drawing" width="500"/>

### differential drive robot control
   ```
   roslaunch ros_keyboard_teleop keyboard_teleop.launch model_type:=diff
   ```
   <img src="https://user-images.githubusercontent.com/76060218/150687855-4f9d8374-9459-495a-9d13-bb09c521ed99.png" alt="drawing" width="500"/>
