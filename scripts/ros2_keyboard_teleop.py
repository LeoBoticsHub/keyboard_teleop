#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped

from keyboard_listener import KeyboardListener, Velocity

class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        # declare params
        self.declare_parameter('model_type', 'diff')
        self.declare_parameter('max_v', 0.5)
        self.declare_parameter('max_w', 0.5)
        self.declare_parameter('min_v', 0.1)
        self.declare_parameter('min_w', 0.1)
        self.declare_parameter('velocity_step', 0.1)
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('cmd_vel_father_frame_id', '')
        self.declare_parameter('use_twist_stamped', False)

        # read params
        model_type = self.get_parameter('model_type').get_parameter_value().string_value
        max_v = self.get_parameter('max_v').get_parameter_value().double_value
        max_w = self.get_parameter('max_w').get_parameter_value().double_value
        min_v = self.get_parameter('min_v').get_parameter_value().double_value
        min_w = self.get_parameter('min_w').get_parameter_value().double_value
        velocity_step = self.get_parameter('velocity_step').get_parameter_value().double_value

        self.use_twist_stamped = self.get_parameter('use_twist_stamped').get_parameter_value().bool_value

        # Select publisher
        cmd_type = Twist
        self.cmd_vel_msg = Twist()
        if self.use_twist_stamped:
            cmd_type = TwistStamped
            self.cmd_vel_msg = TwistStamped()
            self.cmd_vel_msg.header.frame_id = self.get_parameter('cmd_vel_father_frame_id').get_parameter_value().string_value
            
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        
        self.cmd_vel_pub = self.create_publisher(cmd_type, cmd_vel_topic, 10)

        # create Keyboard listener
        self.listener = KeyboardListener(
            model_type=model_type, max_v=max_v, max_w=max_w, 
            min_v=min_v, min_w=min_w, velocity_step=velocity_step
        )

        # Set the publish velocity timer
        self.timer = self.create_timer(0.1, self.publish_vel)

    def publish_vel(self):
        vel: Velocity
        vel = self.listener.get_velocity()

        self.cmd_vel_msg.linear.x = vel.x
        self.cmd_vel_msg.linear.y = vel.y
        self.cmd_vel_msg.angular.z = vel.w

        if self.use_twist_stamped:
            self.cmd_vel_msg.header.stamp = self.get_clock().now()

        self.get_logger().info(f"\033[1A\rVelocity [Vx, Vy, w]: [{vel.x}, {vel.y}, {vel.w}]        ")
        
        self.cmd_vel_pub.publish(self.cmd_vel_msg)



if __name__ == "__main__":

    rclpy.init()

    keyboard_node = KeyboardTeleopNode()

    rclpy.spin(keyboard_node)

    keyboard_node.destroy_node()
    rclpy.shutdown()
