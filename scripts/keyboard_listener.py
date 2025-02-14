#!/usr/bin/env python

import os
import subprocess
import time

from pynput import keyboard

# import rospy
# from geometry_msgs.msg import TwistStamped, Twist

class Velocity:

    def __init__(self, x, y, w) -> None:
        self.x = 0.0
        self.y = 0.0
        self.w = 0.0


class KeyboardListener:

    def __init__(self, model_type='diff', max_v=0.5, max_w=0.5, min_v=0.1, min_w=0.1, velocity_step=0.1, linear_acceleration=0.5, angular_acceleration=0.5) -> None:
        
        self.model_type=model_type
        self.script_terminal_id = str(self.get_focused_window())
        self.velocity_cmd = Velocity(0.0, 0.0, 0.0)
        self.velocity = Velocity(0.0, 0.0, 0.0)
        self.linear_vel = max_v/2
        self.angular_vel = max_w/2
        epsilon = 0.01
        self.max_v = max_v - epsilon
        self.max_w = max_w - epsilon
        self.min_v = min_v + epsilon
        self.min_w = min_w + epsilon
        self.velocity_step = velocity_step

        self.linear_acceleration = linear_acceleration 
        self.angular_acceleration = angular_acceleration
        self.prev_time = time.time()

        # choose keyboard handling depending on robot type
        if model_type == 'omni':
            self.usage = self.omni_usage
        elif model_type == 'diff':
            self.usage = self.diff_usage
        else:
            print(f"\nThe model type {model_type} does not exists.\nchoose between [omni|diff].\n")
            exit(1)

        # print usage before starting the loop
        self.usage()

        # start keyboard listeer in non blocking fashion
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

    def get_focused_window(self):
        return subprocess.run(['xdotool', 'getwindowfocus'], capture_output=True).stdout.decode('utf-8').split()
    
    def omni_usage(self):
        print("\n-------------------------------------------------------------\n"
            "------              Omni Keyboard Teleop               ------\n"
            "-------------------------------------------------------------\n\n"
            "\t   Keyboard Keys\t |    Corresponding movements\n\n"
            "\tq\tw\te\t | \t\u2190\t\u2191\t\u2192\n\n"
            "\ta\ts\td\t | \t\u27F2\t\u2193\t\u27F3\n\n"
            "\t\t\tincrease/decrease speed\n\n"
            "\t\t\tlinear (v)\tangular (w)\n"
            "\tincrease:\tt \u2191\t\ty \u2191\n"
            "\tdecrease:\tg \u2193\t\th \u2193\n")


    def diff_usage(self):
        print("\n-------------------------------------------------------------\n"
            "------              Diff Keyboard Teleop               ------\n"
            "-------------------------------------------------------------\n\n"
            "\t   Keyboard Keys\t |    Corresponding movements\n\n"
            "\t\tw\t\t | \t\t\u2191\t\n\n"
            "\ta\ts\td\t | \t\u27F2\t\u2193\t\u27F3\n\n"
            "\t\t\tincrease/decrease speed\n\n"
            "\t\t\tlinear (v)\tangular (w)\n"
            "\tincrease:\tt \u2191\t\ty \u2191\n"
            "\tdecrease:\tg \u2193\t\th \u2193\n")
        
    def get_velocity(self):
        if self.model_type == 'diff':
            self.velocity_cmd.y = 0.0

        curr_time = time.time()
        delta_t = curr_time - self.prev_time
        self.prev_time = curr_time

        vx_diff = self.velocity.x - self.velocity_cmd.x
        vy_diff = self.velocity.y - self.velocity_cmd.y
        w_diff = self.velocity.w - self.velocity_cmd.w

        if abs(vx_diff) > 1e-5:
            if vx_diff < 0: # cmd_ref > curr_vel cmd_ref need acelleration 
                self.velocity.x += self.linear_acceleration * delta_t
                self.velocity.x = min(self.velocity.x, self.velocity_cmd.x)
            else: # curr_vel > cmd_ref need decelleration
                self.velocity.x -= self.linear_acceleration * delta_t
                self.velocity.x = max(self.velocity.x, self.velocity_cmd.x)

        if abs(vy_diff) > 1e-5:
            if vy_diff < 0: # cmd_ref > curr_vel cmd_ref need acelleration 
                self.velocity.y += self.linear_acceleration * delta_t
                self.velocity.y = min(self.velocity.y, self.velocity_cmd.y)
            else: # curr_vel > cmd_ref need decelleration
                self.velocity.y -= self.linear_acceleration * delta_t
                self.velocity.y = max(self.velocity.y, self.velocity_cmd.y)

        if abs(w_diff) > 1e-5:
            if w_diff < 0: # cmd_ref > curr_vel cmd_ref need acelleration 
                self.velocity.w += self.linear_acceleration * delta_t
                self.velocity.w = min(self.velocity.w, self.velocity_cmd.w)
            else: # curr_vel > cmd_ref need decelleration
                self.velocity.w -= self.linear_acceleration * delta_t
                self.velocity.w = max(self.velocity.w, self.velocity_cmd.w)

        return self.velocity


    def on_press(self, key):

        current_terminal_id = str(self.get_focused_window())

        if current_terminal_id == self.script_terminal_id:
            try:
                if key.char == 'w':
                    self.velocity_cmd.x = self.linear_vel
                elif key.char == 's':
                    self.velocity_cmd.x = -self.linear_vel
                elif key.char == 'd':
                    self.velocity_cmd.w = -self.angular_vel
                elif key.char == 'a':
                    self.velocity_cmd.w = self.angular_vel
                elif key.char == 'e':
                    self.velocity_cmd.y = -self.linear_vel
                elif key.char == 'q':
                    self.velocity_cmd.y = self.linear_vel
                elif key.char == 't':
                    if self.linear_vel < self.max_v:
                        self.linear_vel += self.velocity_step
                elif key.char == 'g':
                    if self.linear_vel > self.min_v:
                        self.linear_vel -= self.velocity_step
                elif key.char == 'y':
                    if self.angular_vel < self.max_w:
                        self.angular_vel += self.velocity_step
                elif key.char == 'h':
                    if self.angular_vel > self.min_w:
                        self.angular_vel -= self.velocity_step
            except AttributeError:
                pass


    def on_release(self, key):

        current_terminal_id = str(self.get_focused_window())

        if current_terminal_id == self.script_terminal_id:
            try:
                if key.char == 'w' or key.char == 's':
                    self.velocity_cmd.x = 0.0
                elif key.char == 'a' or key.char == 'd':
                    self.velocity_cmd.w = 0.0
                elif key.char == 'q' or key.char == 'e':
                    self.velocity_cmd.y = 0.0
            except AttributeError:
                pass
