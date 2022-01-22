#!/usr/bin/env python

import sys
import subprocess

try:
    from pynput import keyboard
except ModuleNotFoundError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pynput"])
    print("pynput python package succesfully installed. Relaunch the application.")

import rospy
from geometry_msgs.msg import TwistStamped

vx, vy, w = 0, 0, 0
lin_vel, ang_vel, zero_vel = 0.5, 0.5, 0.0


def usage():
    print("\n-------------------------------------------------------------------------\n"
          "------                         Keyboard Teleop                     ------\n"
          "-------------------------------------------------------------------------\n\n"
          "\t   Keyboard Keys\t |    Corresponding movements\n\n"
          "\tq\tw\te\t | \t\u27F2\t\u2191\t\u27F3\n\n"
          "\ta\ts\td\t | \t\u2190\t\u2193\t\u2192\n\n")


def on_press(key):
    global vx, vy, w, lin_vel, ang_vel, zero_vel
    try:
        if key.char == 'w':
            vx = lin_vel
        elif key.char == 's':
            vx = -lin_vel
        elif key.char == 'a':
            vy = lin_vel
        elif key.char == 'd':
            vy = -lin_vel
        elif key.char == 'e':
            w = -ang_vel
        elif key.char == 'q':
            w = ang_vel
        else:
            rospy.logwarn("Command {0} does NOT exists.".format(key.char))
    except AttributeError:
        rospy.logwarn("Command {0} does NOT exists.".format(key))


def on_release(key):
    global vx, vy, w, zero_vel
    try:
        if key.char == 'w' or key.char == 's':
            vx = zero_vel
        elif key.char == 'a' or  key.char == 'd':
            vy = zero_vel
        elif key.char == 'q' or key.char == 'e':
            w = zero_vel
    except AttributeError:
        pass


if __name__ == '__main__':

    rospy.init_node('keyboard_teleop_node')

    # initialize parameters
    cmd_vel_topic = rospy.get_param("~cmd_vel_topic_name")
    cmd_vel_father_frame_id = rospy.get_param("~cmd_vel_father_frame_id")
    lin_vel = rospy.get_param("~max_lin_vel", 0.5)
    ang_vel = rospy.get_param("~max_ang_vel", 0.5)

    # initialize variables
    seq = 0

    # cmd vel publisher
    pub = rospy.Publisher(cmd_vel_topic, TwistStamped, queue_size=10)

    # print usage before starting the loop
    usage()

    # start keyboard listener in non blocking fashion
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    # initialize msg, loop and key command
    key_cmd = ''
    rate = rospy.Rate(10)
    cmd_vel_msg = TwistStamped()
    cmd_vel_msg.header.frame_id = cmd_vel_father_frame_id

    while not rospy.is_shutdown():
        seq += 1
        
        # write cmd message
        cmd_vel_msg.header.seq = seq
        cmd_vel_msg.header.stamp = rospy.Time.now()
        cmd_vel_msg.twist.linear.x = vx
        cmd_vel_msg.twist.linear.y = vy
        cmd_vel_msg.twist.angular.z = w

        # publish command
        pub.publish(cmd_vel_msg)

        rate.sleep()
