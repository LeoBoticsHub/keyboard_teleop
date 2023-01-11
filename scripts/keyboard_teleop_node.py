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

def get_focused_window():
    return subprocess.run(['xdotool', 'getwindowfocus'], capture_output=True).stdout.decode('utf-8').split()

def omni_usage():
    print("\n-------------------------------------------------------------\n"
          "------              Omni Keyboard Teleop               ------\n"
          "-------------------------------------------------------------\n\n"
          "\t   Keyboard Keys\t |    Corresponding movements\n\n"
          "\tq\tw\te\t | \t\u27F2\t\u2191\t\u27F3\n\n"
          "\ta\ts\td\t | \t\u2190\t\u2193\t\u2192\n\n"
          "\t\t\tincrease/decrease speed\n\n"
          "\t\t\tlinear (v)\tangular (w)\n"
          "\tincrease:\tt \u2191\t\ty \u2191\n"
          "\tdecrease:\tg \u2193\t\th \u2193\n")


def diff_usage():
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


def print_key_warn(key):
    global err_command_number
    print("\n\n")
    usage()
    print()
    rospy.logwarn("\b [{0}] Command {1} does NOT exists.\n".format(err_command_number, key))
    err_command_number += 1


def diff_on_press(key):
    global vx, w, lin_vel, ang_vel, zero_vel, usage
    try:
        if key.char == 'w':
            vx = lin_vel
        elif key.char == 's':
            vx = -lin_vel
        elif key.char == 'd':
            w = -ang_vel
        elif key.char == 'a':
            w = ang_vel
        elif key.char == 't':
            if lin_vel < max_v-0.01:
                lin_vel += 0.1
        elif key.char == 'g':
            if lin_vel > 0.11:
                lin_vel -= 0.1
        elif key.char == 'y':
            if ang_vel < max_w-0.01:
                ang_vel += 0.1
        elif key.char == 'h':
            if ang_vel > 0.11:
                ang_vel -= 0.1
        else:
            print_key_warn(key)
    except AttributeError:
        print_key_warn(key)


def diff_on_release(key):
    global vx, w, zero_vel
    try:
        if key.char == 'w' or key.char == 's':
            vx = zero_vel
        elif key.char == 'a' or key.char == 'd':
            w = zero_vel
    except AttributeError:
        pass


def omni_on_press(key):
    global vx, vy, w, lin_vel, ang_vel, zero_vel, usage
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
        elif key.char == 't':
            if lin_vel < max_v-0.01:
                lin_vel += 0.1
        elif key.char == 'g':
            if lin_vel > 0.11:
                lin_vel -= 0.1
        elif key.char == 'y':
            if ang_vel < max_w-0.01:
                ang_vel += 0.1
        elif key.char == 'h':
            if ang_vel > 0.11:
                ang_vel -= 0.1
        else:
            print_key_warn(key)
    except AttributeError:
        print_key_warn(key)


def omni_on_release(key):
    global vx, vy, w, zero_vel
    try:
        if key.char == 'w' or key.char == 's':
            vx = zero_vel
        elif key.char == 'a' or key.char == 'd':
            vy = zero_vel
        elif key.char == 'q' or key.char == 'e':
            w = zero_vel
    except AttributeError:
        pass


if __name__ == '__main__':

    script_terminal_id = str(get_focused_window())

    # init ros node
    rospy.init_node('keyboard_teleop_node')

    # declare globals
    global usage
    global lin_vel, ang_vel, zero_vel
    global max_v, max_w, err_command_number

    # initialize parameters
    model_type = rospy.get_param('~model_type', "omni")
    cmd_vel_topic = rospy.get_param("~cmd_vel_topic_name")
    cmd_vel_father_frame_id = rospy.get_param("~cmd_vel_father_frame_id")
    max_v = rospy.get_param("~max_lin_vel", 1)
    max_w = rospy.get_param("~max_ang_vel", 1)
    terminal_focus = rospy.get_param("~terminal_focus", True)

    # initialize variables
    seq = 0
    err_command_number = 0
    lin_vel, ang_vel, zero_vel = max_v/2, max_w/2, 0.0

    # cmd vel publisher
    pub = rospy.Publisher(cmd_vel_topic, TwistStamped, queue_size=10)

    # choose keyboard handling depending on robot type
    if model_type == 'omni':
        on_press = omni_on_press
        on_release = omni_on_release
        usage = omni_usage
    elif model_type == 'diff':
        on_press = diff_on_press
        on_release = diff_on_release
        usage = diff_usage
    else:
        print()
        rospy.logerr("The model type \'{0}\' does not exists.\nchoose between [omni|diff].\n".format(model_type))
        exit(1)

    # print usage before starting the loop
    usage()

    # start keyboard listeer in non blocking fashion
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    # initialize msg, loop and key command
    key_cmd = ''
    rate = rospy.Rate(300)
    cmd_vel_msg = TwistStamped()
    cmd_vel_msg.header.frame_id = cmd_vel_father_frame_id

    while not rospy.is_shutdown():
        seq += 1

        print("linear velocity: %.1f, angular velocity: %.1f     " % (lin_vel, ang_vel), end='\r')

        # write cmd message
        cmd_vel_msg.header.seq = seq
        cmd_vel_msg.header.stamp = rospy.Time.now()
        cmd_vel_msg.twist.linear.x = vx
        cmd_vel_msg.twist.linear.y = vy
        cmd_vel_msg.twist.angular.z = w
        
        current_terminal_id = str(get_focused_window())

        if not terminal_focus:
            pub.publish(cmd_vel_msg)
        elif current_terminal_id == script_terminal_id:
            pub.publish(cmd_vel_msg)

        rate.sleep()
