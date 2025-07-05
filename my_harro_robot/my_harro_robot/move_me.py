#!/usr/bin/env python3

import sys
import termios
import tty
import threading

import rclpy
from geometry_msgs.msg import Twist


INSTRUCTIONS = """
    Arrow keys to move:
        ↑ : forward
        ↓ : backward
        ← : turn left
        → : turn right
    Space bar : stop
    CTRL-C     : quit
    R : rotate continuously
"""

MOVE_BINDINGS = {
    '\x1b[A': ( 1.0,  0.0),
    '\x1b[B': (-1.0,  0.0),
    '\x1b[D': ( 0.0,  1.0),
    '\x1b[C': ( 0.0, -1.0),
    ' ':      ( 0.0,  0.0),
    'r':      ( 0.0,  3.0),  # Rotate in place
    'R':      ( 0.0,  3.0)   # Rotate in place
}


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':            # possible arrow
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch1 + ch2 + ch3
        else:
            return ch1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def teleop_node():
    rclpy.init()
    node = rclpy.create_node('my_bot_mover')
    pub  = node.create_publisher(Twist, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        print(INSTRUCTIONS)
        while True:
            key = get_key()

            if key == '\x03':
                break

            # Look up motion; default to stopping
            linear_x, angular_z = MOVE_BINDINGS.get(key, (0.0, 0.0))

            twist = Twist()
            twist.linear.x  = linear_x * 3.0   # scale forward/backward
            twist.angular.z = angular_z * 1.0  # scale rotation
            pub.publish(twist)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # stop the robot before exit
        twist = Twist()
        pub.publish(twist)
        rclpy.shutdown()


if __name__ == '__main__':
    teleop_node()