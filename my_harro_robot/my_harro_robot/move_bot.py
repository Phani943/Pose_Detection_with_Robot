#!/usr/bin/env python3

import time
import rclpy
from geometry_msgs.msg import Twist
from threading import Thread, Event
from pynput import keyboard

INSTRUCTIONS = """
    Arrow keys to move:
        ↑ : forward
        ↓ : backward
        ← : turn left
        → : turn right
    Space bar : stop
    r : continuous rotate
    s : toggle slow mode
    q : quit
"""

# Mapping for key actions
MOVE_BINDINGS = {
    'up':    ( 1.0,  0.0),
    'down':  (-1.0, 0.0),
    'left':  (0.0,  1.0),
    'right': (0.0, -1.0),
    'space': (0.0,  0.0),
}

class TeleopController:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('teleop_better')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = 0.8
        self.rotate_mode = False
        self.slow_mode = False
        self.stop_event = Event()

        self.spin_thread = Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        self.control_thread = Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def control_loop(self):
        """Continuously publish based on current mode"""
        while not self.stop_event.is_set():
            twist = Twist()
            if self.rotate_mode:
                twist.angular.z = 1.0 if not self.slow_mode else 0.3
            self.pub.publish(twist)
            time.sleep(0.1)  # Publish at 10 Hz

    def stop_robot(self):
        twist = Twist()
        self.pub.publish(twist)

    def handle_key(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                name = key.char.lower()
            else:
                name = key.name  # e.g., 'up', 'down', 'space'
        except:
            return

        if name == 'q':
            self.stop_event.set()
            return False  # stop the listener

        elif name == 'r':
            print("🔄 Rotate mode ON")
            self.rotate_mode = True

        elif name == 's':
            self.slow_mode = not self.slow_mode
            print("🐢 Slow mode:", self.slow_mode)

        elif name in MOVE_BINDINGS:
            lin, ang = MOVE_BINDINGS[name]
            if self.slow_mode:
                lin *= 0.6
                ang *= 0.6
            twist = Twist()
            twist.linear.x = lin * self.speed
            twist.angular.z = ang
            self.pub.publish(twist)
            self.rotate_mode = False

        elif name == 'space':
            self.stop_robot()
            self.rotate_mode = False


    def start(self):
        print(INSTRUCTIONS)
        with keyboard.Listener(on_press=self.handle_key) as listener:
            listener.join()

        # On exit
        self.stop_robot()
        rclpy.shutdown()


def main():
    TeleopController().start()

if __name__ == '__main__':
    main()
