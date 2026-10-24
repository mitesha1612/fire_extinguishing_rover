#!/usr/bin/env python3

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


TOPIC = '/diff_drive_controller/cmd_vel'
LINEAR_SPEED = 0.25
ANGULAR_SPEED = 0.8
KEY_TIMEOUT = 0.25
PUBLISH_PERIOD = 0.05

ARROW_KEYS = {
    b'[A': 'up',
    b'[B': 'down',
    b'[C': 'right',
    b'[D': 'left',
    b'OA': 'up',
    b'OB': 'down',
    b'OC': 'right',
    b'OD': 'left',
}


def read_key(fd):
    key = os.read(fd, 1)
    if key != b'\x1b':
        return key

    sequence = b''
    for _ in range(2):
        ready, _, _ = select.select([fd], [], [], 0.03)
        if not ready:
            return None
        sequence += os.read(fd, 1)
    return ARROW_KEYS.get(sequence)


def publish_velocity(node, publisher, forward, rotation):
    message = TwistStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.twist.linear.x = forward * LINEAR_SPEED
    message.twist.angular.z = rotation * ANGULAR_SPEED
    publisher.publish(message)


def main():
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        raise SystemExit('Arrow-key teleop requires an interactive terminal.')

    terminal_settings = termios.tcgetattr(fd)
    rclpy.init()
    node = rclpy.create_node('fire_rover_arrow_teleop')
    publisher = node.create_publisher(TwistStamped, TOPIC, 10)
    spray_publisher = node.create_publisher(Bool, '/fire/water_spray', 10)
    forward = 0
    rotation = 0
    last_arrow_time = 0.0
    water_spraying = False

    try:
        tty.setraw(fd)
        print('Arrow keys move. Press Space to spray; another key stops the spray. Ctrl+C quits.')
        while rclpy.ok():
            ready, _, _ = select.select([fd], [], [], PUBLISH_PERIOD)
            if ready:
                key = read_key(fd)
                if key == b'\x03':
                    break
                direction = key if key in ('up', 'down', 'left', 'right') else None
                if direction == 'up':
                    forward, rotation = 1, 0
                    water_spraying = False
                elif direction == 'down':
                    forward, rotation = -1, 0
                    water_spraying = False
                elif direction == 'left':
                    forward, rotation = 0, 1
                    water_spraying = False
                elif direction == 'right':
                    forward, rotation = 0, -1
                    water_spraying = False
                elif key == b' ':
                    forward, rotation = 0, 0
                    water_spraying = True
                else:
                    forward, rotation = 0, 0
                    water_spraying = False
                last_arrow_time = time.monotonic()

            if time.monotonic() - last_arrow_time > KEY_TIMEOUT:
                forward, rotation = 0, 0

            publish_velocity(node, publisher, forward, rotation)
            spray_message = Bool()
            spray_message.data = water_spraying
            spray_publisher.publish(spray_message)
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        publish_velocity(node, publisher, 0, 0)
        spray_message = Bool()
        spray_message.data = False
        spray_publisher.publish(spray_message)
        termios.tcsetattr(fd, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
