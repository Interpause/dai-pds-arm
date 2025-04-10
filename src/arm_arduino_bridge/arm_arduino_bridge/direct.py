"""Directly control servos."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

from .interface import N_SERVO


def main(args=None):
    rclpy.init(args=args)
    node = Node("direct_ctrl")

    pub = node.create_publisher(UInt16MultiArray, "cmd", 1)

    msg = UInt16MultiArray()
    msg.data = [0] * N_SERVO
    while True:
        servo = input(f"Select servo (1-{N_SERVO}): ")
        try:
            servo = int(servo)
            assert 1 <= servo <= N_SERVO
        except Exception:
            print(f"Invalid servo number: {servo}")
            continue
        value = input("Input pulsewidth (100-2900 or 0): ")
        try:
            value = int(value)
            assert 100 <= value <= 2900 or value == 0
        except Exception:
            print(f"Invalid pulsewidth: {value}")
            continue
        msg.data[servo - 1] = value

        print(f"Set servo {servo} to {value}.")
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        print(
            f"Current values:\n{' '.join(f'{i + 1}={s}' for i, s in enumerate(msg.data))}\n"
        )
