"""Uses http to communicate with Arduino to send & retrieve sensor_msgs.JointState."""

from dataclasses import dataclass
from urllib.parse import urljoin

import numpy as np
import rclpy
import requests
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class Servo:
    """Servo calibration settings."""

    min_pulse: int = 500
    max_pulse: int = 2500
    min_angle: float = 0.0
    max_angle: float = 180.0

    def us2ang(self, pulse: int) -> float:
        """Convert microseconds to angle."""
        return np.interp(
            pulse, (self.min_pulse, self.max_pulse), (self.min_angle, self.max_angle)
        )

    def ang2us(self, ang: float) -> int:
        """Convert angle to microseconds."""
        # NOTE: We don't wrap angles here, so ensure the digital robot model is accurate!
        return int(
            np.interp(
                ang, (self.min_angle, self.max_angle), (self.min_pulse, self.max_pulse)
            )
        )


DEFAULT_URL = "http://192.168.242.140/"

# Actually, servos are open loop and can't feedback position.
UPDATE_RATE = 50

# TODO: Don't hardcode.
JOINT2SERVO = {
    "base_revolute": "1_servo",
    "double_revolute_1": "2_servo",
    "double_spin": "3_servo",
    "double_revolute_2": "4_servo",
    "middle_revolute": "5_servo",
    "middle_spin": "6_servo",
    "end_revolute_1": "7_servo",
    "end_revolute_2": "8_servo",
    "effector_spin": "9_servo",
}
SERVO2JOINT = {v: k for k, v in JOINT2SERVO.items()}

# Values were manually calibrated. Angle ranges should be from digital robot model.
# TODO: Only 1_servo has been calibrated, tape label the rest & calibrate.
SERVOS = {
    "1_servo": Servo(450, 2400, 0.0, 180.0),
    "2_servo": Servo(500, 2500, -180.0, 0.0),
    "3_servo": Servo(500, 2500, -90.0, 90.0),
    "4_servo": Servo(500, 2500, 0.0, 180.0),
    "5_servo": Servo(500, 2500, -180.0, 0.0),
    "6_servo": Servo(500, 2500, -90.0, 90.0),
    "7_servo": Servo(500, 2500, -90.0, 90.0),
    "8_servo": Servo(500, 2500, 0.0, 180.0),
    "9_servo": Servo(500, 2500),
}


class Bridge(Node):
    """Bridge between ROS2 and Arduino over http."""

    def __init__(self):
        """Initialize."""
        super(Bridge, self).__init__("arm_arduino_bridge")

        self.declare_parameter("arduino_url", DEFAULT_URL)

        self.sess = requests.Session()

        self.state_pub = self.create_publisher(JointState, "robot_joint_states", 3)

        self.create_subscription(JointState, "robot_joint_commands", self._cb_cmd, 3)
        self.create_timer(1 / UPDATE_RATE, self._update_state)

    @property
    def url(self):
        """Get URL."""
        return self.get_parameter("arduino_url").get_parameter_value().string_value

    def _cb_cmd(self, msg: JointState):
        """Execute joint state."""
        params = {}
        params_deg = {}
        for i, joint in enumerate(msg.name):
            servo = JOINT2SERVO.get(joint, None)
            if servo is None:
                self.get_logger().warn(f"Unknown joint: {joint}", once=True)
                continue
            deg = np.rad2deg(msg.position[i])
            params_deg[servo] = deg
            params[servo] = SERVOS[servo].ang2us(deg)
        try:
            self.get_logger().info(
                f"Cmd Received:\n{params}\n{params_deg}", throttle_duration_sec=3
            )
            self.sess.get(urljoin(self.url, "write"), params=params)
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            return

    def _update_state(self):
        """Update joint state."""
        try:
            resp = self.sess.get(urljoin(self.url, "read"))
            resp.raise_for_status()
            arr = resp.json()
            assert isinstance(arr, list)
        except Exception as e:
            self.get_logger().error(f"Failed to read state: {e}")
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        for i, pulse in enumerate(arr):
            servo = f"{i + 1}_servo"
            joint = SERVO2JOINT.get(servo, None)
            if joint is None:
                self.get_logger().warn(f"Unknown servo: {servo}", once=True)
                continue
            out.name.append(joint)
            out.position.append(np.deg2rad(SERVOS[servo].us2ang(pulse)))

        debug = {joint: np.rad2deg(ang) for joint, ang in zip(out.name, out.position)}
        self.get_logger().info(f"State sent:\n{debug}", throttle_duration_sec=3)

        self.state_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    bridge = Bridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
