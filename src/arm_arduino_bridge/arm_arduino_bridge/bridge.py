"""Uses http to communicate with Arduino to send & retrieve sensor_msgs.JointState."""

import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial.tools import list_ports
from serial.tools.list_ports_common import ListPortInfo
from std_msgs.msg import UInt16MultiArray

from .interface import HEARTBEAT_PERIOD, N_SERVO, USBridge

ESTABLISH_TIMEOUT = 5  # seconds
DEV_ID = "10c4:ea60"
BAUD_RATE = 921600
PLATFORMIO_DIR = Path("/workspaces/dai-pds/src/arm_arduino_bridge/platformio")


@dataclass
class Servo:
    """Servo calibration settings."""

    mid_pulse: int = 1500
    pulse_range: int = 1000

    # To avoid some issues with np.deg2rad possibly convert 0 to 2pi, just work
    # in radians.
    min_angle: float = 0.0
    max_angle: float = np.pi

    flip: bool = False

    def us2rad(self, pulse: int) -> float:
        """Convert microseconds to angle."""
        min_pulse = self.mid_pulse - self.pulse_range
        max_pulse = self.mid_pulse + self.pulse_range
        if self.flip:
            min_pulse, max_pulse = -max_pulse, -min_pulse
            pulse = -pulse
        return np.interp(
            pulse, (min_pulse, max_pulse), (self.min_angle, self.max_angle)
        )

    def rad2us(self, ang: float) -> int:
        """Convert angle to microseconds."""
        min_pulse = self.mid_pulse - self.pulse_range
        max_pulse = self.mid_pulse + self.pulse_range
        if self.flip:
            min_pulse, max_pulse = -max_pulse, -min_pulse
        # NOTE: We don't wrap angles here, so ensure the digital robot model is accurate!
        pulse = int(
            np.interp(
                ang,
                (self.min_angle, self.max_angle),
                (min_pulse, max_pulse),
            )
        )
        return -pulse if self.flip else pulse


# TODO: Don't hardcode.
JOINT2SERVO = {
    "servo_1": "1_servo",
    "servo_2": "2_servo",
    "servo_3": "3_servo",
    "servo_4": "4_servo",
    "servo_5": "5_servo",
    "servo_6": "6_servo",
    "servo_7": "7_servo",
    "servo_8": "8_servo",
}
SERVO2JOINT = {v: k for k, v in JOINT2SERVO.items()}

# Values were manually calibrated. Angle ranges should be from digital robot model.
# TODO: Only 1_servo has been calibrated, tape label the rest & calibrate.
SERVOS = {
    "1_servo": Servo(1500, 1000, 0.0, np.pi, True),
    "2_servo": Servo(1500, 1000, -np.pi, 0.0),
    "3_servo": Servo(1500, 1000, -np.pi / 2, np.pi / 2),
    "4_servo": Servo(1500, 1000, -np.pi, 0.0, True),
    "5_servo": Servo(1500, 1000, -np.pi, 0.0),
    "6_servo": Servo(1500, 1000, -np.pi, 0.0, True),
    "7_servo": Servo(1500, 1000, -np.pi / 2, np.pi / 2),
    "8_servo": Servo(1500, 1000, 0.0, np.pi, True),
}
# SERVOS = {
#     "1_servo": Servo(1500, 1000, -np.pi, 0.0),
#     "2_servo": Servo(1500, 1000, 0.0, np.pi),
#     "3_servo": Servo(1500, 1000, -np.pi / 2, np.pi / 2),
#     "4_servo": Servo(1500, 1000, -np.pi, 0.0),
#     "5_servo": Servo(1500, 1000, 0.0, np.pi),
#     "6_servo": Servo(1500, 1000, -np.pi, 0.0),
# }


def find_arduino_port(dev_id: str):
    try:
        # Assume first Arduino device found is the one we want.
        dev: ListPortInfo = next(list_ports.grep(dev_id))
    except StopIteration:
        return None
    return dev.device


class Bridge(Node):
    """Bridge between ROS2 and Arduino over http."""

    def __init__(self):
        """Initialize."""
        super(Bridge, self).__init__("arm_arduino_bridge")

        self.state_pub = self.create_publisher(JointState, "robot_joint_states", 1)

        self.create_subscription(
            JointState, "robot_joint_commands", self._cb_joint_cmd, 1
        )
        self.create_subscription(UInt16MultiArray, "cmd", self._cb_direct_ctrl, 1)

        self.usb = None
        self.__has_init_connect = False

        # initial connection cannot be done in __init__ so start after delay
        self.create_timer(0.1, self._initial_connect)
        self.create_timer(HEARTBEAT_PERIOD / 1000 / 2, self.heartbeat)
        self.create_timer(0.01, self._update_state)

    def _initial_connect(self):
        if self.__has_init_connect:
            return
        self.__has_init_connect = True
        self.connect()

    def connect(self):
        while True:
            try:
                self.get_logger().info("Running platformio/scripts/build.sh")
                self.get_logger().info(
                    "Will take a long while for fresh install due to library installation."
                )
                subprocess.run(
                    [str(PLATFORMIO_DIR / "scripts" / "build.sh"), "--skip-same"],
                    check=True,
                )

                self.get_logger().info("Arduino code uploaded/skipped")

            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Exit {e.returncode}: {e.output}")
                self.get_logger().info("Retrying...")
                continue

            # Arduino port will change as a result of flashing. Redetect.
            port = None
            while port is None:
                port = find_arduino_port(DEV_ID)
                time.sleep(0.2)
            self.get_logger().info(f"Arduino port: {port}")
            self.usb = USBridge(port, BAUD_RATE, self.get_logger())

            # Try until Linux is done configuring device.
            while True:
                try:
                    self.usb.open()
                    break
                except serial.SerialException:
                    time.sleep(0.2)

            start = time.monotonic()
            while (
                not self.usb.is_connected
                and time.monotonic() - start < ESTABLISH_TIMEOUT
            ):
                time.sleep(0.2)

            if self.usb.is_connected:
                break
            else:
                self.get_logger().error("Establishment timed out. Retrying...")
                self.usb.close()
                self.usb = None

        self.get_logger().info("Connection established, Arduino bridge started.")

    def disconnect(self):
        if self.usb is None:
            return
        self.usb.close()

    def heartbeat(self):
        if self.usb is None:
            return
        try:
            self.usb.heartbeat()
            if not self.usb.is_connected:
                raise serial.SerialTimeoutException()
        except (serial.SerialTimeoutException, serial.SerialException):
            self.get_logger().error("Heartbeat timed out. Reconnecting...")
            self.disconnect()
            self.connect()

    def _cb_joint_cmd(self, msg: JointState):
        """Execute joint state."""
        if self.usb is None:
            return
        params = {}
        params_deg = {}
        for i, joint in enumerate(msg.name):
            servo = JOINT2SERVO.get(joint, None)
            if servo is None:
                self.get_logger().warn(f"Unknown joint: {joint}", once=True)
                continue
            rad = msg.position[i]
            params_deg[servo] = np.rad2deg(rad)
            params[servo] = SERVOS[servo].rad2us(rad)
        self.get_logger().info(
            f"Cmd Received:\n{params}\n{params_deg}", throttle_duration_sec=3
        )
        vals = []
        for i in range(N_SERVO):
            servo = f"{i + 1}_servo"
            vals.append(params.get(servo, 0))
        try:
            self.usb.send_servos(vals)
        except (serial.SerialTimeoutException, serial.SerialException):
            self.get_logger().error("Motor write timed out. Reconnecting...")
            self.disconnect()
            self.connect()

    def _cb_direct_ctrl(self, msg: UInt16MultiArray):
        """Directly control servos."""
        if self.usb is None:
            return
        if len(msg.data) != N_SERVO:
            self.get_logger().error(
                f"Expected {N_SERVO} servo values, got {len(msg.data)}."
            )
            return
        try:
            self.usb.send_servos(msg.data)
        except (serial.SerialTimeoutException, serial.SerialException):
            self.get_logger().error("Motor write timed out. Reconnecting...")
            self.disconnect()
            self.connect()

    def _update_state(self):
        """Update joint state."""
        if self.usb is None or self.usb.servo_vals is None:
            return
        arr = self.usb.servo_vals
        self.usb.servo_vals = None

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        for i, pulse in enumerate(arr):
            servo = f"{i + 1}_servo"
            joint = SERVO2JOINT.get(servo, None)
            if joint is None:
                self.get_logger().warn(f"Unknown servo: {servo}", once=True)
                continue
            out.name.append(joint)
            out.position.append(SERVOS[servo].us2rad(pulse))

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
