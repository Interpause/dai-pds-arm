"""New API library for the Arduino bridge using github.com/yesbotics/simple-serial-protocol-python."""

import time
from typing import List

import serial
from simple_serial_protocol import (
    AbstractSerialPort,
    CommandParam,
    SimpleSerialProtocol,
)
from simple_serial_protocol.common import Byte
from simple_serial_protocol.param_type.ParamTypeString import ParamTypeString
from simple_serial_protocol.param_type.ParamTypeUnsignedInt16 import (
    ParamTypeUnsignedInt16,
)

# Refer to `lib/USBridge/USBridge.h` for const values.
CMD_HANDSHAKE = "s"
CMD_HEARTBEAT = "h"
CMD_WRITE_SERVO = "w"
CMD_READ_SERVO = "r"
CMD_DEBUG = "d"
EOT_CHAR = "\n"  # SimpleSerialProtocol.__CHAR_EOT

N_SERVO = 16

HEARTBEAT_PERIOD = 200  # ms
HEARTBEAT_TIMEOUT = 400  # ms
HEARTBEAT_MISSES = 2
# TODO: Further consider the effects of read and write timeout when serial connection is shaky.
# See: https://pyserial.readthedocs.io/en/latest/pyserial_api.html
READ_TIMEOUT = None  # ms
WRITE_TIMEOUT = 100  # ms

USE_CHECKSUM = False  # DEBUG: Add checksum to verify serial integrity.


# NOTE: recv callbacks should only care if SSP is running, while senders must make sure its already connected.
class USBridge:
    def __init__(self, port: str, baud: int, logger):
        # Give ser logger to log every byte read and written.
        # ser = SerialWrapper(port, baud, logger)
        # ser = SerialWrapper(port, baud)
        # self.ssp = SimpleSerialProtocol(ser, self._cb_error)
        self.ssp = SimpleSerialProtocol(port, baud, self._cb_error)
        self.log = logger

        self.ssp.register_command(CMD_HANDSHAKE, self._cb_handshake)
        self.ssp.register_command(CMD_HEARTBEAT, self._cb_heartbeat)
        self.ssp.register_command(
            CMD_READ_SERVO,
            self._cb_servo,
            [ParamTypeUnsignedInt16.NAME] * (N_SERVO + (1 if USE_CHECKSUM else 0)),
        )
        self.ssp.register_command(CMD_DEBUG, self._cb_debug, [ParamTypeString.NAME])

        # TODO: The ssp library isn't robust to msg corruptions and throws uncaught errors in its own listener thread.
        # Theres no equivalent to the arduino's library don't die on non-registered commands...
        self.ssp.register_command(EOT_CHAR, self._clear_buffer)

        self._reset()

    def _reset(self):
        self.is_running = False
        self.is_connected = False
        self.servo_vals = None
        self.__last_hb_recv = 0.0
        self.__last_hb_sent = 0.0
        self.__hb_misses = 0

    def open(self):
        self.ssp.init()
        self._reset()
        self.is_running = True

    def close(self):
        self.ssp.dispose()
        self._reset()

    def heartbeat(self):
        """Send and receive heartbeat."""
        if not self.is_connected:
            return

        now = time.monotonic()
        if now - self.__last_hb_recv > HEARTBEAT_TIMEOUT / 1000:
            self.__hb_misses += 1
            self.__last_hb_recv = now

        if self.__hb_misses >= HEARTBEAT_MISSES:
            # TODO: Instead of listener.py checking if usb.is_connected, throw error instead?
            self.close()

        if now - self.__last_hb_sent > HEARTBEAT_PERIOD / 1000:
            self.send_heartbeat()

    def _cb_handshake(self):
        if self.is_connected:
            return

        if not self.is_running:
            return
        self.ssp.write_command(CMD_HANDSHAKE)
        self.log.info("Arduino handshake returned!")
        self.is_connected = True
        self.__last_hb_recv = time.monotonic()
        self.__hb_misses = 0

    def _cb_heartbeat(self):
        # Must be connected else heartbeat doesn't count.
        if not self.is_connected:
            return
        self.__last_hb_recv = time.monotonic()
        self.__hb_misses = 0

    def _cb_debug(self, debug_msg: str):
        if not self.is_running:
            return
        self.log.warning(f"[Arduino] {debug_msg}")

    def _cb_servo(self, *vals: int):
        if not self.is_running:
            return
        vals = list(vals)

        if USE_CHECKSUM:
            # Remove checksum from servo values.
            cksum = vals[-1]
            vals = vals[:-1]
            if cksum != sum(vals) % 65536:
                self.log.error("Checksum mismatch!")
                return

        if len(vals) != N_SERVO:
            self.log.error(f"Expected {N_SERVO} servo values, got {len(vals)}.")
            return
        self.servo_vals = vals

    def _clear_buffer(self):
        """Serial established halfway through a command so clear buffer."""
        self.ssp._serial_port.flush()

    def send_servos(self, vals: List[int]):
        if not self.is_connected:
            return
        assert len(vals) == N_SERVO, f"Expected {N_SERVO} servo values."

        args = []
        for v in vals:
            args.append(CommandParam(type=ParamTypeUnsignedInt16.NAME, value=v))

        # DEBUG: Add checksum to verify serial integrity.
        if USE_CHECKSUM:
            cksum = sum(vals) % 65536
            args.append(CommandParam(type=ParamTypeUnsignedInt16.NAME, value=cksum))

        self.ssp.write_command(CMD_WRITE_SERVO, args)

    def send_heartbeat(self):
        if not self.is_connected:
            return
        self.ssp.write_command(CMD_HEARTBEAT)
        self.__last_hb_sent = time.monotonic()

    def _cb_error(self, e: Exception):
        # TODO: exception handling
        self.log.error(f"Error: {e}")


class SerialWrapper(AbstractSerialPort):
    """Serial for SimpleSerialProtocol.

    Implementation below is very similar to:
    https://github.com/yesbotics/simple-serial-protocol-python/blob/a435a092b78a2cf789c3a2d8980cd6eebe065549/src/simple_serial_protocol/serial_port/PySerialSerialPort.py
    but with logging.
    """

    def __init__(self, portname: str, baudrate: int, logger=None):
        super(SerialWrapper, self).__init__(portname, baudrate)
        self.__log = logger
        self.__ser = None

    @property
    def is_open(self) -> bool:
        return self.__ser is not None

    def log(self, msg: str):
        if self.__log is not None:
            self.__log.info(msg)

    def open(self) -> None:
        if self.is_open:
            self.close()
        self.__ser = serial.Serial(
            self._portname,
            self._baudrate,
            timeout=None if READ_TIMEOUT is None else READ_TIMEOUT / 1000,
            write_timeout=None if WRITE_TIMEOUT is None else WRITE_TIMEOUT / 1000,
        )

    def close(self) -> None:
        if not self.is_open:
            return
        self.__ser.close()
        self.__ser = None

    def available(self) -> int:
        if not self.is_open:
            return 0
        return self.__ser.in_waiting

    def read(self) -> Byte:
        if not self.is_open:
            # TODO: not sure if this is bad for SimpleSerialProtocol.
            return None
        byte = self.__ser.read()
        self.log(f"R: {byte}")
        return byte[0]

    def write(self, buffer: bytes) -> None:
        if not self.is_open:
            return
        self.__ser.write(buffer)
        self.log(f"W: {buffer}")

    def flush(self) -> None:
        if not self.is_open:
            return
        self.__ser.flush()
        self.__ser.reset_input_buffer()
        self.__ser.reset_output_buffer()
