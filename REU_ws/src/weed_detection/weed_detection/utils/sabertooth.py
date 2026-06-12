# Niels Keller
# Sabertooth class
#
# Relevant Documentation:
# https://www.dimensionengineering.com/datasheets/Sabertooth2x12.pdf

import numpy as np
import serial
import time


def linear_map_constrain_int(value, from_low, from_high, to_low, to_high):
    factor = (value - from_low) / (from_high - from_low)
    mapped_val = (to_high - to_low) * factor + to_low
    return round(min(to_high, max(to_low, mapped_val)))


class SaberToothMotorDriver:

    def __init__(self, motor1_reversed, motor2_reversed,
                 accel_rate=80.0, decel_rate=120.0, call_rate_hz=10.0):
        """
        Motor class constructor.

        Parameters
        ----------
        motor1_reversed : bool
        motor2_reversed : bool
        accel_rate : float
            Max speed increase per second (actuator units/s).
            e.g. 80 means 0->100% in ~1.25 seconds.
        decel_rate : float
            Max speed decrease per second. Faster than accel so the robot
            can stop promptly when commanded. 120 = 100->0 in ~0.83 seconds.
        call_rate_hz : float
            Expected rate at which updateMotorSpeed is called.
            Must match your control loop timer (default 10 Hz).
        """
        self.sabertooth_UART_serial = serial.Serial(
            port="/dev/ttyTHS0",
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        time.sleep(1)

        self.motor1_reversed = motor1_reversed
        self.motor2_reversed = motor2_reversed

        # Trapezoidal profile parameters
        self._dt = 1.0 / call_rate_hz
        self._accel_step = accel_rate * self._dt   # max delta per call when speeding up
        self._decel_step = decel_rate * self._dt   # max delta per call when slowing down

        # Current ramped speeds (float, -100 to 100)
        self._current_left = 0.0
        self._current_right = 0.0

    # ------------------------------------------------------------------
    # Internal ramp helper
    # ------------------------------------------------------------------

    def _ramp(self, current, target):
        """
        Step `current` toward `target` by at most one accel/decel step.

        Deceleration is used when the magnitude is shrinking OR when the
        sign is flipping (we treat a sign flip as decel-through-zero first).
        """
        error = target - current

        # Determine whether this is an acceleration or deceleration move.
        # It's deceleration if we're reducing magnitude or crossing zero.
        speeding_up = (abs(target) > abs(current)) and (np.sign(target) == np.sign(current) or current == 0.0)

        max_step = self._accel_step if speeding_up else self._decel_step

        if abs(error) <= max_step:
            return target                          # close enough, snap to target
        else:
            return current + np.sign(error) * max_step

    # ------------------------------------------------------------------
    # Low-level motor writes (unchanged from original)
    # ------------------------------------------------------------------

    def _write_motor1(self, speed):
        """Send speed (-100 to 100) directly to motor 1 (right)."""
        if self.motor1_reversed:
            speed = -speed
        send_val = 64  # stop
        if speed < 0:
            send_val = linear_map_constrain_int(100 + speed, 0, 100, 1, 64)
        elif speed > 0:
            send_val = linear_map_constrain_int(speed, 0, 100, 64, 127)
        self.sabertooth_UART_serial.write([send_val])

    def _write_motor2(self, speed):
        """Send speed (-100 to 100) directly to motor 2 (left)."""
        if self.motor2_reversed:
            speed = -speed
        send_val = 192  # stop
        if speed < 0:
            send_val = linear_map_constrain_int(100 + speed, 0, 100, 128, 192)
        elif speed > 0:
            send_val = linear_map_constrain_int(speed, 0, 100, 192, 255)
        self.sabertooth_UART_serial.write([send_val])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_motor1_speed(self, speed):
        """Direct set — bypasses ramp. Use updateMotorSpeed for normal operation."""
        self.__motor1_speed = speed
        self._write_motor1(speed)

    def set_motor2_speed(self, speed):
        """Direct set — bypasses ramp. Use updateMotorSpeed for normal operation."""
        self.__motor2_speed = speed
        self._write_motor2(speed)

    def get_motor1_speed(self):
        return self._current_right

    def get_motor2_speed(self):
        return self._current_left

    def updateMotorSpeed(self, left_speed, right_speed):
        """
        Command both wheel speeds (-100 to 100).

        Internally applies a trapezoidal velocity profile so the hardware
        receives smooth, rate-limited commands rather than step changes.
        Call this at a fixed rate (matching call_rate_hz) for best results.
        """
        self._current_left  = self._ramp(self._current_left,  left_speed)
        self._current_right = self._ramp(self._current_right, right_speed)

        self._write_motor2(self._current_left)
        self._write_motor1(self._current_right)

    def all_motors_off(self):
        """Emergency stop — bypasses ramp and immediately halts both motors."""
        self._current_left = 0.0
        self._current_right = 0.0
        self.sabertooth_UART_serial.write([0])
        print("ALL MOTORS OFF SENT")

    def __del__(self):
        self.sabertooth_UART_serial.write([0])