import serial
import time

class Motor:
    MAXIMUM_SPEED = 400
    MINIMUM_SPEED = 0

    def __init__(self):
        self.ser = serial.Serial("/dev/tty.usbmodem14101", 9600)
        self.gear = 1
        self.left_speed = 0
        self.right_speed = 0
        self.stop()

    @staticmethod
    def __format_serial_speed_input(speed):
        return "{:04d}".format(speed)

    def set_speed(self, left, right):
        if left < Motor.MINIMUM_SPEED or left > Motor.MAXIMUM_SPEED:
            raise ValueError("illegal left motor speed set at {}".format(left))
        if right < Motor.MINIMUM_SPEED or right > Motor.MAXIMUM_SPEED:
            raise ValueError("illegal right motor speed set at {}".format(right))
        self.left_speed = left
        self.right_speed = right
        left = left * self.gear
        right = right * self.gear
        serial_command = "M{}{}".format(Motor.__format_serial_speed_input(left), Motor.__format_serial_speed_input(right))
        self.ser.write(bytes(serial_command, 'utf-8'))

    def accelerate(self, boost):
        maximum_boost = min(Motor.MAXIMUM_SPEED - self.left_speed, Motor.MAXIMUM_SPEED - self.right_speed)
        result_boost = min(maximum_boost, boost)
        self.set_speed(self.left_speed + result_boost, self.right_speed + result_boost)

    def decelerate(self, brake):
        maximum_brake = min(self.left_speed, self.right_speed)
        result_brake = min(maximum_brake, brake)
        self.set_speed(self.left_speed - result_brake, self.right_speed - result_brake)

    def stop(self):
        self.set_speed(0, 0)

    def forward_gear(self):
        if self.gear == 1:
            return
        else:
            self.reverse_gear()

    def reverse_gear(self):
        self.set_speed(0, 0)
        self.gear = -self.gear
        time.sleep(1) # Protect DC motor

    def left_turn(self, ratio, faster_motor_speed=-1):
        if ratio < 0 or ratio > 1:
            raise ValueError("illegal turning ratio of {}".format(ratio))
        if faster_motor_speed < 0:
            faster_motor_speed = max(self.left_speed, self.right_speed)
        slower_motor_speed = int(faster_motor_speed * ratio)
        self.set_speed(slower_motor_speed, faster_motor_speed)

    def right_turn(self, ratio, faster_motor_speed=-1):
        if ratio < 0 or ratio > 1:
            raise ValueError("illegal turning ratio of {}".format(ratio))
        if faster_motor_speed < 0:
            faster_motor_speed = max(self.left_speed, self.right_speed)
        slower_motor_speed = int(faster_motor_speed * ratio)
        self.set_speed(faster_motor_speed, slower_motor_speed)

    def rotate_clockwise(self, seconds = 1):
        self.right_turn(0, 50) # arbitrary number
        time.sleep(seconds)
        self.stop()
