import serial
import time

class Motor:
    MAXIMUM_SPEED = 400
    MINIMUM_SPEED = 0
    ROTATE_SPEED = 70

    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", 9600) #try write_timeout=0
        self.gear = 1
        self.left_speed = 0
        self.right_speed = 0
        self.stop()
        self.gate_open = False
        time.sleep(2) # sleep is required for the motor to have enough set up time, reason unclear
        
        #self.step_motor_up() # Wind the gate motor up

    @staticmethod
    def __format_serial_speed_input(speed):
        return "{:04d}".format(speed)

    # Ignore the internal safety
    def __force_set_speed(self, left, right):
        serial_command = "M{}{}".format(Motor.__format_serial_speed_input(left),
                                        Motor.__format_serial_speed_input(right))
        self.ser.write(bytes(serial_command, 'utf-8'))

    def set_speed(self, left, right):
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
        if faster_motor_speed < 0:
            faster_motor_speed = max(self.left_speed, self.right_speed)
        slower_motor_speed = int(faster_motor_speed * ratio)
        self.set_speed(slower_motor_speed, faster_motor_speed)

    def right_turn(self, ratio, faster_motor_speed=-1):
        if faster_motor_speed < 0:
            faster_motor_speed = max(self.left_speed, self.right_speed)
        slower_motor_speed = int(faster_motor_speed * ratio)
        self.set_speed(faster_motor_speed, slower_motor_speed)

    def rotate_clockwise(self):
        self.__force_set_speed(-self.ROTATE_SPEED, self.ROTATE_SPEED)

    def step_motor_up(self):
        if self.gate_open:
            return
        else:
            self.ser.write(bytes("U", 'utf-8'))
            self.gate_open = True

    def step_motor_down(self):
        if self.gate_open:
            self.ser.write(bytes("D", 'utf-8'))
            self.gate_open = False
        else:
            return
