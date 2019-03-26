from enum import Enum
from motor_control import Motor
import time

class RetrieverState(Enum):
    SEARCH = "search"
    WAIT = "wait"
    TRACK = "track"
    CAPTURE = "capture"
    OFFER = "offer"
    RELEASE = "release"
    ERROR = "error"

class Core:
    def __init__(self):
        self.motor = Motor()
        self.state = RetrieverState.SEARCH

    def run(self):
        while True:
            next_state = getattr(self, self.state.value)()
            self.state = next_state

    def search(self):
        self.motor.forward_gear()
        while not ball_detected():
            self.motor.rotate_clockwise()
        if is_with_ballhandler():
            return RetrieverState.WAIT
        else:
            return RetrieverState.TRACK

    @staticmethod
    def wait():
        while is_with_ballhandler():
            time.sleep(1)
        return RetrieverState.TRACK

    def track(self):
        while True:
            left_motor_speed, right_motor_speed = get_direction()
            self.motor.set_speed(left_motor_speed, right_motor_speed)
            if left_motor_speed == 0 and right_motor_speed == 0:
                return RetrieverState.CAPTURE

    def capture(self):
        # TODO
        return RetrieverState.OFFER

    def offer(self):
        # TODO
        return RetrieverState.RELEASE

    def release(self):
        # TODO
        return RetrieverState.SEARCH

    def error(self):
        self.motor.stop()
        while True:
            print("error!!!!!!")

def main():
    Core().run()

if __name__ == "__main__":
    main()