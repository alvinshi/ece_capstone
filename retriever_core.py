from enum import Enum
from motor_control import Motor, DummyMotor
from ultrasonic_control import Ultrasonic
from stereo import Stereo
import time
import numpy as np
import json
import sys
sys.path.insert(0,'./SDK/build')
import cam_run
from darknet import Darknet
from util import *

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
        inputJSON='PARAM.json'
        ########
        #SETUP
        ########
        fp = open(inputJSON)
        projectDict = json.load(fp)
        fp.close()
        self.vis=projectDict["VISUALIZE"]
        self.move=projectDict["MOVE"]
        if self.move:
            self.motor = Motor()
        else:
            self.motor = DummyMotor()
        self.Stereo=Stereo(projectDict)
        self.state = RetrieverState.SEARCH
        self.cam=cam_run.Camera(projectDict)
        self.img=0
        self.ball_center=0
        self.player_center=0
        self.max_unfound=projectDict["MAX_UNFOUND"]
        self.dist_thresh=projectDict["DIST_THRESHOLD"] #unit m
        #################
        #PID SETUP
        #################
        self.KP=projectDict["KP"]
        self.KD=projectDict["KD"]
        self.BASE_SPEED=projectDict["BASE_SPEED"]
        self.MAX_SPEED=projectDict["MAX_SPEED"] #400
        self.MIN_SPEED=projectDict["MIN_SPEED"] #-400
        self.BASE_SPEED=projectDict["BASE_SPEED"]
        self.IDLE_SPEED=projectDict["IDLE_SPEED"]
        self.CENTER_X=projectDict["IMG_CENTER_X"] #center of image

        # Ultrasonic
        self.ultrasonic = Ultrasonic()
        

    def run(self):
        while True:
            next_state = getattr(self, self.state.value)()
            self.state = next_state
            print(self.state)

    def search(self):
        print('Search State: Start Searching')
        self.motor.stop()
        self.motor.forward_gear()
        self.motor.rotate_clockwise()
        img = self.cam.grab_img()
        ball_center = self.cam.detect_ball(img[0])

        while ball_center == 0: #no ball detected
            img = self.cam.grab_img()
            ball_center = self.cam.detect_ball(img[0])
            if self.vis:
                self.cam.display_img(ball_center,0, img[0])

        print("Search State: Ball Detected")
        self.motor.stop()
        time.sleep(0.5) #Sleep 0.5 second to protect the motor

        # Check if the player is around
        player_center = self.cam.detect_player(img[0])
        distance = self.Stereo.measure_dist(img, ball_center, player_center)
        
        if player_center != 0 and self.dist_thresh and distance != 0:
            print("Search Phase: Transition to Wait State")
            return RetrieverState.WAIT
        else:
            print("Search Phase: Transition to Track State")
            return RetrieverState.TRACK

    def wait(self):
        print("Wait State: Start Waiting")
        while True:
            img = self.cam.grab_img()
            ball_center = self.cam.detect_ball(img[0])
            player_center = self.cam.detect_player(img[0])
            if self.vis:
                self.cam.display_img(ball_center, player_center, img[0])
            
            if ball_center != 0:
                if player_center != 0:
                    distance = self.Stereo.measure_dist(img, ball_center, player_center)
                    if distance > self.dist_thresh or distance == 0:
                        print("Wait State: Ball is far enough from the player, transition to Wait State")
                        return RetrieverState.TRACK
                else:
                    print("Wait State: Ball out of control, transition to Wait State")
                    return RetrieverState.TRACK
            elif ball_center == 0:
                print("Wait State: Lost track of the ball, transition to Search State")
                return RetrieverState.SEARCH

    def track(self):
        print('Track State: Start Tracking')
        num_not_found = 0
        pre_error = 0
        while True:
            img = self.cam.grab_img()
            ball_center = self.cam.detect_ball(img[0])
            player_center = self.cam.detect_player(img[0])
            if self.vis:
                self.cam.display_img(ball_center,player_center,img[0])
            
            if ball_center != 0:
                num_not_found = 0
                if player_center != 0:
                    distance = self.Stereo.measure_dist(img, ball_center, player_center)
                    if distance < self.dist_thresh and distance != 0:
                        print("Track State: Ball back in control, transition Wait State")
                        self.motor.stop()
                        return RetrieverState.WAIT
                else:
                    (left_speed, right_speed, pre_error) = self.__pid_speed(ball_center, pre_error)
                    self.motor.set_speed(int(left_speed), int(right_speed))
                    print("Track State: Tracking with {} {}".format(left_speed, right_speed))
            else:
                num_not_found += 1
                print("Track State: Lost the ball for {} iterations".format(num_not_found))
                if num_not_found >= self.max_unfound:
                    print("Track State: Permanent loss, transition to Search State")
                    return RetrieverState.SEARCH

                self.motor.set_speed(self.IDLE_SPEED, self.IDLE_SPEED)
                ball_in = self.ultrasonic.measure()
                if ball_in:
                    self.motor.stop()
                    print("Track State: Ball in, transition to Capture State")
                    return RetrieverState.CAPTURE

    def capture(self):
        print("Capture State: Start Capturing")
        self.motor.step_motor_down()

        #To Confirm the ball is in
        if self.ultrasonic.measure():
            print("Capture State: Successfully Captured")
        else:
            self.motor.step_motor_up()
            print("Capture State: Failure, transition to search state")
            return RetrieverState.SEARCH

        while True:
            time.sleep(1)
            # For now just hand in here
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
            print("ERROR!!!!!!")

    def __pid_speed(self, ball_center, error):
        (ball_x, ball_y) = ball_center
        pre_error = error
        error = ball_x - self.CENTER_X

        d_speed = self.KP * error + self.KD * (error - pre_error)
        right_speed = self.BASE_SPEED - d_speed
        left_speed = self.BASE_SPEED + d_speed
        if right_speed > self.MAX_SPEED:
            right_speed = self.MAX_SPEED
        elif right_speed < self.MIN_SPEED:
            right_speed = self.MIN_SPEED
        if left_speed > self.MAX_SPEED:
            left_speed = self.MAX_SPEED
        elif left_speed < self.MIN_SPEED:
            left_speed = self.MIN_SPEED
        return left_speed, right_speed, error

def main():
    Core().run()

if __name__ == "__main__":
    main()
