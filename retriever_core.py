from enum import Enum
from motor_control import Motor, DummyMotor
from ultrasonic_led_control import UltrasonicAndLED
from stereo import Stereo

import time
import signal
import numpy as np
import json
import sys
sys.path.insert(0,'./SDK/build')
from cam_run import Camera
from darknet import Darknet
from util import *

class RetrieverState(Enum):
    SEARCH = "search"
    WAIT = "wait"
    TRACK = "track"
    CAPTURE = "capture"
    PLAYER_SEARCH = "player_search"
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
            # Initialize motor and set gate position
            if len(sys.argv) > 1 and sys.argv[1] == "up":
                self.motor = Motor(True, False)
            elif len(sys.argv) > 1 and sys.argv[1] == "down":
                self.motor = Motor(False, True)
            else:
                self.motor = Motor()
        else:
            self.motor = DummyMotor()
        self.stereo=Stereo(projectDict)
        self.state = RetrieverState.SEARCH
        self.cam = Camera(projectDict)
        self.img=0
        self.ball_center=0
        self.player_center=0
        self.max_unfound=projectDict["MAX_UNFOUND"]
        self.max_player_unfound = projectDict["MAX_PLAYER_UNFOUND"]
        self.dist_thresh=projectDict["DIST_THRESHOLD"] #unit m
        self.OFFER_THRESHOD = projectDict["OFFER_THRESHOLD"]
        #################
        #PID SETUP
        #################
        self.KP=projectDict["KP"]
        self.KD=projectDict["KD"]
        self.KI=projectDict["KI"]
        self.BASE_SPEED=projectDict["BASE_SPEED"]
        self.MAX_SPEED=projectDict["MAX_SPEED"] #400
        self.MIN_SPEED=projectDict["MIN_SPEED"] #-400
        self.BASE_SPEED=projectDict["BASE_SPEED"]
        self.IDLE_SPEED=projectDict["IDLE_SPEED"]
        self.CENTER_X=projectDict["IMG_CENTER_X"] #center of image

        # Ultrasonic
        self.ultra_led = UltrasonicAndLED()

    def run(self):
        while True:
            next_state = getattr(self, self.state.value)()
            self.state = next_state
            print(self.state)

    def search(self):
        print('Search State: Start Searching')
        self.ultra_led.to_wait()
        self.motor.stop()
        time.sleep(1)
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
        time.sleep(1) #Sleep 1 second to protect the motor

        # Check if the player is around
        player_center = self.cam.detect_player(img[0])
        distance = self.stereo.measure_dist(img, ball_center, player_center)
        
        if player_center != 0 and self.dist_thresh and distance != 0:
            print("Search Phase: Transition to Wait State")
            return RetrieverState.WAIT
        else:
            print("Search Phase: Transition to Track State")
            return RetrieverState.TRACK

    def wait(self):
        print("Wait State: Start Waiting")
        self.ultra_led.to_wait()
        loss_count = 0
        while True:
            img = self.cam.grab_img()
            ball_center = self.cam.detect_ball(img[0])
            player_center = self.cam.detect_player(img[0])
            if self.vis:
                self.cam.display_img(ball_center, player_center, img[0])
            
            if ball_center != 0:
                if player_center != 0:
                    distance = self.stereo.measure_dist(img, ball_center, player_center)
                    if distance > self.dist_thresh:
                    # if distance > self.dist_thresh or distance == 0:
                        print("Wait State: Ball is far enough from the player, transition to Track State")
                        return RetrieverState.TRACK
                else:
                    print("Wait State: Ball out of control, transition to Track State")
                    return RetrieverState.TRACK
            elif ball_center == 0:
                loss_count+=1
                if loss_count > 3:
                    print("Wait State: Lost track of the ball, transition to Search State")
                    return RetrieverState.SEARCH

    def track(self):
        print('Track State: Start Tracking')
        self.ultra_led.to_track()
        num_not_found = 0
        pre_error = 0
        total_error = 0
        while True:
            img = self.cam.grab_img()
            ball_center = self.cam.detect_ball(img[0])
            player_center = self.cam.detect_player(img[0])
            if self.vis:
                self.cam.display_img(ball_center,player_center,img[0])

            if ball_center != 0: # Ball found
                num_not_found = 0
                if player_center != 0: # Player in the scope
                    distance = self.stereo.measure_dist(img, ball_center, player_center)
                    if distance < self.dist_thresh and distance != 0:
                        print("Track State: Ball back in control, transition Wait State")
                        self.motor.stop()
                        return RetrieverState.WAIT
                    else:
                        (left_speed, right_speed, pre_error) = self.__pid_speed(ball_center, pre_error)
                        self.motor.set_speed(int(left_speed), int(right_speed))
                        print("Track State: Tracking with {} {}".format(left_speed, right_speed))
                else: # Player not in the scope
                    (left_speed, right_speed, pre_error) = self.__pid_speed(ball_center, pre_error)
                    self.motor.set_speed(int(left_speed), int(right_speed))
                    print("Track State: Tracking with {} {}".format(left_speed, right_speed))
            else: # Lose ball
                num_not_found += 1
                print("Track State: Lost the ball for {} iterations".format(num_not_found))
                self.motor.set_speed(self.IDLE_SPEED, self.IDLE_SPEED)
                print("Track State: Set speed after lost")
                if num_not_found >= self.max_unfound:
                    print("Track State: Permanent loss, transition to Search State")
                    return RetrieverState.SEARCH

                ball_in = self.ultra_led.measure()
                if ball_in:
                    self.motor.stop()
                    print("Track State: Ball in, transition to Capture State")
                    return RetrieverState.CAPTURE

    def capture(self):
        print("Capture State: Start Capturing")
        self.motor.step_motor_down()
        print("Capture State: Successfully captured, transition to player_search state")
        return RetrieverState.PLAYER_SEARCH

        #To Confirm the ball is in
        # TODO: incompatible with led control
        # if self.ultra_led.measure():
        #     print("Capture State: Successfully captured, transition to player_search state")
        #     return RetrieverState.PLAYER_SEARCH
        # else:
        #     self.motor.step_motor_up()
        #     print("Capture State: Failure, transition to search state")
        #     return RetrieverState.SEARCH

    def player_search(self):
        print('Player_Search State: Start Searching')
        self.ultra_led.to_player_search()
        self.motor.stop()
        self.motor.forward_gear()
        self.motor.set_speed(100, 0)
        img = self.cam.grab_img()
        player_center = self.cam.detect_player(img[0])

        while player_center == 0:
            img = self.cam.grab_img()
            player_center = self.cam.detect_player(img[0])

        print("Player_Search State: Player detected, transition to offer state")
        self.motor.stop()
        return RetrieverState.OFFER

    def offer(self):
        self.ultra_led.to_player_approach()
        print('Offer State: Start approaching the player')
        num_not_found = 0
        pre_error = 0
        total_error=0
        while True:
            img = self.cam.grab_img()
            player_center = self.cam.detect_player(img[0])

            if player_center != 0: # Player in scope
                num_not_found = 0
                player_distance = self.stereo.measure_player_dist(img, player_center)
          
                if player_distance < self.OFFER_THRESHOD and player_distance != 0:
                    print("Offer State: Close enough, transition to the release state")
                    return RetrieverState.RELEASE

                (left_speed, right_speed, pre_error) = self.__pid_speed(player_center, pre_error)
                self.motor.set_speed(int(left_speed), int(right_speed))
                print("Offer State: Approaching with {} {} distance: {}".format(left_speed, right_speed,player_distance))
            else: # Lost Player
                num_not_found += 1
                self.motor.set_speed(self.IDLE_SPEED, self.IDLE_SPEED)
                print("Offer State: Lost the player for {} iterations".format(num_not_found))
                if num_not_found >= self.max_player_unfound:
                    print("Offer State: Permanent loss, transition to Player_Search State")
                    return RetrieverState.PLAYER_SEARCH

    def release(self):
        print('Release State: Start release')
        self.ultra_led.to_return()
        self.motor.stop()
        self.motor.step_motor_up()
        self.motor.reverse_gear()
        self.motor.set_speed(self.IDLE_SPEED, self.IDLE_SPEED)
        time.sleep(5)
        self.motor.stop()
        self.motor.forward_gear()
        time.sleep(5) # One loop done, take a break
        print('Release State: Release done, return to search state')
        return RetrieverState.SEARCH

    def error(self):
        self.motor.stop()
        time.sleep(1)
        while True:
            print("ERROR!!!!!!")

    def __pid_speed(self, ball_center, error):
        (ball_x, ball_y) = ball_center
        pre_error = error
        error = ball_x - self.CENTER_X
        
        d_speed = self.KP * error + self.KD * (error - pre_error)# + self.KI * sum_error
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
    core = Core()

    def signal_handler(*args):
        print('You pressed Ctrl+C!')
        core.ultra_led.to_wait()
        core.motor.stop()
        core.motor.step_motor_down()
        core.cam.close_cam()
        print('System exit')
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    core.run()

if __name__ == "__main__":
    main()
