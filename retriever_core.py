from enum import Enum
from motor_control import Motor
import time
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
        #self.motor = Motor()
        self.state = RetrieverState.SEARCH
        self.cam=cam_run.Camera()
        self.img=0
        self.ball_center=0
        self.player_center=0
        #################
        #PID SETUP
        #################
        self.KP=4
        self.KD=0.7
        self.BASE_SPEED=0
        self.MAX_SPEED=100 #400
        self.MIN_SPEED=0 #-400
        self.CENTER_X=375 #center of image

    def run(self):
        while True:
            next_state = getattr(self, self.state.value)()
            self.state = next_state
            print(self.state)

    def search(self):
        print('searching')
        #self.motor.forward_gear()
        self.img=self.cam.grab_img()
        self.ball_center=self.cam.detect_ball(self.img[0])
        while  self.ball_center==0: #true or false
            self.img=self.cam.grab_img()
            self.ball_center=self.cam.detect_ball(self.img[0])
            self.cam.display_img(self.ball_center,0,self.img[0])
            #self.motor.rotate_clockwise()
        if 0:
            return RetrieverState.WAIT
        else:
            return RetrieverState.TRACK

    @staticmethod
    def wait():
        while is_with_ballhandler(): #true or false
            time.sleep(1)
        return RetrieverState.TRACK

    def PID_speed(self,ball_center,error,pre_error):
        d_speed=0
        left_speed=0
        right_speed=0
        (ball_x,ball_y)=ball_center
        if(abs(error)<=50):
            error=error*0.5
        pre_error=error
        error=ball_x-self.CENTER_X
        
        d_speed=self.KP*error+self.KD*(error-pre_error)
        right_speed=self.BASE_SPEED-d_speed
        left_speed=self.BASE_SPEED+d_speed
        if(right_speed>self.MAX_SPEED):
            right_speed=self.MAX_SPEED
        elif(right_speed<self.MIN_SPEED):
            right_speed=self.MIN_SPEED
        if(left_speed>self.MAX_SPEED):
            left_speed=self.MAX_SPEED
        elif(left_speed<self.MIN_SPEED):
            left_speed=self.MIN_SPEED
        return (left_speed,right_speed,error,pre_error)
        
    def track(self):
        print('tracking')
        left_speed=0
        right_speed=0
        error=0
        pre_error=0
        while True:
            self.img=self.cam.grab_img()
            self.ball_center=self.cam.detect_ball(self.img[0])
            self.cam.display_img(self.ball_center,0,self.img[0])
            if(self.ball_center==0):
                left_speed=min(left_speed,right_speed)
                right_speed=left_speed
            else:
                (left_speed,right_speed,error,pre_error)=self.PID_speed(self.ball_center,error,pre_error)
            #self.motor.set_speed(left_motor_speed, right_motor_speed)
            #if left_speed == 0 and right_speed == 0:
                #return RetrieverState.CAPTURE

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
