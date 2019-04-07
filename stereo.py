import cv2
import numpy as np
import math
import json
import time
import  matplotlib as plt
class Stereo:
    def __init__(self,projectDict):
        
        ##############
        #CAMERA PARAM
        ##############
        self.LEFT_MTX = np.asarray(projectDict["LEFT_MTX"])
        self.RIGHT_MTX= np.asarray(projectDict["RIGHT_MTX"])
        self.LEFT_DIST= np.asarray(projectDict["LEFT_DIST"])
        self.RIGHT_DIST= np.asarray(projectDict["RIGHT_DIST"])
        self.R= np.asarray(projectDict["R"])
        self.T= np.asarray(projectDict["T"])
        self.R1= np.asarray(projectDict["R1"])
        self.R2= np.asarray(projectDict["R2"])
        self.P1= np.asarray(projectDict["P1"])
        self.P2= np.asarray(projectDict["P2"])
        self.Q= np.asarray(projectDict["Q"])
        numDisp=projectDict["NUM_DISP"]
        block_size=projectDict["BLOCK_SIZE"]
        X=np.asarray(projectDict["IMG_SIZE_X"])
        Y=np.asarray(projectDict["IMG_SIZE_Y"])
        self.img_size=(X,Y)
        self.Map_Left_1,self.Map_Left_2=cv2.fisheye.initUndistortRectifyMap(self.LEFT_MTX,self.LEFT_DIST,self.R1,self.P1,self.img_size,cv2.CV_32FC1)
        self.Map_Right_1,self.Map_Right_2=cv2.fisheye.initUndistortRectifyMap(self.RIGHT_MTX,self.RIGHT_DIST,self.R2,self.P2,self.img_size,cv2.CV_32FC1)
        
        self.stereo = cv2.StereoBM_create(numDisparities=numDisp, blockSize=block_size)
        
        
        
    def measure_dist(self, img_pair,ball_center,player_center):
        t1=time.time()
        img_left=img_pair[0]
        img_right=img_pair[1]
        img_left_rect=cv2.remap(img_left, self.Map_Left_1, self.Map_Left_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        img_right_rect=cv2.remap(img_right, self.Map_Right_1, self.Map_Right_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgl = cv2.cvtColor(img_left_rect, cv2.COLOR_BGR2GRAY)
        imgr = cv2.cvtColor(img_right_rect, cv2.COLOR_BGR2GRAY)
        disparity = self.stereo.compute(imgl,imgr)
        points=cv2.reprojectImageTo3D(disparity,self.Q,handleMissingValues=True)
        
        (b_x,b_y)=ball_center
        (p_x,p_y)=player_center
        bp=points[b_y][b_x]
        pp=points[p_y][p_x]
        print(bp[2],pp[2])
        if bp[2]>100 or pp[2]>100:
            print("measure fail")
            return 0
        bp=bp*1000
        pp=pp*1000
        #too far from ball, move closer
        if bp[2]>1:
            return 0
        
        a=bp[0]-pp[0]
        b=bp[1]-pp[1]
        c=bp[2]-pp[2]
        dist=math.sqrt(math.pow(a,2)+math.pow(b,2)+math.pow(c,2))
        print(time.time()-t1)
        print(bp)
        print(dist)
        #plt.imshow(disparity,'gray')
        #plt.show()
        return dist
