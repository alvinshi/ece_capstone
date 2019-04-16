import cv2
import numpy as np
import math
import json
import time
from matplotlib import pyplot as plt

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
        ##############
        #STEREO PARAM
        ##############
        minDisp=projectDict["MIN_DISP"]
        numDisp=projectDict["NUM_DISP"]
        block_size=projectDict["BLOCK_SIZE"]
        disp12=projectDict["DISP12MaxDiff"]
        uniqueness=projectDict["uniquenessRatio"]
        preFilterCap=projectDict["preFilterCap"]
        speckleRange=projectDict["speckleRange"]
        speckleWindowSize=projectDict["speckleWindowSize"]
        sgbmP1=8*block_size*block_size
        sgbmP2=32*block_size*block_size
        
        X=np.asarray(projectDict["IMG_SIZE_X"])
        Y=np.asarray(projectDict["IMG_SIZE_Y"])
        self.img_size=(X,Y)
        self.Map_Left_1,self.Map_Left_2=cv2.fisheye.initUndistortRectifyMap(self.LEFT_MTX,self.LEFT_DIST,self.R1,self.P1,self.img_size,cv2.CV_32FC1)
        self.Map_Right_1,self.Map_Right_2=cv2.fisheye.initUndistortRectifyMap(self.RIGHT_MTX,self.RIGHT_DIST,self.R2,self.P2,self.img_size,cv2.CV_32FC1)
        
        self.stereo = cv2.StereoSGBM_create(minDisparity=minDisp, 
                                          numDisparities=numDisp,
                                          blockSize=block_size,
                                          P1=sgbmP1,
                                          P2=sgbmP2,
                                          disp12MaxDiff=disp12,
                                          preFilterCap=preFilterCap,
                                          uniquenessRatio=uniqueness,
                                          speckleWindowSize=speckleWindowSize,
                                          speckleRange=speckleRange,
                                          )
    
    #return dist or 0 to keep tracking                                   
    def measure_player_dist(self,img_pair,player_center):
        if player_center==0:
            print("invalid player center value")
            return 0
        img_left=img_pair[0]
        img_right=img_pair[1]
        img_left_rect=cv2.remap(img_left, self.Map_Left_1, self.Map_Left_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        img_right_rect=cv2.remap(img_right, self.Map_Right_1, self.Map_Right_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgl = cv2.cvtColor(img_left_rect, cv2.COLOR_BGR2GRAY)
        imgr = cv2.cvtColor(img_right_rect, cv2.COLOR_BGR2GRAY)

        disparity = self.stereo.compute(imgl,imgr)
        points=cv2.reprojectImageTo3D(disparity,self.Q,handleMissingValues=True)
        
        (p_x,p_y)=player_center
        pp=points[p_y][p_x]

        if pp[2]>100: #out of range
            print("got invalid player dist")
            return 0 
        #units in m
        pp=pp*10
        #point to point distance
        dist=math.sqrt(math.pow(pp[0],2)+math.pow(pp[1],2)+math.pow(pp[2],2))
        print("Player distance {}".format(dist))
        return dist
    
    #return dist or 0 to keep tracking
    def measure_ball_dist(self,img_pair,ball_center):
        if ball_center==0:
            print("invalid ball center value")
            return 0
        img_left=img_pair[0]
        img_right=img_pair[1]
        img_left_rect=cv2.remap(img_left, self.Map_Left_1, self.Map_Left_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        img_right_rect=cv2.remap(img_right, self.Map_Right_1, self.Map_Right_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgl = cv2.cvtColor(img_left_rect, cv2.COLOR_BGR2GRAY)
        imgr = cv2.cvtColor(img_right_rect, cv2.COLOR_BGR2GRAY)

        disparity = self.stereo.compute(imgl,imgr)
        points=cv2.reprojectImageTo3D(disparity,self.Q,handleMissingValues=True)
        
        (b_x,b_y)=ball_center
        bp=points[b_y][b_x]

        if bp[2]>100: #out of range
            print("got invalid ball dist")
            return 0 
        #units in m
        bp=bp*10
        #point to point distance
        dist=math.sqrt(math.pow(bp[0],2)+math.pow(bp[1],2)+math.pow(bp[2],2))
        return dist
    
    #return dist or 0 to keep tracking   
    def measure_dist(self, img_pair,ball_center,player_center):
        #no need to measure dist, missing one obj
        if player_center==0 or ball_center==0: 
            print("invalid obj center value")
            return 0
        img_left=img_pair[0]
        img_right=img_pair[1]
        img_left_rect=cv2.remap(img_left, self.Map_Left_1, self.Map_Left_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        img_right_rect=cv2.remap(img_right, self.Map_Right_1, self.Map_Right_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgl = cv2.cvtColor(img_left_rect, cv2.COLOR_BGR2GRAY)
        imgr = cv2.cvtColor(img_right_rect, cv2.COLOR_BGR2GRAY)

        disparity = self.stereo.compute(imgl,imgr)
        points=cv2.reprojectImageTo3D(disparity,self.Q,handleMissingValues=True)
        
        (p_x,p_y)=player_center
        pp=points[p_y][p_x]   
        (b_x,b_y)=ball_center
        bp=points[b_y][b_x]
        #plt.imshow(disparity,'gray')
        #plt.show()
        if bp[2]>1000 or pp[2]>1000: #out of range
            print("got invalid dist")
            return 0 
        #units in m
        bp=bp*10
        pp=pp*10
        #too far from ball, keep doing current tracking and move closer
        # if bp[2]>1:
        #     print("ball far, moving closer")
        #     return 0
        
        a=bp[0]-pp[0]
        b=bp[1]-pp[1]
        c=bp[2]-pp[2]
        dist=math.sqrt(math.pow(a,2)+math.pow(b,2)+math.pow(c,2))
        
        print("ball to player distance {}".format(dist))
        return dist
