from __future__ import division
import cv2
import apriltag
import cam
import os
import time
import copy
import threading
import torch 
import torch.nn as nn
from torch.autograd import Variable
import numpy as np
import cv2 
from util import *
import argparse
import os 
import os.path as osp
from darknet import Darknet
import pickle as pkl
import pandas as pd
import random 
        
class Camera:
    ##############################
    #Yolo PARAMETERS DARKNET SETUP 
    ##############################
    def __init__(self,projectDict):
        cam.cam_run()
        self.batch_size=projectDict["BATCH_SIZE"]
        self.confidence=projectDict["CONFIDENCE"]
        self.nms_thesh=projectDict["NMS_THRESH"]
        self.CUDA=torch.cuda.is_available()
        self.num_classes=projectDict["NUM_CLASSES"]
        self.root_path=projectDict["ROOT_PATH"]
        name_file=projectDict["OBJ_NAME_PATH"]
        cfg_file=projectDict["CFG_PATH"]
        weight_file=projectDict["WEIGHTS_PATH"]
        self.classes=load_classes(name_file)
        print("Loading network.....")
        self.model = Darknet(cfg_file)
        
        self.model.load_weights(weight_file)
        print("Network successfully loaded")
        self.model.net_info["height"] = 416
        self.inp_dim = int(self.model.net_info["height"])
        assert self.inp_dim % 32 == 0 
        assert self.inp_dim > 32
        #If there's a GPU availible, put the model on GPU
        if self.CUDA:
            self.model.cuda()
        #Set the model in evaluation mode
        self.model.eval()
        self.april_detector=apriltag.Detector()

    def detect_ball(self,img):
        center=0
        images='left.jpg'
        #img=left_img[len(left_img)-1]
        if(img.shape[0]>0 and img.shape[1]>0):
            write = 0
            imlist = []
            imlist.append(osp.join(osp.realpath('.'), images))
            loaded_ims=[img]
            im_batches = list(map(prep_image, loaded_ims, [self.inp_dim for x in range(len(imlist))]))
            im_dim_list = [(x.shape[1], x.shape[0]) for x in loaded_ims]
            im_dim_list = torch.FloatTensor(im_dim_list).repeat(1,2)
            if self.CUDA:
                im_dim_list = im_dim_list.cuda()
        
            start_det_loop = time.time()
            for i, batch in enumerate(im_batches):
                #load the image 
                if self.CUDA:
                    batch = batch.cuda()
                with torch.no_grad():
                    prediction = self.model(Variable(batch), self.CUDA)

                prediction = write_results(prediction, self.confidence, self.num_classes, nms_conf = self.nms_thesh)
                if (type(prediction) == int):
                    for im_num, image in enumerate(imlist[i*self.batch_size: min((i +  1)*self.batch_size, len(imlist))]):
                        im_id = i*self.batch_size + im_num 
                    continue
                prediction[:,0] += i*self.batch_size
                if not write:                      #If we have't initialised output
                    output = prediction  
                    write = 1
                else:
                    output = torch.cat((output,prediction))
                for im_num, image in enumerate(imlist[i*self.batch_size: min((i +  1)*self.batch_size, len(imlist))]):
                    im_id = i*self.batch_size + im_num
                    objs = [self.classes[int(x[-1])] for x in output if int(x[0]) == im_id]
                if self.CUDA:
                    torch.cuda.synchronize()       
            try:
                output
            except NameError:
                return 0
            im_dim_list = torch.index_select(im_dim_list, 0, output[:,0].long())
            scaling_factor = torch.min(416/im_dim_list,1)[0].view(-1,1)
            
            output[:,[1,3]] -= (self.inp_dim - scaling_factor*im_dim_list[:,0].view(-1,1))/2
            output[:,[2,4]] -= (self.inp_dim - scaling_factor*im_dim_list[:,1].view(-1,1))/2
            output[:,1:5] /= scaling_factor

            for i in range(output.shape[0]):
                output[i, [1,3]] = torch.clamp(output[i, [1,3]], 0.0, im_dim_list[i,0])
                output[i, [2,4]] = torch.clamp(output[i, [2,4]], 0.0, im_dim_list[i,1])

            center=self.yolo_draw_img(output[0],loaded_ims)
            assert len(center)==2
            return center #X,Y
    
    def detect_player(self,img):
        center=0
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detector=self.april_detector
        result=detector.detect(gray) 
        if(len(result)==0):
            center=0
        else:
            center=result[0].center
            center=(int(center[0]),int(center[1]))
        return center
    
    def display_img(self,ball_center,player_center,img):
        if(ball_center!=0):
            cv2.circle(img, ball_center,10,(0,0,255), -1)
        if(player_center!=0):
            cv2.circle(img, player_center,10,(255,0,0), -1)
        cv2.imshow('af',img)
        cv2.waitKey(1)
    
    def grab_img(self):
        cam.grab_img()
        r_img=cv2.imread(self.root_path+'right.jpg')
        l_img=cv2.imread(self.root_path+'left.jpg')
        return (l_img,r_img)
    
    def yolo_draw_img(self,x, results):
        c1 = tuple(x[1:3].int())
        c2 = tuple(x[3:5].int())
        img = results[int(x[0])]
        x1=c1[0]
        y1=c1[1]
        x2=c2[0]
        y2=c2[1]
        x=abs(x1-x2)/2+min(x1,x2)
        y=abs(y1-y2)/2+min(y1,y2)
        center=(int(x),int(y))
        return center
    
    def close_cam(self):
        cam.close_cam()

'''
camera=Camera()
ball_center=0
player_center=0
while(1):
    (l,r)=camera.grab_img()
    ball_center=camera.detect_ball(l)
    #player_center=detect_player(april_detector)
    #camera.display_img(ball_center,player_center,l)
'''

























