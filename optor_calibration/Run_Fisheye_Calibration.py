import cv2
import numpy as np
import glob
import save_calibration as sc
import cam_calibration as cc
import extract_template as et
import json
import time
from matplotlib import pyplot as plt

def undistort(img_path,K,D,DIM):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    #P=cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM,np.eye(3))
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_32FC1)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

def fisheye_calibrate(img_path,board_size,VISUALIZE,save_result,save_dir,size,undistort_prefix=''):
	CHECKERBOARD=board_size
	subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 26, 0.1)
	calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_FIX_SKEW+cv2.fisheye.CALIB_CHECK_COND
	objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float64)
	objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
	objp=objp*size
	_img_shape = None
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.
	
	for fname in img_path:
		
		img = cv2.imread(fname)
		if _img_shape == None:
			_img_shape = img.shape[:2]
		else:
			assert _img_shape == img.shape[:2], "All images must share the same size."
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
		# If found, add object points, image points (after refining them)
		if ret == True:
			objpoints.append(objp)
			corners2=cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
			imgpoints.append(corners2)
			if(VISUALIZE):
				img = cv2.drawChessboardCorners(img, (CHECKERBOARD[0],CHECKERBOARD[1]), corners2,ret)
				cv2.imshow('img',img)
				cv2.waitKey(500)
	N_OK = len(objpoints)
	K = np.zeros((3, 3))
	D = np.zeros((4, 1))
	rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
	tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
	rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(objpoints,imgpoints,
	        gray.shape[::-1],K,D,rvecs,tvecs,calibration_flags,
	        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
	
	if(save_result):
		count=0
		for fname in left_images:
			dst=undistort(fname,K,D,_img_shape[::-1  ])
			cv2.imwrite(save_dir+undistort_prefix+'_undistort'+str(count)+'.jpg',dst)
			count+=1
	imgpoints=np.asarray(imgpoints,dtype=np.float64)
	objpoints=np.asarray(objpoints,dtype=np.float64)

	objpoints = np.reshape(objpoints, (N_OK, 1, CHECKERBOARD[0]*CHECKERBOARD[1], 3))
	imgpoints = np.reshape(imgpoints, (N_OK, 1, CHECKERBOARD[0]*CHECKERBOARD[1], 2))
	return rms,K,D,imgpoints,objpoints,_img_shape[::-1]


inputJSON='param_fish.json'

if __name__ == "__main__":
	fp = open(inputJSON)
	projectDict = json.load(fp)
	fp.close()
	left_img_folder    = projectDict["LEFT_IMG_DIR"]
	right_img_folder   = projectDict["RIGHT_IMG_DIR"]
	template_folder=projectDict["TEMPLATE_DIR"]
	BOARD_SIZE_R=projectDict["BOARD_ROW"]
	BOARD_SIZE_C=projectDict["BOARD_COL"]
	MATCH_THRESH=projectDict["MATCH_THRESH"]
	VISUALIZE=projectDict["VISUALIZE"]
	board_type=projectDict["BOARD_TYPE"]
	save_undistort_img=projectDict["SAVE_UNDISTORT_IMG"]
	save_result=projectDict["SAVE_CALIB_RESULT"]
	save_dir=projectDict["RESULT_DIR"]
	IMG_FORMAT=projectDict["IMG_FORMAT"]
	TEMPLATE_FORMAT=projectDict["TEMPLATE_FORMAT"]
	EXTRACT_TMP=projectDict["EXTRACT_TEMPLATE"]
	TEMPLATE_SIZE=projectDict["TEMPLATE_SIZE"]
	DISPARITY=projectDict["DISPARITY"]
	
	TARGET_SIZE=0.026
	calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_FIX_SKEW+cv2.fisheye.CALIB_FIX_INTRINSIC+cv2.fisheye.CALIB_CHECK_COND
	
	if(EXTRACT_TMP):
		et.extract_template(template_folder,img_folder,IMG_FORMAT,TEMPLATE_FORMAT,TEMPLATE_SIZE )
	#load calibration images and templates
	left_images=glob.glob(left_img_folder+'*'+IMG_FORMAT)
	right_images=glob.glob(right_img_folder+'*'+IMG_FORMAT)

	R = np.zeros((3, 3), dtype=np.float64)

	T = np.zeros(( 1, 3), dtype=np.float64)
	
	CHECKERBOARD = (BOARD_SIZE_C,BOARD_SIZE_R)
	left_rms,left_K,left_D,left_imgpoints,objpoints,img_size=fisheye_calibrate(left_images,CHECKERBOARD,VISUALIZE,save_result,save_dir,TARGET_SIZE,undistort_prefix='left')
	right_rms,right_K,right_D,right_imgpoints,objpoints,img_size=fisheye_calibrate(right_images,CHECKERBOARD,VISUALIZE,save_result,save_dir,TARGET_SIZE,undistort_prefix='right')
	
	
	error,left_K,left_D,right_K,right_D,R,T=cv2.fisheye.stereoCalibrate(objpoints,left_imgpoints,right_imgpoints,
						left_K,left_D,right_K,right_D,img_size,R,T,
						flags=calibration_flags)

	new_mtx_left=cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(left_K, left_D, img_size,np.eye(3))
	new_mtx_right=cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(right_K, right_D,img_size,np.eye(3))

	R1,R2,P1,P2,Q=cv2.fisheye.stereoRectify(left_K,left_D,right_K,right_D,img_size,R,T,balance=0,flags=cv2.CALIB_ZERO_DISPARITY)
	print(error)
	print(R)
	print(T)
	print(left_K)
	print(left_D)
	print(right_K)
	print(right_D)
	print(R1)
	print(R2)
	print(P1)
	print(P2)
	print(Q)
	
	map_left_1,map_left_2=cv2.fisheye.initUndistortRectifyMap(left_K,left_D,R1,P1,img_size,cv2.CV_32FC1)
	map_right_1,map_right_2=cv2.fisheye.initUndistortRectifyMap(right_K,right_D,R2,P2,img_size,cv2.CV_32FC1)

	
	
	img_t=cv2.imread(left_images[0])
	img_v=cv2.imread(right_images[0])
	img_rect_t=cv2.remap(img_t,map_left_1,map_left_2,interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	img_rect_v=cv2.remap(img_v,map_right_1,map_right_2,interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

	
	total_size = (max(img_rect_t.shape[0], img_rect_v.shape[0]),
                  img_rect_t.shape[1] + img_rect_v.shape[1], 3)
	img = np.zeros(total_size, dtype=np.uint8)
	img[:img_rect_t.shape[0], :img_rect_t.shape[1]] = img_rect_t
	img[:img_rect_v.shape[0], img_rect_t.shape[1]:] = img_rect_v
	# draw horizontal lines every 25 px accross the side by side image
	for i in range(20, img.shape[0], 25):
		cv2.line(img, (0, i), (img.shape[1], i), (255, 0, 0))
	
	cv2.imshow('imgRectified', img)
	#cv2.waitKey(0)
	cv2.imwrite('test.jpg',img)
	
	if(DISPARITY):
	    imgl = cv2.cvtColor(img_rect_t, cv2.COLOR_BGR2GRAY)
	    imgr = cv2.cvtColor(img_rect_v, cv2.COLOR_BGR2GRAY)
	    stereo = cv2.StereoBM_create(numDisparities=80, blockSize=15)
	    t=time.time()
	    disparity = stereo.compute(imgl,imgr)
	    
	    points=cv2.reprojectImageTo3D(disparity,Q,handleMissingValues=True)
	    print(time.time()-t)
	    plt.imshow(disparity,'gray')
	    plt.show()
	    cv2.imwrite('disparity.jpg',disparity)
	
	
	
	
