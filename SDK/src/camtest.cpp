#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <boost/python.hpp>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "optorusb.h"
#include "optorcam.h"
#include "optorimu.h"


using namespace std;
using namespace cv;
bool close_img_viewer=false;
bool visensor_Close_IMU_viewer=false;

timeval left_stamp,right_stamp;

int grab_img()
{
    Mat img_left(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
    double left_timestamp;
    Mat img_right(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
    double right_timestamp;
    visensor_imudata img_imudata;
    
    if(visensor_is_leftcam_open() && visensor_is_rightcam_open())
    {
        if(visensor_is_left_img_new())
        {
            visensor_get_left_latest_img(img_left.data,&left_timestamp,&img_imudata);
            //printf("L-Time: %8.6f, IMUTime: %8.6f\n",left_timestamp,img_imudata.timestamp);
            //imshow("left",img_left);
            imwrite("/home/wayne/Capstone/ece_capstone/SDK/build/left.jpg",img_left);
        }
        if(visensor_is_right_img_new())
        {
            visensor_get_right_latest_img(img_right.data,&right_timestamp);
            //printf("R-Time: %8.6f\n",right_timestamp);
            //imshow("right",img_right);
            imwrite("/home/wayne/Capstone/ece_capstone/SDK/build/right.jpg",img_right);
        }
    }
    return 0;
}

int grabImg_right()
{
    Mat img_right(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
    double right_timestamp;
    if(visensor_is_rightcam_open())
    {
        if(visensor_is_right_img_new())
        {
            visensor_get_right_latest_img(img_right.data,&right_timestamp);
            //printf("R-Time: %8.6f\n",right_timestamp);
            //imshow("right",img_right);
            imwrite("right.jpg",img_right);
        }
    }
    return 0; 
    //waitKey(1);
}


void* show_imuData(void *)
{
    visensor_imudata imudata;
    while(!visensor_Close_IMU_viewer)
    {
        if(visensor_imu_have_fresh_data())
        {
            visensor_get_imudata_latest(&imudata);
        }
        usleep(100);
    }
    pthread_exit(NULL);
}

int cam_run()
{
    visensor_load_settings("/home/wayne/Capstone/ece_capstone/SDK/optor_VISensor_Setups.txt");
    
    int r = visensor_Start_Cameras();
    if(r<0)
    {
        printf("Opening cameras failed...\r\n");
        return 1;
    }
     
    /*int fd=visensor_Start_IMU();
    if(fd<0)
    {
        printf("visensor_open_port error...\r\n");
        return 1;
    1*/
    printf("visensor_open_port success...\r\n");
    
    //usleep(100000);

    //Create img_show thread
    /* 
    pthread_t showimg_thread;
    int temp;
    if(temp = pthread_create(&showimg_thread, NULL, opencv_showimg, NULL))
        printf("Failed to create thread opencv_showimg\r\n");
    //Create show_imuData thread
    pthread_t showimu_thread;
    if(temp = pthread_create(&showimu_thread, NULL, show_imuData, NULL))
        printf("Failed to create thread show_imuData\r\n");
    */
    /*while(1)
    {
        // Do - Nothing :)
        sleep(1);
    }

    close_img_viewer=true;
    visensor_Close_IMU_viewer=true;
    if(showimg_thread !=0)
    {
        pthread_join(showimg_thread,NULL);
    }
    if(showimu_thread !=0)
    {
        pthread_join(showimu_thread,NULL);
    }

    visensor_Close_Cameras();
    
    visensor_Close_IMU();
    */
    return 0;
}

void cam_close()
{
    close_img_viewer=true;
    visensor_Close_IMU_viewer=true;
    visensor_Close_Cameras();
    visensor_Close_IMU();
}

BOOST_PYTHON_MODULE(cam)
{
    using namespace boost::python;
    def("cam_run", cam_run);
    def("close_cam",cam_close);
    def("grab_img",grab_img);
    def("grab_img_right",grabImg_right);
}


