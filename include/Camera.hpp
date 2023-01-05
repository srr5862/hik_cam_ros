#ifndef CAMERA_H
#define CAMERA_H

#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "ros/ros.h"

using namespace std;
namespace camera
{
    cv::Mat frame;
    bool frame_empty = 0;
    pthread_mutex_t mutex;
    #define MAX_IMAGE_DATA_SIZE (4 * 3648 * 5472)
    
    enum CameraProperties{
        CAM_PROP_FRAMERATEEnable,
        CAM_PROP_FRAMERATE,
        CAM_PROP_BURSTFRAMECOUNT,
        CAM_PROP_HEIGHT,
        CAM_PROP_WIDTH,
        CAM_PROP_TRIGGER_MODE,
        CAM_PROP_TRIGGER_SOURCE
    };

    class Camera
    {
    public:
        Camera(ros::NodeHandle &node);
        ~Camera();
        static void *workthread(void *pUser);
        void ReadImg(cv::Mat &img);
        bool set(camera::CameraProperties type,float value);

    private:
        void *handle;
        pthread_t threadID;
        int nRet;
        int height;
        int width;
        int BrustFrameCount;
        bool FrameRateEnable;
        int FrameRate;
        int TriggerMode;
        int TriggerSource;
    };
    Camera::Camera(ros::NodeHandle &node)
    {
        handle = NULL;
        node.param("width",width,5472);
        node.param("height",height,3648);
        node.param("FrameRateEnable",FrameRateEnable,true);
        node.param("FrameRate",FrameRate,60);
        node.param("BrustFrameCount",BrustFrameCount,1);
        node.param("TriggerMode",TriggerMode,1);
        node.param("TriggerSouce",TriggerSource,7);


        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            cout << "MV_CC_EnumDevices fail" << endl;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                cout << "device " << i << endl;
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (pDeviceInfo == NULL)
                {
                    break;
                }
            }
        }
        else
        {
            cout << "can not find device" <<endl;
        }
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
        if (MV_OK != nRet)
        {
            cout << "create handel failed" <<endl;
        }
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            cout << "open device error" <<endl;
        }

        this->set(CAM_PROP_WIDTH,width);
        this->set(CAM_PROP_HEIGHT,height);
        this->set(CAM_PROP_FRAMERATEEnable,FrameRateEnable);
        if (FrameRateEnable)    this->set(CAM_PROP_FRAMERATE,FrameRate);
        this->set(CAM_PROP_BURSTFRAMECOUNT,BrustFrameCount);
        this->set(CAM_PROP_TRIGGER_MODE,TriggerMode);
        this->set(CAM_PROP_TRIGGER_SOURCE,TriggerSource);
        nRet = MV_CC_SetEnumValue(handle,"TriggerMode",0);
        if (MV_OK == nRet){
            cout << "success set trigger" <<endl;
        }
        cout << "start grapping" <<endl;
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet){
            cout << "grabbing error" <<endl;
        }
        nRet = pthread_mutex_init(&mutex,NULL);
        if (nRet != 0)
        {
            cout << "thread create error"<<endl;
            exit(-1);
        }
        nRet = pthread_create(&threadID,NULL,workthread,handle);

        
    }
    Camera::~Camera() {
        int nRet;
        pthread_join(threadID,NULL);
        nRet = MV_CC_StopGrabbing(handle);
        nRet = MV_CC_CloseDevice(handle);
        nRet = MV_CC_DestroyHandle(handle);
        pthread_mutex_destroy(&mutex);
    }

    bool Camera::set(camera::CameraProperties type,float value){
        switch (type)
        {
        case CAM_PROP_WIDTH:
            nRet = MV_CC_SetIntValue(handle,"Width",value);
            if (nRet != MV_OK)  cout << "width error" <<endl;
            break;
        case CAM_PROP_HEIGHT:
            nRet = MV_CC_SetIntValue(handle,"Height",value);
            if (nRet != MV_OK)  cout << "height error" <<endl;
            break;
        case CAM_PROP_FRAMERATEEnable:
            nRet = MV_CC_SetBoolValue(handle,"AcquisitionFrameRateEnable",value);
            if (nRet != MV_OK)  cout << "framerateenable  error" <<endl;
            break;
        case CAM_PROP_FRAMERATE:
            nRet = MV_CC_SetFloatValue(handle,"AcquisitionFrameRate",value);
            if (nRet != MV_OK)  cout << "framerate error" <<endl;
            break;
        case CAM_PROP_BURSTFRAMECOUNT:
            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);
            if (nRet != MV_OK)  cout << "BurstFrameCount error" <<endl;
            break;
        case CAM_PROP_TRIGGER_MODE:
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value);
            if (nRet != MV_OK)  cout << "TriggerMode error" <<endl;
            break;
        case CAM_PROP_TRIGGER_SOURCE:
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value);
            if (nRet != MV_OK)  cout << "TriggerSource error" <<endl;
            break;
        
        default:
            return 0;
        }
        return nRet;

    }

    void* Camera::workthread(void* pUser) 
    {
        int nRet = 1;
        int empty_frame = 0;
        unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char * pDataForRGB = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MVCC_INTVALUE stParam;
        double startTime;

        // memset(&stParam,0,sizeof(MVCC_INTVALUE));
        // nRet = MV_CC_GetIntValue(pUser,"PayloadSize",&stParam);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        // memset(&stImageInfo,0,sizeof(MV_FRAME_OUT_INFO_EX));

        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        
        // unsigned int nDataSize = stParam.nCurValue;
        if (NULL == pData){
            cout << "pData is None" <<endl;
            return NULL;
        }   
       
        while(ros::ok()){
            // nRet = MV_CC_SetCommandValue(pUser,"TriggerSoftware");
        //     if (MV_OK != nRet){
        //         cout << "set error"<< nRet <<endl;
        //         break;
        // }
            startTime = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(pUser,pData,MAX_IMAGE_DATA_SIZE,&stImageInfo,60);
            if( nRet != MV_OK){
                if(++empty_frame > 10000){
                    ROS_INFO("There are more than 10000+ empty frame\n");
                    exit(-1);
                }
                continue;
            }
            
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = pDataForRGB;
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;
            nRet = MV_CC_ConvertPixelType(pUser,&stConvertParam);
            pthread_mutex_lock(&mutex);
            camera::frame = cv::Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC3,pDataForRGB).clone();
            frame_empty = 0;
            // cv::cvtColor(camera::frame,camera::frame,cv::COLOR_BGR2RGB);
            pthread_mutex_unlock(&mutex);
            double time = ((double)cv::getTickCount() - startTime) / cv::getTickFrequency();
            cout << "FPS:" << 1 / time << endl;
        }
        free(pDataForRGB);
        free(pData);
        return 0;

    }  
    void Camera::ReadImg(cv::Mat &img){
        pthread_mutex_lock(&mutex);
        if(frame_empty){
            img = cv::Mat();
        }else{
            img = camera::frame;
            frame_empty = 1;
            
        }
        pthread_mutex_unlock(&mutex);

    }

}
#endif