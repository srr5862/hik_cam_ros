#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "ros/ros.h"

namespace camera
{
    cv::Mat frame;
    bool frame_empty = 0;
    pthread_mutex_t mutex;

    class Camera
    {
    public:
        Camera(ros::NodeHandle &node);
        ~Camera();
        static void *workthread(void *pUser);
        void ReadImg(cv::Mat &img);

    private:
        void *handle;
        pthread_t threadID;
        int nRet;
    };
    Camera::Camera(ros::NodeHandle &node)
    {
        handle = NULL;
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail");
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (pDeviceInfo == NULL)
                {
                    break;
                }
            }
        }
        else
        {
            printf("find no device\n");
        }
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
        if (MV_OK != nRet)
        {
            printf("create camera error\n");
        }
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("open error\n");
        }
        nRet = MV_CC_SetEnumValue(handle,"TriggerMode",0);
        if (MV_OK == nRet){
            printf("set tirgger \n");
        }
        printf("start gtapping\n");
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet){
            printf("grabbing failed\n");
        }
        nRet = pthread_mutex_init(&mutex,NULL);
        if (nRet != 0)
        {
            perror("pthread_create failed\n");
            exit(-1);
        }
        nRet = pthread_create(&threadID,NULL,workthread,handle);
        printf("success thread\n");
        
    }
    Camera::~Camera() {
        int nRet;
        pthread_join(threadID,NULL);
        nRet = MV_CC_StopGrabbing(handle);
        nRet = MV_CC_CloseDevice(handle);
        nRet = MV_CC_DestroyHandle(handle);
        pthread_mutex_destroy(&mutex);
    }
    static void* Camera::workthread(void* pUser) {
        int nRet;
        int empty_frame = 0;
        unsigned char*  pDataForRGB = NULL;
        MVCC_INTVALUE stParam;
        nRet = MV_CC_GetIntValue(pUser,"PayloadSize",&stParam);
        memset(&stParam,0,sizeof(MVCC_INTVALUE));
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        memset(&stImageInfo,0,sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData){
        printf("pData is NULL\n");
        return NULL;
        }   
        unsigned int nDataSize = stParam.nCurValue;
        while(ros::ok()){
            nRet = MV_CC_GetOneFrameTimeout(pUser,pData,nDataSize,&stImageInfo,15);
            if( nRet != MV_OK){
                if(++empty_frame > 100){
                    ROS_INFO("There are more than 100+ empty frame\n");
                    exit(-1);
                }
                continue;
            }
            pDataForRGB = (unsigned char *)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = pDataForRGB;
            stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight *  4 + 2048;
            nRet = MV_CC_ConvertPixelType(pUser,&stConvertParam);
            pthread_mutex_lock(&mutex);
            camera::frame = cv::Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC3,pDataForRGB).clone();
            cv::cvtColor(camera::frame,camera::frame,cv::COLOR_BGR2RGB);
            pthread_mutex_unlock(&mutex);
            cv::imshow("test",frame);
        }

    }

}