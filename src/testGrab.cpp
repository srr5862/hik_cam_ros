#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"


using namespace std;

bool g_bExit = false;

unsigned char * pDataForRGB = NULL;
void PressEnterToExit(void){
    int c;
    while( (c = getchar()) != '\n' && c != EOF);
    fprintf(stderr,"\n press enter to exit\n");
    while(getchar() != '\n');
    g_bExit = true;
    sleep(1);
}


static void* workthread(void* pUser){
    int nRet = MV_OK;
    MVCC_INTVALUE stParam;
    memset(&stParam,0,sizeof(MVCC_INTVALUE));
    //获取数据包大小
    nRet = MV_CC_GetIntValue(pUser,"PayloadSize",&stParam);
    printf("currentValue is %d\n",stParam.nCurValue);
    if (MV_OK != nRet){
        printf("get payload failed\n");
        return NULL;
    }
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo,0,sizeof(MV_FRAME_OUT_INFO_EX));
    printf("currentValue is %d\n",stParam.nCurValue);
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData){
        printf("pData is NULL\n");
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;
    while(1){
        nRet = MV_CC_SetCommandValue(pUser,"TriggerSoftware");
        if (MV_OK != nRet){
            printf("set trigger software failed\n");
            break;
        }
        nRet = MV_CC_GetOneFrameTimeout(pUser,pData,nDataSize,&stImageInfo,1000);
        if (MV_OK == nRet){ 
            printf("success get frame width[%d],height[%d],nFrameNum[%d]\n",stImageInfo.nWidth,stImageInfo.nHeight,stImageInfo.nFrameNum);
            printf("start convert\n");
            pDataForRGB = (unsigned char *)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
            if (NULL == pDataForRGB){
                printf("malloc failed\n");
                break;
            }
            MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = pDataForRGB;
            stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight *  4 + 2048;
            nRet = MV_CC_ConvertPixelType(pUser,&stConvertParam);   
            if (MV_OK != nRet){
                printf("save faile\n");
                break;
            }
            cv::Mat img =cv::Mat(stImageInfo.nHeight,stImageInfo.nWidth,CV_8UC3,pDataForRGB);
            cv::cvtColor(img,img,cv::COLOR_RGB2BGR);
        }else{
            printf("get one frame failed\n");
            break;  
        }
        if(g_bExit) break;
    }
}


int main(){
    int nRet = MV_OK;
    void* handle = NULL;
    do
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList,0,sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE,&stDeviceList);
        if (MV_OK != nRet){
            printf("MV_CC_EnumDevices fail");
            break;
        }
        if (stDeviceList.nDeviceNum > 0){
            for(int i = 0; i < stDeviceList.nDeviceNum; i++){
                printf("[device %d]:\n",i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (pDeviceInfo == NULL){
                    break;
                }
            }
        }else{
            printf("find no device\n");
            break;
        }
        unsigned int nIndex = 0;
        printf("input camera index:\n");
        scanf("%d",&nIndex);
        nRet = MV_CC_CreateHandle(&handle,stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet){
            printf("create camera error\n");
            break;
        }
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet){
            printf("open error\n");
            break;
        }
        nRet = MV_CC_SetEnumValue(handle,"TriggerMode",1);
        if (MV_OK != nRet){
            printf("set trigger mode failed\n");
            break;
        }else{
            printf("set trigger result%d\n",nRet);
        }
        nRet = MV_CC_SetEnumValue(handle,"TriggerSource",MV_TRIGGER_SOURCE_SOFTWARE);
        if (MV_OK != nRet){
            printf("set trigger source failed\n");
            break;
        }else{
            printf("trigger source is %d\n",MV_TRIGGER_SOURCE_SOFTWARE);
        }
        printf("start gtapping\n");
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet){
            printf("grabbing failed\n");
            break;
        }
        pthread_t nthreadID;
        nRet = pthread_create(&nthreadID,NULL,workthread,handle);
        printf("nRet is %d\n",nRet);
        PressEnterToExit(); 



    } while (0);
    

}