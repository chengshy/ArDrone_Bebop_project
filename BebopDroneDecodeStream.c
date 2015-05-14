/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
/**
 * @file BebopDroneDecodeStream.c
 * @brief This file contains sources about basic arsdk example decoding video stream from a BebopDrone with ffmpeg
 * @date 08/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>

#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream/ARStream.h>

#include "BebopDroneDecodeStream.h"
//++++++++++++++ More Headers ++++++++++
#include <time.h>
#include <math.h>
//++++++++++++++++++++++++++++++++++++++
/*****************************************
 *
 *             define :
 *
 *****************************************/
#define TAG "BebopDroneReceiveStream"
#define BD_IP_ADDRESS "192.168.42.1"
#define BD_DISCOVERY_PORT 44444
#define BD_C2D_PORT 54321 // should be read from Json
#define BD_D2C_PORT 43210

#define BD_NET_CD_NONACK_ID 10
#define BD_NET_CD_ACK_ID 11
#define BD_NET_CD_EMERGENCY_ID 12
#define BD_NET_CD_VIDEO_ACK_ID 13
#define BD_NET_DC_NAVDATA_ID 127
#define BD_NET_DC_EVENT_ID 126
#define BD_NET_DC_VIDEO_DATA_ID 125

#define BD_NET_DC_VIDEO_FRAG_SIZE 1000
#define BD_NET_DC_VIDEO_MAX_NUMBER_OF_FRAG 128

#define BD_RAW_FRAME_BUFFER_SIZE 50
#define BD_RAW_FRAME_POOL_SIZE 50

#define ERROR_STR_LENGTH 2048
#define PI 3.1415926

int getNextDataCallback(uint8_t **data, void *customData);
void* Decode_RunDataThread(void *customData);
RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager);
void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame);
void flushFifo(BD_MANAGER_t *deviceManager);
void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx);
RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data);
/**
 * @brief Component of a frame.
 */
struct RawFrame_t
{
    uint8_t *data; /**< data buffer*/
    uint32_t size; /**< size of the buffer */
    uint8_t isIframe;
};

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

static ARNETWORK_IOBufferParam_t c2dParams[] = {
    /* Non-acknowledged commands. */
    {
        .ID = BD_NET_CD_NONACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 2,
        .dataCopyMaxSize = 128,
        .isOverwriting = 1,
    },
    /* Acknowledged commands. */
    {
        .ID = BD_NET_CD_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Emergency commands. */
    {
        .ID = BD_NET_CD_EMERGENCY_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 10,
        .ackTimeoutMs = 100,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 1,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Video ACK (Initialized later) */
    {
        .ID = BD_NET_CD_VIDEO_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
        .sendingWaitTimeMs = 0,
        .ackTimeoutMs = 0,
        .numberOfRetry = 0,
        .numberOfCell = 0,
        .dataCopyMaxSize = 0,
        .isOverwriting = 0,
    },
};
static const size_t numC2dParams = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);

static ARNETWORK_IOBufferParam_t d2cParams[] = {
    {
        .ID = BD_NET_DC_NAVDATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    {
        .ID = BD_NET_DC_EVENT_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Video data (Initialized later) */
    {
        .ID = BD_NET_DC_VIDEO_DATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_UNINITIALIZED,
        .sendingWaitTimeMs = 0,
        .ackTimeoutMs = 0,
        .numberOfRetry = 0,
        .numberOfCell = 0,
        .dataCopyMaxSize = 0,
        .isOverwriting = 0,
    },
};
static const size_t numD2cParams = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);

static int commandBufferIds[] = {
    BD_NET_DC_NAVDATA_ID,
    BD_NET_DC_EVENT_ID,
};
static const size_t numOfCommandBufferIds = sizeof(commandBufferIds) / sizeof(int);

int gIHMRun = 0;
char gErrorStr[ERROR_STR_LENGTH];

//+++++++++++++++++++++++++++ More Instances +++++++++++
clock_t startTime, currentTime;
double timePassed;
//++++++++++++++++++++++++++++++++++++++++++++++++++

// reader thread
void *readerRun (void* data)
{
    BD_MANAGER_t *deviceManager = NULL;
    int bufferId = 0;
    int failed = 0;
    
    // Allocate some space for incoming data.
    const size_t maxLength = 128 * 1024;
    void *readData = malloc (maxLength);
    if (readData == NULL)
    {
        failed = 1;
    }
    
    if (!failed)
    {
        // get thread data.
        if (data != NULL)
        {
            bufferId = ((READER_THREAD_DATA_t *)data)->readerBufferId;
            deviceManager = ((READER_THREAD_DATA_t *)data)->deviceManager;
            
            if (deviceManager == NULL)
            {
                failed = 1;
            }
        }
        else
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        while (deviceManager->run)
        {
            eARNETWORK_ERROR netError = ARNETWORK_OK;
            int length = 0;
            int skip = 0;
            
            // read data
            netError = ARNETWORK_Manager_ReadDataWithTimeout (deviceManager->netManager, bufferId, readData, maxLength, &length, 1000);
            if (netError != ARNETWORK_OK)
            {
                if (netError != ARNETWORK_ERROR_BUFFER_EMPTY)
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNETWORK_Manager_ReadDataWithTimeout () failed : %s", ARNETWORK_Error_ToString(netError));
                }
                skip = 1;
            }
            
            if (!skip)
            {
                // Forward data to the CommandsManager
                eARCOMMANDS_DECODER_ERROR cmdError = ARCOMMANDS_DECODER_OK;
                cmdError = ARCOMMANDS_Decoder_DecodeBuffer ((uint8_t *)readData, length);
                if ((cmdError != ARCOMMANDS_DECODER_OK) && (cmdError != ARCOMMANDS_DECODER_ERROR_NO_CALLBACK))
                {
                    char msg[128];
                    ARCOMMANDS_Decoder_DescribeBuffer ((uint8_t *)readData, length, msg, sizeof(msg));
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARCOMMANDS_Decoder_DecodeBuffer () failed : %d %s", cmdError, msg);
                }
            }
        }
    }
    
    if (readData != NULL)
    {
        free (readData);
        readData = NULL;
    }
    
    return NULL;
}

// decoder thread
void* Decode_RunDataThread(void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARCODECS_ERROR error;
    ARCODECS_Manager_Frame_t *decodedFrame = NULL;
    while (!deviceManager->decodingCanceled)
    {
        ARSAL_Mutex_Lock (&(deviceManager->mutex));
        RawFrame_t *rawFrame = deviceManager->rawFrameFifo[deviceManager->fifoReadIdx];
        ARSAL_Mutex_Unlock (&(deviceManager->mutex));
        
        if (rawFrame != NULL && rawFrame->isIframe == 1)
        {
            deviceManager->hasReceivedFirstIFrame = 1;
        }
        
        if (rawFrame != NULL)
        {
            if (deviceManager->hasReceivedFirstIFrame)
            {
                decodedFrame = ARCODECS_Manager_Decode(deviceManager->decoder, &error);
            }
            
            if (decodedFrame != NULL)
            {
                //ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Frame has been decoded ");
                
                // this part is only needed to copy the YUV frame into the file
                int pic_size = avpicture_get_size(PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height);
                
                uint8_t *decodedOut = malloc(pic_size);
                
                // in case your install of FFMpeg supports av_image_copy_to_buffer, you can replace the AVFrame creation and initialisation by this block
                /*uint8_t *src_data[4];
                src_data[0] = decodedFrame->componentArray[0].data;
                src_data[1] = decodedFrame->componentArray[1].data;
                src_data[2] = decodedFrame->componentArray[2].data;
                src_data[3] = NULL;
                
                int src_linesize[4];
                src_linesize[0] = decodedFrame->componentArray[0].lineSize;
                src_linesize[1] = decodedFrame->componentArray[1].lineSize;
                src_linesize[2] = decodedFrame->componentArray[2].lineSize;
                src_linesize[3] = 0;
                
                av_image_copy_to_buffer(decodedOut, pic_size,
                                        (const uint8_t *const *)src_data, src_linesize,
                                        PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height, 1);*/
                
                AVFrame *avFrame = avcodec_alloc_frame();
                if (avFrame != NULL)
                {
                    avFrame->width = decodedFrame->width;
                    avFrame->height = decodedFrame->height;
                    avFrame->format = AV_PIX_FMT_YUV420P;

                    avpicture_fill((AVPicture*)avFrame, NULL, PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height);
                    avFrame->linesize[0] = decodedFrame->componentArray[0].lineSize;
                    avFrame->linesize[1] = decodedFrame->componentArray[1].lineSize;
                    avFrame->linesize[2] = decodedFrame->componentArray[2].lineSize;
                    
                    avFrame->data[0] = decodedFrame->componentArray[0].data;
                    avFrame->data[1] = decodedFrame->componentArray[1].data;
                    avFrame->data[2] = decodedFrame->componentArray[2].data;

                    avpicture_layout((AVPicture*)avFrame, PIX_FMT_YUV420P, decodedFrame->width, decodedFrame->height, decodedOut, pic_size);
                    avcodec_free_frame(&avFrame);
                }
                
                if (decodedOut != NULL)
                {
                    fwrite(decodedOut, pic_size, 1, deviceManager->video_out);
                }
                
                free (decodedOut);
                decodedOut = NULL;
            }
            
            ARSAL_Mutex_Lock (&(deviceManager->mutex));
            putRawFrameBackToPool(deviceManager, deviceManager->fifoReadIdx);
            deviceManager->fifoReadIdx = (deviceManager->fifoReadIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
            ARSAL_Mutex_Unlock (&(deviceManager->mutex));
        }
    }
    
    return (void*)0;
}

// looper thread
void *looperRun (void* data)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)data;
    
    if(deviceManager != NULL)
    {
        while (deviceManager->run)
        {   
            //++++++++++++ Read Time ++++++++++
            currentTime = clock();
            timePassed = (double) (currentTime - startTime)/CLOCKS_PER_SEC;
            deviceManager->timePassed = timePassed;
            IHM_PrintTimePassed(deviceManager->ihm,deviceManager->timePassed);
            if(deviceManager->logingFlag){
                fprintf(deviceManager->logFileCallback,"%f    %f    %f    %f    %f    %f    %f    %f    \n",  
                        deviceManager->timePassed, deviceManager->speed.speedX,deviceManager->speed.speedY,deviceManager->speed.speedZ,
                        deviceManager->pos.roll, deviceManager->pos.pitch, deviceManager->pos.yaw, deviceManager->pos.altitude);

                fprintf(deviceManager->logFileControlA,"%f %f %f %f %f %f %f %f %f %f \n",  
                        deviceManager->timePassed, deviceManager->desiredStates.x_posd, deviceManager->desiredStates.y_posd,deviceManager->desiredStates.z_posd,
                        deviceManager->desiredStates.x_veld, deviceManager->desiredStates.y_veld, deviceManager->desiredStates.z_veld,
                        deviceManager->odometry.x_pos, deviceManager->odometry.y_pos, deviceManager->odometry.z_pos);

                fprintf(deviceManager->logFileControlB, "%f %d     %d     %d     %f %f %f %f\n", deviceManager->timePassed, deviceManager->dataPCMD.roll, deviceManager->dataPCMD.pitch, deviceManager->dataPCMD.gaz,
                        deviceManager->droneSpeed.speedX, deviceManager->droneSpeed.speedY, deviceManager->droneSpeed.speedZ, deviceManager->flyStartYaw);

                fprintf(deviceManager->logFileControlC, "%f %f %f %f %f\n", deviceManager->timePassed, deviceManager->printData.x_veld, deviceManager->printData.y_veld, deviceManager->printData.pitch_error, deviceManager->printData.roll_error);
            }
            //+++++++++++++++++++++++++++++++++ 
            
            sendPCMD(deviceManager);
            
            sendCameraOrientation(deviceManager);
            usleep(50000);
        }
    }
    
    return NULL;
}

int main (int argc, char *argv[])
{
    /* local declarations */
    int failed = 0;
    BD_MANAGER_t *deviceManager = malloc(sizeof(BD_MANAGER_t));

    pid_t child = 0;
    
    // fork the process to launch ffplay
    if ((child = fork()) == 0)
    {
        // redirect stdout and stderr of mplayer to dev/null to avoid messing with ncurse
        int stdout_fd = open("/dev/null", O_RDONLY);
        if (stdout_fd == -1)
            return -1;
        dup2(stdout_fd, STDOUT_FILENO);
        close(stdout_fd);
        
        int stderr_fd = open("/dev/null", O_RDONLY);
        if (stderr_fd == -1)
            return -1;
        dup2(stderr_fd, STDERR_FILENO);
        close(stderr_fd);

        execlp("mplayer", "mplayer", "./video_decoded_fifo", "-demuxer", "rawvideo", "-rawvideo", "w=640:h=368:fps=30:format=i420", ">/dev/null", "2>/dev/null", NULL);
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Missing mplayer, you will not see the video. Please install mplayer.");
        return -1;
    }
    
    if (deviceManager == NULL)
    {
        failed = 1;
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "deviceManager alloc error !");
    }

    if (!failed)
    {
        ARSAL_PRINT (ARSAL_PRINT_INFO, TAG, "-- Starting --");
        
        // initialize jsMnager
        deviceManager->alManager = NULL;
        deviceManager->netManager = NULL;
        deviceManager->streamReader = NULL;
        deviceManager->looperThread = NULL;
        deviceManager->rxThread = NULL;
        deviceManager->txThread = NULL;
        deviceManager->videoRxThread = NULL;
        deviceManager->videoTxThread = NULL;
        deviceManager->d2cPort = BD_D2C_PORT;
        deviceManager->c2dPort = BD_C2D_PORT; //deviceManager->c2dPort = 0; // Should be read from json
        deviceManager->arstreamAckDelay = 0; // Should be read from json
        deviceManager->arstreamFragSize = BD_NET_DC_VIDEO_FRAG_SIZE; // Should be read from json
        deviceManager->arstreamFragNb   = BD_NET_DC_VIDEO_MAX_NUMBER_OF_FRAG; // Should be read from json
        deviceManager->video_out = fopen("./video_decoded_fifo", "w");
        deviceManager->decoder = NULL;
        deviceManager->decodingCanceled = 1;
        deviceManager->decodingThread = NULL;
        
        deviceManager->ihm = NULL;
        
        deviceManager->hasReceivedFirstIFrame = 0;
        
        deviceManager->freeRawFramePool = NULL;
        deviceManager->rawFramePoolCapacity = 0;
        deviceManager->lastRawFrameFreeIdx = 0;
        
        deviceManager->rawFrameFifo = NULL;
        deviceManager->fifoReadIdx = 0;
        deviceManager->fifoWriteIdx = BD_RAW_FRAME_BUFFER_SIZE - 1;
        deviceManager->run = 1;
        
        deviceManager->dataPCMD.flag = 0;
        deviceManager->dataPCMD.roll = 0;
        deviceManager->dataPCMD.pitch = 0;
        deviceManager->dataPCMD.yaw = 0;
        deviceManager->dataPCMD.gaz = 0;
        
        deviceManager->dataCam.tilt = 0;
        deviceManager->dataCam.pan = 0;
        
        deviceManager->flyingState = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_MAX;
        //+++++ States Added ++++++++++++++++++++++++++++
        deviceManager->speed.speedX = 0;
        deviceManager->speed.speedY = 0;
        deviceManager->speed.speedZ = 0;
        deviceManager->timePassed = 0;
        deviceManager->pos.roll = 0;
        deviceManager->pos.pitch = 0;
        deviceManager->pos.yaw = 0;
        deviceManager->pos.altitude = 0;
        deviceManager->autoStartTime = 0;

        deviceManager->pid_Coe.kPxy = 100; //100
        deviceManager->pid_Coe.kIxy = 0;
        deviceManager->pid_Coe.kDxy = 100; // 60
        deviceManager->pid_Coe.kPz = 225; // 300
        deviceManager->pid_Coe.kIz = 0;
        deviceManager->pid_Coe.kDz = 180; // 40
        
        deviceManager->logFileCallback = NULL;
        deviceManager->logFileControlA = NULL;
        deviceManager->logFileControlB = NULL;
        deviceManager->logFileControlC = NULL;
        deviceManager->logNum = 0;
        deviceManager->autoFly = 0;        // 0 for manully ; 1 for line; 2 for squre; 3 for circle; 4 for figure 8;
        deviceManager->logingFlag = 0;
        // deviceManager->wobbleEnable = 0;
        deviceManager->desiredStates.x_posd = 0;
        deviceManager->desiredStates.y_posd = 0;
        deviceManager->desiredStates.z_posd = 0;
        deviceManager->desiredStates.x_veld = 0;
        deviceManager->desiredStates.y_veld = 0;
        deviceManager->desiredStates.z_veld = 0;
        deviceManager->odometry.x_pos = 0;
        deviceManager->odometry.y_pos = 0;
        deviceManager->odometry.z_pos = 0;
        deviceManager->dtime = 0;
        deviceManager->previousTime = 0;
        deviceManager->droneSpeed.speedX = 0;
        deviceManager->droneSpeed.speedY = 0;
        deviceManager->droneSpeed.speedZ = 0;
        deviceManager->lineTrajCoe = malloc (sizeof(lineTrajCoe_t));
        deviceManager->squareTrajCoe = malloc(sizeof(squareTrajCoe_t));
        deviceManager->circleTrajCoe = malloc(sizeof(circleTrajCoe_t));
        deviceManager->eightTrajCoe = malloc(sizeof(eightTrajCoe_t));

        deviceManager->worldSpeed.speedX = 0;
        deviceManager->worldSpeed.speedY = 0;
        deviceManager->worldSpeed.speedZ = 0;
        deviceManager->flyStartYaw = 0;

        deviceManager->printData.x_veld = 0;
        deviceManager->printData.y_veld = 0;
        deviceManager->printData.pitch_error = 0;
        deviceManager->printData.roll_error = 0;
        deviceManager->flyStartAltitude = 0;
        //+++++++++++++++++++++++++++++++++++++++++++++++
    }
    
    if (!failed)
    {
        failed = ardiscoveryConnect (deviceManager);
    }

    if (!failed)
    {
        ARSTREAM_Reader_InitStreamDataBuffer (&d2cParams[2], BD_NET_DC_VIDEO_DATA_ID, deviceManager->arstreamFragSize, deviceManager->arstreamFragNb);
        ARSTREAM_Reader_InitStreamAckBuffer (&c2dParams[3], BD_NET_CD_VIDEO_ACK_ID);
    }

    if (!failed)
    {
        /* start */
        failed = startNetwork (deviceManager);
    }
    
    if (!failed)
    {
        failed = startDecoder(deviceManager);
    }

    if (!failed)
    {
        failed = startVideo (deviceManager);
    }
    
    if (!failed)
    {
        int cmdSend = sendDate(deviceManager);
        failed = !cmdSend;
    }
    
    if (!failed)
    {
        int cmdSend = sendAllSettings(deviceManager);
        failed = !cmdSend;
    }
    
    if (!failed)
    {
        int cmdSend = sendAllStates(deviceManager);
        failed = !cmdSend;
    }
    // //+++++++++++++++ Wobble cancellation state +++++++
    // if (!failed)
    // {
    //     int cmdSend = sendWobble(deviceManager);
    //     failed = !cmdSend;
    // }
    // //++++++++++++++++++++++++++++++++++++++++
    if (!failed)
    {
        int cmdSend = sendBeginStream(deviceManager);
        failed = !cmdSend;
    }

    if (!failed)
    {
        // allocate reader thread array.
        deviceManager->readerThreads = calloc(numOfCommandBufferIds, sizeof(ARSAL_Thread_t));
        
        if (deviceManager->readerThreads == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // allocate reader thread data array.
        deviceManager->readerThreadsData = calloc(numOfCommandBufferIds, sizeof(READER_THREAD_DATA_t));
        
        if (deviceManager->readerThreadsData == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Allocation of reader threads data failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // Create and start reader threads.
        int readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
        {
            // initialize reader thread data
            deviceManager->readerThreadsData[readerThreadIndex].deviceManager = deviceManager;
            deviceManager->readerThreadsData[readerThreadIndex].readerBufferId = commandBufferIds[readerThreadIndex];
            
            if (ARSAL_Thread_Create(&(deviceManager->readerThreads[readerThreadIndex]), readerRun, &(deviceManager->readerThreadsData[readerThreadIndex])) != 0)
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of reader thread failed.");
                failed = 1;
            }
        }
    }
    
    if (!failed)
    {
        // Create and start looper thread.
        startTime = clock();//++++++++++++++ Start Time Set ++++++++++++++
        if (ARSAL_Thread_Create(&(deviceManager->looperThread), looperRun, deviceManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of looper thread failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {
        deviceManager->ihm = IHM_New (&onInputEvent);
        if (deviceManager->ihm != NULL)
        {
            gErrorStr[0] = '\0';
            ARSAL_Print_SetCallback (customPrintCallback); //use a custom callback to print, for not disturb ncurses IHM
            
            IHM_PrintHeader(deviceManager->ihm, "-- Bebop Drone Decode Video Stream --");
            registerARCommandsCallbacks (deviceManager);
            IHM_setCustomData(deviceManager->ihm, deviceManager);
            
            gIHMRun = 1;
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of IHM failed.");
            failed = 1;
        }
    }
    
    if (!failed)
    {   
        IHM_PrintInstruction(deviceManager->ihm, "Arrow keys to move ; \n'a' = auto fly ; 'z' = up ; 's' = down ; 'q' = yaw left ; 'd' = yaw right; \nSpacebar to take off/land ; \n'k' = move camera down ; 'i' = move camera up ; 'j' = move camera left ; 'l' = move camera right ;\n'm' = EMERGENCY\n'esc' to quit");
        
        while (gIHMRun)
        {
            usleep(50);
        }
        
        IHM_PrintInfo(deviceManager->ihm, "Disconnecting ...");
    }

    if (deviceManager != NULL)
    {
        deviceManager->run = 0; // break threads loops

        // Stop looper Thread
        if (deviceManager->looperThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->looperThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->looperThread));
            deviceManager->looperThread = NULL;
        }
        
        /* stop */
        if (deviceManager->readerThreads != NULL)
        {
            // Stop reader Threads
            int readerThreadIndex = 0;
            for (readerThreadIndex = 0 ; readerThreadIndex < numOfCommandBufferIds ; readerThreadIndex++)
            {
                if (deviceManager->readerThreads[readerThreadIndex] != NULL)
                {
                    ARSAL_Thread_Join(deviceManager->readerThreads[readerThreadIndex], NULL);
                    ARSAL_Thread_Destroy(&(deviceManager->readerThreads[readerThreadIndex]));
                    deviceManager->readerThreads[readerThreadIndex] = NULL;
                }
            }
            
            // free reader thread array
            free (deviceManager->readerThreads);
            deviceManager->readerThreads = NULL;
        }
        
        if (deviceManager->readerThreadsData != NULL)
        {
            // free reader thread data array
            free (deviceManager->readerThreadsData);
            deviceManager->readerThreadsData = NULL;
        }

        stopVideo (deviceManager);
        stopDecoder (deviceManager);
        stopNetwork (deviceManager);
        fclose (deviceManager->video_out);
        free (deviceManager);
    }

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

    if (child > 0)
    {
        kill(child, SIGKILL);
    }

    return 0;
}

/************************** Connection part **************************/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- ARDiscovery Connection");
    
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData = ARDISCOVERY_Connection_New (ARDISCOVERY_Connection_SendJsonCallback, ARDISCOVERY_Connection_ReceiveJsonCallback, deviceManager, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while creating discoveryData : %s", ARDISCOVERY_Error_ToString(err));
        failed = 1;
    }
    
    if (!failed)
    {
        eARDISCOVERY_ERROR err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, BD_DISCOVERY_PORT, BD_IP_ADDRESS);
        if (err != ARDISCOVERY_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error while opening discovery connection : %s", ARDISCOVERY_Error_ToString(err));
            failed = 1;
        }
    }
    
    ARDISCOVERY_Connection_Delete(&discoveryData);
    
    return failed;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    
    if ((dataTx != NULL) && (dataTxSize != NULL) && (deviceManager != NULL))
    {
        *dataTxSize = sprintf((char *)dataTx, "{ \"%s\": %d,\n \"%s\": \"%s\",\n \"%s\": \"%s\" }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, deviceManager->d2cPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "toto",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "tata") + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }
    
    return err;
}

eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    
    if ((dataRx != NULL) && (dataRxSize != 0) && (deviceManager != NULL))
    {
        char *json = malloc(dataRxSize + 1);
        strncpy(json, (char *)dataRx, dataRxSize);
        json[dataRxSize] = '\0';
        
        //read c2dPort ...
        
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "    - ReceiveJson:%s ", json);
        
        free(json);
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }
    
    return err;
}

/************************** Network part **************************/
int startNetwork (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARNetwork");

    // Create the ARNetworkALManager
    deviceManager->alManager = ARNETWORKAL_Manager_New(&netAlError);
    if (netAlError != ARNETWORKAL_OK)
    {
        failed = 1;
    }

    if (!failed)
    {
        // Initilize the ARNetworkALManager
        netAlError = ARNETWORKAL_Manager_InitWifiNetwork(deviceManager->alManager, BD_IP_ADDRESS, BD_C2D_PORT, BD_D2C_PORT, 1);
        if (netAlError != ARNETWORKAL_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create the ARNetworkManager.
        deviceManager->netManager = ARNETWORK_Manager_New(deviceManager->alManager, numC2dParams, c2dParams, numD2cParams, d2cParams, pingDelay, onDisconnectNetwork, deviceManager, &netError);
        if (netError != ARNETWORK_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->rxThread), ARNETWORK_Manager_ReceivingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Rx thread failed.");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->txThread), ARNETWORK_Manager_SendingThreadRun, deviceManager->netManager) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of Tx thread failed.");
            failed = 1;
        }
    }

    // Print net error
    if (failed)
    {
        if (netAlError != ARNETWORKAL_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWorkAL Error : %s", ARNETWORKAL_Error_ToString(netAlError));
        }

        if (netError != ARNETWORK_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARNetWork Error : %s", ARNETWORK_Error_ToString(netError));
        }
    }

    return failed;
}

void stopNetwork (BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARNetwork");

    // ARNetwork cleanup
    if (deviceManager->netManager != NULL)
    {
        ARNETWORK_Manager_Stop(deviceManager->netManager);
        if (deviceManager->rxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->rxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->rxThread));
            deviceManager->rxThread = NULL;
        }

        if (deviceManager->txThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->txThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->txThread));
            deviceManager->txThread = NULL;
        }
    }

    if (deviceManager->alManager != NULL)
    {
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        ARNETWORKAL_Manager_CloseWifiNetwork(deviceManager->alManager);
    }

    ARNETWORK_Manager_Delete(&(deviceManager->netManager));
    ARNETWORKAL_Manager_Delete(&(deviceManager->alManager));
}

void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData)
{
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "onDisconnectNetwork ...");
    gIHMRun = 0;
}

/************************** Video part **************************/
int startVideo(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARSTREAM_ERROR err = ARSTREAM_OK;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Start ARStream");

    deviceManager->videoFrameSize = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
    deviceManager->videoFrame = malloc (deviceManager->videoFrameSize);
    if (deviceManager->videoFrame == NULL)
    {
        failed = 1;
    }
    
    if (! failed)
    {
        deviceManager->streamReader = ARSTREAM_Reader_New (deviceManager->netManager, BD_NET_DC_VIDEO_DATA_ID, BD_NET_CD_VIDEO_ACK_ID, frameCompleteCallback, deviceManager->videoFrame, deviceManager->videoFrameSize, deviceManager->arstreamFragSize, deviceManager->arstreamAckDelay, deviceManager, &err);
        if (err != ARSTREAM_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during ARStream creation : %s", ARSTREAM_Error_ToString(err));
            failed = 1;
        }
    }

    if (!failed)
    {
        // Create and start Tx and Rx threads.
        if (ARSAL_Thread_Create(&(deviceManager->videoRxThread), ARSTREAM_Reader_RunDataThread, deviceManager->streamReader) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video Rx thread failed.");
            failed = 1;
        }

        if (ARSAL_Thread_Create(&(deviceManager->videoTxThread), ARSTREAM_Reader_RunAckThread, deviceManager->streamReader) != 0)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video Tx thread failed.");
            failed = 1;
        }
    }

    return failed;
}

void stopVideo(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    eARSTREAM_ERROR err = ARSTREAM_OK;

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Stop ARStream");

    if (deviceManager->streamReader)
    {
        ARSTREAM_Reader_StopReader(deviceManager->streamReader);

        // Optionnal, but better for speed:
        ARNETWORKAL_Manager_Unlock(deviceManager->alManager);

        if (deviceManager->videoRxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->videoRxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->videoRxThread));
            deviceManager->videoRxThread = NULL;
        }
        if (deviceManager->videoTxThread != NULL)
        {
            ARSAL_Thread_Join(deviceManager->videoTxThread, NULL);
            ARSAL_Thread_Destroy(&(deviceManager->videoTxThread));
            deviceManager->videoTxThread = NULL;
        }

        ARSTREAM_Reader_Delete (&(deviceManager->streamReader));
    }
}

uint8_t *frameCompleteCallback (eARSTREAM_READER_CAUSE cause, uint8_t *frame, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *custom)
{
    uint8_t *ret = NULL;
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)custom;
    
    switch(cause)
    {
        case ARSTREAM_READER_CAUSE_FRAME_COMPLETE:
        {
            //ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Did receive a frame");
            
            RawFrame_t *freeFrame = getFrameFromData(deviceManager, frame);
            if (freeFrame != NULL)
            {
                freeFrame->data = frame;
                freeFrame->size = frameSize;
                freeFrame->isIframe = isFlushFrame;
            }
            
            if (isFlushFrame)
            {
                // since we received an iFrame, flush the fifo
                flushFifo(deviceManager);
            }

            if (freeFrame != NULL)
            {
                // push the received frame into the fifo
                freeFrame->data = frame;
                freeFrame->size = frameSize;
                
                addFreeRawFrameToFifo(deviceManager, freeFrame);
            }
            
            // get free frame for the next frame
            RawFrame_t *freeRawFrame = getFreeRawFrame(deviceManager);
            ret = freeRawFrame->data;
            *newBufferCapacity = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
        }

            break;
        case ARSTREAM_READER_CAUSE_FRAME_TOO_SMALL:
            /* This case should not happen, as we've allocated a frame pointer to the maximum possible size. */
        {
            RawFrame_t *freeRawFrame = getFreeRawFrame(deviceManager);
            ret = freeRawFrame->data;
            *newBufferCapacity = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
        }
            break;
        case ARSTREAM_READER_CAUSE_COPY_COMPLETE:
            /* Same as before ... but return value are ignored, so we just do nothing */
            break;
        case ARSTREAM_READER_CAUSE_CANCEL:
            /* Called when the library closes, return values ignored, so do nothing here */
            break;
        default:
            break;
    }

    return ret;
}

/************************** Commands part **************************/
int sendPCMD(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    // Send pcmd command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingPCMD(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->dataPCMD.flag, (uint8_t)deviceManager->dataPCMD.roll, deviceManager->dataPCMD.pitch, (uint8_t)deviceManager->dataPCMD.yaw, (uint8_t)deviceManager->dataPCMD.gaz, 0);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendCameraOrientation(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    // Send camera orientation command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3CameraOrientation(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->dataCam.tilt, (uint8_t)deviceManager->dataCam.pan);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendDate(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send date");
    
    // Send date command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentDate(cmdBuffer, sizeof(cmdBuffer), &cmdSize, "2015-04-20");
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    if (sentStatus)
    {
        // Send time command
        cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentTime(cmdBuffer, sizeof(cmdBuffer), &cmdSize, "'T'101533+0200");
        if (cmdError == ARCOMMANDS_GENERATOR_OK)
        {
            netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
        }
        
        if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
            sentStatus = 0;
        }
    }
    
    return sentStatus;
}

int sendAllSettings(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all settings");
    
    // Send get all settings command
    cmdError = ARCOMMANDS_Generator_GenerateCommonSettingsAllSettings(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all settings command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
    
}

int sendAllStates(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send get all states");
    
    // Send get all states command
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send get all states command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendBeginStream(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Streaming Begin");
    
    // Send Streaming begin command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(cmdBuffer, sizeof(cmdBuffer), &cmdSize, 1);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send Streaming command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendTakeoff(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send take off");
    
    // Send take off command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingTakeOff(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send takeoff command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendLanding(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send landing");
    
    // Send landing command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingLanding(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send landing command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

int sendEmergency(BD_MANAGER_t *deviceManager)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- Send Emergency");
    
    // Send emergency command
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3PilotingEmergency(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_EMERGENCY_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "Failed to send emergency command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}

eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;
    
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "    - arnetworkCmdCallback %d, cause:%d ", buffer_id, cause);
    
    if (cause == ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT)
    {
        retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP;
    }
    
    return retval;
}
// //+++++++++++++++++++++++++++++++++++ More Send ++++++++++++++++++++++++++++
// int sendWobble(BD_MANAGER_t *deviceManager)
// {
//     int sentStatus = 1;
//     u_int8_t cmdBuffer[128];
//     int32_t cmdSize = 0;
//     eARCOMMANDS_GENERATOR_ERROR cmdError;
//     eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
//     // Send pcmd command
//     cmdError = ARCOMMANDS_Generator_GenerateARDrone3DebugVideoEnableWobbleCancellation(cmdBuffer, sizeof(cmdBuffer), &cmdSize, (uint8_t)deviceManager->wobbleEnable);
//     if (cmdError == ARCOMMANDS_GENERATOR_OK)
//     {
//         // The commands sent in loop should be sent to a buffer not acknowledged ; here BD_NET_CD_NONACK_ID
//         netError = ARNETWORK_Manager_SendData(deviceManager->netManager, BD_NET_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
//     }
    
//     if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
//     {
//         sentStatus = 0;
//     }
    
//     return sentStatus;
// }
// //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
/************************** Commands callback part **************************/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager)
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback(batteryStateChangedCallback, deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateFlyingStateChangedCallback(flyingStateChangedCallback, deviceManager);
    // ADD HERE THE CALLBACKS YOU ARE INTERESTED IN
    //+++++++++++++More Call Back ++++++++++++++++++++++++++++
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCallback(speedChangedCallback,deviceManager);
    // ARCOMMANDS_Decoder_SetCommonCommonStateCurrentTimeChangedCallback(curTimeChangedCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAltitudeChangedCallback(altitudeChangedCallback,deviceManager);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAttitudeChangedCallback(attitudeChangedCallback,deviceManager);
    // ARCOMMANDS_Decoder_SetARDrone3DebugVideoEnableWobbleCancellationCallback(wobbleCancellationCallback,deviceManager);
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}

void unregisterARCommandsCallbacks ()
{
    ARCOMMANDS_Decoder_SetCommonCommonStateBatteryStateChangedCallback (NULL, NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateFlyingStateChangedCallback(NULL, NULL);
    //+++++++++++++++++++++++++++++++++++++ More +++++++++++++++++++++++++++++++=
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateSpeedChangedCallback(NULL,NULL);
    // ARCOMMANDS_Decoder_SetCommonCommonStateCurrentTimeChangedCallback(NULL,NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAltitudeChangedCallback(NULL,NULL);
    ARCOMMANDS_Decoder_SetARDrone3PilotingStateAttitudeChangedCallback(NULL,NULL);
    // ARCOMMANDS_Decoder_SetARDrone3DebugVideoEnableWobbleCancellationCallback(NULL,NULL); 
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

} 

void batteryStateChangedCallback (uint8_t percent, void *custom)
{
    // callback of changing of battery level
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
    {
        IHM_PrintBattery (deviceManager->ihm, percent);
    }
}

void flyingStateChangedCallback (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state, void *custom)
{
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    if (deviceManager != NULL)
    {
        deviceManager->flyingState = state;
        
        switch (deviceManager->flyingState) {
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : landed");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_TAKINGOFF:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : taking off");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : hovering");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : flying");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDING:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : landing");
                break;
            case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_EMERGENCY:
                IHM_PrintInfo(deviceManager->ihm, "Flying state : emergency");
                break;
            default:
                break;
        }
    }
}

//+++++++++++++++++++++++++ More Call Back and Diaplay +++++++++++++
void speedChangedCallback (float speedX, float speedY, float speedZ, void *custom)
{
    // callback of changing of battery level
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
    {   
        deviceManager->speed.speedX = speedX;
        deviceManager->speed.speedY = speedY;
        deviceManager->speed.speedZ = speedZ;
        IHM_PrintSpeed (deviceManager->ihm, speedX, speedY, speedZ);
    }
}

// void curTimeChangedCallback (char *curTime, void *custom){
//     BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
//     if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
//     {
//         IHM_PrintCurTime (deviceManager->ihm, curTime);
//     }
// }

void altitudeChangedCallback (double altitude, void *custom){
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
    {   
        deviceManager->pos.altitude = altitude;
        IHM_PrintAltitude (deviceManager->ihm, altitude);
    }
}

void attitudeChangedCallback (float roll, float pitch, float yaw, void *custom){
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
    
    if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
    {   
        deviceManager->pos.roll = roll;
        deviceManager->pos.pitch = pitch;
        deviceManager->pos.yaw = yaw;
        IHM_PrintAttitude (deviceManager->ihm, roll, pitch, yaw);
    }
}

// void wobbleCancellationCallback (uint8_t enable, void *custom){
//     BD_MANAGER_t *deviceManager = (BD_MANAGER_t*)custom;
//     if ((deviceManager != NULL) && (deviceManager->ihm != NULL))
//     {   
//         IHM_PrintWobbleCancellationState (deviceManager->ihm, enable);
//     }
// }
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/************************** Decoding part **************************/
int startDecoder(BD_MANAGER_t *deviceManager)
{
    int failed = 0;
    
    // create ffmpeg decoder
    if (!failed)
    {
        eARCODECS_ERROR error;
        deviceManager->decoder = ARCODECS_Manager_New (getNextDataCallback, deviceManager, &error);
        if (error != ARCODECS_OK)
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        if (ARSAL_Mutex_Init (&(deviceManager->mutex)) != 0)
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // prepare the pool : a large number of frame allocated
        deviceManager->freeRawFramePool = malloc(sizeof(RawFrame_t*) * BD_RAW_FRAME_POOL_SIZE);
        if (deviceManager->freeRawFramePool != NULL)
        {
            deviceManager->rawFramePoolCapacity = BD_RAW_FRAME_POOL_SIZE;
        }
        else
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during pool init");
        }
    }
    
    if (!failed)
    {
        // Allocate all frames of the pool
        int i = 0;
        for (i = 0; (!failed) && (i < deviceManager->rawFramePoolCapacity); i++)
        {
            RawFrame_t *rawFrame = malloc(sizeof(RawFrame_t));
            if (rawFrame != NULL)
            {
                rawFrame->size = deviceManager->arstreamFragSize * deviceManager->arstreamFragNb;
                rawFrame->isIframe = 0;
                rawFrame->data = malloc (rawFrame->size);
                if (rawFrame->data == NULL)
                {
                    failed = 1;
                }
                else
                {
                    deviceManager->freeRawFramePool[i] = rawFrame;
                }
            }
            else
            {
                failed = 1;
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error during pool object init");
            }
        }
        
        if (!failed)
        {
            deviceManager->lastRawFrameFreeIdx = 0;
        }
    }
    
    if (!failed)
    {
        // Init the frame fifo
        deviceManager->rawFrameFifo = calloc (BD_RAW_FRAME_BUFFER_SIZE, sizeof(RawFrame_t*));
        if (deviceManager->rawFrameFifo == NULL)
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        // Create the decoding thread
        deviceManager->decodingCanceled = 0;
        if (ARSAL_Thread_Create(&(deviceManager->decodingThread), Decode_RunDataThread, deviceManager) != 0)
        {
            deviceManager->decodingCanceled = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Creation of video decodingThread failed.");
            failed = 1;
        }
    }
    
    return failed;
}

void stopDecoder(BD_MANAGER_t *deviceManager)
{
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Stop decoding");
    if (deviceManager->decodingCanceled == 0)
    {
        deviceManager->decodingCanceled = 1;
        ARSAL_Thread_Join(deviceManager->decodingThread, NULL);
        ARSAL_Thread_Destroy(&(deviceManager->decodingThread));
        deviceManager->decodingThread = NULL;
    }
    
    if (deviceManager->videoFrame)
    {
        free (deviceManager->videoFrame);
        deviceManager->videoFrame = NULL;
    }
    
    if (deviceManager->freeRawFramePool != NULL)
    {
        flushFifo(deviceManager);
        int i = 0;
        for (i = 0; i < deviceManager->rawFramePoolCapacity; i++)
        {
            free(deviceManager->freeRawFramePool[i]);
            deviceManager->freeRawFramePool[i] = NULL;
        }
        
        free(deviceManager->freeRawFramePool);
        deviceManager->freeRawFramePool = NULL;
    }

    if (deviceManager->rawFrameFifo)
    {
        free (deviceManager->rawFrameFifo);
        deviceManager->rawFrameFifo = NULL;
    }

    if (deviceManager->mutex != NULL)
    {
        ARSAL_Mutex_Destroy(&deviceManager->mutex);
        deviceManager->mutex = NULL;
    }

    if (deviceManager->decoder != NULL)
    {
        ARCODECS_Manager_Delete(&deviceManager->decoder);
    }
}

RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager)
{
    // get a free raw frame in the pool
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *freeRawFrame = NULL;
    if (deviceManager->lastRawFrameFreeIdx < deviceManager->rawFramePoolCapacity)
    {
        freeRawFrame = deviceManager->freeRawFramePool[deviceManager->lastRawFrameFreeIdx];
        if (freeRawFrame == NULL)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Free frame is null.");
        }

        deviceManager->lastRawFrameFreeIdx++;
    }
    else
    {
        // there is no more free raw frame in the pool, create a new one and add it to the pool
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "No more frame free, need to alloc a new one and add it to the pool");
        freeRawFrame = malloc(sizeof(RawFrame_t));
        if (freeRawFrame != NULL)
        {
            RawFrame_t **freeRawFramePoolReallocated = realloc(deviceManager->freeRawFramePool, deviceManager->rawFramePoolCapacity + 1);
            if (freeRawFramePoolReallocated != NULL)
            {
                deviceManager->freeRawFramePool = freeRawFramePoolReallocated;
                deviceManager->rawFramePoolCapacity++;
                
                deviceManager->freeRawFramePool[deviceManager->lastRawFrameFreeIdx] = freeRawFrame;
                deviceManager->lastRawFrameFreeIdx++;
            }
        }
    }
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    return freeRawFrame;
}

RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data)
{
    // get the frame which has the given data
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *rawFrame = NULL;
    int i = 0;
    for (i = 0; i < deviceManager->rawFramePoolCapacity; i++)
    {
        RawFrame_t *currentRawFrame = deviceManager->freeRawFramePool[i];
        if (currentRawFrame != NULL && currentRawFrame->data != NULL)
        {
            if (currentRawFrame->data == data)
            {
                rawFrame = currentRawFrame;
                break;
            }
        }
    }
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    return rawFrame;
}

void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame)
{
    // put the frame from the pool to the Fifo
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    int idxToWrite = (deviceManager->fifoWriteIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
    
    // put the old frame on the free frame pool
    if (deviceManager->rawFrameFifo[idxToWrite] != NULL)
    {
        putRawFrameBackToPool(deviceManager, idxToWrite);
    }
    
    // put the new rawFrame at the write idx
    deviceManager->rawFrameFifo[idxToWrite] = rawFrame;
    deviceManager->fifoWriteIdx = idxToWrite;
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
}

void flushFifo(BD_MANAGER_t *deviceManager)
{
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    
    int shouldContinue = 1;
    int currentRawFrameIdx = deviceManager->fifoReadIdx;
    do
    {
        if (deviceManager->rawFrameFifo[currentRawFrameIdx] != NULL)
        {
            putRawFrameBackToPool(deviceManager, currentRawFrameIdx);
        }
        currentRawFrameIdx = (currentRawFrameIdx + 1) % BD_RAW_FRAME_BUFFER_SIZE;
    } while (currentRawFrameIdx != deviceManager->fifoReadIdx);
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
}

void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx)
{
    // remove the frame from the fifo and put it back to the pool => declare the frame as unused
    if (deviceManager->rawFrameFifo[fifoIdx] != NULL)
    {
        deviceManager->lastRawFrameFreeIdx--;
        
        deviceManager->rawFrameFifo[fifoIdx] = NULL;
    }
}

int getNextDataCallback(uint8_t **data, void *customData)
{
    // this callback is used by the decoder to get the frame to decode
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    int size = 0;
    
    ARSAL_Mutex_Lock (&(deviceManager->mutex));
    RawFrame_t *rawFrame = deviceManager->rawFrameFifo[deviceManager->fifoReadIdx];
    ARSAL_Mutex_Unlock (&(deviceManager->mutex));
    
    if (data != NULL && rawFrame != NULL)
    {
        *data = rawFrame->data;
        size = rawFrame->size;
        
    }
    return size;
}


/************************** IHM callbacks **************************/
void onInputEvent (eIHM_INPUT_EVENT event, void *customData)
{
    // Manage IHM input events
    BD_MANAGER_t *deviceManager = (BD_MANAGER_t *)customData;
    
    if(deviceManager->autoFly == 1){
        switch (event){
            case IHM_INPUT_EVENT_RESET:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_EMERGENCY:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                        sendEmergency(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_NONE:
                if(deviceManager != NULL)
                    {   
                        getLineDesiredState(deviceManager);
                        speedWorldFrameConversion(deviceManager);
                        odometryUpdate(deviceManager);
                        W2C(deviceManager);
                        pidControl(deviceManager);
                    }
                break;
            default:
                break;
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++

        }
    }
    else if(deviceManager->autoFly == 2){
        switch (event){
            case IHM_INPUT_EVENT_RESET:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_EMERGENCY:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                        sendEmergency(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_NONE:
                if(deviceManager != NULL)
                    {   
                        getSquareDesiredState(deviceManager);
                        speedWorldFrameConversion(deviceManager);
                        odometryUpdate(deviceManager);
                        W2C(deviceManager);
                        pidControl(deviceManager);
                    }
                break;
            default:
                break;
        }
    }   
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++            
    else if(deviceManager->autoFly == 3){
        switch (event){
            case IHM_INPUT_EVENT_RESET:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_EMERGENCY:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                        sendEmergency(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_NONE:
                if(deviceManager != NULL)
                    {   
                        getCircleDesiredState(deviceManager);
                        speedWorldFrameConversion(deviceManager);
                        odometryUpdate(deviceManager);
                        W2C(deviceManager);
                        pidControl(deviceManager);
                    }
                break;
            default:
                break;
            }
        }
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++
    else if(deviceManager->autoFly == 4){
        switch (event){
            case IHM_INPUT_EVENT_RESET:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_EMERGENCY:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                        sendEmergency(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_NONE:
                if(deviceManager != NULL)
                    {   
                        getEightDesiredState(deviceManager);
                        speedWorldFrameConversion(deviceManager);
                        odometryUpdate(deviceManager);
                        W2C(deviceManager);
                        pidControl(deviceManager);
                    }
                break;
            default:
                break;
            }
        }
        //+++++++++++++++++++++++++++++++++++++++++++++++++++
    else if(deviceManager->autoFly == 5){
        switch (event){
            case IHM_INPUT_EVENT_RESET:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_EMERGENCY:
                if(deviceManager != NULL)
                    {   
                        endAutoFly(deviceManager);
                        sendEmergency(deviceManager);
                    }
                break;
            case IHM_INPUT_EVENT_NONE:
                if(deviceManager != NULL)
                    {   
                        getCircleDesiredState(deviceManager);
                        speedWorldFrameConversion(deviceManager);
                        odometryUpdate(deviceManager);
                        W2C(deviceManager);
                        pidControl(deviceManager);
                    }
                break;
            default:
                break;
            }
        }
        //++++++++++++++++++++++++
    else{
        switch (event)
        {
            case IHM_INPUT_EVENT_EXIT:
                gIHMRun = 0;
                break;
            case IHM_INPUT_EVENT_EMERGENCY:
                if(deviceManager != NULL)
                {
                    sendEmergency(deviceManager);
                }
                break;
            case IHM_INPUT_EVENT_TAKEOFF_LANDING:
                if(deviceManager != NULL)
                {
                    if (deviceManager->flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED)
                    {
                        sendTakeoff(deviceManager);
                    }
                    else if ((deviceManager->flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING) ||
                             (deviceManager->flyingState == ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING))
                    {
                        sendLanding(deviceManager);
                    }
                        
                }
                break;
            case IHM_INPUT_EVENT_FORWARD:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.flag = 1;
                    deviceManager->dataPCMD.pitch = 50;
                }
                break;
            case IHM_INPUT_EVENT_BACK:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.flag = 1;
                    deviceManager->dataPCMD.pitch = -50;
                }
                break;
            case IHM_INPUT_EVENT_RIGHT:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.flag = 1;
                    deviceManager->dataPCMD.roll = 50;
                }
                break;
            case IHM_INPUT_EVENT_LEFT:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.flag = 1;
                    deviceManager->dataPCMD.roll = -50;
                }
                break;
            case IHM_INPUT_EVENT_YAW_RIGHT:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.yaw = 50;
                }
                break;
            case IHM_INPUT_EVENT_YAW_LEFT:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.yaw = -50;
                }
                break;
            case IHM_INPUT_EVENT_UP:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.gaz = 50;
                }
                break;
            case IHM_INPUT_EVENT_DOWN:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.gaz = -50;
                }
                break;
            case IHM_INPUT_EVENT_CAM_UP:
                if(deviceManager != NULL)
                {
                    deviceManager->dataCam.tilt += 2;
                    if (deviceManager->dataCam.tilt > 80)
                    {
                        deviceManager->dataCam.tilt = 80;
                    }
                }
                break;
            case IHM_INPUT_EVENT_CAM_DOWN:
                if(deviceManager != NULL)
                {
                    deviceManager->dataCam.tilt -= 2;
                    if (deviceManager->dataCam.tilt < -80)
                    {
                        deviceManager->dataCam.tilt = -80;
                    }
                }
                break;
            case IHM_INPUT_EVENT_CAM_RIGHT:
                if(deviceManager != NULL)
                {
                    deviceManager->dataCam.pan += 2;
                    if (deviceManager->dataCam.pan > 80)
                    {
                        deviceManager->dataCam.pan = 80;
                    }
                }
                break;
            case IHM_INPUT_EVENT_CAM_LEFT:
                if(deviceManager != NULL)
                {
                    deviceManager->dataCam.pan -= 2;
                    if (deviceManager->dataCam.pan < -80)
                    {
                        deviceManager->dataCam.pan = -80;
                    }
                }
                break;
            case IHM_INPUT_EVENT_NONE:
                if(deviceManager != NULL)
                {
                    deviceManager->dataPCMD.flag = 0;
                    deviceManager->dataPCMD.roll = 0;
                    deviceManager->dataPCMD.pitch = 0;
                    deviceManager->dataPCMD.yaw = 0;
                    deviceManager->dataPCMD.gaz = 0;
                }
                break;
            //+++++++++++++++++Input Case Added ++++++++++++++++++++
            case IHM_INPUT_EVENT_AUTOLINEMODE:
                if(deviceManager != NULL)
                {   
                    startAutoFly(deviceManager,1);
                    lineTrajCoeGenrator(deviceManager, 1.5, 1.5, 1.5, 5);//1.5, 1.5, 1.5, 5
                }
                break;
            case IHM_INPUT_EVENT_AUTOSQUAREMODE:
                if(deviceManager != NULL)
                {   
                    startAutoFly(deviceManager,2);
                    squareTrajCoeGenrator(deviceManager, 1, 1, 1, 25); //1.5, 1.5, 0, 15;  1.5, 1.5, 1.5, 15
                }
                break;
            case IHM_INPUT_EVENT_AUTOCIRCLEMODE:
                if(deviceManager != NULL)
                {   
                    startAutoFly(deviceManager,3);
                    circleTrajCoeGenrator(deviceManager, 0.5, 10, 0);  // 1.25, 15, 0 ;  0.5, 10, 1
                }
                break;
            case IHM_INPUT_EVENT_AUTOEIGHTMODE:
                if(deviceManager != NULL)
                {   
                    startAutoFly(deviceManager,4);
                    eightTrajCoeGenrator(deviceManager, 0.5, 25);
                }
                break;
            case IHM_INPUT_EVENT_AUTOVERTICALCIRCLEMODE:
                if(deviceManager != NULL)
                {   
                    startAutoFly(deviceManager,5);
                    circleTrajCoeGenrator(deviceManager, 0.5, 10,1);
                }
                break;
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++
            default:
                break;
        }
    }
}

int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
    // Custom callback used when ncurses is runing for not disturb the IHM
    
    if ((level == ARSAL_PRINT_ERROR) && (strcmp(TAG, tag) == 0))
    {
        // save the last Error
        vsnprintf(gErrorStr, (ERROR_STR_LENGTH - 1), format, va);
        gErrorStr[ERROR_STR_LENGTH - 1] = '\0';
    }
    
    return 1;
}

//+++++++++++++++++++ Controler ++++++++++++++++++++++++++++++++
//Conversion from sensor frame to control frame  WORLD FRAME
void speedWorldFrameConversion(BD_MANAGER_t *deviceManager){

    deviceManager->worldSpeed.speedX = deviceManager->speed.speedX * cos(deviceManager->flyStartYaw) + deviceManager->speed.speedY * sin(deviceManager->flyStartYaw);
    deviceManager->worldSpeed.speedY = -(deviceManager->speed.speedY *cos(deviceManager->flyStartYaw) - deviceManager->speed.speedX * sin(deviceManager->flyStartYaw));
    deviceManager->worldSpeed.speedZ = -deviceManager->speed.speedZ;
}
    
void W2C(BD_MANAGER_t *deviceManager){
    deviceManager->droneSpeed.speedX = deviceManager->speed.speedX * cos(deviceManager->pos.yaw) + deviceManager->speed.speedY * sin(deviceManager->pos.yaw);
    deviceManager->droneSpeed.speedY = -(deviceManager->speed.speedY * cos(deviceManager->pos.yaw) - deviceManager->speed.speedX * sin(deviceManager->pos.yaw));
    deviceManager->droneSpeed.speedZ = deviceManager->worldSpeed.speedZ;
}

void odometryUpdate(BD_MANAGER_t *deviceManager){
    deviceManager->dtime = deviceManager->timePassed - deviceManager->previousTime;
    deviceManager->odometry.x_pos += deviceManager->dtime*deviceManager->worldSpeed.speedX;
    deviceManager->odometry.y_pos += deviceManager->dtime*deviceManager->worldSpeed.speedY;
    deviceManager->odometry.z_pos = deviceManager->pos.altitude - deviceManager->flyStartAltitude;
    deviceManager->previousTime = deviceManager->timePassed;
}

void pidControl(BD_MANAGER_t *deviceManager){
    double pitch, roll, gaz;
    double maximum = 100;
    double minimum = -maximum;
    double delta_x = deviceManager->desiredStates.x_posd - deviceManager->odometry.x_pos;
    double delta_y = deviceManager->desiredStates.y_posd - deviceManager->odometry.y_pos;
    double delta_yaw = deviceManager->pos.yaw - deviceManager->flyStartYaw;
    double pitch_pos_error = delta_x * cos(delta_yaw) - delta_y * sin(delta_yaw);
    double roll_pos_error = delta_x * sin(delta_yaw) + delta_y * cos(delta_yaw);
    double x_veld = deviceManager->desiredStates.x_veld * cos(delta_yaw) - deviceManager->desiredStates.y_veld * sin(delta_yaw);
    double y_veld = deviceManager->desiredStates.x_veld * sin(delta_yaw) + deviceManager->desiredStates.y_veld * cos(delta_yaw);
    pitch = deviceManager->pid_Coe.kPxy*(pitch_pos_error) + deviceManager->pid_Coe.kDxy*(x_veld - deviceManager->droneSpeed.speedX);
    roll = - (deviceManager->pid_Coe.kPxy*(roll_pos_error) + deviceManager->pid_Coe.kDxy*(y_veld - deviceManager->droneSpeed.speedY));
    gaz = deviceManager->pid_Coe.kPz*(deviceManager->desiredStates.z_posd - deviceManager->odometry.z_pos) + deviceManager->pid_Coe.kDz*(deviceManager->desiredStates.z_veld - deviceManager->droneSpeed.speedZ);
    if(pitch > maximum){
        pitch = maximum;
    }
    else if(pitch < minimum){
        pitch = minimum;
    }

    if(roll > maximum){
        roll = maximum;
    }
    else if(roll < minimum){
        roll = minimum;
    }

    if(gaz > maximum){
        gaz = maximum;
    }
    else if(gaz < minimum){
        gaz = minimum;
    }

    deviceManager->dataPCMD.gaz = gaz;
    deviceManager->dataPCMD.roll = roll;
    deviceManager->dataPCMD.pitch = pitch;
    deviceManager->printData.pitch_error = pitch_pos_error;
    deviceManager->printData.roll_error = roll_pos_error;
    deviceManager->printData.x_veld = x_veld;
    deviceManager->printData.y_veld = y_veld;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++ Trajectory Generator +++++++++++++++++++++
void lineTrajCoeGenrator(BD_MANAGER_t *deviceManager, double xf, double yf, double zf, double tf){
    deviceManager->lineTrajCoe->c3x = 10*xf/pow(tf,3);
    deviceManager->lineTrajCoe->c4x = -15*xf/pow(tf,4);
    deviceManager->lineTrajCoe->c5x = 6*xf/pow(tf,5);
    deviceManager->lineTrajCoe->c3y = 10*yf/pow(tf,3);
    deviceManager->lineTrajCoe->c4y = -15*yf/pow(tf,4);
    deviceManager->lineTrajCoe->c5y = 6*yf/pow(tf,5);
    deviceManager->lineTrajCoe->c3z = 10*zf/pow(tf,3);
    deviceManager->lineTrajCoe->c4z = -15*zf/pow(tf,4);
    deviceManager->lineTrajCoe->c5z = 6*zf/pow(tf,5);
    deviceManager->lineTrajCoe->totalTime = tf;
}

void getLineDesiredState(BD_MANAGER_t *deviceManager){
    double t = deviceManager->timePassed - deviceManager->autoStartTime;
    if(t>=0 && t<=deviceManager->lineTrajCoe->totalTime){
        deviceManager->desiredStates.x_posd = deviceManager->lineTrajCoe->c3x*pow(t,3) + deviceManager->lineTrajCoe->c4x*pow(t,4) + deviceManager->lineTrajCoe->c5x*pow(t,5);
        deviceManager->desiredStates.y_posd = deviceManager->lineTrajCoe->c3y*pow(t,3) + deviceManager->lineTrajCoe->c4y*pow(t,4) + deviceManager->lineTrajCoe->c5y*pow(t,5);
        deviceManager->desiredStates.z_posd = deviceManager->lineTrajCoe->c3z*pow(t,3) + deviceManager->lineTrajCoe->c4z*pow(t,4) + deviceManager->lineTrajCoe->c5z*pow(t,5);
        deviceManager->desiredStates.x_veld = 3*deviceManager->lineTrajCoe->c3x*pow(t,2) + 4*deviceManager->lineTrajCoe->c4x*pow(t,3) + 5*deviceManager->lineTrajCoe->c5x*pow(t,4);
        deviceManager->desiredStates.y_veld = 3*deviceManager->lineTrajCoe->c3y*pow(t,2) + 4*deviceManager->lineTrajCoe->c4y*pow(t,3) + 5*deviceManager->lineTrajCoe->c5y*pow(t,4);
        deviceManager->desiredStates.z_veld = 3*deviceManager->lineTrajCoe->c3z*pow(t,2) + 4*deviceManager->lineTrajCoe->c4z*pow(t,3) + 5*deviceManager->lineTrajCoe->c5z*pow(t,4);
    }
    else{
        double tf = deviceManager->lineTrajCoe->totalTime;
        deviceManager->desiredStates.x_posd = deviceManager->lineTrajCoe->c3x*pow(tf,3) + deviceManager->lineTrajCoe->c4x*pow(tf,4) + deviceManager->lineTrajCoe->c5x*pow(tf,5);
        deviceManager->desiredStates.y_posd = deviceManager->lineTrajCoe->c3y*pow(tf,3) + deviceManager->lineTrajCoe->c4y*pow(tf,4) + deviceManager->lineTrajCoe->c5y*pow(tf,5);
        deviceManager->desiredStates.z_posd = deviceManager->lineTrajCoe->c3z*pow(tf,3) + deviceManager->lineTrajCoe->c4z*pow(tf,4) + deviceManager->lineTrajCoe->c5z*pow(tf,5);
        deviceManager->desiredStates.x_veld = 3*deviceManager->lineTrajCoe->c3x*pow(tf,2) + 4*deviceManager->lineTrajCoe->c4x*pow(tf,3) + 5*deviceManager->lineTrajCoe->c5x*pow(tf,4);
        deviceManager->desiredStates.y_veld = 3*deviceManager->lineTrajCoe->c3y*pow(tf,2) + 4*deviceManager->lineTrajCoe->c4y*pow(tf,3) + 5*deviceManager->lineTrajCoe->c5y*pow(tf,4);
        deviceManager->desiredStates.z_veld = 3*deviceManager->lineTrajCoe->c3z*pow(tf,2) + 4*deviceManager->lineTrajCoe->c4z*pow(tf,3) + 5*deviceManager->lineTrajCoe->c5z*pow(tf,4);
    }
};

void squareTrajCoeGenrator(BD_MANAGER_t *deviceManager, double xd, double yd, double zd, double tf){
        double ts = tf/4;

        deviceManager->squareTrajCoe->line1Coe.c3x = 10*xd/pow(ts,3);
        deviceManager->squareTrajCoe->line1Coe.c4x = -15*xd/pow(ts,4);
        deviceManager->squareTrajCoe->line1Coe.c5x = 6*xd/pow(ts,5);
        deviceManager->squareTrajCoe->line1Coe.c3y = 0;
        deviceManager->squareTrajCoe->line1Coe.c4y = 0;
        deviceManager->squareTrajCoe->line1Coe.c5y = 0;
        deviceManager->squareTrajCoe->line1Coe.c3z = 10*(0.5*zd)/pow(ts,3);
        deviceManager->squareTrajCoe->line1Coe.c4z = -15*(0.5*zd)/pow(ts,4);
        deviceManager->squareTrajCoe->line1Coe.c5z = 6*(0.5*zd)/pow(ts,5);
        deviceManager->squareTrajCoe->line1Coe.totalTime = tf/4;

        deviceManager->squareTrajCoe->line2Coe.c3x = 0;
        deviceManager->squareTrajCoe->line2Coe.c4x = 0;
        deviceManager->squareTrajCoe->line2Coe.c5x = 0;
        deviceManager->squareTrajCoe->line2Coe.c3y = 10*yd/pow(ts,3);
        deviceManager->squareTrajCoe->line2Coe.c4y = -15*yd/pow(ts,4);
        deviceManager->squareTrajCoe->line2Coe.c5y = 6*yd/pow(ts,5);
        deviceManager->squareTrajCoe->line2Coe.c3z = 10*(0.5*zd)/pow(ts,3);
        deviceManager->squareTrajCoe->line2Coe.c4z = -15*(0.5*zd)/pow(ts,4);
        deviceManager->squareTrajCoe->line2Coe.c5z = 6*(0.5*zd)/pow(ts,5);
        deviceManager->squareTrajCoe->line2Coe.totalTime = tf/4;

        deviceManager->squareTrajCoe->line3Coe.c3x = 10*(-xd)/pow(ts,3);
        deviceManager->squareTrajCoe->line3Coe.c4x = -15*(-xd)/pow(ts,4);
        deviceManager->squareTrajCoe->line3Coe.c5x = 6*(-xd)/pow(ts,5);
        deviceManager->squareTrajCoe->line3Coe.c3y = 0;
        deviceManager->squareTrajCoe->line3Coe.c4y = 0;
        deviceManager->squareTrajCoe->line3Coe.c5y = 0;
        deviceManager->squareTrajCoe->line3Coe.c3z = 10*(-0.5*zd)/pow(ts,3);
        deviceManager->squareTrajCoe->line3Coe.c4z = -15*(-0.5*zd)/pow(ts,4);
        deviceManager->squareTrajCoe->line3Coe.c5z = 6*(-0.5*zd)/pow(ts,5);
        deviceManager->squareTrajCoe->line3Coe.totalTime = tf/4;

        deviceManager->squareTrajCoe->line4Coe.c3x = 0;
        deviceManager->squareTrajCoe->line4Coe.c4x = 0;
        deviceManager->squareTrajCoe->line4Coe.c5x = 0;
        deviceManager->squareTrajCoe->line4Coe.c3y = 10*(-yd)/pow(ts,3);
        deviceManager->squareTrajCoe->line4Coe.c4y = -15*(-yd)/pow(ts,4);
        deviceManager->squareTrajCoe->line4Coe.c5y = 6*(-yd)/pow(ts,5);
        deviceManager->squareTrajCoe->line4Coe.c3z = 10*(-0.5*zd)/pow(ts,3);
        deviceManager->squareTrajCoe->line4Coe.c4z = -15*(-0.5*zd)/pow(ts,4);
        deviceManager->squareTrajCoe->line4Coe.c5z = 6*(-0.5*zd)/pow(ts,5);
        deviceManager->squareTrajCoe->line4Coe.totalTime = tf/4;

        deviceManager->squareTrajCoe->xd = xd;
        deviceManager->squareTrajCoe->yd = yd;
        deviceManager->squareTrajCoe->zd = zd;
        deviceManager->squareTrajCoe->totalTime = tf;
}

void getSquareDesiredState(BD_MANAGER_t *deviceManager){
    double t = deviceManager->timePassed - deviceManager->autoStartTime;
    double tf = deviceManager->squareTrajCoe->totalTime;
    double xd = deviceManager->squareTrajCoe->xd;
    double yd = deviceManager->squareTrajCoe->yd;
    double zd = deviceManager->squareTrajCoe->zd;

    if (t>=0 && t<tf/4){
        double c3x = deviceManager->squareTrajCoe->line1Coe.c3x;
        double c4x = deviceManager->squareTrajCoe->line1Coe.c4x;
        double c5x = deviceManager->squareTrajCoe->line1Coe.c5x;
        double c3y = deviceManager->squareTrajCoe->line1Coe.c3y;
        double c4y = deviceManager->squareTrajCoe->line1Coe.c4y;
        double c5y = deviceManager->squareTrajCoe->line1Coe.c5y;
        double c3z = deviceManager->squareTrajCoe->line1Coe.c3z;
        double c4z = deviceManager->squareTrajCoe->line1Coe.c4z;
        double c5z = deviceManager->squareTrajCoe->line1Coe.c5z;
        deviceManager->desiredStates.x_posd = c3x*pow(t,3) + c4x*pow(t,4) + c5x*pow(t,5);
        deviceManager->desiredStates.y_posd = c3y*pow(t,3) + c4y*pow(t,4) + c5y*pow(t,5);
        deviceManager->desiredStates.z_posd = c3z*pow(t,3) + c4z*pow(t,4) + c5z*pow(t,5);
        deviceManager->desiredStates.x_veld = 3*c3x*pow(t,2) + 4*c4x*pow(t,3) + 5*c5x*pow(t,4);
        deviceManager->desiredStates.y_veld = 3*c3y*pow(t,2) + 4*c4y*pow(t,3) + 5*c5y*pow(t,4);
        deviceManager->desiredStates.z_veld = 3*c3z*pow(t,2) + 4*c4z*pow(t,3) + 5*c5z*pow(t,4);
    }
    else if(t >= tf/4 && t<tf/2){
        t = t - tf/4;
        double c3x = deviceManager->squareTrajCoe->line2Coe.c3x;
        double c4x = deviceManager->squareTrajCoe->line2Coe.c4x;
        double c5x = deviceManager->squareTrajCoe->line2Coe.c5x;
        double c3y = deviceManager->squareTrajCoe->line2Coe.c3y;
        double c4y = deviceManager->squareTrajCoe->line2Coe.c4y;
        double c5y = deviceManager->squareTrajCoe->line2Coe.c5y;
        double c3z = deviceManager->squareTrajCoe->line2Coe.c3z;
        double c4z = deviceManager->squareTrajCoe->line2Coe.c4z;
        double c5z = deviceManager->squareTrajCoe->line2Coe.c5z;
        deviceManager->desiredStates.x_posd = c3x*pow(t,3) + c4x*pow(t,4) + c5x*pow(t,5) + xd;
        deviceManager->desiredStates.y_posd = c3y*pow(t,3) + c4y*pow(t,4) + c5y*pow(t,5);
        deviceManager->desiredStates.z_posd = c3z*pow(t,3) + c4z*pow(t,4) + c5z*pow(t,5) + zd/2;
        deviceManager->desiredStates.x_veld = 3*c3x*pow(t,2) + 4*c4x*pow(t,3) + 5*c5x*pow(t,4);
        deviceManager->desiredStates.y_veld = 3*c3y*pow(t,2) + 4*c4y*pow(t,3) + 5*c5y*pow(t,4);
        deviceManager->desiredStates.z_veld = 3*c3z*pow(t,2) + 4*c4z*pow(t,3) + 5*c5z*pow(t,4);

    }
    else if(t >= tf/2 && t<tf*0.75){
        t = t - tf/2;
        double c3x = deviceManager->squareTrajCoe->line3Coe.c3x;
        double c4x = deviceManager->squareTrajCoe->line3Coe.c4x;
        double c5x = deviceManager->squareTrajCoe->line3Coe.c5x;
        double c3y = deviceManager->squareTrajCoe->line3Coe.c3y;
        double c4y = deviceManager->squareTrajCoe->line3Coe.c4y;
        double c5y = deviceManager->squareTrajCoe->line3Coe.c5y;
        double c3z = deviceManager->squareTrajCoe->line3Coe.c3z;
        double c4z = deviceManager->squareTrajCoe->line3Coe.c4z;
        double c5z = deviceManager->squareTrajCoe->line3Coe.c5z;
        deviceManager->desiredStates.x_posd = c3x*pow(t,3) + c4x*pow(t,4) + c5x*pow(t,5) + xd;
        deviceManager->desiredStates.y_posd = c3y*pow(t,3) + c4y*pow(t,4) + c5y*pow(t,5) + yd;
        deviceManager->desiredStates.z_posd = c3z*pow(t,3) + c4z*pow(t,4) + c5z*pow(t,5) + zd;
        deviceManager->desiredStates.x_veld = 3*c3x*pow(t,2) + 4*c4x*pow(t,3) + 5*c5x*pow(t,4);
        deviceManager->desiredStates.y_veld = 3*c3y*pow(t,2) + 4*c4y*pow(t,3) + 5*c5y*pow(t,4);
        deviceManager->desiredStates.z_veld = 3*c3z*pow(t,2) + 4*c4z*pow(t,3) + 5*c5z*pow(t,4);
    }
    else if(t >= tf*0.75 && t<tf){
        t = t - tf*0.75;
        double c3x = deviceManager->squareTrajCoe->line4Coe.c3x;
        double c4x = deviceManager->squareTrajCoe->line4Coe.c4x;
        double c5x = deviceManager->squareTrajCoe->line4Coe.c5x;
        double c3y = deviceManager->squareTrajCoe->line4Coe.c3y;
        double c4y = deviceManager->squareTrajCoe->line4Coe.c4y;
        double c5y = deviceManager->squareTrajCoe->line4Coe.c5y;
        double c3z = deviceManager->squareTrajCoe->line4Coe.c3z;
        double c4z = deviceManager->squareTrajCoe->line4Coe.c4z;
        double c5z = deviceManager->squareTrajCoe->line4Coe.c5z;
        deviceManager->desiredStates.x_posd = c3x*pow(t,3) + c4x*pow(t,4) + c5x*pow(t,5) + 0;
        deviceManager->desiredStates.y_posd = c3y*pow(t,3) + c4y*pow(t,4) + c5y*pow(t,5) + yd;
        deviceManager->desiredStates.z_posd = c3z*pow(t,3) + c4z*pow(t,4) + c5z*pow(t,5) + zd/2;
        deviceManager->desiredStates.x_veld = 3*c3x*pow(t,2) + 4*c4x*pow(t,3) + 5*c5x*pow(t,4);
        deviceManager->desiredStates.y_veld = 3*c3y*pow(t,2) + 4*c4y*pow(t,3) + 5*c5y*pow(t,4);
        deviceManager->desiredStates.z_veld = 3*c3z*pow(t,2) + 4*c4z*pow(t,3) + 5*c5z*pow(t,4);
    }
    else{
        deviceManager->desiredStates.x_posd = 0;
        deviceManager->desiredStates.y_posd = 0;
        deviceManager->desiredStates.z_posd = 0;
        deviceManager->desiredStates.x_veld = 0;
        deviceManager->desiredStates.y_veld = 0;
        deviceManager->desiredStates.z_veld = 0;
    }
};


void circleTrajCoeGenrator(BD_MANAGER_t *deviceManager, double r, double tf, int vertical){
    deviceManager->circleTrajCoe->r = r;
    deviceManager->circleTrajCoe->totalTime = tf;
    deviceManager->circleTrajCoe->vertical = vertical;
}

void getCircleDesiredState(BD_MANAGER_t *deviceManager){
    double t = deviceManager->timePassed - deviceManager->autoStartTime;
    double tf = deviceManager->circleTrajCoe->totalTime;
    double r = deviceManager->circleTrajCoe->r;
    if(deviceManager->circleTrajCoe->vertical == 0){
        if(t>=0 && t<=tf){
            deviceManager->desiredStates.x_posd = sin(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r;
            deviceManager->desiredStates.y_posd = -r + cos(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r;
            deviceManager->desiredStates.z_posd = 0;
            deviceManager->desiredStates.x_veld = -cos(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r * (12*PI*pow(t,2)/pow(tf,3) - 12*PI*t/pow(tf,2));
            deviceManager->desiredStates.y_veld = sin(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r * (12*PI*pow(t,2)/pow(tf,3) - 12*PI*t/pow(tf,2));
            deviceManager->desiredStates.z_veld = 0;
        }
        else{
            deviceManager->desiredStates.x_posd = 0;
            deviceManager->desiredStates.y_posd = 0;
            deviceManager->desiredStates.z_posd = 0;
            deviceManager->desiredStates.x_veld = 0;
            deviceManager->desiredStates.y_veld = 0;
            deviceManager->desiredStates.z_veld = 0;
        }
    }
    else{
        if(t>=0 && t<=tf){
            deviceManager->desiredStates.x_posd = sin(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r;
            deviceManager->desiredStates.y_posd = 0;
            deviceManager->desiredStates.z_posd = -( -r + cos(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r);
            deviceManager->desiredStates.x_veld = -cos(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r * (12*PI*pow(t,2)/pow(tf,3) - 12*PI*t/pow(tf,2));
            deviceManager->desiredStates.y_veld = 0;
            deviceManager->desiredStates.z_veld = -(sin(6*PI*pow(t,2)/pow(tf,2) - 4*PI*pow(t,3)/pow(tf,3)) * r * (12*PI*pow(t,2)/pow(tf,3) - 12*PI*t/pow(tf,2)));
        }
        else{
            deviceManager->desiredStates.x_posd = 0;
            deviceManager->desiredStates.y_posd = 0;
            deviceManager->desiredStates.z_posd = 0;
            deviceManager->desiredStates.x_veld = 0;
            deviceManager->desiredStates.y_veld = 0;
            deviceManager->desiredStates.z_veld = 0;
        }
    }
};


void eightTrajCoeGenrator(BD_MANAGER_t *deviceManager, double r, double tf){
    deviceManager->eightTrajCoe->r = r;
    deviceManager->eightTrajCoe->c3 = 10*(4*PI)/pow(tf,3);
    deviceManager->eightTrajCoe->c4 = -15*(4*PI)/pow(tf,4);
    deviceManager->eightTrajCoe->c5 = 6*(4*PI)/pow(tf,5);
    deviceManager->eightTrajCoe->totalTime = tf;
}

void getEightDesiredState(BD_MANAGER_t *deviceManager){
    double t = deviceManager->timePassed - deviceManager->autoStartTime;
    double tf = deviceManager->eightTrajCoe->totalTime;
    double r = deviceManager->eightTrajCoe->r;
    double c3 = deviceManager->eightTrajCoe->c3;
    double c4 = deviceManager->eightTrajCoe->c4;
    double c5 = deviceManager->eightTrajCoe->c5;
    double a = c3*pow(t,3) + c4*pow(t,4) + c5*pow(t,5);

    if (a>=0 && a<PI){
        deviceManager->desiredStates.x_posd = sin(a) * r;
        deviceManager->desiredStates.y_posd = -(r- r*cos(a));
        deviceManager->desiredStates.z_posd = 0;
        deviceManager->desiredStates.x_veld = r*cos((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.y_veld = -r*sin((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.z_veld = 0;
    }
    else if(a >= PI && a<2*PI){
        a = a-PI;
        deviceManager->desiredStates.x_posd = -sin(a)*r;
        deviceManager->desiredStates.y_posd = -2*r - (r-r*cos(a)); 
        deviceManager->desiredStates.z_posd = 0;
        deviceManager->desiredStates.x_veld = r*cos((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.y_veld = r*sin((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.z_veld = 0;
    }
    else if(a >= 2*PI && a<3*PI){
        a = a - 2*PI;
        deviceManager->desiredStates.x_posd = sin(a)*r;
        deviceManager->desiredStates.y_posd = -4*r + (r - r*cos(a));
        deviceManager->desiredStates.z_posd = 0;
        deviceManager->desiredStates.x_veld = r*cos((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.y_veld = r*sin((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.z_veld = 0;
    }
    else if(a>=3*PI && a<4*PI){
        a = a - 3*PI;
        deviceManager->desiredStates.x_posd = -sin(a)*r;
        deviceManager->desiredStates.y_posd = -2*r + (r-r*cos(a));
        deviceManager->desiredStates.z_posd = 0;
        deviceManager->desiredStates.x_veld = r*cos((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.y_veld = -r*sin((40*PI*pow(t,3))/pow(tf,3) - (60*PI*pow(t,4))/pow(tf,4) + (24*PI*pow(t,5))/pow(tf,5))*((120*PI*pow(t,2))/pow(tf,3) - (240*PI*pow(t,3))/pow(tf,4) + (120*PI*pow(t,4))/pow(tf,5));
        deviceManager->desiredStates.z_veld = 0;
    }
    else{
        deviceManager->desiredStates.x_posd = 0;
        deviceManager->desiredStates.y_posd = 0;
        deviceManager->desiredStates.z_posd = 0;
        deviceManager->desiredStates.x_veld = 0;
        deviceManager->desiredStates.y_veld = 0;
        deviceManager->desiredStates.z_veld = 0;
    }

};

// void eightTrajCoeGenrator(BD_MANAGER_t *deviceManager, double r, double tf){
//     deviceManager->eightTrajCoe->r = r;
//     deviceManager->eightTrajCoe->totalTime = tf;
// }

// void getEightDesiredState(BD_MANAGER_t *deviceManager){
//     double t = deviceManager->timePassed - deviceManager->autoStartTime;
//     double tf = deviceManager->eightTrajCoe->totalTime;
//     double r = deviceManager->eightTrajCoe->r;

//     double ts = tf/2;

//     if (t>=0 && t<tf/4){
//         deviceManager->desiredStates.x_posd = sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r;
//         deviceManager->desiredStates.y_posd = -r + cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r;
//         deviceManager->desiredStates.z_posd = 0;
//         deviceManager->desiredStates.x_veld = -cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2));
//         deviceManager->desiredStates.y_veld = sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2));
//         deviceManager->desiredStates.z_veld = 0;
//     }
//     else if(t >= tf/4 && t<tf/2){
//         deviceManager->desiredStates.x_posd = sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r;
//         deviceManager->desiredStates.y_posd = -4*r-(-r + cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r);
//         deviceManager->desiredStates.z_posd = 0;
//         deviceManager->desiredStates.x_veld = -cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2));
//         deviceManager->desiredStates.y_veld = -(sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2)));
//         deviceManager->desiredStates.z_veld = 0;
//     }
//     else if(t >= tf/2 && t<tf*0.75){
//         t = t - tf/2;
//         deviceManager->desiredStates.x_posd = sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r;
//         deviceManager->desiredStates.y_posd = -(-r + cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r) -4*r;
//         deviceManager->desiredStates.z_posd = 0;
//         deviceManager->desiredStates.x_veld = -cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2));
//         deviceManager->desiredStates.y_veld = -(sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2)));
//         deviceManager->desiredStates.z_veld = 0;
//     }
//     else if(t >= tf*0.75 && t<tf){
//         t = t - tf/2;
//         deviceManager->desiredStates.x_posd = sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r;
//         deviceManager->desiredStates.y_posd = -r + cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r;
//         deviceManager->desiredStates.z_posd = 0;
//         deviceManager->desiredStates.x_veld = -cos(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2));
//         deviceManager->desiredStates.y_veld = sin(6*PI*pow(t,2)/pow(ts,2) - 4*PI*pow(t,3)/pow(ts,3)) * r * (12*PI*pow(t,2)/pow(ts,3) - 12*PI*t/pow(ts,2));
//         deviceManager->desiredStates.z_veld = 0;
//     }
//     else{
//         deviceManager->desiredStates.x_posd = 0;
//         deviceManager->desiredStates.y_posd = 0;
//         deviceManager->desiredStates.z_posd = 0;
//         deviceManager->desiredStates.x_veld = 0;
//         deviceManager->desiredStates.y_veld = 0;
//         deviceManager->desiredStates.z_veld = 0;
//     }
// };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++ util +++++++++++++++++++++++++++++++++++
void startAutoFly(BD_MANAGER_t *deviceManager, int typeNum){
    deviceManager->autoFly = typeNum;
    deviceManager->logNum += 1;
    deviceManager->dataPCMD.flag = 1;
    deviceManager->autoStartTime = deviceManager->timePassed;
    deviceManager->previousTime = deviceManager->autoStartTime;
    char logFileNameCallback[50];
    char logFileNameControlA[50];
    char logFileNameControlB[50];
    char logFileNameControlC[50];
    char logNumStr[5];
    strcpy(logFileNameCallback,"callbackLog");
    sprintf(logNumStr, "%d", deviceManager->logNum);
    strcat(logFileNameCallback, logNumStr);
    strcat(logFileNameCallback, ".txt");
    strcpy(logFileNameControlA,"controlLogA");
    strcat(logFileNameControlA,logNumStr);
    strcat(logFileNameControlA,".txt");
    strcpy(logFileNameControlB,"controlLogB");
    strcat(logFileNameControlB,logNumStr);
    strcat(logFileNameControlB,".txt");
    strcpy(logFileNameControlC,"controlLogC");
    strcat(logFileNameControlC,logNumStr);
    strcat(logFileNameControlC,".txt");
    deviceManager->logFileCallback = fopen(logFileNameCallback,"w+");
    deviceManager->logFileControlA = fopen(logFileNameControlA,"w+");
    deviceManager->logFileControlB = fopen(logFileNameControlB,"w+");
    deviceManager->logFileControlC = fopen(logFileNameControlC,"w+");
    fprintf(deviceManager->logFileCallback, "Time  speedX  speedY  speedZ  roll  pitch  yaw  altitude  \n");    
    fprintf(deviceManager->logFileControlA, "Time Desired_X_position  Desired_Y_position  Desired_Z_position  Desired_X_vel  Desired_Y_vel  Desired_Z_vel  X_Odometry  Y_Odometry  Z_Odometry  \n");
    fprintf(deviceManager->logFileControlB, "Time Roll_Control Pitch_Control Gaz_Control  DroneSpeed_X  DroneSpeed_Y  DroneSpeed_Z  FlyStartYaw\n");
    fprintf(deviceManager->logFileControlC, "Time DroneDesiredSpeedX DroneDesiredSpeedY DroneDesiredPositionX DroneDesiredPositionY\n");
    deviceManager->logingFlag = 1;
    deviceManager->odometry.x_pos = 0;
    deviceManager->odometry.y_pos = 0;
    deviceManager->odometry.z_pos = 0;
    deviceManager->flyStartYaw = deviceManager->pos.yaw;
    deviceManager->flyStartAltitude = deviceManager->pos.altitude;
}

void endAutoFly(BD_MANAGER_t *deviceManager) {
    deviceManager->dataPCMD.flag = 0;
    deviceManager->dataPCMD.flag = 0;
    deviceManager->dataPCMD.roll = 0;
    deviceManager->dataPCMD.pitch = 0;
    deviceManager->dataPCMD.yaw = 0;
    deviceManager->dataPCMD.gaz = 0;
    deviceManager->autoFly  = 0;
    deviceManager->desiredStates.x_posd = 0;
    deviceManager->desiredStates.y_posd = 0;
    deviceManager->desiredStates.z_posd = 0;
    deviceManager->desiredStates.x_veld = 0;
    deviceManager->desiredStates.y_veld = 0;
    deviceManager->desiredStates.z_veld = 0;
    fclose(deviceManager->logFileCallback);
    fclose(deviceManager->logFileControlA);
    fclose(deviceManager->logFileControlB);
    fclose(deviceManager->logFileControlC);
    deviceManager->logingFlag = 0;
}