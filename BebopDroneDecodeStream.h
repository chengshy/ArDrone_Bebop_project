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
#ifndef _SDK_EXAMPLE_BD_H_
#define _SDK_EXAMPLE_BD_H_

#include <ihm.h>
#include <DecoderManager.h>
#include <libARCommands/ARCommands.h>

typedef struct
{
    int flag;
    int roll;
    int pitch;
    int yaw;
    int gaz;
}BD_PCMD_t;

typedef struct _ARDrone3CameraData_t_
{
    int tilt;
    int pan;
} BD_Cam_t;

// ++++++++++++++ M: More States ++++++++++++++++++
// Speed 
typedef struct{
    float speedX;
    float speedY;
    float speedZ;
} BD_Speed_t;
    
typedef struct{
    float roll;
    float pitch;
    float yaw;
    float altitude;
} BD_Pos_t;

typedef struct{
    double kPxy; 
    double kIxy;
    double kDxy;
    double kPz; 
    double kIz;
    double kDz;
} pid_C_t;

typedef struct 
{
    double x_posd;
    double y_posd;
    double z_posd;
    double x_veld;
    double y_veld;
    double z_veld;
} desiredStates_t;

typedef struct{
    double x_pos;
    double y_pos;
    double z_pos;
} odometry_t;

typedef struct{
    double c3x;
    double c3y;
    double c3z;
    double c4x;
    double c4y;
    double c4z;
    double c5x;
    double c5y;
    double c5z;
    double totalTime;
} lineTrajCoe_t;

typedef struct{
    double x_veld;
    double y_veld;
    double pitch_error;
    double roll_error;
} dataPrint_t; 

typedef struct{
    double r;
    double totalTime;
    int vertical;
} circleTrajCoe_t;

typedef struct{
    lineTrajCoe_t line1Coe;
    lineTrajCoe_t line2Coe;
    lineTrajCoe_t line3Coe;
    lineTrajCoe_t line4Coe;
    double xd;
    double yd;
    double zd;    
    double totalTime;
} squareTrajCoe_t;

typedef struct{
    double c3;
    double c4;
    double c5;
    double r;
    double totalTime;
} eightTrajCoe_t;
// +++++++++++++ M End +++++++++++++++++++++++++++

typedef struct READER_THREAD_DATA_t READER_THREAD_DATA_t;

typedef struct RawFrame_t RawFrame_t;
typedef struct
{
    ARNETWORKAL_Manager_t *alManager;
    ARNETWORK_Manager_t *netManager;
    ARSTREAM_Reader_t *streamReader;
    ARSAL_Thread_t looperThread;
    ARSAL_Thread_t rxThread;
    ARSAL_Thread_t txThread;
    ARSAL_Thread_t videoTxThread;
    ARSAL_Thread_t videoRxThread;
    int d2cPort;
    int c2dPort;
    int arstreamFragSize;
    int arstreamFragNb;
    int arstreamAckDelay;
    uint8_t *videoFrame;
    uint32_t videoFrameSize;
    
    BD_PCMD_t dataPCMD;
    BD_Cam_t dataCam;
    
    ARCODECS_Manager_t *decoder;
    int decodingCanceled;
    ARSAL_Thread_t decodingThread;
    
    int hasReceivedFirstIFrame;
    RawFrame_t **freeRawFramePool;
    int rawFramePoolCapacity;
    int lastRawFrameFreeIdx;
    RawFrame_t **rawFrameFifo;
    int fifoReadIdx;
    int fifoWriteIdx;
    
    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState;
    
    FILE *video_out;
    
    ARSAL_Mutex_t mutex;
    
    ARSAL_Thread_t *readerThreads;
    READER_THREAD_DATA_t *readerThreadsData;
    int run;
    
    IHM_t *ihm;

    //States Added+++++++++++++++++
    BD_Speed_t speed;
    double timePassed;
    BD_Pos_t pos;
    double autoStartTime;
    pid_C_t pid_Coe;
    FILE *logFileCallback;
    FILE *logFileControlA;
    FILE *logFileControlB;
    FILE *logFileControlC;
    int logNum;
    int autoFly;
    int logingFlag;
    desiredStates_t desiredStates;
    odometry_t odometry;
    double dtime;
    double previousTime;
    BD_Speed_t droneSpeed;
    squareTrajCoe_t* squareTrajCoe;
    lineTrajCoe_t *lineTrajCoe;
    circleTrajCoe_t *circleTrajCoe;
    eightTrajCoe_t *eightTrajCoe;

    // uint8_t wobbleEnable;
    BD_Speed_t worldSpeed;
    double flyStartYaw;
    dataPrint_t printData;
    double flyStartAltitude;
    //+++++++++++++++++++++++++++++
} BD_MANAGER_t;

struct READER_THREAD_DATA_t
{
    BD_MANAGER_t *deviceManager;
    int readerBufferId;
};

/** Connection part **/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/** Network part **/
int startNetwork (BD_MANAGER_t *deviceManager);
void stopNetwork (BD_MANAGER_t *deviceManager);
void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData);

/** Video part **/
int startVideo (BD_MANAGER_t *deviceManager);
void stopVideo (BD_MANAGER_t *deviceManager);
uint8_t *frameCompleteCallback (eARSTREAM_READER_CAUSE cause, uint8_t *frame, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *custom);

/** decoding part **/
int startDecoder (BD_MANAGER_t *deviceManager);
void stopDecoder (BD_MANAGER_t *deviceManager);
int getNextDataCallback(uint8_t **data, void *customData);
RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager);
void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame);
void flushFifo(BD_MANAGER_t *deviceManager);
void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx);
RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data);

/** Commands part **/
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause);
int sendPCMD(BD_MANAGER_t *deviceManager);
int sendCameraOrientation(BD_MANAGER_t *deviceManager);
int sendDate(BD_MANAGER_t *deviceManager);
int sendAllStates(BD_MANAGER_t *deviceManager);
int sendAllSettings(BD_MANAGER_t *deviceManager);
int sendTakeoff(BD_MANAGER_t *deviceManager);
int sendLanding(BD_MANAGER_t *deviceManager);
int sendEmergency(BD_MANAGER_t *deviceManager);
int sendBeginStream(BD_MANAGER_t *deviceManager);
//++++++++++++++++++++++++++ More Commends ++++
// int sendWobble(BD_MANAGER_t *deviceManager);
//++++++++++++++++++++++++++++++++++++++++++++

/** Commands callback part **/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager);
void unregisterARCommandsCallbacks();
void batteryStateChangedCallback (uint8_t percent, void *custom);
void flyingStateChangedCallback (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state, void *custom);
//+++++++ More Callback ++++++++++++++++
void speedChangedCallback (float speedX, float speedY, float speedZ, void *custom);
// void curTimeChangedCallback (char *curTime, void *custom);
void altitudeChangedCallback (double altitude, void *custom);
void attitudeChangedCallback (float roll, float pitch, float yaw, void *custom);
// void wobbleCancellationCallback (uint8_t enable, void *custom);
//++++++++++++++++++++++++++++++++++++++

/** IHM callbacks **/
void onInputEvent (eIHM_INPUT_EVENT event, void *customData);
int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va);

//++++++++++++ Trajectory and control ++++++++++++++++
void pidControl(BD_MANAGER_t *deviceManager);

void lineTrajCoeGenrator(BD_MANAGER_t *deviceManager, double xf, double yf, double zf, double tf);
void getLineDesiredState(BD_MANAGER_t *deviceManager);

void squareTrajCoeGenrator(BD_MANAGER_t *deviceManager, double xd, double yd, double zd, double tf);
void getSquareDesiredState(BD_MANAGER_t *deviceManager);

void circleTrajCoeGenrator(BD_MANAGER_t *deviceManager, double r, double tf, int vertical);
void getCircleDesiredState(BD_MANAGER_t *deviceManager);

void eightTrajCoeGenrator(BD_MANAGER_t *deviceManager, double r, double tf);
void getEightDesiredState(BD_MANAGER_t *deviceManager);

void speedWorldFrameConversion(BD_MANAGER_t *deviceManager);
void odometryUpdate(BD_MANAGER_t *deviceManager);
void W2C(BD_MANAGER_t *deviceManager);
//+++++++++++++++++ util  ++++++++++++++++++++++++
void startAutoFly(BD_MANAGER_t *deviceManager, int typeNum);
void endAutoFly(BD_MANAGER_t *deviceManager);




#endif /* _SDK_EXAMPLE_BD_H_ */
