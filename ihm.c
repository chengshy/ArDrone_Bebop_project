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
 * @file ihm.c
 * @brief This file contains sources about ncurses IHM used by arsdk example "BebopDroneDecodeStream"
 * @date 15/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>
#include <curses.h>
#include <string.h>

#include <libARSAL/ARSAL.h>

#include "ihm.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/

#define HEADER_X 0
#define HEADER_Y 0

#define INSTRUCTION_X 0
#define INSTRUCTION_Y 2

#define BATTERY_X 0
#define BATTERY_Y 10

#define INFO_X 0
#define INFO_Y 16

//++++++++++++ Print More ++++++++++++++++++
#define SPEED_X 0
#define SPEED_Y 11

#define TIME_X 0
#define TIME_Y 14

#define ALTITUDE_X 0
#define ALTITUDE_Y 13

#define ATTITUDE_X 0
#define ATTITUDE_Y 12

// #define WCSTATE_X 0
// #define WCSTATE_Y 15
//+++++++++++++++++++++++++++++++++++++++++
/*****************************************
 *
 *             private header:
 *
 ****************************************/
void *IHM_InputProcessing(void *data);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

IHM_t *IHM_New (IHM_onInputEvent_t onInputEventCallback)
{
    int failed = 0;
    IHM_t *newIHM = NULL;
    
    // check parameters
    if (onInputEventCallback == NULL)
    {
        failed = 1;
    }
    
    if (!failed)
    {
        //  Initialize IHM
        newIHM = malloc(sizeof(IHM_t));
        if (newIHM != NULL)
        {
            //  Initialize ncurses
            newIHM->mainWindow = initscr();
            newIHM->inputThread = NULL;
            newIHM->run = 1;
            newIHM->onInputEventCallback = onInputEventCallback;
            newIHM->customData = NULL;
        }
        else
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        //raw();                  // Line buffering disabled
        cbreak();
        keypad(stdscr, TRUE);
        noecho();               // Don't echo() while we do getch
        timeout(100);
        
        refresh();
    }
    
    if (!failed)
    {
        //start input thread
        if(ARSAL_Thread_Create(&(newIHM->inputThread), IHM_InputProcessing, newIHM) != 0)
        {
            failed = 1;
        }
    }
    
    if (failed)
    {
        IHM_Delete (&newIHM);
    }

    return  newIHM;
}

void IHM_Delete (IHM_t **ihm)
{
    //  Clean up

    if (ihm != NULL)
    {
        if ((*ihm) != NULL)
        {
            (*ihm)->run = 0;
            
            if ((*ihm)->inputThread != NULL)
            {
                ARSAL_Thread_Join((*ihm)->inputThread, NULL);
                ARSAL_Thread_Destroy(&((*ihm)->inputThread));
                (*ihm)->inputThread = NULL;
            }
            
            delwin((*ihm)->mainWindow);
            (*ihm)->mainWindow = NULL;
            endwin();
            refresh();
            
            free (*ihm);
            (*ihm) = NULL;
        }
    }
}

void IHM_setCustomData(IHM_t *ihm, void *customData)
{
    if (ihm != NULL)
    {
        ihm->customData = customData;
    }
}

void *IHM_InputProcessing(void *data)
{
    IHM_t *ihm = (IHM_t *) data;
    int key = 0;
    
    if (ihm != NULL)
    {
        while (ihm->run)
        {
            key = getch();
            
            if (key == 27) // escape character
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_EXIT, ihm->customData);
                }
            }
            else if(key == KEY_UP)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_FORWARD, ihm->customData);
                }
            }
            else if(key == KEY_DOWN)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_BACK, ihm->customData);
                }
            }
            else if(key == KEY_LEFT)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_LEFT, ihm->customData);
                }
            }
            else if(key == KEY_RIGHT)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_RIGHT, ihm->customData);
                }
            }
            else if(key == 'm')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_EMERGENCY, ihm->customData);
                }
            }
            else if(key == 'z')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_UP, ihm->customData);
                }
            }
            else if(key == 's')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_DOWN, ihm->customData);
                }
            }
            else if(key == 'q')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_YAW_LEFT, ihm->customData);
                }
            }
            else if(key == 'd')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_YAW_RIGHT, ihm->customData);
                }
            }
            else if(key == 'i')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_UP, ihm->customData);
                }
            }
            else if(key == 'k')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_DOWN, ihm->customData);
                }
            }
            else if(key == 'j')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_LEFT, ihm->customData);
                }
            }
            else if(key == 'l')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_CAM_RIGHT, ihm->customData);
                }
            }
            else if(key == ' ')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_TAKEOFF_LANDING, ihm->customData);
                }
            }
            //++++++++++++++++++Case Added +++++++++++++++++++++++++++++++++
            else if (key == '1')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_AUTOLINEMODE, ihm->customData);
                }
            }

            else if (key == '2')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_AUTOSQUAREMODE, ihm->customData);
                }
            }

            else if (key == '3')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_AUTOCIRCLEMODE, ihm->customData);
                }
            }

            else if (key == '4')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_AUTOEIGHTMODE, ihm->customData);
                }
            }

            else if (key == '5')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_AUTOVERTICALCIRCLEMODE, ihm->customData);
                }
            }

            else if (key == 'r')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_RESET, ihm->customData);
                }
            }

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            else
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
                }
            }
            
            usleep(10);
        }
    }
    
    return NULL;
}

void IHM_PrintHeader(IHM_t *ihm, char *headerStr)
{
    if (ihm != NULL)
    {
        move(HEADER_Y, 0);   // move to begining of line
        clrtoeol();          // clear line
        mvprintw(HEADER_Y, HEADER_X, headerStr);
    }
}

void IHM_PrintInstruction(IHM_t *ihm, char *instructionStr)
{
    if (ihm != NULL)
    {
        move(INSTRUCTION_Y, 0);   // move to begining of line
        clrtoeol();          // clear line
        mvprintw(INSTRUCTION_Y, INSTRUCTION_X, instructionStr);
    }
}

void IHM_PrintInfo(IHM_t *ihm, char *infoStr)
{
    if (ihm != NULL)
    {
        move(INFO_Y, 0);    // move to begining of line
        clrtoeol();         // clear line
        mvprintw(INFO_Y, INFO_X, infoStr);
    }
}

void IHM_PrintBattery(IHM_t *ihm, uint8_t percent)
{
    if (ihm != NULL)
    {
        move(BATTERY_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        mvprintw(BATTERY_Y, BATTERY_X, "Battery: %d", percent);
    }
}

//+++++++++++++++++ Print More +++++++++++++++++
void IHM_PrintSpeed(IHM_t *ihm, float speedX, float speedY, float speedZ)
{
    if (ihm != NULL)
    {
        move(SPEED_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        mvprintw(SPEED_Y, SPEED_X, "Speed X: %f    Speed Y: %f    Speed Z: %f", speedX, speedY, speedZ);
    }
}

// void IHM_PrintCurTime(IHM_t *ihm, char *curTime)
// {
//     if (ihm != NULL)
//     {
//         move(TIME_Y, 0);     // move to begining of line
//         clrtoeol();             // clear line
//         mvprintw(TIME_Y, TIME_X, "Current Time: %s", curTime);
//     }
// }

void IHM_PrintAltitude(IHM_t *ihm, double altitude)
{
    if (ihm != NULL)
    {
        move(ALTITUDE_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        mvprintw(ALTITUDE_Y, ALTITUDE_X, "Altitude: %g", altitude);
    }
}

void IHM_PrintAttitude(IHM_t *ihm, float roll, float pitch, float yaw){
    if (ihm != NULL)
    {
        move(ATTITUDE_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        mvprintw(ATTITUDE_Y, ATTITUDE_X, "Roll: %f    Pitch: %f    Yaw: %f", roll,pitch,yaw);
    }
}

void IHM_PrintTimePassed(IHM_t *ihm, double time){
    if (ihm != NULL)
    {
        move(TIME_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        mvprintw(TIME_Y, TIME_X, "Time Passed: %f seconds", time);
    }
}

// void IHM_PrintWobbleCancellationState(IHM_t *ihm, uint8_t enable){
//     if (ihm != NULL)
//     {
//         move(WCSTATE_Y, 0);     // move to begining of line
//         clrtoeol();             // clear line
//         mvprintw(WCSTATE_Y, WCSTATE_X, "Wobble Cancellation State: %d", enable);
//     }
// }
//++++++++++++++++++++++++++++++++++++++++++++++++
