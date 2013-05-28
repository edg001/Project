// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2013 puku0x
// https://github.com/puku0x/cvdrone
//
// This source file is part of CV Drone library.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of EITHER:
// (1) The GNU Lesser General Public License as published by the Free
//     Software Foundation; either version 2.1 of the License, or (at
//     your option) any later version. The text of the GNU Lesser
//     General Public License is included with this library in the
//     file cvdrone-license-LGPL.txt.
// (2) The BSD-style license that is included with this library in
//     the file cvdrone-license-BSD.txt.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
// cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
// ARDrone::ARDrone(IP address of your AR.Drone)
// Description : Constructor of ARDrone class.
// --------------------------------------------------------------------------
ARDrone::ARDrone(const char *ardrone_addr)
{
    // IP Address
    strncpy(ip, ARDRONE_DEFAULT_ADDR, 16);

    // Sequence number
    seq = 1;

    // Camera image
    img = NULL;

    // Navdata
    ZeroMemory(&navdata, sizeof(NAVDATA));

    // Video
    pFormatCtx  = NULL;
    pCodecCtx   = NULL;
    pFrame      = NULL;
    pFrameBGR   = NULL;
    bufferBGR   = NULL;
    pConvertCtx = NULL;

    // Thread for commnad
    flagCommand   = 0;
    threadCommand = INVALID_HANDLE_VALUE;
    mutexCommand  = INVALID_HANDLE_VALUE;

    // Thread for navdata
    flagNavdata   = 0;
    threadNavdata = INVALID_HANDLE_VALUE;
    mutexNavdata  = INVALID_HANDLE_VALUE;

    // Thread for video
    flagVideo   = 0;
    threadVideo = INVALID_HANDLE_VALUE;
    mutexVideo  = INVALID_HANDLE_VALUE;

    // When IP address is specified, open it
    if (ardrone_addr) open(ardrone_addr);
}

// --------------------------------------------------------------------------
// ARDrone::~ARDrone()
// Description : Destructor of ARDrone class.
// --------------------------------------------------------------------------
ARDrone::~ARDrone()
{
    // Finalize the AR.Drone
    close();
}

// --------------------------------------------------------------------------
// ARDrone::open(IP address of AR.Drone)
// Description  : Initialize the AR.Drone.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::open(const char *ardrone_addr)
{
    // Initialize WSA
    WSAData wsaData;
    WSAStartup(MAKEWORD(1,1), &wsaData);

    // Initialize FFmpeg
    av_register_all();
    avformat_network_init();
    av_log_set_level(AV_LOG_QUIET);

    // Save IP address
    strncpy(ip, ardrone_addr, 16);

    // Get version informations
    if (!getVersionInfo()) return 0;
    printf("AR.Drone Ver. %d.%d.%d\n", version.major, version.minor, version.revision); //do a simple FTP test, no harm in doing this

    // Initialize AT Command
    if (!initCommand()) return 0; //needed for setting video codec and init stream + channel

    // Initialize Navdata
    if (!initNavdata()) return 0; //dont need nav data, disable this

    // Initialize Video
    if (!initVideo()) return 0;

    // Wait for updating the state
    Sleep(500);

    // Reset emergency
    resetWatchDog();
    resetEmergency();

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::update()
// Description  : Update the informations.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int ARDrone::update(void)
{
    // Check threads
    if (!flagCommand) return 0;
    //if (!flagNavdata) return 0;
    if (!flagVideo) return 0;

    return 1;
}

// --------------------------------------------------------------------------
// ARDrone::close()
// Description  : Finalize
// Return value : NONE
// --------------------------------------------------------------------------
void ARDrone::close(void)
{
    // Stop AR.Drone
    if (!onGround()) landing();

    // Finalize video
    finalizeVideo();

    // Finalize Navdata
    finalizeNavdata();

    // Finalize AT command
    finalizeCommand();

    // Finalize WSA
    WSACleanup();
}