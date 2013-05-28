/* SFUN_MATADD matrix support example.
 *   C-MEX S-function for matrix add with one input port, one output port and
 *   one parameter.
 *
 *  Input Signal:  2-D or n-D array
 *  Parameter:     2-D or n-D array
 *  Output Signal: 2-D or n-D array
 *
 *  Input        parameter     output
 *  --------------------------------
 *  scalar       scalar        scalar
 *  scalar       matrix        matrix     (input scalar expansion)
 *  matrix       scalar        matrix     (parameter scalar expansion)
 *  matrix       matrix        matrix
 *
 *  Author: M. Shakeri
 *  Copyright 1990-2009 The MathWorks, Inc.
 *  $Revision: 1.1.6.1 $  $Date: 2009/03/31 00:16:07 $
 */
#define S_FUNCTION_NAME  multi_inputs
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "ardrone/ardrone.h"
#include "ardrone/uvlc.h"



/*----------------Sfunc macros----------------------------------*/
#define NUM_INPUTS          2
/* Input Port  0 */
#define IN_PORT_0_NAME      u0_packets
#define INPUT_0_WIDTH       1
#define INPUT_DIMS_0_COL    24000
#define INPUT_0_DTYPE       uint8_T
#define INPUT_0_COMPLEX     COMPLEX_NO
#define IN_0_FRAME_BASED    FRAME_NO
#define IN_0_BUS_BASED      0
#define IN_0_BUS_NAME       
#define IN_0_DIMS           2-D
#define INPUT_0_FEEDTHROUGH 1
#define IN_0_ISSIGNED        0
#define IN_0_WORDLENGTH      8
#define IN_0_FIXPOINTSCALING 1
#define IN_0_FRACTIONLENGTH  9
#define IN_0_BIAS            0
#define IN_0_SLOPE           0.125
/* Input Port  1 */
#define IN_PORT_1_NAME      u1_packet_size
#define INPUT_1_WIDTH       1
#define INPUT_DIMS_1_COL    1
#define INPUT_1_DTYPE       uint32_T
#define INPUT_1_COMPLEX     COMPLEX_NO
#define IN_1_FRAME_BASED    FRAME_NO
#define IN_1_BUS_BASED      0
#define IN_1_BUS_NAME       
#define IN_1_DIMS           1-D
#define INPUT_1_FEEDTHROUGH 1
#define IN_1_ISSIGNED        0
#define IN_1_WORDLENGTH      8
#define IN_1_FIXPOINTSCALING 1
#define IN_1_FRACTIONLENGTH  9
#define IN_1_BIAS            0
#define IN_1_SLOPE           0.125

#define NPARAMS              0

#define SAMPLE_TIME_0        INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES      0
#define DISC_STATES_IC       [0]
#define NUM_CONT_STATES      0
#define CONT_STATES_IC       [0]

#define SFUNWIZ_GENERATE_TLC 1
#define SOURCEFILES "__SFB__"
#define PANELINDEX           6
#define USE_SIMSTRUCT        0
#define SHOW_COMPILE_STEPS   0                   
#define CREATE_DEBUG_MEXFILE 0
#define SAVE_CODE_ONLY       0
#define SFUNWIZ_REVISION     3.0
/*--------------------------------------------------------------*/

/*Define some video format related stuff*/
#define format_width  320
#define format_height  240

int isInitialized = 0;
int isVideoSocketInit = 0;

UDPSocket sockCommand;
UDPSocket sockNavdata;
UDPSocket sockVideo;

WSAData wsaData;

unsigned long int seq = 1;
char ip[16];
    

//bufferBGR = 0;
//bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height) * sizeof(uint8_t));    
//av_free(bufferBGR);
/*Done defining video format related stuff*/
/*--------------------------------------------------*/
/*--------------------------------------------------*/
// //   
// // enum {PARAM = 0, NUM_PARAMS};
// // 
// // #define PARAM_ARG ssGetSFcnParam(S, PARAM)
// // 
// // #define EDIT_OK(S, ARG) \
// // (!((ssGetSimMode(S) == SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ARG)))
// // 
// // 
// // #ifdef MATLAB_MEX_FILE
// // #define MDL_CHECK_PARAMETERS
// // /* Function: mdlCheckParameters =============================================
// //  * Abstract:
// //  *    Verify parameter settings.
// //  */
// // static void mdlCheckParameters(SimStruct *S)
// // {
// //     if(EDIT_OK(S, PARAM_ARG)){
// //         /* Check that parameter value is not empty*/
// //         if( mxIsEmpty(PARAM_ARG) ) {
// //             ssSetErrorStatus(S, "Invalid parameter specified. The parameter "
// //             "must be non-empty");
// //             return;
// //         }
// //     }
// // } /* end mdlCheckParameters */
// // #endif


#define MDL_START                      /* Change to #undef to remove function */
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
 
//     //initialize drone
//     strncpy(ip, ARDRONE_DEFAULT_ADDR, 16);
//     seq = 0; //reset sequence counter
//     if (!sockCommand.open(ip, ARDRONE_COMMAND_PORT)) {
//         //CVDRONE_ERROR("UDPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_COMMAND_PORT, __FILE__, __LINE__);
// 	   printf("hello world your drone doesnt work \n");
//         //return 0;
//     }
// 
// 	// Set video codec
// 	sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", seq++, 0x20);   // UVLC_CODEC
// 	Sleep(100);
//         
// 	// Set video channel to default
// 	sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"0\"\r", seq++);
// 	Sleep(100);
//     
//     	//NOTE: we may not need to set these.. consider removing or putting a check in between
// 	sockCommand.sendf("AT*COMWDG=%d\r", seq++); //reset watchdog timer	
// 	sockCommand.sendf("AT*REF=%d,290717952\r", seq++);  //reset emergency
//     //Sleep(500); //why is this needed???
//     sockVideo.open(ip, ARDRONE_VIDEO_PORT); //we need to constantly open the video port?
// 	isInitialized = 1;
    
}
#endif /*  MDL_START */





/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Initialize the sizes array
 */
static void mdlInitializeSizes(SimStruct *S)
{
    
    /*-----Inputs-------------*/
    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, NPARAMS);
     if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
	 return; /* Parameter mismatch will be reported by Simulink */
     }

    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);

    if (!ssSetNumInputPorts(S, 2)) return;
    /*Input Port 0 */
    inputDimsInfo.width = INPUT_0_WIDTH;
    ssSetInputPortDimensionInfo(S,  0, &inputDimsInfo);
    ssSetInputPortMatrixDimensions(  S , 0, INPUT_0_WIDTH, INPUT_DIMS_0_COL);
    ssSetInputPortFrameData(S,  0, IN_0_FRAME_BASED);

    ssSetInputPortDataType(S, 0, SS_UINT8);
    ssSetInputPortComplexSignal(S,  0, INPUT_0_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/

    /*Input Port 1 */
    ssSetInputPortWidth(S,  1, INPUT_1_WIDTH); /* */
    ssSetInputPortDataType(S, 1, SS_UINT32);
    ssSetInputPortComplexSignal(S,  1, INPUT_1_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 1, INPUT_1_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/
    
    
    /*-----Outputs-------------*/
       
    /* Allow signal dimensions greater than 2 */
    ssAllowSignalsWithMoreThan2D(S);
    
    /* Set number of input and output ports */
    if (!ssSetNumOutputPorts(S,1)) return;
    
    /* Set dimensions of input and output ports */
    
        int_T my_dims[3]; //create a 3x3x3 matrix
   
        DECL_AND_INIT_DIMSINFO(di); /* Initializes structure */
//         int_T              pSize = mxGetNumberOfDimensions(PARAM_ARG);
//         const int_T       *pDims = mxGetDimensions(PARAM_ARG);
            
        my_dims[0] = format_height;
        my_dims[1] = format_width;
        my_dims[2] = 3;
            
        di.width   = format_height*format_width*3;
        di.numDims = 3;
        di.dims    = my_dims;
        if(!ssSetOutputPortDimensionInfo(S, 0, &di)) return;
    
         
    ssSetNumSampleTimes(S, 1);
    
    
    
    /*----------*/
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR | 
		     SS_OPTION_WORKS_WITH_CODE_REUSE));
  
} /* end mdlInitializeSizes */


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
  ssSetInputPortDataType( S, 0, SS_DOUBLE);
 ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *   Compute the outputs of the S-function.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *y     = ssGetOutputPortRealSignal(S,0);
	unsigned char OutputValue;
    unsigned int total_packets;
    unsigned char fill_buffer[122880];
    int increment = 0;

//     const uint8_T   *u0_packets  = (const uint8_T*) ssGetInputPortSignal(S,0); //packet vector
//     const uint32_T   *u1_packet_size  = (const uint32_T*) ssGetInputPortSignal(S,1); //packet size
//     real_T        *y0  = (real_T *)ssGetOutputPortRealSignal(S,0);
//     
//     total_packets = u1_packet_size[0]; //grab packet size
//    
//     for (increment = 0; increment<total_packets;increment++)
//     {
// 	fill_buffer[increment] = u0_packets[increment]; //fill buffer with input values
//     }
//     
    unsigned char  *u0_packets  = (unsigned char*) ssGetInputPortSignal(S,0); //packet vector
    unsigned int   *u1_packet_size  = (unsigned int*) ssGetInputPortSignal(S,1); //packet size
   
    
    total_packets = u1_packet_size[0]; //grab packet size
//    
//     for (increment = 0; increment<total_packets;increment++)
//     {
// 	fill_buffer[increment] = u0_packets[increment]; //fill buffer with input values
//     }
//   
    // Video
    AVFormatContext *pFormatCtx;
    AVCodecContext  *pCodecCtx;
    AVFrame         *pFrame, *pFrameBGR;
    uint8_t  *bufferBGR;
    SwsContext      *pConvertCtx;

    pFormatCtx  = NULL;
    pCodecCtx   = NULL;
    pFrame      = NULL;
    pFrameBGR   = NULL;
    bufferBGR   = NULL;
    pConvertCtx = NULL;  

    // Initialize FFmpeg
    av_register_all();
    avformat_network_init();
    av_log_set_level(AV_LOG_QUIET);
     
     // Set codec
	pCodecCtx = avcodec_alloc_context3(NULL);
	pCodecCtx->width = format_width;
	pCodecCtx->height = format_height;
//     
    bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height) * sizeof(uint8_t));    


	if (total_packets > 0) {
    // Decode UVLC video
  // UVLC::DecodeVideo(fill_buffer, total_packets, bufferBGR, &pCodecCtx->width, &pCodecCtx->height);
	 UVLC::DecodeVideo(u0_packets, total_packets, bufferBGR, &pCodecCtx->width, &pCodecCtx->height);
    }
    
    int BGRsize = avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height) * sizeof(uint8_t);
    int total_count = 0;    

    
    for (int i = 0;i<format_height; i++) //height
    {
        for (int j=0;j<format_width;j++) //width
        {
            for (int k=2;k>=0;k--)//reverse order since output is BGR, very annoying
            {
            //*(output + k + j*3 +i*9) = total_count; //column wise
           // *(y + (format_height*format_width)*k + j*format_height + i) = *(bufferBGR + total_count);  //row wise
            *(y + (format_height*format_width)*k + j*format_height + i)  = *(bufferBGR + total_count);
                total_count = total_count + 1;
            }
        }
    
    }
    // Deallocate the buffer
    av_free(bufferBGR);
    bufferBGR = NULL;
    // Deallocate the codec
    avcodec_close(pCodecCtx);
    pCodecCtx = NULL;
    
    
  

} /* end mdlOutputs */





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    Called when the simulation is terminated.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
	//sockVideo.close();
    //sockNavdata.close();
    //sockCommand.close();
 
} /* end mdlTerminate */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

/* [EOF] sfun_matadd.c */
