/*
 * BeatTracker main driver
 * To be used for DARwIn Dance Synthesis Project.
 * Code originally by David Grunberg (spring 2012)
 * Revised by Mark Koh (winter 2014)
 *
 * Description:
 * This program reads an input signal from the line in port on DARwIn (hopefully this
 * will be extended to microphone as well) and tracks the beats in it.  When the porgram
 * recieves a beat, it sends a UDP packet to localhost at port 9930, where the
 * DanceSynth application will recieve it.
 *
 * Currently, the DanceSynth program will spawn a process of this application so that
 * they do not need to be run separately.  This application may be run standalone, however.
 *
 * **PortAudio**
 * This program uses the PortAudio Portable Audio Library.
 * For more information see: http://www.portaudio.com
 * Copyright (c) 1999-2000 Ross Bencina and Phil Burk
 *
 */

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>

// Our libraries
#include "portaudio.h"
#include "BeatTrackerMath.h"
#include "BeatTracker.h"
#include "InitializeBeatVariables.h"


// UDP settings 
#define BUFLEN 70
#define NPACK 1000
#define PORT 9930
#define SRV_IP "127.0.0.1"

// Useful definitions
#define PI (3.14159265)

//PortAudio definitions
#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (512)
#define NUM_CHANNELS (2)
/* #define DITHER_FLAG     (paDitherOff) */
#define DITHER_FLAG     (0) /**/


// Input and output devices.  This may change to a reference instead of a macro
// THIS IS SOMEHOW THE ISSUE
#define INPUT_DEVICE (Pa_GetDefaultInputDevice())
//#define INPUT_DEVICE (1)
#define OUTPUT_DEVICE (Pa_GetDefaultOutputDevice())

//Change these values if you want to use a short input/output instead of a float
#define USE_FLOAT_INPUT        (1)
#define USE_FLOAT_OUTPUT       (1)

#if USE_FLOAT_INPUT
#define INPUT_FORMAT  paFloat32
typedef float INPUT_SAMPLE;
#else
#define INPUT_FORMAT  paInt16
typedef short INPUT_SAMPLE;
#endif

#if USE_FLOAT_OUTPUT
#define OUTPUT_FORMAT  paFloat32
typedef float OUTPUT_SAMPLE;
#else
#define OUTPUT_FORMAT  paInt16
typedef short OUTPUT_SAMPLE;
#endif

// Macro functions (used for wire callback.  Way faster than function poiters)
#define CONVERT_IN_TO_OUT(in)  ((OUTPUT_SAMPLE) ((in) * gInOutScaler))

// Declare our global variables
int beatHere;
int lastBeatLocation[2];
int beatDifferences;
int beatDifferencesO;
int beatDifferencesx[3];
int beatSend;
double TimeVar;

//Set up a scaling variable
double gInOutScaler = 1.0;
struct sockaddr_in si_other;
int s, slen=sizeof(si_other);
char buf[BUFLEN];
char buf3[2];
int myIndex;
int GesBuf[700]; //changed from 300 size to 700 sizei
int GesBuf2[30000];
struct timeval currentTime;


// Set up our structs for PortAudio
typedef struct WireConfig_s
{
  int isInputInterleaved;
  int isOutputInterleaved;
  int numInputChannels;
  int numOutputChannels;
  int framesPerCallback;
} WireConfig_t;


// Function prototypes
static PaError TestConfiguration( WireConfig_t *config );
static int wireCallback( const void *inputBuffer, 
    void *outputBuffer, 
    unsigned long framesPerBuffer, 
    const PaStreamCallbackTimeInfo* timeInfo, 
    PaStreamCallbackFlags statusFlags, 
    void *userData );
int SoundIsDone();
int ErrorOccurred(PaError err);
void initBeatTracker();

void printStreamParameters(const PaStreamParameters _params);
void printDeviceInfo(const PaDeviceInfo *device);
/***********************************************************************************************/
/*                             Main Program Execution                                          */
/***********************************************************************************************/

// Our main function
int main(int argc, char** argv) {

  printf("Sanity check...\n");


  PaError err = paNoError;
  WireConfig_t CONFIG;
  WireConfig_t *config = &CONFIG;
  //int configIndex = 0;


  //Initialize BeatTracker data
  initBeatTracker();

  //Check for errors
  err = Pa_Initialize();
  if( err != paNoError )
    printf("Error occurred while initializing PortAudio.");
    //ErrorOccurred(err);

  //I don't think we need this at all...
  //Remove
  buf[0]=255;
  buf[1]=103;
  buf[2]=114;


  s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(PORT);
  if (inet_aton(SRV_IP, &si_other.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }

  //We can use this when trying to change input devices
  printf("Default input device: %d, Output: %d\n",(int) INPUT_DEVICE, (int) OUTPUT_DEVICE);



  //Set the config variable, which controls the number of channels, whether the sound is interleavened, and how many samples are gathered in one frame
  config->isInputInterleaved = 0;
  config->isOutputInterleaved = 0;
  config->numInputChannels = 1;
  config->numOutputChannels = 1;
  config->framesPerCallback = frameSize;

  //Initialize the callback.  The program will then stay in this loop until termination is triggered.
  err = TestConfiguration( config ); 

  //If we've made it here, then we're done tracking and everything is healthy
  err = paNoError;

  //Terminate PortAudio and exit
  Pa_Terminate();
  printf("Done tracking.\n"); 
  fflush(stdout);
  return 0;

}

/*
 * This is the big function of the program.  This function will be called
 * whenever PortAudio need audio.  The BeatTracker then adds the current 
 * sample and checks if it's a beat. If it is, then a packet is sent to the
 * server.
 */
static int wireCallback( const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void *userData ) {
  //Define input and output variables
  INPUT_SAMPLE *in;
  OUTPUT_SAMPLE *out;
  int inStride;
  int outStride;
  int inDone = 0;
  int outDone = 0;
  WireConfig_t *config = (WireConfig_t *) userData;
  unsigned int i;
  int inChannel, outChannel;
  time_t tStart = clock();


  printf("In the wire\n");

  /* This may get called with NULL inputBuffer during initial setup. */
  if( inputBuffer == NULL) return 0;
  printf("Still in the wire\n");
  //Get an audio frame
  inChannel=0;
  outChannel=0;
  while( !(inDone && outDone) )
  {
    if( config->isInputInterleaved )
    {
      in = ((INPUT_SAMPLE*)inputBuffer) + inChannel;
      inStride = config->numInputChannels;
    }
    else
    {
      in = ((INPUT_SAMPLE**)inputBuffer)[inChannel];
      inStride = 1;
    }
    BeatTrackFrame(in, beatHere);

    //Check if we have a beat (the set some stuff)
    if (beatPing==1){
      //Put a click in the frame
      in[0]=1;
      lastBeatLocation[0] = lastBeatLocation[1];
      lastBeatLocation[1] = fNum;
      beatDifferences = lastBeatLocation[1]-lastBeatLocation[0];
      beatDifferencesx[0] = beatDifferences;
      beatDifferencesx[1] = beatDifferences*2;
      beatDifferencesx[2] = beatDifferences*3;

    }

    //If we're 200 frames in (make sure we don't start too early)
    if (fNum>200)
    {
      // Make sure this is a frame where we want to start the robot gesture (start halfway between beats)
      if ((fNum==lastBeatLocation[1]+beatDifferencesx[0]/2)) //&& beatDifferences>40 && beatDifferences<60 && fNum-lastBeatLocation[1]>=61)
      {
        if (beatDifferences<24)
        {
          //  beatDifferences=beatDifferences*2;
        }
        //Get beat difference in terms of second (change 1024 to frames per buffer?)
        beatDifferencesO = round((double) (beatDifferences-4)*1024.0/44100.0*100.0);
        buf[0]=255;
        buf[1]=103;
        buf[2]=114;
        buf[3]=beatDifferencesO;
        if (beatDifferences>=17 && fNum-beatSend>15)
        {

          buf[4] = GesBuf2[myIndex];
          myIndex=myIndex+1;
          //Get the time (SYSTEM CALL - REMOVE) (switch frame number)
          gettimeofday(&currentTime, NULL);
          //TimeVar = currentTime.tv_sec+((double)currentTime.tv_usec/100000);
          printf("UDP spot %i\n",buf[4]);
          //printf("TimeVar: %f\n",TimeVar);
          buf3[0] = 255;
          buf3[1] = 116;
          //This may be a flag which is always on
          if (buf[4]>1)
          {
            sendto(s, buf3, 2, 0, (struct sockaddr *)&si_other, slen);
            beatSend = fNum;
            //printf("Sent UDP packet.\n\n");
          }
          //printf("Time Diff: %f\n", (double)Pa_GetStreamTime());
          //tStart=clock();
        }
      }
    }

      //Play the audio (for now it's just a click?)
      if( config->isOutputInterleaved )
      {
        out = ((OUTPUT_SAMPLE*)outputBuffer) + outChannel;
        outStride = config->numOutputChannels;
      }
      else
      {
        out = ((OUTPUT_SAMPLE**)outputBuffer)[outChannel];
        outStride = 1;
      }

      for( i=0; i<framesPerBuffer; i++ )
      {
        *out = CONVERT_IN_TO_OUT(*in );
        out += outStride;
        in += inStride;
      }


    //Check to make sure we're not done (does this apply to input streams?)
    if(inChannel < (config->numInputChannels - 1)) inChannel++;
    else inDone = 1;
    if(outChannel < (config->numOutputChannels - 1)) outChannel++;
    else outDone = 1;
  }
  printf("Leaving the wire\n");

 // return paContinue;
}


//Initialize the BeatTracker data.  This function needs a lot of revision
void initBeatTracker(){

  //Some general initialization stuff
  InitializeBeatVariables();
  lastBeatLocation[0] = 0;
  lastBeatLocation[1] = 0;
  beatDifferences = 0;
  beatSend = 0;

  //Not sure what the hell this is.  
  //Try removing this (once beatTracker is working)
  GesBuf[0]=47;
  GesBuf[1]=47;
  GesBuf[2]=47;
  GesBuf[3]=47;
  GesBuf[4]=47;
  GesBuf[5]=47;
  GesBuf[6]=47;
  GesBuf[7]=47;
  GesBuf[8]=47;
  GesBuf[9]=47;
  GesBuf[10]=48;
  GesBuf[11]=48;
  GesBuf[12]=48;
  GesBuf[13]=48;
  GesBuf[14]=48;
  GesBuf[15]=48;
  GesBuf[16]=48;
  GesBuf[17]=48;
  GesBuf[18]=49;
  GesBuf[19]=50;
  GesBuf[20]=49;
  GesBuf[21]=50;
  GesBuf[22]=49;
  GesBuf[23]=50;
  GesBuf[24]=49;
  GesBuf[25]=50;
  GesBuf[26]=47;
  GesBuf[27]=47;
  GesBuf[28]=47;
  GesBuf[29]=47;
  GesBuf[30]=47;
  GesBuf[31]=47;
  GesBuf[32]=47;
  GesBuf[33]=47;
  GesBuf[34]=47;
  GesBuf[35]=47;
  GesBuf[36]=47;
  GesBuf[37]=47;
  GesBuf[38]=47;
  GesBuf[39]=48;
  GesBuf[40]=48;
  GesBuf[41]=48;
  GesBuf[42]=48;
  GesBuf[43]=48;
  GesBuf[44]=48;
  GesBuf[45]=48;
  GesBuf[46]=48;
  GesBuf[47]=49;
  GesBuf[48]=50;
  GesBuf[49]=49;
  GesBuf[50]=50;
  GesBuf[51]=49;
  GesBuf[52]=50;
  GesBuf[53]=49;
  GesBuf[54]=50;
  myIndex=0;

  //Erm, fill this one in too?
  for (myIndex=0;myIndex<30000;myIndex++) {
    if (fmod(myIndex,2)==0)
    {
      GesBuf2[myIndex]=51;
    }
    else
    {
      GesBuf2[myIndex]=52;
    }
  }
  myIndex=0;


}

static PaError TestConfiguration( WireConfig_t *config )
{
  int c;
  PaError err = paNoError;
  PaStream *stream;
  PaStreamParameters inputParameters, outputParameters;

  // Print info for all available devices 
  int deviceCount = Pa_GetDeviceCount();
  printf("Number of devices: %d\n", deviceCount);
  int i;
  for (i = 0; i < deviceCount; i++) {
      printf("Device %d:\n", i);
      printDeviceInfo(Pa_GetDeviceInfo(i));
  }

  //Errors with the audio input
  inputParameters.device = Pa_GetDefaultInputDevice();              /* default input device */
  //inputParameters.device = 1;
  /*printf("Default Device: %d\n", Pa_GetDefaultInputDevice());
  */
  if (inputParameters.device == paNoDevice) {
    fprintf(stderr,"Error: No default input device.\n");
    return err;
  }

  inputParameters.channelCount = config->numInputChannels;
  inputParameters.sampleFormat = INPUT_FORMAT | (config->isInputInterleaved ? 0 : paNonInterleaved);
  //We want to use this one (instead of the one above) but it breaks beat tracker
  inputParameters.sampleFormat = INPUT_FORMAT;
  inputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = NULL;

  //Errors with the audio output
  outputParameters.device = Pa_GetDefaultOutputDevice();            /* default output device */
  if (outputParameters.device == paNoDevice) {
    fprintf(stderr,"Error: No default output device.\n");
    return err;
  }
  outputParameters.channelCount = config->numOutputChannels;
  outputParameters.sampleFormat = OUTPUT_FORMAT | (config->isOutputInterleaved ? 0 : paNonInterleaved);
  outputParameters.sampleFormat = OUTPUT_FORMAT;
  outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;
  
  // Issue somewhere here too
  printStreamParameters(inputParameters);
  printStreamParameters(outputParameters);

  //Errors with the stream
  err = Pa_OpenStream(
      &stream,
      &inputParameters,
      &outputParameters,
      SAMPLE_RATE,
      //config->framesPerCallback, /* frames per buffer */
      FRAMES_PER_BUFFER, /* frames per buffer */
      paClipOff, /* we won't output out of range samples so don't bother clipping them */
      wireCallback,
      config );
  if( err != paNoError ) {
    printf("Error opening stream\n");
    return err;
  }
  printf("Stream open\n");

  //Can't open the stream
  err = Pa_StartStream( stream );
  if( err != paNoError ) {
    printf("Error starting stream\n");
    return err;
  }

  printf("Stream started\n");

  //Wait for a character from the command line. If it's enter, this will return. 
  c = getchar();

  //Can't close the stream
  printf("Closing stream.\n");
  err = Pa_CloseStream( stream );
  if( err != paNoError ) 
    return err;

  //If the enter key's pressed, return and stop tracking
  if( c == 'q' ) return 1;

}

void printStreamParameters(const PaStreamParameters _params) {
    
    printf("\n");
    printf("device            = %s\n", Pa_GetDeviceInfo(_params.device)->name);
    printf("channelCount      = %d\n", _params.channelCount);
    printf("sampleFormat      = ");
    
    bool flagTrue = false;
    bool lastTrue = false;
    flagTrue = _params.sampleFormat & paFloat32;
    printf("%s", flagTrue ? "paFloat32" : "\b\b\b ");
    lastTrue = flagTrue;
    flagTrue = _params.sampleFormat & paInt32;
    printf("%s%s", lastTrue ? " | " : "", flagTrue ? "paInt32" : "");
    lastTrue = flagTrue;
    flagTrue = _params.sampleFormat & paInt24;
    printf("%s%s", lastTrue ? " | " : "", flagTrue ? "paInt24" : "");
    lastTrue = flagTrue;
    flagTrue = _params.sampleFormat & paInt16;
    printf("%s%s", lastTrue ? " | " : "", flagTrue ? "paInt16" : "");
    lastTrue = flagTrue;
    flagTrue = _params.sampleFormat & paInt8;
    printf("%s%s", lastTrue ? " | " : "", flagTrue ? "paInt8" : "");
    lastTrue = flagTrue;
    flagTrue = _params.sampleFormat & paCustomFormat;
    printf("%s%s", lastTrue ? " | " : "", flagTrue ? "paCustomFormat" : "");
    lastTrue = flagTrue;
    flagTrue = _params.sampleFormat & paNonInterleaved;
    printf("%s%s", lastTrue ? " | " : "", flagTrue ? "paNonInterleaved" : "");
    printf("\n");
    
    printf("suggestedLatency  = %f\n", _params.suggestedLatency);
    printf("\n");
}

void printDeviceInfo(const PaDeviceInfo *device) {
    
    printf( "Name                        = %s\n",       device->name);
    printf( "Host API                    = %s\n",       Pa_GetHostApiInfo(device->hostApi)->name);
    printf( "Max inputs = %d\n",                        device->maxInputChannels);
    printf( "Max outputs = %d\n",                       device->maxOutputChannels);
    printf( "Default low input latency   = %8.4f\n",    device->defaultLowInputLatency);
    printf( "Default low output latency  = %8.4f\n",    device->defaultLowOutputLatency);
    printf( "Default high input latency  = %8.4f\n",    device->defaultHighInputLatency);
    printf( "Default high output latency = %8.4f\n",    device->defaultHighOutputLatency);
}

