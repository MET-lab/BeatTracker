/** @file patest_wire.c
  @ingroup test_src
  @brief Pass input directly to output.

  Note that some HW devices, for example many ISA audio cards
  on PCs, do NOT support full duplex! For a PC, you normally need
  a PCI based audio card such as the SBLive.

  @author Phil Burk  http://www.softsynth.com

  While adapting to V19-API, I excluded configs with framesPerCallback=0
  because of an assert in file pa_common/pa_process.c. Pieter, Oct 9, 2003.

*/
/*
 * $Id$
 *
 * This program uses the PortAudio Portable Audio Library.
 * For more information see: http://www.portaudio.com
 * Copyright (c) 1999-2000 Ross Bencina and Phil Burk
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * The text above constitutes the entire PortAudio license; however, 
 * the PortAudio community also makes the following non-binding requests:
 *
 * Any person wishing to distribute modifications to the Software is
 * requested to send the modifications to the original developer so that
 * they can be incorporated into the canonical version. It is also 
 * requested that these non-binding requests be included along with the 
 * license above.
 */



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "portaudio.h"
#include "BeatTrackerMath.h"
#include "BeatTracker.h"
#include "InitializeBeatVariables.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

//#define SRV_IP "192.168.0.193"
#define BUFLEN 70 //Changed from 5 to 70 
#define NPACK 1000
#define PORT 9930
#define SRV_IP "127.0.0.1"
//Useful definitions
#define PI (3.14159265)
#define SAMPLE_RATE            (44100)

//MARK - EXTRA INPUTS FOR MIC USE
#define FRAMES_PER_BUFFER (512)


//Variable indicating whether a beat is in the frame or not
int beatHere;
int lastBeatLocation[2];
int beatDifferences;
int beatDifferencesO;
int beatDifferencesx[3];
int beatSend;
double TimeVar;

//Define a structure for the audio stream
typedef struct WireConfig_s
{
  int isInputInterleaved;
  int isOutputInterleaved;
  int numInputChannels;
  int numOutputChannels;
  int framesPerCallback;
} WireConfig_t;




#define USE_FLOAT_INPUT        (1)
#define USE_FLOAT_OUTPUT       (1)

/* Latencies set to defaults. */

//Set some types
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
//Set up the input and output devices
#define CONVERT_IN_TO_OUT(in)  ((OUTPUT_SAMPLE) ((in) * gInOutScaler))

#define INPUT_DEVICE           (Pa_GetDefaultInputDevice())
//#define INPUT_DEVICE           1 //This should be the front mic channel
#define OUTPUT_DEVICE          (Pa_GetDefaultOutputDevice())
static PaError TestConfiguration( WireConfig_t *config );

//This is the function that grabs audio for the beat tracker, then plays it back
static int wireCallback( const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void *userData );
/* This routine will be called by the PortAudio engine when audio is needed.
 ** It may be called at interrupt level on some machines so don't do anything
 ** that could mess up the system like calling malloc() or free().
 */

static int wireCallback( const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void *userData )
{
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

  /* This may get called with NULL inputBuffer during initial setup. */
  if( inputBuffer == NULL) return 0;

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
    //BEAT TRACKING CODE
    BeatTrackFrame(in, beatHere);

    if (beatPing==1){
      in[0]=1;
      //sprintf(buf, "%c%c",(char)255,(char)116);
      //sendto(s, buf, BUFLEN, 0, &si_other, slen);
      //if (fNum-lastBeatLocation[1]>120)
      //{

      lastBeatLocation[0] = lastBeatLocation[1];
      lastBeatLocation[1] = fNum;
      beatDifferences = lastBeatLocation[1]-lastBeatLocation[0];
      //}
      beatDifferencesx[0] = beatDifferences;
      beatDifferencesx[1] = beatDifferences*2;
      beatDifferencesx[2] = beatDifferences*3;

      //while (beatDifferences<120)
      //{
      //  beatDifferences = beatDifferences+beatDifferencesO;
      //}
      //  out[0]=1;
    }
    //END BEAT TRACKING CODE

    if (fNum>200)
    {
      //if (fNum==lastBeatLocation[1]+beatDifferences/2) {

      //}

      if ((fNum==lastBeatLocation[1]+beatDifferencesx[0]/2)) //&& beatDifferences>40 && beatDifferences<60 && fNum-lastBeatLocation[1]>=61)
      {
        if (beatDifferences<24)
        {
          //  beatDifferences=beatDifferences*2;
        }
        beatDifferencesO = round((double) (beatDifferences-4)*1024.0/44100.0*100.0);
        //printf("Beat Difference: %i\n",beatDifferencesO);
        //sprintf(buf, "%c%c%c%c%c", (char)255,'r','g',(char)beatDifferencesO,(char)47);
        //buf[0] = beatDifferencesO;
        //buf[1] = fNum;
        buf[0]=255;
        buf[1]=103;
        buf[2]=114;
        buf[3]=beatDifferencesO;
        if (beatDifferences>=17 && fNum-beatSend>15)
        {

          //  buf[4] = fmod(round((fNum+in[0]*109)),20);
          //  if (buf[4]<16)
          //  {
          //    buf[4]=51;
          //  }
          //  else if (buf[4]<19)
          //  {
          //    buf[4]=47;
          //  }
          //  else if (buf[4]==19)
          //  {
          //    buf[4]=52;
          //  }
          //  if (buf[4]==51 || buf[4]==52)
          //  {
          //    if (beatDifferences<24)
          //    {
          //      buf[3]=buf[3]*2;
          //    }
          //  }

          //buf[4]=47;
          //if (beatDifferencesO>100)
          //{
          buf[4] = GesBuf2[myIndex];
          myIndex=myIndex+1;
          //Get the time
          gettimeofday(&currentTime, NULL);
          TimeVar = currentTime.tv_sec+((double)currentTime.tv_usec/100000);
          printf("UDP spot %i\n",buf[4]);
          //printf("TimeVar: %f\n",TimeVar);
          buf3[0] = 255;
          buf3[1] = 116;
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

      //Play the audio
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

      if(inChannel < (config->numInputChannels - 1)) inChannel++;
      else inDone = 1;
      if(outChannel < (config->numOutputChannels - 1)) outChannel++;
      else outDone = 1;
    }
    return 0;
  }

  void sighandler(int sig)
  {
  printf("Signal recieved: %d\n",sig);

  //Pass the signal onto any program in our process group
  //(COMMENT THIS OUT IF YOU'RE NOT CREATING ANY CHILD PROCESSES, OTHERWISE YOU'LL KILL THE SHELL)
  kill(-1,SIGABRT);

  exit(0);
  }
  

  /*******************************************************************/
  int main(void)
  {
  
    //signal(SIGABRT, &sighandler);
    
    
    //Set up some variables for PortAudio
    PaError err = paNoError;
    WireConfig_t CONFIG;
    WireConfig_t *config = &CONFIG;



    //BEAT TRACKER CODE
    InitializeBeatVariables();
    lastBeatLocation[0] = 0;
    lastBeatLocation[1] = 0;
    beatDifferences = 0;
    beatSend = 0;
    //END BEAT TRACKER CODE

    //SERIAL INITIALIZATION CODE
    s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    buf[0]=255;
    buf[1]=103;
    buf[2]=114;
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    if (inet_aton(SRV_IP, &si_other.sin_addr)==0) {
      fprintf(stderr, "inet_aton() failed\n");
      exit(1);
    }

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
    //END SERIAL INITIALIZATION CODE  

    //Check for errors
    err = Pa_Initialize();
    if( err != paNoError )
      ErrorOccurred(err);
    //Set scaling variables for the audio
    if( INPUT_FORMAT == OUTPUT_FORMAT )
    {
      gInOutScaler = 1.0;
    }
    else if( (INPUT_FORMAT == paInt16) && (OUTPUT_FORMAT == paFloat32) )
    {
      gInOutScaler = 1.0/32768.0;
    }
    else if( (INPUT_FORMAT == paFloat32) && (OUTPUT_FORMAT == paInt16) )
    {
      gInOutScaler = 32768.0;
    }

    //printf("Default input device: %d, Output: %d\n",(int) Pa_GetDefaultInputDevice(), (int)Pa_GetDefaultOutputDevice());


    //Set the config variable, which controls the number of channels, whether the sound is interleavened, and how many samples are gathered in one frame
    config->isInputInterleaved = 0;
    config->isOutputInterleaved = 0;
    config->numInputChannels = 1;
    config->numOutputChannels = 1;
    config->framesPerCallback = frameSize;
    err = TestConfiguration( config ); //The 'main' function stays here during the program's execution. err triggers when the user hits the 'enter' key
    //Only get here after pressing 'enter.' This indicates that we're done tracking.
    FILE *f1;
    f1 = fopen("beats","wt");
    for (i=0;i<1000;i++) {
      //fprintf(f1,"Detected: %i\n",beatsDetected[i]);
      //printf("Detected: %i\n", beatsDetected[i]);
    }
    fclose(f1);
    err = paNoError;

    SoundIsDone();
  }

  static PaError TestConfiguration( WireConfig_t *config )
  {
    int c;
    PaError err = paNoError;
    PaStream *stream;
    PaStreamParameters inputParameters, outputParameters;

    //Check for a variety of errors

    //Errors with the audio input
    inputParameters.device = INPUT_DEVICE;              /* default input device */
    /*printf("Default Device: %d\n", Pa_GetDefaultInputDevice());
      inputParameters.device = 2;

    //int DeviceCount = Pa_GetDeviceCount();
    printf("Number of devices: %d\n",Pa_GetDeviceCount());*/
    if (inputParameters.device == paNoDevice) {
      fprintf(stderr,"Error: No default input device.\n");
      return err;
    }
    inputParameters.channelCount = config->numInputChannels;
    inputParameters.sampleFormat = INPUT_FORMAT | (config->isInputInterleaved ? 0 : paNonInterleaved);
    
    inputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    //Errors with the audio output
    outputParameters.device = OUTPUT_DEVICE;            /* default output device */
    if (outputParameters.device == paNoDevice) {
      fprintf(stderr,"Error: No default output device.\n");
      return err;
    }
    outputParameters.channelCount = config->numOutputChannels;
    outputParameters.sampleFormat = OUTPUT_FORMAT | (config->isOutputInterleaved ? 0 : paNonInterleaved);
    outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    //Errors with the stream
    err = Pa_OpenStream(
        &stream,
        &inputParameters,
        &outputParameters,
        SAMPLE_RATE,
        config->framesPerCallback, /* frames per buffer */
        paClipOff, /* we won't output out of range samples so don't bother clipping them */
        wireCallback,
        config );
    if( err != paNoError )
      return err;

    //Can't open the stream
    err = Pa_StartStream( stream );
    if( err != paNoError ) 
      return err;
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

  int SoundIsDone() {
    Pa_Terminate();
    printf("Done tracking.\n"); fflush(stdout);
    printf("Hit ENTER to quit.\n");  fflush(stdout);
    getchar();
    return 0;
  }

  int ErrorOccurred(PaError err) {
    Pa_Terminate();
    fprintf( stderr, "An error occured while using the portaudio stream\n" );
    fprintf( stderr, "Error number: %d\n", err );
    fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
    printf("Hit ENTER to quit.\n");  fflush(stdout);
    getchar();
    return -1;
  }
