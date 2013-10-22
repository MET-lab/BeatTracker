#ifndef INITIALIZEBEATVARIABLES_H
#define INITIALIZEBEATVARIABLES_H
#include "math.h"

//Declares variables to be used in the beat tracker
//First, global parameters
#define fs 44100
#define frameSize 1024
#define numSubbands 8
#define numPDFs 5
#define timeInHistory 5.95
#define autoCorrSize 256//round(timeInHistory*fs/frameSize)
//int fs ; //Sampling frequency
//float timeInHistory ; //Length of time (in seconds) used for autocorrelation and energy histories
//int numSubbands ; //Number of subbands
//int frameSize ; // Size of the audio frame
//int autoCorrSize; //Numbero f frames in autocorrelation and related vectors
//#define autoCorrSize = round(timeInHistory*fs/frameSize);
//Frame variables
int fStart;
int fHop;
int fEnd;
int fNum;

//Loop variables
int i,j,n,o;

//FFT twiddle factors
float frameTwid[frameSize]; //Twiddles to take the FFT of the audio
float autoTwid[autoCorrSize*2]; //Twiddles to take the FFT of the onset functions (for autocorrelation)
float invAutoTwid[autoCorrSize*2]; //Twiddles to take the IFFT of the onset functions (for autocorrelation)

//Onset function detection, subband filter, and subband weighting variables
float inFrame[frameSize*2]; //Input frame, both real and complex values.

float triangleSubbandFilters[frameSize][numSubbands];

int reweightTempoAndSubband; //Flag indicating whether the subband weights and the tempo pdf need to be recalculated
int framesUntilWeightingDone; //Number of frames that the subband weights and tempo pdf will be recalculated, assuming everything else goes right

float weights[numSubbands]; //Bank of weights to multiply the subbands by
float currentSubbandWeights[numSubbands]; //The particular weights being used for a given frame
float weightHistory[30000]; //Keeps the history of prior weightings so that an average over history can be found. Eventually implement a circular buffer here
int weightHistoryIndex; //Location in the weightHistory matrix to list the current 'most periodic' subband
int SubPeak; //Most periodic subband for a given frame
double weightThreshold; //Parameter used to determine if subbands are sufficiently periodic

float autoCorrRatio[numSubbands]; //Ratio of autocorrelation maximums to energies; used for weighting subbands
int AnyRatio; //Flag indicating that at least one subband is sufficiently periodic for reweighting
float ratioDiff[numSubbands][2]; //Difference between the subband ratios
float ratioCheck[numSubbands][3]; //Flags indicating which subbands have relatively strong ratios
float RatioDifferences[numSubbands]; //Combined differences between each ratio and the first and second subband ratios (current ratio*2 - first ratio - second ratio)
float maxRatioDiff[2]; //Used to contain the maximum RatioDifferences value

float subbandModeCount[numSubbands]; //Contains the number of times in the last six seconds each subband was the peak
float maxSubbandMode[2]; //The maximum of the subbandModeCount variable; the mode (subband that was the most often the most periodic) and the location of the mode (i.e., which subband it belongs to)
int maxSubband; //The subband value in maxSubbandMode


float frameSpectrum[frameSize*2]; //Spectrum of the current frame
float frameSpectrumMag[frameSize]; //Magnitude spectrum of the current frame
float SummedSub[numSubbands]; //Energy values for each subband in the frame; found by multiplying the magnitude spectrum by the filterbanks and the weights
float prevEnergy[numSubbands]; //SummedSub of the previous frame
float curEnergy[numSubbands]; //SummedSub of the current frame
float onsetFunction[autoCorrSize][numSubbands]; //The onset function for each subband; found by subtracting prevEnergy from curEnergy, then rectifying the result


//Autocorrelation variables
float oneSubbandDiffEnergy[autoCorrSize*4]; //onsetFunction for a subband, padded with 0's so that the FFT is twice the size of the function (and with 0's for complex values as well)
float autoCorrOutOneSub[autoCorrSize*4]; //FFT of a subband onset function
float autoCorrOutOneSubMag[autoCorrSize*2]; //Magnitude spectrum of a subband onset function
float autoMag[autoCorrSize*2][numSubbands]; //Buffer to save the magnitude spectrums of all onset functions

float toIFFT[autoCorrSize*4]; //Magnitude spectrum, with 0's for the complex components
float fftOut[autoCorrSize*4]; //IFFT of a magnitude spectrum
float fftCorr[autoCorrSize*2]; //fftOut, sorted to be an autocorrelation. Used to determine the autocorrelation ratios
float tempobank[autoCorrSize*2-1]; //Bank of tempo values. The sum of the fftCorr values for all subbands

float TempoHold[245]; //Values from tempobank corresponding to tempos between 50 and 250 BPM. 
float maxAll[2]; //Used to store the maximum value of each autocorrelation (equivalent to its total energy)
float maxTempos[2]; //Used to store the maximum value of TempoHold for each autocorrelation. 
//maxTempos and maxAll are used to find the autocorrelation ratios


//Tempo estimation variables
float pDF[numPDFs][42]; //PDFs to weight the tempo values
float tempoPDFCenters[numPDFs]; //Centers of the tempo PDFs then the predicted tempos from each PDF, Units are beats per minute.
int tempoPDFChoice; //Index of the PDF being used to weight tempo values for a given frame
float bPMDelayConversion; //Parameter used to convert lags to tempos
int twoFiftyPoint; //Upper tempo bound; lag position where tempo=250 beats per minute
int fiftyPoint; //Lower tempo bound; lag position where tempo=50 beats per minute
int doubleBeatDetect_Tempo[numPDFs]; //Determines if current frame at a given tempo is too close to a previous beat to contain a beat itself. 
float beatHistory_Tempo[numPDFs][5]; //Stores a few beat locations for each tempo to evaluate if there is a consistent pattern
float diffBeats[5]; //Difference of beats in beatHistory_Tempo
float allDelays[numPDFs]; //Delays for the tempos selected by using each PDF to weight the tempobank variable
int tempoFlagging[numPDFs]; //Flags indicating which PDFs are producing consistent beats
int tempoFlag; //A single flag indicating which PDF will be used for the final beat prediction
int beatPredicted_Tempo[0]; //Flag indicating whether, for each tempo, a beat was predicted
float tempoMax[2]; //Maximum tempo value and its index
int tempoind; //Index of maximum tempo value; used to calculate what the tempo is
float TempoHold2[42];
float TempoHold3[46][5];
float HelpFulVector[10];

//Beat phase detection variables
float frameEnergy; //Energy in current frame
float prevFrameEnergy; //Energy in previous frame
float enDiff; //Difference between frameEnergy and prevFrameEnergy
float diffFrameEnergy[autoCorrSize]; //History of enDiff values (rectified). 
double energyThreshold; //Parameter used to determine if a frame has enough energy to contain a beat
int shiftVec[4]; //Allows for a little 'jitter' when determining if previous beat locations in diffFrameEnergy were sufficiently large

int doubleBeatDetect_Beats; //Determines if current frame is too close to a previous beat to contain a beat itself. Used for the final beat prediction sequence
int doubleBeatDetect_Pattern; //Determines if current frame is too close to a previous beat to contain a beat itself. Used for the pattern-recognition portion
int minBeatSpacing; //Sets the minimum required number of frames between beats
int modeCount2[4]; //For the last few beats, determines how often each spacing occurs (for instance, if five beats are each 20 frames away from the next, then '20' occurs 4 times).
int modeIndex; //Marks which spacing is the most common so far 
int modeBeatSpacing; //Most common beat spacing
int beatSpacing; //Stores the average distance between the last few beats. Based on modeBeatSpacing, but only set if the values are all reasonably consistent (spacings in the last few beats are all very similar)
int lastBeat; // Stores location of last beat
float beatHistory_Pattern[5]; //Stores a few beat locations to evaluate if there is a consistent pattern

int delayInd[4]; //Locations of previous beats, given a starting frame and a tempo delay
int chosenDelay; //Delay based on the selected tempo
float FourBeatValue[2]; //Sum of diffFrameEnergy values, spaced at the tempo delay, beginning at the current frame
float OtherBeatValue[1000]; //Sum of diffFrameEnergy values, spaced at the tempo delay, but beginning not at the current frame
float maxFour[2]; //Maximum of FourBeatValue
float maxOther[2]; //Maximum of OtherBeatValue
int beatPredicted[0]; //Flag indicating whether a beat was predicted (based on FourBeatValue and OtherBeatValue)

int beatPing; //Marks whether a beat has been predicted (using the history buffers as well as the energy values)

//Beat vector variables
int beatsDetected[1000];  //Beats found in frames
int beatsPredicted[1000]; //Beats predicted based on the beat locations and tempo
							//For debugging, we need to store all beat locations. Assume 1000 beats max. For the online system we can discard old beats and not worry about exceeding this limit.
float beatCon[1000];			//Confidence levels of various beats
float Confidence;
//Now declare some useful functions
void InitializeBeatVariables(void); //Initializes the variables
void MakeSubbandFilter(float triFilterBank[][8],int band,int startPosition,int peakPosition,int endPosition,int firstHalf,int secondHalf,float startMag,float endMag); //Makes the subband filters


float diffSubEnergies[autoCorrSize][numSubbands]; //This isn't used for anything, but taking it out breaks the system for some reason

#endif //Defining the header file





