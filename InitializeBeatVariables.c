#include "InitializeBeatVariables.h"
#include "math.h"
#include "BeatTrackerMath.h"
#define PI (3.14159265)
void InitializeBeatVariables(void) {
	
	//Initialize variables used throughout the tracking application
	
	//First, declare local variables
	
	//Variables for triangle filterbanks
	float centerFreq[numSubbands]; 
	float peakIndexPositions[numSubbands+1]; 
	int firstSubbandPeak = 200; //Start at 200 Hz. Declare variables indicating the location of the peak centers
	//Variables for tempo calculation
	int sigma=20;
	int tempovals[42];
	int index;
	
	//Now, initialize variables
	
	//Initialize frame variables
	fStart=1;
	fHop = frameSize; //Use a full hop
	fEnd = frameSize;
	fNum = 1;

	//Initialize the triangle filterbanks
	for (i=1;i<8;i++) {
		centerFreq[i] = firstSubbandPeak*pow(2,i-1); //Define each peak as being at twice the frequency of the previous one
		peakIndexPositions[i] = round(centerFreq[i]/(fs/frameSize)); //Convert frequency to the index of the frame spectrums
	}
	centerFreq[0]=0;
	peakIndexPositions[0]=0;
	peakIndexPositions[numSubbands]=frameSize/2-1; //Define the first and last peaks
	
	for (i=0;i<frameSize;i++) {
		for (j=0;j<numSubbands;j++) {
			triangleSubbandFilters[i][j]=0; //Initialize subbands to 0
		}
	}
	
	
	for (j=0;j<numSubbands;j++) {

		if (j==0) {
			MakeSubbandFilter(triangleSubbandFilters,j,0,(int) peakIndexPositions[j],(int)peakIndexPositions[j+1],0,1,0,0);
		}
		else {
			if (j<(numSubbands-1)) {
				MakeSubbandFilter(triangleSubbandFilters,j,(int)peakIndexPositions[j-1],(int)peakIndexPositions[j],(int)peakIndexPositions[j+1],1,1,0,0);
			}
			else {
				MakeSubbandFilter(triangleSubbandFilters,j,(int)peakIndexPositions[j-1],(int)peakIndexPositions[j],(int)peakIndexPositions[j+1],1,1,0,.22);
			}
		}
	}
	
	//Initialize the subband history buffers and variables that will be used to weight the subbands
	
	for (i=0;i<autoCorrSize;i++) {
		for (j=0;j<numSubbands;j++) {
		onsetFunction[i][j]=0;
		}
	}
	for (i=0;i<numSubbands;i++) {
		autoCorrRatio[i]=1;
		currentSubbandWeights[i]=0;
		prevEnergy[i]=0;
		curEnergy[i]=0;
	}
	
	for (i=0;i<30000;i++) {
		weightHistory[i]= 0;
	}
	
	reweightTempoAndSubband=1;
	framesUntilWeightingDone = autoCorrSize;
	weights[0]=1;
	weights[1] = (double) 5/9;
	weights[2] = (double) 5/14;
	weights[3] = (double) 5/29;
	weights[4] = (double) 5/57;
	weights[5] = (double) 5/114;
	weights[6] = (double) 5/228;
	weights[7] = (double) 5/456; //Define the weights that will be used for the subbands

	weightThreshold=-.2;
	weightHistoryIndex=0;
	
	//Define variables that will be used to weight the tempo values
	
	for (i=0;i<numPDFs;i++) {
		tempoPDFCenters[i]=80+i*20; //From 80 to 160 BPM in increments of 20 BPM
	}
	
	tempoPDFChoice=3;
	bPMDelayConversion = (float) fs*60/(fEnd-fStart);
	twoFiftyPoint = ceil(bPMDelayConversion/250);
	//printf("Two Fifty Point: %i \n",twoFiftyPoint);
	fiftyPoint = floor(bPMDelayConversion/50);
	for (i=twoFiftyPoint;i<=fiftyPoint;i++) {
		index = i-twoFiftyPoint;
		tempovals[index] = ceil(bPMDelayConversion/i);
	}
	
	
	for (i=0;i<5;i++) {
		for (j=0;j<42;j++) {
			pDF[i][j] = sqrt(1/(2.0*PI*sigma*sigma))*exp(-1/(sigma*sigma*2.0)*(tempovals[j]-tempoPDFCenters[i])*(tempovals[j]-tempoPDFCenters[i])); //Calculate the PDFs
		}
	}
	
	for (i=0;i<42;i++) {
		TempoHold2[i]=0;
	}
	
	
	//Initialize variables that will be used to find the beat locations

	frameEnergy=0;
	energyThreshold=.8;
	shiftVec[0]=0;
	shiftVec[1]=1;
	shiftVec[2]=1;
	shiftVec[3]=1;
	
	for (i=0;i<1000;i++) {
		beatsDetected[i]=0;
		beatsPredicted[i]=0;
		beatCon[i]=0;
	}
	beatPredicted[0] = 0;
	
	
	//Initialize variables that will be used to detect beat patterns
	for (i=0;i<5;i++) {
		beatHistory_Pattern[i]=0;
		for (j=0;j<5;j++) {
			beatHistory_Tempo[i][j] = 0;
		}
	}
	beatSpacing=0;
	lastBeat=0;
	
	//Initialize variables that will be used to detect double beating
	doubleBeatDetect_Beats=0;
	doubleBeatDetect_Pattern=0;
	for (i=0;i<5;i++) {
		doubleBeatDetect_Tempo[i]=0;
	}
	minBeatSpacing=16;

	//Initialize FFT twiddle factors
	computeTwiddleFactors(frameTwid, frameSize, 1);
	computeTwiddleFactors(autoTwid, autoCorrSize*2, 1);
	computeTwiddleFactors(invAutoTwid,autoCorrSize*2, -1);
	
	Confidence = 0;
	
	for (i=0;i<autoCorrSize+1;i++) {
		diffFrameEnergy[i]=0;
	}
	
}

void MakeSubbandFilter(float triFilterBank[][8],int band,int startPosition,int peakPosition,int endPosition,int firstHalf,int secondHalf,float startMag,float endMag) {
	//Calculates the triangle filters used to separate the audio into subbands
	int counter;

	if (firstHalf==1) {
		double ratio = (double) (1-startMag)/(peakPosition-startPosition);
		counter=0;
		for (i=startPosition;i<=peakPosition;i++) {
			
			triFilterBank[i][band] = counter*ratio;
			counter++;
		}
	}
	if (secondHalf==1) {
		double ratio = (double) (1-endMag)/(endPosition-peakPosition);
		counter=0;
		for (i=peakPosition;i<=endPosition;i++) {
				
			triFilterBank[i][band] = 1-counter*ratio;
			counter++;
		}	
	}
	
	
	
	
}
