/*
 *  BeatTracker.c
 *  BeatTracker
 *
 *  Created by David Grunberg on 8/29/10.
 *  Copyright 2010 Drexel University. All rights reserved.
 *
 */

#include "BeatTracker.h"
#include "BeatTrackerMath.h"
#include "InitializeBeatVariables.h"
#include "math.h"
#include "stdio.h"

void BeatTrackFrame(float* in, int beatHere) {
	CreateOnsetFunction(in); //Calculate the onset function. High values indicate that a beat is relatively likely.
	UpdateDoubleClickVars(); //Update variables that ensure that there is a minimum spacing between beats
	CalculateTempo(); //Calculate the tempo of the audio
	beatPing=0;
	DetermineBeatPhase(); //Determine if a beat is in this frame
	if (beatPing==1) {
		beatHere=1;
	//	printf("%i\n",fNum); //If there's a beat, display a message indicating this
	}
	
}

void CreateOnsetFunction(float* in) {
	//This function calculates the onset function used to determine tempo and phase
	//The onset function is based on the differentiated spectrum of the frames
	GetSubbandMagSpectrum(in); //Calculate the magnitude spectrum of the input frame
	ReWeightSubbands(); //Weight the subbands to emphasize the most periodic ones
	GetSubbandEnergies(); //Calculates and buffers the energies of the subbands
}

void GetSubbandMagSpectrum(float* in) {
	//Calculate the spectrum of the current frame
	for (i=0;i<frameSize;i++) {
		inFrame[i] = in[i];
		inFrame[i+frameSize] = 0; //Prepare frame for FFTing. The 0's are the complex values.
	}
	FFT(inFrame,frameSize,frameTwid,frameSpectrum,1); //Calculate the spectrum with the FFT
	magnitude(frameSpectrum, frameSize, frameSpectrumMag, 1); //Calculate the magnitude of the spectrum
}

void ReWeightSubbands(void) {
	//Weight the subbands based on their periodicities. Periodicity is determined based on the ratio of the maximum autocorrelation value to the total energy for each subband onset function.
	if (reweightTempoAndSubband==1) { //The subbands are reweighted based on this flag. It is set to 1 if the last few beats were inconsistently spaced within the last 6 seconds.
		AnyRatio=0; //This is a flag indicating whether or not any ratio is large enough to be the maximum subband
		for (i=0;i<numSubbands;i++) {
			for (j=0;j<2;j++) {
				ratioDiff[i][j] = autoCorrRatio[j]-autoCorrRatio[i]; //Find the difference between each ratio and the first two ratios
				if (ratioDiff[i][j]<weightThreshold) { //Flag any of the differences that are larger (in magnitude) than a threshold
					ratioCheck[i][j]=1;
				}
				else {
					ratioCheck[i][j]=0;
				}
			}
			if (ratioCheck[i][0]==1 && ratioCheck[i][1]==1) {
				RatioDifferences[i] = autoCorrRatio[0]+autoCorrRatio[1]-2*autoCorrRatio[i]; //Save any differences that are large enough
				RatioDifferences[i] = RatioDifferences[i]*-1; //This is so larger values are better (so the max function can be used)
				AnyRatio=1; //Set a flag that there's at least one ratio large enough that its subband might be the dominant subband
			}
			else {
				RatioDifferences[i]=0;
			}
		}
		if (AnyRatio==1) { //If at least one ratio is large enough
			max(RatioDifferences,numSubbands,maxRatioDiff);
			SubPeak=maxRatioDiff[1]; //Set the strongest subband as the one with the largest difference between its ratio and the ratios of the first two subbands
		}
		else {
			SubPeak=0; //Otherwise, the first subband is the largest by default
		}
		weightHistory[ weightHistoryIndex] = SubPeak; //Add the subband information to the history
		weightHistoryIndex++;
		for (i=0;i<numSubbands;i++) {
			subbandModeCount[i]=0; //Initialize the mode values
		}
		
		//Find the mode of the last six seconds of the history (or less if we're less than six seconds into the music)
		if (fNum<=autoCorrSize) {
			for (i=0;i<fNum;i++) {
				subbandModeCount[(int) weightHistory[i]]++;
			}
		}
		else {
			for (i=weightHistoryIndex-autoCorrSize;i<=weightHistoryIndex;i++) {
				subbandModeCount[(int) weightHistory[i]]++;
			}
		}
		max(subbandModeCount,numSubbands,maxSubbandMode);
		maxSubband = maxSubbandMode[1]; 
		
		//Set the weights based on proximity to the strongest subband
		for (i=0;i<numSubbands;i++) {
			currentSubbandWeights[i] = (double) weights[(int) fabs(maxSubband-i)];
		}
	}
}

void GetSubbandEnergies(void) {
	//Now that the weights and the subband mangitude spectrums are known, the spectrums are filtered and weighed so that the energies are calculated. Then the energies are buffered and differentiated to make the onset functions
	for (i=0;i<numSubbands;i++) {
		//Determine the energies of the weighted subbands
		SummedSub[i]=0;
		for (j=0;j<frameSize;j++) {
			SummedSub[i] = SummedSub[i]+frameSpectrumMag[j]*triangleSubbandFilters[j][i]*currentSubbandWeights[i]; //Calculate the weighted energies for each subband
		}
		prevEnergy[i]=curEnergy[i];
		curEnergy[i]=SummedSub[i];  
		for (j=0;j<autoCorrSize-1;j++) { 
			onsetFunction[j][i] = onsetFunction[j+1][i]; //Cycle the energies in the buffer. Size of buffer determined experimentally.
		}
	//	onsetFunction[autoCorrSize-1][i]=0;
		onsetFunction[autoCorrSize-1][i] = curEnergy[i]-prevEnergy[i]; //Add this frame's differentiated energy to the last slot in the buffer.
		if (onsetFunction[autoCorrSize-1][i]<0) {
			onsetFunction[autoCorrSize-1][i]=0; //Rectify the onset function values
		}
	}
}	


void UpdateDoubleClickVars(void) {
	//Update the variables that prevent two beats from occuring too close to each other
	doubleBeatDetect_Beats--;
	doubleBeatDetect_Pattern--;
	for (i=0;i<5;i++) {
		doubleBeatDetect_Tempo[i]--;
	}
	framesUntilWeightingDone--; //Also update this variable, which determines if the current frame is close enough to the last frame with 'inconsistent' beats that the subbands and tempo still need to be reweighed
}

void CalculateTempo(void) {
	//This function determines the tempo of the audio
	GetOnsetFunctionMagSpectrum(); //First part of autocorrelation; go from time to frequency domain
	FinishAutocorrelation(); //Second part of autocorrelation; go from frequency to lag domain. Also calculate the subband ratios if needed
}

void GetOnsetFunctionMagSpectrum(void) {
	//Obtains the squared magnitude spectrum for each onset function.
	for (i=0;i<numSubbands;i++) {
		for (j=0;j<autoCorrSize;j++) {
			oneSubbandDiffEnergy[j] = onsetFunction[j][i];
			oneSubbandDiffEnergy[j+autoCorrSize] = 0;
			oneSubbandDiffEnergy[j+autoCorrSize*2] = 0;
			oneSubbandDiffEnergy[j+autoCorrSize*3] = 0; //Set up the autocorrelation FFT. FFT with twice as many points as are in the function, so zero-pad to that length, then add 0's for the complex values as well
		}
		FFT(oneSubbandDiffEnergy,autoCorrSize*2,autoTwid,autoCorrOutOneSub,1); //Get the spectrum of each onset function with an FFT.
		magnitude(autoCorrOutOneSub,autoCorrSize*2,autoCorrOutOneSubMag,0); //Get the squared magnitude of the spectrum. 
		for (j=0;j<autoCorrSize*2;j++) {
			autoMag[j][i] = autoCorrOutOneSubMag[j]; //Save the magnitudes in a buffer.
		}		
	}
}

void FinishAutocorrelation(void) {
	//Uses IFFTs to get the autocorrelations of the onset functions. The IFFTs may be taken for each function individually or for their sum, depending on whether the individual autocorrelations are needed to determine which subband is most periodic.
	
	if (reweightTempoAndSubband==1) { //If the subbands are being reweighted, the autocorrelations for each subband are taken individually. Then the ratios are calculated and the autorcorrelations are summed.
		for (i=0;i<autoCorrSize*2-1;i++) {
			tempobank[i]=0; //Initialize the tempobank
		}
		for (j=0;j<numSubbands;j++) { 
			for (i=0;i<autoCorrSize*2;i++) {
				toIFFT[i] = autoMag[i][j];
				toIFFT[i+autoCorrSize*2]=0; //For each subband, set up the IFFT by adding 0's for the complex values in the magnitude spectrum
			}
			FFT(toIFFT,autoCorrSize*2,invAutoTwid,fftOut,-1); //Take the IFFT
			
			//Need to reorganize the IFFT (max value at 0) so that the max value is at 0 lag (the center of the autocorrelation)
			for (i=0;i<(autoCorrSize-2);i++) {
				fftCorr[i] = fftOut[i+autoCorrSize+2];
				tempobank[i] = tempobank[i]+fftCorr[i];
			}
			for (i=autoCorrSize-2;i<autoCorrSize*2-1;i++) {
				fftCorr[i] = fftOut[i-(autoCorrSize-2)];
				tempobank[i]=tempobank[i]+fftCorr[i]; //Sort the IFFT to be the autocorrelation, and sum the autocorrelations in the tempobank variable
			}
			CalculateSubbandRatios(); //Calculate the autocorrelation ratios of each subband
		//	printf("%f  ",autoCorrRatio[j]);
		}
	//	printf("%i  \n", fNum);
	}
	else {
		for (i=0;i<autoCorrSize*2;i++) {
			toIFFT[i]=0;
			for (j=0;j<numSubbands;j++) {
				toIFFT[i]=toIFFT[i]+autoMag[i][j]; //Sum magnitudes before taking the IFFT
			}
			toIFFT[i+autoCorrSize*2] = 0; //Set up IFFT (add 0's for the complex values)
		}
		FFT(toIFFT,autoCorrSize*2,invAutoTwid,fftOut,-1); //Take the IFFT
		
		for (i=0;i<autoCorrSize-2;i++) {
			tempobank[i] = fftOut[i+autoCorrSize+2];
		}
		for (i=autoCorrSize-2;i<autoCorrSize*2-1;i++) {
			tempobank[i] = fftOut[i-(autoCorrSize-2)]; //Rearrange IFFT as before
		}
	}
}

void CalculateSubbandRatios(void) {
	for (i=266;i<266+245;i++) {
		TempoHold[i-266]=fftCorr[i]; //Get the values in the range of possible tempo values. 
	}
	max(TempoHold,245,maxTempos); //Maximum autocorrelation value in the tempo range
	max(fftCorr,autoCorrSize*2-1,maxAll); //Total energy (maximum value in the full autocorrelation)
	autoCorrRatio[j] = maxTempos[0]/maxAll[0]; // Determine the ratios
}

void DetermineBeatPhase(void) {
	//Now that the tempo is known and the onset function exists, determine if this frame contains a beat
	
	UpdateFrameEnergyHistory(); //Put energy values into the history
	DetermineOptimalPDF(); //Use the pattern of beats predicted for each PDF to see which one is the best one (producing the most consistent series of beats) to weight the tempobank values
	EstimateIfBeatInFrame(chosenDelay, beatPredicted); //Using just the frame energy and tempo values, estimate if a beat is in this frame
	//printf("%f  %f  %i  %i\n",FourBeatValue[0],FourBeatValue[1], chosenDelay, fNum);
	UpdateBeatPattern(); //Update a history of the beats
	DecideIfBeatInFrame(); //Using the history, the spacings of the previous beats, the tempo, and the energy values, determine if a beat is in this frame
	DetermineNewBeatSpacing(); //Update the beat spacing
	FindTempoForEachPDF(); //Using each possible PDF, determine if a beat is in this frame
	IncrementFrameIndexes(); //Increment the frame indexes
}

void UpdateFrameEnergyHistory(void) {
	//Add the energy difference between this and the previous frame to a history
	prevFrameEnergy = frameEnergy;
	frameEnergy=0;
	for (i=0;i<numSubbands;i++) {
		frameEnergy = frameEnergy+SummedSub[i]; //Calculate the total energy in the frame
	}
	enDiff = frameEnergy-prevFrameEnergy; //Find the difference in energy in this and the previous frames
	if (enDiff<0) {
		enDiff=0; //Rectify difference
	}
	for (i=0;i<autoCorrSize-1;i++) {
		diffFrameEnergy[i] = diffFrameEnergy[i+1]; //Cycle the difference buffer
	}
	diffFrameEnergy[autoCorrSize-1] = enDiff; //Add the latest difference to the buffer
	
}

void EstimateIfBeatInFrame(int theDelay, int thePredictor[]) {
	//Using the frame energy, and a specified delay (based on the tempo), estimate if a beat is in this frame. If so, set a flag in thePredictor 
	FourBeatValue[0]=0;
	FourBeatValue[1]=0;
	for (n=0;n<4;n++) {
		delayInd[n]=autoCorrSize-1-round(theDelay*n);  //Assuming a beat is in this frame and the delay is correct, these indices will correspond to the last four beat locations (counting this one)
		FourBeatValue[0] = FourBeatValue[0]+diffFrameEnergy[delayInd[n]]; //Find the total energy difference values for the last four beat locations (assuming a beat is in this frame)
		FourBeatValue[1] = FourBeatValue[1]+diffFrameEnergy[delayInd[n]+shiftVec[n]]; //Since the beats will likely not be separated by exactly an integer number of frames, also find the energy differences for frames shifted one frame up.
	}
	
	for (n=0;n<1000;n++) {
		OtherBeatValue[n]=0;
	}
	for (n=1;n<=theDelay-2;n++) {
		for (o=0;o<4;o++) {
			delayInd[o]=autoCorrSize-1-round(theDelay*o)-n;  //Assuming a beat is in the preceding frame i and the delay is correct, these indices will correspond to the last four beat locations (counting this one)
			OtherBeatValue[n]=OtherBeatValue[n]+diffFrameEnergy[delayInd[o]]; //Now find the differences for all other sets of four frames spaced one delay apart
		}
	}
	max(FourBeatValue,2,maxFour);
	max(OtherBeatValue,1000,maxOther);
	if (maxFour[0]>=energyThreshold*maxOther[0]) { //If the FourBeatValue is some threshold of the max of all the other energy values in the frame, we estimate a beat
		thePredictor[0]=1;
	}
	else
	{
		thePredictor[0]=0;
	}
}

void DetermineOptimalPDF(void) {
	
	for (i=0;i<numPDFs;i++) {
		allDelays[i] = (float) bPMDelayConversion/tempoPDFCenters[i]; //Calculate the delay for the tempo determined by each PDF
	//	printf("%f  ",allDelays[i]);
		tempoFlagging[i]=0;
	//	printf("%f  ",allDelays[i]);
	}
//	printf("%i\n",fNum);
	//printf("%i \n",fNum);
	if (reweightTempoAndSubband==1) {
		for (j=0;j<numPDFs;j++) {
			EstimateIfBeatInFrame(round(allDelays[j]),beatPredicted_Tempo); //For each tempo, estimate if a beat is in this frame
			HelpFulVector[j] = beatPredicted_Tempo[0];
			//HelpFulVector[2*j+1] = maxOther[0];
			//	printf("%f  %f  ",FourBeatValue[0],FourBeatValue[1]);
			if (beatPredicted_Tempo[0]==1 && doubleBeatDetect_Tempo[j]<0) {
				doubleBeatDetect_Tempo[j]=minBeatSpacing;
				for (i=0;i<4;i++) {
					beatHistory_Tempo[j][i] = beatHistory_Tempo[j][i+1]; 
				}
				beatHistory_Tempo[j][4] = fNum;  //If one is, add this frame to the appropriate beat history vector
			}
			for (i=0;i<4;i++) {
				diffBeats[i] = beatHistory_Tempo[j][i+1]-beatHistory_Tempo[j][i]; //Take the difference of the beat history vectors
			}
			if (fabs(diffBeats[0]-diffBeats[1])<2&&fabs(diffBeats[0]-diffBeats[2])<2&&fabs(diffBeats[0]-diffBeats[3])<2&&fabs(diffBeats[1]-diffBeats[2])<2&&fabs(diffBeats[1]-diffBeats[3])<2&&fabs(diffBeats[2]-diffBeats[3])<2) {
				tempoFlagging[j]=1; //If a history vector is consistent for five beats, flag it
			}
			else {
				tempoFlagging[j]=0;
			}
		}
		
		//printf("%i\n",fNum);
		if (tempoFlagging[2]==1) { //Depending on which vectors are consistent, choose a PDF. Bias towards the middle one (120 BPM mean).
			tempoFlag=3;
		}
		else if (tempoFlagging[1]==1) {
			tempoFlag=2;
		}
		else if (tempoFlagging[3]==1) {
			tempoFlag=4;
		}
		else if (tempoFlagging[4]==1) {
			tempoFlag=5;
		}
		else if (tempoFlagging[0]==1) {
			tempoFlag=1;
		}
		else
		{
			tempoFlag=3;
		}
	}
	chosenDelay = round(allDelays[(int) tempoFlag-1]); //Determine the delay based on the tempo predicted with the selected PDF
	//printf("%f\n",allDelays[(int) tempoFlag-1]);
}

void UpdateBeatPattern(void) {
	//If a beat was predicted in this frame, add to a history vector so that the pattern of beats can be updated
	if (beatPredicted[0]==1 && doubleBeatDetect_Pattern<0) {
		doubleBeatDetect_Pattern=minBeatSpacing;
		for (i=0;i<4;i++) {
			beatHistory_Pattern[i] = beatHistory_Pattern[i+1]; 
		}
		beatHistory_Pattern[4]=fNum; //If we estimate a new beat, add this value to the history
	}
}
void DetermineNewBeatSpacing(void) {
	//Now that the system has estimated if a beat is in this frame, update the beat spacing prediction
	if ((fNum-lastBeat)>44) {
		beatSpacing=0; //If there hasn't been a beat in over a second, the spacing is probably screwed up. Reset the beat spacing.
	}
	reweightTempoAndSubband=1;
	
	for (i=0;i<4;i++) {
		diffBeats[i] = beatHistory_Pattern[i+1]-beatHistory_Pattern[i]; //Find the distance between the last few sets of beats
		modeCount2[i]=0;
	}
	for (i=0;i<4;i++) {
		for (j=0;j<4;j++) {
			if (diffBeats[j]==diffBeats[i]) {
				modeCount2[i]=modeCount2[i]+1; //Determine the mode by incrementing a value in a vector every time one difference matches another
			}
		}
	}
	modeBeatSpacing = diffBeats[0];
	modeIndex = 0;
	//Determine the mode based on logic. If there is one mode, choose it. If multiple numbers occur with the same frequency, choose the larger value of them.
	for (i=1;i<4;i++) {
		if ((modeCount2[i]>modeCount2[modeIndex]) ||(modeBeatSpacing>diffBeats[i] && modeCount2[i]==modeCount2[modeIndex])) {
			modeBeatSpacing=diffBeats[i];
			modeIndex=i;
		}
	}
	if (fabs(diffBeats[0]-diffBeats[1])<2&&fabs(diffBeats[0]-diffBeats[2])<2&&fabs(diffBeats[0]-diffBeats[3])<2&&fabs(diffBeats[1]-diffBeats[2])<2&&fabs(diffBeats[1]-diffBeats[3])<2&&fabs(diffBeats[2]-diffBeats[3])<2) {
		if (modeBeatSpacing != (double) beatSpacing/2-.5 && modeBeatSpacing != (double) beatSpacing/2+.5 && modeBeatSpacing != (double) beatSpacing/2 && modeBeatSpacing != 2*beatSpacing) {
			beatSpacing = modeBeatSpacing; //But only change the spacing if the differences are consistent and if the spacing won't change to double- or half-spacing.
		}
	}	
}

void FindTempoForEachPDF(void) {
	//Weight the tempobank values by each PDF, then determine the tempo for each weighting
	if (reweightTempoAndSubband==1) {
		for (j=0;j<numPDFs;j++) {
			for (i=0;i<42;i++) {
				TempoHold2[i] = tempobank[(int) (511/2+.5+twoFiftyPoint+i)]*pDF[j][i]; //Weight the set of tempo values (between 50 and 250 BPM) by each PDF
				TempoHold3[i][j] = TempoHold2[i];
			}
			max(TempoHold2,42,tempoMax);
			tempoind = tempoMax[1]+twoFiftyPoint+1;
			TempoHold3[42][j] = tempoMax[0];
			TempoHold3[43][j] = tempoMax[1];
			TempoHold3[44][j] = tempoind;
			
			tempoPDFCenters[j] = round(bPMDelayConversion/tempoind); //Find the tempo as the maximum weighted value.
			TempoHold3[45][j] = tempoPDFCenters[j];
			//	printf("%f  ",tempoPDFCenters[j]);
		}
	}
	else { //If not reweighting, just use the one PDF we've already chosen
		j=tempoFlag-1;
		for (i=0;i<42;i++) {
			TempoHold2[i] = tempobank[(int) (511/2+.5+twoFiftyPoint+i)]*pDF[j][i]; //Weight the tempobank
		}
		max(TempoHold2,42,tempoMax);
		tempoind = tempoMax[1]+twoFiftyPoint+1;
		tempoPDFCenters[j] = round(bPMDelayConversion/tempoind); //Find the tempo
	//	printf("%f  ",tempoPDFCenters[j]);
	}
//	printf("%f\n",tempoPDFCenters[(int)tempoFlag-1]);
}

void IncrementFrameIndexes(void) {	
	//Increment the starting and ending sample values, and the frame number
	fStart=fStart+fHop;
	fEnd=fEnd+fHop;
	fNum++; 
}

void DecideIfBeatInFrame(void) {
	//Final determination if a beat is in this frame
	//A beat is marked in this frame if one of three conditions holds:
	//1. A beat is predicted based on energy values, this frame is more than the minimum distance from the previous frame, and there is no consistent beat spacing found yet
	//2. A beat is predicted based on energy values, this frame is more than the minimum distance from the previous frame, and this frame is within 2 frames of where a beat would be expected based on previous spacings
	//3. No beat is predicted based on energy values but the spacing indicates that there should have been a beat in the previous frame (this means that the tracker probably missed one)
	if ((beatPredicted[0]==1&&doubleBeatDetect_Beats<0&&beatSpacing==0)||(beatPredicted[0]==1&&doubleBeatDetect_Beats<0&&fabs(fNum-beatSpacing-lastBeat)<3&&beatSpacing>0)||(beatSpacing>0&&fNum-beatSpacing-lastBeat==1)) {
		if (beatPredicted[0]==1&&doubleBeatDetect_Beats<0&&beatSpacing==0)
		{
			Confidence = 0;
		}
		else if (beatPredicted[0]==1&&doubleBeatDetect_Beats<0&&fabs(fNum-beatSpacing-lastBeat)<3&&beatSpacing>0)
		{
			if (Confidence == 0)
			{
				Confidence = .5;
			}
			else
			{
				Confidence = Confidence+.2;
			}
			if (Confidence > 1) 
			{
				Confidence = 1;
			}
		}
		if 	(beatSpacing>0&&fNum-beatSpacing-lastBeat==1)
		{
			if (Confidence >= .3) 
			{
				Confidence = Confidence - .3;
			}
			else 
			{
				Confidence = 0;
			}
		}
				
			
		for (i=0;i<999;i++) {
			beatsDetected[i]=beatsDetected[i+1]; //Cycle beat location buffer
			beatCon[i] = beatCon[i+1];
		}
		beatsDetected[999] = fStart; //Add this frame to the beat vector
		beatCon[999] = Confidence;
		lastBeat=fNum;
		doubleBeatDetect_Beats=minBeatSpacing; //Update beat spacing variables
		beatPing=1; //Mark that a beat is here for display later
		
	}
}

