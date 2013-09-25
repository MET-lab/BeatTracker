#ifndef BEATTRACKER_H
#define BEATTRACKER_H
/*
 *  BeatTracker.h
 *  BeatTracker
 *
 *  Created by David Grunberg on 8/29/10.
 *  Copyright 2010 Drexel University. All rights reserved.
 *
 */

void BeatTrackFrame(float* in, int beatHere);
void CreateOnsetFunction(float* in);
void GetSubbandMagSpectrum(float* in);
void ReWeightSubbands(void);
void GetSubbandEnergies(void);
void UpdateDoubleClickVars(void);
void CalculateTempo(void);
void GetOnsetFunctionMagSpectrum(void);
void FinishAutocorrelation(void);
void CalculateSubbandRatios(void);
void DetermineBeatPhase(void);
void UpdateFrameEnergyHistory(void);
void DetermineOptimalPDF(void);
void EstimateIfBeatInFrame(int theDelay, int thePredictor[]);
void UpdateBeatPattern(void);
void DecideIfBeatInFrame(void);
void DetermineNewBeatSpacing(void) ;
void FindTempoForEachPDF(void);
void IncrementFrameIndexes(void);


#endif