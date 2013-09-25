#ifndef BEATTRACKERMATH_H
#define BEATTRACKERMATH_H
/*
 *  BeatTrackerMath.h
 *  BeatTracker
 *
 *  Created by David Grunberg on 8/29/10.
 *  Copyright 2010 Drexel University. All rights reserved.
 *
 */


//General math functions
void linspace(float startPoint[], float lastPoint[], int len, float* ans);
void tripulse(int length, int pulsePoint, float magnitude[], float* ans);
void MakeVector(int length, int MyValue, float* ans);
void polarToComplex(float mag, float phase, float* ans);
void max(float vecIn[], int len, float* ans);
void min(float vecIn[], int len, float* ans);
void sum(float vecIn[], float* y, int len) ;
void magnitude(float vecIn[], int len, float * y, int squared);
void difference(float vecIn[], int len, float* y);


//FFT functions

void computeTwiddleFactors(float* twiddle, int fftLength, float sign);
void FFTHelper(float* x, int fftLength, float* X, float* scratch, float* twiddle, int imagStart);
void FFT(float* x, int fftLength, float* twiddles, float* output, int sign);

#endif