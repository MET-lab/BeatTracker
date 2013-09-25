/*
 *  BeatTrackerMath.c
 *  BeatTracker
 *
 *  Created by David Grunberg on 8/29/10.
 *  Copyright 2010 Drexel University. All rights reserved.
 *
 */

#include "BeatTrackerMath.h"
#include "math.h"
#include "stdlib.h"
#define PI (3.14159265)


//These are FFT functions

void polarToComplex(float mag, float phase, float* ans) {
    ans[0] = mag * cos(phase);
    ans[1] = mag * sin(phase);
}

void computeTwiddleFactors(float* twiddle, int fftLength, float sign) {
    int p;
    float temp[2];
	
    for (p = 0; p < fftLength / 2; p++) {
        polarToComplex(1, sign * 2 * PI * (p) / fftLength, temp);
        twiddle[(2 * p)] = temp[0];
        twiddle[(2 * p) + 1] = temp[1];
    }
}

void FFTHelper(float* x, int fftLength, float* X, float* scratch,
			   float* twiddle, int imagStart) {
    int k2, m2, n2;
    int skip;
    /* int imagStart = fftLength; */
    int evenItr = fftLength & 0x55555555;
	
    float* E, *D;
    float* Xp, *Xp2, *XStart;
    float temp[2], temp2[2];
	
    /* Special Case */
    if (fftLength == 1) {
        X[0] = x[0];
        X[1] = x[imagStart];
        return;
    }
	
    E = x;
	
    for (n2 = 1; n2 < fftLength; n2 *= 2) {
        XStart = evenItr ? scratch : X;
        skip = (fftLength) / (2 * n2);
        Xp = XStart;
        Xp2 = XStart + (fftLength / 2);
        for (k2 = 0; k2 != n2; k2++) {
			
            temp[0] = twiddle[2 * (k2 * skip)];
            temp[1] = twiddle[2 * (k2 * skip) + 1];
			
            for (m2 = 0; m2 != skip; ++m2) {
                D = E + (skip);
				
                temp2[0] = (*D * temp[0]) - (*(D + imagStart) * temp[1]);
                temp2[1] = (*D * temp[1]) + (*(D + imagStart) * temp[0]);
				
                *Xp = *E + temp2[0];
                *(Xp + imagStart) = *(E + imagStart) + temp2[1];
				
                *Xp2 = *E - temp2[0];
                *(Xp2 + imagStart) = *(E + imagStart) - temp2[1];
				
                Xp = Xp + 1;
                Xp2 = Xp2 + 1;
                E = E + 1;
            }
            E = E + skip;
        }
        E = XStart;
        evenItr = !evenItr;
    }
}


void FFT(float* x, int fftLength, float* twiddles, float* output, int sign) {
	int p = 0;
    float* scratch = (float*) malloc(sizeof (float) * (2 * fftLength));
    FFTHelper(x, fftLength, output, scratch, twiddles, fftLength);
	

    if (sign == -1) {
        for (p = 0; p < fftLength; p++) {
            output[p] /= fftLength;
            output[p + fftLength] /= fftLength;
        }
    }
	
    free(scratch);
}


/**********************************************************
 Function: linspace
 Generates a vector linearly moving between two points
 
 Parameters
 startPoint: first point of vector
 endPoint: last point of vector
 number: number of points in vector. Must be >=2.
 ans: output vector
 **********************************************************/
void linspace(float startPoint[], float lastPoint[], int len, float* ans) {
	float increment;
	int p;
	ans[0] = startPoint[0];
	ans[len-1] = lastPoint[0];
	increment = (lastPoint[0]-startPoint[0])/(len-1);
	//printf("%f %f %f\n", ans[0],ans[len-1],increment);
	for (p=1;p<len-1;p++) {
		ans[p] = ans[p-1]+increment;
	}
}

/**********************************************************
 Function: tripulse
 Generates a triangular pulse
 
 Parameters:
 length: length of output vector
 pulsePoint: x-point at which the triangle peaks
 magnitude: y-point at which the triangle peaks (peak is at (pulsePoint,magnitude).
 ans: output vector.
 **********************************************************/
void tripulse(int length, int pulsePoint, float magnitude[], float* ans) {
	int len1;
	int len2;
	int j2;
	int i2;
	float temp[pulsePoint];
	float temp2[length-pulsePoint];
	float zeroPoint[1];
	
	len1 = pulsePoint;
	len2 = length-pulsePoint;
	zeroPoint[0] = 0.0;
	
	linspace(zeroPoint,magnitude,len1,temp);
	linspace(magnitude,zeroPoint,len2,temp2);
	for (i2=0;i2<pulsePoint;i2++) {
		ans[i2] = temp[i2];
	}
	j2=0;
	for (i2=pulsePoint;i2<length;i2++) {
		ans[i2] = temp2[j2];
		j2=j2+1;
	}
}

/**********************************************************
 Function: MakeVector
 Initializes a vector
 
 Parameters:
 length: length of output vector
 MyValue: value of each point in the vector
 ans: output vector
 **********************************************************/
void MakeVector(int length, int MyValue, float* ans) {
	int i2;
	for (i2=0; i2<length; i2++){
		ans[i2] = MyValue;
	}
}

/**********************************************************
 Function: sum
 Sums a vector
 
 Parameters:
 vecIn: input vector
 y: output sum
 len: length of input vector
 **********************************************************/
void sum(float vecIn[], float* y, int len) {
	int i2;	
	for (i2=0;i2<len;i2++){
		y[0] = y[0]+vecIn[i2];
	}
}

/**********************************************************
 Function: magnitude
 Calculates the magnitude of a vector that has complex values
 
 Parameters:
 vecIn: input vector. First half of vector should be parts portions of each value, second half should be complex parts
 len: length of output vector
 y: output vector
 squared: flag indicating whether the magnitude should be squared or not
 **********************************************************/
void magnitude(float vecIn[], int len, float * y, int squared) {
	int i2;
	double temp;
	double temp1;
	double temp2;
	for (i2=0;i2<len;i2++) {
		temp = vecIn[i2]*vecIn[i2];
		temp1 = vecIn[i2+len]*vecIn[i2+len];
		temp2 = temp+temp1;
		if (squared==1)
		{
			y[i2] = sqrt(temp2);
		}
		else
		{
			y[i2] = temp2;
		}
	}
}

/**********************************************************
 Function: max
 Finds the maximum value of a vector, as well as that value's index
 
 Parameters:
 vecIn: input vector. 
 len: length of input vector
 ans: maximum value and location
 **********************************************************/
void max(float vecIn[], int len, float* ans) {
	int p;
	ans[0] = vecIn[0];
	ans[1] = 0;
	for (p=1; p<len; p++) {		
		if (ans[0]<vecIn[p]) {
			ans[0] = vecIn[p];
			ans[1] = p;
		}
	}
}
