/*
 * libmfcc.c - Code implementation for libMFCC
 * Copyright (c) 2010 Jeremy Sawruk
 *
 * This code is released under the MIT License. 
 * For conditions of distribution and use, see the license in LICENSE
 */

#include <math.h>
#include "libmfcc.h"
#include "Macro_Definitions.h"
#include "arm_math.h"
/*
 * Computes the specified (mth) MFCC
 *
 * spectralData - array of doubles containing the results of FFT computation. This data is already assumed to be purely real
 * samplingRate - the rate that the original time-series data was sampled at (i.e 44100)
 * NumFilters - the number of filters to use in the computation. Recommended value = 48
 * binSize - the size of the spectralData array, usually a power of 2
 * m - The mth MFCC coefficient to compute
 *
 */

double Magn[25] = {0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.015000,0.006705,0.011800,0.011016,0.010284,0.009601,0.008963,0.008368,0.007812,0.007293,0.006808};
double centFreq[26]={66.666664,133.333328,200.000000,266.666656,333.333344,400.000000,466.666656,533.333313,600.000000,666.666687,733.333313,800.000000,866.666687,933.333313,1149.794200,1231.625398,1319.280547,1413.174140,1513.750167,1621.484221,1736.885739,1860.500418,1992.912791,2134.748992,2286.679718};
char serialt[16];

double GetCoefficient(double* spectralData, unsigned int samplingRate, unsigned int NumFilters, unsigned int binSize, unsigned int m)
{
	double result = 0.0f;
	double outerSum = 0.0f;
	double innerSum = 0.0f;
	unsigned int k, l;

	// 0 <= m < L
	if(m >= NumFilters)
	{
		// This represents an error condition - the specified coefficient is greater than or equal to the number of filters. The behavior in this case is undefined.
		return 0.0f;
	}

	result = NormalizationFactor(NumFilters, m);



	for(l = 1; l <= NumFilters; l++)
	{
		// Compute inner sum
		innerSum = 0.0f;
		for(k = 0; k < binSize - 1; k++)
		{
			innerSum += fabs(spectralData[k] *GetFilterParameter(samplingRate, binSize, k, l));  //MelFilterbanks[NumFilters][binSize]
		}

		if(innerSum > 0.0f)
		{
			innerSum = log(innerSum); // The log of 0 is undefined, so don't use it
		}

		innerSum = innerSum * cos(((m * PI) / NumFilters) * (l - 0.5f));

		outerSum += innerSum;
	}

	result *= outerSum;
	return result;
}

/* 
 * Computes the Normalization Factor (Equation 6)
 * Used for internal computation only - not to be called directly
 */
double NormalizationFactor(int NumFilters, int m)
{
	double normalizationFactor = 0.0f;

	if(m == 0)
	{
//		normalizationFactor = 0.204124;
		normalizationFactor = sqrt(1.0f / NumFilters);
	}
	else 
	{
//		normalizationFactor = 0.288675;
		normalizationFactor = sqrt(2.0f / NumFilters);
	}
//	sprintf(serialt,"%i\t%lf\r\n",m,normalizationFactor);serialPrint(serialt);
	return normalizationFactor;
}

/* 
 * Compute the filter parameter for the specified frequency and filter bands (Eq. 2)
 * Used for internal computation only - not the be called directly
 */
double GetFilterParameter(unsigned int samplingRate, unsigned int binSize, unsigned int frequencyBand, unsigned int filterBand)
{
	double filterParameter = 0.0f;

	double boundary = (frequencyBand * samplingRate) / binSize;		// k * Fs / N
	double prevCenterFrequency = GetCenterFrequency(filterBand - 1);		// fc(l - 1) etc.
	double thisCenterFrequency = GetCenterFrequency(filterBand);
	double nextCenterFrequency = GetCenterFrequency(filterBand + 1);

	if(boundary >= 0 && boundary < prevCenterFrequency)
	{
		filterParameter = 0.0f;
	}
	else if(boundary >= prevCenterFrequency && boundary < thisCenterFrequency)
	{
		filterParameter = (boundary - prevCenterFrequency) / (thisCenterFrequency - prevCenterFrequency);
		filterParameter *= GetMagnitudeFactor(filterBand);
	}
	else if(boundary >= thisCenterFrequency && boundary < nextCenterFrequency)
	{
		filterParameter = (boundary - nextCenterFrequency) / (thisCenterFrequency - nextCenterFrequency);
		filterParameter *= GetMagnitudeFactor(filterBand);
	}
	else if(boundary >= nextCenterFrequency && boundary < samplingRate)
	{
		filterParameter = 0.0f;
	}

	return filterParameter;
}

/* 
 * Compute the band-dependent magnitude factor for the given filter band (Eq. 3)
 * Used for internal computation only - not the be called directly
 */

double GetMagnitudeFactor(unsigned int filterBand)
{
	double magnitudeFactor = 0.0f;

//	magnitudeFactor = Magn[filterBand];
//	sprintf(serialt,"%i\t%lf\r\n",filterBand,magnitudeFactor);serialPrint(serialt);
	if(filterBand >= 1 && filterBand <= 14)
	{
		magnitudeFactor = 0.015;
	}
	else if(filterBand >= 14 && filterBand <= 48)
	{
		magnitudeFactor = 2.0f / (GetCenterFrequency(filterBand + 1) - GetCenterFrequency(filterBand -1));
	}


	return magnitudeFactor;
}

/*
 * Compute the center frequency (fc) of the specified filter band (l) (Eq. 4)
 * This where the mel-frequency scaling occurs. Filters are specified so that their
 * center frequencies are equally spaced on the mel scale
 * Used for internal computation only - not the be called directly
 */
double GetCenterFrequency(unsigned int filterBand)
{
	double centerFrequency = 0.0f;
	double exponent;

	if(filterBand == 0)
	{
		centerFrequency = 0;
	}
//	else{
//	centerFrequency = centFreq[filterBand];
//	}
	else if(filterBand >= 1 && filterBand <= 14)
	{
		centerFrequency = (200.0f * filterBand) / 3.0f;
	}
	else
	{
		exponent = filterBand - 14.0f;
		centerFrequency = pow(1.0711703, exponent);
		centerFrequency *= 1073.4;
	}

	return centerFrequency;
}
