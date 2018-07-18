/*
 * fft.c
 *
 *  Created on: 2 Ïêô 2017
 *      Author: Thanos
 */

#include "fft.h"



void arm_max_f32(
	  float32_t * pSrc,
	  uint32_t blockSize,
	  float32_t * pResult,
	  uint32_t * pIndex)
	{
	#ifndef ARM_MATH_CM0_FAMILY

	  /* Run the below code for Cortex-M4 and Cortex-M3 */
	  float32_t maxVal1, maxVal2, out;               /* Temporary variables to store the output value. */
	  uint32_t blkCnt, outIndex, count;              /* loop counter */

	  /* Initialise the count value. */
	  count = 0u;
	  /* Initialise the index value to zero. */
	  outIndex = 0u;
	  /* Load first input value that act as reference value for comparision */
	  out = *pSrc++;

	  /* Loop unrolling */
	  blkCnt = (blockSize - 1u) >> 2u;

	  /* Run the below code for Cortex-M4 and Cortex-M3 */
	  while(blkCnt > 0u)
	  {
	    /* Initialize maxVal to the next consecutive values one by one */
	    maxVal1 = *pSrc++;

	    maxVal2 = *pSrc++;

	    /* compare for the maximum value */
	    if(out < maxVal1)
	    {
	      /* Update the maximum value and its index */
	      out = maxVal1;
	      outIndex = count + 1u;
	    }

	    maxVal1 = *pSrc++;

	    /* compare for the maximum value */
	    if(out < maxVal2)
	    {
	      /* Update the maximum value and its index */
	      out = maxVal2;
	      outIndex = count + 2u;
	    }

	    maxVal2 = *pSrc++;

	    /* compare for the maximum value */
	    if(out < maxVal1)
	    {
	      /* Update the maximum value and its index */
	      out = maxVal1;
	      outIndex = count + 3u;
	    }

	    /* compare for the maximum value */
	    if(out < maxVal2)
	    {
	      /* Update the maximum value and its index */
	      out = maxVal2;
	      outIndex = count + 4u;
	    }

	    count += 4u;

	    /* Decrement the loop counter */
	    blkCnt--;
	  }

	  /* if (blockSize - 1u) is not multiple of 4 */
	  blkCnt = (blockSize - 1u) % 4u;

	#else

	  /* Run the below code for Cortex-M0 */
	  float32_t maxVal1, out;                        /* Temporary variables to store the output value. */
	  uint32_t blkCnt, outIndex;                     /* loop counter */

	  /* Initialise the index value to zero. */
	  outIndex = 0u;
	  /* Load first input value that act as reference value for comparision */
	  out = *pSrc++;

	  blkCnt = (blockSize - 1u);

	#endif /* #ifndef ARM_MATH_CM0_FAMILY */

	  while(blkCnt > 0u)
	  {
	    /* Initialize maxVal to the next consecutive values one by one */
	    maxVal1 = *pSrc++;

	    /* compare for the maximum value */
	    if(out < maxVal1)
	    {
	      /* Update the maximum value and it's index */
	      out = maxVal1;
	      outIndex = blockSize - blkCnt;
	    }


	    /* Decrement the loop counter */
	    blkCnt--;

	  }

	  /* Store the maximum value and it's index into destination pointers */
	  *pResult = out;
	  *pIndex = outIndex;
	}
	void arm_cmplx_mag_f32(
	  float32_t * pSrc,
	  float32_t * pDst,
	  uint32_t numSamples)
	{
	  float32_t realIn, imagIn;                      /* Temporary variables to hold input values */

	#ifndef ARM_MATH_CM0_FAMILY

	  /* Run the below code for Cortex-M4 and Cortex-M3 */
	  uint32_t blkCnt;                               /* loop counter */

	  /*loop Unrolling */
	  blkCnt = numSamples >> 2u;

	  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	   ** a second loop below computes the remaining 1 to 3 samples. */
	  while(blkCnt > 0u)
	  {

	    /* C[0] = sqrt(A[0] * A[0] + A[1] * A[1]) */
	    realIn = *pSrc++;
	    imagIn = *pSrc++;
	    /* store the result in the destination buffer. */
	    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

	    realIn = *pSrc++;
	    imagIn = *pSrc++;
	    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

	    realIn = *pSrc++;
	    imagIn = *pSrc++;
	    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

	    realIn = *pSrc++;
	    imagIn = *pSrc++;
	    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);


	    /* Decrement the loop counter */
	    blkCnt--;
	  }

	  /* If the numSamples is not a multiple of 4, compute any remaining output samples here.
	   ** No loop unrolling is used. */
	  blkCnt = numSamples % 0x4u;

	  while(blkCnt > 0u)
	  {
	    /* C[0] = sqrt(A[0] * A[0] + A[1] * A[1]) */
	    realIn = *pSrc++;
	    imagIn = *pSrc++;
	    /* store the result in the destination buffer. */
	    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

	    /* Decrement the loop counter */
	    blkCnt--;
	  }

	#else

	  /* Run the below code for Cortex-M0 */

	  while(numSamples > 0u)
	  {
	    /* out = sqrt((real * real) + (imag * imag)) */
	    realIn = *pSrc++;
	    imagIn = *pSrc++;
	    /* store the result in the destination buffer. */
	    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

	    /* Decrement the loop counter */
	    numSamples--;
	  }

	#endif /* #ifndef ARM_MATH_CM0_FAMILY */

	}
	void arm_bitreversal_f32(
	float32_t * pSrc,
	uint16_t fftSize,
	uint16_t bitRevFactor,
	uint16_t * pBitRevTab)
	{
	   uint16_t fftLenBy2, fftLenBy2p1;
	   uint16_t i, j;
	   float32_t in;

	   /*  Initializations */
	   j = 0u;
	   fftLenBy2 = fftSize >> 1u;
	   fftLenBy2p1 = (fftSize >> 1u) + 1u;

	   /* Bit Reversal Implementation */
	   for (i = 0u; i <= (fftLenBy2 - 2u); i += 2u)
	   {
	      if(i < j)
	      {
	         /*  pSrc[i] <-> pSrc[j]; */
	         in = pSrc[2u * i];
	         pSrc[2u * i] = pSrc[2u * j];
	         pSrc[2u * j] = in;

	         /*  pSrc[i+1u] <-> pSrc[j+1u] */
	         in = pSrc[(2u * i) + 1u];
	         pSrc[(2u * i) + 1u] = pSrc[(2u * j) + 1u];
	         pSrc[(2u * j) + 1u] = in;

	         /*  pSrc[i+fftLenBy2p1] <-> pSrc[j+fftLenBy2p1] */
	         in = pSrc[2u * (i + fftLenBy2p1)];
	         pSrc[2u * (i + fftLenBy2p1)] = pSrc[2u * (j + fftLenBy2p1)];
	         pSrc[2u * (j + fftLenBy2p1)] = in;

	         /*  pSrc[i+fftLenBy2p1+1u] <-> pSrc[j+fftLenBy2p1+1u] */
	         in = pSrc[(2u * (i + fftLenBy2p1)) + 1u];
	         pSrc[(2u * (i + fftLenBy2p1)) + 1u] =
	         pSrc[(2u * (j + fftLenBy2p1)) + 1u];
	         pSrc[(2u * (j + fftLenBy2p1)) + 1u] = in;

	      }

	      /*  pSrc[i+1u] <-> pSrc[j+1u] */
	      in = pSrc[2u * (i + 1u)];
	      pSrc[2u * (i + 1u)] = pSrc[2u * (j + fftLenBy2)];
	      pSrc[2u * (j + fftLenBy2)] = in;

	      /*  pSrc[i+2u] <-> pSrc[j+2u] */
	      in = pSrc[(2u * (i + 1u)) + 1u];
	      pSrc[(2u * (i + 1u)) + 1u] = pSrc[(2u * (j + fftLenBy2)) + 1u];
	      pSrc[(2u * (j + fftLenBy2)) + 1u] = in;

	      /*  Reading the index for the bit reversal */
	      j = *pBitRevTab;

	      /*  Updating the bit reversal index depending on the fft length  */
	      pBitRevTab += bitRevFactor;
	   }
	}
	void arm_radix8_butterfly_f32(
	float32_t * pSrc,
	uint16_t fftLen,
	const float32_t * pCoef,
	uint16_t twidCoefModifier)
	{
	   uint32_t ia1, ia2, ia3, ia4, ia5, ia6, ia7;
	   uint32_t i1, i2, i3, i4, i5, i6, i7, i8;
	   uint32_t id;
	   uint32_t n1, n2, j;

	   float32_t r1, r2, r3, r4, r5, r6, r7, r8;
	   float32_t t1, t2;
	   float32_t s1, s2, s3, s4, s5, s6, s7, s8;
	   float32_t p1, p2, p3, p4;
	   float32_t co2, co3, co4, co5, co6, co7, co8;
	   float32_t si2, si3, si4, si5, si6, si7, si8;
	   const float32_t C81 = 0.70710678118f;

	   n2 = fftLen;

	   do
	   {
	      n1 = n2;
	      n2 = n2 >> 3;
	      i1 = 0;

	      do
	      {
	         i2 = i1 + n2;
	         i3 = i2 + n2;
	         i4 = i3 + n2;
	         i5 = i4 + n2;
	         i6 = i5 + n2;
	         i7 = i6 + n2;
	         i8 = i7 + n2;
	         r1 = pSrc[2 * i1] + pSrc[2 * i5];
	         r5 = pSrc[2 * i1] - pSrc[2 * i5];
	         r2 = pSrc[2 * i2] + pSrc[2 * i6];
	         r6 = pSrc[2 * i2] - pSrc[2 * i6];
	         r3 = pSrc[2 * i3] + pSrc[2 * i7];
	         r7 = pSrc[2 * i3] - pSrc[2 * i7];
	         r4 = pSrc[2 * i4] + pSrc[2 * i8];
	         r8 = pSrc[2 * i4] - pSrc[2 * i8];
	         t1 = r1 - r3;
	         r1 = r1 + r3;
	         r3 = r2 - r4;
	         r2 = r2 + r4;
	         pSrc[2 * i1] = r1 + r2;
	         pSrc[2 * i5] = r1 - r2;
	         r1 = pSrc[2 * i1 + 1] + pSrc[2 * i5 + 1];
	         s5 = pSrc[2 * i1 + 1] - pSrc[2 * i5 + 1];
	         r2 = pSrc[2 * i2 + 1] + pSrc[2 * i6 + 1];
	         s6 = pSrc[2 * i2 + 1] - pSrc[2 * i6 + 1];
	         s3 = pSrc[2 * i3 + 1] + pSrc[2 * i7 + 1];
	         s7 = pSrc[2 * i3 + 1] - pSrc[2 * i7 + 1];
	         r4 = pSrc[2 * i4 + 1] + pSrc[2 * i8 + 1];
	         s8 = pSrc[2 * i4 + 1] - pSrc[2 * i8 + 1];
	         t2 = r1 - s3;
	         r1 = r1 + s3;
	         s3 = r2 - r4;
	         r2 = r2 + r4;
	         pSrc[2 * i1 + 1] = r1 + r2;
	         pSrc[2 * i5 + 1] = r1 - r2;
	         pSrc[2 * i3]     = t1 + s3;
	         pSrc[2 * i7]     = t1 - s3;
	         pSrc[2 * i3 + 1] = t2 - r3;
	         pSrc[2 * i7 + 1] = t2 + r3;
	         r1 = (r6 - r8) * C81;
	         r6 = (r6 + r8) * C81;
	         r2 = (s6 - s8) * C81;
	         s6 = (s6 + s8) * C81;
	         t1 = r5 - r1;
	         r5 = r5 + r1;
	         r8 = r7 - r6;
	         r7 = r7 + r6;
	         t2 = s5 - r2;
	         s5 = s5 + r2;
	         s8 = s7 - s6;
	         s7 = s7 + s6;
	         pSrc[2 * i2]     = r5 + s7;
	         pSrc[2 * i8]     = r5 - s7;
	         pSrc[2 * i6]     = t1 + s8;
	         pSrc[2 * i4]     = t1 - s8;
	         pSrc[2 * i2 + 1] = s5 - r7;
	         pSrc[2 * i8 + 1] = s5 + r7;
	         pSrc[2 * i6 + 1] = t2 - r8;
	         pSrc[2 * i4 + 1] = t2 + r8;

	         i1 += n1;
	      } while(i1 < fftLen);

	      if(n2 < 8)
	         break;

	      ia1 = 0;
	      j = 1;

	      do
	      {
	         /*  index calculation for the coefficients */
	         id  = ia1 + twidCoefModifier;
	         ia1 = id;
	         ia2 = ia1 + id;
	         ia3 = ia2 + id;
	         ia4 = ia3 + id;
	         ia5 = ia4 + id;
	         ia6 = ia5 + id;
	         ia7 = ia6 + id;

	         co2 = pCoef[2 * ia1];
	         co3 = pCoef[2 * ia2];
	         co4 = pCoef[2 * ia3];
	         co5 = pCoef[2 * ia4];
	         co6 = pCoef[2 * ia5];
	         co7 = pCoef[2 * ia6];
	         co8 = pCoef[2 * ia7];
	         si2 = pCoef[2 * ia1 + 1];
	         si3 = pCoef[2 * ia2 + 1];
	         si4 = pCoef[2 * ia3 + 1];
	         si5 = pCoef[2 * ia4 + 1];
	         si6 = pCoef[2 * ia5 + 1];
	         si7 = pCoef[2 * ia6 + 1];
	         si8 = pCoef[2 * ia7 + 1];

	         i1 = j;

	         do
	         {
	            /*  index calculation for the input */
	            i2 = i1 + n2;
	            i3 = i2 + n2;
	            i4 = i3 + n2;
	            i5 = i4 + n2;
	            i6 = i5 + n2;
	            i7 = i6 + n2;
	            i8 = i7 + n2;
	            r1 = pSrc[2 * i1] + pSrc[2 * i5];
	            r5 = pSrc[2 * i1] - pSrc[2 * i5];
	            r2 = pSrc[2 * i2] + pSrc[2 * i6];
	            r6 = pSrc[2 * i2] - pSrc[2 * i6];
	            r3 = pSrc[2 * i3] + pSrc[2 * i7];
	            r7 = pSrc[2 * i3] - pSrc[2 * i7];
	            r4 = pSrc[2 * i4] + pSrc[2 * i8];
	            r8 = pSrc[2 * i4] - pSrc[2 * i8];
	            t1 = r1 - r3;
	            r1 = r1 + r3;
	            r3 = r2 - r4;
	            r2 = r2 + r4;
	            pSrc[2 * i1] = r1 + r2;
	            r2 = r1 - r2;
	            s1 = pSrc[2 * i1 + 1] + pSrc[2 * i5 + 1];
	            s5 = pSrc[2 * i1 + 1] - pSrc[2 * i5 + 1];
	            s2 = pSrc[2 * i2 + 1] + pSrc[2 * i6 + 1];
	            s6 = pSrc[2 * i2 + 1] - pSrc[2 * i6 + 1];
	            s3 = pSrc[2 * i3 + 1] + pSrc[2 * i7 + 1];
	            s7 = pSrc[2 * i3 + 1] - pSrc[2 * i7 + 1];
	            s4 = pSrc[2 * i4 + 1] + pSrc[2 * i8 + 1];
	            s8 = pSrc[2 * i4 + 1] - pSrc[2 * i8 + 1];
	            t2 = s1 - s3;
	            s1 = s1 + s3;
	            s3 = s2 - s4;
	            s2 = s2 + s4;
	            r1 = t1 + s3;
	            t1 = t1 - s3;
	            pSrc[2 * i1 + 1] = s1 + s2;
	            s2 = s1 - s2;
	            s1 = t2 - r3;
	            t2 = t2 + r3;
	            p1 = co5 * r2;
	            p2 = si5 * s2;
	            p3 = co5 * s2;
	            p4 = si5 * r2;
	            pSrc[2 * i5]     = p1 + p2;
	            pSrc[2 * i5 + 1] = p3 - p4;
	            p1 = co3 * r1;
	            p2 = si3 * s1;
	            p3 = co3 * s1;
	            p4 = si3 * r1;
	            pSrc[2 * i3]     = p1 + p2;
	            pSrc[2 * i3 + 1] = p3 - p4;
	            p1 = co7 * t1;
	            p2 = si7 * t2;
	            p3 = co7 * t2;
	            p4 = si7 * t1;
	            pSrc[2 * i7]     = p1 + p2;
	            pSrc[2 * i7 + 1] = p3 - p4;
	            r1 = (r6 - r8) * C81;
	            r6 = (r6 + r8) * C81;
	            s1 = (s6 - s8) * C81;
	            s6 = (s6 + s8) * C81;
	            t1 = r5 - r1;
	            r5 = r5 + r1;
	            r8 = r7 - r6;
	            r7 = r7 + r6;
	            t2 = s5 - s1;
	            s5 = s5 + s1;
	            s8 = s7 - s6;
	            s7 = s7 + s6;
	            r1 = r5 + s7;
	            r5 = r5 - s7;
	            r6 = t1 + s8;
	            t1 = t1 - s8;
	            s1 = s5 - r7;
	            s5 = s5 + r7;
	            s6 = t2 - r8;
	            t2 = t2 + r8;
	            p1 = co2 * r1;
	            p2 = si2 * s1;
	            p3 = co2 * s1;
	            p4 = si2 * r1;
	            pSrc[2 * i2]     = p1 + p2;
	            pSrc[2 * i2 + 1] = p3 - p4;
	            p1 = co8 * r5;
	            p2 = si8 * s5;
	            p3 = co8 * s5;
	            p4 = si8 * r5;
	            pSrc[2 * i8]     = p1 + p2;
	            pSrc[2 * i8 + 1] = p3 - p4;
	            p1 = co6 * r6;
	            p2 = si6 * s6;
	            p3 = co6 * s6;
	            p4 = si6 * r6;
	            pSrc[2 * i6]     = p1 + p2;
	            pSrc[2 * i6 + 1] = p3 - p4;
	            p1 = co4 * t1;
	            p2 = si4 * t2;
	            p3 = co4 * t2;
	            p4 = si4 * t1;
	            pSrc[2 * i4]     = p1 + p2;
	            pSrc[2 * i4 + 1] = p3 - p4;

	            i1 += n1;
	         } while(i1 < fftLen);

	         j++;
	      } while(j < n2);

	      twidCoefModifier <<= 3;
	   } while(n2 > 7);
	}

	void arm_cfft_radix8by2_f32( arm_cfft_instance_f32 * S, float32_t * p1)
	{
	    uint32_t    L  = S->fftLen;
	    float32_t * pCol1, * pCol2, * pMid1, * pMid2;
	    float32_t * p2 = p1 + L;
	    const float32_t * tw = (float32_t *) S->pTwiddle;
	    float32_t t1[4], t2[4], t3[4], t4[4], twR, twI;
	    float32_t m0, m1, m2, m3;
	    uint32_t l;

	    pCol1 = p1;
	    pCol2 = p2;

	    //    Define new length
	    L >>= 1;
	    //    Initialize mid pointers
	    pMid1 = p1 + L;
	    pMid2 = p2 + L;

	    // do two dot Fourier transform
	    for ( l = L >> 2; l > 0; l-- )
	    {
	        t1[0] = p1[0];
	        t1[1] = p1[1];
	        t1[2] = p1[2];
	        t1[3] = p1[3];

	        t2[0] = p2[0];
	        t2[1] = p2[1];
	        t2[2] = p2[2];
	        t2[3] = p2[3];

	        t3[0] = pMid1[0];
	        t3[1] = pMid1[1];
	        t3[2] = pMid1[2];
	        t3[3] = pMid1[3];

	        t4[0] = pMid2[0];
	        t4[1] = pMid2[1];
	        t4[2] = pMid2[2];
	        t4[3] = pMid2[3];

	        *p1++ = t1[0] + t2[0];
	        *p1++ = t1[1] + t2[1];
	        *p1++ = t1[2] + t2[2];
	        *p1++ = t1[3] + t2[3];    // col 1

	        t2[0] = t1[0] - t2[0];
	        t2[1] = t1[1] - t2[1];
	        t2[2] = t1[2] - t2[2];
	        t2[3] = t1[3] - t2[3];    // for col 2

	        *pMid1++ = t3[0] + t4[0];
	        *pMid1++ = t3[1] + t4[1];
	        *pMid1++ = t3[2] + t4[2];
	        *pMid1++ = t3[3] + t4[3]; // col 1

	        t4[0] = t4[0] - t3[0];
	        t4[1] = t4[1] - t3[1];
	        t4[2] = t4[2] - t3[2];
	        t4[3] = t4[3] - t3[3];    // for col 2

	        twR = *tw++;
	        twI = *tw++;

	        // multiply by twiddle factors
	        m0 = t2[0] * twR;
	        m1 = t2[1] * twI;
	        m2 = t2[1] * twR;
	        m3 = t2[0] * twI;

	        // R  =  R  *  Tr - I * Ti
	        *p2++ = m0 + m1;
	        // I  =  I  *  Tr + R * Ti
	        *p2++ = m2 - m3;

	        // use vertical symmetry
	        //  0.9988 - 0.0491i <==> -0.0491 - 0.9988i
	        m0 = t4[0] * twI;
	        m1 = t4[1] * twR;
	        m2 = t4[1] * twI;
	        m3 = t4[0] * twR;

	        *pMid2++ = m0 - m1;
	        *pMid2++ = m2 + m3;

	        twR = *tw++;
	        twI = *tw++;

	        m0 = t2[2] * twR;
	        m1 = t2[3] * twI;
	        m2 = t2[3] * twR;
	        m3 = t2[2] * twI;

	        *p2++ = m0 + m1;
	        *p2++ = m2 - m3;

	        m0 = t4[2] * twI;
	        m1 = t4[3] * twR;
	        m2 = t4[3] * twI;
	        m3 = t4[2] * twR;

	        *pMid2++ = m0 - m1;
	        *pMid2++ = m2 + m3;
	    }

	    // first col
	    arm_radix8_butterfly_f32( pCol1, L, (float32_t *) S->pTwiddle, 2u);
	    // second col
	    arm_radix8_butterfly_f32( pCol2, L, (float32_t *) S->pTwiddle, 2u);
	}
	void arm_cfft_radix8by4_f32( arm_cfft_instance_f32 * S, float32_t * p1)
	{
	    uint32_t    L  = S->fftLen >> 1;
	    float32_t * pCol1, *pCol2, *pCol3, *pCol4, *pEnd1, *pEnd2, *pEnd3, *pEnd4;
	    const float32_t *tw2, *tw3, *tw4;
	    float32_t * p2 = p1 + L;
	    float32_t * p3 = p2 + L;
	    float32_t * p4 = p3 + L;
	    float32_t t2[4], t3[4], t4[4], twR, twI;
	    float32_t p1ap3_0, p1sp3_0, p1ap3_1, p1sp3_1;
	    float32_t m0, m1, m2, m3;
	    uint32_t l, twMod2, twMod3, twMod4;

	    pCol1 = p1;         // points to real values by default
	    pCol2 = p2;
	    pCol3 = p3;
	    pCol4 = p4;
	    pEnd1 = p2 - 1;     // points to imaginary values by default
	    pEnd2 = p3 - 1;
	    pEnd3 = p4 - 1;
	    pEnd4 = pEnd3 + L;

	    tw2 = tw3 = tw4 = (float32_t *) S->pTwiddle;

	    L >>= 1;

	    // do four dot Fourier transform

	    twMod2 = 2;
	    twMod3 = 4;
	    twMod4 = 6;

	    // TOP
	    p1ap3_0 = p1[0] + p3[0];
	    p1sp3_0 = p1[0] - p3[0];
	    p1ap3_1 = p1[1] + p3[1];
	    p1sp3_1 = p1[1] - p3[1];

	    // col 2
	    t2[0] = p1sp3_0 + p2[1] - p4[1];
	    t2[1] = p1sp3_1 - p2[0] + p4[0];
	    // col 3
	    t3[0] = p1ap3_0 - p2[0] - p4[0];
	    t3[1] = p1ap3_1 - p2[1] - p4[1];
	    // col 4
	    t4[0] = p1sp3_0 - p2[1] + p4[1];
	    t4[1] = p1sp3_1 + p2[0] - p4[0];
	    // col 1
	    *p1++ = p1ap3_0 + p2[0] + p4[0];
	    *p1++ = p1ap3_1 + p2[1] + p4[1];

	    // Twiddle factors are ones
	    *p2++ = t2[0];
	    *p2++ = t2[1];
	    *p3++ = t3[0];
	    *p3++ = t3[1];
	    *p4++ = t4[0];
	    *p4++ = t4[1];

	    tw2 += twMod2;
	    tw3 += twMod3;
	    tw4 += twMod4;

	    for (l = (L - 2) >> 1; l > 0; l-- )
	    {
	        // TOP
	        p1ap3_0 = p1[0] + p3[0];
	        p1sp3_0 = p1[0] - p3[0];
	        p1ap3_1 = p1[1] + p3[1];
	        p1sp3_1 = p1[1] - p3[1];
	        // col 2
	        t2[0] = p1sp3_0 + p2[1] - p4[1];
	        t2[1] = p1sp3_1 - p2[0] + p4[0];
	        // col 3
	        t3[0] = p1ap3_0 - p2[0] - p4[0];
	        t3[1] = p1ap3_1 - p2[1] - p4[1];
	        // col 4
	        t4[0] = p1sp3_0 - p2[1] + p4[1];
	        t4[1] = p1sp3_1 + p2[0] - p4[0];
	        // col 1 - top
	        *p1++ = p1ap3_0 + p2[0] + p4[0];
	        *p1++ = p1ap3_1 + p2[1] + p4[1];

	        // BOTTOM
	        p1ap3_1 = pEnd1[-1] + pEnd3[-1];
	        p1sp3_1 = pEnd1[-1] - pEnd3[-1];
	        p1ap3_0 = pEnd1[0] + pEnd3[0];
	        p1sp3_0 = pEnd1[0] - pEnd3[0];
	        // col 2
	        t2[2] = pEnd2[0]  - pEnd4[0] + p1sp3_1;
	        t2[3] = pEnd1[0] - pEnd3[0] - pEnd2[-1] + pEnd4[-1];
	        // col 3
	        t3[2] = p1ap3_1 - pEnd2[-1] - pEnd4[-1];
	        t3[3] = p1ap3_0 - pEnd2[0]  - pEnd4[0];
	        // col 4
	        t4[2] = pEnd2[0]  - pEnd4[0]  - p1sp3_1;
	        t4[3] = pEnd4[-1] - pEnd2[-1] - p1sp3_0;
	        // col 1 - Bottom
	        *pEnd1-- = p1ap3_0 + pEnd2[0] + pEnd4[0];
	        *pEnd1-- = p1ap3_1 + pEnd2[-1] + pEnd4[-1];

	        // COL 2
	        // read twiddle factors
	        twR = *tw2++;
	        twI = *tw2++;
	        // multiply by twiddle factors
	        //  let    Z1 = a + i(b),   Z2 = c + i(d)
	        //   =>  Z1 * Z2  =  (a*c - b*d) + i(b*c + a*d)

	        // Top
	        m0 = t2[0] * twR;
	        m1 = t2[1] * twI;
	        m2 = t2[1] * twR;
	        m3 = t2[0] * twI;

	        *p2++ = m0 + m1;
	        *p2++ = m2 - m3;
	        // use vertical symmetry col 2
	        // 0.9997 - 0.0245i  <==>  0.0245 - 0.9997i
	        // Bottom
	        m0 = t2[3] * twI;
	        m1 = t2[2] * twR;
	        m2 = t2[2] * twI;
	        m3 = t2[3] * twR;

	        *pEnd2-- = m0 - m1;
	        *pEnd2-- = m2 + m3;

	        // COL 3
	        twR = tw3[0];
	        twI = tw3[1];
	        tw3 += twMod3;
	        // Top
	        m0 = t3[0] * twR;
	        m1 = t3[1] * twI;
	        m2 = t3[1] * twR;
	        m3 = t3[0] * twI;

	        *p3++ = m0 + m1;
	        *p3++ = m2 - m3;
	        // use vertical symmetry col 3
	        // 0.9988 - 0.0491i  <==>  -0.9988 - 0.0491i
	        // Bottom
	        m0 = -t3[3] * twR;
	        m1 = t3[2] * twI;
	        m2 = t3[2] * twR;
	        m3 = t3[3] * twI;

	        *pEnd3-- = m0 - m1;
	        *pEnd3-- = m3 - m2;

	        // COL 4
	        twR = tw4[0];
	        twI = tw4[1];
	        tw4 += twMod4;
	        // Top
	        m0 = t4[0] * twR;
	        m1 = t4[1] * twI;
	        m2 = t4[1] * twR;
	        m3 = t4[0] * twI;

	        *p4++ = m0 + m1;
	        *p4++ = m2 - m3;
	        // use vertical symmetry col 4
	        // 0.9973 - 0.0736i  <==>  -0.0736 + 0.9973i
	        // Bottom
	        m0 = t4[3] * twI;
	        m1 = t4[2] * twR;
	        m2 = t4[2] * twI;
	        m3 = t4[3] * twR;

	        *pEnd4-- = m0 - m1;
	        *pEnd4-- = m2 + m3;
	    }

	    //MIDDLE
	    // Twiddle factors are
	    //  1.0000  0.7071-0.7071i  -1.0000i  -0.7071-0.7071i
	    p1ap3_0 = p1[0] + p3[0];
	    p1sp3_0 = p1[0] - p3[0];
	    p1ap3_1 = p1[1] + p3[1];
	    p1sp3_1 = p1[1] - p3[1];

	    // col 2
	    t2[0] = p1sp3_0 + p2[1] - p4[1];
	    t2[1] = p1sp3_1 - p2[0] + p4[0];
	    // col 3
	    t3[0] = p1ap3_0 - p2[0] - p4[0];
	    t3[1] = p1ap3_1 - p2[1] - p4[1];
	    // col 4
	    t4[0] = p1sp3_0 - p2[1] + p4[1];
	    t4[1] = p1sp3_1 + p2[0] - p4[0];
	    // col 1 - Top
	    *p1++ = p1ap3_0 + p2[0] + p4[0];
	    *p1++ = p1ap3_1 + p2[1] + p4[1];

	    // COL 2
	    twR = tw2[0];
	    twI = tw2[1];

	    m0 = t2[0] * twR;
	    m1 = t2[1] * twI;
	    m2 = t2[1] * twR;
	    m3 = t2[0] * twI;

	    *p2++ = m0 + m1;
	    *p2++ = m2 - m3;
	    // COL 3
	    twR = tw3[0];
	    twI = tw3[1];

	    m0 = t3[0] * twR;
	    m1 = t3[1] * twI;
	    m2 = t3[1] * twR;
	    m3 = t3[0] * twI;

	    *p3++ = m0 + m1;
	    *p3++ = m2 - m3;
	    // COL 4
	    twR = tw4[0];
	    twI = tw4[1];

	    m0 = t4[0] * twR;
	    m1 = t4[1] * twI;
	    m2 = t4[1] * twR;
	    m3 = t4[0] * twI;

	    *p4++ = m0 + m1;
	    *p4++ = m2 - m3;

	    // first col
	    arm_radix8_butterfly_f32( pCol1, L, (float32_t *) S->pTwiddle, 4u);
	    // second col
	    arm_radix8_butterfly_f32( pCol2, L, (float32_t *) S->pTwiddle, 4u);
	    // third col
	    arm_radix8_butterfly_f32( pCol3, L, (float32_t *) S->pTwiddle, 4u);
	    // fourth col
	    arm_radix8_butterfly_f32( pCol4, L, (float32_t *) S->pTwiddle, 4u);
	}

	void arm_cfft_f32(
	    const arm_cfft_instance_f32 * S,
	    float32_t * p1,
	    uint8_t ifftFlag,
	    uint8_t bitReverseFlag)
	{
	    uint32_t  L = S->fftLen, l;
	    float32_t invL, * pSrc;

	    if(ifftFlag == 1u)
	    {
	        /*  Conjugate input data  */
	        pSrc = p1 + 1;
	        for(l=0; l<L; l++)
	        {
	            *pSrc = -*pSrc;
	            pSrc += 2;
	        }
	    }

	    switch (L)
	    {
	    case 16:
	    case 128:
	    case 1024:
	        arm_cfft_radix8by2_f32  ( (arm_cfft_instance_f32 *) S, p1);
	        break;
	    case 32:
	    case 256:
	    case 2048:
	        arm_cfft_radix8by4_f32  ( (arm_cfft_instance_f32 *) S, p1);
	        break;
	    case 64:
	    case 512:
	    case 4096:
	        arm_radix8_butterfly_f32( p1, L, (float32_t *) S->pTwiddle, 1);
	        break;
	    }

	    if( bitReverseFlag )
	        arm_bitreversal_32((uint32_t*)p1,S->bitRevLength,S->pBitRevTable);

	    if(ifftFlag == 1u)
	    {
	        invL = 1.0f/(float32_t)L;
	        /*  Conjugate and scale output data */
	        pSrc = p1;
	        for(l=0; l<L; l++)
	        {
	            *pSrc++ *=   invL ;
	            *pSrc  = -(*pSrc) * invL;
	            pSrc++;
	        }
	    }
	}
	void arm_radix4_butterfly_inverse_q31(
	  q31_t * pSrc,
	  uint32_t fftLen,
	  q31_t * pCoef,
	  uint32_t twidCoefModifier)
	{
	#if defined(ARM_MATH_CM7)
	  uint32_t n1, n2, ia1, ia2, ia3, i0, i1, i2, i3, j, k;
	  q31_t t1, t2, r1, r2, s1, s2, co1, co2, co3, si1, si2, si3;
	  q31_t xa, xb, xc, xd;
	  q31_t ya, yb, yc, yd;
	  q31_t xa_out, xb_out, xc_out, xd_out;
	  q31_t ya_out, yb_out, yc_out, yd_out;

	  q31_t *ptr1;
	  q63_t xaya, xbyb, xcyc, xdyd;

	  /* input is be 1.31(q31) format for all FFT sizes */
	  /* Total process is divided into three stages */
	  /* process first stage, middle stages, & last stage */

	  /* Start of first stage process */

	  /* Initializations for the first stage */
	  n2 = fftLen;
	  n1 = n2;
	  /* n2 = fftLen/4 */
	  n2 >>= 2u;
	  i0 = 0u;
	  ia1 = 0u;

	  j = n2;

	  do
	  {

	    /* input is in 1.31(q31) format and provide 4 guard bits for the input */

	    /*  index calculation for the input as, */
	    /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2u], pSrc[i0 + 3fftLen/4] */
	    i1 = i0 + n2;
	    i2 = i1 + n2;
	    i3 = i2 + n2;

	    /*  Butterfly implementation */
	    /* xa + xc */
	    r1 = (pSrc[2u * i0] >> 4u) + (pSrc[2u * i2] >> 4u);
	    /* xa - xc */
	    r2 = (pSrc[2u * i0] >> 4u) - (pSrc[2u * i2] >> 4u);

	    /* xb + xd */
	    t1 = (pSrc[2u * i1] >> 4u) + (pSrc[2u * i3] >> 4u);

	    /* ya + yc */
	    s1 = (pSrc[(2u * i0) + 1u] >> 4u) + (pSrc[(2u * i2) + 1u] >> 4u);
	    /* ya - yc */
	    s2 = (pSrc[(2u * i0) + 1u] >> 4u) - (pSrc[(2u * i2) + 1u] >> 4u);

	    /* xa' = xa + xb + xc + xd */
	    pSrc[2u * i0] = (r1 + t1);
	    /* (xa + xc) - (xb + xd) */
	    r1 = r1 - t1;
	    /* yb + yd */
	    t2 = (pSrc[(2u * i1) + 1u] >> 4u) + (pSrc[(2u * i3) + 1u] >> 4u);
	    /* ya' = ya + yb + yc + yd */
	    pSrc[(2u * i0) + 1u] = (s1 + t2);

	    /* (ya + yc) - (yb + yd) */
	    s1 = s1 - t2;

	    /* yb - yd */
	    t1 = (pSrc[(2u * i1) + 1u] >> 4u) - (pSrc[(2u * i3) + 1u] >> 4u);
	    /* xb - xd */
	    t2 = (pSrc[2u * i1] >> 4u) - (pSrc[2u * i3] >> 4u);

	    /*  index calculation for the coefficients */
	    ia2 = 2u * ia1;
	    co2 = pCoef[ia2 * 2u];
	    si2 = pCoef[(ia2 * 2u) + 1u];

	    /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	    pSrc[2u * i1] = (((int32_t) (((q63_t) r1 * co2) >> 32)) -
	                     ((int32_t) (((q63_t) s1 * si2) >> 32))) << 1u;

	    /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	    pSrc[2u * i1 + 1u] = (((int32_t) (((q63_t) s1 * co2) >> 32)) +
	                          ((int32_t) (((q63_t) r1 * si2) >> 32))) << 1u;

	    /* (xa - xc) - (yb - yd) */
	    r1 = r2 - t1;
	    /* (xa - xc) + (yb - yd) */
	    r2 = r2 + t1;

	    /* (ya - yc) + (xb - xd) */
	    s1 = s2 + t2;
	    /* (ya - yc) - (xb - xd) */
	    s2 = s2 - t2;

	    co1 = pCoef[ia1 * 2u];
	    si1 = pCoef[(ia1 * 2u) + 1u];

	    /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	    pSrc[2u * i2] = (((int32_t) (((q63_t) r1 * co1) >> 32)) -
	                     ((int32_t) (((q63_t) s1 * si1) >> 32))) << 1u;

	    /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	    pSrc[(2u * i2) + 1u] = (((int32_t) (((q63_t) s1 * co1) >> 32)) +
	                            ((int32_t) (((q63_t) r1 * si1) >> 32))) << 1u;

	    /*  index calculation for the coefficients */
	    ia3 = 3u * ia1;
	    co3 = pCoef[ia3 * 2u];
	    si3 = pCoef[(ia3 * 2u) + 1u];

	    /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	    pSrc[2u * i3] = (((int32_t) (((q63_t) r2 * co3) >> 32)) -
	                     ((int32_t) (((q63_t) s2 * si3) >> 32))) << 1u;

	    /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	    pSrc[(2u * i3) + 1u] = (((int32_t) (((q63_t) s2 * co3) >> 32)) +
	                            ((int32_t) (((q63_t) r2 * si3) >> 32))) << 1u;

	    /*  Twiddle coefficients index modifier */
	    ia1 = ia1 + twidCoefModifier;

	    /*  Updating input index */
	    i0 = i0 + 1u;

	  } while(--j);

	  /* data is in 5.27(q27) format */
	  /* each stage provides two down scaling of the input */


	  /* Start of Middle stages process */

	  twidCoefModifier <<= 2u;

	  /*  Calculation of second stage to excluding last stage */
	  for (k = fftLen / 4u; k > 4u; k >>= 2u)
	  {
	    /*  Initializations for the first stage */
	    n1 = n2;
	    n2 >>= 2u;
	    ia1 = 0u;

	    for (j = 0; j <= (n2 - 1u); j++)
	    {
	      /*  index calculation for the coefficients */
	      ia2 = ia1 + ia1;
	      ia3 = ia2 + ia1;
	      co1 = pCoef[ia1 * 2u];
	      si1 = pCoef[(ia1 * 2u) + 1u];
	      co2 = pCoef[ia2 * 2u];
	      si2 = pCoef[(ia2 * 2u) + 1u];
	      co3 = pCoef[ia3 * 2u];
	      si3 = pCoef[(ia3 * 2u) + 1u];
	      /*  Twiddle coefficients index modifier */
	      ia1 = ia1 + twidCoefModifier;

	      for (i0 = j; i0 < fftLen; i0 += n1)
	      {
	        /*  index calculation for the input as, */
	        /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2u], pSrc[i0 + 3fftLen/4] */
	        i1 = i0 + n2;
	        i2 = i1 + n2;
	        i3 = i2 + n2;

	        /*  Butterfly implementation */
	        /* xa + xc */
	        r1 = pSrc[2u * i0] + pSrc[2u * i2];
	        /* xa - xc */
	        r2 = pSrc[2u * i0] - pSrc[2u * i2];

	        /* ya + yc */
	        s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];
	        /* ya - yc */
	        s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

	        /* xb + xd */
	        t1 = pSrc[2u * i1] + pSrc[2u * i3];

	        /* xa' = xa + xb + xc + xd */
	        pSrc[2u * i0] = (r1 + t1) >> 2u;
	        /* xa + xc -(xb + xd) */
	        r1 = r1 - t1;
	        /* yb + yd */
	        t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];
	        /* ya' = ya + yb + yc + yd */
	        pSrc[(2u * i0) + 1u] = (s1 + t2) >> 2u;

	        /* (ya + yc) - (yb + yd) */
	        s1 = s1 - t2;

	        /* (yb - yd) */
	        t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];
	        /* (xb - xd) */
	        t2 = pSrc[2u * i1] - pSrc[2u * i3];

	        /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	        pSrc[2u * i1] = (((int32_t) (((q63_t) r1 * co2) >> 32u)) -
	                         ((int32_t) (((q63_t) s1 * si2) >> 32u))) >> 1u;

	        /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	        pSrc[(2u * i1) + 1u] =
	          (((int32_t) (((q63_t) s1 * co2) >> 32u)) +
	           ((int32_t) (((q63_t) r1 * si2) >> 32u))) >> 1u;

	        /* (xa - xc) - (yb - yd) */
	        r1 = r2 - t1;
	        /* (xa - xc) + (yb - yd) */
	        r2 = r2 + t1;

	        /* (ya - yc) +  (xb - xd) */
	        s1 = s2 + t2;
	        /* (ya - yc) -  (xb - xd) */
	        s2 = s2 - t2;

	        /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	        pSrc[2u * i2] = (((int32_t) (((q63_t) r1 * co1) >> 32)) -
	                         ((int32_t) (((q63_t) s1 * si1) >> 32))) >> 1u;

	        /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	        pSrc[(2u * i2) + 1u] = (((int32_t) (((q63_t) s1 * co1) >> 32)) +
	                                ((int32_t) (((q63_t) r1 * si1) >> 32))) >> 1u;

	        /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	        pSrc[(2u * i3)] = (((int32_t) (((q63_t) r2 * co3) >> 32)) -
	                           ((int32_t) (((q63_t) s2 * si3) >> 32))) >> 1u;

	        /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	        pSrc[(2u * i3) + 1u] = (((int32_t) (((q63_t) s2 * co3) >> 32)) +
	                                ((int32_t) (((q63_t) r2 * si3) >> 32))) >> 1u;
	      }
	    }
	    twidCoefModifier <<= 2u;
	  }
	#else
	  uint32_t n1, n2, ia1, ia2, ia3, i0, j, k;
	  q31_t t1, t2, r1, r2, s1, s2, co1, co2, co3, si1, si2, si3;
	  q31_t xa, xb, xc, xd;
	  q31_t ya, yb, yc, yd;
	  q31_t xa_out, xb_out, xc_out, xd_out;
	  q31_t ya_out, yb_out, yc_out, yd_out;

	  q31_t *ptr1;
	  q31_t *pSi0;
	  q31_t *pSi1;
	  q31_t *pSi2;
	  q31_t *pSi3;
	  q63_t xaya, xbyb, xcyc, xdyd;

	  /* input is be 1.31(q31) format for all FFT sizes */
	  /* Total process is divided into three stages */
	  /* process first stage, middle stages, & last stage */

	  /* Start of first stage process */

	  /* Initializations for the first stage */
	  n2 = fftLen;
	  n1 = n2;
	  /* n2 = fftLen/4 */
	  n2 >>= 2u;

	  ia1 = 0u;

	  j = n2;

	  pSi0 = pSrc;
	  pSi1 = pSi0 + 2 * n2;
	  pSi2 = pSi1 + 2 * n2;
	  pSi3 = pSi2 + 2 * n2;

	  do
	  {
	    /*  Butterfly implementation */
	    /* xa + xc */
	    r1 = (pSi0[0] >> 4u) + (pSi2[0] >> 4u);
	    /* xa - xc */
	    r2 = (pSi0[0] >> 4u) - (pSi2[0] >> 4u);

	    /* xb + xd */
	    t1 = (pSi1[0] >> 4u) + (pSi3[0] >> 4u);

	    /* ya + yc */
	    s1 = (pSi0[1] >> 4u) + (pSi2[1] >> 4u);
	    /* ya - yc */
	    s2 = (pSi0[1] >> 4u) - (pSi2[1] >> 4u);

	    /* xa' = xa + xb + xc + xd */
	    *pSi0++ = (r1 + t1);
	    /* (xa + xc) - (xb + xd) */
	    r1 = r1 - t1;
	    /* yb + yd */
	    t2 = (pSi1[1] >> 4u) + (pSi3[1] >> 4u);
	    /* ya' = ya + yb + yc + yd */
	    *pSi0++ = (s1 + t2);

	    /* (ya + yc) - (yb + yd) */
	    s1 = s1 - t2;

	    /* yb - yd */
	    t1 = (pSi1[1] >> 4u) - (pSi3[1] >> 4u);
	    /* xb - xd */
	    t2 = (pSi1[0] >> 4u) - (pSi3[0] >> 4u);

	    /*  index calculation for the coefficients */
	    ia2 = 2u * ia1;
	    co2 = pCoef[ia2 * 2u];
	    si2 = pCoef[(ia2 * 2u) + 1u];

	    /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	    *pSi1++ = (((int32_t) (((q63_t) r1 * co2) >> 32)) -
	                     ((int32_t) (((q63_t) s1 * si2) >> 32))) << 1u;

	    /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	    *pSi1++ = (((int32_t) (((q63_t) s1 * co2) >> 32)) +
	                          ((int32_t) (((q63_t) r1 * si2) >> 32))) << 1u;

	    /* (xa - xc) - (yb - yd) */
	    r1 = r2 - t1;
	    /* (xa - xc) + (yb - yd) */
	    r2 = r2 + t1;

	    /* (ya - yc) + (xb - xd) */
	    s1 = s2 + t2;
	    /* (ya - yc) - (xb - xd) */
	    s2 = s2 - t2;

	    co1 = pCoef[ia1 * 2u];
	    si1 = pCoef[(ia1 * 2u) + 1u];

	    /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	    *pSi2++ = (((int32_t) (((q63_t) r1 * co1) >> 32)) -
	                     ((int32_t) (((q63_t) s1 * si1) >> 32))) << 1u;

	    /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	    *pSi2++ = (((int32_t) (((q63_t) s1 * co1) >> 32)) +
	                            ((int32_t) (((q63_t) r1 * si1) >> 32))) << 1u;

	    /*  index calculation for the coefficients */
	    ia3 = 3u * ia1;
	    co3 = pCoef[ia3 * 2u];
	    si3 = pCoef[(ia3 * 2u) + 1u];

	    /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	    *pSi3++ = (((int32_t) (((q63_t) r2 * co3) >> 32)) -
	                     ((int32_t) (((q63_t) s2 * si3) >> 32))) << 1u;

	    /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	    *pSi3++ = (((int32_t) (((q63_t) s2 * co3) >> 32)) +
	                            ((int32_t) (((q63_t) r2 * si3) >> 32))) << 1u;

	    /*  Twiddle coefficients index modifier */
	    ia1 = ia1 + twidCoefModifier;

	  } while(--j);

	  /* data is in 5.27(q27) format */
	  /* each stage provides two down scaling of the input */


	  /* Start of Middle stages process */

	  twidCoefModifier <<= 2u;

	  /*  Calculation of second stage to excluding last stage */
	  for (k = fftLen / 4u; k > 4u; k >>= 2u)
	  {
	    /*  Initializations for the first stage */
	    n1 = n2;
	    n2 >>= 2u;
	    ia1 = 0u;

	    for (j = 0; j <= (n2 - 1u); j++)
	    {
	      /*  index calculation for the coefficients */
	      ia2 = ia1 + ia1;
	      ia3 = ia2 + ia1;
	      co1 = pCoef[ia1 * 2u];
	      si1 = pCoef[(ia1 * 2u) + 1u];
	      co2 = pCoef[ia2 * 2u];
	      si2 = pCoef[(ia2 * 2u) + 1u];
	      co3 = pCoef[ia3 * 2u];
	      si3 = pCoef[(ia3 * 2u) + 1u];
	      /*  Twiddle coefficients index modifier */
	      ia1 = ia1 + twidCoefModifier;

	      pSi0 = pSrc + 2 * j;
	      pSi1 = pSi0 + 2 * n2;
	      pSi2 = pSi1 + 2 * n2;
	      pSi3 = pSi2 + 2 * n2;

	      for (i0 = j; i0 < fftLen; i0 += n1)
	      {
	        /*  Butterfly implementation */
	        /* xa + xc */
	        r1 = pSi0[0] + pSi2[0];

	        /* xa - xc */
	        r2 = pSi0[0] - pSi2[0];


	        /* ya + yc */
	        s1 = pSi0[1] + pSi2[1];

	        /* ya - yc */
	        s2 = pSi0[1] - pSi2[1];


	        /* xb + xd */
	        t1 = pSi1[0] + pSi3[0];


	        /* xa' = xa + xb + xc + xd */
	        pSi0[0] = (r1 + t1) >> 2u;
	        /* xa + xc -(xb + xd) */
	        r1 = r1 - t1;
	        /* yb + yd */
	        t2 = pSi1[1] + pSi3[1];

	        /* ya' = ya + yb + yc + yd */
	        pSi0[1] = (s1 + t2) >> 2u;
	        pSi0 += 2 * n1;

	        /* (ya + yc) - (yb + yd) */
	        s1 = s1 - t2;

	        /* (yb - yd) */
	        t1 = pSi1[1] - pSi3[1];

	        /* (xb - xd) */
	        t2 = pSi1[0] - pSi3[0];


	        /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	        pSi1[0] = (((int32_t) (((q63_t) r1 * co2) >> 32u)) -
	                         ((int32_t) (((q63_t) s1 * si2) >> 32u))) >> 1u;

	        /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	        pSi1[1] =

	          (((int32_t) (((q63_t) s1 * co2) >> 32u)) +
	           ((int32_t) (((q63_t) r1 * si2) >> 32u))) >> 1u;
	        pSi1 += 2 * n1;

	        /* (xa - xc) - (yb - yd) */
	        r1 = r2 - t1;
	        /* (xa - xc) + (yb - yd) */
	        r2 = r2 + t1;

	        /* (ya - yc) +  (xb - xd) */
	        s1 = s2 + t2;
	        /* (ya - yc) -  (xb - xd) */
	        s2 = s2 - t2;

	        /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	        pSi2[0] = (((int32_t) (((q63_t) r1 * co1) >> 32)) -
	                         ((int32_t) (((q63_t) s1 * si1) >> 32))) >> 1u;

	        /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	        pSi2[1] = (((int32_t) (((q63_t) s1 * co1) >> 32)) +
	                                ((int32_t) (((q63_t) r1 * si1) >> 32))) >> 1u;
	        pSi2 += 2 * n1;

	        /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	        pSi3[0] = (((int32_t) (((q63_t) r2 * co3) >> 32)) -
	                           ((int32_t) (((q63_t) s2 * si3) >> 32))) >> 1u;

	        /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	        pSi3[1] = (((int32_t) (((q63_t) s2 * co3) >> 32)) +
	                                ((int32_t) (((q63_t) r2 * si3) >> 32))) >> 1u;
	        pSi3 += 2 * n1;
	      }
	    }
	    twidCoefModifier <<= 2u;
	  }
	#endif

	  /* End of Middle stages process */

	  /* data is in 11.21(q21) format for the 1024 point as there are 3 middle stages */
	  /* data is in 9.23(q23) format for the 256 point as there are 2 middle stages */
	  /* data is in 7.25(q25) format for the 64 point as there are 1 middle stage */
	  /* data is in 5.27(q27) format for the 16 point as there are no middle stages */


	  /* Start of last stage process */


	  /*  Initializations for the last stage */
	  j = fftLen >> 2;
	  ptr1 = &pSrc[0];

	  /*  Calculations of last stage */
	  do
	  {
	#ifndef ARM_MATH_BIG_ENDIAN
	    /* Read xa (real), ya(imag) input */
	    xaya = *__SIMD64(ptr1)++;
	    xa = (q31_t) xaya;
	    ya = (q31_t) (xaya >> 32);

	    /* Read xb (real), yb(imag) input */
	    xbyb = *__SIMD64(ptr1)++;
	    xb = (q31_t) xbyb;
	    yb = (q31_t) (xbyb >> 32);

	    /* Read xc (real), yc(imag) input */
	    xcyc = *__SIMD64(ptr1)++;
	    xc = (q31_t) xcyc;
	    yc = (q31_t) (xcyc >> 32);

	    /* Read xc (real), yc(imag) input */
	    xdyd = *__SIMD64(ptr1)++;
	    xd = (q31_t) xdyd;
	    yd = (q31_t) (xdyd >> 32);

	#else

	    /* Read xa (real), ya(imag) input */
	    xaya = *__SIMD64(ptr1)++;
	    ya = (q31_t) xaya;
	    xa = (q31_t) (xaya >> 32);

	    /* Read xb (real), yb(imag) input */
	    xbyb = *__SIMD64(ptr1)++;
	    yb = (q31_t) xbyb;
	    xb = (q31_t) (xbyb >> 32);

	    /* Read xc (real), yc(imag) input */
	    xcyc = *__SIMD64(ptr1)++;
	    yc = (q31_t) xcyc;
	    xc = (q31_t) (xcyc >> 32);

	    /* Read xc (real), yc(imag) input */
	    xdyd = *__SIMD64(ptr1)++;
	    yd = (q31_t) xdyd;
	    xd = (q31_t) (xdyd >> 32);


	#endif

	    /* xa' = xa + xb + xc + xd */
	    xa_out = xa + xb + xc + xd;

	    /* ya' = ya + yb + yc + yd */
	    ya_out = ya + yb + yc + yd;

	    /* pointer updation for writing */
	    ptr1 = ptr1 - 8u;

	    /* writing xa' and ya' */
	    *ptr1++ = xa_out;
	    *ptr1++ = ya_out;

	    xc_out = (xa - xb + xc - xd);
	    yc_out = (ya - yb + yc - yd);

	    /* writing xc' and yc' */
	    *ptr1++ = xc_out;
	    *ptr1++ = yc_out;

	    xb_out = (xa - yb - xc + yd);
	    yb_out = (ya + xb - yc - xd);

	    /* writing xb' and yb' */
	    *ptr1++ = xb_out;
	    *ptr1++ = yb_out;

	    xd_out = (xa + yb - xc - yd);
	    yd_out = (ya - xb - yc + xd);

	    /* writing xd' and yd' */
	    *ptr1++ = xd_out;
	    *ptr1++ = yd_out;

	  } while(--j);

	  /* output is in 11.21(q21) format for the 1024 point */
	  /* output is in 9.23(q23) format for the 256 point */
	  /* output is in 7.25(q25) format for the 64 point */
	  /* output is in 5.27(q27) format for the 16 point */

	  /* End of last stage process */
	}

	void arm_radix4_butterfly_q31(
	  q31_t * pSrc,
	  uint32_t fftLen,
	  q31_t * pCoef,
	  uint32_t twidCoefModifier)
	{
	#if defined(ARM_MATH_CM7)
	  uint32_t n1, n2, ia1, ia2, ia3, i0, i1, i2, i3, j, k;
	  q31_t t1, t2, r1, r2, s1, s2, co1, co2, co3, si1, si2, si3;

	  q31_t xa, xb, xc, xd;
	  q31_t ya, yb, yc, yd;
	  q31_t xa_out, xb_out, xc_out, xd_out;
	  q31_t ya_out, yb_out, yc_out, yd_out;

	  q31_t *ptr1;
	  q63_t xaya, xbyb, xcyc, xdyd;
	  /* Total process is divided into three stages */

	  /* process first stage, middle stages, & last stage */


	  /* start of first stage process */

	  /*  Initializations for the first stage */
	  n2 = fftLen;
	  n1 = n2;
	  /* n2 = fftLen/4 */
	  n2 >>= 2u;
	  i0 = 0u;
	  ia1 = 0u;

	  j = n2;

	  /*  Calculation of first stage */
	  do
	  {
	    /*  index calculation for the input as, */
	    /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2u], pSrc[i0 + 3fftLen/4] */
	    i1 = i0 + n2;
	    i2 = i1 + n2;
	    i3 = i2 + n2;

	    /* input is in 1.31(q31) format and provide 4 guard bits for the input */

	    /*  Butterfly implementation */
	    /* xa + xc */
	    r1 = (pSrc[(2u * i0)] >> 4u) + (pSrc[(2u * i2)] >> 4u);
	    /* xa - xc */
	    r2 = (pSrc[2u * i0] >> 4u) - (pSrc[2u * i2] >> 4u);

	    /* xb + xd */
	    t1 = (pSrc[2u * i1] >> 4u) + (pSrc[2u * i3] >> 4u);

	    /* ya + yc */
	    s1 = (pSrc[(2u * i0) + 1u] >> 4u) + (pSrc[(2u * i2) + 1u] >> 4u);
	    /* ya - yc */
	    s2 = (pSrc[(2u * i0) + 1u] >> 4u) - (pSrc[(2u * i2) + 1u] >> 4u);

	    /* xa' = xa + xb + xc + xd */
	    pSrc[2u * i0] = (r1 + t1);
	    /* (xa + xc) - (xb + xd) */
	    r1 = r1 - t1;
	    /* yb + yd */
	    t2 = (pSrc[(2u * i1) + 1u] >> 4u) + (pSrc[(2u * i3) + 1u] >> 4u);

	    /* ya' = ya + yb + yc + yd */
	    pSrc[(2u * i0) + 1u] = (s1 + t2);

	    /* (ya + yc) - (yb + yd) */
	    s1 = s1 - t2;

	    /* yb - yd */
	    t1 = (pSrc[(2u * i1) + 1u] >> 4u) - (pSrc[(2u * i3) + 1u] >> 4u);
	    /* xb - xd */
	    t2 = (pSrc[2u * i1] >> 4u) - (pSrc[2u * i3] >> 4u);

	    /*  index calculation for the coefficients */
	    ia2 = 2u * ia1;
	    co2 = pCoef[ia2 * 2u];
	    si2 = pCoef[(ia2 * 2u) + 1u];

	    /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
	    pSrc[2u * i1] = (((int32_t) (((q63_t) r1 * co2) >> 32)) +
	                     ((int32_t) (((q63_t) s1 * si2) >> 32))) << 1u;

	    /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
	    pSrc[(2u * i1) + 1u] = (((int32_t) (((q63_t) s1 * co2) >> 32)) -
	                            ((int32_t) (((q63_t) r1 * si2) >> 32))) << 1u;

	    /* (xa - xc) + (yb - yd) */
	    r1 = r2 + t1;
	    /* (xa - xc) - (yb - yd) */
	    r2 = r2 - t1;

	    /* (ya - yc) - (xb - xd) */
	    s1 = s2 - t2;
	    /* (ya - yc) + (xb - xd) */
	    s2 = s2 + t2;

	    co1 = pCoef[ia1 * 2u];
	    si1 = pCoef[(ia1 * 2u) + 1u];

	    /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
	    pSrc[2u * i2] = (((int32_t) (((q63_t) r1 * co1) >> 32)) +
	                     ((int32_t) (((q63_t) s1 * si1) >> 32))) << 1u;

	    /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
	    pSrc[(2u * i2) + 1u] = (((int32_t) (((q63_t) s1 * co1) >> 32)) -
	                            ((int32_t) (((q63_t) r1 * si1) >> 32))) << 1u;

	    /*  index calculation for the coefficients */
	    ia3 = 3u * ia1;
	    co3 = pCoef[ia3 * 2u];
	    si3 = pCoef[(ia3 * 2u) + 1u];

	    /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
	    pSrc[2u * i3] = (((int32_t) (((q63_t) r2 * co3) >> 32)) +
	                     ((int32_t) (((q63_t) s2 * si3) >> 32))) << 1u;

	    /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
	    pSrc[(2u * i3) + 1u] = (((int32_t) (((q63_t) s2 * co3) >> 32)) -
	                            ((int32_t) (((q63_t) r2 * si3) >> 32))) << 1u;

	    /*  Twiddle coefficients index modifier */
	    ia1 = ia1 + twidCoefModifier;

	    /*  Updating input index */
	    i0 = i0 + 1u;

	  } while(--j);

	  /* end of first stage process */

	  /* data is in 5.27(q27) format */


	  /* start of Middle stages process */


	  /* each stage in middle stages provides two down scaling of the input */

	  twidCoefModifier <<= 2u;


	  for (k = fftLen / 4u; k > 4u; k >>= 2u)
	  {
	    /*  Initializations for the first stage */
	    n1 = n2;
	    n2 >>= 2u;
	    ia1 = 0u;

	    /*  Calculation of first stage */
	    for (j = 0u; j <= (n2 - 1u); j++)
	    {
	      /*  index calculation for the coefficients */
	      ia2 = ia1 + ia1;
	      ia3 = ia2 + ia1;
	      co1 = pCoef[ia1 * 2u];
	      si1 = pCoef[(ia1 * 2u) + 1u];
	      co2 = pCoef[ia2 * 2u];
	      si2 = pCoef[(ia2 * 2u) + 1u];
	      co3 = pCoef[ia3 * 2u];
	      si3 = pCoef[(ia3 * 2u) + 1u];
	      /*  Twiddle coefficients index modifier */
	      ia1 = ia1 + twidCoefModifier;

	      for (i0 = j; i0 < fftLen; i0 += n1)
	      {
	        /*  index calculation for the input as, */
	        /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2u], pSrc[i0 + 3fftLen/4] */
	        i1 = i0 + n2;
	        i2 = i1 + n2;
	        i3 = i2 + n2;

	        /*  Butterfly implementation */
	        /* xa + xc */
	        r1 = pSrc[2u * i0] + pSrc[2u * i2];
	        /* xa - xc */
	        r2 = pSrc[2u * i0] - pSrc[2u * i2];

	        /* ya + yc */
	        s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];
	        /* ya - yc */
	        s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

	        /* xb + xd */
	        t1 = pSrc[2u * i1] + pSrc[2u * i3];

	        /* xa' = xa + xb + xc + xd */
	        pSrc[2u * i0] = (r1 + t1) >> 2u;
	        /* xa + xc -(xb + xd) */
	        r1 = r1 - t1;

	        /* yb + yd */
	        t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];
	        /* ya' = ya + yb + yc + yd */
	        pSrc[(2u * i0) + 1u] = (s1 + t2) >> 2u;

	        /* (ya + yc) - (yb + yd) */
	        s1 = s1 - t2;

	        /* (yb - yd) */
	        t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];
	        /* (xb - xd) */
	        t2 = pSrc[2u * i1] - pSrc[2u * i3];

	        /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
	        pSrc[2u * i1] = (((int32_t) (((q63_t) r1 * co2) >> 32)) +
	                         ((int32_t) (((q63_t) s1 * si2) >> 32))) >> 1u;

	        /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
	        pSrc[(2u * i1) + 1u] = (((int32_t) (((q63_t) s1 * co2) >> 32)) -
	                                ((int32_t) (((q63_t) r1 * si2) >> 32))) >> 1u;

	        /* (xa - xc) + (yb - yd) */
	        r1 = r2 + t1;
	        /* (xa - xc) - (yb - yd) */
	        r2 = r2 - t1;

	        /* (ya - yc) -  (xb - xd) */
	        s1 = s2 - t2;
	        /* (ya - yc) +  (xb - xd) */
	        s2 = s2 + t2;

	        /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
	        pSrc[2u * i2] = (((int32_t) (((q63_t) r1 * co1) >> 32)) +
	                         ((int32_t) (((q63_t) s1 * si1) >> 32))) >> 1u;

	        /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
	        pSrc[(2u * i2) + 1u] = (((int32_t) (((q63_t) s1 * co1) >> 32)) -
	                                ((int32_t) (((q63_t) r1 * si1) >> 32))) >> 1u;

	        /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
	        pSrc[2u * i3] = (((int32_t) (((q63_t) r2 * co3) >> 32)) +
	                         ((int32_t) (((q63_t) s2 * si3) >> 32))) >> 1u;

	        /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
	        pSrc[(2u * i3) + 1u] = (((int32_t) (((q63_t) s2 * co3) >> 32)) -
	                                ((int32_t) (((q63_t) r2 * si3) >> 32))) >> 1u;
	      }
	    }
	    twidCoefModifier <<= 2u;
	  }
	#else
	  uint32_t n1, n2, ia1, ia2, ia3, i0, j, k;
	  q31_t t1, t2, r1, r2, s1, s2, co1, co2, co3, si1, si2, si3;

	  q31_t xa, xb, xc, xd;
	  q31_t ya, yb, yc, yd;
	  q31_t xa_out, xb_out, xc_out, xd_out;
	  q31_t ya_out, yb_out, yc_out, yd_out;

	  q31_t *ptr1;
	  q31_t *pSi0;
	  q31_t *pSi1;
	  q31_t *pSi2;
	  q31_t *pSi3;
	  q63_t xaya, xbyb, xcyc, xdyd;
	  /* Total process is divided into three stages */

	  /* process first stage, middle stages, & last stage */


	  /* start of first stage process */

	  /*  Initializations for the first stage */
	  n2 = fftLen;
	  n1 = n2;
	  /* n2 = fftLen/4 */
	  n2 >>= 2u;

	  ia1 = 0u;

	  j = n2;

	  pSi0 = pSrc;
	  pSi1 = pSi0 + 2 * n2;
	  pSi2 = pSi1 + 2 * n2;
	  pSi3 = pSi2 + 2 * n2;

	  /*  Calculation of first stage */
	  do
	  {
	    /* input is in 1.31(q31) format and provide 4 guard bits for the input */

	    /*  Butterfly implementation */
	    /* xa + xc */
	    r1 = (pSi0[0] >> 4u) + (pSi2[0] >> 4u);
	    /* xa - xc */
	    r2 = (pSi0[0] >> 4u) - (pSi2[0] >> 4u);

	    /* xb + xd */
	    t1 = (pSi1[0] >> 4u) + (pSi3[0] >> 4u);

	    /* ya + yc */
	    s1 = (pSi0[1] >> 4u) + (pSi2[1] >> 4u);
	    /* ya - yc */
	    s2 = (pSi0[1] >> 4u) - (pSi2[1] >> 4u);

	    /* xa' = xa + xb + xc + xd */
	    *pSi0++ = (r1 + t1);
	    /* (xa + xc) - (xb + xd) */
	    r1 = r1 - t1;
	    /* yb + yd */
	    t2 = (pSi1[1] >> 4u) + (pSi3[1] >> 4u);

	    /* ya' = ya + yb + yc + yd */
	    *pSi0++ = (s1 + t2);

	    /* (ya + yc) - (yb + yd) */
	    s1 = s1 - t2;

	    /* yb - yd */
	    t1 = (pSi1[1] >> 4u) - (pSi3[1] >> 4u);
	    /* xb - xd */
	    t2 = (pSi1[0] >> 4u) - (pSi3[0] >> 4u);

	    /*  index calculation for the coefficients */
	    ia2 = 2u * ia1;
	    co2 = pCoef[ia2 * 2u];
	    si2 = pCoef[(ia2 * 2u) + 1u];

	    /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
	    *pSi1++ = (((int32_t) (((q63_t) r1 * co2) >> 32)) +
	                     ((int32_t) (((q63_t) s1 * si2) >> 32))) << 1u;

	    /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
	    *pSi1++ = (((int32_t) (((q63_t) s1 * co2) >> 32)) -
	                            ((int32_t) (((q63_t) r1 * si2) >> 32))) << 1u;

	    /* (xa - xc) + (yb - yd) */
	    r1 = r2 + t1;
	    /* (xa - xc) - (yb - yd) */
	    r2 = r2 - t1;

	    /* (ya - yc) - (xb - xd) */
	    s1 = s2 - t2;
	    /* (ya - yc) + (xb - xd) */
	    s2 = s2 + t2;

	    co1 = pCoef[ia1 * 2u];
	    si1 = pCoef[(ia1 * 2u) + 1u];

	    /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
	    *pSi2++ = (((int32_t) (((q63_t) r1 * co1) >> 32)) +
	                     ((int32_t) (((q63_t) s1 * si1) >> 32))) << 1u;

	    /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
	    *pSi2++ = (((int32_t) (((q63_t) s1 * co1) >> 32)) -
	                            ((int32_t) (((q63_t) r1 * si1) >> 32))) << 1u;

	    /*  index calculation for the coefficients */
	    ia3 = 3u * ia1;
	    co3 = pCoef[ia3 * 2u];
	    si3 = pCoef[(ia3 * 2u) + 1u];

	    /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
	    *pSi3++ = (((int32_t) (((q63_t) r2 * co3) >> 32)) +
	                     ((int32_t) (((q63_t) s2 * si3) >> 32))) << 1u;

	    /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
	    *pSi3++ = (((int32_t) (((q63_t) s2 * co3) >> 32)) -
	                            ((int32_t) (((q63_t) r2 * si3) >> 32))) << 1u;

	    /*  Twiddle coefficients index modifier */
	    ia1 = ia1 + twidCoefModifier;

	  } while(--j);

	  /* end of first stage process */

	  /* data is in 5.27(q27) format */


	  /* start of Middle stages process */


	  /* each stage in middle stages provides two down scaling of the input */

	  twidCoefModifier <<= 2u;


	  for (k = fftLen / 4u; k > 4u; k >>= 2u)
	  {
	    /*  Initializations for the first stage */
	    n1 = n2;
	    n2 >>= 2u;
	    ia1 = 0u;

	    /*  Calculation of first stage */
	    for (j = 0u; j <= (n2 - 1u); j++)
	    {
	      /*  index calculation for the coefficients */
	      ia2 = ia1 + ia1;
	      ia3 = ia2 + ia1;
	      co1 = pCoef[ia1 * 2u];
	      si1 = pCoef[(ia1 * 2u) + 1u];
	      co2 = pCoef[ia2 * 2u];
	      si2 = pCoef[(ia2 * 2u) + 1u];
	      co3 = pCoef[ia3 * 2u];
	      si3 = pCoef[(ia3 * 2u) + 1u];
	      /*  Twiddle coefficients index modifier */
	      ia1 = ia1 + twidCoefModifier;

	      pSi0 = pSrc + 2 * j;
	      pSi1 = pSi0 + 2 * n2;
	      pSi2 = pSi1 + 2 * n2;
	      pSi3 = pSi2 + 2 * n2;

	      for (i0 = j; i0 < fftLen; i0 += n1)
	      {
	        /*  Butterfly implementation */
	        /* xa + xc */
	        r1 = pSi0[0] + pSi2[0];

	        /* xa - xc */
	        r2 = pSi0[0] - pSi2[0];


	        /* ya + yc */
	        s1 = pSi0[1] + pSi2[1];

	        /* ya - yc */
	        s2 = pSi0[1] - pSi2[1];


	        /* xb + xd */
	        t1 = pSi1[0] + pSi3[0];


	        /* xa' = xa + xb + xc + xd */
	        pSi0[0] = (r1 + t1) >> 2u;
	        /* xa + xc -(xb + xd) */
	        r1 = r1 - t1;

	        /* yb + yd */
	        t2 = pSi1[1] + pSi3[1];

	        /* ya' = ya + yb + yc + yd */
	        pSi0[1] = (s1 + t2) >> 2u;
	        pSi0 += 2 * n1;

	        /* (ya + yc) - (yb + yd) */
	        s1 = s1 - t2;

	        /* (yb - yd) */
	        t1 = pSi1[1] - pSi3[1];

	        /* (xb - xd) */
	        t2 = pSi1[0] - pSi3[0];


	        /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
	        pSi1[0] = (((int32_t) (((q63_t) r1 * co2) >> 32)) +
	                         ((int32_t) (((q63_t) s1 * si2) >> 32))) >> 1u;

	        /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
	        pSi1[1] = (((int32_t) (((q63_t) s1 * co2) >> 32)) -
	                                ((int32_t) (((q63_t) r1 * si2) >> 32))) >> 1u;
	        pSi1 += 2 * n1;

	        /* (xa - xc) + (yb - yd) */
	        r1 = r2 + t1;
	        /* (xa - xc) - (yb - yd) */
	        r2 = r2 - t1;

	        /* (ya - yc) -  (xb - xd) */
	        s1 = s2 - t2;
	        /* (ya - yc) +  (xb - xd) */
	        s2 = s2 + t2;

	        /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
	        pSi2[0] = (((int32_t) (((q63_t) r1 * co1) >> 32)) +
	                         ((int32_t) (((q63_t) s1 * si1) >> 32))) >> 1u;

	        /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
	        pSi2[1] = (((int32_t) (((q63_t) s1 * co1) >> 32)) -
	                                ((int32_t) (((q63_t) r1 * si1) >> 32))) >> 1u;
	        pSi2 += 2 * n1;

	        /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
	        pSi3[0] = (((int32_t) (((q63_t) r2 * co3) >> 32)) +
	                         ((int32_t) (((q63_t) s2 * si3) >> 32))) >> 1u;

	        /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
	        pSi3[1] = (((int32_t) (((q63_t) s2 * co3) >> 32)) -
	                                ((int32_t) (((q63_t) r2 * si3) >> 32))) >> 1u;
	        pSi3 += 2 * n1;
	      }
	    }
	    twidCoefModifier <<= 2u;
	  }
	#endif

	  /* End of Middle stages process */

	  /* data is in 11.21(q21) format for the 1024 point as there are 3 middle stages */
	  /* data is in 9.23(q23) format for the 256 point as there are 2 middle stages */
	  /* data is in 7.25(q25) format for the 64 point as there are 1 middle stage */
	  /* data is in 5.27(q27) format for the 16 point as there are no middle stages */


	  /* start of Last stage process */
	  /*  Initializations for the last stage */
	  j = fftLen >> 2;
	  ptr1 = &pSrc[0];

	  /*  Calculations of last stage */
	  do
	  {

	#ifndef ARM_MATH_BIG_ENDIAN

	    /* Read xa (real), ya(imag) input */
	    xaya = *__SIMD64(ptr1)++;
	    xa = (q31_t) xaya;
	    ya = (q31_t) (xaya >> 32);

	    /* Read xb (real), yb(imag) input */
	    xbyb = *__SIMD64(ptr1)++;
	    xb = (q31_t) xbyb;
	    yb = (q31_t) (xbyb >> 32);

	    /* Read xc (real), yc(imag) input */
	    xcyc = *__SIMD64(ptr1)++;
	    xc = (q31_t) xcyc;
	    yc = (q31_t) (xcyc >> 32);

	    /* Read xc (real), yc(imag) input */
	    xdyd = *__SIMD64(ptr1)++;
	    xd = (q31_t) xdyd;
	    yd = (q31_t) (xdyd >> 32);

	#else

	    /* Read xa (real), ya(imag) input */
	    xaya = *__SIMD64(ptr1)++;
	    ya = (q31_t) xaya;
	    xa = (q31_t) (xaya >> 32);

	    /* Read xb (real), yb(imag) input */
	    xbyb = *__SIMD64(ptr1)++;
	    yb = (q31_t) xbyb;
	    xb = (q31_t) (xbyb >> 32);

	    /* Read xc (real), yc(imag) input */
	    xcyc = *__SIMD64(ptr1)++;
	    yc = (q31_t) xcyc;
	    xc = (q31_t) (xcyc >> 32);

	    /* Read xc (real), yc(imag) input */
	    xdyd = *__SIMD64(ptr1)++;
	    yd = (q31_t) xdyd;
	    xd = (q31_t) (xdyd >> 32);


	#endif

	    /* xa' = xa + xb + xc + xd */
	    xa_out = xa + xb + xc + xd;

	    /* ya' = ya + yb + yc + yd */
	    ya_out = ya + yb + yc + yd;

	    /* pointer updation for writing */
	    ptr1 = ptr1 - 8u;

	    /* writing xa' and ya' */
	    *ptr1++ = xa_out;
	    *ptr1++ = ya_out;

	    xc_out = (xa - xb + xc - xd);
	    yc_out = (ya - yb + yc - yd);

	    /* writing xc' and yc' */
	    *ptr1++ = xc_out;
	    *ptr1++ = yc_out;

	    xb_out = (xa + yb - xc - yd);
	    yb_out = (ya - xb - yc + xd);

	    /* writing xb' and yb' */
	    *ptr1++ = xb_out;
	    *ptr1++ = yb_out;

	    xd_out = (xa - yb - xc + yd);
	    yd_out = (ya + xb - yc - xd);

	    /* writing xd' and yd' */
	    *ptr1++ = xd_out;
	    *ptr1++ = yd_out;


	  } while(--j);

	  /* output is in 11.21(q21) format for the 1024 point */
	  /* output is in 9.23(q23) format for the 256 point */
	  /* output is in 7.25(q25) format for the 64 point */
	  /* output is in 5.27(q27) format for the 16 point */

	  /* End of last stage process */

	}

	void arm_cfft_radix4_q31(
	  const arm_cfft_radix4_instance_q31 * S,
	  q31_t * pSrc)
	{
	  if(S->ifftFlag == 1u)
	  {
	    /* Complex IFFT radix-4 */
	    arm_radix4_butterfly_inverse_q31(pSrc, S->fftLen, S->pTwiddle,
	                                     S->twidCoefModifier);
	  }
	  else
	  {
	    /* Complex FFT radix-4 */
	    arm_radix4_butterfly_q31(pSrc, S->fftLen, S->pTwiddle,
	                             S->twidCoefModifier);
	  }


	  if(S->bitReverseFlag == 1u)
	  {
	    /*  Bit Reversal */
	    arm_bitreversal_q31(pSrc, S->fftLen, S->bitRevFactor, S->pBitRevTable);
	  }

	}
	void arm_cfft_radix4by2_q31(
		    q31_t * pSrc,
		    uint32_t fftLen,
		    const q31_t * pCoef)
		{
		    uint32_t i, l;
		    uint32_t n2, ia;
		    q31_t xt, yt, cosVal, sinVal;
		    q31_t p0, p1;

		    n2 = fftLen >> 1;
		    ia = 0;
		    for (i = 0; i < n2; i++)
		    {
		        cosVal = pCoef[2*ia];
		        sinVal = pCoef[2*ia + 1];
		        ia++;

		        l = i + n2;
		        xt = (pSrc[2 * i] >> 2) - (pSrc[2 * l] >> 2);
		        pSrc[2 * i] = (pSrc[2 * i] >> 2) + (pSrc[2 * l] >> 2);

		        yt = (pSrc[2 * i + 1] >> 2) - (pSrc[2 * l + 1] >> 2);
		        pSrc[2 * i + 1] = (pSrc[2 * l + 1] >> 2) + (pSrc[2 * i + 1] >> 2);

		        mult_32x32_keep32_R(p0, xt, cosVal);
		        mult_32x32_keep32_R(p1, yt, cosVal);
		        multAcc_32x32_keep32_R(p0, yt, sinVal);
		        multSub_32x32_keep32_R(p1, xt, sinVal);

		        pSrc[2u * l] = p0 << 1;
		        pSrc[2u * l + 1u] = p1 << 1;

		    }

		    // first col
		    arm_radix4_butterfly_q31( pSrc, n2, (q31_t*)pCoef, 2u);
		    // second col
		    arm_radix4_butterfly_q31( pSrc + fftLen, n2, (q31_t*)pCoef, 2u);

		    for (i = 0; i < fftLen >> 1; i++)
		    {
		        p0 = pSrc[4*i+0];
		        p1 = pSrc[4*i+1];
		        xt = pSrc[4*i+2];
		        yt = pSrc[4*i+3];

		        p0 <<= 1;
		        p1 <<= 1;
		        xt <<= 1;
		        yt <<= 1;

		        pSrc[4*i+0] = p0;
		        pSrc[4*i+1] = p1;
		        pSrc[4*i+2] = xt;
		        pSrc[4*i+3] = yt;
		    }

		}

		void arm_cfft_radix4by2_inverse_q31(
		    q31_t * pSrc,
		    uint32_t fftLen,
		    const q31_t * pCoef)
		{
		    uint32_t i, l;
		    uint32_t n2, ia;
		    q31_t xt, yt, cosVal, sinVal;
		    q31_t p0, p1;

		    n2 = fftLen >> 1;
		    ia = 0;
		    for (i = 0; i < n2; i++)
		    {
		        cosVal = pCoef[2*ia];
		        sinVal = pCoef[2*ia + 1];
		        ia++;

		        l = i + n2;
		        xt = (pSrc[2 * i] >> 2) - (pSrc[2 * l] >> 2);
		        pSrc[2 * i] = (pSrc[2 * i] >> 2) + (pSrc[2 * l] >> 2);

		        yt = (pSrc[2 * i + 1] >> 2) - (pSrc[2 * l + 1] >> 2);
		        pSrc[2 * i + 1] = (pSrc[2 * l + 1] >> 2) + (pSrc[2 * i + 1] >> 2);

		        mult_32x32_keep32_R(p0, xt, cosVal);
		        mult_32x32_keep32_R(p1, yt, cosVal);
		        multSub_32x32_keep32_R(p0, yt, sinVal);
		        multAcc_32x32_keep32_R(p1, xt, sinVal);

		        pSrc[2u * l] = p0 << 1;
		        pSrc[2u * l + 1u] = p1 << 1;

		    }

		    // first col
		    arm_radix4_butterfly_inverse_q31( pSrc, n2, (q31_t*)pCoef, 2u);
		    // second col
		    arm_radix4_butterfly_inverse_q31( pSrc + fftLen, n2, (q31_t*)pCoef, 2u);

		    for (i = 0; i < fftLen >> 1; i++)
		    {
		        p0 = pSrc[4*i+0];
		        p1 = pSrc[4*i+1];
		        xt = pSrc[4*i+2];
		        yt = pSrc[4*i+3];

		        p0 <<= 1;
		        p1 <<= 1;
		        xt <<= 1;
		        yt <<= 1;

		        pSrc[4*i+0] = p0;
		        pSrc[4*i+1] = p1;
		        pSrc[4*i+2] = xt;
		        pSrc[4*i+3] = yt;
		    }
		}

	void arm_cfft_q31(
	    const arm_cfft_instance_q31 * S,
	    q31_t * p1,
	    uint8_t ifftFlag,
	    uint8_t bitReverseFlag)
	{
	    uint32_t L = S->fftLen;

	    if(ifftFlag == 1u)
	    {
	        switch (L)
	        {
	        case 16:
	        case 64:
	        case 256:
	        case 1024:
	        case 4096:
	            arm_radix4_butterfly_inverse_q31  ( p1, L, (q31_t*)S->pTwiddle, 1 );
	            break;

	        case 32:
	        case 128:
	        case 512:
	        case 2048:
	            arm_cfft_radix4by2_inverse_q31  ( p1, L, S->pTwiddle );
	            break;
	        }
	    }
	    else
	    {
	        switch (L)
	        {
	        case 16:
	        case 64:
	        case 256:
	        case 1024:
	        case 4096:
	            arm_radix4_butterfly_q31  ( p1, L, (q31_t*)S->pTwiddle, 1 );
	            break;

	        case 32:
	        case 128:
	        case 512:
	        case 2048:
	            arm_cfft_radix4by2_q31  ( p1, L, S->pTwiddle );
	            break;
	        }
	    }

	    if( bitReverseFlag )
	        arm_bitreversal_32((uint32_t*)p1,S->bitRevLength,S->pBitRevTable);
	}

	/**
	* @} end of ComplexFFT group
	*/


	/* ----------------------------------------------------------------------
	** Internal helper function used by the FFTs
	** ------------------------------------------------------------------- */

	/*
	* @brief  Core function for the floating-point CFFT butterfly process.
	* @param[in, out] *pSrc            points to the in-place buffer of floating-point data type.
	* @param[in]      fftLen           length of the FFT.
	* @param[in]      *pCoef           points to the twiddle coefficient buffer.
	* @param[in]      twidCoefModifier twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.
	* @return none.
	*/

	void arm_radix4_butterfly_f32(
	float32_t * pSrc,
	uint16_t fftLen,
	float32_t * pCoef,
	uint16_t twidCoefModifier)
	{

	   float32_t co1, co2, co3, si1, si2, si3;
	   uint32_t ia1, ia2, ia3;
	   uint32_t i0, i1, i2, i3;
	   uint32_t n1, n2, j, k;

	#ifndef ARM_MATH_CM0_FAMILY_FAMILY

	   /* Run the below code for Cortex-M4 and Cortex-M3 */

	   float32_t xaIn, yaIn, xbIn, ybIn, xcIn, ycIn, xdIn, ydIn;
	   float32_t Xaplusc, Xbplusd, Yaplusc, Ybplusd, Xaminusc, Xbminusd, Yaminusc,
	   Ybminusd;
	   float32_t Xb12C_out, Yb12C_out, Xc12C_out, Yc12C_out, Xd12C_out, Yd12C_out;
	   float32_t Xb12_out, Yb12_out, Xc12_out, Yc12_out, Xd12_out, Yd12_out;
	   float32_t *ptr1;
	   float32_t p0,p1,p2,p3,p4,p5;
	   float32_t a0,a1,a2,a3,a4,a5,a6,a7;

	   /*  Initializations for the first stage */
	   n2 = fftLen;
	   n1 = n2;

	   /* n2 = fftLen/4 */
	   n2 >>= 2u;
	   i0 = 0u;
	   ia1 = 0u;

	   j = n2;

	   /*  Calculation of first stage */
	   do
	   {
	      /*  index calculation for the input as, */
	      /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
	      i1 = i0 + n2;
	      i2 = i1 + n2;
	      i3 = i2 + n2;

	      xaIn = pSrc[(2u * i0)];
	      yaIn = pSrc[(2u * i0) + 1u];

	      xbIn = pSrc[(2u * i1)];
	      ybIn = pSrc[(2u * i1) + 1u];

	      xcIn = pSrc[(2u * i2)];
	      ycIn = pSrc[(2u * i2) + 1u];

	      xdIn = pSrc[(2u * i3)];
	      ydIn = pSrc[(2u * i3) + 1u];

	      /* xa + xc */
	      Xaplusc = xaIn + xcIn;
	      /* xb + xd */
	      Xbplusd = xbIn + xdIn;
	      /* ya + yc */
	      Yaplusc = yaIn + ycIn;
	      /* yb + yd */
	      Ybplusd = ybIn + ydIn;

	      /*  index calculation for the coefficients */
	      ia2 = ia1 + ia1;
	      co2 = pCoef[ia2 * 2u];
	      si2 = pCoef[(ia2 * 2u) + 1u];

	      /* xa - xc */
	      Xaminusc = xaIn - xcIn;
	      /* xb - xd */
	      Xbminusd = xbIn - xdIn;
	      /* ya - yc */
	      Yaminusc = yaIn - ycIn;
	      /* yb - yd */
	      Ybminusd = ybIn - ydIn;

	      /* xa' = xa + xb + xc + xd */
	      pSrc[(2u * i0)] = Xaplusc + Xbplusd;
	      /* ya' = ya + yb + yc + yd */
	      pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

	      /* (xa - xc) + (yb - yd) */
	      Xb12C_out = (Xaminusc + Ybminusd);
	      /* (ya - yc) + (xb - xd) */
	      Yb12C_out = (Yaminusc - Xbminusd);
	      /* (xa + xc) - (xb + xd) */
	      Xc12C_out = (Xaplusc - Xbplusd);
	      /* (ya + yc) - (yb + yd) */
	      Yc12C_out = (Yaplusc - Ybplusd);
	      /* (xa - xc) - (yb - yd) */
	      Xd12C_out = (Xaminusc - Ybminusd);
	      /* (ya - yc) + (xb - xd) */
	      Yd12C_out = (Xbminusd + Yaminusc);

	      co1 = pCoef[ia1 * 2u];
	      si1 = pCoef[(ia1 * 2u) + 1u];

	      /*  index calculation for the coefficients */
	      ia3 = ia2 + ia1;
	      co3 = pCoef[ia3 * 2u];
	      si3 = pCoef[(ia3 * 2u) + 1u];

	      Xb12_out = Xb12C_out * co1;
	      Yb12_out = Yb12C_out * co1;
	      Xc12_out = Xc12C_out * co2;
	      Yc12_out = Yc12C_out * co2;
	      Xd12_out = Xd12C_out * co3;
	      Yd12_out = Yd12C_out * co3;

	      /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	      //Xb12_out -= Yb12C_out * si1;
	      p0 = Yb12C_out * si1;
	      /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	      //Yb12_out += Xb12C_out * si1;
	      p1 = Xb12C_out * si1;
	      /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	      //Xc12_out -= Yc12C_out * si2;
	      p2 = Yc12C_out * si2;
	      /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	      //Yc12_out += Xc12C_out * si2;
	      p3 = Xc12C_out * si2;
	      /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	      //Xd12_out -= Yd12C_out * si3;
	      p4 = Yd12C_out * si3;
	      /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	      //Yd12_out += Xd12C_out * si3;
	      p5 = Xd12C_out * si3;

	      Xb12_out += p0;
	      Yb12_out -= p1;
	      Xc12_out += p2;
	      Yc12_out -= p3;
	      Xd12_out += p4;
	      Yd12_out -= p5;

	      /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
	      pSrc[2u * i1] = Xc12_out;

	      /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
	      pSrc[(2u * i1) + 1u] = Yc12_out;

	      /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
	      pSrc[2u * i2] = Xb12_out;

	      /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
	      pSrc[(2u * i2) + 1u] = Yb12_out;

	      /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
	      pSrc[2u * i3] = Xd12_out;

	      /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
	      pSrc[(2u * i3) + 1u] = Yd12_out;

	      /*  Twiddle coefficients index modifier */
	      ia1 += twidCoefModifier;

	      /*  Updating input index */
	      i0++;

	   }
	   while(--j);

	   twidCoefModifier <<= 2u;

	   /*  Calculation of second stage to excluding last stage */
	   for (k = fftLen >> 2u; k > 4u; k >>= 2u)
	   {
	      /*  Initializations for the first stage */
	      n1 = n2;
	      n2 >>= 2u;
	      ia1 = 0u;

	      /*  Calculation of first stage */
	      j = 0;
	      do
	      {
	         /*  index calculation for the coefficients */
	         ia2 = ia1 + ia1;
	         ia3 = ia2 + ia1;
	         co1 = pCoef[ia1 * 2u];
	         si1 = pCoef[(ia1 * 2u) + 1u];
	         co2 = pCoef[ia2 * 2u];
	         si2 = pCoef[(ia2 * 2u) + 1u];
	         co3 = pCoef[ia3 * 2u];
	         si3 = pCoef[(ia3 * 2u) + 1u];

	         /*  Twiddle coefficients index modifier */
	         ia1 += twidCoefModifier;

	         i0 = j;
	         do
	         {
	            /*  index calculation for the input as, */
	            /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
	            i1 = i0 + n2;
	            i2 = i1 + n2;
	            i3 = i2 + n2;

	            xaIn = pSrc[(2u * i0)];
	            yaIn = pSrc[(2u * i0) + 1u];

	            xbIn = pSrc[(2u * i1)];
	            ybIn = pSrc[(2u * i1) + 1u];

	            xcIn = pSrc[(2u * i2)];
	            ycIn = pSrc[(2u * i2) + 1u];

	            xdIn = pSrc[(2u * i3)];
	            ydIn = pSrc[(2u * i3) + 1u];

	            /* xa - xc */
	            Xaminusc = xaIn - xcIn;
	            /* (xb - xd) */
	            Xbminusd = xbIn - xdIn;
	            /* ya - yc */
	            Yaminusc = yaIn - ycIn;
	            /* (yb - yd) */
	            Ybminusd = ybIn - ydIn;

	            /* xa + xc */
	            Xaplusc = xaIn + xcIn;
	            /* xb + xd */
	            Xbplusd = xbIn + xdIn;
	            /* ya + yc */
	            Yaplusc = yaIn + ycIn;
	            /* yb + yd */
	            Ybplusd = ybIn + ydIn;

	            /* (xa - xc) + (yb - yd) */
	            Xb12C_out = (Xaminusc + Ybminusd);
	            /* (ya - yc) -  (xb - xd) */
	            Yb12C_out = (Yaminusc - Xbminusd);
	            /* xa + xc -(xb + xd) */
	            Xc12C_out = (Xaplusc - Xbplusd);
	            /* (ya + yc) - (yb + yd) */
	            Yc12C_out = (Yaplusc - Ybplusd);
	            /* (xa - xc) - (yb - yd) */
	            Xd12C_out = (Xaminusc - Ybminusd);
	            /* (ya - yc) +  (xb - xd) */
	            Yd12C_out = (Xbminusd + Yaminusc);

	            pSrc[(2u * i0)] = Xaplusc + Xbplusd;
	            pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

	            Xb12_out = Xb12C_out * co1;
	            Yb12_out = Yb12C_out * co1;
	            Xc12_out = Xc12C_out * co2;
	            Yc12_out = Yc12C_out * co2;
	            Xd12_out = Xd12C_out * co3;
	            Yd12_out = Yd12C_out * co3;

	            /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	            //Xb12_out -= Yb12C_out * si1;
	            p0 = Yb12C_out * si1;
	            /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	            //Yb12_out += Xb12C_out * si1;
	            p1 = Xb12C_out * si1;
	            /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	            //Xc12_out -= Yc12C_out * si2;
	            p2 = Yc12C_out * si2;
	            /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	            //Yc12_out += Xc12C_out * si2;
	            p3 = Xc12C_out * si2;
	            /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	            //Xd12_out -= Yd12C_out * si3;
	            p4 = Yd12C_out * si3;
	            /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	            //Yd12_out += Xd12C_out * si3;
	            p5 = Xd12C_out * si3;

	            Xb12_out += p0;
	            Yb12_out -= p1;
	            Xc12_out += p2;
	            Yc12_out -= p3;
	            Xd12_out += p4;
	            Yd12_out -= p5;

	            /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
	            pSrc[2u * i1] = Xc12_out;

	            /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
	            pSrc[(2u * i1) + 1u] = Yc12_out;

	            /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
	            pSrc[2u * i2] = Xb12_out;

	            /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
	            pSrc[(2u * i2) + 1u] = Yb12_out;

	            /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
	            pSrc[2u * i3] = Xd12_out;

	            /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
	            pSrc[(2u * i3) + 1u] = Yd12_out;

	            i0 += n1;
	         } while(i0 < fftLen);
	         j++;
	      } while(j <= (n2 - 1u));
	      twidCoefModifier <<= 2u;
	   }

	   j = fftLen >> 2;
	   ptr1 = &pSrc[0];

	   /*  Calculations of last stage */
	   do
	   {
	      xaIn = ptr1[0];
	      yaIn = ptr1[1];
	      xbIn = ptr1[2];
	      ybIn = ptr1[3];
	      xcIn = ptr1[4];
	      ycIn = ptr1[5];
	      xdIn = ptr1[6];
	      ydIn = ptr1[7];

	      /* xa + xc */
	      Xaplusc = xaIn + xcIn;

	      /* xa - xc */
	      Xaminusc = xaIn - xcIn;

	      /* ya + yc */
	      Yaplusc = yaIn + ycIn;

	      /* ya - yc */
	      Yaminusc = yaIn - ycIn;

	      /* xb + xd */
	      Xbplusd = xbIn + xdIn;

	      /* yb + yd */
	      Ybplusd = ybIn + ydIn;

	      /* (xb-xd) */
	      Xbminusd = xbIn - xdIn;

	      /* (yb-yd) */
	      Ybminusd = ybIn - ydIn;

	      /* xa' = xa + xb + xc + xd */
	      a0 = (Xaplusc + Xbplusd);
	      /* ya' = ya + yb + yc + yd */
	      a1 = (Yaplusc + Ybplusd);
	      /* xc' = (xa-xb+xc-xd) */
	      a2 = (Xaplusc - Xbplusd);
	      /* yc' = (ya-yb+yc-yd) */
	      a3 = (Yaplusc - Ybplusd);
	      /* xb' = (xa+yb-xc-yd) */
	      a4 = (Xaminusc + Ybminusd);
	      /* yb' = (ya-xb-yc+xd) */
	      a5 = (Yaminusc - Xbminusd);
	      /* xd' = (xa-yb-xc+yd)) */
	      a6 = (Xaminusc - Ybminusd);
	      /* yd' = (ya+xb-yc-xd) */
	      a7 = (Xbminusd + Yaminusc);

	      ptr1[0] = a0;
	      ptr1[1] = a1;
	      ptr1[2] = a2;
	      ptr1[3] = a3;
	      ptr1[4] = a4;
	      ptr1[5] = a5;
	      ptr1[6] = a6;
	      ptr1[7] = a7;

	      /* increment pointer by 8 */
	      ptr1 += 8u;
	   } while(--j);

	#else

	   float32_t t1, t2, r1, r2, s1, s2;

	   /* Run the below code for Cortex-M0 */

	   /*  Initializations for the fft calculation */
	   n2 = fftLen;
	   n1 = n2;
	   for (k = fftLen; k > 1u; k >>= 2u)
	   {
	      /*  Initializations for the fft calculation */
	      n1 = n2;
	      n2 >>= 2u;
	      ia1 = 0u;

	      /*  FFT Calculation */
	      j = 0;
	      do
	      {
	         /*  index calculation for the coefficients */
	         ia2 = ia1 + ia1;
	         ia3 = ia2 + ia1;
	         co1 = pCoef[ia1 * 2u];
	         si1 = pCoef[(ia1 * 2u) + 1u];
	         co2 = pCoef[ia2 * 2u];
	         si2 = pCoef[(ia2 * 2u) + 1u];
	         co3 = pCoef[ia3 * 2u];
	         si3 = pCoef[(ia3 * 2u) + 1u];

	         /*  Twiddle coefficients index modifier */
	         ia1 = ia1 + twidCoefModifier;

	         i0 = j;
	         do
	         {
	            /*  index calculation for the input as, */
	            /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
	            i1 = i0 + n2;
	            i2 = i1 + n2;
	            i3 = i2 + n2;

	            /* xa + xc */
	            r1 = pSrc[(2u * i0)] + pSrc[(2u * i2)];

	            /* xa - xc */
	            r2 = pSrc[(2u * i0)] - pSrc[(2u * i2)];

	            /* ya + yc */
	            s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];

	            /* ya - yc */
	            s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

	            /* xb + xd */
	            t1 = pSrc[2u * i1] + pSrc[2u * i3];

	            /* xa' = xa + xb + xc + xd */
	            pSrc[2u * i0] = r1 + t1;

	            /* xa + xc -(xb + xd) */
	            r1 = r1 - t1;

	            /* yb + yd */
	            t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];

	            /* ya' = ya + yb + yc + yd */
	            pSrc[(2u * i0) + 1u] = s1 + t2;

	            /* (ya + yc) - (yb + yd) */
	            s1 = s1 - t2;

	            /* (yb - yd) */
	            t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];

	            /* (xb - xd) */
	            t2 = pSrc[2u * i1] - pSrc[2u * i3];

	            /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
	            pSrc[2u * i1] = (r1 * co2) + (s1 * si2);

	            /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
	            pSrc[(2u * i1) + 1u] = (s1 * co2) - (r1 * si2);

	            /* (xa - xc) + (yb - yd) */
	            r1 = r2 + t1;

	            /* (xa - xc) - (yb - yd) */
	            r2 = r2 - t1;

	            /* (ya - yc) -  (xb - xd) */
	            s1 = s2 - t2;

	            /* (ya - yc) +  (xb - xd) */
	            s2 = s2 + t2;

	            /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
	            pSrc[2u * i2] = (r1 * co1) + (s1 * si1);

	            /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
	            pSrc[(2u * i2) + 1u] = (s1 * co1) - (r1 * si1);

	            /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
	            pSrc[2u * i3] = (r2 * co3) + (s2 * si3);

	            /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
	            pSrc[(2u * i3) + 1u] = (s2 * co3) - (r2 * si3);

	            i0 += n1;
	         } while( i0 < fftLen);
	         j++;
	      } while(j <= (n2 - 1u));
	      twidCoefModifier <<= 2u;
	   }

	#endif /* #ifndef ARM_MATH_CM0_FAMILY_FAMILY */

	}

	/*
	* @brief  Core function for the floating-point CIFFT butterfly process.
	* @param[in, out] *pSrc            points to the in-place buffer of floating-point data type.
	* @param[in]      fftLen           length of the FFT.
	* @param[in]      *pCoef           points to twiddle coefficient buffer.
	* @param[in]      twidCoefModifier twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.
	* @param[in]      onebyfftLen      value of 1/fftLen.
	* @return none.
	*/

	void arm_radix4_butterfly_inverse_f32(
	float32_t * pSrc,
	uint16_t fftLen,
	float32_t * pCoef,
	uint16_t twidCoefModifier,
	float32_t onebyfftLen)
	{
	   float32_t co1, co2, co3, si1, si2, si3;
	   uint32_t ia1, ia2, ia3;
	   uint32_t i0, i1, i2, i3;
	   uint32_t n1, n2, j, k;

	#ifndef ARM_MATH_CM0_FAMILY_FAMILY

	   float32_t xaIn, yaIn, xbIn, ybIn, xcIn, ycIn, xdIn, ydIn;
	   float32_t Xaplusc, Xbplusd, Yaplusc, Ybplusd, Xaminusc, Xbminusd, Yaminusc,
	   Ybminusd;
	   float32_t Xb12C_out, Yb12C_out, Xc12C_out, Yc12C_out, Xd12C_out, Yd12C_out;
	   float32_t Xb12_out, Yb12_out, Xc12_out, Yc12_out, Xd12_out, Yd12_out;
	   float32_t *ptr1;
	   float32_t p0,p1,p2,p3,p4,p5,p6,p7;
	   float32_t a0,a1,a2,a3,a4,a5,a6,a7;


	   /*  Initializations for the first stage */
	   n2 = fftLen;
	   n1 = n2;

	   /* n2 = fftLen/4 */
	   n2 >>= 2u;
	   i0 = 0u;
	   ia1 = 0u;

	   j = n2;

	   /*  Calculation of first stage */
	   do
	   {
	      /*  index calculation for the input as, */
	      /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
	      i1 = i0 + n2;
	      i2 = i1 + n2;
	      i3 = i2 + n2;

	      /*  Butterfly implementation */
	      xaIn = pSrc[(2u * i0)];
	      yaIn = pSrc[(2u * i0) + 1u];

	      xcIn = pSrc[(2u * i2)];
	      ycIn = pSrc[(2u * i2) + 1u];

	      xbIn = pSrc[(2u * i1)];
	      ybIn = pSrc[(2u * i1) + 1u];

	      xdIn = pSrc[(2u * i3)];
	      ydIn = pSrc[(2u * i3) + 1u];

	      /* xa + xc */
	      Xaplusc = xaIn + xcIn;
	      /* xb + xd */
	      Xbplusd = xbIn + xdIn;
	      /* ya + yc */
	      Yaplusc = yaIn + ycIn;
	      /* yb + yd */
	      Ybplusd = ybIn + ydIn;

	      /*  index calculation for the coefficients */
	      ia2 = ia1 + ia1;
	      co2 = pCoef[ia2 * 2u];
	      si2 = pCoef[(ia2 * 2u) + 1u];

	      /* xa - xc */
	      Xaminusc = xaIn - xcIn;
	      /* xb - xd */
	      Xbminusd = xbIn - xdIn;
	      /* ya - yc */
	      Yaminusc = yaIn - ycIn;
	      /* yb - yd */
	      Ybminusd = ybIn - ydIn;

	      /* xa' = xa + xb + xc + xd */
	      pSrc[(2u * i0)] = Xaplusc + Xbplusd;

	      /* ya' = ya + yb + yc + yd */
	      pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

	      /* (xa - xc) - (yb - yd) */
	      Xb12C_out = (Xaminusc - Ybminusd);
	      /* (ya - yc) + (xb - xd) */
	      Yb12C_out = (Yaminusc + Xbminusd);
	      /* (xa + xc) - (xb + xd) */
	      Xc12C_out = (Xaplusc - Xbplusd);
	      /* (ya + yc) - (yb + yd) */
	      Yc12C_out = (Yaplusc - Ybplusd);
	      /* (xa - xc) + (yb - yd) */
	      Xd12C_out = (Xaminusc + Ybminusd);
	      /* (ya - yc) - (xb - xd) */
	      Yd12C_out = (Yaminusc - Xbminusd);

	      co1 = pCoef[ia1 * 2u];
	      si1 = pCoef[(ia1 * 2u) + 1u];

	      /*  index calculation for the coefficients */
	      ia3 = ia2 + ia1;
	      co3 = pCoef[ia3 * 2u];
	      si3 = pCoef[(ia3 * 2u) + 1u];

	      Xb12_out = Xb12C_out * co1;
	      Yb12_out = Yb12C_out * co1;
	      Xc12_out = Xc12C_out * co2;
	      Yc12_out = Yc12C_out * co2;
	      Xd12_out = Xd12C_out * co3;
	      Yd12_out = Yd12C_out * co3;

	      /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	      //Xb12_out -= Yb12C_out * si1;
	      p0 = Yb12C_out * si1;
	      /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	      //Yb12_out += Xb12C_out * si1;
	      p1 = Xb12C_out * si1;
	      /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	      //Xc12_out -= Yc12C_out * si2;
	      p2 = Yc12C_out * si2;
	      /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	      //Yc12_out += Xc12C_out * si2;
	      p3 = Xc12C_out * si2;
	      /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	      //Xd12_out -= Yd12C_out * si3;
	      p4 = Yd12C_out * si3;
	      /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	      //Yd12_out += Xd12C_out * si3;
	      p5 = Xd12C_out * si3;

	      Xb12_out -= p0;
	      Yb12_out += p1;
	      Xc12_out -= p2;
	      Yc12_out += p3;
	      Xd12_out -= p4;
	      Yd12_out += p5;

	      /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	      pSrc[2u * i1] = Xc12_out;

	      /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	      pSrc[(2u * i1) + 1u] = Yc12_out;

	      /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	      pSrc[2u * i2] = Xb12_out;

	      /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	      pSrc[(2u * i2) + 1u] = Yb12_out;

	      /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	      pSrc[2u * i3] = Xd12_out;

	      /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	      pSrc[(2u * i3) + 1u] = Yd12_out;

	      /*  Twiddle coefficients index modifier */
	      ia1 = ia1 + twidCoefModifier;

	      /*  Updating input index */
	      i0 = i0 + 1u;

	   } while(--j);

	   twidCoefModifier <<= 2u;

	   /*  Calculation of second stage to excluding last stage */
	   for (k = fftLen >> 2u; k > 4u; k >>= 2u)
	   {
	      /*  Initializations for the first stage */
	      n1 = n2;
	      n2 >>= 2u;
	      ia1 = 0u;

	      /*  Calculation of first stage */
	      j = 0;
	      do
	      {
	         /*  index calculation for the coefficients */
	         ia2 = ia1 + ia1;
	         ia3 = ia2 + ia1;
	         co1 = pCoef[ia1 * 2u];
	         si1 = pCoef[(ia1 * 2u) + 1u];
	         co2 = pCoef[ia2 * 2u];
	         si2 = pCoef[(ia2 * 2u) + 1u];
	         co3 = pCoef[ia3 * 2u];
	         si3 = pCoef[(ia3 * 2u) + 1u];

	         /*  Twiddle coefficients index modifier */
	         ia1 = ia1 + twidCoefModifier;

	         i0 = j;
	         do
	         {
	            /*  index calculation for the input as, */
	            /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
	            i1 = i0 + n2;
	            i2 = i1 + n2;
	            i3 = i2 + n2;

	            xaIn = pSrc[(2u * i0)];
	            yaIn = pSrc[(2u * i0) + 1u];

	            xbIn = pSrc[(2u * i1)];
	            ybIn = pSrc[(2u * i1) + 1u];

	            xcIn = pSrc[(2u * i2)];
	            ycIn = pSrc[(2u * i2) + 1u];

	            xdIn = pSrc[(2u * i3)];
	            ydIn = pSrc[(2u * i3) + 1u];

	            /* xa - xc */
	            Xaminusc = xaIn - xcIn;
	            /* (xb - xd) */
	            Xbminusd = xbIn - xdIn;
	            /* ya - yc */
	            Yaminusc = yaIn - ycIn;
	            /* (yb - yd) */
	            Ybminusd = ybIn - ydIn;

	            /* xa + xc */
	            Xaplusc = xaIn + xcIn;
	            /* xb + xd */
	            Xbplusd = xbIn + xdIn;
	            /* ya + yc */
	            Yaplusc = yaIn + ycIn;
	            /* yb + yd */
	            Ybplusd = ybIn + ydIn;

	            /* (xa - xc) - (yb - yd) */
	            Xb12C_out = (Xaminusc - Ybminusd);
	            /* (ya - yc) +  (xb - xd) */
	            Yb12C_out = (Yaminusc + Xbminusd);
	            /* xa + xc -(xb + xd) */
	            Xc12C_out = (Xaplusc - Xbplusd);
	            /* (ya + yc) - (yb + yd) */
	            Yc12C_out = (Yaplusc - Ybplusd);
	            /* (xa - xc) + (yb - yd) */
	            Xd12C_out = (Xaminusc + Ybminusd);
	            /* (ya - yc) -  (xb - xd) */
	            Yd12C_out = (Yaminusc - Xbminusd);

	            pSrc[(2u * i0)] = Xaplusc + Xbplusd;
	            pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

	            Xb12_out = Xb12C_out * co1;
	            Yb12_out = Yb12C_out * co1;
	            Xc12_out = Xc12C_out * co2;
	            Yc12_out = Yc12C_out * co2;
	            Xd12_out = Xd12C_out * co3;
	            Yd12_out = Yd12C_out * co3;

	            /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	            //Xb12_out -= Yb12C_out * si1;
	            p0 = Yb12C_out * si1;
	            /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	            //Yb12_out += Xb12C_out * si1;
	            p1 = Xb12C_out * si1;
	            /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	            //Xc12_out -= Yc12C_out * si2;
	            p2 = Yc12C_out * si2;
	            /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	            //Yc12_out += Xc12C_out * si2;
	            p3 = Xc12C_out * si2;
	            /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	            //Xd12_out -= Yd12C_out * si3;
	            p4 = Yd12C_out * si3;
	            /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	            //Yd12_out += Xd12C_out * si3;
	            p5 = Xd12C_out * si3;

	            Xb12_out -= p0;
	            Yb12_out += p1;
	            Xc12_out -= p2;
	            Yc12_out += p3;
	            Xd12_out -= p4;
	            Yd12_out += p5;

	            /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	            pSrc[2u * i1] = Xc12_out;

	            /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	            pSrc[(2u * i1) + 1u] = Yc12_out;

	            /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	            pSrc[2u * i2] = Xb12_out;

	            /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	            pSrc[(2u * i2) + 1u] = Yb12_out;

	            /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	            pSrc[2u * i3] = Xd12_out;

	            /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	            pSrc[(2u * i3) + 1u] = Yd12_out;

	            i0 += n1;
	         } while(i0 < fftLen);
	         j++;
	      } while(j <= (n2 - 1u));
	      twidCoefModifier <<= 2u;
	   }
	   /*  Initializations of last stage */

	   j = fftLen >> 2;
	   ptr1 = &pSrc[0];

	   /*  Calculations of last stage */
	   do
	   {
	      xaIn = ptr1[0];
	      yaIn = ptr1[1];
	      xbIn = ptr1[2];
	      ybIn = ptr1[3];
	      xcIn = ptr1[4];
	      ycIn = ptr1[5];
	      xdIn = ptr1[6];
	      ydIn = ptr1[7];

	      /*  Butterfly implementation */
	      /* xa + xc */
	      Xaplusc = xaIn + xcIn;

	      /* xa - xc */
	      Xaminusc = xaIn - xcIn;

	      /* ya + yc */
	      Yaplusc = yaIn + ycIn;

	      /* ya - yc */
	      Yaminusc = yaIn - ycIn;

	      /* xb + xd */
	      Xbplusd = xbIn + xdIn;

	      /* yb + yd */
	      Ybplusd = ybIn + ydIn;

	      /* (xb-xd) */
	      Xbminusd = xbIn - xdIn;

	      /* (yb-yd) */
	      Ybminusd = ybIn - ydIn;

	      /* xa' = (xa+xb+xc+xd) * onebyfftLen */
	      a0 = (Xaplusc + Xbplusd);
	      /* ya' = (ya+yb+yc+yd) * onebyfftLen */
	      a1 = (Yaplusc + Ybplusd);
	      /* xc' = (xa-xb+xc-xd) * onebyfftLen */
	      a2 = (Xaplusc - Xbplusd);
	      /* yc' = (ya-yb+yc-yd) * onebyfftLen  */
	      a3 = (Yaplusc - Ybplusd);
	      /* xb' = (xa-yb-xc+yd) * onebyfftLen */
	      a4 = (Xaminusc - Ybminusd);
	      /* yb' = (ya+xb-yc-xd) * onebyfftLen */
	      a5 = (Yaminusc + Xbminusd);
	      /* xd' = (xa-yb-xc+yd) * onebyfftLen */
	      a6 = (Xaminusc + Ybminusd);
	      /* yd' = (ya-xb-yc+xd) * onebyfftLen */
	      a7 = (Yaminusc - Xbminusd);

	      p0 = a0 * onebyfftLen;
	      p1 = a1 * onebyfftLen;
	      p2 = a2 * onebyfftLen;
	      p3 = a3 * onebyfftLen;
	      p4 = a4 * onebyfftLen;
	      p5 = a5 * onebyfftLen;
	      p6 = a6 * onebyfftLen;
	      p7 = a7 * onebyfftLen;

	      /* xa' = (xa+xb+xc+xd) * onebyfftLen */
	      ptr1[0] = p0;
	      /* ya' = (ya+yb+yc+yd) * onebyfftLen */
	      ptr1[1] = p1;
	      /* xc' = (xa-xb+xc-xd) * onebyfftLen */
	      ptr1[2] = p2;
	      /* yc' = (ya-yb+yc-yd) * onebyfftLen  */
	      ptr1[3] = p3;
	      /* xb' = (xa-yb-xc+yd) * onebyfftLen */
	      ptr1[4] = p4;
	      /* yb' = (ya+xb-yc-xd) * onebyfftLen */
	      ptr1[5] = p5;
	      /* xd' = (xa-yb-xc+yd) * onebyfftLen */
	      ptr1[6] = p6;
	      /* yd' = (ya-xb-yc+xd) * onebyfftLen */
	      ptr1[7] = p7;

	      /* increment source pointer by 8 for next calculations */
	      ptr1 = ptr1 + 8u;

	   } while(--j);

	#else

	   float32_t t1, t2, r1, r2, s1, s2;

	   /* Run the below code for Cortex-M0 */

	   /*  Initializations for the first stage */
	   n2 = fftLen;
	   n1 = n2;

	   /*  Calculation of first stage */
	   for (k = fftLen; k > 4u; k >>= 2u)
	   {
	      /*  Initializations for the first stage */
	      n1 = n2;
	      n2 >>= 2u;
	      ia1 = 0u;

	      /*  Calculation of first stage */
	      j = 0;
	      do
	      {
	         /*  index calculation for the coefficients */
	         ia2 = ia1 + ia1;
	         ia3 = ia2 + ia1;
	         co1 = pCoef[ia1 * 2u];
	         si1 = pCoef[(ia1 * 2u) + 1u];
	         co2 = pCoef[ia2 * 2u];
	         si2 = pCoef[(ia2 * 2u) + 1u];
	         co3 = pCoef[ia3 * 2u];
	         si3 = pCoef[(ia3 * 2u) + 1u];

	         /*  Twiddle coefficients index modifier */
	         ia1 = ia1 + twidCoefModifier;

	         i0 = j;
	         do
	         {
	            /*  index calculation for the input as, */
	            /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
	            i1 = i0 + n2;
	            i2 = i1 + n2;
	            i3 = i2 + n2;

	            /* xa + xc */
	            r1 = pSrc[(2u * i0)] + pSrc[(2u * i2)];

	            /* xa - xc */
	            r2 = pSrc[(2u * i0)] - pSrc[(2u * i2)];

	            /* ya + yc */
	            s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];

	            /* ya - yc */
	            s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

	            /* xb + xd */
	            t1 = pSrc[2u * i1] + pSrc[2u * i3];

	            /* xa' = xa + xb + xc + xd */
	            pSrc[2u * i0] = r1 + t1;

	            /* xa + xc -(xb + xd) */
	            r1 = r1 - t1;

	            /* yb + yd */
	            t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];

	            /* ya' = ya + yb + yc + yd */
	            pSrc[(2u * i0) + 1u] = s1 + t2;

	            /* (ya + yc) - (yb + yd) */
	            s1 = s1 - t2;

	            /* (yb - yd) */
	            t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];

	            /* (xb - xd) */
	            t2 = pSrc[2u * i1] - pSrc[2u * i3];

	            /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	            pSrc[2u * i1] = (r1 * co2) - (s1 * si2);

	            /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	            pSrc[(2u * i1) + 1u] = (s1 * co2) + (r1 * si2);

	            /* (xa - xc) - (yb - yd) */
	            r1 = r2 - t1;

	            /* (xa - xc) + (yb - yd) */
	            r2 = r2 + t1;

	            /* (ya - yc) +  (xb - xd) */
	            s1 = s2 + t2;

	            /* (ya - yc) -  (xb - xd) */
	            s2 = s2 - t2;

	            /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	            pSrc[2u * i2] = (r1 * co1) - (s1 * si1);

	            /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	            pSrc[(2u * i2) + 1u] = (s1 * co1) + (r1 * si1);

	            /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	            pSrc[2u * i3] = (r2 * co3) - (s2 * si3);

	            /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	            pSrc[(2u * i3) + 1u] = (s2 * co3) + (r2 * si3);

	            i0 += n1;
	         } while( i0 < fftLen);
	         j++;
	      } while(j <= (n2 - 1u));
	      twidCoefModifier <<= 2u;
	   }
	   /*  Initializations of last stage */
	   n1 = n2;
	   n2 >>= 2u;

	   /*  Calculations of last stage */
	   for (i0 = 0u; i0 <= (fftLen - n1); i0 += n1)
	   {
	      /*  index calculation for the input as, */
	      /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
	      i1 = i0 + n2;
	      i2 = i1 + n2;
	      i3 = i2 + n2;

	      /*  Butterfly implementation */
	      /* xa + xc */
	      r1 = pSrc[2u * i0] + pSrc[2u * i2];

	      /* xa - xc */
	      r2 = pSrc[2u * i0] - pSrc[2u * i2];

	      /* ya + yc */
	      s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];

	      /* ya - yc */
	      s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

	      /* xc + xd */
	      t1 = pSrc[2u * i1] + pSrc[2u * i3];

	      /* xa' = xa + xb + xc + xd */
	      pSrc[2u * i0] = (r1 + t1) * onebyfftLen;

	      /* (xa + xb) - (xc + xd) */
	      r1 = r1 - t1;

	      /* yb + yd */
	      t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];

	      /* ya' = ya + yb + yc + yd */
	      pSrc[(2u * i0) + 1u] = (s1 + t2) * onebyfftLen;

	      /* (ya + yc) - (yb + yd) */
	      s1 = s1 - t2;

	      /* (yb-yd) */
	      t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];

	      /* (xb-xd) */
	      t2 = pSrc[2u * i1] - pSrc[2u * i3];

	      /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
	      pSrc[2u * i1] = r1 * onebyfftLen;

	      /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
	      pSrc[(2u * i1) + 1u] = s1 * onebyfftLen;

	      /* (xa - xc) - (yb-yd) */
	      r1 = r2 - t1;

	      /* (xa - xc) + (yb-yd) */
	      r2 = r2 + t1;

	      /* (ya - yc) + (xb-xd) */
	      s1 = s2 + t2;

	      /* (ya - yc) - (xb-xd) */
	      s2 = s2 - t2;

	      /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
	      pSrc[2u * i2] = r1 * onebyfftLen;

	      /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
	      pSrc[(2u * i2) + 1u] = s1 * onebyfftLen;

	      /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
	      pSrc[2u * i3] = r2 * onebyfftLen;

	      /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
	      pSrc[(2u * i3) + 1u] = s2 * onebyfftLen;
	   }

	#endif /* #ifndef ARM_MATH_CM0_FAMILY_FAMILY */
	}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif




