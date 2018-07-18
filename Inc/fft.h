/*
 * fft.h
 *
 *  Created on: 2 Ïêô 2017
 *      Author: Thanos
 */

#ifndef FFT_H_
#define FFT_H_

//#define TEST_LENGTH_SAMPLES 2048

#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include "Macro_Definitions.h"


void arm_radix4_butterfly_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    q31_t * pCoef,
    uint32_t twidCoefModifier);

void arm_radix4_butterfly_inverse_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    q31_t * pCoef,
    uint32_t twidCoefModifier);

extern void arm_bitreversal_32(
    uint32_t * pSrc,
    const uint16_t bitRevLen,
    const uint16_t * pBitRevTable);

void arm_cfft_radix4by2_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    const q31_t * pCoef);

void arm_cfft_radix4by2_inverse_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    const q31_t * pCoef);

void arm_max_f32(
	  float32_t * pSrc,
	  uint32_t blockSize,
	  float32_t * pResult,
	  uint32_t * pIndex);

void arm_cmplx_mag_f32(
	  float32_t * pSrc,
	  float32_t * pDst,
	  uint32_t numSamples);

void arm_bitreversal_f32(
	float32_t * pSrc,
	uint16_t fftSize,
	uint16_t bitRevFactor,
	uint16_t * pBitRevTab);

void arm_radix8_butterfly_f32(
	float32_t * pSrc,
	uint16_t fftLen,
	const float32_t * pCoef,
	uint16_t twidCoefModifier);

void arm_cfft_radix8by2_f32( arm_cfft_instance_f32 * S, float32_t * p1);

void arm_cfft_radix8by4_f32( arm_cfft_instance_f32 * S, float32_t * p1);

void arm_cfft_f32(
	    const arm_cfft_instance_f32 * S,
	    float32_t * p1,
	    uint8_t ifftFlag,
	    uint8_t bitReverseFlag);

void arm_rfft_fast_f32(
	arm_rfft_fast_instance_f32 * S,
	float32_t * p, float32_t * pOut,
	uint8_t ifftFlag);

void merge_rfft_f32(
arm_rfft_fast_instance_f32 * S,
float32_t * p, float32_t * pOut);

void stage_rfft_f32(
  arm_rfft_fast_instance_f32 * S,
  float32_t * p, float32_t * pOut);

void arm_rfft_f32(
  const arm_rfft_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst);

void arm_split_rfft_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pATable,
  float32_t * pBTable,
  float32_t * pDst,
  uint32_t modifier);

void arm_split_rifft_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pATable,
  float32_t * pBTable,
  float32_t * pDst,
  uint32_t modifier);

void arm_radix4_butterfly_inverse_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier,
float32_t onebyfftLen);

void arm_radix4_butterfly_f32(
float32_t * pSrc,
uint16_t fftLen,
float32_t * pCoef,
uint16_t twidCoefModifier);



#endif /* FFT_H_ */
