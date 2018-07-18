/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

#include "math.h"
#include "arm_math.h"
#include "fft.h"
#include "Macro_Definitions.h"
#include "filterCoef.h"
#include "s25fl512s.h"
#include "libmfcc.h"
#include "FlashDriver.h"
#include "sd_diskio.h"
#include "arm_abs_f32.h"
#include "lcd.h"
#include "ffneuralnet.h"
#include <stdlib.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Neural Network Variables  */
#define sizehid 90
#define sizein 36
#define sizeout 3
extern double WeightIH[sizehid][sizein];
extern double WeightHO[sizeout][sizehid];
extern double biases1[sizehid];
extern double biases2[sizeout];
volatile uint32_t user1 = 0;
volatile uint32_t user2 = 0;
volatile uint32_t user3 = 0;

/* Callbacks for DMA Transfers */

volatile uint8_t readwritedone = 0;
volatile uint8_t adcReady = 0;

/* End of Callbacks for DMA Transfers */

/* SD Card Variables */

extern uint8_t retSD; /* Return value for SD */
extern char SD_Path[]; /* SD logical drive path */
FATFS fs;
FIL fp;
FIL fp1;
char name[] = "rec.txt";
char name1[] = "mfcc.txt";
char me[] = "me.txt";
uint32_t stored;

/* End of SD Card With DMA Variables */

/*      MFCC Variables      */

extern double dctcoef[24][24];
extern double cepLifter[12];
extern const double bank[24][129];
extern uint16_t hamming[128];
uint8_t NoOfFilters = 24;


/*      End Of MFCC Variables      */

/*      FFT Variables      */

uint32_t fftSize = 128;
uint8_t ifftFlag = 0;
uint32_t doBitReverse = 1;
#define FrameStep 64
#define FrameLength 256
float32_t xmax[1] = {-255};

/*       End Of FFT Variables      */

/*  LCD and Menu Variables  */


extern uint8_t lcd_chars;
extern uint8_t lcd_lines;
extern uint8_t *lcd_line_addresses;


/*  End of LCD and Menu Variables  */

/*		 General Variables 		*/

#define BUFFER_SIZE  45000
float32_t* fbuffer;
//uint8_t aTxBuffer1[BUFFER_SIZE];
uint8_t* aTxBuffer1;
volatile uint8_t buffSelect = 1;
volatile uint8_t buff1full = 0;
//volatile uint8_t buff2full = 0;
volatile uint32_t buffIndex = 0;
volatile uint8_t doneRec = 0;
int id;
/*		End of General Variables 		*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void fft(uint32_t fftSize,float32_t buffer[],float32_t fft_out[fftSize]);
void mfcc (float32_t fft_out[] , int NoOfFilters);
double sigmoid(double x);
static void softmax( size_t input_len,double* input);
void feedforward(int coef, double nnin[coef]);
// Allows for smarter functionality if set



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*							 GENERAL INSTRUCTIONS FOR THE CODE BELOW						 */

/*	Flash Memory Instructions:
 *
 *					    1)In order to write data you MUST FISRT ERASE the sectors you are going
 *		         		  to write to   ( Master_EraseFlash(flash_addr,NoOfSectors); )
 *					    2)To read data  ( Master_ReadFromFlash(flash_addr,*buffer,NoOfBytes); )
 *				 	    3)To write data ( Master_WriteToFlash_Page(flash_addr,*buffer, NoOfPages); )
 *				   	    4)After each read an write we MUST 	(readwritedone = 0; )
 *	 	 	 	  		  Flash Memory Mapping: 256 Sectors and each of them has 256KB storage.
 *		 	 	 	 	  We can write to them using pages of 512B maximum.
 */

/*  LCD Screen Instructions:
 *
 * 						1)lcd_reset() : Used to reset and initialize the clock
 * 						2)lcd_write('',(value of RS)) : Used to write only one character
 * 						3)lcd_display_settings((turn LCD On),(Underline the last character written),(blink the last character written))
 * 						4)lcd_display_address() : sets the address to a specific point
 * 						5)lcd_print("") : prints a whole string
 *
 */

/* I2S Microphone Instructions:
 *
 * 						1)HAL_I2S_Receive_DMA(&hi2s2,buffer,buffer_size) : Used to read buffer_size data values from the microphone in uint16_t format stored to a buffer .
 * 						2)My Microphone reads data in uint24_t format
 * 						3)After each Recieve_DMA I need to wait for the recording to finish using while(i2sReady != 1){}
 *
 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	// Allocate memory of size BUFFER_SIZE to aTxBuffer1



	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	MX_ADC1_Init();

	/* USER CODE BEGIN 2 */

	/* Initialization for LCD */
	lcd_chars = 16;
	lcd_lines = 2;
	uint8_t addresses[] = {0x40};
	lcd_line_addresses = addresses;
	lcd_reset();
	lcd_display_settings(1, 1, 0);
	delay(1000);
	/* End of Initialization for LCD */

	/* Initialization for Flash Memory */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
	//	uint8_t aTxBuffer[1024];
	//	uint8_t aTxTransmit[512];
	uint32_t flash_addr=0x00000;
	//Master_EraseFlash(flash_addr,1);
	/* End of Initialization for Flash Memory */

	/* Initialization for UART */
	char serialText[6];
	char serialText1[11];
	char serialText2[16];
	/* End of Initialization for UART */

	/* Loop Indexes */
	int i,j,k,m;
	/* End of Loop Indexes */

	lcd_clear();
	lcd_reset();


	while(1){


		if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) != 0 || HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) != 0){
			lcd_print("Welcome to C&S");
			delay(2000);
			lcd_clear();
			lcd_reset();
			lcd_print("LeftBtn:Record\n");
			lcd_print("RightBtn:Back");
			doneRec = 0;
			aTxBuffer1 = (uint8_t *) malloc(BUFFER_SIZE * sizeof(uint8_t));
			fbuffer = (float32_t *)malloc(BUFFER_SIZE/2 * sizeof(float32_t));
			while(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) != 0 || HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) != 0){
				if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) != 1){

					serialPrint("Mounting SD Card...\r\n");
					retSD = f_mount(&fs, SD_Path, 1);
					while ( retSD != FR_OK)
					{
						//Unlink and Link SD driver
						FATFS_UnLinkDriver(SD_Path);
						FATFS_LinkDriver(&SD_Driver,SD_Path);
						retSD = f_mount(&fs,"", 1);
						sprintf(serialText1,"\r\nFRESULT:%02d\r\n",(uint8_t) retSD); serialPrint(serialText1);

					}

					serialPrint("Opening File...\r\n");
					retSD = f_open(&fp, name, FA_CREATE_ALWAYS | FA_WRITE);
					if (FR_OK != retSD)
					{
						while (1)
							serialPrint("Problem with f_open...\r\n");
					}

					lcd_clear();
					lcd_reset();
					lcd_print("Recording...");
					/* Polling mode Recording */
					HAL_TIM_Base_Start_IT(&htim2);
					doneRec = 0;
					buff1full = 0;
					while(doneRec != 1){
						if (buff1full == 1){
							doneRec=1;
						}
					}
					HAL_TIM_Base_Stop_IT(&htim2);

					serialPrint("Writing File...\r\n");
					f_write(&fp,aTxBuffer1,BUFFER_SIZE,&stored);
					f_close(&fp);
					//free(aTxBuffer1);	//Used while creating the Training Dataset
//					for (i=0;i<45000;i++){	//Used while creating the Training Dataset
//						sprintf(serialText1,"%03d\r\n",aTxBuffer1[i]);serialPrint(serialText1);  //Used while creating the Training Dataset
//					}
					serialPrint("\r\nSD Done!!! \r\n");
					lcd_clear();
					lcd_reset();
					lcd_print("SD Done!");
					uint32_t NumOfSamples =0;
					float32_t mean;
					double sum =0;
					//aTxBuffer1 = (uint8_t *) malloc(45000 * sizeof(uint8_t));
					//f_open(&fp,me,FA_READ); //Used while creating the Training Dataset
					//for (int l =0;l<4;l++){	//Used while creating the Training Dataset
					//f_read(&fp,aTxBuffer1,45000,&stored);	//Used while creating the Training Dataset
					for (i=0;i<BUFFER_SIZE;i++){
						if (xmax[0] < aTxBuffer1[i]){
							xmax[0] = aTxBuffer1[i];
						}
						sum += aTxBuffer1[i];
						NumOfSamples++;
					}
					//}	//Used while creating the Training Dataset
					//f_close(&fp);	//Used while creating the Training Dataset
					mean = sum/NumOfSamples;
					arm_abs_f32(xmax,xmax,1);
					xmax[0]= xmax[0]-mean;
					float32_t hambuffer[256];
					lcd_clear();
					lcd_reset();
					lcd_print("Done Recording");
					delay(500);
					lcd_clear();
					lcd_reset();
					lcd_print("Data Analysis...");
					// Data Analysis Code
					f_open(&fp1, name1, FA_CREATE_ALWAYS | FA_WRITE);//Used while creating the Training Dataset
					//	f_open(&fp,me,FA_READ);	//Used while creating the Training Dataset
					//for (int l =0;l<4;l++){	//Used while creating the Training Dataset
					//f_read(&fp,aTxBuffer1,45000,&stored);	//Used while creating the Training Dataset

					for(k=0;k<2;k++){
						for(i=0;i<22500;i++)
						{
							fbuffer[i] = aTxBuffer1[i+22500*k] - mean;
							fbuffer[i] =  ((fbuffer[i] - xmax[0])/(4*xmax[0]-1));
						}
						for(i=0;i<2*175;i++){
							for(j=0;j<2*fftSize;j++){
								if ((j%2)==0){
									hambuffer[j]=(fbuffer[i*FrameStep+j/2]*hamming[j/2])/10000;
								}else{
									hambuffer[j] = 0;
								}//sprintf(serialText1,"%.05f\r\n",hambuffer[j]);serialPrint(serialText1); //Used while creating the Training Dataset
							}//sprintf(serialText1,"%03d,%05d\r\n",i,(i*FrameStep+j/2));serialPrint(serialText1); //Used while creating the Training Dataset


							/* DFT the Buffer */
							float32_t fft_out[fftSize];
							fft(fftSize,hambuffer,fft_out);
							/* MFCC from the fft_out */
							mfcc(fft_out,NoOfFilters);

						}
					}
					//}	//Used while creating the Training Dataset
					serialPrint("\r\nMFCC Done!\r\n");
					f_close(&fp1);
					f_close(&fp);
					free(aTxBuffer1);free(fbuffer);
					id = 0;
					sprintf(serialText1,"%d,%d,%d\r\n",user1,user2,user3);serialPrint(serialText1);
					if (user1 > user2 && user1 > user3){
						if( user1 > 400 ){
							id = 1;
						}
					}else if (user2 > user1 && user2 > user3){
						if ( user2 > 350 ){
							id = 2;
						}
					}else if (user3 > user1 && user3 > user2 ){
						if (user3 > 380){
							id = 3;
						}
					}else{
						id = 0;
					}
					lcd_clear();
					lcd_reset();

					if(id == 1 ){
						lcd_print("Welcome, Thanos!");
						lcd_print(" Door Unlocked.");
					}else if ( id == 2 ){
						lcd_print("Welcome, Chris! ");
						lcd_print(" Door Unlocked.");
					}else if ( id == 3 ){
						lcd_print("Welcome, Neoklis!");
						lcd_print(" Door Unlocked.");
					}else{
						lcd_print("Not Identified.\n");
						lcd_print("Try Again!");

					}
					user1 = 0;
					user2 = 0;
					user3 = 0;
					delay(2000);
					lcd_clear();
					lcd_reset();
					break;
				}
				else if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) != 1){
					lcd_clear();
					lcd_reset();
					break;
				}
			}

		}
	}



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */




		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 2;

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_SlaveConfigTypeDef sSlaveConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

	TIM_SlaveConfigTypeDef sSlaveConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 42;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 125;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* DMA2_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
			|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA3 SPI_CS_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_3|SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
			|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PG5 PG6 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Tx and Rx Transfer completed callback.
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	readwritedone++;

}


/**
 * @brief Tx Half Transfer completed callback.
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */



void fft(uint32_t fftSize,float32_t buffer[], float32_t fft_out[fftSize]){

	int i;
	char serialt[8];

	/* 					Complex Fast Fourier Transformation					 */

	/* Process the data through the CFFT/CIFFT module *The buffer must be larger than len */
	if (fftSize ==64)
		arm_cfft_f32(&arm_cfft_sR_f32_len64,buffer,ifftFlag,doBitReverse);
	else if(fftSize == 128)
		arm_cfft_f32(&arm_cfft_sR_f32_len128,buffer,ifftFlag,doBitReverse);
	else if (fftSize ==256)
		arm_cfft_f32(&arm_cfft_sR_f32_len256,buffer,ifftFlag,doBitReverse);
	else if (fftSize == 512)
		arm_cfft_f32(&arm_cfft_sR_f32_len512,buffer,ifftFlag,doBitReverse);
	else if (fftSize == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024,buffer,ifftFlag,doBitReverse);
	else
		serialPrint("Supports up to 1024 bin fft\r\n");
	/* 				End of Complex Fast Fourier Transformation 				*/
	float32_t afbuffer[256];
	arm_abs_f32(buffer,afbuffer,256);
	//for(i=0;i<256;i++){
	//sprintf(serialt,"%.03f\r\n",afbuffer[i]);serialPrint(serialt);
	//	}
	for(i=0;i<2*fftSize;i+=2){

		fft_out[i/2] = ((afbuffer[i]*afbuffer[i])/fftSize);

	}
}

void mfcc (float32_t fft_out[] , int NoOfFilters){
	int coef = 24;
	double MFCC[coef];
	double preMFCC[coef];
	int i,j,k,m;
	char serialt[16];


	for (k=0;k<24;k++){
		preMFCC[k]=0;
		for (m=0;m<128;m++){
			preMFCC[k] += bank[k][m]*fft_out[m];
		}preMFCC[k]=(log10(preMFCC[k]));
		//				sprintf(serialt,"%.03f\r\n",preMFCC[k]);serialPrint(serialt);
	}

	for( i=0; i<24; i++ ){
		//	MFCC[i] =0;
		for (m=0;m<24;m++){
			MFCC[i] += dctcoef[i][m]*preMFCC[m];
		}
		if (i<13 && i!=0){
			f_write(&fp1, MFCC+i,sizeof(MFCC[0]),&stored);	//Used only for Training Recording
			//sprintf(serialt,"MFCC %i %.03f\r\n",i,MFCC[i]);serialPrint(serialt);
		}
	}//serialPrint("\r\n");
	//delay(1000);
	//Calculate deltas
	double dMFCC[12];
	double ddMFCC[12];
	
	int a = (-4)*36;
	fseek(&fp1,a,SEEK_CUR);
	for ( i=0; i<24; i++){
		if (i<12){
			f_read(&fp1,ddMFCC+i,sizeof(MFCC[0]),&stored);
		}
		if (i>12){
			f_read(&fp1,dMFCC+i,sizeof(MFCC[0]),&stored);
			}
	}
	for( i=0; i<12; i++){
		dMFCC[i] = (MFCC[i] - dMFCC[i])/2;
	}
	for( i=0; i<12; i++){
		ddMFCC[i] = (ddMFCC[i] - 2*MFCC[i] +dMFCC[i]);
	}
	fseek(&fp1,0,SEEK_END);
	for (i=1;i<=12;i++){
		f_write(&fp1, dMFCC+i,sizeof(dMFCC[0]),&stored);
		f_write(&fp1, ddMFCC+i,sizeof(ddMFCC[0]),&stored);
	}

	double nnin[36];
	k=0;
	m=0;

	int in =36;
	//Calculate delta-deltas
	for(i=0;i<sizein;i++){
		if (i < 12){
			nnin[i] = MFCC[i+1];
		}else if (i%2 == 0){
			nnin[i] = dMFCC[k];
			k++;
		}else{
			nnin[i] = ddMFCC[m];
			m++;
		}
		//sprintf(serialt,"%.03f\r\n",nnin[i]);serialPrint(serialt);

	}

	feedforward(in,nnin);


}
/* Activation Function */
double sigmoid(double x){
	return 2.0/(1.0 + exp(-2*x)) - 1.0 ;
}
static void softmax(size_t input_len,double* input) {

  double m = -INFINITY;
  for (size_t i = 0; i < input_len; i++) {
    if (input[i] > m) {
      m = input[i];
    }
  }

  double sum = 0.0;
  for (size_t i = 0; i < input_len; i++) {
    sum += exp(input[i] - m);
  }

  double offset = m + log(sum);
  for (size_t i = 0; i < input_len; i++) {
    input[i] = exp(input[i] - offset);
  }
}


void feedforward(int coef,double nnin[coef]){
	int i,j,p;

	uint8_t ind = 0;
	double *tout;
	tout = (double *) malloc(sizeout*sizeof(double));

	double* sumIH;
	sumIH = (double *) malloc(sizehid*sizeof(double));

	double* sumHO;
	sumHO = (double *) malloc(sizeout*sizeof(double));

	char serialt[8];
	for(i=0;i<sizehid;i++){
		sumIH[i] =0;
		for(j=0;j<sizein;j++){
			sumIH[i] += WeightIH[i][j]*nnin[j];
		}
		sumIH[i] = sigmoid(sumIH[i] + biases1[i]);
		//sprintf(serialt,"%.03f\r\n",sumIH[i]);serialPrint(serialt);
	}

	// Hidden Layer -> Output Layer
	for(j=0;j<sizeout;j++){
		sumHO[j] = 0;
		for(i=0;i<sizehid;i++){
			sumHO[j]+= WeightHO[j][i]*sumIH[i];

		}//sprintf(serialt,"%.03f\r\n",sumHO[j]);serialPrint(serialt);
		tout[j] = sumHO[j] + biases2[j];

		//sprintf(serialt,"%.03f\r\n",tout[j]);serialPrint(serialt);
	}//serialPrint("\r\n");
	softmax(3,tout);
	for (i=0;i<3;i++){
	//	sprintf(serialt,"%.03f\r\n",tout[i]);serialPrint(serialt);
	}
	float32_t max = tout[0];
	for (p=0;p<sizeout;p++){
		if (tout[p]>max){
			max = tout[p];
			ind = p;
			//sprintf(serialt,"%d\r\n",max);serialPrint(serialt);
		}
	}
	if (ind == 0){
		user1++;
	}
	if (ind == 1){
		user2++;
	}
	if (ind == 2){
		user3++;
	}
	free(sumIH);free(sumHO);free(tout);
}


/* RFFT Implementation
 *
 *
 * 		static float32_t output[256]={0};
 *		arm_status status;
 *		arm_rfft_fast_instance_f32 S;
 *
 *		status = arm_rfft_fast_init_f32(&S,fftSize);
 *		arm_rfft_fast_f32(&S,buffer,output,0);
 *		for(i=0;i<256;i++){
 *			sprintf(serialt,"%.03f\r\n",output[i]);serialPrint(serialt);
 *		}
 */
/* Menu First Try

   		lcd_print("_Welcome to C&S_");
		lcd_display_address(0x40);
		lcd_print("Press Any Button");
		if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) != 1 || HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) != 1){
			lcd_clear();
			lcd_reset();
			lcd_print("__Left Btn:Rec__");
			lcd_display_address(0x40);
			lcd_print("_Right Btn:Back_");
			while(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) != 1 || HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) != 1){
				if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) != 1){
					lcd_clear();
					lcd_reset();
					lcd_print("Recording...");
					delay(2000);
					// Recording Code
					lcd_clear();
					lcd_reset();
					lcd_print("Done Recording...");
					lcd_display_address(0x40);
					lcd_print("Data Analysis...");
					// Data Analysis Code
					while(1);
				}
				else if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) != 1){
					lcd_clear();
					lcd_reset();
					break;
				}
			}

		}

 */

/*	FLASH MEMORY TEST: Erase, Write and then Read a whole sector of the flash!
	for(int i=0;i<512;i++){
		if(i<256)
		aTxBuffer[i]= i;
		else
		aTxBuffer[i]= i-256;

	}
	Master_EraseFlash(flash_addr,1);
	for (flash_addr = 0;flash_addr<262144;flash_addr+=512){
	Master_WriteToFlash_Page(flash_addr, aTxBuffer, 1);

	}


	for (flash_addr = 0;flash_addr<262144;flash_addr+=512){
	Master_ReadFromFlash(flash_addr,aTxTransmit,512);


		sprintf(serialText,"%03d\r\n",aTxTransmit[1]); serialPrint(serialText);
		sprintf(serialText,"%03d\r\n",aTxTransmit[511]); serialPrint(serialText);
	}
	for(int i = 1;i<512;i++){
	sprintf(serialText,"%03d\r\n",aTxTransmit[i]); serialPrint(serialText);
	}

 			Buffer Oustoglou DO NOT DELETE
	uint32_t bufferOust[size/3];
	uint8_t p=0;
	for(int i=2;i<12000;i+=4)
	{
		sprintf(serialText1,"%08d\r\n",(uint32_t)((buffer[i]<<8)|(buffer[i+1]>>8)));serialPrint(serialText1);
		bufferOust[i-2-p] = ((buffer[i]<<8)|(buffer[i+1]>>8));
		if(bufferOust[i-2-p] >= 4194304){
			bufferOust[i-2-p] = bufferOust[i-2-p] - 8388608;
		}
		sprintf(serialText1,"%08d\r\n",bufferOust[i-2-p]);serialPrint(serialText1);
		k+=4;
	}
 */

/*	MFCC FIRST TRY: Failure! Compute a better filter with 128fftbins
		for (k=0;k<26;k++){
			for (m=0;m<128;m++){
				OutputInMatrix[k][m]=MelFilterbanks[k][m]*testOutput[m];
			}
		}
		for (k=0;k<26;k++){
			MFCC[k]=0;
			for (m=0;m<129;m++){
				MFCC[k] +=	OutputInMatrix[m][k];
			}
			sprintf(serialText2,"%1.02f\r\n",k,MFCC[k]);serialPrint(serialText2);
		}
		for (k=0;k<NoOfFilters;k++){
			MFCC[k]=log(fabs(MFCC[k]));
			MFCC[k]=(MFCC[k]*cos((((13*PI)/NoOfFilters))*(k-0.5f)))/NoOfFilters;
			MFCC[k]=MFCC[k]*dct[k];
			if (k<13){
				sprintf(serialText2,"MFCC %i %1.02f\r\n",k,MFCC[k]);serialPrint(serialText2);
			}
		}
 */

/* libmfcc
	float32_t mfcc_result;
	for(int coeff = 0; coeff < 13; coeff++){
		mfcc_result = GetCoefficient(fbuffer, 8000, 26, 128, coeff);
		sprintf(serialText1,"%i %1.02f\r\n", coeff, mfcc_result);serialPrint(serialText1);
	}
	serialPrint("\r\nMFCC Done!\r\n");
 */

/* 	   LCD Test
 * while(1) {
		 Simple functions
			        lcd_clear();
			        lcd_write('L', 1);
			        lcd_write('A', 1);
			        lcd_write('L', 1);
			        lcd_write('A', 1);
			        lcd_display_address(0x40);
			        lcd_print("stm32-hd44780");
			        delay(3000);

		 Smart functions (1)
			        lcd_clear();
		lcd_print(" DOYLEYEI ");
		lcd_display_address(0x40);
		lcd_print("   :D  :D  :D   ");
			        delay(1000);

			        // Smart functions (2)
			        lcd_clear();
			        lcd_print("aaaaaaaaaaaaaaaa aaaaaa");
			        delay(3000);

	}
 */


/*	SD Card
FATFS FatFs;
UINT byteCount;
FIL SDFile;
FRESULT result;

serialPrint("Mounting SD Card...\r\n");
result = f_mount(&FatFs,(TCHAR const*)SD_Path, 1);
while ( result != FR_OK)
{
	//Unlink and Link SD driver
	FATFS_UnLinkDriver(SD_Path);
	FATFS_LinkDriver(&SD_Driver,SD_Path);
	result = f_mount(&FatFs,"", 1);
	sprintf(serialText,"\r\nFRESULT:%02d\r\n",(uint8_t) result); serialPrint(serialText);

}
serialPrint("Opening File...\r\n");
if (f_open(&SDFile, "lala.txt", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
	while (1)
		serialPrint("Problem with f_open...\r\n");
lcd_clear();
lcd_reset();
lcd_print("Writing SD!");
serialPrint("Writing File...\r\n");
flash_addr = 0;
for (flash_addr = 0;flash_addr<(2*84480);flash_addr+=512){
	Master_ReadFromFlash(flash_addr,aTxTransmit,512);
	f_write(&SDFile,aTxTransmit,512,&byteCount);
}
f_close(&SDFile);
serialPrint("\r\nSD Done!!! \r\n");
lcd_clear();
lcd_reset();
lcd_print("SD Done!");
 */

//j=0;
//HAL_TIM_Base_Start_IT(&htim2);
//while(j<25){
//		if (buff1full == 1){
//
//			Master_WriteToFlash_Page(flash_addr, aTxBuffer1, 16);
//			flash_addr+=0x2000;
//			buff1full = 0;
//			j++;
//		}
//		if(buff2full == 1){
//			Master_WriteToFlash_Page(flash_addr, aTxBuffer2, 16);
//			flash_addr+=0x2000;
//			buff2full = 0;
//			j++;
//		}
//
//}
//HAL_TIM_Base_Stop_IT(&htim2);

/* Signal Analysis Using SD Card
 * 	retSD =f_open(&fp, name, FA_READ);
	if (FR_OK != retSD)
	{
		while (1)
			serialPrint("Problem with f_open...\r\n");
	}
	f_read(&fp,aTxBuffer,1024,&stored);
	uint32_t NumOfSamples =0;
	float32_t mean;
	double sum =0;
	while (stored == 1024){
		for (i=0;i<1024;i++){
			if (xmax[0] < aTxBuffer[i]){
				xmax[0] = aTxBuffer[i];
			}
			sum += aTxBuffer[i];
			NumOfSamples++;
		}
		f_read(&fp,aTxBuffer,1024,&stored);
	}
	f_close(&fp);

	mean = sum/NumOfSamples;
	arm_abs_f32(xmax,xmax,1);
	xmax[0]= xmax[0]-mean;


	f_open(&fp, name, FA_READ);
	f_read(&fp,aTxBuffer1,4096,&stored);
	float32_t hambuffer[256];
	float32_t fbuffer[4096];
	while (stored == 4096)
	{
		for(i=0;i<4096;i++)
		{
			fbuffer[i] = aTxBuffer1[i] - mean;
			fbuffer[i] =  ((fbuffer[i] - xmax[0])/(4*xmax[0]-1));
		}

		for(i=0;i<16;i++)
		{
			for(j=0;j<256;j++)
			{
				hambuffer[j]=fbuffer[i*FrameStep+j];
			}

			// DFT the Buffer
			float32_t fft_out[fftSize/2];
			fft(fftSize,hambuffer,fft_out);
			// MFCC from the fft_out
			mfcc(fft_out,NoOfFilters);

		}
		f_read(&fp,aTxBuffer1,4096,&stored);
	}

	f_close(&fp);
	serialPrint("\r\nMFCC Done!\r\n");
 */


/*      Code without Menu     */
//	serialPrint("Mounting SD Card...\r\n");
//	retSD = f_mount(&fs, SD_Path, 1);
//	while ( retSD != FR_OK)
//	{
//		//Unlink and Link SD driver
//		FATFS_UnLinkDriver(SD_Path);
//		FATFS_LinkDriver(&SD_Driver,SD_Path);
//		retSD = f_mount(&fs,"", 1);
//		sprintf(serialText1,"\r\nFRESULT:%02d\r\n",(uint8_t) retSD); serialPrint(serialText1);
//
//	}
//
//	serialPrint("Opening File...\r\n");
//	retSD = f_open(&fp, name, FA_CREATE_ALWAYS | FA_WRITE);
//	if (FR_OK != retSD)
//	{
//		while (1)
//			serialPrint("Problem with f_open...\r\n");
//	}
//
//
//	/* Recording */
//	lcd_clear();
//	lcd_reset();
//	lcd_print("Recording...");
//
//	serialPrint("\r\nRecording\r\n");
//
//	/* Polling mode Recording */
//	HAL_TIM_Base_Start_IT(&htim2);
//	while(doneRec != 1){
//		if (buff1full == 1){
//			doneRec=1;
//		}
//	}
//	HAL_TIM_Base_Stop_IT(&htim2);
//
//	lcd_clear();
//	lcd_reset();
//	lcd_print("Writing SD!");
//	serialPrint("Writing File...\r\n");
//	f_write(&fp,aTxBuffer1,45000,&stored);
//	f_close(&fp);
//	//	free(aTxBuffer1);
//	serialPrint("\r\nSD Done!!! \r\n");
//	lcd_clear();
//	lcd_reset();
//	lcd_print("SD Done!");
//	uint32_t NumOfSamples =0;
//	float32_t mean;
//	double sum =0;
//	//f_open(&fp,me,FA_READ);
//	//for (int l =0;l<4;l++){
//	//f_read(&fp,aTxBuffer1,45000,&stored);
//
//
//	for (i=0;i<45000;i++){
//		if (xmax[0] < aTxBuffer1[i]){
//			xmax[0] = aTxBuffer1[i];
//		}
//		sum += aTxBuffer1[i];
//		NumOfSamples++;
//	}
//	//}
//	//f_close(&fp);
//	mean = sum/NumOfSamples;
//	arm_abs_f32(xmax,xmax,1);
//	xmax[0]= xmax[0]-mean;
//
//	float32_t hambuffer[256];
//
//	f_open(&fp1, name1, FA_CREATE_ALWAYS | FA_WRITE);
//	//f_open(&fp,me,FA_READ);
//
//	//for (int l =0;l<4;l++){
//	//f_read(&fp,aTxBuffer1,45000,&stored);
//	for(k=0;k<2;k++){
//		for(i=0;i<22500;i++)
//		{
//			fbuffer[i] = aTxBuffer1[i+22500*k] - mean;
//			fbuffer[i] =  ((fbuffer[i] - xmax[0])/(4*xmax[0]-1));
//
//		}
//
//		for(i=0;i<2*175;i++){
//			for(j=0;j<2*fftSize;j++)
//			{
//				if ((j%2)==0){
//					hambuffer[j]=(fbuffer[i*FrameStep+j/2]*hamming[j/2])/10000;
//				}else{
//					hambuffer[j] = 0;
//				}//sprintf(serialText1,"%.05f\r\n",hambuffer[j]);serialPrint(serialText1);
//			}//sprintf(serialText1,"%03d,%05d\r\n",i,(i*FrameStep+j/2));serialPrint(serialText1);
//
//
//			/* DFT the Buffer */
//			float32_t fft_out[fftSize];
//			fft(fftSize,hambuffer,fft_out);
//			/* MFCC from the fft_out */
//			mfcc(fft_out,NoOfFilters);
//
//		}
//	}
//
//	//}
//
//	f_close(&fp1);
//	//f_close(&fp);
//	serialPrint("\r\nMFCC Done!\r\n");
//	lcd_clear();
//	lcd_reset();
//	lcd_print("MFCC Done!");
/*     End of Code without Menu     */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

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

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
