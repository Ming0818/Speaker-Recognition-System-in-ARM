/*
 * FlashDriver.h
 *
 *  Created on: 2 Οκτ 2017
 *      Author: Thanos
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FlashDriver_H
#define __FlashDriver_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"



//global variables
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

//not used, but don't delete, because it is used in other functions defined earlier
#define FLASH_HOLD_ENABLE      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET) //PC7, LOW=HOLD
#define FLASH_HOLD_DISABLE      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET) //PC7, HIGH=DISABLE
#define FLASH_WP_ENABLE      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET) //PC8, LOW=WRITE PROTECT
#define FLASH_WP_DISABLE      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET) //PC8, HIGH=DISABLE
#define FLASH_CHIP_ENABLE      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET) //PA4, LOW=cs SELECTED
#define FLASH_CHIP_DISABLE      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET) //PA4, HIGH=DISABLE


//test erasing, writing and reading data for pages(should be <64). return 0 if everything is right
unsigned char TestFlash(uint32_t flash_addr, uint16_t pages);
//read data from flash. 
void Master_ReadFromFlash(uint32_t flash_addr, uint8_t *pData, uint16_t size);
//write to flash. must be pages. Each page is 512 byte
void Master_WriteToFlash_Page(uint32_t flash_addr, uint8_t *pData, uint16_t pages);
//erase sectors. each sector has size of 64KB
void Master_EraseFlash(uint32_t flash_addr, uint16_t sectors);
//check if flash exists. return 0 if OK.
unsigned char VerifyFlashMemory(void);
#ifdef __cplusplus
}
#endif
#endif /*__NozzleDriveBSP_H */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
