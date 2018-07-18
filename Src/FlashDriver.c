/*
 * FlashDriver.h
 *
 *  Created on: 2 Οκτ 2017
 *      Author: Thanos
 */

/* Includes ------------------------------------------------------------------*/


#include "Macro_Definitions.h"
#include "string.h"
#include "main.h"
#include "FlashDriver.h"


/* private var definition */
/* Private define ------------------------------------------------------------*/
#define SPI_ACK_BYTES             0xA5A5
#define SPI_NACK_BYTES            0xDEAD
#define SPI_TIMEOUT_MAX           0x1000
#define SPI_SLAVE_SYNBYTE         0x53
#define SPI_MASTER_SYNBYTE        0xAC

//define commands
#define WREN    0x06   //writing enable
#define WRDI  0x04   //writing disable
#define READ4  0x13   //reading data
#define PP4   0x12         //page writing
#define ERASE4 0xdc   //erase 256kB block, 4 byte addr
#define RDSR1  0x05    //read SR1 status register   . last bit is WIP status bit
#define RDID  0x9f   //device id

/* Defines used for transfer communication */
#define PAGE_SIZE		512  //max 512 each time
#define DATA_LENGTH     PAGE_SIZE

extern volatile uint8_t readwritedone;

/* Buffer used for transmission */
uint8_t aTxMasterBuffer[DATA_LENGTH] ;//512 buffer
/* Buffer used for reception */
uint8_t aRxBuffer[DATA_LENGTH];
uint8_t ReceiveTemp[50];  //temp buffer to receive a few bytes

uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

//temp var
uint16_t LocalTemp16hex=0;//temp var
char serialt[5];


//check if flash exists. return 0 if OK.
unsigned char VerifyFlashMemory(void)
{
	//read flash device id to verify
	uint8_t commands[10];
	commands[0]=RDID;//read device id
	FLASH_CHIP_ENABLE; //enable the chip, CS=0
	if(HAL_SPI_TransmitReceive(&hspi1, commands,ReceiveTemp, 8,SPI_TIMEOUT_MAX) != HAL_OK)
	{
		serialPrint("Read Device ID Error\r\n");
	}
	FLASH_CHIP_DISABLE; //pull CS pin to high voltage
	if(ReceiveTemp[1]!=1)return 1; //byte0, device id should be 01h
	if(ReceiveTemp[2]!=2)return 1; //byte1, 02h=512Mb
	if(ReceiveTemp[3]!=0x20)return 1; //byte2,20h=512Mb
	if(ReceiveTemp[4]!=0x4d)return 1; //byte3,4D
	if(ReceiveTemp[5]!=0x00)return 1; //byte4,01h, 4-kb parameter sector with uniform 64kB
	if(ReceiveTemp[6]!=0x80)return 1; //byte5,80h

	return 0;

}
//check if the operation of flash has been complete. 0 done, otherwise timeout
unsigned char CheckIsWritingReadingComplete(uint16_t retry_times)
{
	uint8_t commands[10];
	uint16_t count=0;
	//check status
	//read the register sr1
	unsigned char op_complete=0;
	//keep checking the status bit until it is complete
	while(!op_complete)
	{

		commands[0]=RDSR1;//read register SR1
		FLASH_CHIP_ENABLE; //enable the chip, CS=0
		if(HAL_SPI_TransmitReceive(&hspi1, commands, ReceiveTemp,2,SPI_TIMEOUT_MAX) != HAL_OK)
		{
			serialPrint("read register sr1 error\r\n");
		}

		FLASH_CHIP_DISABLE;

		//receiveBuff[0] is garbage
		if((ReceiveTemp[1]&0x01) ==0)op_complete=1; //list bit changes to 0 means done
		else delay(1);//delay 100us

		count++;
		LocalTemp16hex=count;
		if(count>retry_times)return 1; //timeout
	}

	return 0;

}
//erase sectors. each sector has size of 256KB
void Master_EraseFlash(uint32_t flash_addr, uint16_t sectors)
{
	//enable writing first

	uint8_t commands[10];	// command to chip
	commands[0]=WREN;	//enable writing
	FLASH_CHIP_ENABLE; //enable the chip, CS=0
	if(HAL_SPI_TransmitReceive(&hspi1, commands,ReceiveTemp, 1,SPI_TIMEOUT_MAX) != HAL_OK)
	{
		serialPrint("Enable writing error\r\n");
	}
	FLASH_CHIP_DISABLE; //must disable first to enable chip writing.

	//now erase sectors.
	for(uint16_t i=0;i<sectors;i++)
	{
		//erase sector
		commands[0]=ERASE4;//erase sector
		commands[1]=(flash_addr>>24)&0xff; //highest byte first
		commands[2]=(flash_addr>>16)&0xff; //2nd byte 
		commands[3]=(flash_addr>>8)&0xff; //3rd byte 
		commands[4]=(flash_addr)&0xff; //last byte 
		FLASH_CHIP_ENABLE; //enable the chip, CS=0
		if(HAL_SPI_TransmitReceive(&hspi1, commands, ReceiveTemp,5,SPI_TIMEOUT_MAX) != HAL_OK)
		{
			serialPrint("Erase sector error\r\n");
		}
		FLASH_CHIP_DISABLE; //must disable first for command to take effect.

		//check status 
		if(CheckIsWritingReadingComplete(300)!=0)//in testing it needs 21370=2.1s for 1 sector. Erase of one sector may take 130ms
		{
			serialPrint("Erase sector timeout\r\n");
		}

		//next flash addr
		flash_addr+=0x40000; //256*1024;  256kB
	}

}

//write to flash. must be pages. Each page is 512 byte
void Master_WriteToFlash_Page(uint32_t flash_addr, uint8_t *pData, uint16_t pages)
{
	//enable writing first
	//send command to chip
	uint8_t commands[10+PAGE_SIZE];
	uint8_t RxDummy[2048];//max 2kB at a time
	readwritedone =0;


	//start to write data to flash
	for(uint16_t i=0;i<pages;i++)
	{
		commands[0]=WREN;
		FLASH_CHIP_ENABLE; //enable the chip, CS=0
		if(HAL_SPI_TransmitReceive_DMA(&hspi1, commands,ReceiveTemp, 1) != HAL_OK)
		{
			serialPrint("Enable writing error\r\n");
		}

		FLASH_CHIP_DISABLE; //must disable first to enable chip writing.
		while (readwritedone != 1){
		}

		readwritedone = 0;
		//write addr , then send data
		commands[0]=PP4;//page write
		commands[1]=(flash_addr>>24)&0xff; //highest byte first
		commands[2]=(flash_addr>>16)&0xff; //2nd byte 
		commands[3]=(flash_addr>>8)&0xff; //3rd byte 
		commands[4]=(flash_addr)&0xff; //last byte 
		//copy the data to array to send

		for(uint16_t p0=0;p0<PAGE_SIZE;p0++)
		{
			commands[5+p0]=pData[p0];//copy to command array
		}
		FLASH_CHIP_ENABLE; //enable the chip, CS=0
		if(HAL_SPI_TransmitReceive_DMA(&hspi1, commands,RxDummy,PAGE_SIZE+5) != HAL_OK)
		{
			serialPrint("Writing Data To Flash Error\r\n");
		}
		while (readwritedone != 1){}
		FLASH_CHIP_DISABLE; //must disable first to enable chip writing.
		readwritedone = 0;

		//check status
		if(CheckIsWritingReadingComplete(2)!=0)//10ms. we use poling mode to write data, should be done after calling the code
		{
			serialPrint("Writing Data Timeout\r\n");
		}

		flash_addr+=PAGE_SIZE;
		pData+=PAGE_SIZE;//move to next page
	}

}

//read data from flash. max 1024B at a time unless you increase buffer size
void Master_ReadFromFlash(uint32_t flash_addr, uint8_t *pData, uint16_t size)
{
	//enable writing first


	//send command to chip
	uint8_t commands[10+PAGE_SIZE*2];
	uint8_t RxBuffer[10+PAGE_SIZE*2];//max 1kB at a time
	readwritedone =0;


	commands[0]=READ4;//page read
	commands[1]=(flash_addr>>24)&0xff; //highest byte first
	commands[2]=(flash_addr>>16)&0xff; //2nd byte
	commands[3]=(flash_addr>>8)&0xff; //3rd byte
	commands[4]=(flash_addr)&0xff; //last byte
	FLASH_CHIP_ENABLE; //enable the chip, CS=0
	if(HAL_SPI_TransmitReceive_DMA(&hspi1, commands,RxBuffer, size+5) != HAL_OK)
	{
		serialPrint("Reading Flash Data Error\r\n");
	}
	while (readwritedone != 1){
	}
	readwritedone =0;
	FLASH_CHIP_DISABLE; //must disable first to enable chip writing.

	//		//check status
	if(CheckIsWritingReadingComplete(1)!=0)//10ms. we use poling mode to write data, should be done after calling the code
	{
		serialPrint("Reading Data Timeout\r\n");
	}

	//transfer to destination buffer
	for(uint16_t p0=0;p0<size;p0++)
	{
		pData[p0]=RxBuffer[5+p0];//there are 5 dummy data in the Rx buffer
	}
	//done

}

//test erasing, writing and reading data for pages(should be <256 pages). return 0 if everything is right
unsigned char TestFlash(uint32_t flash_addr, uint16_t pages)
{
	//first erase the sectors.
	//decide which sector to test
	uint16_t sector_starts=flash_addr/256/1024; //64kB
	flash_addr=sector_starts*256*1024;
	Master_EraseFlash(flash_addr,1);//only erase 1sector, 64kB for testing

	//now start to test
	uint16_t dataTest=0x0000;
	for(uint16_t i=0;i<pages;i++)
	{
		//fill the buffer first with testing data
		for(uint16_t p0=0;p0<PAGE_SIZE/2;p0++)
		{
			aTxMasterBuffer[2*p0]=dataTest&0xff;
			aTxMasterBuffer[2*p0+1]=(dataTest>>8)&0xff;//higher byte
			//			sprintf(serialt,"%03d\r\n",aTxMasterBuffer[p0]);serialPrint(serialt);

			dataTest+=1;
		}
		//now write to flash
		Master_WriteToFlash_Page(flash_addr,aTxMasterBuffer,1);//one page only
		//read data to buffer
		Master_ReadFromFlash(flash_addr,aRxBuffer,PAGE_SIZE);
		if(Buffercmp(aTxMasterBuffer,aRxBuffer,PAGE_SIZE)==1)
		{
			serialPrint("Writing Page/Reading Error\r\n");
		}
		//move to next page
		flash_addr+=PAGE_SIZE;
	}
	return 1;
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param  BufferLength: buffer's length
 * @retval 1: pBuffer identical to pBuffer1
 *         0: pBuffer differs from pBuffer1
 */
uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
	while (BufferLength--)
	{
		if (*pBuffer1 != *pBuffer2)
		{
			return 1;
		}

		pBuffer1++;
		pBuffer2++;
	}

	return 0;
}
/**
 * @}
 */

/**
 * @}
 */


