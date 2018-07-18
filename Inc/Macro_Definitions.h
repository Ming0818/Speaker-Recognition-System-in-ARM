/********************************** MACRO DEFINITIONS *****************************/
#define   serialPrint(X)  	(HAL_UART_Transmit(&huart1, X, sizeof(X), 500))
#define   pinWrite(X,Y,Z) 	(HAL_GPIO_WritePin(X, Y, Z))
#define	  pinRead(X,Y)		(HAL_GPIO_ReadPin(X,Y))
#define   delay(X)    		(HAL_Delay(X))
