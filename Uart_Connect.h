
/***************************************************************************************
 *
 *  Copyright (C),2012 PGS CO.,LTD all rights reserved
 *
 *  FileName:    main.h
 *
 *  Description: main.h文件包含于所有文件当中。
 *
 *  History:
 *	   <Author>      <Date>      <Version >      <Description>
 *	                 2019.03.05      V0.1	         Create initial version.
 *
****************************************************************************************/
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _Uart_Connect_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#define _Uart_Connect_H_


#include "widget.h"

#include <QApplication>




/* Includes ------------------------------------------------------------------*/



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */






/***************************************************************************************
                                  5. Debug define
***************************************************************************************/
typedef enum
{
    UART_PERFORM_FAIL = 0x00,
    UART_PERFORM_SUCCCESS = 0x01,
}UART_RESULT_T;
/***************************************************************************************
                                  6. Common define

***************************************************************************************/
#define POLY        0x1021



/***************************************************************************************
                                  7. Common Function

***************************************************************************************/

uint8_t CRCSense(uint8_t * buf,uint16_t len);

QByteArray QString2Hex(QString str);

char C_ConvertHexChar(char ch);

uint16_t crc16(unsigned char *addr, int num, uint16_t crc);

uint8_t Uart_Flash_Data_Check(uint8_t * buf,uint16_t len);

/***************************************************************************************/




#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

