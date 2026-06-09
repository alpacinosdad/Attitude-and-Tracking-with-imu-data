/*
*******************************************************************************
** Copyright (C), 2019-2024, PGS R&D CENTER
** FileName: Uart_connect.c
** <author>: hongming.cui
** Date:
** Description:
**
** Others:
** Function List:
**   1.
** History:
**  <修订者>    <修订时间>    <修订内容>    <修订版本>
*******************************************************************************
*/
//*****************************************************************************
// File Include
//*****************************************************************************
#include "Uart_Connect.h"




//*****************************************************************************
// Local Defines
//*****************************************************************************



//*****************************************************************************
// Local structures
//*****************************************************************************




//*****************************************************************************
// Global Data
//*****************************************************************************



//*****************************************************************************
// Local Data
//*****************************************************************************



//*****************************************************************************
// Local Functions Declaration
//*****************************************************************************



//*****************************************************************************
// API Functions
//*****************************************************************************



/**************************************************************************
 * 函数名ˇ:QString2Hex()
 * 描述  ˇ:将字符串变更为字符数组数据
 * 输入  ˇ:字符串
 * 返回  ˇ:字符数组
 *************************************************************************/

QByteArray QString2Hex(QString str)
{
    QByteArray senddata;
    int hexdata,lowhexdata;
    int hexdatalen = 0;
    int len = str.length();
    senddata.resize(len/2);
    char lstr,hstr;
    for(int i=0; i<len; )
    {
        hstr=str[i].toLatin1();
        if(hstr == ' ')
        {
            i++;
            continue;
        }
        i++;
        if(i >= len)
           break;
        lstr = str[i].toLatin1();
        hexdata = C_ConvertHexChar(hstr);
        lowhexdata = C_ConvertHexChar(lstr);
        if((hexdata == 16) || (lowhexdata == 16))
            break;
        else
           hexdata = hexdata*16+lowhexdata;
        i++;
        senddata[hexdatalen] = (char)hexdata;
        hexdatalen++;
    }
    senddata.resize(hexdatalen);

    return senddata;
}


/**************************************************************************
 * 函数名ˇ:C_ConvertHexChar()
 * 描述  ˇ:将16进制数据转化为字符型数据
 * 输入  ˇ:16进制变量
 * 返回  ˇ:字符型变量
 *************************************************************************/

char C_ConvertHexChar(char ch)
{
    if((ch >= '0') && (ch <= '9'))
            return ch-0x30;
        else if((ch >= 'A') && (ch <= 'F'))
            return ch-'A'+10;
        else if((ch >= 'a') && (ch <= 'f'))
            return ch-'a'+10;
        else return (-1);
}


/**************************************************************************
 * 函数名ˇ:crc16()
 * 描述  ˇ:对输入的数据进行CRC16校验
 * 输入  ˇ:addr为相关数据的首地址，num为需要校验数据的位数，crc为取校验的异或值
 * 返回  ˇ:返回校验结果两个字节
 *************************************************************************/

uint16_t crc16(unsigned char *addr, int num, uint16_t crc)
{
    int i;
    for (; num > 0; num--)              /* Step through bytes in memory */
    {
        crc = crc ^ (*addr++ << 8);     /* Fetch byte from memory, XOR into CRC top byte*/
        for (i = 0; i < 8; i++)             /* Prepare to rotate 8 bits */
        {
            if (crc & 0x8000)            /* b15 is set... */
            crc = (crc << 1) ^ POLY;    /* rotate and XOR with polynomic */
            else                          /* b15 is clear... */
            crc <<= 1;                  /* just rotate */
        }                             /* Loop for 8 bits */
        crc &= 0xFFFF;                  /* Ensure CRC remains 16-bit value */
    }                               /* Loop until num=0 */

      return(crc);                    /* Return updated CRC */
}




/**************************************************************************
 * 函数名ˇ:CRCSense()
 * 描述  ˇ:对接收到的数据进行crc16校验的验证，验证OK把标志位变更为检测OK
 * 输入  ˇ无
 * 返回  ˇCRC校验结果
 *************************************************************************/

uint8_t CRCSense(uint8_t * buf,uint16_t len)
{
    unsigned int CRCCheck = 0X0000;

    if(len < 520)
    {
        CRCCheck = crc16(buf,len,0Xffff);
    }
    else
    {
        CRCCheck = 0x0001;
    }

    if(0x0000 == CRCCheck)
    {
        return UART_PERFORM_SUCCCESS;
    }
    else
    {
        return UART_PERFORM_FAIL;
    }

}



/**************************************************************************
 * 函数名ˇ:CRCSense()
 * 描述  ˇ:对接收到的数据进行crc16校验的验证，验证OK把标志位变更为检测OK
 * 输入  ˇ无
 * 返回  ˇCRC校验结果
 *************************************************************************/

uint8_t Uart_Flash_Data_Check(uint8_t * buf,uint16_t len)
{
    uint8_t Check_state;
    /*uint16_t First_Len,Second_Len,Third_Len;

    First_Len = buf[1] * 256 + buf[2] + 7;

    Second_Len = First_Len + buf[1 + First_Len]*256 + buf[2+ First_Len] + 7;

    Third_Len = Second_Len + buf[1 + Second_Len]*256 + buf[2+ Second_Len] + 7;


    if((First_Len) == len)
    {
        if(buf[0] == 0xBB)
        {
            if(UART_PERFORM_SUCCCESS == CRCSense(buf,len))
            {
                Check_state = SUCCESS;

            }
            else
            {
                Check_state = ERR_PACKET_CRC;
            }
        }
        else
        {
             Check_state =  ERR_PACKET_HEAD;
        }
    }
    else
    {
        if(Second_Len == len)
        {
            if((buf[0] == 0xBB)&&(buf[0 + First_Len] == 0xBB))
            {
                if(UART_PERFORM_SUCCCESS == CRCSense(buf,len))
                {
                    Check_state = SUCCESS;
                }
                else
                {
                    Check_state = ERR_PACKET_CRC;
                }
            }
            else
            {
                Check_state =  ERR_PACKET_HEAD;
            }
        }
        else if(Third_Len == len)
        {
            if((buf[0] == 0xBB)&&(buf[0 + First_Len] == 0xBB)&&(buf[0 + Second_Len] == 0xBB))
            {
                if(UART_PERFORM_SUCCCESS == CRCSense(buf,len))
                {
                    Check_state = SUCCESS;

                }
                else
                {
                    Check_state = ERR_PACKET_CRC;
                }
            }
            else
            {
                Check_state =  ERR_PACKET_HEAD;
            }
        }
        else
        {
            Check_state =  ERR_PACKET_NUM;
        }
    }
*/

    if(UART_PERFORM_SUCCCESS == CRCSense(buf,len))
    {
        Check_state = SUCCESS;

    }
    else
    {
        Check_state = ERR_PACKET_CRC;
    }
    return Check_state;

}

