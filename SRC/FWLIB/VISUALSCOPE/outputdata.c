#include "outputdata.h"
#include "uart.h"


extern float OutData[4];

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++)
	  {      
        CRC_Temp ^= Buf[i];
			
        for (j=0;j<8;j++) 
		    {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  // unsigned char TxBuf[4];
  for(i=0;i<4;i++)
  {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
  }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  for(i=0;i<10;i++)
	{
		UART_SendData(OutData_UARTx,databuf[i]);
	}
}

void OutPut_DataInit(void)
{
	UART_InitTypeDef UART_InitStruct1;
	UART_InitStruct1.UARTxMAP = UART2_RX_PD2_TX_PD3;
	UART_InitStruct1.UART_BaudRate  = 115200;
	UART_Init(&UART_InitStruct1);
}

