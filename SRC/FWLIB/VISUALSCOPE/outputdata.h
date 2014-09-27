
#ifndef _outputdata_H
#define _outputdata_H

#include "sys.h"

#define OutData_UARTx       UART2
#define OutData_UARTx_MAP   UART2_RX_PD2_TX_PD3

extern float OutData[4];
void OutPut_Data(void); //需要UART_Init()函数配合使用，
                            //因此在使用时序定义UART初始化结构体
void OutPut_DataInit(void);
#endif 
