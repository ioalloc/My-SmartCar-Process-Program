#include "encoder.h"
#include "sys.h"
#include "lptm.h"
int counter;
void encoder_init()
{
	LPTM_InitTypeDef LPTM_InitStruct1;//计数器
	LPTM_InitStruct1.LPTMxMap = LPTM_CH2_PC5;
	LPTM_InitStruct1.LPTM_InitCompareValue = 200;          //在PC模式下无意义
	LPTM_InitStruct1.LPTM_Mode = LPTM_Mode_PC_FALLING;     //下降沿触发计数模式
	LPTM_Init(&LPTM_InitStruct1);
}
int get_encoder()
{
		//读取LPTM值
		counter = LPTM_GetTimerCounterValue(LPTMR0);
		//清空计数器
		LPTM_ResetTimeCounter(LPTMR0);
	return counter;
}