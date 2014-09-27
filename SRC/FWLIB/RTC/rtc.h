/**
  ******************************************************************************
  * @file    rtc.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   超核K60固件库 实时时钟驱动 头文件
  ******************************************************************************
  */
#ifndef __RTC_H_
#define	__RTC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "sys.h"
//时间结构体
typedef struct 
{
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;			
	uint8_t Month;
	uint8_t Date;
	uint8_t Week;	
	uint16_t Year;
	uint32_t TSRValue;
}RTC_CalanderTypeDef;		

//本构件实现的函数接口列表
void RTC_Init(void);
void RTC_SecondIntProcess(RTC_CalanderTypeDef * RTCx);
uint8_t RTC_SetData(RTC_CalanderTypeDef * RTCx);

#ifdef __cplusplus
}
#endif

#endif
