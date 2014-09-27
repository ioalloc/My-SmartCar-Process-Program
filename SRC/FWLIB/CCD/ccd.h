#ifndef __CCD_H__
#define __CCD_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "sys.h"

	 
void CCD1_init();
void CCD2_init();
void CCD3_init();
void CCD1_Value(uint16_t us);
void CCD2_Value(uint16_t us);
void CCD3_Value(uint16_t us);
void CCD_Value(uint16_t us);
void Adapted_Black_Thread();
void Adapted2_Black_Thread();
void road_middle_width();
void modify_CCD_Middle();
void CCD_Delay(int time);	 
void guassfilter();
void HeapAdjust(int array[],int i,int nLength);
void HeapSort(int array[],int length);
	 
#ifdef __cplusplus
}
#endif

#endif