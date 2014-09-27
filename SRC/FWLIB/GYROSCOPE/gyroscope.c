#include "gyroscope.h"
#include "adc.h"
#include "sys.h"
uint8_t gyro_value=0;
uint8_t last_gyro_value=0;
ADC_InitTypeDef gyro_ADC;//ADC0_SE18(32)  E25
void gyro_init()
{
	gyro_ADC.ADCxMap = ADC0_SE18_E25;
	gyro_ADC.ADC_Precision = ADC_PRECISION_8BIT;////ADC 精度定义 8BIT
	gyro_ADC.ADC_TriggerSelect = ADC_TRIGGER_SW;  //软件触发(A 通道可使用软/硬件触发 B 通道只能使用硬件触发)
	ADC_Init(&gyro_ADC);
}
void get_gyro_value()
{
	last_gyro_value=gyro_value;
	gyro_value=ADC_GetConversionValue(ADC0_SE18_E25);//AD取值
}