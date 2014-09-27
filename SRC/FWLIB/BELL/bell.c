#include "bell.h"
#include "gpio.h"
//#include "sys.h"

void bell_init()
{
	GPIO_InitTypeDef bell;
	bell.GPIOx = PTA;                             //C端口
	bell.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	bell.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	bell.GPIO_Pin = GPIO_Pin_5;                  //PC11引脚
	bell.GPIO_Mode = GPIO_Mode_OPP;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&bell); 
}
void bell_open()
{
	GPIO_SetBits(PTA,GPIO_Pin_5);
	
}
void bell_open_ms(int time)
{
	volatile static int counter=0;
	if(counter<time)
		bell_open();
	else 
	{
		bell_close();
		counter=0;
	}
	counter++;	
}
void bell_close()
{
	GPIO_ResetBits(PTA,GPIO_Pin_5);
}