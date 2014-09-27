#include "switch.h"
#include "gpio.h"

int SPEED_KEYS;
int PEOPLE_KEYS;
int RAMP_PEOPLE_KEYS=1;//1 - 检测坡道和人字       0 - 不检测坡道和人字
int STOP_KEYS=1;//1 - 检测起跑线     0 - 不检测起跑线
void switch_init()
{
	GPIO_InitTypeDef switch1,switch2,switch3,switch4,switch5,switch6,switch7,switch8;
	switch1.GPIOx = PTC;                             //C端口
	switch1.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch1.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch1.GPIO_Pin = GPIO_Pin_15;                  //PC11引脚
	switch1.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch1); 
	
	switch2.GPIOx = PTC;                             //C端口
	switch2.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch2.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch2.GPIO_Pin = GPIO_Pin_14;                  //PC10引脚
	switch2.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch2); 
	
	switch3.GPIOx = PTC;                             //C端口
	switch3.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch3.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch3.GPIO_Pin = GPIO_Pin_13;                  //PC9引脚
	switch3.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch3); 
	
	switch4.GPIOx = PTC;                             //C端口
	switch4.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch4.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch4.GPIO_Pin = GPIO_Pin_12;                  //PC8引脚
	switch4.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch4);   
	
	switch5.GPIOx = PTC;                             //C端口
	switch5.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch5.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch5.GPIO_Pin = GPIO_Pin_11;                  //PC11引脚
	switch5.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch5); 
	
	switch6.GPIOx = PTC;                             //C端口
	switch6.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch6.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch6.GPIO_Pin = GPIO_Pin_10;                  //PC10引脚
	switch6.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch6); 
	
	switch7.GPIOx = PTC;                             //C端口
	switch7.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch7.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch7.GPIO_Pin = GPIO_Pin_9;                  //PC9引脚
	switch7.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch7); 
	
	switch8.GPIOx = PTC;                             //C端口
	switch8.GPIO_InitState = Bit_RESET;                //初始化后输出高电平
	switch8.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	switch8.GPIO_Pin = GPIO_Pin_8;                  //PC8引脚
	switch8.GPIO_Mode = GPIO_Mode_IPU;               //上拉输入
	//执行GPIO初始化
	GPIO_Init(&switch8);   
}

int get_speed_switch()//获得初始速度控制量
{
	int temp=0;
	temp+=!GPIO_ReadInputDataBit(PTC,GPIO_Pin_15);
	temp+=!GPIO_ReadInputDataBit(PTC,GPIO_Pin_14)*2;
	temp+=!GPIO_ReadInputDataBit(PTC,GPIO_Pin_13)*4;
	temp+=!GPIO_ReadInputDataBit(PTC,GPIO_Pin_12)*8;
	return temp;
}
int is_open_ramp_people_function()//获得人字弯控制量
{
	int key=0;
	key+=!GPIO_ReadInputDataBit(PTC,GPIO_Pin_11);
	return key;
}
int is_open_stop_function()//获得是否开启停车功能
{
	int key=0;
	key+=!GPIO_ReadInputDataBit(PTC,GPIO_Pin_10);
	return key;
}