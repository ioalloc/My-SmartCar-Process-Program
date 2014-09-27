#include "ccd.h"
//#include "stdio.h"
#include "gpio.h"
#include "adc.h"
#include "delay.h"
#include "uart.h"
#include "stdbool.h"
#define PTA_SET *(uint32_t *)0x400ff004u
#define PTA_CLEAN *(uint32_t *)0x400ff008u
#define PTB_SET *(uint32_t *)0x400ff044u
#define PTB_CLEAN *(uint32_t *)0x400ff048u


uint8_t ADC_Value[128];
uint8_t ADC2_Value[128];
uint8_t ADC3_Value[128];
int ADC_Offset[128];
int up_down_Offset[128];
int upright_Offset[128];
uint8_t CCD_Threshold=50;//ccd 门限值
uint8_t Last_CCD_Threshold;//ccd 门限值

uint8_t CCD2_Threshold=50;//ccd 门限值
uint8_t Last_CCD2_Threshold;//ccd 门限值

uint8_t CCD3_Threshold=0;//ccd 门限值
uint8_t Last_CCD3_Threshold;//ccd 门限值

int ramp_before_steer_pwm;//上坡前的舵机的打角
int Road_Middle=64;//左大，右小
int Road_Width=68;
int CCD_Forward=0;//ccd 左边的黑线的数组的下标
int last_CCD_Forward=80;//ccd 左边的黑线的数组的下标
int CCD_Left=34;//ccd 左边的黑线的数组的下标
int CCD_Right=98;//ccd 右边的黑线的数组的下标
int Black_Left=34;//ccd 左边的黑线以左是否为黑
int Black_Right=98;//ccd 右边的黑线以右是否为黑
int White_Middle=60;//middle is white?
int Thread_Left=30;//ccd 左边的黑线的数组的下标
int Thread_Right=97;//ccd 右边的黑线的数组的下标
int CCD_Middle=0;
int last_CCD_Middle;
int last_CCD_Left=0;//ccd 上一次左边的黑线的数组的下标
int last_CCD_Right=0;//ccd 上一次右边的黑线的数组的下标

extern int system_speed;
extern int target_speed;
extern int top_speed;
extern int middle_speed;
extern int min_speed;
int ADC_MAX=0,ADC_MIN=255,last_ADC_MAX=0,last_ADC_MIN=255;
int ADC2_MAX=0,ADC2_MIN=255,last_ADC2_MAX=0,last_ADC2_MIN=255;
int ADC3_MAX=0,ADC3_MIN=255,last_ADC3_MAX=0,last_ADC3_MIN=255;

int CCD3_Left=10;//ccd 左边的黑线的数组的下标
int CCD3_Right=10;//ccd 右边的黑线的数组的下标
int CCD3_Middle=0;
int last_CCD3_Middle;
int last_CCD3_Left=0;//ccd 上一次左边的黑线的数组的下标
int last_CCD3_Right=0;//ccd 上一次右边的黑线的数组的下标

GPIO_InitTypeDef SI_1;//PTB10
GPIO_InitTypeDef KLC_1;//PTB3
ADC_InitTypeDef ADC_1;//PTB2

GPIO_InitTypeDef SI_2;//PTB0
GPIO_InitTypeDef KLC_2;//PTB1
ADC_InitTypeDef ADC_2;//PTA17

GPIO_InitTypeDef SI_3;//PTB0
GPIO_InitTypeDef KLC_3;//PTB1
ADC_InitTypeDef ADC_3;//PTA17
/*c
ccd 初始化
*/
void CCD1_init()
{

	
	//初始化默认的调试端口
	SI_1.GPIOx = PTB;                             //C端口
	SI_1.GPIO_InitState = Bit_SET;                //初始化后输出高电平
	SI_1.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	SI_1.GPIO_Pin = GPIO_Pin_10;                  //PB2引脚
	SI_1.GPIO_Mode = GPIO_Mode_OPP;               //推挽输出
	//执行GPIO初始化
	GPIO_Init(&SI_1);
	
	KLC_1.GPIOx = PTB;                             //C端口
	KLC_1.GPIO_InitState = Bit_SET;                //初始化后输出高电平
	KLC_1.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	KLC_1.GPIO_Pin = GPIO_Pin_3;                  //PB3引脚
	KLC_1.GPIO_Mode = GPIO_Mode_OPP;               //推挽输出
	//执行GPIO初始化
	GPIO_Init(&KLC_1);
	
	ADC_1.ADCxMap = ADC0_SE12_PB2;
	ADC_1.ADC_Precision = ADC_PRECISION_8BIT;////ADC 精度定义 8BIT
	ADC_1.ADC_TriggerSelect = ADC_TRIGGER_SW;  //软件触发(A 通道可使用软/硬件触发 B 通道只能使用硬件触发)
	ADC_Init(&ADC_1);
}
void CCD2_init()
{

	// SI_2	  PTA17
		// KLC_2  PTB9
		// ADC_2  PTB1 ADC0_SE9_PB1
	//初始化默认的调试端口
	SI_2.GPIOx = PTA;                             //C端口
	SI_2.GPIO_InitState = Bit_SET;                //初始化后输出高电平
	SI_2.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	SI_2.GPIO_Pin = GPIO_Pin_17;                  //PB2引脚
	SI_2.GPIO_Mode = GPIO_Mode_OPP;               //推挽输出
	//执行GPIO初始化
	GPIO_Init(&SI_2);
	
	KLC_2.GPIOx = PTB;                             //C端口
	KLC_2.GPIO_InitState = Bit_SET;                //初始化后输出高电平
	KLC_2.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	KLC_2.GPIO_Pin = GPIO_Pin_9;                  //PB3引脚
	KLC_2.GPIO_Mode = GPIO_Mode_OPP;               //推挽输出
	//执行GPIO初始化
	GPIO_Init(&KLC_2);
	
	ADC_2.ADCxMap = ADC0_SE9_PB1;
	ADC_2.ADC_Precision = ADC_PRECISION_8BIT;////ADC 精度定义 8BIT
	ADC_2.ADC_TriggerSelect = ADC_TRIGGER_SW;  //软件触发(A 通道可使用软/硬件触发 B 通道只能使用硬件触发)
	ADC_Init(&ADC_2);
}
void CCD3_init()
{
	//SI_3   ADC0_SE8_PB0
	//KLC_3  PB20
	//ADC_3  ADC0_SE14_PC0
	
	//初始化默认的调试端口
	SI_3.GPIOx = PTB;                             //C端口
	SI_3.GPIO_InitState = Bit_SET;                //初始化后输出高电平
	SI_3.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	SI_3.GPIO_Pin = GPIO_Pin_0;                  //PB2引脚
	SI_3.GPIO_Mode = GPIO_Mode_OPP;               //推挽输出
	//执行GPIO初始化
	GPIO_Init(&SI_3);
	
	KLC_3.GPIOx = PTB;                             //C端口
	KLC_3.GPIO_InitState = Bit_SET;                //初始化后输出高电平
	KLC_3.GPIO_IRQMode = GPIO_IT_DISABLE;          //不时能中断
	KLC_3.GPIO_Pin = GPIO_Pin_20;                  //PB3引脚
	KLC_3.GPIO_Mode = GPIO_Mode_OPP;               //推挽输出
	//执行GPIO初始化
	GPIO_Init(&KLC_3);
	
	ADC_3.ADCxMap = ADC0_SE14_PC0;
	ADC_3.ADC_Precision = ADC_PRECISION_8BIT;////ADC 精度定义 8BIT
	ADC_3.ADC_TriggerSelect = ADC_TRIGGER_SW;  //软件触发(A 通道可使用软/硬件触发 B 通道只能使用硬件触发)
	ADC_Init(&ADC_3);
	
	
	
}
void CCD1_Value(uint16_t us)//线性CCD采集
{
		//SI   B10
	// KLC   B3
	//GPIO_SetBits(PTB,GPIO_Pin_2);
	uint8_t i=0;
		GPIO_ResetBits(PTB,GPIO_Pin_10);         /* SI为低电平*/
		GPIO_ResetBits(PTB,GPIO_Pin_3);
		CCD_Delay(us);
	
    GPIO_SetBits(PTB,GPIO_Pin_10);     /* SI为高电平*/
	CCD_Delay(us);
    GPIO_SetBits(PTB,GPIO_Pin_3);         /* CLK为高电平 */
	CCD_Delay(us);
	
    GPIO_ResetBits(PTB,GPIO_Pin_10);         /* SI为低电平*/
	CCD_Delay(us);
	ADC_Value[0]=ADC_GetConversionValue(ADC0_SE12_PB2);
	GPIO_ResetBits(PTB,GPIO_Pin_3);    /* CLK为低电平*/
	
    for(i=1; i<128; i++)
    {
				CCD_Delay(us);
			//DelayUs(2);
			GPIO_SetBits(PTB,GPIO_Pin_3);    /* CLK为高电平*/
			CCD_Delay(us);
			
			ADC_Value[i]=ADC_GetConversionValue(ADC0_SE12_PB2);//AD取值
				GPIO_ResetBits(PTB,GPIO_Pin_3);    /* CLK为低电平*/
				
				//CCD_Delay(us);
				//DelayUs(20);
    }
		CCD_Delay(us);
    GPIO_SetBits(PTB,GPIO_Pin_10);         /* SI为高电平*/
		CCD_Delay(us);
		GPIO_ResetBits(PTB,GPIO_Pin_10);         /* SI为低电平*/
}
void CCD2_Value(uint16_t us)
{
		// SI_2	  PTA17
		// KLC_2  PTB9
		// ADC_2  PTB1 ADC0_SE9_PB1
	//GPIO_SetBits(PTB,GPIO_Pin_1);
	//GPIO_ResetBits(PTB,GPIO_Pin_1);
		uint8_t i=0;
    GPIO_SetBits(PTA,GPIO_Pin_17);     /* SI为高电平*/
    DelayUs(us);
    GPIO_SetBits(PTB,GPIO_Pin_9);         /* CLK为高电平 */
    DelayUs(us);
    GPIO_ResetBits(PTA,GPIO_Pin_17);         /* SI为低电平*/
		DelayUs(us);
    GPIO_ResetBits(PTB,GPIO_Pin_9);      /* CLK为低电平*/
	
		ADC2_Value[i]=ADC_GetConversionValue(ADC0_SE9_PB1);//AD取值
		
		//UART_printf("%x",ADC_Value);
    for(i=1; i<128; i++)
    {
        DelayUs(us);
        GPIO_SetBits(PTB,GPIO_Pin_9);    /* CLK为高电平*/
				DelayUs(us);
        GPIO_ResetBits(PTB,GPIO_Pin_9);    /* CLK为低电平*/
				//CCDPixelshow();	
				ADC2_Value[i]=ADC_GetConversionValue(ADC0_SE9_PB1);//AD取值
				//UART_printf("%x",ADC_Value);
    }
    DelayUs(us);
		//ADC_Value=255;
		//UART_printf("%x",ADC_Value);
    GPIO_SetBits(PTB,GPIO_Pin_9);       /* CLK为高电平*/
		DelayUs(us);
    GPIO_ResetBits(PTB,GPIO_Pin_9);        /* CLK为低电平*/
		
}

void CCD3_Value(uint16_t us)//线性CCD采集
{
		//SI_3   ADC0_SE8_PB0
		//KLC_3  PB20
		//ADC_3  ADC0_SE14_PC0
	
	//GPIO_SetBits(PTB,GPIO_Pin_2);
	uint8_t i=0;
	GPIO_ResetBits(PTB,GPIO_Pin_0);         /* SI为低电平*/
	GPIO_ResetBits(PTB,GPIO_Pin_20);
	CCD_Delay(us);
	
    GPIO_SetBits(PTB,GPIO_Pin_0);     /* SI为高电平*/
	CCD_Delay(us);
    GPIO_SetBits(PTB,GPIO_Pin_20);         /* CLK为高电平 */
	CCD_Delay(us);
	
    GPIO_ResetBits(PTB,GPIO_Pin_0);         /* SI为低电平*/
	CCD_Delay(us);
	ADC3_Value[0]=ADC_GetConversionValue(ADC0_SE14_PC0)-6;
	GPIO_ResetBits(PTB,GPIO_Pin_20);    /* CLK为低电平*/
	
    for(i=1; i<128; i++)
    {
			CCD_Delay(us);
			//DelayUs(2);
			GPIO_SetBits(PTB,GPIO_Pin_20);    /* CLK为高电平*/
			CCD_Delay(us);
			
			ADC3_Value[i]=ADC_GetConversionValue(ADC0_SE14_PC0)-6;//AD取值
			GPIO_ResetBits(PTB,GPIO_Pin_20);    /* CLK为低电平*/
				
				//CCD_Delay(us);
				//DelayUs(20);
    }
		CCD_Delay(us);
    GPIO_SetBits(PTB,GPIO_Pin_0);         /* SI为高电平*/
		CCD_Delay(us);
		GPIO_ResetBits(PTB,GPIO_Pin_0);         /* SI为低电平*/
}
void Adapted_Black_Thread()//自适应的黑线判决  动态有选择的改变判决门限
{
	int i=0,sum=0,sumcount = 0,m_count[256] = {0};
	//volatile static int ADC_MAX=0,ADC_MIN=255,last_ADC_MAX=0,last_ADC_MIN=255;
	last_CCD_Left=CCD_Left;//记住上一次的值
	last_CCD_Right=CCD_Right;
	
	CCD_Left=0;
	CCD_Right=0;
	Black_Left=0;
	White_Middle=0;
	Black_Right=0;
	
	last_ADC_MAX=ADC_MAX;
	last_ADC_MIN=ADC_MIN;
	ADC_MAX=0;
	ADC_MIN=255;
	
	ADC_Offset[0]=0;
	for(i=0;i<127;i++)//计算差分  刹车用
		ADC_Offset[i+1]=(int)ADC_Value[i+1]-(int)ADC_Value[i];
	
	for(i=0;i<128;i++)
	{
		if(ADC_Value[i]>ADC_MAX)
			ADC_MAX=ADC_Value[i];
		if(ADC_Value[i]<ADC_MIN)
			ADC_MIN=ADC_Value[i];
	}
	
	
	if(ADC_MAX-ADC_MIN<ADC_MIN)//最强的点和最弱的点的差都要很大才行
	{
		ADC_MAX=last_ADC_MAX;
		ADC_MIN=last_ADC_MIN;
	}
	
	CCD_Threshold=(ADC_MAX+ADC_MIN)/2;
	CCD_Threshold=(ADC_MAX+CCD_Threshold)/2;
	CCD_Threshold=((ADC_MAX+ADC_MIN)/2+CCD_Threshold)/2;
	

	for(i=0;i<128;i++)//二值化
	{
			if(ADC_Value[i]<CCD_Threshold)
				ADC_Value[i]=0;
			else 
				ADC_Value[i]=1;
	}
	
	for(i=1;i<127;i++)//去掉单独的噪点
	{
		if(ADC_Value[i-1]!=ADC_Value[i] && ADC_Value[i]!=ADC_Value[i+1])
			ADC_Value[i]=ADC_Value[i+1];
	}
	up_down_Offset[0]=0;
	for(i=1;i<128;i++)//计算差分  人字识别
	{
		if(ADC_Value[i]>ADC_Value[i-1])
		up_down_Offset[i]=1;
		else if(ADC_Value[i]<ADC_Value[i-1])
			up_down_Offset[i]=-1;
		else 
			up_down_Offset[i]=0;
	}
	
	for(i=0;i<64;i++)//数左边 0 的个数
	{
		if(ADC_Value[i]==0)
			CCD_Left++;
	}
	for(i=64;i<128;i++)
		if(ADC_Value[i]==0)
			CCD_Right++;
		
	for(i=0;i<125;i++)
	{
		if(ADC_Value[i]==0&&ADC_Value[i+1]==0&&ADC_Value[i+2]==1&&ADC_Value[i+3]==1)
			Thread_Left=i+1;
		if(ADC_Value[i]==1&&ADC_Value[i+1]==1&&ADC_Value[i+2]==0&&ADC_Value[i+3]==0)
			Thread_Right=i+2;
	}
	
	//计算CCD_Left左边是否全黑  CCD_Right右边是否全黑
	for(i=0;i<CCD_Left;i++)
	{
		if(ADC_Value[i]==0)
			Black_Left++;
	}
	for(i=127;i>127-CCD_Right;i--)
	{
		if(ADC_Value[i]==0)
			Black_Right++;
	}
	for(i=CCD_Left;i<128-CCD_Right;i++)
	{
		if(ADC_Value[i]==1)
			White_Middle++;
	}
	
	//UART_printf("%d    %d   %d    %d\r\n",CCD_Left,CCD_Right,ADC_MAX,ADC_MIN);
	//UART_printf("%d    %d    %d     %d    %d    %d\r\n\r\n",CCD_Left,abs(CCD_Middle),CCD_Right,ADC_MAX,ADC_MIN,system_speed);
	last_CCD_Middle=CCD_Middle;

	CCD_Middle=CCD_Right-CCD_Left;
}
void Adapted2_Black_Thread()//自适应的黑线判决  动态有选择的改变判决门限
{
	int i=0,average=0,sum=0;
	CCD_Forward=0;
	ADC2_MAX=0;
	ADC2_MIN=255;
	//用的是CCD1 的阈值
	for(i=25;i<97;i++)
	{
		if(ADC2_Value[i]>ADC2_MAX)
			ADC2_MAX=ADC2_Value[i];
		if(ADC2_Value[i]<ADC2_MIN)
			ADC2_MIN=ADC2_Value[i];
		sum+=ADC2_Value[i];
	}
	average=sum/72;
	
	if(ADC2_MAX-ADC2_MIN<ADC2_MIN)//最强的点和最弱的点的差都要很大才行
	{
		if(average>=CCD_Threshold)
			CCD2_Threshold=0;
		else
			CCD2_Threshold=255;
	}
	else
	{
		CCD2_Threshold=(ADC2_MAX+ADC2_MIN)/2;
		CCD2_Threshold=(ADC2_MAX+CCD2_Threshold)/2;
		CCD2_Threshold=((ADC2_MAX+ADC2_MIN)/2+CCD2_Threshold)/2;
	}
	
	for(i=0;i<128;i++)//二值化
	{
			if(ADC2_Value[i]<CCD2_Threshold)
				ADC2_Value[i]=0;
			else 
				ADC2_Value[i]=1;
	}
	
/*	for(i=1;i<127;i++)//去掉单独的噪点
	{
		if(ADC2_Value[i-1]!=ADC2_Value[i] && ADC2_Value[i]!=ADC2_Value[i+1])
			ADC2_Value[i]=ADC2_Value[i+1];
	}*/
	upright_Offset[0]=0;
	for(i=1;i<128;i++)//计算差分  人字识别
	{
		if(ADC2_Value[i]>ADC2_Value[i-1])
			upright_Offset[i]=1;
		else if(ADC2_Value[i]<ADC2_Value[i-1])
			upright_Offset[i]=-1;
		else 
			upright_Offset[i]=0;
	}
	
	for(i=25;i<95;i++)//只用了25到95下标的点
	{
		if(ADC2_Value[i]+ADC2_Value[i+1]+ADC2_Value[i+2]>1)//0多于3个
		{
			CCD_Forward++;
		}
		if(ADC2_Value[i]+ADC2_Value[i+1]+ADC2_Value[i+2]<2)//2个zero
			break;
	}
}

void Adapted3_Black_Thread()//自适应的黑线判决  动态有选择的改变判决门限
{
	int i=0,sum=0,sumcount = 0,m_count[256] = {0};
	//volatile static int ADC_MAX=0,ADC_MIN=255,last_ADC_MAX=0,last_ADC_MIN=255;
	last_CCD3_Left=CCD3_Left;//记住上一次的值
	last_CCD3_Right=CCD3_Right;
	
	CCD3_Left=0;
	CCD3_Right=0;
	
	last_ADC3_MAX=ADC3_MAX;
	last_ADC3_MIN=ADC3_MIN;
	ADC3_MAX=0;
	ADC3_MIN=255;
	
	for(i=0;i<128;i++)
	{
		if(ADC3_Value[i]>ADC3_MAX)
			ADC3_MAX=ADC3_Value[i];
		if(ADC3_Value[i]<ADC3_MIN)
			ADC3_MIN=ADC3_Value[i];
	}
	
	
	if(ADC3_MAX-ADC3_MIN<ADC3_MIN)//最强的点和最弱的点的差都要很大才行
	{
		ADC3_MAX=last_ADC3_MAX;
		ADC3_MIN=last_ADC3_MIN;
	}
	
	CCD3_Threshold=(ADC3_MAX+ADC3_MIN)/2;
	CCD3_Threshold=(ADC3_MAX+CCD3_Threshold)/2;
	CCD3_Threshold=((ADC3_MAX+ADC3_MIN)/2+CCD3_Threshold)/2;
	

	for(i=0;i<128;i++)//二值化
	{
			if(ADC3_Value[i]<CCD3_Threshold)
				ADC3_Value[i]=0;
			else 
				ADC3_Value[i]=1;
	}
	
	for(i=1;i<127;i++)//去掉单独的噪点
	{
		if(ADC3_Value[i-1]!=ADC3_Value[i] && ADC3_Value[i]!=ADC3_Value[i+1])
			ADC3_Value[i]=ADC3_Value[i+1];
	}
	
	for(i=0;i<64;i++)//数左边 0 的个数
	{
		if(ADC3_Value[i]==0)
			CCD3_Left++;
	}
	for(i=64;i<128;i++)
		if(ADC3_Value[i]==0)
			CCD3_Right++;
	
	//UART_printf("%d    %d   %d    %d\r\n",CCD_Left,CCD_Right,ADC_MAX,ADC_MIN);
	//UART_printf("%d    %d    %d     %d    %d    %d\r\n\r\n",CCD_Left,abs(CCD_Middle),CCD_Right,ADC_MAX,ADC_MIN,system_speed);
	last_CCD3_Middle=CCD3_Middle;

	CCD_Middle=CCD_Right-CCD_Left;
}
void road_middle_width()
{
	int i=0,left,right,last_road_middle;
	last_road_middle=Road_Middle;
	for(i=Road_Middle;i<128;i++)
	{
		if(ADC_Value[i]!=1)
		{
			right=i;
			break;
		}
		else
			right=i;
	}
	for(i=Road_Middle;i>=0;i--)
	{
		if(ADC_Value[i]!=1)
		{
			left=i;
			break;
		}
		else
			left=i;
	}
	Road_Middle=(left+right)/2;
	if(abs(Road_Middle-last_road_middle)>6)
		Road_Middle=last_road_middle*9/10+Road_Middle*1/10;
	if(right>=left)
		Road_Width=right-left;
}
void modify_CCD_Middle()
{
	int err=0,i=0;
	err=last_CCD_Middle-CCD_Middle;
	//for(i=0;i<64;i++)
		//if()
}
void guassfilter()//高斯滤波
{
	int i;
	int guasstemplate[5] = {4,6,9,6,4};
	uint8_t  pixeltmp[128];
	for(i = 2;i<126;i++)
	{
		pixeltmp[i]=(ADC_Value[i-2]*guasstemplate[0]
		+ADC_Value[i-1]*guasstemplate[1]
		+ADC_Value[i]*guasstemplate[2]
		+ADC_Value[i+1]*guasstemplate[3]
		+ADC_Value[i+2]*guasstemplate[4])
		/(guasstemplate[0]+guasstemplate[1]+guasstemplate[2]+guasstemplate[3]+guasstemplate[4]);
	}
	for(i=2;i<126;i++)
		ADC_Value[i]=pixeltmp[i];
}

void CCD_Delay(int time)//延时10个for 循环
{
	int i=0;
	for(i=0;i<time;i++){}
}

void CCD_Value(uint16_t us)
{
    uint32_t CCD1_ADCMAP = ADC0_SE12_PB2;
    uint32_t CCD2_ADCMAP = ADC0_SE9_PB1;

    ADC_MapTypeDef *pADC_Map_CCD1 = (ADC_MapTypeDef*)&CCD1_ADCMAP;
    ADC_MapTypeDef *pADC_Map_CCD2 = (ADC_MapTypeDef*)&CCD2_ADCMAP;
    
		//SI   B10
	// KLC   B3
	//GPIO_SetBits(PTB,GPIO_Pin_2);
	uint8_t i=0;
		
    PTB_SET |= (1<<10);     
    PTA_SET |= (1<<17);
    PTB_SET |= (1<<3);    
    PTB_SET |= (1<<9);
	CCD_Delay(us);

    PTB_CLEAN |= (1<<10);         
	PTA_CLEAN |= (1<<17);

    for(i=0; i<128; i++)
    {
			
			PTB_CLEAN |= (1<<3);    
            PTB_CLEAN |= (1<<9);

			CCD_Delay(us);
            


            ADC0->SC1[pADC_Map_CCD1->ADC_IsChlAB] &= ~(ADC_SC1_ADCH_MASK);	
            ADC0->SC1[pADC_Map_CCD1->ADC_IsChlAB] |= pADC_Map_CCD1->ADC_Chl;
            
CCD_Delay(us);
            while((ADC0->SC1[pADC_Map_CCD1->ADC_IsChlAB] & ADC_SC1_COCO_MASK) == 0);
								ADC_Value[i] = ADC0->R[pADC_Map_CCD1->ADC_IsChlAB]-6;
CCD_Delay(us);	
						ADC0->SC1[pADC_Map_CCD2->ADC_IsChlAB] &= ~(ADC_SC1_ADCH_MASK);	
            ADC0->SC1[pADC_Map_CCD2->ADC_IsChlAB] |= pADC_Map_CCD2->ADC_Chl;
            while((ADC0->SC1[pADC_Map_CCD2->ADC_IsChlAB] & ADC_SC1_COCO_MASK) == 0);
							ADC2_Value[i] = ADC0->R[pADC_Map_CCD2->ADC_IsChlAB]-6;
                        
			//ADC_Value[i]=ADC_GetConversionValue(ADC0_SE12_PB2);
			PTB_SET |= (1<<3);    
            PTB_SET |= (1<<9);
			
			CCD_Delay(us);
    }
		PTB_CLEAN |= (1<<3);    
        PTB_CLEAN |= (1<<9);
		CCD_Delay(us);
}

