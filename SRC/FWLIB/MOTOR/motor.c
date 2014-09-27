#include "motor.h"
#include "ftm.h"
#include "stdbool.h"
int target_speed=42;
int super_speed=0;   //超速
int top_speed=0;   //运行试动态改变最大速度 
int middle_speed=0;   //运行试动中等速度
int min_speed=0;   //运行试动态能达到的最小速度速度 
int system_speed;
int motor_pwm=0;
int fast_de_speed=255;//当它小于一定值时 快速倒转减速

bool cancel_protect=false;
extern bool confirm;
extern int people_wan;
extern int CCD_Forward;
extern int CCD_Middle;
extern int CCD_Left;//ccd 左边的黑线的数组的下标
extern int CCD_Right;//ccd 右边的黑线的数组的下标
extern int ramp_time;
extern bool ramp;
extern bool hold_turn;
extern int SPEED_KEYS;
void motor_PID()
{
	volatile static int err=0,last_err=0,derr,motorKd=50,motorKp= 130;
	//volatile static int motor_pwm=0;
	
	if(target_speed<=min_speed+CCD_Forward/4)
		target_speed++;
	else if(target_speed>min_speed+CCD_Forward/4)
		target_speed--;
	
	//五档
	if(SPEED_KEYS==9 && target_speed<=min_speed+CCD_Forward/10)
		target_speed++;
	else if(SPEED_KEYS==9 && target_speed>min_speed+CCD_Forward/10)
		target_speed--;
//	if(people_wan!=0 && confirm==true)//确认是人字弯
//		target_speed=50;
	if(SPEED_KEYS==0)
		target_speed=70;
	if(ramp==true && ramp_time<50)//上坡加速
		target_speed=67;
	
	if(hold_turn==true)//控制弯道中的速度
		wan_speed();
	
	//快速减速  电机反转减速
	if(fast_de_speed<12)
		de_speed();
/*	if(target_speed-system_speed>0)
		UART_printf("%d   %d   %d\r\n",target_speed,system_speed,abs(target_speed-system_speed));
	else 
		UART_printf("%d   %d   -%d\r\n",target_speed,system_speed,abs(target_speed-system_speed));
	*/
	err=target_speed-system_speed;
	derr=err-last_err;
	last_err=err;
	motor_pwm=motor_pwm+err*motorKd+derr*motorKp;//derr*motorKp;
	
	//保护电机代码
	//motor_pwm=protect(motor_pwm);
	
	if(motor_pwm>10000)
	{
		motor_pwm=10000;
	}
	if(motor_pwm>0){
		set_motor_pwm(motor_pwm,0);	
		//UART_printf("%d   %d\r\n",motor_pwm,system_speed);
		//UART_printf("%d   %d   %d\r\n",target_speed,system_speed,motor_pwm);
	}
	if(motor_pwm<=0)//电机反转减速
	{
		if(motor_pwm<-10000)
			motor_pwm=-10000;
		//UART_printf("%d   %d\r\n",-motor_pwm,system_speed);
		//UART_printf("%d   %d   -%d\r\n",target_speed,system_speed,-motor_pwm);
		set_motor_pwm(0,-motor_pwm);
	}
	
/*	if(system_speed>top_speed*3/2)
	{
		set_motor_pwm(0,0);
		DelayMs(50);
		//NVIC_DisableIRQ(PIT2_IRQn);
	}
*/
	
	fast_de_speed++;
}

void wan_speed()
{
	switch(SPEED_KEYS)
	{
		case 1: ;break;                  //一档
		case 2:target_speed=55;break;   //二档
		case 4:target_speed=60;break;   //三档
		case 8:target_speed=60;break;   //四档
		case 9:target_speed=65;break;   //五档
		default :target_speed=60;break;   //
	}
}
void de_speed()
{
	switch(SPEED_KEYS)
	{
		case 0:target_speed=60;break;   //三档target_speed=55;
		case 4:break;   //三档target_speed=55;
		case 8:target_speed=60;break;   //四档
		case 9:target_speed=65;break;   //五档
	}
}
int protect(int pwm)
{
	volatile static int original_speed=0;//车未开始跑很快时的速度
	original_speed=system_speed;
	if(original_speed < 5 && pwm>3000 && cancel_protect==false)
		pwm=1500;
	if(system_speed>min_speed)
	{
		cancel_protect=true;
	}
	if(cancel_protect==true && pwm>8000 && system_speed<10)
		pwm=0;
	return pwm;
}



void motor_init()
{
	FTM_InitTypeDef pwm1;
	FTM_InitTypeDef pwm2;
	
	pwm1.Frequency = 10000;                // 10KHZ
	pwm1.FTMxMAP = FTM0_CH0_PC1;          //FTM0_CH0 PC1引脚
	pwm1.FTM_Mode = FTM_Mode_EdgeAligned; //边沿对齐模式
	pwm1.InitalDuty = 0;               //初始化后产生40%的占空比
	FTM_Init(&pwm1);
	//FTM_PWM_ChangeDuty(FTM0_CH2_PC3,8000);
	
	pwm2.Frequency = 10000;                // 10KHZ
	pwm2.FTMxMAP = FTM0_CH1_PC2;          //FTM0_CH0 PC2引脚
	pwm2.FTM_Mode = FTM_Mode_EdgeAligned; //边沿对齐模式
	pwm2.InitalDuty = 0;               //初始化后产生40%的占空比
	FTM_Init(&pwm2);
}
void set_motor_pwm(uint32_t pwm1,uint32_t pwm2)
{
	FTM_PWM_ChangeDuty(FTM0_CH0_PC1,pwm1);
	FTM_PWM_ChangeDuty(FTM0_CH1_PC2,pwm2);
}
