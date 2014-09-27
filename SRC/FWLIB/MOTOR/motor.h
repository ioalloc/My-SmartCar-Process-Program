#include "sys.h"
void motor_init();
void set_motor_pwm(uint32_t pwm1,uint32_t pwm2);
void motor_PID();
void wan_speed();
void de_speed();
int protect(int pwm);