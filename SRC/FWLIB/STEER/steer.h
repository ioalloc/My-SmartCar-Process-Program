#include "sys.h"
#include "stdbool.h"
void steer_init();
void set_steer_angle(uint16_t steer);
void steer_PID();
void turn_where();
int is_people_bent();
bool confirm_people_bent();
bool people_second();
bool people_third();
void straight_speed_Kp_Kd();
void memory_turn();
bool delete_wan_speed();
void speed_Kp_Kd(int err,int derr);
void wan_speed_Kp_Kd(int err,int derr);
int whereIsRoad(bool hold_turn);
int cross_road(int *cross_road_counter);
int ramp_steer_pwm();
int ramp_middle();
int hold_deadly_pwm();
bool left_right_wrong();
void changeParameter(uint8_t receiveBuffer[]);
void steer_PID_test();