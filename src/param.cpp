#include "mine.h"
Param::Param(){
};
Param::Param(uint16_t first_speed,uint16_t last_speed,uint16_t turn_speed, uint16_t accel){
	speed_param[0] = first_speed;
	speed_param[1] = last_speed;
	speed_param[2] = turn_speed;
	speed_param[3] = accel;
}
void Param::set_all_param(uint16_t first_speed,uint16_t last_speed,uint16_t turn_speed, uint16_t accel){
	speed_param[0] = first_speed;
	speed_param[1] = last_speed;
	speed_param[2] = turn_speed;
	speed_param[3] = accel;
}
void Param::set_first_param(uint16_t speed){
	speed_param[0] = speed;
}
void Param::set_last_param(uint16_t speed){
	speed_param[1] = speed;
}
void Param::set_turn_param(uint16_t speed){
	speed_param[2] = speed;
}
void Param::set_accel_param(uint16_t accel){
	speed_param[3] = accel;
}

uint16_t Param::get_first_param(){
	return speed_param[0];
}
uint16_t Param::get_last_param(){
	return speed_param[1];
}
uint16_t Param::get_turn_param(){
	return speed_param[2];
}
uint16_t Param::get_accel_param(){
	return speed_param[3];
}
void ParamList::setting(){
Param param1(0,100,100,10);
Param param2(0,300,300,10);
Param param3(0,600,600,10);
Param param4(0,1600,600,10);
Param param5(0,1000,1000,5);
Param param6(0,1500,1000,10);

param_list.push_back(param1);
param_list.push_back(param2);
param_list.push_back(param3);
param_list.push_back(param4);
param_list.push_back(param5);
param_list.push_back(param6);
}
