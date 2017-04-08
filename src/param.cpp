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
