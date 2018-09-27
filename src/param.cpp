#include "mine.h"
Param::Param(){}
Param::Param(uint16_t first_speed,uint16_t last_speed,uint16_t turn_speed, uint16_t small_turn_speed,uint16_t accel){
	speed_param[0] = first_speed;
	speed_param[1] = last_speed;
	speed_param[2] = turn_speed;
	speed_param[3] = small_turn_speed;
	speed_param[4] = accel;
}
void Param::set_all_param(uint16_t first_speed,uint16_t last_speed,uint16_t turn_speed, uint16_t small_turn_speed,uint16_t accel){
	speed_param[0] = first_speed;
	speed_param[1] = last_speed;
	speed_param[2] = turn_speed;
	speed_param[3] = small_turn_speed;
	speed_param[4] = accel;
}
void ParamList::setting(){
	Param param0(0,100,100,600,10);
	Param param1(0,600,600,600,10);
	Param param2(0,1000,600,600,5);
	Param param3(0,1800,600,600,5);
	Param param4(0,2300,600,600,10);
	Param param5(0,1800,800,600,10);
	Param param6(0,3000,800,600,10);
	Param param7(0,2500,1000,600,10);
	Param param8(0,3500,1000,600,10);

	param_list.push_back(param0);
	param_list.push_back(param1);
	param_list.push_back(param2);
	param_list.push_back(param3);
	param_list.push_back(param4);
	param_list.push_back(param5);
	param_list.push_back(param6);
	param_list.push_back(param7);
	param_list.push_back(param8);
}
