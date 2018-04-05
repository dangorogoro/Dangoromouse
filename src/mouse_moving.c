#include "mine.h"
int16_t left_speed = 0,right_speed = 0;
int16_t last_left_speed = 0,last_right_speed = 0;
volatile uint8_t ENCODER_start = 0;
int16_t search_velocity = 600;
float MmConvWheel = (4096.0 * 44.0 / 9.0 / 1000.0 / 78.0);  //79.0
void suction_motor_setting(){
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=70-1;//look period
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
}
void suction_start(uint16_t po){
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = po; //to-1000
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; //to-1000
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
}
void suction_stop(){
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;//look period

	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
}
void mouse_motor_setting(){
	//motor left OC1 OC2
	//motor right OC3 OC4
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=10-1;//look period
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}
void stop_motor(){
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OC1Init(TIM3,&TIM_OCInitStructure); //
	TIM_OC2Init(TIM3,&TIM_OCInitStructure); //
	TIM_OC3Init(TIM3,&TIM_OCInitStructure); //
	TIM_OC4Init(TIM3,&TIM_OCInitStructure); //
}
void set_left_motor(int16_t speed){
	if(speed>=0){
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period-speed-1; //to-1000
		TIM_OC1Init(TIM3,&TIM_OCInitStructure); //
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period-1; //to-1000
		TIM_OC2Init(TIM3,&TIM_OCInitStructure); //
	}
	else{
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period-1; //to-1000
		TIM_OC1Init(TIM3,&TIM_OCInitStructure); //
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period+speed-1; //to-1000
		TIM_OC2Init(TIM3,&TIM_OCInitStructure); //
	}
}
void set_speed(int16_t left_speed,int16_t right_speed){
	set_left_motor(-left_speed);
	set_right_motor(-right_speed);
}
void set_right_motor(int16_t speed){
	if(speed>=0){
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period-1; //to-1000
		TIM_OC3Init(TIM3,&TIM_OCInitStructure); //
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period-speed-1; //to-1000
		TIM_OC4Init(TIM3,&TIM_OCInitStructure); //
	}
	else{
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period+speed-1; //to-100
		TIM_OC3Init(TIM3,&TIM_OCInitStructure); //
		TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse=TIM3_Period-1; //to-100
		TIM_OC4Init(TIM3,&TIM_OCInitStructure); //
	}
}
void encoder_setting(){
	TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM8,ENABLE);
}
void read_encoder(){
	left_speed = TIM2->CNT;
	right_speed = TIM8->CNT;
	if((mode_select % 10) != 2){
	TIM2->CNT = 0;
	TIM8->CNT = 0;
	//if(left_speed - last_left_speed > 150) left_speed = last_left_speed;
	//if(right_speed - last_right_speed > 150) right_speed = last_right_speed;
	len_counter += (left_speed + right_speed) / 2;
	}
}
uint8_t set_param(){
	uint8_t value = (uint16_t)TIM_GetCounter(TIM2) / 1000 % 16;
	return value;
}

float left_e_sum = 0.f,right_e_sum = 0.f;
float left_e = 0.f,right_e = 0.f;
float left_e_old = 0.f,right_e_old=0.f;

void speed_controller(int16_t target_speed,float target_rad){
	float GYRO_rad = 0;
	if(GYRO_start == ON){
		GYRO_rad = (float)(ReadGYRO()-GYRO_offset_data)/16.4/180.0*3.14;
		degree += GYRO_rad*180.0/3.14/1000.0;
		GYRO_start = OFF;
	}
	const float left_target  = (float)(target_speed - target_rad * WheelDistance / 2.0);
	const float right_target = (float)(target_speed + target_rad * WheelDistance / 2.0);
	const float left_Kp = 12.0,right_Kp = 12.0; // 2.5 2.6 1.0 1.10
	const float left_Ki = 40.0,right_Ki = 40.0; //7.0 7.2
	//const float left_Kd=0.001,right_Kd=0.001;
	left_e_old  = left_e;
	right_e_old = right_e;
	left_e	=	(float)(left_target  - left_speed / (float)MmConvWheel);
	right_e	=	(float)(right_target - right_speed / (float)MmConvWheel);
	//if(fabs(left_e_old - left_e) > 100) left_e = left_e_old;
	//if(fabs(right_e_old - right_e) > 100) right_e = right_e_old;
	left_e_sum  += left_e / 1000.0;
	right_e_sum += right_e / 1000.0;
	//set_speed(left_e*left_Kp+left_e_sum*left_Ki+(left_e-left_e_old)*1000.0f*left_Kd,right_e*right_Kp+right_e_sum*right_Ki+(right_e-right_e_old)*1000.0*right_Kd);
	set_speed(left_e * left_Kp + left_e_sum * left_Ki,right_e * right_Kp + right_e_sum * right_Ki);
}
void mouse_turn(const uint8_t value){
	while(1){
		degree = 0;
		set_speed(-25,25);
		if(value == 0){
			while(degree<87){
				if(GYRO_start==ON){
					GYRO_sampling();
					GYRO_start=OFF;
				}
			}
		}
		set_speed(0,0);
		stop_motor();
		degree=0;
		Delay_ms(1000);
	}
}
void go_straight(float po){
	float now_speed = (left_speed + right_speed) / 2.0 / MmConvWheel;
	float target_speed = now_speed;
	if(ENCODER_start == ON){
		read_encoder();
		if(target_speed < search_velocity - 30 ) target_speed += 30;
		else target_speed = search_velocity;
		speed_controller(target_speed, -target_speed / 20.0 * po);
		ENCODER_start = OFF;
	}
}
void turn_back(int16_t target_direction){
	set_speed(0,0);
	Delay_ms(300);
	while(degree >= (target_direction - 2) * 90){
		if(ENCODER_start == ON){
			read_encoder();
			speed_controller(0,-8.00);
			ENCODER_start = OFF;
		}
	}
}
void turn_side(int16_t target_direction,int8_t wall_dir){
	float current_degree = degree;
	float first_turn_degree,second_turn_degree;
	set_speed(0,0);
	Delay_ms(100);
	float last_rad = 5.0;
	float target_rad = 0.0;
	bool first_flag = false;
	bool second_flag = false;
	if(wall_dir == 1){
		while(degree <= (target_direction + (int16_t)wall_dir) * 90){
			if(ENCODER_start == ON){
				read_encoder();
				if(target_rad < last_rad && first_flag == false) target_rad += 0.30;
				else if(first_flag == false && target_rad >= last_rad){
					target_rad = last_rad;
					first_flag = true;
					first_turn_degree = degree;
					second_turn_degree = (target_direction + wall_dir) * 90 + current_degree - first_turn_degree;
				}
				if(first_flag == true && degree >= second_turn_degree && second_flag == false)	second_flag = true;
				if(second_flag == true){
					target_rad -= 0.30;
					if(target_rad <= 0.0f)	break;
				}
				speed_controller(0,target_rad);
				ENCODER_start = OFF;
			}
		}
	}
	else{
		while(degree >= (target_direction + (int16_t)wall_dir) * 90){
			if(ENCODER_start == ON){
				read_encoder();
				if(target_rad > -last_rad && first_flag == false) target_rad -= 0.30;
				else if(first_flag == false && target_rad <= -last_rad){
					target_rad = -last_rad;
					first_flag = true;
					first_turn_degree = degree;
					second_turn_degree = (target_direction + wall_dir) * 90 + current_degree - first_turn_degree;
				}
				if(first_flag == true && degree <= second_turn_degree && second_flag == false)	second_flag = true;
				if(second_flag == true){
					target_rad += 0.30;
					if(target_rad >= 0.0f)	break;
				}

				speed_controller(0,target_rad);
				ENCODER_start = OFF;
			}
		}
	}
	set_speed(0,0);
	Delay_ms(100);
}
void go_left(int16_t target_degree){
	float start_degree = degree;
	float last_rad = search_velocity / 50.0; //50 30 15
	float rad_size = last_rad / 20.0;
	float target_rad = 0;
	int8_t init_flag = 0;
	float first_degree = 0.0;
	while(degree <= target_degree){
		if(ENCODER_start == ON){
			read_encoder();
			if(init_flag == 0)	target_rad = (target_rad >= last_rad) ? last_rad : target_rad + rad_size;
			if(init_flag == 0 && target_rad >= last_rad){
				init_flag = 1;
				first_degree = degree;
			}
			if(init_flag == 1 && start_degree - first_degree <= degree - target_degree){
				if(target_rad >= 0.0) target_rad = target_rad - rad_size;
				else break;	
			}
			speed_controller(search_velocity,target_rad);
			ENCODER_start = OFF;
		}
	}
}
void go_right(int16_t target_degree){
	float start_degree = degree;
	float last_rad = search_velocity / 50.0;
	float rad_size = last_rad / 20.0;
	float target_rad = 0;
	int8_t init_flag = 0;
	float first_degree = 0.0;
	while(degree >= target_degree){
		if(ENCODER_start == ON){
			read_encoder();
			if(init_flag == 0)	target_rad = (target_rad >= last_rad) ? last_rad : target_rad + rad_size;
			if(init_flag == 0 && target_rad >= last_rad){
				init_flag = 1;
				first_degree = degree;
			}
			if(init_flag == 1 && start_degree - first_degree >= degree - target_degree){
				if(target_rad >= 0.0) target_rad = target_rad - rad_size;
				else break;	
			}
			speed_controller(search_velocity,- target_rad);
			ENCODER_start = OFF;
		}
	}
}

void go_back(float po){
	float now_speed = (left_speed + right_speed) / 2.0 / MmConvWheel;
	float target_speed = now_speed;
	if(ENCODER_start == ON){
		read_encoder();
		if(target_speed > -search_velocity / 2) target_speed -= 50;
		else target_speed = -search_velocity / 2;
		speed_controller(target_speed, -target_speed / 15.0 * po);
		ENCODER_start = OFF;
	}
}
void start_wall(int16_t po){
	set_speed(0,0);
	reset_e();
	len_counter = 0;
	while(len_counter > len_measure(-80)){
		go_back(0);
	}
	degree = po * 90.0;
	reset_e();
	len_counter = 0;
	set_speed(0,0);
	Delay_ms(300);
	while(len_counter < len_measure(150)){
		const float target_theta =  (degree - po * 90.0) / 180.0 * PI;
		go_straight(target_theta);
	}
	len_counter = 0;
}
int32_t len_measure(int32_t length){
	return length * MmConvWheel * 1000;
}
void reset_e(){
	left_e_old  = 0;
	right_e_old = 0;
	left_e  = 0;
	right_e = 0;
	left_e_sum  = 0;
	right_e_sum = 0;
}
void start_withoutwall(int16_t po){
	set_speed(0,0);
	reset_e();
	len_counter = 0;
	while(len_counter > len_measure(-60)){
		go_back(0);
	}
	reset_e();
	len_counter = 0;
	set_speed(0,0);
	Delay_ms(300);
	while(len_counter < len_measure(150)){
		const float target_theta =  (degree - po * 90.0) / 180.0 * PI;
		go_straight(target_theta);
	}
	len_counter = 0;
}
