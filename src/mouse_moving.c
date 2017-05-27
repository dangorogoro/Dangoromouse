#include "mine.h"
int16_t left_speed=0,right_speed=0;
volatile uint8_t ENCODER_start=0;
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
	set_left_motor(left_speed);
	set_right_motor(right_speed);
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
	left_speed=(int16_t)TIM_GetCounter(TIM2);
	right_speed=(int16_t)TIM_GetCounter(TIM8);
	if((mode_select%10)!=2){
		TIM2->CNT = 0;
		TIM8->CNT = 0;
		len_counter += (left_speed + right_speed) / 2 ;
	}
}
uint8_t set_param(){
	uint8_t value = (uint16_t)TIM_GetCounter(TIM2) / 1000 % 16;
	return value;
}

float left_e_sum=0,right_e_sum=0;
float left_e=0,right_e=0;
float left_e_old=0,right_e_old=0;

void speed_controller(int16_t target_speed,float target_rad){
	float GYRO_rad = 0;
	if(GYRO_start == ON){
		GYRO_rad = (float)(ReadGYRO()-GYRO_offset_data)/16.4/180.0*3.14;
		degree += GYRO_rad*180.0/3.14/1000.0;
		GYRO_start = OFF;
	}
	const float left_target  = (target_speed - target_rad * WheelDistance / 2.0) * MmConvWheel;
	const float right_target = (target_speed + target_rad * WheelDistance / 2.0) * MmConvWheel;
	const float left_Kp = 2.0,right_Kp = 2.10; //1.0 1.10
	const float left_Ki = 6.0,right_Ki = 6.5; //7.0 7.2
	//const float left_Kd=0.001,right_Kd=0.001;
	left_e_old  = left_e;
	right_e_old = right_e;
	left_e	=	left_target  - left_speed;
	right_e	=	right_target -  right_speed;
	//set_speed(left_e*left_Kp+left_e_sum*left_Ki+(left_e-left_e_old)*1000.0f*left_Kd,right_e*right_Kp+right_e_sum*right_Ki+(right_e-right_e_old)*1000.0*right_Kd);
	set_speed(left_e*left_Kp+left_e_sum*left_Ki,right_e*right_Kp+right_e_sum*right_Ki);
	left_e_sum  += left_e/1000.0f;
	right_e_sum += right_e/1000.0f;
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
	if(ENCODER_start == ON){
		read_encoder();
		speed_controller(300, -30.0 * po);
		ENCODER_start=OFF;
	}
}

void turn_back(int16_t target_direction){
	while(degree >= (target_direction - 2) * 90){
		if(ENCODER_start == ON){
			read_encoder();
			speed_controller(0,-3.00);
			ENCODER_start=OFF;
		}
	}
}
void go_left(int16_t target_degree){
	while(degree <= target_degree){
		if(ENCODER_start == ON){
			read_encoder();
			speed_controller(100,100/90.0);
			ENCODER_start=OFF;
		}
	}
}
void go_right(int16_t target_degree){
	while(degree >= target_degree){
		if(ENCODER_start == ON){
			read_encoder();
			speed_controller(100,-100/90.0);
			ENCODER_start=OFF;
		}
	}
}
void go_back(){
	if(ENCODER_start == ON){
		read_encoder();
		speed_controller(-300,0);
		ENCODER_start=OFF;
	}
}
void start_wall(int16_t po){
	set_speed(0,0);
	reset_e();
	len_counter = 0;
	while(len_counter > len_measure(-170)){
		if(ENCODER_start==ON){
			read_encoder();
			speed_controller(-300,0);
			ENCODER_start=OFF;
		}
	}
	degree = po * 90.0;
	reset_e();
	len_counter = 0;
	set_speed(0,0);
	Delay_ms(1000);
	while(len_counter < len_measure(140)){
		const float target_theta =  (degree - po * 90.0) / 180.0 * PI;
		if(ENCODER_start == ON){
			read_encoder();
			speed_controller(100,- 30.0f *  target_theta);
			ENCODER_start=OFF;
		}
	}
	len_counter = 0;
}
int32_t len_measure(int32_t length){
	return length*MmConvWheel*1000;
}
void reset_e(){
	left_e_old  = 0;
	right_e_old = 0;
	left_e  = 0;
	right_e = 0;
	left_e_sum  = 0;
	right_e_sum = 0;
}
