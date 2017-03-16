#include "mine.h"
TIM_OCInitTypeDef TIM_OCInitStructure;
uint8_t z_point=19;
void set_servo(){
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=30-1;//look period
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}
void moving_servo(uint16_t angle,uint8_t mode){
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=angle-1; //20-40
	switch(mode){
		case 1:
			TIM_OC1Init(TIM3,&TIM_OCInitStructure); //30->135度
			break;
		case 2:
			TIM_OC2Init(TIM3,&TIM_OCInitStructure); //30->135度
			break;
		case 3:
			TIM_OC4Init(TIM3,&TIM_OCInitStructure); //30->135度
			break;
	}
}
void stop_servo(uint8_t mode){
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	switch(mode){
		case 1:
			TIM_OC1Init(TIM3,&TIM_OCInitStructure); //30->135度
			break;
		case 2:
			TIM_OC2Init(TIM3,&TIM_OCInitStructure); //30->135度
			break;
		case 3:
			TIM_OC4Init(TIM3,&TIM_OCInitStructure); //30->135度
			break;

	}
}
void get_servo(){
	moving_servo(26,1);//was 27
	moving_servo(34,2);
	Delay_ms(100);
	//stop_servo(1);
	//stop_servo(2);
	//Delay_ms(1000);
}
void release_servo(){
	moving_servo(34,1);
	moving_servo(25,2);
	Delay_ms(100);
	//stop_servo(1);
	//stop_servo(2);
	//Delay_ms(100);
}
void z_servo(){//19-36
	/*moving_servo(19,3);
	Delay_ms(2000);
	moving_servo(36,3);
	*/
	if(z_point<=19)	z_point=19;
	if(z_point>=43) z_point=43;
	moving_servo(z_point,3);
	Delay_ms(100);
	}
