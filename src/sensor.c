#include "mine.h" 
volatile uint8_t SENSOR_start = 0;
volatile uint8_t SENSOR_reset = 0;

uint16_t led_1 = 0,led_2 = 0,led_3 = 0,led_4 = 0;
uint16_t led_1_threshold = 2330,led_2_threshold = 2335,led_3_threshold= 2310,led_4_threshold = 2650; //2330
int16_t sensor_sub = 0;
void led_flash_setting(){
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=15-1;//look period
	TIM_OC1Init(TIM9,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM9,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM9,ENABLE);
	TIM_CtrlPWMOutputs(TIM9,ENABLE);
	TIM_Cmd(TIM9,ENABLE);
}
void led_flash(){
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 15-1;
	TIM_OC1Init(TIM9,&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 15-1;
	TIM_OC2Init(TIM9,&TIM_OCInitStructure);
}
void led_stop(){
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=15-1;//look period

	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OC1Init(TIM9,&TIM_OCInitStructure);
	TIM_OC2Init(TIM9,&TIM_OCInitStructure);
}
void led_get(){
	volatile uint16_t value = 0;
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_3Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	value = ADC_GetConversionValue(ADC1);
	led_1 = (value>=led_1) ? value : led_1;

	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_3Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	value = ADC_GetConversionValue(ADC1);
	led_2 = (value>=led_2) ? value : led_2;

	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,1,ADC_SampleTime_3Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	value = ADC_GetConversionValue(ADC1);
	led_3 = (value>=led_3) ? value : led_3;

	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,1,ADC_SampleTime_3Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	value = ADC_GetConversionValue(ADC1);
	led_4 = (value>=led_4) ? value : led_4;
	SENSOR_start=OFF;
}
uint8_t read_wall(uint8_t RobotDirection){
	volatile uint8_t WallData = 0xf0;
	volatile uint8_t DirValue = (~RobotDirection) & (RobotDirection-1);
	//2wei 12 side 34 front
	
	DirValue = (DirValue & 0x55) + (DirValue >> 1 & 0x55);
	DirValue = (DirValue & 0x33) + (DirValue >> 2 & 0x33);
	DirValue = (DirValue & 0x0f) + (DirValue >> 4 & 0x0f); //countbits MSB
	if(led_1 >= led_1_threshold){ //2110
		GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_SET);
		WallData |= (0x08 << DirValue) % 0x0f;
	}
	else	GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_RESET);
	//if(led_2 >= 2650 &&  led_3 >= 2900 ){ //2320
	if(led_3 >= led_3_threshold && led_4 >= led_4_threshold ){ //2320
		GPIO_WriteBit(GPIOB,GPIO_Pin_11 | GPIO_Pin_14,Bit_SET);
		WallData |= (0x01 << DirValue) % 0x0f;
	}
	else	GPIO_WriteBit(GPIOB,GPIO_Pin_11 | GPIO_Pin_14,Bit_RESET);
	if(led_2 >= led_2_threshold){	//2110
		GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_SET);
		WallData |= (0x02 << DirValue) % 0x0f;
	}
	else	GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_RESET);
	return WallData;
}
void sensor_works(){
	SENSOR_reset = OFF;
	SENSOR_start = OFF;
	while(SENSOR_reset == OFF);
	SENSOR_reset = OFF;
	SENSOR_start = OFF;
	while(SENSOR_reset == OFF){
		if(SENSOR_start == ON)
			led_get();
	}
	SENSOR_reset = OFF;
}
void reset_led(){
	led_1 = 0;
	led_2 = 0;
	led_3 = 0;
	led_4 = 0;
}
void mouse_start(){
	led_flash();
	uint8_t flag = 1;
	while(flag){
		if(SENSOR_reset == ON){
			if(led_3 >= 2400 && led_4 >= 3000){
				flag = 0;
				led_fullon();
			}
			reset_led();
			SENSOR_reset = OFF;
		}
		if(SENSOR_start == ON)	led_get();
	}
	Delay_ms(500);
	led_fulloff();
	led_stop();
	//reset_led();
	//SENSOR_reset = OFF;
	//SENSOR_start = OFF;
}
