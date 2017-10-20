#include "mine.h"
uint16_t race_sensor0;
uint16_t race_sensor1;
uint16_t race_sensor2;
uint16_t race_sensor3;
uint16_t line_color0;
uint16_t line_color1;
uint16_t line_color2;
uint16_t line_color3;
void ADC_setting(){
	ADC_DeInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
  ADC_StructInit(&ADC_InitStructure);

	ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_10Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;

	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge= ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion=5;	//how many channels do you want to set
	ADC_Init(ADC1,&ADC_InitStructure);
	Delay_ms(500);

	ADC_Cmd(ADC1,ENABLE);
}
void battery_check(){
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14,1,ADC_SampleTime_56Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	const uint16_t value = ADC_GetConversionValue(ADC1);
	USART_printf("BATTERY%d\n\r",value);
	if(value>=4096){
		ADC_setting();
		battery_check();
	}
	else if(value<3120){
		GPIO_WriteBit(GPIOB,GPIO_Pin_11,Bit_SET);
		GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET);
		GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET);
		GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_SET);
		GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_SET);
		start_buzzer(20);
		Delay_ms(1000);
		stop_buzzer();
		start_buzzer(10);//10good
		Delay_ms(1000);
		stop_buzzer();
		while(1);
	}

}
void start_ADC(){
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_56Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	race_sensor0=ADC_GetConversionValue(ADC1);
	Delay_ms(3);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_56Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	race_sensor1=ADC_GetConversionValue(ADC1);
	Delay_ms(3);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_4,1,ADC_SampleTime_56Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	race_sensor2=ADC_GetConversionValue(ADC1);
	Delay_ms(3);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_56Cycles);//
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	race_sensor3=ADC_GetConversionValue(ADC1);
	Delay_ms(3);
}
/*
void check_line(){
	start_ADC();
	if(race_sensor0>boader_line) line_color0=B;
		line_color0=W;
	if(race_sensor1>boader_line) line_color1=B;
		line_color1=W;
	if(race_sensor2>boader_line) line_color2=B;
		line_color2=W;
	if(race_sensor3>boader_line) line_color3=B;
		line_color3=W;
}*/
