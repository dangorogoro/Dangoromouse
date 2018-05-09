#include "mine.h"


float degree = 0;
float degree_old = 0;
float GYRO_offset_data;
volatile float GYRO_old = 0;
volatile float GYRO_new = 0;
volatile uint8_t GYRO_start = 0;
int32_t speed = 0;
int32_t GYRO_flag = 0;

void SPI_setting(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_Cmd(SPI1,ENABLE);
}
void MPU6500_setting(){
	WriteReg(0x6B,0x80);
	Delay_ms(100);
	WriteReg(0x6B,0x00);
	Delay_ms(100);
	WriteReg(0x1A,0x00);
	Delay_ms(100);
	WriteReg(0x1B,0x18);
	Delay_ms(100);
}
int16_t ReadGYRO(){
	int16_t data;
	data=(int16_t)(((uint16_t)ReadReg(0x47)<<8) | ((uint16_t)ReadReg(0x48)));
	return data;
}
int16_t ReadZAccel(){
	return (int16_t)(((uint16_t)ReadReg(0x3f)<<8) | ((uint16_t)ReadReg(0x40)));
}
uint8_t SPI_exchange(uint8_t TX_Data){
	uint8_t RX_Data = 0;
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1,(TX_Data));
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);
	RX_Data= SPI_I2S_ReceiveData(SPI1);
	return RX_Data;
}

uint8_t ReadReg(uint8_t reg){
	uint8_t data;
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
	SPI_exchange(reg | 0x80);
	data = SPI_exchange(0);
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);
	return data;
}
void WriteReg(uint8_t reg, uint8_t data){
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
	SPI_exchange(reg | 0x00);
	SPI_exchange(data);
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);
}
void GYRO_offset(){
	float sum = 0;
	for (int i=0;i<1000;i++){
		sum += ReadGYRO();
		Delay_ms(1);
	}
	GYRO_offset_data = sum / 1000.0;
	USART_printf("%d\r\n",(int)GYRO_offset_data);
}
void GYRO_NameCall(){
	USART_printf("MPU6500...%d",ReadReg(0x75));
}
void GYRO_sampling(){
	GYRO_old = GYRO_new;
	GYRO_new = ReadGYRO() - GYRO_offset_data;
	degree_old = degree;
	degree += (float)(GYRO_old + GYRO_new) / 16.4 / 2.0 / 1000.0;
	//USART_printf("degree%d\r\n",(int32_t)(degree));
	//USART_printf("%d...%d\r\n",left_speed,right_speed);
	const float p = 50.0,d = 0.05;//80
	speed = (int32_t)(degree * p + (degree - degree_old) * 1000.0 * d);
}
bool checkZAccel(){
	int16_t threshold =  -2000;

	if(threshold > ReadZAccel())	GYRO_flag ++;
	else if(button_return == 1) GYRO_flag += 1000;
	else GYRO_flag = 0;
	if(GYRO_flag >= 1000){
		start_buzzer(10);
		return true;
	}
	else return false;
}	
