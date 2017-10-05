#include "mine.h"
void USART_setting(){
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx;
	//USART_Init(USART1,&USART_InitStructure);
	//USART_Cmd(USART1,ENABLE);
	USART_Init(USART3,&USART_InitStructure);
	USART_Cmd(USART3,ENABLE);
}
void USART_putc(USART_TypeDef* USARTx,char c){
	while(!(USARTx->SR & 0x00000040)); 
	USART_SendData(USARTx,c);
}
void USART_puts(USART_TypeDef* USARTx,const char *s){
	for(uint16_t i=0;s[i]!=0;i++) USART_putc(USARTx,s[i]);
}
void USART_printf(const char* format, ...){
	va_list list;
	va_start(list,format);
	uint16_t len =vsnprintf(0,0,format,list);
	char *str;
	str = (char *)malloc(len+1);
	vsprintf(str,format,list);
	USART_puts(USART3,str);
	//USART_puts(USART2,str);
	free(str);
	va_end(list);
	return ;
}

void _sbrk_r(){
	return ;
}
/*void usart_write(uint16_t value){
	uint16_t length=0;
	uint16_t value_copy=value;
	char str[10];
	char *p=str;
	while(value_copy/10){
		value_copy/=10;
		length++;
	}
	value_copy=value;
	str[length+1]='\0';
	while(1){
		str[length]=value_copy%10+'0';
		value_copy/=10;
		if(length==0)break;
		length--;
	}
	while(*p!='\0'){
		USART_SendData(USART2,*p);
		Delay_ms(1);
		p++;
	}
}*/

