#ifndef MY_CONFIG_H
#define MY_CONFIG_H
//extern uint8_t button_left,button_right,button_back,button_forward,button_1,button_2,button_b;
//#define	button_a			GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15)
#define	button_a			GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)
#define	button_return	GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)
#define ON 1
#define OFF 0
extern uint8_t mode_select;
void GPIO_setting();
uint8_t encoder_paramset();
void led_fullon();
void led_fulloff();
#endif
