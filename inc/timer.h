#ifndef MY_TIMER_H
#define MY_TIMER_H
extern NVIC_InitTypeDef NVIC_InitStructure;
extern TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
extern uint32_t timer_counter;
extern int16_t left_speed,right_speed;
extern int32_t len_counter;
#define TIM2_Period 65535-1
#define TIM3_Period 1000-1
#define TIM4_Period 1000-1 
#define TIM5_Period 21-1
#define TIM7_Period 84-1
#define TIM8_Period 65535-1
#define TIM9_Period 50-1

void TIMER_setting();
void TIM2_IRQHandler();
void TIM3_IRQHandler();
void TIM4_IRQHandler();
void TIM5_IRQHandler();
void TIM7_IRQHandler();
void start_buzzer(uint16_t value);
void stop_buzzer();
void pipi(uint16_t value);
#endif
