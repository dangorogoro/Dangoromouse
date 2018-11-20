#include "mine.h"
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
bool timer_clock = OFF;
bool traject_clock = OFF;
bool wall_detect = OFF;
uint32_t timer_counter = 0;
static uint16_t buzzer_counter = 0;
int32_t len_counter = 0;
bool stop_flag = false;
uint32_t comeback_clock = 0;
void TIMER_setting(){
  NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /*
   * TIM2----ENCODER
   * TIM3----MOTOR
   * TIM4----BUZZER
   * TIM5----SEIGYO
   * TIM8----ENCODER
   * TIM9----FLASH
   */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);

  TIM_TimeBaseStructure.TIM_Period = TIM1_Period;
  TIM_TimeBaseStructure.TIM_Prescaler = 5;
  TIM_TimeBaseStructure.TIM_ClockDivision= 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period=TIM2_Period;
  TIM_TimeBaseStructure.TIM_Prescaler=1-1;
  TIM_TimeBaseStructure.TIM_ClockDivision=0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
  /*	
      TIM_TimeBaseStructure.TIM_Period=TIM7_Period;
      TIM_TimeBaseStructure.TIM_Prescaler=2-1	;	
      TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
      TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);
      */
  TIM_TimeBaseStructure.TIM_Period=TIM5_Period; //160kHz
  TIM_TimeBaseStructure.TIM_Prescaler=25-1;
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period=TIM4_Period;
  TIM_TimeBaseStructure.TIM_Prescaler=100-1;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

  //for servo mouse dc
  TIM_TimeBaseStructure.TIM_Period=TIM3_Period;
  TIM_TimeBaseStructure.TIM_Prescaler=1-1;
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period=TIM9_Period;
  TIM_TimeBaseStructure.TIM_Prescaler=168-1;
  TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period=TIM8_Period;
  TIM_TimeBaseStructure.TIM_Prescaler=1-1;
  TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Falling);

  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2,ENABLE);
  TIM_Cmd(TIM8,ENABLE);
}

void TIM5_IRQHandler(){ //100khz -> 160khz
  if(TIM_GetITStatus(TIM5,TIM_IT_Update)!=RESET){
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
    timer_counter = timer_counter % 200000 + 1;
    if(timer_counter % 160 == 0){
      GYRO_start = ON;
      ENCODER_start = ON;
      //SENSOR_reset = ON;
      if(button_return == true) stop_flag = true;
    }
    SENSOR_start = ON;
    if(timer_counter % 64 == 0)	SENSOR_reset = ON;
    if(timer_counter % 800 == 0){
      traject_clock = ON;
      wall_detect = ON;
    }
    if(timer_counter % 1600 == 0){
      timer_clock = ON; // 0.01sec
      comeback_clock++;
    }
  }
}
/*
   void TIM2_IRQHandler(){
   if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET){
   TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//GPIO_ToggleBits(GPIOD,GPIO_Pin_2);
//GPIO_ToggleBits(GPIOB,GPIO_Pin_11);
}
}
*/
void TIM3_IRQHandler(){
  if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET){
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
    //GPIO_ToggleBits(GPIOB,GPIO_Pin_4);
    //GPIO_ToggleBits(GPIOB,GPIO_Pin_0);
  }
}

void TIM4_IRQHandler(){
  if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET){
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    buzzer_counter=buzzer_counter%3+1;
    if(buzzer_counter==1)
      GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);
    else
      GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_RESET);
  }
}

void TIM7_IRQHandler(){
  if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET){
    TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
    GPIO_ToggleBits(GPIOB,GPIO_Pin_12);
  }
}
void start_buzzer(uint16_t value){
  TIM_TimeBaseStructure.TIM_Period=40000/value;//10000 1sec 10000 8399
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM4,ENABLE);
}
void stop_buzzer(){
  TIM_Cmd(TIM4,DISABLE);
}
void pipi(uint16_t value){
  start_buzzer(value * 2);
  Delay_ms(100);
  stop_buzzer();
}
