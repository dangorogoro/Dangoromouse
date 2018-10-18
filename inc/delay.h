#ifndef MY_DELAY_H
#define MY_DELAY_H

static __IO uint32_t TimingDelay; // __IO -- volatile

void SysTick_Handler();

void SysTickTimer_Config();
void Delay_ms(uint32_t mSecs);
void Delay_us(uint32_t uSecs);

#endif
