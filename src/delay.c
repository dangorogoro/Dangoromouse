#include "mine.h"
RCC_ClocksTypeDef RCC_Clocks;
uint32_t uCountFreq;
uint32_t mCountFreq;
void SysTick_Handler() {
	if (TimingDelay != 0)TimingDelay--;
}
void SysTickTimer_Config(){
	RCC_GetClocksFreq(&RCC_Clocks);
	mCountFreq = RCC_Clocks.HCLK_Frequency/1000;
	uCountFreq = RCC_Clocks.HCLK_Frequency/1000000;
	SysTick_Config(mCountFreq);
}
void Delay_ms(uint32_t mSecs) {
	SysTick->LOAD = (mCountFreq & SysTick_LOAD_RELOAD_Msk) - 1;
	TimingDelay = mSecs;
	while (TimingDelay != 0);
}

// Do delay for nSecs microseconds
void Delay_us(uint32_t uSecs) {
	SysTick->LOAD = (uCountFreq & SysTick_LOAD_RELOAD_Msk) - 1;
	TimingDelay = uSecs;
	while (TimingDelay != 0);
}
