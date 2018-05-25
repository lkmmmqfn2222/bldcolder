#ifndef _BSP_H
#define _BSP_H

#include "include_config.h"

#define  CIRCUL_BUF_LEN		600
#define ADC_CHANNEL_SIZE 5
#define	 MAX_PWM_DUTY	2399

#define PWM(a,b,c) do{\
	TIM1->CCR1 = a;\
	TIM1->CCR2 = b; \
	TIM1->CCR3 = c; \
}while(0)



void ms_sleep(uint16_t time);
uint32_t system_time();
void bldc_drv_init(void);
void hall_config(void);
void tim16_init(void);
uint32_t us_timex10();
void phase_enable(char a, char b, char c);
void uart1_init(u32 baudrate);

#endif // !_BSP_H
