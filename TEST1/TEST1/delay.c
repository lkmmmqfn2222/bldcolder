#include "delay.h"
#include "bsp.h"

void delay_init()
{

#ifdef OS_CRITICAL_METHOD 	//if OS_CRITICAL_METHOD id defined 
	u32 reload; 				//it means that ucosII will be used
#endif
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 	//  HCLK/8
	fac_us = SystemCoreClock / 8000000; 	//1/8 of the system clock  

#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	reload = SystemCoreClock / 8000000; 		//每秒钟的计数次数 单位为K	   
	reload *= 1000000 / OS_TICKS_PER_SEC; //根据OS_TICKS_PER_SEC设定溢出时间
	//reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
	fac_ms = 1000 / OS_TICKS_PER_SEC; //代表ucos可以延时的最少单位	   
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;    	//开启SYSTICK中断
	SysTick->LOAD = reload;  	//每1/OS_TICKS_PER_SEC秒中断一次	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;    	//开启SYSTICK    
#else
	fac_ms = (u16)fac_us * 1000; // if not use ucos, every ms comprise of systick   
#endif
}


void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = nus*fac_us;  //load time to delay	  		 
	SysTick->VAL = 0x00;         //clear the counter
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;           //begin
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp&(1 << 16)))
		;//waiting
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;        //disable the counter
	SysTick->VAL = 0X00;        //clear the counter	 
}

void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms*fac_ms; //load time (SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;            //clear the counter
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;           //begin  
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp&(1 << 16)))
		;//waiting   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;        //disable the counter
	SysTick->VAL = 0X00;        //clear the counter  	    
}