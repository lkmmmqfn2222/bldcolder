#include "bsp.h"
#include "bldc.h"

uint8_t receive_dma[CIRCUL_BUF_LEN];
uint16_t adc_dma[ADC_CHANNEL_SIZE];

void uart1_init(u32 baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);

	// 	配置DMA,TDR寄存器填写.
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->TDR);  //外设寄存器地址
	//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SRC_Const_Buffer;//待发送缓冲区
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; //外设作为目标地址
	DMA_InitStructure.DMA_BufferSize = 0; //传输大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //源地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //每次发送都是Byte
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //单次传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //中等优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //不是内存到内存传输
	DMA_Init(DMA1_Channel2, &DMA_InitStructure); //初始化

	// 使能DMA USART传输
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//配置DMA,RDR寄存器填写.
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->RDR);  //外设寄存器地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&receive_dma); //待发送缓冲区
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //内存作为目标地址
	DMA_InitStructure.DMA_BufferSize = CIRCUL_BUF_LEN; //传输大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //源地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //每次发送都是Byte
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //中等优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //不是内存到内存传输
	DMA_Init(DMA1_Channel3, &DMA_InitStructure); //初始化

	// 使能DMA USART传输
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

void bldc_drv_init(void)
{
	
	/********************
	 *bridge control enable
	 *PB13 EN3 for PWM_CH1
	 *PB14 EN2 for PWM_CH2
	 *PB15 EN1 for PWM_CH3
	 ********************/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	/********************
	 *tim1 pwm out config 
	 *PA8	TIM1_CH1
	 *PA9	TIM1_CH2
	 *PA10  TIM1_CH3
	 *********************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	//AF TO TIM FUNCTION
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);  	// CH1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);  	// CH2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);  	// CH3
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseStructure.TIM_Period = 2399;//48m/2.4K=20K PWM
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = 1;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_SelectCOM(TIM1, ENABLE);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR2);//be triggered by tim3
	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM_Cmd(TIM1, ENABLE);
}

void hall_config(void)
{
	/**********************
	 *PB0 HALL_W
	 *PB4 HALL_U
	 *PB5 HALL_V
	 *@20180522 v1.0 using gpio nvic 
	 *@
	 **/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//external nvic
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line4|EXTI_Line5);
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0| EXTI_Line4 | EXTI_Line5;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

/****************************
 *tim16 is used as velocity calculating
 *48M/0.48K=100KHz
 *@100kHz=0.01ms
 *@65535*0.01ms=0.65536s for a cycle
 *
 *@systick=48M
 *@1/1000=1ms
 *@systick base = 1ms
 **/

void tim16_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 480 - 1;
	timerInitStructure.TIM_Period = 0xFFFF; 
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM16, &timerInitStructure);
	TIM_Cmd(TIM16, ENABLE);
	
	//0.65536s interrupt once
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM16_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
	NVIC_Init(&NVIC_InitStruct);
	TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);	
	
	
	if (SysTick_Config((SystemCoreClock) / 1000))
	{
		while (1)
			;
	}
	NVIC_SetPriority(SysTick_IRQn, 0xff);
}


volatile uint32_t _us_timex10 = 0;
volatile uint16_t ms_timer = 0;
volatile uint32_t sys_tick = 0;


/***************************************
 *systick is redefined by the function 
 *time16_init
 **/

void SysTick_Handler()
{
	sys_tick += 1;
	if (ms_timer > 0)
		ms_timer--;

	IWDG_ReloadCounter();
}

/*****************************************
 *systick=1ms see tim16_init function
 *
 **/
void ms_sleep(uint16_t time)
{
	ms_timer = time;
	while (ms_timer){ continue; }
	;
}

uint32_t system_time()
{
	return sys_tick;
}


/*****************************************
 *get the current value of tim16 
 *and then add to us timex10
 **/

uint32_t us_timex10()
{
	return _us_timex10 + (uint32_t)(TIM16->CNT);
}

/******************************************
 *once tim16 nvic was detected 
 *this timer countered 65536 times
 *
 **/
void TIM16_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
		_us_timex10 += 65535;
	}
}

/****************************************
 *
 *this function will be excuted once 
 *get a hallgpio interrupt
 *and get the count time to calculate
 *the velocity of motor
 **/

extern motor_t *motor;

void hall_interrupt(void)
{
	static u8 HallValue;
	static u32 HallTime;
	u16 HallNow = GPIO_ReadOutputData(GPIOB);
	u8 x = ((HallNow >> 4) & 0x01)+((HallNow>>4)&0x02)+((HallNow<<2)&0x03);
	u32 t = us_timex10();
	if (HallValue!=x)
	{
		HallValue = x; 
		bldc_hall_trigger(motor, HallNow, t - HallTime);
		HallTime = t;
	}
}


void phase_enable(char a,char b,char c)
{
	if (c)
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);

	if (b)
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);

	if (a)
		GPIO_SetBits(GPIOA, GPIO_Pin_15);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}


void EXTI0_1_IRQHandler()
{
	hall_interrupt();
	EXTI_ClearFlag(EXTI_Line0 );
	EXTI_ClearITPendingBit(EXTI_Line2 | EXTI_Line3);
}
void EXTI4_15_IRQHandler()
{
	hall_interrupt();
	EXTI_ClearFlag(EXTI_Line4 | EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line4 | EXTI_Line5);
}


void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}