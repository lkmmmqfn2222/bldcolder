#include "bsp.h"

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
	// 	配置DMA,RDR寄存器填写.
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

void pwm_tim_init()
{
	
}