/*
 * File      : USART_rt.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 */

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_usart.h"
#include <stm32f10x_dma.h>

/*
 * Use UART1 as console output and finsh input
 * interrupt Rx and poll Tx (stream mode)
 *
 * Use UART2 with interrupt Rx and poll Tx
 * Use UART3 with DMA Tx and interrupt Rx -- DMA channel 2
 *
 * USART DMA setting on STM32
 * USART1 Tx --> DMA Channel 4
 * USART1 Rx --> DMA Channel 5
 * USART2 Tx --> DMA Channel 7
 * USART2 Rx --> DMA Channel 6
 * USART3 Tx --> DMA Channel 2
 * USART3 Rx --> DMA Channel 3
 */

 /*
#ifdef USING_UART1
struct stm32_serial_int_rx uart1_int_rx;
struct stm32_serial_device uart1 =
{
	USART1,
	&uart1_int_rx,
	NULL
};
struct rt_device uart1_device;
#endif

#ifdef USING_UART2
struct stm32_serial_int_rx uart2_int_rx;
struct stm32_serial_device uart2 =
{
	USART2,
	&uart2_int_rx,
	NULL
};
struct rt_device uart2_device;
#endif

#ifdef USING_UART3
struct stm32_serial_int_rx uart3_int_rx;
struct stm32_serial_dma_tx uart3_dma_tx;
struct stm32_serial_device uart3 =
{
	USART3,
	&uart3_int_rx,
	&uart3_dma_tx
};
struct rt_device uart3_device;
#endif
*/

#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804

/* USART1_REMAP = 0 */
#define UART1_GPIO_TX		GPIO_Pin_9
#define UART1_GPIO_RX		GPIO_Pin_10
#define UART1_GPIO			GPIOA
#define RCC_APBPeriph_UART1	RCC_APB2Periph_USART1
#define UART1_TX_DMA		DMA1_Channel4
#define UART1_RX_DMA		DMA1_Channel5

#if defined(STM32F10X_LD) || defined(STM32F10X_MD) || defined(STM32F10X_CL)
#define UART2_GPIO_TX	    GPIO_Pin_5
#define UART2_GPIO_RX	    GPIO_Pin_6
#define UART2_GPIO	    	GPIOD
#define RCC_APBPeriph_UART2	RCC_APB1Periph_USART2
#else /* for STM32F10X_HD */
/* USART2_REMAP = 0 */
#define UART2_GPIO_TX		GPIO_Pin_2
#define UART2_GPIO_RX		GPIO_Pin_3
#define UART2_GPIO			GPIOA
#define RCC_APBPeriph_UART2	RCC_APB1Periph_USART2
#define UART2_TX_DMA		DMA1_Channel7
#define UART2_RX_DMA		DMA1_Channel6
#endif

/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_RX		GPIO_Pin_11
#define UART3_GPIO_TX		GPIO_Pin_10
#define UART3_GPIO			GPIOB
#define RCC_APBPeriph_UART3	RCC_APB1Periph_USART3
#define UART3_TX_DMA		DMA1_Channel2
#define UART3_RX_DMA		DMA1_Channel3


/* USART3_REMAP[1:0] = 00 */
#define UART4_GPIO_RX		GPIO_Pin_11
#define UART4_GPIO_TX		GPIO_Pin_10
#define UART4_GPIO			GPIOC
#define RCC_APBPeriph_UART4	RCC_APB1Periph_UART4



static void BSP_USART_RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

#ifdef USING_UART1
	/* Enable USART1 and GPIOA clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
#endif

#ifdef USING_UART2

#if (defined(STM32F10X_LD) || defined(STM32F10X_MD) || defined(STM32F10X_CL))
    /* Enable AFIO and GPIOD clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);

    /* Enable the USART2 Pins Software Remapping */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#else
    /* Enable AFIO and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
#endif

	/* Enable USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif

#ifdef USING_UART3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/* Enable USART3 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

#ifdef USING_UART4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	/* Enable USART3 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/* DMA clock enable */
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
}

static void BSP_USART_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USING_UART1
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
#endif

#ifdef USING_UART2
	/* Configure USART2 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

	/* Configure USART2 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
#endif

#ifdef USING_UART3

	/* Configure USART3 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

	/* Configure USART3 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
#endif

#ifdef USING_UART4

	/* Configure USART3 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = UART4_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

	/* Configure USART4 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART4_GPIO, &GPIO_InitStructure);
#endif
}

static void BSP_USART_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

#ifdef USING_UART1
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef USING_UART2
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef USING_UART3
	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the DMA1 Channel2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef USING_UART4
	/* Enable the USART4 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

}

static void BSP_USART_DMA_Configuration(void)
{
//#if defined (USING_UART3)
#if defined (USING_UART3)
	DMA_InitTypeDef DMA_InitStructure;

	/* fill init structure */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	/* DMA1 Channel5 (triggered by USART3 Tx event) Config */
	DMA_DeInit(UART3_TX_DMA);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Base;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_Init(UART3_TX_DMA, &DMA_InitStructure);
	DMA_ITConfig(UART3_TX_DMA, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(DMA1_FLAG_TC5);
#endif
}

/*
 * Init all related hardware in here
 * rt_bsp_serial_init() will register all supported USART device
 */
void BSP_USART_Init()
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;

	BSP_USART_RCC_Configuration();
	BSP_USART_GPIO_Configuration();
	BSP_USART_NVIC_Configuration();
	//BSP_USART_DMA_Configuration();

	/* uart init */
#ifdef USING_UART1
        USART_DeInit(USART1);  //复位串口1
	USART_InitStructure.USART_BaudRate = BAUD1;        
 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;        
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStructure);

	/* enable interrupt */
	//BSP_IntVectSet(BSP_INT_ID_USART1, BSP_USART1_ISR);
	//BSP_IntEn(BSP_INT_ID_USART1);
	//USART_ClearITPendingBit(USART1, USART_IT_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        //开启PE错误接收中断Bit 8PEIE: PE interrupt enable
//---lanxb    
	USART_ITConfig(USART1, USART_IT_PE, ENABLE);
	//CR2 开启ERR中断
	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
//---lanxb	
	//USART_ITConfig(USART1, USART_IT_TC, ENABLE);
        
	BSP_USART1_Init();       
        USART_Cmd(USART1, ENABLE);
#endif

#ifdef USING_UART2
        USART_DeInit(USART2);  //复位串口1
	USART_InitStructure.USART_BaudRate = BAUD2;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStructure);

	/* Enable USART2 DMA Rx request */
	//BSP_IntVectSet(BSP_INT_ID_USART2, BSP_USART2_ISR);
	//BSP_IntEn(BSP_INT_ID_USART2);
	//USART_ClearITPendingBit(USART2, USART_IT_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        //---lanxb    
	USART_ITConfig(USART2, USART_IT_PE, ENABLE);
	//CR2 开启ERR中断
	USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
//---lanxb
	//USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	USART_Cmd(USART2, ENABLE);
	BSP_USART2_Init();
#endif

#ifdef USING_UART3
        USART_DeInit(USART3);  //复位串口1
	USART_InitStructure.USART_BaudRate = BAUD3;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(USART3, &USART_InitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStructure);

	/* Enable USART2 DMA Rx request */
	//BSP_IntVectSet(BSP_INT_ID_USART3, BSP_USART3_ISR);
	//BSP_IntEn(BSP_INT_ID_USART3);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//---lanxb	  
	USART_ITConfig(USART3, USART_IT_PE, ENABLE);
	//CR2 开启ERR中断
	USART_ITConfig(USART3, USART_IT_ERR, ENABLE);
//---lanxb
	USART_Cmd(USART3, ENABLE);
	BSP_USART3_Init();
#endif

#ifdef USING_UART4
        USART_DeInit(UART4);  //复位串口1
	USART_InitStructure.USART_BaudRate = BAUD4;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(UART4, &USART_InitStructure);
	USART_ClockInit(UART4, &USART_ClockInitStructure);

	/* Enable USART2 DMA Rx request */
	//BSP_IntVectSet(BSP_INT_ID_USART4, BSP_USART4_ISR);
	//BSP_IntEn(BSP_INT_ID_USART4);

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
//---lanxb	  
	USART_ITConfig(UART4, USART_IT_PE, ENABLE);
	//CR2 开启ERR中断
	USART_ITConfig(UART4, USART_IT_ERR, ENABLE);
//---lanxb
	USART_Cmd(UART4, ENABLE);
	BSP_USART4_Init();
#endif

//#ifdef USING_UART3
#if 0
	USART_InitStructure.USART_BaudRate = BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_Init(USART3, &USART_InitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStructure);

	//uart3_dma_tx.dma_channel = UART3_TX_DMA;

	/* Enable USART3 DMA Tx request */
	//USART_DMACmd(USART3, USART_DMAReq_Tx , ENABLE);

	/* enable interrupt */
	BSP_IntVectSet(BSP_INT_ID_USART3, BSP_USART3_ISR);
	BSP_IntEn(BSP_INT_ID_USART3);
	USART_ClearITPendingBit(USART3, USART_IT_TC);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART3, USART_IT_TC, ENABLE);
	USART_Cmd(USART3, ENABLE);
#endif
}


//////////////////////////////////////////////////////////////////////////////////////
//
// usart1 write and read
//
//////////////////////////////////////////////////////////////////////////////////////
pppfifo_t usart1_ppfifo;
pvfifo_t usart1_vfifo;


//----------------------------------------------------------------------------
void BSP_USART1_Init(void)
{
	usart1_ppfifo =	ppfifo_alloc(1100);
	usart1_vfifo =	vfifo_alloc(1100);
}

unsigned int BSP_USART1_Write(char* data, unsigned int len)
{
    int i=0 ;   
    for(i=0;i<len;i++)
    {
        USART_SendData(USART1,data[i]);   
    }
   
    return len;
}

void USART1_IRQHandler(void)
{
	uint8_t chr;
//lanxb	
	uint8_t volatile ushTemp;
	//开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE   错误
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
	{//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
	ushTemp = USART_ReceiveData(USART1); //取出来扔掉
	USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
	if(USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
	{//同  @arg USART_IT_NE     : Noise Error interrupt
	USART_ClearFlag(USART1, USART_FLAG_NE);
	}
	if(USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET)
	{//同   @arg USART_IT_FE     : Framing Error interrupt
	USART_ClearFlag(USART1, USART_FLAG_FE);
	}
	if(USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET)
	{//同  @arg USART_IT_PE     : Parity Error interrupt
	USART_ClearFlag(USART1, USART_FLAG_PE);
	}	
//lanxb		
        
	if(USART_GetITStatus(USART1, USART_IT_RXNE))
	{
	       chr = USART_ReceiveData(USART1);
               ppfifo_in(usart1_ppfifo,&chr,1);
	}
	
	if(USART_GetITStatus(USART1, USART_IT_TXE))
	{
		USART_ClearITPendingBit(USART1, USART_IT_TXE);	
	}
}


unsigned int BSP_USART1_Printf(char* fmt, ...)
{
	va_list args;
	unsigned int len;
	char *buf = malloc(512);
		
	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	len = BSP_USART1_Write(buf, strlen(buf));
	free(buf);
	return len;
}


//////////////////////////////////////////////////////////////////////////////////////
//
// usart2 write and read
//
//////////////////////////////////////////////////////////////////////////////////////
pppfifo_t usart2_ppfifo;
pvfifo_t usart2_vfifo;

void BSP_USART2_Init(void)
{
    usart2_ppfifo = ppfifo_alloc(1100);
    usart2_vfifo = vfifo_alloc(1100);
}


unsigned int BSP_USART2_Write(char* data, unsigned int len)
{
    int i=0; 
    
    for(i=0;i<len;i++)
    {
        USART_SendData(USART2, data[i]);   
    }  
    
    return len;
}

void USART2_IRQHandler(void)
{
    uint8_t chr;	
    //lanxb 
    uint8_t volatile ushTemp;
    
    //开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE	错误
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
        ushTemp = USART_ReceiveData(USART2); //取出来扔掉
        USART_ClearFlag(USART2, USART_FLAG_ORE);
    }
    if(USART_GetFlagStatus(USART2, USART_FLAG_NE) != RESET)
    {//同  @arg USART_IT_NE 	: Noise Error interrupt
        USART_ClearFlag(USART2, USART_FLAG_NE);
    }
    if(USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET)
    {//同	@arg USART_IT_FE	 : Framing Error interrupt
        USART_ClearFlag(USART2, USART_FLAG_FE);
    }
    if(USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET)
    {//同  @arg USART_IT_PE 	: Parity Error interrupt
        USART_ClearFlag(USART2, USART_FLAG_PE);
    }	
    //lanxb 
    if(USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        chr = USART_ReceiveData(USART2);
        ppfifo_in(usart2_ppfifo,&chr,1);
    }
    if(USART_GetITStatus(USART2, USART_IT_TXE))
    {
        USART_ClearITPendingBit(USART2, USART_IT_TXE);
    }
}


unsigned int BSP_USART2_Printf(char* fmt, ...)
{
    va_list args;
    unsigned int len;
    char *buf = malloc(512);

    va_start(args, fmt);
    vsprintf(buf, fmt, args);
    va_end(args);
    len = BSP_USART2_Write(buf, strlen(buf));
    free(buf);

    return len;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// usart3 write and read
//
//////////////////////////////////////////////////////////////////////////////////////
pppfifo_t usart3_ppfifo;
pvfifo_t usart3_vfifo;

void BSP_USART3_Init(void)
{
    usart3_ppfifo = ppfifo_alloc(1100);
    usart3_vfifo = vfifo_alloc(1100);
}


unsigned int BSP_USART3_Write(char* data, unsigned int len)
{
    int i=0 ;   

    for(i=0;i<len;i++)
    {
        USART_SendData(USART3,data[i]);   
    }

    return len;
}

void USART3_IRQHandler(void)
{
	uint8_t chr;	
//lanxb 
	uint8_t volatile ushTemp;
        
	//开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE	错误
	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
	{//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
	ushTemp = USART_ReceiveData(USART3); //取出来扔掉
	USART_ClearFlag(USART3, USART_FLAG_ORE);
	}
	if(USART_GetFlagStatus(USART3, USART_FLAG_NE) != RESET)
	{//同  @arg USART_IT_NE 	: Noise Error interrupt
	USART_ClearFlag(USART3, USART_FLAG_NE);
	}
	if(USART_GetFlagStatus(USART3, USART_FLAG_FE) != RESET)
	{//同	@arg USART_IT_FE	 : Framing Error interrupt
	USART_ClearFlag(USART3, USART_FLAG_FE);
	}
	if(USART_GetFlagStatus(USART3, USART_FLAG_PE) != RESET)
	{//同  @arg USART_IT_PE 	: Parity Error interrupt
	USART_ClearFlag(USART3, USART_FLAG_PE);
	}	
//lanxb 	
	if(USART_GetITStatus(USART3, USART_IT_RXNE))
	{
		chr = USART_ReceiveData(USART3);
		ppfifo_in(usart3_ppfifo,&chr,1);

	}
	if(USART_GetITStatus(USART3, USART_IT_TXE))
	{
		USART_ClearITPendingBit(USART3, USART_IT_TXE);
		
	}
}

unsigned int BSP_USART3_Printf(char* fmt, ...)
{
	va_list args;
	unsigned int len;
	char *buf = malloc(512);
		
	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	len = BSP_USART3_Write(buf, strlen(buf));
	free(buf);
        
	return len;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// usart4 write and read
//
//////////////////////////////////////////////////////////////////////////////////////
pppfifo_t usart4_ppfifo;
pvfifo_t usart4_vfifo;

void BSP_USART4_Init(void)
{
    usart4_ppfifo = ppfifo_alloc(1100);
    usart4_vfifo = vfifo_alloc(1100);
}


unsigned int BSP_USART4_Write(char* data, unsigned int len)
{
    int i=0 ; 
    
    for(i=0;i<len;i++)
    {
        USART_SendData(UART4,data[i]);   
    }

    return len;
}


void UART4_IRQHandler(void)
{
	uint8_t chr;	
        //lanxb 
	uint8_t volatile ushTemp;
	//开启CR3,bit0的EIE: Error interrupt enable, 处理USART_IT_ERR,USART_IT_ORE_ER,USART_IT_NE,USART_IT_FE	错误
	if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) != RESET)
	{//同  @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set  
	ushTemp = USART_ReceiveData(UART4); //取出来扔掉
	USART_ClearFlag(UART4, USART_FLAG_ORE);
	}
	if(USART_GetFlagStatus(UART4, USART_FLAG_NE) != RESET)
	{//同  @arg USART_IT_NE 	: Noise Error interrupt
	USART_ClearFlag(UART4, USART_FLAG_NE);
	}
	if(USART_GetFlagStatus(UART4, USART_FLAG_FE) != RESET)
	{//同	@arg USART_IT_FE	 : Framing Error interrupt
	USART_ClearFlag(UART4, USART_FLAG_FE);
	}
	if(USART_GetFlagStatus(UART4, USART_FLAG_PE) != RESET)
	{//同  @arg USART_IT_PE 	: Parity Error interrupt
	USART_ClearFlag(UART4, USART_FLAG_PE);
	}	
//lanxb 
	if(USART_GetITStatus(UART4, USART_IT_RXNE))
	{
		chr = USART_ReceiveData(UART4);
		ppfifo_in(usart4_ppfifo,&chr,1);

	}
	if(USART_GetITStatus(UART4, USART_IT_TXE))
	{
		USART_ClearITPendingBit(UART4, USART_IT_TXE);	
	}
}


unsigned int BSP_USART4_Printf(char* fmt, ...)
{
    va_list args;
    unsigned int len;
    char *buf = malloc(512);

    va_start(args, fmt);
    vsprintf(buf, fmt, args);
    va_end(args);
    len = BSP_USART4_Write(buf, strlen(buf));
    free(buf);
    
    return len;
}
