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
#define UART1_GPIO		GPIOA
#define RCC_APBPeriph_UART1	RCC_APB2Periph_USART1
#define UART1_TX_DMA		DMA1_Channel4
#define UART1_RX_DMA		DMA1_Channel5

/* USART2_REMAP = 0 */
#if defined(STM32F10X_LD) || defined(STM32F10X_MD) || defined(STM32F10X_CL)
#define UART2_GPIO_TX	        GPIO_Pin_5
#define UART2_GPIO_RX	        GPIO_Pin_6
#define UART2_GPIO	    	GPIOD
#define RCC_APBPeriph_UART2	RCC_APB1Periph_USART2
#else /* for STM32F10X_HD */
#define UART2_GPIO_TX		GPIO_Pin_2
#define UART2_GPIO_RX		GPIO_Pin_3
#define UART2_GPIO		GPIOA
#define RCC_APBPeriph_UART2	RCC_APB1Periph_USART2
#define UART2_TX_DMA		DMA1_Channel7
#define UART2_RX_DMA		DMA1_Channel6
#endif

/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_RX		GPIO_Pin_11
#define UART3_GPIO_TX		GPIO_Pin_10
#define UART3_GPIO		GPIOB
#define RCC_APBPeriph_UART3	RCC_APB1Periph_USART3
#define UART3_TX_DMA		DMA1_Channel2
#define UART3_RX_DMA		DMA1_Channel3

/* USART4_REMAP[1:0] = 00 */
#define UART4_GPIO_RX		GPIO_Pin_11
#define UART4_GPIO_TX		GPIO_Pin_10
#define UART4_GPIO		GPIOC
#define RCC_APBPeriph_UART4	RCC_APB1Periph_UART4

void BSP_USART1_Init(void);
void BSP_USART2_Init(void);
void BSP_USART3_Init(void);
void BSP_USART4_Init(void);

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
    /* Configure USART4 Rx as input floating */
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

#ifdef USING_UART1
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef USING_UART2
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef USING_UART3
    /* Enable the USART3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable the DMA1 Channel2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef USING_UART4
    /* Enable the USART4 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
    USART_DeInit(USART1);
    
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
    USART_Init(USART1, &USART_InitStructure);
    USART_ClockInit(USART1, &USART_ClockInitStructure);
    
    /* enable interrupt */
    BSP_USART1_Init();
    ///BSP_IntVectSet(BSP_INT_ID_USART1, BSP_USART1_ISR);   //使用uCOS-III打开
    ///BSP_IntEn(BSP_INT_ID_USART1);                        //使用uCOS-III打开
    //USART_ClearITPendingBit(USART1, USART_IT_TC);   
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    USART_Cmd(USART1, ENABLE);
        
#endif

#ifdef USING_UART2
    USART_DeInit(USART2);
    
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
    USART_Init(USART2, &USART_InitStructure);
    USART_ClockInit(USART2, &USART_ClockInitStructure);
    
    /* Enable USART2 DMA Rx request */
    BSP_USART2_Init();
    //BSP_IntVectSet(BSP_INT_ID_USART2, BSP_USART2_ISR);
    //BSP_IntEn(BSP_INT_ID_USART2);
    //USART_ClearITPendingBit(USART2, USART_IT_TC);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART2, USART_IT_TC, ENABLE);
    USART_Cmd(USART2, ENABLE);
    
#endif

#ifdef USING_UART3
    
    USART_DeInit(USART3);
    
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
    
    /* Enable USART3 DMA Rx request */
    BSP_USART3_Init();
    //BSP_IntVectSet(BSP_INT_ID_USART3, BSP_USART3_ISR);
    //BSP_IntEn(BSP_INT_ID_USART3);   
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    
    USART_Cmd(USART3, ENABLE);
    
#endif

#ifdef USING_UART4
    
    USART_DeInit(UART4);
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
    USART_Init(UART4, &USART_InitStructure);
    USART_ClockInit(UART4, &USART_ClockInitStructure);
    
    /* Enable USART4 DMA Rx request */
    BSP_USART4_Init();
    //BSP_IntVectSet(BSP_INT_ID_USART4, BSP_USART4_ISR);
    //BSP_IntEn(BSP_INT_ID_USART4);
        
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    
    USART_Cmd(UART4, ENABLE);
    
#endif

//#ifdef USING_UART3
#ifdef 0
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
    //BSP_IntVectSet(BSP_INT_ID_USART3, BSP_USART3_ISR);
    //BSP_IntEn(BSP_INT_ID_USART3);
    USART_ClearITPendingBit(USART3, USART_IT_TC);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    USART_Cmd(USART3, ENABLE);
#endif
}

/***************************************************************************************
*
*  USART1  ---   调试用
*
***************************************************************************************/
#if 1

ppfifo_t*    usart1_ppreceive;
kfifo_t*     usart1_receive;

//kfifo_t *usart1_tx_fifo;
//kfifo_t *usart1_rx_fifo;

void BSP_USART1_Init(void)
{
    //usart1_tx_fifo = kfifo_alloc(512);
    //usart1_rx_fifo = kfifo_alloc(512);
    
    usart1_ppreceive = ppfifo_alloc(512);   //接触摸屏
    usart1_receive = kfifo_alloc(512);
}

unsigned int BSP_USART1_Write(char* data, unsigned int len)
{
    // USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    // len = kfifo_in(usart1_tx_fifo, (unsigned char*)data, len);
    // USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    int i ;
    
    for(i=0;i<len;i++)
    {
        USART_SendData(USART1,data[i]); 
        while( USART_GetFlagStatus(USART1, USART_FLAG_TC)!= SET);      //等待发送完成      
    }
    
    return len;
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

void USART1_IRQHandler(void)                	             //串口1中断服务程序
{
    unsigned char chr;

#if 0                                                        //使用FIFO---接收
    if(USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        chr = USART_ReceiveData(USART1);
        kfifo_in(usart1_rx_fifo, &chr, 1);
    }
#endif
    
#if 1                                                        //使用乒乓FIFO---接收
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    //接收中断
    {
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);       //清除中断标志.
        chr = USART_ReceiveData(USART1);
        USART_SendData(USART1,chr); 
        ppfifo_in(usart1_ppreceive, &chr, 1);                //字符串命令
    }   
#endif 
    
#if 0                                                        //发送
    if(USART_GetITStatus(USART1, USART_IT_TXE))
    {
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
        
        if(kfifo_out(usart1_tx_fifo, &chr, 1))
        {
            USART_SendData(USART1, chr);
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
#endif
}
#endif

/***************************************************************************************
*
*  USART2   ---   和主板通信
*
***************************************************************************************/
#if 1

ppfifo_t*    usart2_ppreceive;
kfifo_t*     usart2_receive;

//kfifo_t *usart2_tx_fifo;
//kfifo_t *usart2_rx_fifo;

void BSP_USART2_Init(void)
{
    //usart2_tx_fifo = kfifo_alloc(256);
    //usart2_rx_fifo = kfifo_alloc(256);
    
    usart2_ppreceive = ppfifo_alloc(512);      //接串口矩阵板
    usart2_receive = kfifo_alloc(512);
}

unsigned int BSP_USART2_Write(char* data, unsigned int len)
{
    //USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    //len = kfifo_in(usart2_tx_fifo, (unsigned char*)data, len);
    //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  
    int i ;
    
    for(i=0;i<len;i++)
    {
        USART_SendData(USART2,data[i]); 
        while( USART_GetFlagStatus(USART2, USART_FLAG_TC)!= SET);      //等待发送完成      
    }
    
    return len;
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

void USART2_IRQHandler(void)                	             //串口2中断服务程序
{
    unsigned char chr;

#if 0    
    if(USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        chr = USART_ReceiveData(USART2);
        kfifo_in(usart2_rx_fifo, &chr, 1);
    }
#endif
    
#if 1
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)    //接收中断
    {
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);       //清除中断标志.
        chr = USART_ReceiveData(USART2);
        USART_SendData(USART3,chr);
        
        ppfifo_in(usart2_ppreceive, &chr, 1);
    }   
#endif 
 
#if 0
    if(USART_GetITStatus(USART2, USART_IT_TXE))             //发送
    {
        USART_ClearITPendingBit(USART2, USART_IT_TXE);
        
        if(kfifo_out(usart2_tx_fifo, &chr, 1))
        {
            USART_SendData(USART2, chr);
        }
        else
        {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
#endif
}
#endif

/***************************************************************************************
*
*  USART3    
*
***************************************************************************************/
#if 1

ppfifo_t*    usart3_ppreceive;
kfifo_t*     usart3_receive;

//kfifo_t *usart3_tx_fifo;
//kfifo_t *usart3_rx_fifo;

void BSP_USART3_Init(void)
{
    //usart3_tx_fifo = kfifo_alloc(256);
    //usart3_rx_fifo = kfifo_alloc(256);
    
    usart3_ppreceive = ppfifo_alloc(512);   //串口调试用
    usart3_receive = kfifo_alloc(512);
}

unsigned int BSP_USART3_Write(char* data, unsigned int len)
{
    //USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    //len = kfifo_in(usart3_tx_fifo, (unsigned char*)data, len);
    //USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    
    int i ;
    
    for(i=0;i<len;i++)
    {
        USART_SendData(USART3,data[i]); 
        while( USART_GetFlagStatus(USART3, USART_FLAG_TC)!= SET);      //等待发送完成      
    }
    
    return len;
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

void USART3_IRQHandler(void)                	             //串口3中断服务程序
{
    unsigned char chr;

#if 0    
    if(USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        chr = USART_ReceiveData(USART3);
        kfifo_in(usart3_rx_fifo, &chr, 1);
    }
#endif
    
#if 1
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    //接收中断
    {
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);       //清除中断标志.
        chr = USART_ReceiveData(USART3);
        BSP_USART3_Printf("usart3 : %d \r\n", chr);
        ppfifo_in(usart3_ppreceive, &chr, 1);
    }   
#endif 

#if 0                                                        //发送
    if(USART_GetITStatus(USART3, USART_IT_TXE))
    {
        USART_ClearITPendingBit(USART3, USART_IT_TXE);
        
        if(kfifo_out(usart3_tx_fifo, &chr, 1))
        {
            USART_SendData(USART3, chr);
        }
        else
        {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
#endif
}
#endif


/***************************************************************************************
*
*  USART4    
*
***************************************************************************************/
#if 1

ppfifo_t*    usart4_ppreceive;
kfifo_t*     usart4_receive;

//kfifo_t *usart4_tx_fifo;
//kfifo_t *usart3_rx_fifo;

void BSP_USART4_Init(void)
{
    //usart4_tx_fifo = kfifo_alloc(256);
    //usart4_rx_fifo = kfifo_alloc(256);
    
    usart4_ppreceive = ppfifo_alloc(512);   //串口调试用
    usart4_receive = kfifo_alloc(512);
}

unsigned int BSP_USART4_Write(char* data, unsigned int len)
{
    //USART_ITConfig(USART4, USART_IT_TXE, DISABLE);
    //len = kfifo_in(usart4_tx_fifo, (unsigned char*)data, len);
    //USART_ITConfig(USART4, USART_IT_TXE, ENABLE);
    
    int i ;
    
    for(i=0;i<len;i++)
    {
        USART_SendData(UART4,data[i]); 
        while( USART_GetFlagStatus(UART4, USART_FLAG_TC)!= SET);      //等待发送完成      
    }
    
    return len;
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

void UART4_IRQHandler(void)                	            //串口4中断服务程序
{
    unsigned char chr;

#if 0    
    if(USART_GetITStatus(UART4, USART_IT_RXNE))             //FIFO接收中断
    {
        chr = USART_ReceiveData(UART4);
        kfifo_in(usart4_rx_fifo, &chr, 1);
    }
#endif
    
#if 1
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)    //乒乓FIFO接收中断
    {
        USART_ClearITPendingBit(UART4,USART_IT_RXNE);       //清除中断标志.
        chr = USART_ReceiveData(UART4);
        ppfifo_in(usart4_ppreceive, &chr, 1);
    }   
#endif 

#if 0
    if(USART_GetITStatus(UART4, USART_IT_TXE))              //接收
    {
        USART_ClearITPendingBit(UART4, USART_IT_TXE);
        
        if(kfifo_out(usart4_tx_fifo, &chr, 1))
        {
            USART_SendData(UART4, chr);
        }
        else
        {
            USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
        }
    }
#endif
}
#endif