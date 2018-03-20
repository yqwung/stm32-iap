/*
 * File      : usart.h
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
 */

#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include  <stm32f10x_conf.h>
#include  "vfifo.h"
#include  "ppfifo.h"
//#include  "unmap_table.h"

#define	USING_UART1
#define	USING_UART2
#define	USING_UART3
#define	USING_UART4

#define	BAUD1			115200
#define	BAUD2			115200
#define	BAUD3		        115200
#define	BAUD4			115200


void BSP_USART_Init(void);

////////////////////////////////////////////////////////////////////////////
extern pppfifo_t usart1_ppfifo;
extern pvfifo_t usart1_vfifo;
//extern pUNMAP_TABLE_HEAD usart1_punmap;

void BSP_USART1_Init(void);
unsigned int BSP_USART1_Write(char* data, unsigned int len);
unsigned int BSP_USART1_Printf(char* fmt, ...);
void BSP_USART1_ISR(void);


////////////////////////////////////////////////////////////////////////////
extern pppfifo_t usart2_ppfifo;
extern pvfifo_t usart2_vfifo;
//extern pUNMAP_TABLE_HEAD usart2_punmap;

void BSP_USART2_Init(void);
unsigned int BSP_USART2_Write(char* data, unsigned int len);
unsigned int BSP_USART2_Printf(char* fmt, ...);
void BSP_USART2_ISR(void);


////////////////////////////////////////////////////////////////////////////
extern pppfifo_t usart3_ppfifo;
extern pvfifo_t usart3_vfifo;
//extern pUNMAP_TABLE_HEAD usart3_punmap;

void BSP_USART3_Init(void);
unsigned int BSP_USART3_Write(char* data, unsigned int len);
unsigned int BSP_USART3_Printf(char* fmt, ...);
void BSP_USART3_ISR(void);


////////////////////////////////////////////////////////////////////////////
extern pppfifo_t usart4_ppfifo;
extern pvfifo_t usart4_vfifo;
//extern pUNMAP_TABLE_HEAD usart4_punmap;

void BSP_USART4_Init(void);
unsigned int BSP_USART4_Write(char* data, unsigned int len);
unsigned int BSP_USART4_Printf(char* fmt, ...);
void BSP_USART4_ISR(void);


#endif
