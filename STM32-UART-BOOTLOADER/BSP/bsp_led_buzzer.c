#include "bsp_led_buzzer.h"


// LED1-->PA0  LED2-->PA1  共阳接法
// BUZZER-->PB5
void BSP_LED_BUZZER_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 	
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOG , ENABLE);  //使能PA,PG端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;// LED1-->PA0  LED2-->PA1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	   //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	           //BUZZER-->PB5 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	   //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      
    GPIO_Init(GPIOG, &GPIO_InitStructure);                 

    //GPIO_SetBits(GPIOA,GPIO_Pin_0);                        //LED1熄灭	
    GPIO_SetBits(GPIOA,GPIO_Pin_1);                        //LED2熄灭		
    GPIO_ResetBits(GPIOG, GPIO_Pin_8);			   //PB5 输出低 关闭蜂鸣器
    
    GPIO_ResetBits(GPIOA,GPIO_Pin_0);
    //GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}

