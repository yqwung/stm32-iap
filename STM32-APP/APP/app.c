#include <stm32f10x_conf.h>
#include "includes.h"


int main(void)
{
    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);      //设置中断向量表指向  SCB->VTOR = FLASH_BASE | 0x20000;  
    //SCB->VTOR = FLASH_BASE | 0x010000; 
    
    unsigned int  rev = 1;
  
    //------------------------------------------------------System Init----------------------------------------------------------//
    NVIC_Configuration();
    SystemInit();
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);      //设置中断向量表指向  SCB->VTOR = FLASH_BASE | 0x20000;
    delay_init();
    BSP_LED_BUZZER_Init();
    BSP_USART_Init();
    
    
    //-------------------------------------------------------USART Test----------------------------------------------------------//
    rev = 1;
    BSP_USART1_Printf("-STM32 IAP For IAR ( 2015-08-03 ) \r\n");
    BSP_USART1_Printf("This is App for STM32   \r\n");
    BSP_USART1_Printf("Version number : %d \r\n", rev);

    delay_ms(1000);   
       
    //--------------------------------------------------------while(1)-----------------------------------------------------------//
    while(1)
    {
        Kick_Dog();    // bootloader里面打开了看门狗，这里要及时喂狗
      
        delay_ms(1000);
        BSP_USART1_Printf("This is App!\r\n");      
    }
}