#include "iap_agreement.h"
#include "cmd_decode.h"
#include "iap.h"
#include "bsp_usart.h"
#include "cpu.h"


extern unsigned char update;         // 握手标志
extern unsigned char toApp;          // 跳转到APP标志


unsigned int package_count_old=0;    // 数据包计数上次
unsigned int package_count_new=0;    // 数据包计数本次


unsigned char package_handshake[8] = {0x55, 0x55,   0x00, 0x08,   0x00, 0x00,   0x55,    0xF9};  // 握手（应答0xAA握手命令，0xAA取反）
unsigned char package_ok[8] =        {0x55, 0x55,   0x00, 0x08,   0x00, 0x00,   0xA5,    0x00};  // 数据包正确，可以开始下次传输
unsigned char package_error[8] =     {0x55, 0x55,   0x00, 0x08,   0x00, 0x00,   0x5A,    0x00};  // 数据包错误，重传  
unsigned char package_end[8] =       {0x55, 0x55,   0x00, 0x08,   0x00, 0x00,   0x0A,    0x00};  // 数据包传输完毕（应答0xF5传输完毕命令，0xF5取反）
  

static unsigned int send_to_pc(unsigned char usart_x, unsigned char* data, unsigned int len)
{
    unsigned int send_len=0;
    
    switch(usart_x)
    {
        case 1 :    
            {
                send_len = BSP_USART1_Write(data, len);
            }
            break; 
        case 2 :    
            {
                send_len = BSP_USART2_Write(data, len);
            }
            break; 
        case 3 :    
            {
                send_len = BSP_USART3_Write(data, len);
            }
            break; 
        case 4 :    
            {
                send_len = BSP_USART4_Write(data, len);
            }
            break; 
        default:
            break;            
    }
    
    return send_len;
}


unsigned int iap_agreement_proc(unsigned char usart_x, unsigned char *buf_in, unsigned int len_in)
{
    unsigned int crc=1;
                
    package_count_new = (unsigned int)(((PACKAGE_INFO*)(buf_in))->package_count[0]<<8) | ((PACKAGE_INFO*)(buf_in))->package_count[1];  // 计算包计数
    
    if( len_in == 8 && ((PACKAGE_INFO*)(buf_in))->package_data[0] == 0xAA )       // 握手命令
    {
        send_to_pc(usart_x, package_handshake, 8);  // 应答握手命令
        
        update = 0;                                 // 握手成功，停止定时器
        
        package_count_old = 1;
    }
    else if( len_in == 8 && ((PACKAGE_INFO*)(buf_in))->package_data[0] == 0xF5 )  // 传输完毕，最后一包
    {                    
        package_end[4] = (unsigned char)(package_count_new>>8);  // 包计数高8位
        package_end[5] = (unsigned char)package_count_new;       // 包计数低8位    
        package_end[7] = CMD_CRC_Data(package_end, 7);           // CRC

        send_to_pc(usart_x, package_end, 8);        // 应答数据包传输完毕命令
        
        delay_ms(100);
        
        toApp = 1;    // 跳转到APP标志
    }
    else if( len_in == 1031  )                                                    // 文件数据传输
    {           
        if( package_count_new == package_count_old )
        {
            crc = CMD_CRC_Check(buf_in, len_in);  // 计算数据包校验和CRC 
            if(crc == 0)   // CRC正确，发送可以接收下一包命令
            {                                                        
                iap_write_appbin(ApplicationAddress+1024*(package_count_new-1), ((PACKAGE_INFO*)(buf_in))->package_data, 1024);   // 接收一包即写到FLASH                            
                package_ok[4] = (unsigned char)(package_count_old>>8);     // 包计数高8位
                package_ok[5] = (unsigned char)package_count_old;          // 包计数低8位    
                package_ok[7] = CMD_CRC_Data(package_ok, 7);               // CRC                
                
                send_to_pc(usart_x, package_ok, 8); 
                                                                                               
                package_count_old ++; 
            }
            else          // CRC错误，重传第package_count_old包
            {
                package_error[4] = (unsigned char)(package_count_old>>8);  // 包计数高8位
                package_error[5] = (unsigned char)package_count_old;       // 包计数低8位    
                package_error[7] = CMD_CRC_Data(package_error, 7);         // CRC
                
                send_to_pc(usart_x, package_error, 8);                   
            }
        } 
        else             // 包计数错误，重传第package_count_old包
        {
            package_error[4] = (unsigned char)(package_count_old>>8);      // 包计数高8位
            package_error[5] = (unsigned char)package_count_old;           // 包计数低8位    
            package_error[7] = CMD_CRC_Data(package_error, 7);             // CRC
                
            send_to_pc(usart_x, package_error, 8); 
        }
    }
   
    return 0;
}