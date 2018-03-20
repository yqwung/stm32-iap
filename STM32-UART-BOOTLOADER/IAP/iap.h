#ifndef __IAP_H
#define __IAP_H	
#include <stdint.h>
#include "sys_config.h"


#define  ApplicationAddress        APP_ADDRESS        // 应用程序起始地址, bootloader 64k


void iap_to_app(void);
void iap_write_appbin(uint32_t appxaddr, uint8_t *appbuf,uint32_t appsize);


#endif