#ifndef __IAP_AGREEMENT_H
#define __IAP_AGREEMENT_H	 


typedef struct _PACKAGE_INFO
{
    unsigned char  package_head[2];          // 同步头
    unsigned char  package_len[2];           // 包总长
    unsigned char  package_count[2];         // 包计数
    unsigned char  package_data[1030];       // 数据(数据中包含1Byte的CRC),实际1025

}PACKAGE_INFO, *pPACKAGE_INFO;


unsigned int iap_agreement_proc(unsigned char usart_x, unsigned char *buf_in, unsigned int len_in);


#endif