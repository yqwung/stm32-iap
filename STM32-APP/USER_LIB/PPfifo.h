#ifndef _PPFIFO_
#define _PPFIFO_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdio.h>
#include "string.h"
#include <stdarg.h>
#include <stdlib.h>

#pragma pack (1)
  
typedef struct ppfifo
{
    unsigned char  mode;
    unsigned int   buffer_toal;
    unsigned int   buffer_len_1;
    unsigned int   buffer_len_2;
    unsigned char* buffer_1;    
    unsigned char* buffer_2;    
    unsigned char* ppdata;
	 
}ppfifo_t,*pppfifo_t;

ppfifo_t* ppfifo_alloc(unsigned int vsize);
void ppfifo_free(ppfifo_t *ppfifo);
unsigned int ppfifo_in(ppfifo_t *fifo,unsigned char *buf, unsigned int len);
unsigned int ppfifo_out(ppfifo_t *fifo,unsigned char *buf);

#pragma pack ()

#ifdef __cplusplus
}
#endif

#endif

