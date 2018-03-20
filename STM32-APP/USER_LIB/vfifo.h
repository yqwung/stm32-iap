#ifndef __VFIFO_H__
#define __VFIFO_H__

#include <stdarg.h>
#include <stdio.h>
#include "string.h"

#include "PPfifo.h"

#ifdef __cplusplus
extern "C" {
#endif
#pragma pack (1)

//volatile

typedef struct kfifo
{
    unsigned char*	in_addr;
    unsigned char*	out_addr;
    unsigned int	mask;
    unsigned int	fifo_size;
    unsigned int	unused_size;
    unsigned int	read_lock;
    unsigned int	write_lock;
    unsigned char*	data;         //装的是真正数据的指针
         
}kfifo_t,*pkfifo_t;


#define kmalloc(size)					malloc(size)
#define kfree(ptr)					free(ptr)
#define kmin(a,b)					((a) >( b )?( b) : (a))
#define kfifo_reset(fifo)				((fifo)->in_addr = (fifo)->out_addr =  (fifo)->data)


kfifo_t* kfifo_alloc(unsigned int size);   //新建一个循环队列
void kfifo_free(kfifo_t *fifo);		   //释放一个循环队列

unsigned int kfifo_in(kfifo_t *fifo,unsigned  char *buf, unsigned int len);           //加入新的数据
unsigned int kfifo_from_pp(kfifo_t *fifo,ppfifo_t *ppfifo);
unsigned int kfifo_out_peek(kfifo_t *fifo, unsigned char *buf, unsigned int len);     //取出一定长度的数据，但是不从队列里面清除
unsigned int kfifo_out(kfifo_t *fifo, unsigned char *buf, unsigned int len);	      //取出一定长度数据，从队列里面清楚


#pragma pack ()

#ifdef __cplusplus
}
#endif

#endif
