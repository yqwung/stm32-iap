#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "vfifo.h"

#pragma pack (1)

kfifo_t* kfifo_alloc(unsigned int vsize)   //创建一个循环队列
{
    kfifo_t* fifo;	
    
    fifo=(kfifo_t*)((unsigned char*)malloc(vsize+sizeof(kfifo_t)));   //用户分配的大小加上队列头的大小;
       
    fifo->data=sizeof(kfifo_t)+(unsigned char*)fifo;
    fifo->fifo_size=vsize;
    fifo->unused_size=vsize;
    fifo->in_addr=fifo->data;
    fifo->out_addr=fifo->data;
    fifo->read_lock=0;
    fifo->write_lock=0;
    memset((unsigned char *)fifo->data,0,vsize);
    
    return fifo;	
}

void kfifo_free(kfifo_t *fifo)    //释放一个循环队列
{	
    if(fifo)
    {
        kfree(fifo);
    }
}

unsigned int kfifo_in(kfifo_t *fifo,unsigned char *buf, unsigned int len)   //进队列
{
    unsigned int l,recycl;
    
    //LOCK
    if((fifo->write_lock > 0) ||(fifo->read_lock > 0))
    {
        return 0;
    }
    fifo->write_lock = 1;
    
    if(len<=0) 
    {
        fifo->write_lock = 0;
        
        return 0;
    }
    
    
    l=kmin(fifo->unused_size,len);  //判断要写入的数据是否超出最大空间;
    fifo->unused_size=fifo->unused_size-l;
    //如果超出就只写入一部分;
    
    //如果没有到队列尾部可以全部写入;
    if((l+fifo->in_addr)<(fifo->fifo_size+fifo->data)) 
    {
        memcpy(fifo->in_addr,buf,l);	//拷贝数据;
        fifo->in_addr=fifo->in_addr+l;  //把指针移动;
    }
    else
    {
        recycl=fifo->fifo_size+fifo->data-fifo->in_addr;
        memcpy(fifo->in_addr,buf,recycl);	//拷贝数据前半部分;
        memcpy(fifo->data,buf+recycl,l-recycl);
        fifo->in_addr=fifo->data+l-recycl;  //把指针移动;   
    }
    
    fifo->write_lock = 0;
    
    return l;
}


unsigned int kfifo_from_pp(kfifo_t *fifo,ppfifo_t *ppfifo)   //将乒乓区的数据移到 vFIFO
{
    unsigned int l=0, recycl=0;
    unsigned int len_1 = 0;
    unsigned int len_2 = 0;
    
    len_1 = ppfifo_out(ppfifo,ppfifo->ppdata);
    
    len_2 = ppfifo_out(ppfifo,(ppfifo->ppdata + len_1));
    if((len_1 == 0)&&(len_2 == 0))
    {
        return 0;
    }
        
    l=kmin(fifo->unused_size,len_1 + len_2);  //判断要写入的数据是否超出最大空间;
    fifo->unused_size=fifo->unused_size-l;
    
    if((l+fifo->in_addr)<(fifo->fifo_size+fifo->data)) 
    {
        memcpy(fifo->in_addr,ppfifo->ppdata,l);	//拷贝数据;
        fifo->in_addr=fifo->in_addr+l;  //把指针移动;
    }
    else
    {
        recycl=fifo->fifo_size+fifo->data-fifo->in_addr;
        memcpy(fifo->in_addr,ppfifo->ppdata,recycl);	//拷贝数据前半部分;
        memcpy(fifo->data,ppfifo->ppdata+recycl,l-recycl);
        fifo->in_addr=fifo->data+l-recycl;  //把指针移动;  
    }
    
    return l;
}


unsigned int kfifo_out(kfifo_t *fifo,unsigned char *buf, unsigned int len)    //出队列
{
    unsigned int l,recycl;
    
    
    if((fifo->write_lock > 0) ||(fifo->read_lock > 0))
    {
        return 0;
    }
    fifo->read_lock = 1;
    if(len<=0) 
    {
        fifo->read_lock = 0;
        
        return 0;
    }
    ///////////////////////;
    
    l=kmin(fifo->fifo_size-fifo->unused_size,len);
    fifo->unused_size=fifo->unused_size+l;
    //  l  是总共能取出去的大小;
    
    //如果没有到队列尾部可以全部读取;
    if((l+fifo->out_addr)<(fifo->fifo_size+fifo->data)) 
    {
        memcpy(buf,fifo->out_addr,l);	//拷贝数据;
        //memset(fifo->out_addr,0,l);
        fifo->out_addr=fifo->out_addr+l;  //把指针移动;
    }
    else
    {
        recycl=fifo->fifo_size+fifo->data-fifo->out_addr;
        memcpy(buf,fifo->out_addr,recycl);	//拷贝数据前半部分;
        //memset(fifo->out_addr,0,recycl);
        memcpy(buf+recycl,fifo->data,l-recycl);
        //memset(fifo->data,0,l-recycl);
        fifo->out_addr=l-recycl+fifo->data;  //把指针移动;
    }
    fifo->read_lock = 0;
    
    return l;
}


#pragma pack ()

#ifdef __cplusplus
}
#endif

