#ifdef __cplusplus
extern "C" {
#endif
  
#include  "PPfifo.h"
  
#pragma pack (1)


ppfifo_t* ppfifo_alloc(unsigned int vsize)
{
    ppfifo_t* ppfifo;
    
    ppfifo=(ppfifo_t*)((unsigned char*)malloc(4 * vsize + sizeof(ppfifo_t)));  
    ppfifo->mode = 0;
    ppfifo->buffer_toal = vsize;
    ppfifo->buffer_len_1 = 0;
    ppfifo->buffer_len_2 = 0;
    ppfifo->buffer_1 = sizeof(ppfifo_t) + (unsigned char*)ppfifo;
    ppfifo->buffer_2 = sizeof(ppfifo_t) + (unsigned char*)ppfifo + vsize;
    ppfifo->ppdata = sizeof(ppfifo_t) + (unsigned char*)ppfifo + vsize + vsize ;
       
    memset((unsigned char *)ppfifo->buffer_1,0,vsize);
    memset((unsigned char *)ppfifo->buffer_2,0,vsize);
    memset((unsigned char *)ppfifo->ppdata,0,vsize);
    
    return ppfifo;
}

void ppfifo_free(ppfifo_t *ppfifo)
{   
    if(ppfifo)
    {
        free(ppfifo);
    }
}

unsigned int ppfifo_in(ppfifo_t *fifo,unsigned char *buf, unsigned int len)
{
    unsigned int i = 0;
    
    if(fifo->mode == 0)
    {
        if((fifo->buffer_toal - fifo->buffer_len_1) < len)
        {
            return 0;
        }
        for(i = 0; i < len; i ++)
        {
            fifo->buffer_1[fifo->buffer_len_1] = buf[i];
            fifo->buffer_len_1 ++;
        }
    }
    else
    {
        if((fifo->buffer_toal - fifo->buffer_len_2) < len)
        {
            return 0;
        }
        
        for(i = 0; i < len; i ++)
        {
            fifo->buffer_2[fifo->buffer_len_2] = buf[i];
            fifo->buffer_len_2 ++;
        }
    }
    
    return len;
}

unsigned int ppfifo_out(ppfifo_t *fifo,unsigned char *buf)
{
    unsigned int i = 0;
    
    if(fifo->mode == 0)
    {
        fifo->mode = 1;  // 有可能被中断
        for(i = 0; i < fifo->buffer_len_1; i ++)
        {
            buf[i] = fifo->buffer_1[i];
        }
        memset(fifo->buffer_1,0,i);
        fifo->buffer_len_1 = 0;
        
        return i;
    }
    else
    {
        fifo->mode = 0;   // 有可能被中断
        
        for(i = 0; i < fifo->buffer_len_2; i ++)
        {
            buf[i] = fifo->buffer_2[i];
        }
        memset(fifo->buffer_2,0,i);
        fifo->buffer_len_2 = 0;
        
        return i;
    }		
}


#pragma pack ()

#ifdef __cplusplus
}
#endif