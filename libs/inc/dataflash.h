//dataflash.h 

#ifndef DATAFLASH_H 
#define DATAFLASH_H 

#include "stm32f10x_conf.h"
#define DF_PAGE_SIZE 528 
#define DF_READ_BUFFER 1 
#define DF_WRITE_BUFFER 0


#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(GPIOA, GPIO_Pin_4)


#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(GPIOA, GPIO_Pin_4)

/* 本程序为AVR单片机SPI口访问AT45DB161D提供一组接口函数 
    通过这一组接口函数可非常容易的完成对AT45DB161D的读写操作 
    AT45DB161D按页组织和操作（读写擦等等）内部的FLASH存储器， 
    每页为528字节(特殊用途时可通过命令配置成512字节)，其内部共集成4096页，即4096*528=2162688字节 
    本程序将为您提供读写这2162688字节存储区的线性操作方法。使用这些函数，用户可不必考虑AT45DB161D 
    内部的存储器组织结构，如同读写一个文件一样进行读写操作。 
*/ 
/*这是SPI口初始化函数，它必须在所有这些接口函数调用之前得到调用*/ 
void SPI_FLASH_Init(void);
/*读操作初始化函数，addr指定接下来的读函数的开始读取位置 
程序内部维护一个当前读取计数器，用户每读一字节该计数器加一*/ 
void df_read_open(u32 addr); 
/*写操作初始化函数，addr指定接下来的写函数的开始写入位置 
程序内部维护一个当前写入计数器，用户每写入一个字节该计数器加一*/ 
void df_write_open(u32 addr); 

/*此函数从当前读位置读取一字节后返回，内部的读计数器加一*/ 
u8 df_getc(void); 
/*此函数向当前写位置写入一字节的数据，并使内部写计数器加一*/ 
void df_putc(u8 c); 
/*此函数从当前读位置读取size个字节的数据到缓冲区buf,并使内部读计数器加size*/ 
void df_read(u8 *buf,u8 size); 
/*此函数从缓冲区buf向当前写位置写入size字节的数据，并使内部写计数器加size*/ 
void df_write(u8 *buf,u8 size); 
/*调整当前读计数器，调用此函数前必须已调用df_read_open*/ 
void df_read_seek(u32 addr); 
/*调整当前写计数器,调用此函数前必须已调用df_write_open*/ 
void df_write_seek(u32 addr); 
/*关闭读操作*/ 
void df_read_close(void); 
/*关闭写操作，所有的写入操作完成后必须调用此函来结束写操作，以便数据能够完整的保存到AT45DB161D主存储器当中*/ 
void df_write_close(void); 

void df_read_deviceid(u8 *buf); 


#endif 
