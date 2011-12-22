// at45db161d.h 

#ifndef AT45DB161D_H 
#define AT45DB161D_H 

//操作码定义 
//以下两个命令码定义中，如果单片机的SCK信号频率高于33MHz请选择后边两个命令码，一般AVR的SCK达不到33MHz为些选择0xd1和0xd3 
// buffer 1 read 
#define BUFFER_1_READ 0xd1 //0xD4 
// buffer 2 read 
#define BUFFER_2_READ 0xd3 //0xD6 

// buffer 1 write 
#define BUFFER_1_WRITE 0x84 

// buffer 2 write 
#define BUFFER_2_WRITE 0x87 

// buffer 1 to main memory page program with built-in erase 
#define B1_TO_MM_PAGE_PROG_WITH_ERASE 0x83 

// buffer 2 to main memory page program with built-in erase 
#define B2_TO_MM_PAGE_PROG_WITH_ERASE 0x86 

// buffer 1 to main memory page program without built-in erase 
#define B1_TO_MM_PAGE_PROG_WITHOUT_ERASE 0x88 

// buffer 2 to main memory page program without built-in erase 
#define B2_TO_MM_PAGE_PROG_WITHOUT_ERASE 0x89 

// main memory page program through buffer 1 
#define MM_PAGE_PROG_THROUGH_B1 0x82 

// main memory page program through buffer 2 
#define MM_PAGE_PROG_THROUGH_B2 0x85 

// auto page rewrite through buffer 1 
#define AUTO_PAGE_REWRITE_THROUGH_B1 0x58 

// auto page rewrite through buffer 2 
#define AUTO_PAGE_REWRITE_THROUGH_B2 0x59 

// main memory page compare to buffer 1 
#define MM_PAGE_TO_B1_COMP 0x60 

// main memory page compare to buffer 2 
#define MM_PAGE_TO_B2_COMP 0x61 

// main memory page to buffer 1 transfer 
#define MM_PAGE_TO_B1_XFER 0x53 

// main memory page to buffer 2 transfer 
#define MM_PAGE_TO_B2_XFER 0x55 

// DataFlash status register for reading density, compare status, 
// and ready/busy status 
#define STATUS_REGISTER 0xD7 

// main memory page read 
#define MAIN_MEMORY_PAGE_READ 0x52 

// erase a 528 byte page 
#define PAGE_ERASE 0x81 

// erase 512 pages 
#define BLOCK_ERASE 0x50 

#endif 
