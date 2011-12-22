
#include "at45db161d.h"
#include "dataflash.h"
/*****下面是全局变量定义******/
static u16 g_CurReadPage;//当前读的页地址
static u16 g_CurReadByte;//当前读的字节(页中地址)
static u16 g_CurWritePage;//当前写的页地址
static u16 g_CurWriteByte;//当前写的字节地址(页中地址)

/*****下面是内部调用的接口函数******/

//从SPI口输出一字节数据
 u8 spi_write(u8 data)
{

  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, data);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}


//检测并等待器件忙状态,8引脚封闭器件没有 RDY/BUSY引脚 为些通过读状态寄存器来检测忙状态
static void df_wait_busy(void)
{
u8 FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */
  spi_write(STATUS_REGISTER);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = spi_write(0xa5);

  }
  while ((FLASH_Status & 0x80) == RESET); /* Busy in progress */

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

//读主存储器指定页到读缓冲区(BUFFER1)
static void load_page_to_buffer(u16 page,u8 buffer)
{
SPI_FLASH_CS_LOW();

if(buffer == DF_READ_BUFFER) 
spi_write(MM_PAGE_TO_B1_XFER); 
else
spi_write(MM_PAGE_TO_B2_XFER);
spi_write((u8)(page >> 6));
spi_write((u8)(page << 2));
spi_write(0x00);

SPI_FLASH_CS_HIGH();

df_wait_busy();
}

//将写缓冲区内容写入到主存储器中指定页
static void write_page_from_buffer(u16 page,u8 buffer)
{
SPI_FLASH_CS_LOW();

if(buffer ==DF_WRITE_BUFFER) 
spi_write(B2_TO_MM_PAGE_PROG_WITH_ERASE);
else
spi_write(B1_TO_MM_PAGE_PROG_WITH_ERASE);
spi_write((u8)(page>>6));
spi_write((u8)(page<<2));
spi_write(0x00); // don't cares

SPI_FLASH_CS_HIGH();

df_wait_busy();
}

//从读缓冲区读数据
static void read_buffer(u16 addr,u8 *data,u8 size)
{
u8 i;

SPI_FLASH_CS_LOW();

spi_write(BUFFER_1_READ);
spi_write(0x00); 
spi_write((u8)(addr>>8));
spi_write((u8)addr); 
for(i=0;i<size;i++) 
data[i]=spi_write(0);

SPI_FLASH_CS_HIGH();
}

//将数据写入写缓冲区
static void write_buffer(u16 addr,u8 *data,u8 size)
{
u8 i;

SPI_FLASH_CS_LOW();

spi_write(BUFFER_2_WRITE);
spi_write(0x00); 
spi_write((u8)(addr>>8));
spi_write((u8)addr); 
for(i=0;i<size;i++) 
spi_write(data[i]);

SPI_FLASH_CS_HIGH();
}

/*****下面是为外部调用而提供的接口函数******/

void SPI_FLASH_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable SPI1 and GPIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure SPI1 pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
								  
  /* Configure I/O for Flash Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* SPI1 configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}

//读初始化功能函数,addr为打开后读到的初始地址
void df_read_open(u32 addr)
{
g_CurReadPage=addr/DF_PAGE_SIZE; 
g_CurReadByte=addr%DF_PAGE_SIZE;
load_page_to_buffer(g_CurReadPage,DF_READ_BUFFER);
}

void df_write_open(u32 addr)
{
g_CurWritePage=addr/DF_PAGE_SIZE;
g_CurWriteByte=addr%DF_PAGE_SIZE;
load_page_to_buffer(g_CurWritePage,DF_WRITE_BUFFER);
}

u8 df_getc(void)
{
u8 c;

read_buffer(g_CurReadByte,&c,1);
g_CurReadByte++;
if(g_CurReadByte ==DF_PAGE_SIZE)
{
g_CurReadPage++;
load_page_to_buffer(g_CurReadPage,DF_READ_BUFFER);
g_CurReadByte=0;
}

return c;
}

void df_putc(u8 c)
{
write_buffer(g_CurWriteByte,&c,1);
g_CurWriteByte++;
if(g_CurWriteByte == DF_PAGE_SIZE)
{
g_CurWriteByte=0;
write_page_from_buffer(g_CurWritePage,DF_WRITE_BUFFER);
g_CurWritePage++;
load_page_to_buffer(g_CurWritePage,DF_WRITE_BUFFER);
}
}

void df_read(u8 *buf,u8 size)
{
u8 temp;
if((g_CurReadByte + size) > DF_PAGE_SIZE) //如果当前页未读取数据不够size字节
{
//读当前页剩余数据
temp=DF_PAGE_SIZE - g_CurReadByte;
read_buffer(g_CurReadByte,buf,temp);

//装入下一页
load_page_to_buffer(++g_CurReadPage,DF_READ_BUFFER);

//从下一页读剩余数据
g_CurReadByte=size-temp;
read_buffer(0,buf+temp,g_CurReadByte);
}
else //如果当前页数据有size字节
{
read_buffer(g_CurReadByte,buf,size);
g_CurReadByte+=size;

//如果当前页数据已全部读完
if(g_CurReadByte==DF_PAGE_SIZE)
{
load_page_to_buffer(++g_CurReadPage,DF_READ_BUFFER);
g_CurReadByte=0; 
}
} 
}

void df_write(u8 *buf,u8 size)
{
u8 temp;

if((g_CurWriteByte + size) > DF_PAGE_SIZE) //如果当前页未写空间不够size字节
{
//写当前页剩余空间的数据
temp=DF_PAGE_SIZE - g_CurWriteByte;
write_buffer(g_CurWriteByte,buf,temp);
//保存当前页
write_page_from_buffer(g_CurWritePage,DF_WRITE_BUFFER);
g_CurWritePage++;
load_page_to_buffer(g_CurWritePage,DF_WRITE_BUFFER);

//写入到下一页对应缓冲区
g_CurWriteByte=size-temp;
write_buffer(0,buf+temp,g_CurWriteByte);
}
else
{
write_buffer(g_CurWriteByte,buf,size);
g_CurWriteByte+=size;

//缓冲已满,写入到主存储区
if(g_CurWriteByte==DF_PAGE_SIZE)
{
g_CurWriteByte=0;
write_page_from_buffer(g_CurWritePage,DF_WRITE_BUFFER);
g_CurWritePage++; 
load_page_to_buffer(g_CurWritePage,DF_WRITE_BUFFER); 
}
} 
}

//调整写指针
void df_read_seek(u32 addr)
{
 df_read_close();
 df_read_open(addr);
}

//调整读指针
void df_write_seek(u32 addr)
{
 df_write_close();
 df_write_open(addr); 
}

void df_read_close(void)
{
 //此处不做任何处理
}

void df_write_close(void)
{
 write_page_from_buffer(g_CurWritePage,DF_WRITE_BUFFER); //缓冲区内容写入到主存储器
}

