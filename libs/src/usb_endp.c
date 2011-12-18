/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 07/05/2010
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/




/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#if 0

uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
extern  uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_out;
extern uint32_t USART_Rx_length;
extern uint8_t  USB_Tx_State;
/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
  if (USB_Tx_State == 1)
  {
    if (USART_Rx_length == 0) 
    {
      USB_Tx_State = 0;
    }
    else 
    {
      if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE){
        USB_Tx_ptr = USART_Rx_ptr_out;
        USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
        
        USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
        USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;    
      }
      else 
      {
        USB_Tx_ptr = USART_Rx_ptr_out;
        USB_Tx_length = USART_Rx_length;
        
        USART_Rx_ptr_out += USART_Rx_length;
        USART_Rx_length = 0;
      }
      
#ifdef USE_STM3210C_EVAL
      USB_SIL_Write(EP1_IN, &USART_Rx_Buffer[USB_Tx_ptr], USB_Tx_length);  
#else
      UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP1, USB_Tx_length);
      SetEPTxValid(ENDP1); 
#endif  
    }
  }
}
*/

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  uint16_t USB_Rx_Cnt;
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);
  
  /* USB data will be immediately processed, this allow next USB traffic beeing 
  NAKed till the end of the USART Xfet */
  
  USB_To_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);
  
#ifndef STM32F10X_CL
  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
#endif /* STM32F10X_CL */
}


/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#ifdef STM32F10X_CL
void INTR_SOFINTR_Callback(void)
#else
void SOF_Callback(void)
#endif /* STM32F10X_CL */
{
  static uint32_t FrameCount = 0;
  
  if(bDeviceState == CONFIGURED)
  {
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
    {
      /* Reset the frame counter */
      FrameCount = 0;
      
      /* Check the data to be sent through IN pipe */
      Handle_USBAsynchXfer();
    }
  }  
}

#endif

static volatile bool tx_in_progress = FALSE;
static volatile u16 rx_pma_count = 0;
static u8 rx_buffer[VIRTUAL_COM_PORT_DATA_SIZE];
static u16 rx_buffer_count = 0;
static u16 rx_buffer_index = 0;

static inline void Enable_USB_Interrupts(void)
{
  //NVIC_RESETPRIMASK();
  const int c = USB_LP_CAN1_RX0_IRQn;
  NVIC->ISER[c >> 5] = 1 << (c & 0x1F);
}

static inline void Disable_USB_Interrupts(void)
{
  //NVIC_SETPRIMASK();
  const int c = USB_LP_CAN1_RX0_IRQn;
  NVIC->ICER[c >> 5] = 1 << (c & 0x1F);
}

// Transmit

void EP1_IN_Callback(void)                  // on transmit
{
  tx_in_progress = FALSE;
}

u32 Virtual_Com_Port_Transmit_Available()
{
  return tx_in_progress ? 0 : VIRTUAL_COM_PORT_DATA_SIZE;
}

u32 Virtual_Com_Port_Transmit_Non_Blocking(u8* data, u32 length)
{
  u32 l;
  if (!length || tx_in_progress) return 0;
  tx_in_progress = TRUE;
  l = length <= VIRTUAL_COM_PORT_DATA_SIZE ? length : VIRTUAL_COM_PORT_DATA_SIZE;
  UserToPMABufferCopy(data, ENDP1_TXADDR, l);
  Disable_USB_Interrupts();
  SetEPTxCount(ENDP1, l);
  SetEPTxValid(ENDP1);
  Enable_USB_Interrupts();
  return l;
}

void Virtual_Com_Port_Transmit(u8* data, u32 length)
{
  while (length)
  {
    u32 l;
    l = Virtual_Com_Port_Transmit_Non_Blocking(data, length);
    data += l;
    length -= l;
  }
}

// Receive

void EP3_OUT_Callback(void)                 // on receive
{
  u16 c;
  rx_pma_count = c = GetEPRxCount(ENDP3);
  if (!c) SetEPRxValid(ENDP3);
}

u32 Virtual_Com_Port_Receive_Available()
{
  return rx_buffer_count > rx_buffer_index ? rx_buffer_count - rx_buffer_index : rx_pma_count;
}

u32 Virtual_Com_Port_Receive_Non_Blocking(u8* data, u32 length)
{
  u32 l;
  if (!(rx_buffer_count > rx_buffer_index))
  {
    u16 c;
    c = rx_pma_count;
    if (!c) return 0;
    PMAToUserBufferCopy(rx_buffer, ENDP3_RXADDR, c);
    rx_buffer_count = c;
    rx_buffer_index = 0;
    rx_pma_count = 0;
    Disable_USB_Interrupts();
    SetEPRxValid(ENDP3);
    Enable_USB_Interrupts();
  }
  l = rx_buffer_count - rx_buffer_index;
  if (length > l) length = l;
  l = length;
  while (length--) *data++ = rx_buffer[rx_buffer_index++];
  return l;
}

u32 Virtual_Com_Port_Receive(u8* data, u32 length)
{
  u32 l;
  do l = Virtual_Com_Port_Receive_Non_Blocking(data, length);
  while (!l && length);
  return l;
}


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

