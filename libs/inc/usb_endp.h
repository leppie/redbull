#ifndef __USB_ENDP_H
#define __USB_ENDP_H

/* WARNING: There is no transmit buffer. Every transmit invocation will cause a USB transaction.
 * Use Virtual_Com_Port_Transmit_Byte() with caution. Note that newlib does buffer stdio output.
 * 
 * The receive side has a small buffer of VIRTUAL_COM_PORT_DATA_SIZE bytes.
 */

u32 Virtual_Com_Port_Transmit_Available();
u32 Virtual_Com_Port_Transmit_Non_Blocking(const u8* data, u32 length);
void Virtual_Com_Port_Transmit(const u8* data, u32 length);

static inline void Virtual_Com_Port_Transmit_Byte(u8 byte)
{
	Virtual_Com_Port_Transmit(&byte, 1);
}

u32 Virtual_Com_Port_Receive_Available();
u32 Virtual_Com_Port_Receive_Non_Blocking(u8* data, u32 length);
u32 Virtual_Com_Port_Receive(u8* data, u32 length);

static inline u8 Virtual_Com_Port_Receive_Byte()
{
	u8 b;
	Virtual_Com_Port_Receive(&b, 1);
	return b;
}

#endif /* __USB_ENDP_H */
