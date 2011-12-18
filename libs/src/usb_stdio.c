
#ifdef USE_USB_STDIO

#include "usb_vcom.h"
#include "usb_pwr.h"

int _isatty(int file)
{
	return 1;
}

int _read(int file, char* ptr, int len)
{
	if (len < 0) return -1;
	return Virtual_Com_Port_Receive((u8*) ptr, len);
}

int _write(int file, char* ptr, int len)
{
  int counter = 1000;
  //if (bDeviceState != CONFIGURED) return -1;
	if (len < 0) return -1;

	do
	{
	  counter--;
	  if (counter == 0)
	  {
	    return -1;
	  }
	}
	while (!Virtual_Com_Port_Transmit_Available());
	Virtual_Com_Port_Transmit((u8*) ptr, len);
	return len;
}

#endif
