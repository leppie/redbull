

#ifndef __ONEWIRE_H
#define __ONEWIRE_H

#ifdef __cplusplus
 extern "C" {
#endif

#if 0
void FindDevices(void);
#endif

float Read_Temperature(void);

unsigned char ow_read_bit(void);
void ow_write_bit(unsigned char bitval);
unsigned char ow_read_byte(void);
void ow_write_byte(unsigned char val);
unsigned char ow_reset(void);
void ow_pin_out(void);
void ow_pin_in(void);
void ow_get_rom(unsigned char *romid);


#ifdef __cplusplus
}
#endif

#endif
