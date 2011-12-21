#include "one-wire.h"
#include "stm32f10x.h"
#include "delay.h"
#include "stdio.h"

#define FALSE 0
#define TRUE 1
////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
//
unsigned char ROM[8]; // ROM Bit
unsigned char lastDiscrep = 0; // last discrepancy
unsigned char doneFlag = 0; // Done flag
unsigned char FoundROM[5][8]; // table of found ROM codes
unsigned char numROMs;
unsigned char dowcrc;
unsigned char dscrc_table[] =
{ 0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 157,
		195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35,
		125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 190,
		224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70,
		24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
		219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
		101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
		248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
		140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147,
		205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236,
		14, 80, 175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82,
		176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145,
		207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105,
		55, 213, 139, 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119,
		244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151,
		201, 74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232,
		10, 84, 215, 137, 107, 53 };

#define PORT	GPIOB
#define ADDRESS GPIO_Pin_13
#define PORTADDRESS ADDRESS


GPIO_InitTypeDef GPIO_InitStructure;

unsigned char ow_reset(void)
{
	u8 presence;
	ow_pin_out();
	GPIO_ResetBits(PORT, ADDRESS);
	MicroDelay(500); // 480us min
	ow_pin_in();
	MicroDelay(60);
	presence = GPIO_ReadInputDataBit(PORT, ADDRESS);
	presence &= 0x01;
	MicroDelay(420);
	return (!presence);
}

unsigned char ow_read_bit(void)
{
	unsigned char val;
	ow_pin_out();
	GPIO_ResetBits(PORT, ADDRESS);
	MicroDelay(3);
	ow_pin_in();
	MicroDelay(10);
	val = GPIO_ReadInputDataBit(PORT, ADDRESS);
	val &= 0x01;
	MicroDelay(57);
	return (val);
}

void ow_write_bit(unsigned char bitval)
{
	if (bitval)
	{
		ow_pin_out();
		GPIO_ResetBits(PORT, ADDRESS);
		MicroDelay(10);
		GPIO_SetBits(PORT, ADDRESS);
		MicroDelay(55);
	}
	else
	{
		ow_pin_out();
		GPIO_ResetBits(PORT, ADDRESS);
		MicroDelay(65);
		GPIO_SetBits(PORT, ADDRESS);
		MicroDelay(5);
	}
}

unsigned char ow_read_byte(void)
{
	unsigned char i;
	unsigned char value = 0;
	for (i = 0; i < 8; i++)
	{
		if (ow_read_bit())
		{
			value |= (0x01 << i);
		}
	}
	return (value);
}

void ow_write_byte(unsigned char val)
{
	int loop;
	// Loop to write each bit in the byte, LS-bit first
	for (loop = 0; loop < 8; loop++)
	{
		ow_write_bit(val & 0x01);
		// shift the data byte for the next bit
		val >>= 1;
	}
}

void ow_pin_out(void)
{

	/* Configure CX_ADC2 as 1-wire port */
	GPIO_InitStructure.GPIO_Pin = ADDRESS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PORT, &GPIO_InitStructure);
	GPIO_SetBits(PORT, ADDRESS);
	//MicroDelay(1);
}

void ow_pin_in(void)
{

	/* Configure CX_ADC2 as 1-wire port */
	GPIO_InitStructure.GPIO_Pin = ADDRESS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(PORT, &GPIO_InitStructure);
	//MicroDelay(1);
}

void ow_get_rom(unsigned char *romid)
{
	ow_write_byte(0x33);
	for (int i = 0; i < 8; i++)
	{
		romid[i] = ow_read_byte();
	}
}

//////////////////////////////////////////////////////////////////////////////
// ONE WIRE CRC
//
unsigned char ow_crc(unsigned char x)
{
	dowcrc = dscrc_table[dowcrc ^ x];
	return dowcrc;
}

#if 0

// NEXT
// The Next function searches for the next device on the 1-Wire bus. If
// there are no more devices on the 1-Wire then false is returned.
//
unsigned char Next(void)
{
	unsigned char m = 1; // ROM Bit index
	unsigned char n = 0; // ROM Byte index
	unsigned char k = 1; // bit mask
	unsigned char x = 0;
	unsigned char discrepMarker = 0; // discrepancy marker
	unsigned char g; // Output bit
	unsigned char nxt; // return value
	int flag;
	nxt = FALSE; // set the next flag to false
	dowcrc = 0; // reset the dowcrc
	flag = ow_reset(); // reset the 1-Wire
	if (!flag || doneFlag) // no parts -> return false
	{
		lastDiscrep = 0; // reset the search
		return FALSE;
	}
	ow_write_byte(0xF0); // send SearchROM command
	do
// for all eight bytes
	{
		x = 0;
		if (ow_read_bit() == 1)
			x = 2;
		delay(6);
		if (ow_read_bit() == 1)
			x |= 1; // and its complement
		if (x == 3) // there are no devices on the 1-Wire
			break;

		else
		{
			if (x > 0) // all devices coupled have 0 or 1
				g = x >> 1; // bit write value for search
			else
			{
// if this discrepancy is before the last
// discrepancy on a previous Next then pick
// the same as last time
				if (m < lastDiscrep)
					g = ((ROM[n] & k) > 0);
				else
					// if equal to last pick 1
					g = (m == lastDiscrep); // if not then pick 0
// if 0 was picked then record
// position with mask k
				if (g == 0)
					discrepMarker = m;
			}
			if (g == 1) // isolate bit in ROM[n] with mask k
				ROM[n] |= k;
			else
				ROM[n] &= ~k;
			ow_write_bit(g); // ROM search write
			m++; // increment bit counter m
			k = k << 1; // and shift the bit mask k
			if (k == 0) // if the mask is 0 then go to new ROM
			{ // byte n and reset mask
				ow_crc(ROM[n]); // accumulate the CRC
				n++;
				k++;
			}
		}
	} while (n < 8); //loop until through all ROM bytes 0-7
	if (m < 65 || dowcrc) // if search was unsuccessful then
		lastDiscrep = 0; // reset the last discrepancy to 0
	else
	{
// search was successful, so set lastDiscrep,
// lastOne, nxt
		lastDiscrep = discrepMarker;
		doneFlag = (lastDiscrep == 0);
		nxt = TRUE; // indicates search is not complete yet, more
// parts remain
	}
	return nxt;
}

// FIRST
// The First function resets the current state of a ROM search and calls
// Next to find the first device on the 1-Wire bus.
//
unsigned char First(void)
{
	lastDiscrep = 0; // reset the rom search last discrepancy global
	doneFlag = FALSE;
	return Next(); // call Next and return its return value
}

// FIND DEVICES
void FindDevices(void)
{
	unsigned char m;
	if (ow_reset()) //Begins when a presence is detected
	{
		if (First()) //Begins when at least one part is found
		{
			numROMs = 0;
			do
			{
				numROMs++;
				for (m = 0; m < 8; m++)
				{
					FoundROM[numROMs][m] = ROM[m]; //Identifies ROM\\number on found device
				}
//				printf("\nROM CODE =%02X%02X%02X%02X\n", FoundROM[5][7],
//						FoundROM[5][6], FoundROM[5][5], FoundROM[5][4],
//						FoundROM[5][3], FoundROM[5][2], FoundROM[5][1],
//						FoundROM[5][0]);
			} while (Next() && (numROMs < 10)); //Continues until no additional devices are found
		}
	}
}
#endif

float Read_Temperature(void)
{
	char data[9];
	int k;
	ow_reset();
	ow_write_byte(0xCC); //Skip ROM
	ow_write_byte(0x44); // Start Conversion
	Delay(94);
	ow_reset();
	ow_write_byte(0xCC); // Skip ROM
	ow_write_byte(0xBE); // Read Scratch Pad
	for (k = 0; k < 9; k++)
	{
		data[k] = ow_read_byte();
	}
	unsigned int raw = (data[1] << 8) | data[0];
	/*
	u8 cfg = (data[4] & 0x60);
	if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
	else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
	else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
	// default is 12 bit resolution, 750 ms conversion time
	 */
	float celsius = (float)raw / 16.0;
	return celsius;
}
