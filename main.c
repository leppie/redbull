/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/main.c
 * @author  MCD Application Team
 * @version V3.3.0
 * @date    04/16/2010
 * @brief   Main program body
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32_eval.h"
#include "stm32_eval_sdio_sd.h"
#include "stm3210e_eval_fsmc_nand.h"
#include "stm3210e_eval_fsmc_nor.h"
#include "stm3210e_eval_fsmc_sram.h"
#include "delay.h"
#include "rtc.h"
#include "stm3210e_eval_lcd.h"
#include "libdcc/dcc_stdio.h"
#include "usb_vcom.h"
#include "ff.h"
#include "numbers.h"
#include "one-wire.h"

void USART_STDIO_Init();

#define Bank1_SRAM3_ADDR    ((__IO uint32_t)0x68000000)

#define LED_PORT GPIOF

#define LED(x) (GPIO_Pin_6 << ((x) - 1))
#define LED_ALL (LED(1) | LED(2) | LED(3) | LED(4) | LED(5))


void LED_On(uint16_t leds)
{
	LED_PORT->ODR = ~leds;
}

void LED_Off(uint16_t leds)
{
	LED_PORT->ODR = leds;
}

void LED_Init(void)
{
	GPIO_InitTypeDef leds;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	leds.GPIO_Pin = LED_ALL;
	leds.GPIO_Mode = GPIO_Mode_Out_PP;
	leds.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(LED_PORT, &leds);

	LED_Off(LED_ALL);
}

void OneWireInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	ow_reset();

	ow_write_byte(0xcc);
	ow_write_byte(0x4e);
	ow_write_byte(0);
	ow_write_byte(0);
	ow_write_byte(0);
	ow_reset();
}

void assert_failed(uint8_t* file, uint32_t line)
{
	while (1);
}

#define FASTFLOOR(x) ( ((x)>0) ? ((int)x) : (((int)x)-1) )

static float MIN(float x, float y)
{
	return x > y ? y : x;
}

static float MAX(float x, float y)
{
	return x < y ? y : x;
}

const uint8_t perm[] =
{ 151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140,
		36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120,
		234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177,
		33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165,
		71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60,
		211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65,
		25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200,
		196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64,
		52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85,
		212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170,
		213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43,
		172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185,
		112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191,
		179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31,
		181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150,
		254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195,
		78, 66, 215, 61, 156, 180, 151, 160, 137, 91, 90, 15, 131, 13, 201, 95,
		96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21,
		10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219,
		203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125,
		136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146,
		158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55,
		46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209,
		76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86,
		164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5,
		202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58,
		17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154,
		163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98,
		108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251,
		34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235,
		249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204,
		176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114,
		67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180 };

typedef struct
{
	float R, G, B;
} ColorF;

static float lerp(float edge0, float edge1, float f)
{
	return edge0 + (edge1 - edge0) * f;
}

static void collerp(ColorF* rgb, float f, ColorF* col1, ColorF* col2)
{
	if (f <= 0.0f)
	{
		*rgb = *col1;
		return;
	}

	if (f >= 1.0f)
	{
		*rgb = *col2;
		return;
	}

	rgb->R = lerp(col1->R, col2->R, f);
	rgb->G = lerp(col1->G, col2->G, f);
	rgb->B = lerp(col1->B, col2->B, f);
}

static float saturate(float x)
{
	return MAX(0, MIN(x, 1));
}

static float smoothstep(float edge0, float edge1, float x)
{
	// Scale, bias and saturate x to 0..1 range
	x = saturate((x - edge0) / (edge1 - edge0));
	// Evaluate polynomial
	return x * x * (3 - 2 * x);
}

static float grad2(int hash, float x, float y)
{
	int h = hash & 7; /* Convert low 3 bits of hash code */
	float u = h < 4 ? x : y; /* into 8 simple gradient directions, */
	float v = h < 4 ? y : x; /* and compute the dot product with (x,y). */
	return (((h & 1) != 0) ? -u : u) + (((h & 2) != 0) ? -2.0f * v : 2.0f * v);
}

static float noise2f(float x, float y, float xscale, float yscale)
{
	const float F2 = 0.366025403f; /* F2 = 0.5*(sqrt(3.0)-1.0) */
	const float G2 = 0.211324865f; /* G2 = (3.0-Math.sqrt(3.0))/6.0 */

	float n0, n1, n2; /* Noise contributions from the three corners */

	/* Skew the input space to determine which simplex cell we're in */
	float s = (x + y) * F2; /* Hairy factor for 2D */
	float xs = x + s;
	float ys = y + s;
	int i = FASTFLOOR(xs);
	int j = FASTFLOOR(ys);

	float t = (float) (i + j) * G2;
	float X0 = i - t; /* Unskew the cell origin back to (x,y) space */
	float Y0 = j - t;
	float x0 = x - X0; /* The x,y distances from the cell origin */
	float y0 = y - Y0;

	float x1, y1, x2, y2;
	int ii, jj;
	float t0, t1, t2;

	/* For the 2D case, the simplex shape is an equilateral triangle. */
	/* Determine which simplex we are in. */
	int i1, j1; /* Offsets for second (middle) corner of simplex in (i,j) coords */
	if (x0 > y0)
	{
		i1 = 1;
		j1 = 0;
	} /* lower triangle, XY order: (0,0)->(1,0)->(1,1) */
	else
	{
		i1 = 0;
		j1 = 1;
	} /* upper triangle, YX order: (0,0)->(0,1)->(1,1) */

	/* A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and */
	/* a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where */
	/* c = (3-sqrt(3))/6 */

	x1 = x0 - i1 + G2; /* Offsets for middle corner in (x,y) unskewed coords */
	y1 = y0 - j1 + G2;
	x2 = x0 - 1.0f + 2.0f * G2; /* Offsets for last corner in (x,y) unskewed coords */
	y2 = y0 - 1.0f + 2.0f * G2;

	/* Wrap the integer indices at 256, to avoid indexing perm[] out of bounds */
	ii = i & 255;
	jj = j & 255;

	/* Calculate the contribution from the three corners */
	t0 = 0.5f - x0 * x0 - y0 * y0;
	if (t0 < 0.0f)
		n0 = 0.0f;
	else
	{
		t0 *= t0;
		n0 = t0 * t0 * grad2(perm[ii + perm[jj]], x0, y0);
	}

	t1 = 0.5f - x1 * x1 - y1 * y1;
	if (t1 < 0.0f)
		n1 = 0.0f;
	else
	{
		t1 *= t1;
		n1 = t1 * t1 * grad2(perm[ii + i1 + perm[jj + j1]], x1, y1);
	}

	t2 = 0.5f - x2 * x2 - y2 * y2;
	if (t2 < 0.0f)
		n2 = 0.0f;
	else
	{
		t2 *= t2;
		n2 = t2 * t2 * grad2(perm[ii + 1 + perm[jj + 1]], x2, y2);
	}

	/* Add contributions from each corner to get the final noise value. */
	/* The result is scaled to return values in the interval [-1,1]. */
	return ((40 * xscale * (n0 + n1 + n2)) + 1) / (2 * yscale); /* TODO: The scale factor is preliminary! */
}

static void colsca(ColorF* rgb, float z)
{
	rgb->R *= z;
	rgb->G *= z;
	rgb->B *= z;
}

#if 0
static void colfinc(ColorF* carne, float p)
{
	carne->R += p;
	carne->G += p;
	carne->B += p;
}
#endif

static void skin(ColorF* rgb, float x, float y)
{
#if 0
	float rx = 2.0f * (x - 0.5f) * 1.33f;
	float ry = 2.0f * (y - 0.5f);

	ColorF carne =
	{	.R = 0.75f, .G = 0.69f, .B = 0.6f};

	float cel = 0.95f + 0.05f * noise2f(64.0f * x, 64.0f * y, 256, 256);
	colsca(&carne, cel);
	carne.R += 0.03f * rx;
	carne.G += 0.03f * ry;
	colsca(&carne, y * 0.1f + 0.9f);

	float bri = noise2f(128.0f * x, 128.0f * y, 256, 256);
	bri = 0.2f + 0.8f * smoothstep(0.0f, 0.3f, bri);
	colfinc(&carne, bri * 0.08f * y);

	float san = 0.50f * noise2f(16.0f * x, 16.0f * y, 256, 256);
	san += 0.25f * noise2f(32.0f * x, 32.0f * y, 256, 256);
	carne.G *= 1.0f - 0.1f * san;

	float osc = 0.500f * noise2f(12.0f * x, 12.0f * y, 256, 256);
	osc += 0.250f * noise2f(24.0f * x, 24.0f * y, 256, 256);
	osc += 0.125f * noise2f(48.0f * x, 48.0f * y, 256, 256);
	colsca(&carne, 0.9f + 0.1f * osc);

	carne.R += 0.08f * x;
	carne.G += 0.01f;

	float pecas = noise2f(32.0f * x, 32.0f * y, 256, 256);
	pecas = smoothstep(0.48f, 0.6f, pecas);

	carne.R *= 1.0f - 0.15f * pecas;
	carne.G *= 1.0f - 0.17f * pecas;
	carne.B *= 1.0f - 0.17f * pecas;

	colfinc(&carne, -0.14f);

	rgb->R = carne.R * 1.25f + 0.3f;
	rgb->G = carne.G * 1.22f + 0.3f;
	rgb->B = carne.B * 1.20f + 0.3f;
#endif
}
#define PI 3.14159265358979323846f
//-----------------------------------------------
// Fast arctan2
float arctan2(float y, float x)
{
	float angle;
	float coeff_1 = PI / 4;
	float coeff_2 = 3 * coeff_1;
	float abs_y = fabs(y) + 1e-10; // kludge to prevent 0/0 condition
	if (x >= 0)
	{
		float r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	}
	else
	{
		float r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	if (y < 0)
		return (-angle); // negate if in quad III or IV
	else
		return (angle);
}

static void eye(ColorF* rgb, float x, float y, float b)
{
	y += 0.05f;
	x -= 0.10f;

	float rx = 2.0f * (x - 0.5f) * 4.0f / 3.0f;
	float ry = 2.0f * (y - 0.5f);
	float a = arctan2(ry, rx);
	float e = rx * rx + ry * ry;
	float r = sqrtf(e);

	ColorF fue =
	{ .R = 1.0f, .G = 1.0f, .B = 1.0f };

	// blood
	float ven = noise2f(24.0f * x, 24.0f * y, 256, 256);
	ven = smoothstep(-.2f, .0f, ven) - smoothstep(.0f, .2f, ven);
	ven += x + x * x * x * x * x * x * 7.0f;
	fue.R += 0.04f - 0.00f * ven;
	fue.G += 0.04f - 0.05f * ven;
	fue.B += 0.04f - 0.05f * ven;

	ColorF den =
	{ .R = 0.35f, .G = 0.75f, .B = 0.45f + e };

	float no = 0.8f + 0.2f * noise2f(4.f * r, 32.f * a / PI, 32, 32);
	colsca(&den, no);

	float f2 = smoothstep(0.025f, 0.035f, e);
	colsca(&den, f2);
	// blend in/out
	collerp(rgb, smoothstep(0.35f, 0.36f, e), &den, &fue);

	// ring
	float ri = smoothstep(.31f, .35f, e) - smoothstep(.35f, .39f, e);
	ri = 1.0f - 0.35f * ri;
	colsca(rgb, ri);

	// reflecion
	float r1 = sqrtf(r * r * r);
	float re = noise2f(2.f + 4.f * r1 * cosf(a), 4.f * r1 * sinf(a), 256, 256);
	re = 0.8f * smoothstep(0.1f, 0.5f, re);

	rgb->R += re * (1.f - rgb->R);
	rgb->G += re * (1.f - rgb->G);
	rgb->B += re * (1.f - rgb->B);

	//shadow
	colsca(rgb, 0.95f + 0.05f * smoothstep(0.0f, 0.2f, -b));
}

void draw(ColorF* c, float x, float y)
{
	//c.R = (x + y) * 0.7071f;
	//c.G = (x + y) * 0.7071f;
	//c.B = (x + y) * 0.7071f;

	//float rx = 2.f * (x - 0.5f) * 1.33f;
	float ry = 2.f * (y - 0.5f);

	float h = 3 * sqrtf(x * x * x) * (1.f - x);

	float e = fabsf(ry) - h;

	float f = smoothstep(0.f, 0.01f, e);

	ColorF cOjo =
	{ 0, 0, 0 };
	ColorF cPiel =
	{ 0, 0, 0 };

	eye(&cOjo, x, y, e);
	skin(&cPiel, x, y);

	collerp(c, f, &cOjo, &cPiel);
}

#define TOBYTE(f) ((uint8_t) (saturate(f) * 255))

uint16_t ColorFToRGB(ColorF* c)
{
	return ASSEMBLE_RGB(TOBYTE(c->R), TOBYTE(c->G), TOBYTE(c->B));
}

#define HEIGHT 240
#define WIDTH 320

#define IMG_HEIGHT 120
#define IMG_WIDTH 160

#define BUFFER Bank1_SRAM3_ADDR

void InitImageBuffer()
{
	uint16_t* buffer = (uint16_t*) BUFFER;
	for (int y = 0; y < IMG_HEIGHT; y++)
	{
		LCD_SetCursor(60 + y, 240);
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

		for (int x = 0; x < IMG_WIDTH; x++)
		{
			ColorF c =
			{ 0, 0, 0 };
			draw(&c, (float) x / (IMG_WIDTH - 1), (float) y / (IMG_HEIGHT - 1));
			uint16_t t = ColorFToRGB(&c);
			*buffer++ = t;
			LCD_WriteRAM(t);
		}
	}
}

typedef struct
{
	__IO uint16_t LCD_REG;
	__IO uint16_t LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0C000000))
#define LCD                ((LCD_TypeDef *) LCD_BASE)

void DrawBuffer(void)
{
#if 0
	uint16_t* buffer = (uint16_t*) BUFFER;

	for (int y = 0; y < IMG_HEIGHT; y++)
	{
		for (int x = 0; x < IMG_WIDTH; x++)
		{
			LCD_WriteRAM(*buffer++);
		}
	}
#else
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA2_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = BUFFER;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &LCD->LCD_RAM; // FSMC
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = IMG_HEIGHT * IMG_WIDTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;

	DMA_Init(DMA2_Channel5, &DMA_InitStructure);
	DMA_Cmd(DMA2_Channel5, ENABLE);

	while (!DMA_GetFlagStatus(DMA2_FLAG_TC5))
		;

	DMA_ClearFlag(DMA2_FLAG_TC5);
#endif
}

static int count = 0;

FRESULT scan_files(char* path)
{
	FRESULT res;
	FILINFO fno;
	DIR dir;
	int i;

	char *fn;
#if _USE_LFN
	static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
	fno.lfname = lfn;
	fno.lfsize = sizeof(lfn);
#endif

	res = f_opendir(&dir, path);
	if (res == FR_OK)
	{
		i = strlen(path);
		for (;;)
		{
			res = f_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0)
				break;
			if (fno.fname[0] == '.')
				continue;
#if _USE_LFN
			fn = *fno.lfname ? fno.lfname : fno.fname;
#else
			fn = fno.fname;
#endif
			if (fno.fattrib & AM_DIR)
			{
				sprintf(&path[i], "/%s", fn);
				printf("%s/\n", path);
				res = scan_files(path);
				if (res != FR_OK)
					break;
				path[i] = 0;
			}
			else
			{
				count++;
				printf("%s/%s\n", path, fn);
			}
		}
	}

	return res;
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;

void DMARead(void)
{
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) Bank1_SRAM3_ADDR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 0xffff;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 Channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void Analog_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable DMA1 channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Analog_RCC_Config(void)
{
	/* ADCCLK = PCLK2/4 overclock ? FTW ! :o) */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	/* Enable peripheral clocks ------------------------------------------------*/
	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1, ADC2 and GPIOC clock */
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_GPIOC,
			ENABLE);
}

void Analog_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC.01, PC.02 and PC.04 (ADC Channel11, Channel12 and Channel14)
	 as analog input ----------------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Analog_Config(void)
{
	Analog_RCC_Config();
	Analog_GPIO_Config();
	Analog_NVIC_Config();

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_SlowInterl;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_13Cycles5);

	/* ADC2 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_SlowInterl;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC2 regular channels configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_13Cycles5);

	/* Enable ADC2 external trigger conversion */
	ADC_ExternalTrigConvCmd(ADC2, ENABLE);

	/* Enable ADC1 DMA: it should be enabled in dual mode even if the DMA is not used */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	Delay(2);

	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(ADC1);

	/* Check the end of ADC1 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC1))
		;

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);

	/* Check the end of ADC1 calibration */
	while (ADC_GetCalibrationStatus(ADC1))
		;

	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);

	Delay(2);

	/* Enable ADC2 reset calibaration register */
	ADC_ResetCalibration(ADC2);

	/* Check the end of ADC2 reset calibration register */
	while (ADC_GetResetCalibrationStatus(ADC2))
		;

	/* Start ADC2 calibaration */
	ADC_StartCalibration(ADC2);

	/* Check the end of ADC2 calibration */
	while (ADC_GetCalibrationStatus(ADC2))
		;

}

/* Private define ------------------------------------------------------------*/
#define DAC_DHR12RD_Address      0x40007420

/* Init Structure definition */
DAC_InitTypeDef DAC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint16_t Sine12bit[32] =
{ 2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056, 3939, 3750, 3495,
		3185, 2831, 2447, 2047, 1647, 1263, 909, 599, 344, 155, 38, 0, 38, 155,
		344, 599, 909, 1263, 1647 };

uint32_t DualSine12bit[32];
uint8_t Idx = 0;

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void DAC_RCC_Configuration(void)
{
	/* DMA2 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	/* TIM2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 * @param  None
 * @retval None
 */
void DAC_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
	 connected to the DAC converter. In order to avoid parasitic consumption,
	 the GPIO pin should be configured in analog */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void DAC_Config(void)
{
	DAC_RCC_Configuration();

	/* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
	 connected to the DAC converter. In order to avoid parasitic consumption,
	 the GPIO pin should be configured in analog */
	DAC_GPIO_Configuration();

	/* TIM2 Configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0x1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM2 TRGO selection */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude =
			DAC_TriangleAmplitude_1023;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* DAC channel2 Configuration */
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);
	/* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_2, ENABLE);

	/* Set DAC dual channel DHR12RD register */
	DAC_SetDualChannelData(DAC_Align_12b_R, 0x00, 0x00);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void toggleLED(int pin, int prev)
{
	if (pin)
	{
	  GPIO_ResetBits(GPIOF, pin);
	  Delay(5);
	}
	if (prev)
	{
	  GPIO_SetBits(GPIOF, prev);
	  Delay(20);
	}
}

void print(int line, int col, char* text)
{
	for (;*text; text++ )
	{
		int c = 319 - (col * LCD_GetFont()->Width);
		if (c > 0)
		{
			LCD_DisplayChar(LINE(line), c, *text);
		}
		else
		{
			break;
		}
	}
}

void printRight(int line, char* text)
{
	int l = strlen(text);
	int col = 40 - l;
	for (;*text; text++, col++ )
	{
		int c = 319 - (col * LCD_GetFont()->Width);
		if (c > 0)
		{
			LCD_DisplayChar(LINE(line), c, *text);
		}
		else
		{
			break;
		}
	}
}

static FATFS fatfs;

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

	char buff[128] = {0};

	Delay_Init();

	USART_STDIO_Init();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

	STM3210E_LCD_Init();

	LCD_SetFont(&Font8x12);
	LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);

	for (int i = 0; i < (320 * 240); i++)
	{
	  LCD_WriteRAM(LCD_COLOR_WHITE);
	}

	for (int i = 0; i < (320 * 240); i++)
	{
	  LCD_WriteRAM(LCD_COLOR_BLACK);
	}

	LCD_DisplayStringLine(LINE(0), (uint8_t*) " initializing REDBULL");

	LCD_DisplayStringLine(LINE(1), (uint8_t*) " CPU ...............................");

	sprintf(buff, "ARM Cortex-M3 @ %dMHz", (int)SystemCoreClock/1000000);

	printRight(1, buff);

	LCD_DisplayStringLine(LINE(2), (uint8_t*) " LCD ............................320x240");

	LCD_DisplayStringLine(LINE(3), (uint8_t*) " LED ..................................");
	LED_Init();

	toggleLED(LED1_PIN, 0);
	toggleLED(LED2_PIN, LED1_PIN);
	toggleLED(LED3_PIN, LED2_PIN);
	toggleLED(LED4_PIN, LED3_PIN);
	toggleLED(LED5_PIN, LED4_PIN);
	toggleLED(LED4_PIN, LED5_PIN);
	toggleLED(LED3_PIN, LED4_PIN);
	toggleLED(LED2_PIN, LED3_PIN);
	toggleLED(LED1_PIN, LED2_PIN);
	toggleLED(0		  , LED1_PIN);

	printRight(3, "5");

	LCD_DisplayStringLine(LINE(4), (uint8_t*) " RTC ................");
	RTC_Init();


	RTC_t rtc =
	{
	  .year = 2011,
	  .month = 12,
	  .mday = 19,
	  .hour = 21,
	  .min = 00
	};

	//RTC_SetTime(&rtc);

	RTC_GetTime(&rtc);
	sprintf(buff, "%04d/%02d/%02d %02d:%02d:%02d", rtc.year, rtc.month, rtc.mday,  rtc.hour, rtc.min, rtc.sec);

	printRight(4, buff);

	LCD_DisplayStringLine(LINE(5), (uint8_t*) " USB .................................");
	Set_USBClock();
	Set_System();
	USB_Interrupts_Config();
	USB_Init();

	printRight(5, "ok");

//IS61LV25616 (512KB)
	LCD_DisplayStringLine(LINE(6), (uint8_t*) " SRAM ................................");
	SRAM_Init();

	uint32_t* RAM = (uint32_t*) Bank1_SRAM3_ADDR;
	uint8_t TESTOK = 1;

	for (uint32_t i = 0; i < (512 * 1024)/4; i++)
	{
		RAM[i] = i;
	}

	for (uint32_t i = 0; i < (512 * 1024)/4; i++)
	{
	   if (RAM[i] != i)
	   {
		  TESTOK = 0;
	   }
	   RAM[i] = 0;
	}

	if (TESTOK)
	{
		printRight(6,"IS61LV25616 512KB");
	}
	else
	{
		printRight(6,"fail");
	}

	//M29W128F (2MB)
	LCD_DisplayStringLine(LINE(7), (uint8_t*) " NOR .................................");
	NOR_Init();
	NOR_IDTypeDef norid;
	NOR_ReadID(&norid);

	printRight(7, "M29W128F 2MB");

	//HY27UF081G2A (128MB)
	LCD_DisplayStringLine(LINE(8), (uint8_t*) " NAND ................................");
	NAND_Init();
	NAND_IDTypeDef nandid;
	NAND_ReadID(&nandid);
	printRight(8, "HY27UF081G2A 128MB");

	LCD_DisplayStringLine(LINE(9), (uint8_t*) " SDIO ................................");
	SD_Init();
	SD_CardInfo cardinfo;
	SD_GetCardInfo(&cardinfo);
	printRight(9, "ok");

	LCD_DisplayStringLine(LINE(10), (uint8_t*) " Analog ..............................");
	Analog_Config();
	printRight(10, "ok");

	LCD_DisplayStringLine(LINE(11), (uint8_t*) " DAC .................................");
	DAC_Config();
	printRight(11, "ok");

	f_mount(0, &fatfs);

	LCD_DisplayStringLine(LINE(12), (uint8_t*) " TEMP ................................");
	OneWireInit();

	while (1)
	{
		RTC_GetTime(&rtc);
		sprintf(buff, "%04d/%02d/%02d %02d:%02d:%02d", rtc.year, rtc.month, rtc.mday,  rtc.hour, rtc.min, rtc.sec);

		printRight(4, buff);

		float c = Read_Temperature();
		sprintf(buff, "%.1f", c);
		printRight(12, buff);

		Delay(900);
	}


	//static char path[1024] = "0:";

	//LCD_Clear(LCD_COLOR_BLUE);


	//LCD_DisplayStringLine(0, (uint8_t*) "Loading...");

	Delay(5000);

	//int i = 10;

	LCD_Clear(LCD_COLOR_BLACK);

	LCD_SetDisplayWindow(0, 0, 239, 319);
	LCD_WriteReg(3, 0x1000);
	LCD_WriteRAM_Prepare();

	DMARead();

	while (1)
	{
		while (DMA_GetFlagStatus(DMA1_FLAG_TE1) == SET)
		{
			printf("DMA error\n");
			Delay(100);
		}

		//uint32_t ms = millis();

		uint16_t rem = DMA_GetCurrDataCounter(DMA1_Channel1);
		uint16_t cnt = 0xffff - rem;

		if (cnt >= 80 && cnt < 160)
		{
			continue;
		}

		uint32_t buff[160] =
		{ 0 };

		uint16_t tillend = cnt > 160 ? 160 : cnt;
		uint16_t fromstart = 160 - tillend;

		uint16_t bcnt = cnt - 160;

		// before
		SRAM_ReadBuffer((uint16_t*) &buff[0], bcnt * 4, tillend * 2);
		// after
		if (fromstart)
		{
			SRAM_ReadBuffer((uint16_t*) &buff[tillend], 0, fromstart * 2);
		}

		int halt = 0;
		uint8_t prevsv = 0;

		for (uint16_t i = 0; i < 320; i++)
		{
			uint16_t val = 0;
			if (i & 1)
			{
				val = buff[i >> 1] & 0x0fff;
			}
			else
			{
				val = (buff[i >> 1] >> 16) & 0x0fff;
			}

#define MAXV (17 * 238)

			val = val > MAXV ? MAXV : val;

			uint8_t sv = val / 17;

			sv = sv > 238 ? 238 : sv;

			sv++;

			if (!prevsv)
			{
				prevsv = sv;
			}

			//halt |= (val > 1126);

			uint16_t pbuf[240];

			uint8_t minv = sv > prevsv ? prevsv : sv;
			uint8_t maxv = sv < prevsv ? prevsv : sv;

			for (uint8_t y = 0; y < 240; y++)
			{
				if (y >= minv && y <= maxv)
				{
					pbuf[y] = LCD_COLOR_YELLOW;
				}
				else
				{
					pbuf[y] = LCD_COLOR_BLACK;
				}
			}

			prevsv = sv;

			DMA_InitTypeDef DMA_InitStructure;

			DMA_DeInit(DMA2_Channel5);
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) pbuf;
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &LCD->LCD_RAM; // FSMC
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
			DMA_InitStructure.DMA_BufferSize = 240;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
			DMA_InitStructure.DMA_PeripheralDataSize =
					DMA_PeripheralDataSize_HalfWord;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
			DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;

			DMA_Init(DMA2_Channel5, &DMA_InitStructure);

			DMA_Cmd(DMA2_Channel5, ENABLE);

			while (!DMA_GetFlagStatus(DMA2_FLAG_TC5));

			DMA_ClearFlag(DMA2_FLAG_TC5);

			//Delay(100);
		}

		if (halt)
		{
			printf(
					"rem: %d cnt: %d tillend: %d fromstart: %d bcnt: %d halt: %d\n",
					rem, cnt, tillend, fromstart, bcnt, halt);
			//Delay(2000);
		}

		//Delay(100 - (millis() - ms));

		/*
		 for (int i = 0; i < 0xffff; i++)
		 {

		 printf("%4f\n", (buf[i] & 0xffff) * 3.3/4095);
		 printf("%4f\n", (buf[i] >> 16) * 3.3/4095);
		 }
		 */
	}

#if 0
	while (0)
	{
		/*
		 static int ystart = 0;

		 uint32_t ms = millis();

		 LED_On(LED1);

		 printf("SD_Detect: %x\n", SD_Detect());
		 printf("SD_GetState: %x\n", SD_GetState() );
		 printf("SD_GetStatus: %x\n", SD_GetStatus() );
		 printf("SD_GetTransferState: %x\n", SD_GetTransferState() );

		 scan_files(path);
		 */
		/*
		 FIL fil;
		 FRESULT r = f_open(&fil, "0:/ff.c", FA_READ);

		 while (1)
		 {
		 uint br;
		 char buffer[1024] = {0};
		 f_read(&fil, buffer, 1024, &br);

		 if (!br)
		 {
		 break;
		 }

		 printf("%s", buffer);
		 }

		 r = f_close(&fil);
		 */

		char buff[128] =
		{	0};
		RTC_t rtc;
		RTC_GetTime(&rtc);
		sprintf(buff, "%02d:%02d:%02d", rtc.hour, rtc.min, rtc.sec);

		if (rtc.sec == 0)
		{
			LCD_Clear(LCD_COLOR_BLACK);
		}

		ystart = rtc.sec * 2;

		int z = 320 - 16;

		for (int k = 0; k < 5; k++)
		{
			char c = buff[k];

			const uint8_t* buffer = NULL;

			switch (c)
			{
				case '0' ... '9':
				buffer = IMAGE[c - '0'];
				z -= 64;
				LCD_SetDisplayWindow(ystart, z, 95, 63);
				LCD_WriteRAM_Prepare();
				for (int y = 0; y < 96; y++)
				{
					for (int x = 0; x < 64; x++)
					{
						uint8_t gc = *buffer++;
						uint8_t gc_2 = gc * 7/8;
						LCD_WriteRAM(ASSEMBLE_RGB(gc_2, gc_2, gc));
					}
				}

				break;
				case ':':
				buffer = IMAGE[10];
				z -= 32;
				LCD_SetDisplayWindow(ystart, z, 95, 31);
				LCD_WriteRAM_Prepare();
				for (int y = 0; y < 96; y++)
				{
					for (int x = 0; x < 32; x++)
					{
						uint8_t gc = *buffer++;
						LCD_WriteRAM(ASSEMBLE_RGB(0, 0, gc));
					}
				}

				break;

			}

		}

		LCD_SetDisplayWindow(0, 0, HEIGHT - 1, WIDTH - 1);
		LCD_DisplayStringLine(LINE(9), (uint8_t*) buff);

		uint32_t diff = millis() - ms;
		printf("Time: %u\n", diff);
		LED_Off(LED_ALL);
		Delay(1000 - diff);

	}

#endif

	LCD_DisplayStringLine(0, (uint8_t*) "Loading eye");
	InitImageBuffer();

	/* Infinite loop */
	while (1)
	{

		//uint16_t r = LCD_ReadReg(3);
		//printf("am: %x i/d: %x \n", (r >> 3) & 1, (r >> 4) & 3);
		//printf("xs: %d xe: %d ys: %d ye: %d \n", LCD_ReadReg(0x50), LCD_ReadReg(0x51), LCD_ReadReg(0x52), LCD_ReadReg(0x53) );

		//uint32_t ms0 = millis();

		LCD_SetDisplayWindow(0, 0, IMG_HEIGHT - 1, IMG_WIDTH - 1);
		LCD_WriteReg(3, 0x1038);
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

		DrawBuffer();

		//uint32_t ms1 = millis();

		LCD_SetDisplayWindow(0, IMG_WIDTH, IMG_HEIGHT - 1, IMG_WIDTH - 1);
		LCD_WriteReg(3, 0x1018);
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

		DrawBuffer();

		//uint32_t ms2 = millis();

		LCD_SetDisplayWindow(IMG_HEIGHT, 0, IMG_HEIGHT - 1, IMG_WIDTH - 1);
		LCD_WriteReg(3, 0x1038);
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

		DrawBuffer();

		//uint32_t ms3 = millis();

		LCD_SetDisplayWindow(IMG_HEIGHT, IMG_WIDTH, IMG_HEIGHT - 1,
				IMG_WIDTH - 1);
		LCD_WriteReg(3, 0x1018);
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */

		DrawBuffer();

		//uint32_t ms4 = millis();

		//uint32_t t0 = ms1 - ms0;
		//uint32_t t1 = ms2 - ms1;
		//uint32_t t2 = ms3 - ms2;
		//uint32_t t3 = ms4 - ms3;

		//printf("t0: %d t1: %d t2: %d t3: %d\n", t0, t1, t2, t3);

		LCD_WriteReg(3, 0x1018);

		Delay(500);
		LCD_SetDisplayWindow(0, 0, HEIGHT - 1, WIDTH - 1);
		LCD_Clear(LCD_COLOR_BLACK);
		Delay(50);

		printf("SD_Detect: %x\n", SD_Detect());
		printf("SD_GetState: %x\n", SD_GetState());

		scan_files("0:");
	}
}

