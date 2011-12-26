//=========================================================================================================================================
#include "stm32f10x_conf.h"
#include "ads7843drv.h"
#include "delay.h"
#include "math.h"
//====================================================================================

#define TRUE 1
#define FALSE 0

coordinate ScreenSample[3] =
{
{ 3388, 920 },
{ 895, 914 },
{ 1767, 3115 } };

//LCD上 对应的点 不能在。H赋值
coordinate DisplaySample[3] =
{
{ 45, 45 },
{ 45, 270 },
{ 190, 190 } };

void TP_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOG, ENABLE);
	// IODIR1 = 0x00;
	//  IODIR1 = IODIR1 | MASK_CS | MASK_DCLK | MASK_DIN;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //推挽
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);




}

void Delayus(int k)
{
	int j;
	for (j = k; j > 0; j--)
		asm volatile ("nop");
}
//====================================================================================
void ReBack()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
static void WR_CMD(unsigned char cmd)
{
	unsigned char buf;
	unsigned char i;
	TP_CS(1);
	TP_DIN(0);
	TP_DCLK(0);
	TP_CS(0);
	for (i = 0; i < 8; i++)
	{
		buf = (cmd >> (7 - i)) & 0x1;
		TP_DIN(buf);
		Delayus(5);
		TP_DCLK(1);
		Delayus(5);
		TP_DCLK(0);
	}
}
//====================================================================================
static unsigned short RD_AD(void)
{
	unsigned short buf = 0, temp;
	unsigned char i;
	TP_DIN(0);
	Delayus(5);
	TP_DCLK(1);
	for (i = 0; i < 12; i++)
	{
		Delayus(5);
		TP_DCLK(0);
		Delayus(5);
		temp = (TP_DOUT) ? 1 : 0;
		buf |= (temp << (11 - i));

		//Delayus(5);
		TP_DCLK(1);
	}
	for (; i < 16; i++)
	{
		Delayus(5);
		TP_DCLK(0);
		Delayus(5);
		TP_DCLK(1);
	}
	TP_CS(1);
	buf &= 0x0fff;
	return (buf);
}
//====================================================================================
int Read_X(void)
{
	int i;
	WR_CMD(CHX);
	// while(TP_BUSY);
	Delayus(5);
	i = RD_AD();
	return i;
}
//====================================================================================
int Read_Y(void)
{
	int i;
	WR_CMD(CHY);
	//while(TP_BUSY);
	Delayus(5);
	i = RD_AD();
	return i;
}
//====================================================================================
void TP_GetAdXY(int *x, int *y)
{
	int adx, ady;
	adx = Read_X();
	ady = Read_Y();
	*x = adx;
	*y = ady;
}

static int TP_X[1], TP_Y[1];

#define  VALUE 2 //差值
coordinate *Read_Ads7846(void)
{
	static coordinate Screen;
	int m0, m1, m2;
	u8 count = 0;
	u16 databuffer[2][9] =
	{
	{ 0 },
	{ 0 } }; //数据组
	u16 temp[3];

	TP_Init();

	do //循环读数9次
	{
		TP_GetAdXY(TP_X, TP_Y);
		databuffer[0][count] = TP_X[0]; //X
		databuffer[1][count] = TP_Y[0];
		count++;
	} while (!TP_INT_IN && count < 9); //当为1时根本就不采样  初始自动打点BUG
	ReBack();
	if (count == 9) //一定要读到9次数据,否则丢弃
	{
		temp[0] = (databuffer[0][0] + databuffer[0][1] + databuffer[0][2]) / 3;
		temp[1] = (databuffer[0][3] + databuffer[0][4] + databuffer[0][5]) / 3;
		temp[2] = (databuffer[0][6] + databuffer[0][7] + databuffer[0][8]) / 3;
		m0 = temp[0] - temp[1];
		m1 = temp[1] - temp[2];
		m2 = temp[2] - temp[0];

		m0 = m0 > 0 ? m0 : (-m0);
		m1 = m1 > 0 ? m1 : (-m1);
		m2 = m2 > 0 ? m2 : (-m2);
		if (m0 > VALUE && m1 > VALUE && m2 > VALUE)
			return 0;
		if (m0 < m1)
		{
			if (m2 < m0)
				Screen.x = (temp[0] + temp[2]) / 2;
			else
				Screen.x = (temp[0] + temp[1]) / 2;
		}
		else if (m2 < m1)
			Screen.x = (temp[0] + temp[2]) / 2;
		else
			Screen.x = (temp[1] + temp[2]) / 2;

		temp[0] = (databuffer[1][0] + databuffer[1][1] + databuffer[1][2]) / 3;
		temp[1] = (databuffer[1][3] + databuffer[1][4] + databuffer[1][5]) / 3;
		temp[2] = (databuffer[1][6] + databuffer[1][7] + databuffer[1][8]) / 3;
		m0 = temp[0] - temp[1];
		m1 = temp[1] - temp[2];
		m2 = temp[2] - temp[0];

		m0 = m0 > 0 ? m0 : (-m0);
		m1 = m1 > 0 ? m1 : (-m1);
		m2 = m2 > 0 ? m2 : (-m2);
		if (m0 > VALUE && m1 > VALUE && m2 > VALUE)
			return 0;

		if (m0 < m1)
		{
			if (m2 < m0)
				Screen.y = (temp[0] + temp[2]) / 2;
			else
				Screen.y = (temp[0] + temp[1]) / 2;
		}
		else if (m2 < m1)
			Screen.y = (temp[0] + temp[2]) / 2;
		else
			Screen.y = (temp[1] + temp[2]) / 2;
		return &Screen;
	}
	Screen.x = 0;
	Screen.y = 0;
	return &Screen;
}

//只有在LCD和触摸屏间的误差角度非常小时，才能运用上面公式
//3个采样点必须不太靠近触摸屏边缘，他们的间隔必须足够大->

//液晶屏上的采样点
/*
 定义2个变量
 matrix matrix ;
 coordinate  display ;

 setCalibrationMatrix( &DisplaySample[0],&ScreenSample[0],&matrix ) ;  //送入值得到参数
 getDisplayPoint(&display, Read_Ads7846(), &matrix ) ;
 display.x display.y  就是所的到的数字    */

unsigned char setCalibrationMatrix(coordinate * displayPtr,
		coordinate * screenPtr, matrix * matrixPtr)
{

	unsigned char retValue = 0;

	matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x)
			* (screenPtr[1].y - screenPtr[2].y))
			- ((screenPtr[1].x - screenPtr[2].x)
					* (screenPtr[0].y - screenPtr[2].y));

	if (matrixPtr->Divider == 0)
	{
		retValue = 1;
	}
	else
	{
		matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x)
				* (screenPtr[1].y - screenPtr[2].y))
				- ((displayPtr[1].x - displayPtr[2].x)
						* (screenPtr[0].y - screenPtr[2].y));

		matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x)
				* (displayPtr[1].x - displayPtr[2].x))
				- ((displayPtr[0].x - displayPtr[2].x)
						* (screenPtr[1].x - screenPtr[2].x));

		matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x
				- screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y
				+ (screenPtr[0].x * displayPtr[2].x
						- screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y
				+ (screenPtr[1].x * displayPtr[0].x
						- screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y;

		matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y)
				* (screenPtr[1].y - screenPtr[2].y))
				- ((displayPtr[1].y - displayPtr[2].y)
						* (screenPtr[0].y - screenPtr[2].y));

		matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x)
				* (displayPtr[1].y - displayPtr[2].y))
				- ((displayPtr[0].y - displayPtr[2].y)
						* (screenPtr[1].x - screenPtr[2].x));

		matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y
				- screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y
				+ (screenPtr[0].x * displayPtr[2].y
						- screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y
				+ (screenPtr[1].x * displayPtr[0].y
						- screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y;
	}

	return (retValue);
}
//如果是1 则代表 是不成功的。。。0是成功的
unsigned char getDisplayPoint(coordinate * displayPtr, coordinate * screenPtr,
		matrix * matrixPtr)
{
	unsigned char retValue = 0;

	if (matrixPtr->Divider != 0)
	{
		if (screenPtr->x == 0 && screenPtr->y == 0)
		{
			displayPtr->x = 999;
			displayPtr->y = 999;
			retValue = 1; //没有按键 不成功
		}
		else
		{
			displayPtr->x = ((matrixPtr->An * screenPtr->x)
					+ (matrixPtr->Bn * screenPtr->y) + matrixPtr->Cn)
					/ matrixPtr->Divider;

			displayPtr->y = ((matrixPtr->Dn * screenPtr->x)
					+ (matrixPtr->En * screenPtr->y) + matrixPtr->Fn)
					/ matrixPtr->Divider;
		}
	}
	else
	{
		retValue = 1;
	}

	return (retValue);
}


