//=========================================================================================================================================
#include "stm32f10x_conf.h"
#include "ads7843drv.h"
#include "delay.h"
#include "math.h"
//====================================================================================

#define TRUE 1
#define FALSE 0

coordinate ScreenSample[3] =	{
                                            { 3388, 920 },
											{ 895, 914 },
                                            { 1767, 3115 }
                                } ;

	 //LCD上 对应的点 不能在。H赋值
coordinate DisplaySample[3] =   {
                                            { 45, 45 },
											{ 45, 270},
                                            { 190,190}
	                            } ;

 void TP_Init(void)
{
   // IODIR1 = 0x00;
  //  IODIR1 = IODIR1 | MASK_CS | MASK_DCLK | MASK_DIN;
  	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15|GPIO_Pin_13|GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽	
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

}

void Delayus( int k)
{
    int j;
    for(j=k;j > 0;j--);    
}
//====================================================================================
 void ReBack()
{
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
}
static void WR_CMD (unsigned char cmd) 
{
    unsigned char buf;
    unsigned char i;
    TP_CS(1);
    TP_DIN(0);
    TP_DCLK(0);
    TP_CS(0);
    for(i=0;i<8;i++) 
    {
        buf=(cmd>>(7-i))&0x1;
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
    unsigned short buf=0,temp;
    unsigned char i;
    TP_DIN(0);
    TP_DCLK(1);
    for(i=0;i<12;i++) 
    {
        Delayus(5);
        TP_DCLK(0);         
        Delayus(5);   
        temp= (TP_DOUT) ? 1:0;
        buf|=(temp<<(11-i));
        
        Delayus(5);
        TP_DCLK(1);
    }
    TP_CS(1);
    buf&=0x0fff;
    return(buf);
}
//====================================================================================
int Read_X(void) 
{ 
    int i;
    WR_CMD(CHX);
   // while(TP_BUSY);
    Delayus(5);
    i=RD_AD();
    return i;   
}
//====================================================================================
int Read_Y(void) 
{ 
    int i;
    WR_CMD(CHY);
    //while(TP_BUSY);
    Delayus(5);
    i=RD_AD();
    return i;    
}
//====================================================================================
void TP_GetAdXY(int *x,int *y) 
{
    int adx,ady;
    adx=Read_X();
    ady=Read_Y();
    *x=adx;
    *y=ady;
}

void TP_DrawPoint(u8 x,u16 y)
{
	if(x>220&&y<12)
	{
		ili9320_Clear(0xffff);
	}else 
	{

		ili9320_SetPoint(x,y,0xf800);     //中心点 
		ili9320_SetPoint(x+1,y,0xf800);
		ili9320_SetPoint(x,y+1,0xf800);
		ili9320_SetPoint(x+1,y+1,0xf800);	
	}		  	
}		
static int TP_X[1],TP_Y[1];

 
#define  VALUE 2 //差值

coordinate *Read_Ads7846(void)
{
    coordinate  Screen ;
    int m0,m1,m2;
	u8 count=0;
	u16 databuffer[2][9]={{0},{0}};//数据组
	u16 temp[3];

 
	 	TP_Init(); 
		
    do					  //循环读数9次
	{		   
            TP_GetAdXY(TP_X,TP_Y);  
			databuffer[0][count]=TP_X[0];  //X
			databuffer[1][count]=TP_Y[0];
			count++;  
	}
	while(!TP_INT_IN&& count<9);  //当为1时根本就不采样  初始自动打点BUG
	ReBack();
	if(count==9)//一定要读到9次数据,否则丢弃
	{  
	 temp[0]=(databuffer[0][0]+databuffer[0][1]+databuffer[0][2])/3;
	 temp[1]=(databuffer[0][3]+databuffer[0][4]+databuffer[0][5])/3;
	 temp[2]=(databuffer[0][6]+databuffer[0][7]+databuffer[0][8])/3;
	 m0=temp[0]-temp[1];
	 m1=temp[1]-temp[2];
	 m2=temp[2]-temp[0];

	 m0=m0>0?m0:(-m0);
	 m1=m1>0?m1:(-m1);
	 m2=m2>0?m2:(-m2);
	 if(m0>VALUE&&m1>VALUE&&m2>VALUE) return 0;
	 if(m0<m1){if(m2<m0) Screen.x=(temp[0]+temp[2])/2;
	             else Screen.x=(temp[0]+temp[1])/2;	}
	 else if(m2<m1) Screen.x=(temp[0]+temp[2])/2;
	             else Screen.x=(temp[1]+temp[2])/2;



     temp[0]=(databuffer[1][0]+databuffer[1][1]+databuffer[1][2])/3;
	 temp[1]=(databuffer[1][3]+databuffer[1][4]+databuffer[1][5])/3;
	 temp[2]=(databuffer[1][6]+databuffer[1][7]+databuffer[1][8])/3;
	 m0=temp[0]-temp[1];
	 m1=temp[1]-temp[2];
	 m2=temp[2]-temp[0];

	 m0=m0>0?m0:(-m0);
	 m1=m1>0?m1:(-m1);
	 m2=m2>0?m2:(-m2);
	 if(m0>VALUE&&m1>VALUE&&m2>VALUE) return 0;

	 if(m0<m1){if(m2<m0) Screen.y=(temp[0]+temp[2])/2;
	             else Screen.y=(temp[0]+temp[1])/2;	}
	 else if(m2<m1) Screen.y=(temp[0]+temp[2])/2;
	             else Screen.y=(temp[1]+temp[2])/2;
	  	return &Screen;
		}
	 Screen.x=0;  
	 Screen.y=0;
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


unsigned char setCalibrationMatrix( coordinate * displayPtr,
                          coordinate * screenPtr,
                          matrix * matrixPtr)
{

    unsigned char  retValue = 0 ;


    
    matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                         ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

    if( matrixPtr->Divider == 0 )
    {
        retValue = 1;
    }
    else
    {
        matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                        ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

        matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                        ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                        (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;

        matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                        ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
    
        matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                        ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                        (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
    }
 
    return( retValue ) ;
}
	//如果是1 则代表 是不成功的。。。0是成功的
unsigned char getDisplayPoint(coordinate * displayPtr,
                     coordinate * screenPtr,
                     matrix * matrixPtr )
{
    unsigned char  retValue =0 ;


    if( matrixPtr->Divider != 0 )
    {
	if(screenPtr->x==0&&screenPtr->y==0)
	  {
	  displayPtr->x=999;
	  displayPtr->y=999;
	  retValue = 1;  //没有按键 不成功
	  }
	else
		{
        displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                          (matrixPtr->Bn * screenPtr->y) + 
                           matrixPtr->Cn 
                        ) / matrixPtr->Divider ;

        displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                          (matrixPtr->En * screenPtr->y) + 
                           matrixPtr->Fn 
                        ) / matrixPtr->Divider ;
		}
    }
    else
    {
        retValue = 1;
    }

    return(retValue);
} 

//X坐标轴滤波算法
u8 TouchX_Penfilter(u16 *px)
{
u8 retval;
u16 tempx;
u16 dx;
static u8 count=0;
static u16 temp[2];
count++;
if(count>2)
{
count=2;
tempx=(temp[0]+*px)/2;
dx=(temp[1]>tempx)?(temp[1]-tempx):(tempx-temp[1]);
if(dx>10)
{
*px=temp[1];
retval=FALSE;
count=0;
}
else
{
temp[0]=temp[1];
temp[1]=*px;
retval=TRUE;
}
}
else
{
temp[0]=temp[1];
temp[1]=*px;
retval=FALSE;
}
return retval;
}

//Y坐标轴滤波算法
u8 TouchY_Penfilter(u16 *py)
{
u8 retval;
u16 tempy;
u16 dy;
static u8 count=0;
static u16 temp[2];
count++;
if(count>2)
{
count=2;
tempy=(temp[0]+*py)/2;
dy=(temp[1]>tempy)?(temp[1]-tempy):(tempy-temp[1]);
if(dy>10)
{
*py=temp[1];
retval=FALSE;
count=0;
}

else
{
temp[0]=temp[1];
temp[1]=*py;
retval=TRUE;
}
}
else
{
temp[0]=temp[1];
temp[1]=*py;
retval=FALSE;
}
return retval;
}

matrix mat ;
coordinate  display ;
Touch_Key  Touch_Pen;
//1代表滤波成功

u8 Touch_Coordinate(void)
{
getDisplayPoint(&display,Read_Ads7846(),&mat);    
if(TouchX_Penfilter(&display.x)||TouchY_Penfilter(&display.y) )
{
Touch_Pen.NowX=display.x; 	 //重新刷新坐标
Touch_Pen.NowY=display.y;
return 1;
}
else
{
Touch_Pen.NowX=display.x; 
Touch_Pen.NowY=display.y;
return 0;
}
}

//按键处理程序
//type:按键响应类型
//0,单点,定点,不扩展.一定要按键松开才返回
//1,单点,滑动,不扩展.滚动条操作/连加操作
//2,扩展按键支持:
//即:MOVE_LEFT,MOVE_RIGHT,MOVE_UP,MOVE_DOWN使能
u8 Touch_Key_Press(u8 type)
{   
	u16 tempx,tempy;//暂时保存X,Y坐标 	 	 
	u8 ml=0,mr=0,mu=0,md=0;//四个方向上移动次数  
	u8 first=1;	  		 	    					   		 
	//按键还是按下的
	//手动按触摸屏的时候,至少需要15ms才能退出这个循环	   	
	do
	{	    
		Touch_Pen.Key_NowState=UP;  //按键状态变为松开    
		if(Touch_Coordinate())      //成功读数
		{	 
			if(first)
			{
			Touch_Pen.LastX=Touch_Pen.NowX; 
            Touch_Pen.LastY=Touch_Pen.NowY;
			tempx=Touch_Pen.NowX;
			tempy=Touch_Pen.NowY;	
			first=0;  //标记清空
			}else if(type==2) //扩展按键														  
			{	  
				if(tempx>Touch_Pen.NowX)ml++;
				else mr++;	   	
				if(tempy>Touch_Pen.NowY)mu++;
				else md++;
				//设定一个门限值,不能让一次移动大于这个值,如果一次大于这个值
				//认为触摸屏误动作了.400的时候,反映比较慢
				if(fabs(tempx-Touch_Pen.NowX)>50||fabs(tempy-Touch_Pen.NowY)>50)//有抖动
				{
					ml=mr=mu=md=0;//全部清掉
					tempx=Touch_Pen.NowX=Touch_Pen.LastX;//坐标复位
					tempy=Touch_Pen.NowY=Touch_Pen.LastY;
					break;//退出数据采集,结果为点采集
				}   
				tempx=Touch_Pen.NowX;tempy=Touch_Pen.NowY;//转移临时坐标 
			}else if(type==1)break; 
			//printf("X:%d Y:%d\n",Pen_Point.X,Pen_Point.Y); 
		}
		delay_ms(10);//10ms消抖 						   		     								 	
	}while(!TP_INT_IN||Touch_Pen.Key_NowState==DOWN);//PEN=0或者按键状态为按下状态;
	delay_ms(50);
	Touch_Pen.Key_NowState=UP;//按键状态变为松开	 
 	//单次/不扩展 键值处理  
	//在 一定范围内
	if(fabs(tempx-Touch_Pen.LastX)<=10&&fabs(tempy-Touch_Pen.LastY)<=10||type<2)//单次按键/不扩展按键功能
	{	 
		return NONE;//没有移动  
	}	 
	//扩展键值处理
	if(fabs(tempx-Touch_Pen.LastX)<=25&&fabs(tempy-Touch_Pen.LastY)<=25)
	return 0;      //滑动距离最少要大于500

	if(fabs(ml-mr)>fabs(mu-md))//数量 满足
	{
		if(fabs(tempx-Touch_Pen.LastX)>fabs(tempy-Touch_Pen.LastY))//质量满足
		{
			if(tempx>Touch_Pen.LastX)return RIGHT;
			else return LEFT; 
		}else						//质量不满足
		{
			if(tempy>Touch_Pen.LastY)return DOWN;
			else return UP;
		}
	}else
	{
		if(fabs(tempy-Touch_Pen.LastY)>fabs(tempx-Touch_Pen.LastX))//质量满足
		{	    
			if(tempy>Touch_Pen.LastY)return DOWN;
			else return UP;			 
		}else						//质量不满足
		{	  
			if(tempx>Touch_Pen.LastX)return RIGHT;
			else return LEFT;
		}
	}   	  
}

//判断触点是不是在指定区域之内
//(x1,y1):起始坐标
//(x2,y2):结束坐标
//返回值 :1,在该区域内.0,不在该区域内.
u8 Is_In_Area(u8 x1,u16 y1,u8 x2,u16 y2)
{
if(Touch_Pen.NowX<=x2&&Touch_Pen.NowX>=x1&&Touch_Pen.NowY<=y2&&Touch_Pen.NowY>=y1)return 1;
else return 0;
}  


