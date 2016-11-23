#include "Ultrasound_HC_SR04.h"

//**********************************************************************************************/
//51
//																					TMOD
//	|		GATE	 |		C/T		|			M1		|			M0		|		GATE		|		C/T		|		M1		|		M0		|
//	|								定时/计数器T1									|								 定时/计数器T0						|
//
//	GATE:	GATE = 0,不需要外部中断引脚参与控制
//				GATE = 1,定时/计数器由TR1/TR0和外部中断引脚共同控制
//
//	C/T:	C/T = 1, 设定为计数器
//				C/T = 0, 设定为定时器
//
//	M1、M0:	00:13位
//					01:16位
//					10:8位自动重装
//					11:8位
//																					TCON
//	|		TF1		|		TR1		|		TF0		|		TR0		|		IE1		|		IT1		|		IE0		|		IT0		|
//	|					定时/计数器T1、T0控制位				|					外部中断1、外部中断0控制位		|
//
//	TF1、TF0：溢出标志位，系统自动置位或清零、用户不能写入
//	TF1 = 1:T1溢出
//	TF0 = 1:T0溢出
//
//	TR1:T1启动或停止位
//	TR1 = 1,启动；TR1 = 0，停止
//	TR0:T0启动或停止位
//	TR0 = 1,启动；TR0 = 0，停止
//
//	IE0(IE1)：外部中断请求标志位
//	IT0(IT1):外部中断触发方式控制位                   //选择有效信号
//            IT0(IT1)=1:脉冲触发方式,下降沿有效。
//            IT0(IT1)=0:电平触发方式,低电平有效。
//
//																														IE
//	|				EA			|				--				|				--				|				ES			|				ET1			|				EX1				|				ET0			|				EX0				|
//	|中断总控制位		|									|									|串行中断控制位	|定时器T1控制位	|外部中断1控制位	|定时器T0控制位	|外部中断0控制位	|
//
//	EA:系统中断允许总控制位
//	EA = 1,允许系统中断
//	EA = 0,禁止系统中断
//	
//	ET1:T1中断允许控制位
//	ET1 = 1,允许T1中断
//	ET1 = 0,禁止T1中断
//	
//	ET0:T0中断允许控制位
//	ET0 = 1,允许T0中断
//	ET0 = 0,禁止T0中断
//
//	EX0(EX1):外部中断允许控制位
//  EX0=1 外部中断0开关闭合   //开外部0中断
//  EX0=0 外部中断0开关断开
//
//  ES: 串口中断允许控制位     
//  ES=1 串口中断开关闭合     //开串口中断
//  ES=0 串口中断开关断开
//
//						T1
//	|		TH1		|		TL1		|
//
// 优先级 高 -> 低
//	（P3.2）外部中断0 -> T0 -> （P3.3）外部中断1 -> T1 -> 串口
//						0						1							2						3				4

unsigned int tick = 0;

#ifdef MCU_USE_51
	#if defined USE_T0_AS_TIME_BASE
		#define	TMOD_VALUE	02H
		#define TIMERH	TH0
		#define TIMERL	TL0
		#define EN_TMR_IT	ET0
		#define TRIG_TMR	TR0
		void TMR_ISR() interrupt1
		{
			tick++;
			if(tick == 2)
			{
				TRIG_IO = 0;
			}
		}
	#elif	defined USE_T1_AS_TIME_BASE
		#define TMOD_VALUE	20H
		#define TIMERH	TH1
		#define TIMERL	TL1
		#define EN_TMR_IT	ET1
		#define TRIG_TMR	TR1
		void TMR_ISR() interrupt3
		{
			
		}
	#endif
	
	#if defined USE_EXT_OSC_110592
		#define V_TH	F6H		//10us
		#define V_TL	F6H
	#elif defined USE_EXT_OSC_120000
		#define V_TH	F6H
		#define V_TL	F6H
	#endif
	
	#if defined USE_EX0
		void EX_ISR() interrupt0
		{
			
		}
	#elif defined USE_EX1
		void EX_ISR() interrupt1
		{
			
		}
	#endif
	#define TRIG_IO 
	#define 
#endif

void DeInit(void)
{
	#ifdef MCU_USE_51
		TMOD = TMOD_VALUE;
		TIMERH = V_TH;
		TIMERL = V_TL;
		TRIG_IO = 0;
		EN_TMR_IT = 1;
		TRIG_TMR = 0;
		EA = 0;		
	#endif
}

void Cal_Signal(void)
{
	#ifdef MCU_USE_51
		EA = 1;
		TRIG_IO = 1;
		TRIG_TMR = 1;
		while(TRIG_IO);
	#endif
}

int GetData(void)
{
	DeInit();
}





