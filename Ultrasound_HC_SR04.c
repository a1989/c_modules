#include "Ultrasound_HC_SR04.h"

//**********************************************************************************************/
//51
//																					TMOD
//	|		GATE	 |		C/T		|			M1		|			M0		|		GATE		|		C/T		|		M1		|		M0		|
//	|								��ʱ/������T1									|								 ��ʱ/������T0						|
//
//	GATE:	GATE = 0,����Ҫ�ⲿ�ж����Ų������
//				GATE = 1,��ʱ/��������TR1/TR0���ⲿ�ж����Ź�ͬ����
//
//	C/T:	C/T = 1, �趨Ϊ������
//				C/T = 0, �趨Ϊ��ʱ��
//
//	M1��M0:	00:13λ
//					01:16λ
//					10:8λ�Զ���װ
//					11:8λ
//																					TCON
//	|		TF1		|		TR1		|		TF0		|		TR0		|		IE1		|		IT1		|		IE0		|		IT0		|
//	|					��ʱ/������T1��T0����λ				|					�ⲿ�ж�1���ⲿ�ж�0����λ		|
//
//	TF1��TF0�������־λ��ϵͳ�Զ���λ�����㡢�û�����д��
//	TF1 = 1:T1���
//	TF0 = 1:T0���
//
//	TR1:T1������ֹͣλ
//	TR1 = 1,������TR1 = 0��ֹͣ
//	TR0:T0������ֹͣλ
//	TR0 = 1,������TR0 = 0��ֹͣ
//
//	IE0(IE1)���ⲿ�ж������־λ
//	IT0(IT1):�ⲿ�жϴ�����ʽ����λ                   //ѡ����Ч�ź�
//            IT0(IT1)=1:���崥����ʽ,�½�����Ч��
//            IT0(IT1)=0:��ƽ������ʽ,�͵�ƽ��Ч��
//
//																														IE
//	|				EA			|				--				|				--				|				ES			|				ET1			|				EX1				|				ET0			|				EX0				|
//	|�ж��ܿ���λ		|									|									|�����жϿ���λ	|��ʱ��T1����λ	|�ⲿ�ж�1����λ	|��ʱ��T0����λ	|�ⲿ�ж�0����λ	|
//
//	EA:ϵͳ�ж������ܿ���λ
//	EA = 1,����ϵͳ�ж�
//	EA = 0,��ֹϵͳ�ж�
//	
//	ET1:T1�ж��������λ
//	ET1 = 1,����T1�ж�
//	ET1 = 0,��ֹT1�ж�
//	
//	ET0:T0�ж��������λ
//	ET0 = 1,����T0�ж�
//	ET0 = 0,��ֹT0�ж�
//
//	EX0(EX1):�ⲿ�ж��������λ
//  EX0=1 �ⲿ�ж�0���رպ�   //���ⲿ0�ж�
//  EX0=0 �ⲿ�ж�0���ضϿ�
//
//  ES: �����ж��������λ     
//  ES=1 �����жϿ��رպ�     //�������ж�
//  ES=0 �����жϿ��ضϿ�
//
//						T1
//	|		TH1		|		TL1		|
//
// ���ȼ� �� -> ��
//	��P3.2���ⲿ�ж�0 -> T0 -> ��P3.3���ⲿ�ж�1 -> T1 -> ����
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





