//*****************************************************************************/
//VCC脚接电压范围为1.9V~3.6V之间
//除电源VCC和接地端，其余脚都可以直接和普通的5V单片机IO口直接相连，无需电平转换。对3V左右的单片机更加适用。
//硬件上面没有SPI的单片机也可以控制本模块，用普通单片机IO口模拟SPI不需要单片机真正的串口介入，只需要普通的
//单片机IO口就可以了，当然用串口也可以了（a:与51系列单片机P0口连接时候，需要加10K的上拉电阻,与其余口连接不需要。
//b:其他系列的单片机，如果是5V的，请参考该系列单片机IO口输出电流大小，如果超过10mA，需要串联电阻分压，否则容易烧
//毁模块!如果是3.3V的，可以直接和NRF24l01模块的IO口线连接。比如AVR系列单片机如果是5V的，一般串接2K的电阻）
//
//工作方式
//收发模式 配置模式 空闲模式 关机模式 工作模式由CE和寄存器内部PWR_UP、PRIM_RX共同控制
//			模式			PWR_UP			PRIM_RX				CE			FIFO寄存器状态
//		接收模式			1						1						1							-
//		发射模式			1						0						1				数据在TX FIFO寄存器中
//		发射模式			1						0					1->0			停留在发射模式，直至数据发送完
//		待机模式II		1						0						1				TX FIFO为空
//		待机模式I			1						-						0				无正在传输的数据
//		掉电模式			0						-						-				-
//
//1、收发模式: 收发模式有EnhancedShockBurstTM收发模式、ShockBurstTM收发模式和直接收发模式三种，收发模式由器件配置字决定

//1.1、EnhancedShockBurstTM收发模式 
//EnhancedShockBurstTM收发模式下，使用片内的先入先出堆栈区，数据低速从微控制器送入，但高速(1Mbps)发射，这样可以尽量节能，
//因此，使用低速的微控制器也能得到很高的射频数据发射速率。与射频协议相关的所有高速信号处理都在片内进行，这种做法有三大好处： 
//尽量节能；低的系统费用(低速微处理器也能进行高速射频发射)；
//数据在空中停留时间短，抗干扰性高。EnhancedShockBurstTM技术同时也减小了整个系统的平均工作电流。  
//在EnhancedShockBurstTM收发模式下， NRF24L01自动处理字头和CRC校验码。在接收数据时，自动把字头和CRC校验码
//移去。在发送数据时，自动加上字头和CRC校验码，在发送模式下，置CE为高，至少10us，将时发送过程完成后。

//1.1.1EnhancedShockBurstTM发射流程  
//A.把接收机的地址和要发送的数据按时序送入NRF24L01； 
//B.配置CONFIG寄存器，使之进入发送模式。  
//C.微控制器把CE置高（至少10us），激发NRF24L01进行EnhancedShockBurstTM发射；   
//D.N24L01的EnhancedShockBurstTM发射
//(1)给射频前端供电；
//(2)射频数据打包(加字头、CRC校验码)；
//(3)高速发射数据包;
//(4)发射完成，NRF24L01进入空闲状态。
//
//1.1.2EnhancedShockBurstTM接收流程  
//A.配置本机地址和要接收的数据包大小；  
//B.配置CONFIG寄存器，使之进入接收模式，把CE置高。 
//C.130us后，NRF24L01进入监视状态，等待数据包的到来；   
//D.当接收到正确的数据包(正确的地址和CRC校验码)，NRF2401自动把字头、地址和CRC校验位移去；  
//E.NRF24L01通过把STATUS寄存器的RX_DR置位(STATUS一般引起微控制器中断)通知微控制器；  
//F.微控制器把数据从NewMsg_RF2401读出；  
//G.所有数据读取完毕后，可以清除STATUS寄存器。NRF2401可以进入四种主要的模式之一。
//
//
//1.2ShockBurstTM收发模式
//ShockBurstTM收发模式可以与Nrf2401a,02,E1及E2兼容
//
//5.2空闲模式
//NRF24L01的空闲模式是为了减小平均工作电流而设计，其最大的优点是，实现节能的同时，缩短芯片的起动时间。在空闲模式下，
//部分片内晶振仍在工作，此时的工作电流跟外部晶振的频率有关。
//
//5.4关机模式  
//在关机模式下，为了得到最小的工作电流，一般此时的工作电流为900nA左右。关机模式下，配置字的内容也会被保持在NRF2401片内，这是该模式与断电状态最大的区别。
//
//引脚		方向		发送模式		接收模式		待机模式		掉电模式
// CE			输入		高电平>10us	高电平			低电平				-				Chip Enable Activates RX or TX mode
// CSN		输入		SPI片选使能，低电平使能
// SCK		输入		SPI时钟
//MOSI		输入		SPI串行输入
//MISO	三态输出	SPI串行输出
//IRQ			输出		中断，低电平使能
//
#include "NRF24L01.h"

typedef unsigned char uchar;
typedef unsigned char uint;

#ifdef USE_51_NO_SPI_INTERFACE
//EXT_OSC 11.0592M
	sbit  MISO = P1^3; 
	sbit  MOSI = P1^4; 
	sbit 	SCK  = P1^2; 
	sbit 	CE   = P1^1; 
	sbit 	CSN  = P3^2; 
	sbit 	IRQ  = P3^3;
	
	#define TX_ADR_WIDTH    5    // 5 uints TX address width 
	#define RX_ADR_WIDTH    5    // 5 uints RX address width 
	#define TX_PLOAD_WIDTH  20   // 20 uints TX payload 
	#define RX_PLOAD_WIDTH  20   // 20 uints TX payload  
	uint const TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01}; //本地地址 
	uint const RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01}; //接收地址
#endif

#define READ_REG        0x00   // 读寄存器指令 
#define WRITE_REG       0x20  // 写寄存器指令 
#define RD_RX_PLOAD     0x61   // 读取接收数据指令 
#define WR_TX_PLOAD     0xA0   // 写待发数据指令 
#define FLUSH_TX        0xE1  // 冲洗发送 FIFO指令 
#define FLUSH_RX        0xE2   // 冲洗接收 FIFO指令 
#define REUSE_TX_PL     0xE3   // 定义重复装载数据指令 
#define NOP             0xFF   // 保留

#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式 
#define EN_AA           0x01  // 自动应答功能设置 
#define EN_RXADDR       0x02  // 可用信道设置 
#define SETUP_AW        0x03  // 收发地址宽度设置 
#define SETUP_RETR      0x04  // 自动重发功能设置 
#define RF_CH           0x05  // 工作频率设置  
#define RF_SETUP        0x06  // 发射速率、功耗功能设置 
#define STATUS          0x07  // 状态寄存器 
#define OBSERVE_TX      0x08  // 发送监测功能  
#define CD              0x09  // 地址检测            
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址 
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址 
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址 
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址 
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址 
#define TX_ADDR         0x10  // 发送地址寄存器  
#define RX_PW_P0        0x11  // 接收频道0接收数据长度 
#define RX_PW_P1        0x12  // 接收频道0接收数据长度 
#define RX_PW_P2        0x13  // 接收频道0接收数据长度 
#define RX_PW_P3        0x14  // 接收频道0接收数据长度 
#define RX_PW_P4        0x15  // 接收频道0接收数据长度 
#define RX_PW_P5        0x16  // 接收频道0接收数据长度 
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置 

void Delay(unsigned int s);  
void inerDelay_us(unsigned char n); 
void init_NRF24L01(void); 
uint SPI_RW(uint uchar); 
uchar SPI_Read(uchar reg); 
void SetRX_Mode(void);  
uint SPI_RW_Reg(uchar reg, uchar value);  
uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars); 
uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars); 
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf); 
void nRF24L01_TxPacket(unsigned char * tx_buf); 

void Delay(unsigned int s) 
{  unsigned int i;  
	for(i=0; i<s; i++);  
	for(i=0; i<s; i++); 
}

uint bdata sta;   //状态标志
sbit RX_DR = sta^6; 
sbit TX_DS = sta^5;
sbit MAX_RT =sta^4;

void inerDelay_us(unsigned char n) 
{  for(;n>0;n--)   
	_nop_(); 
}

void init_NRF24L01(void) 
{      
	inerDelay_us(100);    
	CE=0;    // chip enable   
	CSN=1;   // Spi  disable    
	SCK=0;   //   
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址   
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址  
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      //  频道0自动 ACK应答允许   
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，如果需要多频道可以参考Page21    
	SPI_RW_Reg(WRITE_REG + RF_CH, 0);        //   设置信道工作为2.4GHZ，收发必须一致  
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节  
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);    //设置发射速率为1MHZ，发射功率为最大值0dB 
}


uint SPI_RW(uint uchar) 
{  
	uint bit_ctr;      
	for(bit_ctr=0;bit_ctr<8;bit_ctr++) // output 8-bit     
	{   
		MOSI = (uchar & 0x80);         // output 'uchar', MSB to MOSI   
		uchar = (uchar << 1);           // shift next bit into MSB.. 
  	SCK = 1;                      // Set SCK high..   
  	uchar |= MISO;           // capture current MISO bit   
  	SCK = 0;                // ..then set SCK low again     
  	}      
  	return(uchar);               // return read uchar 
}

uchar SPI_Read(uchar reg)
{
	uchar reg_val;
	CSN = 0; // CSN low, initialize SPI communication...
	SPI_RW(reg); // Select register to read from.. 
	reg_val = SPI_RW(0); // ..then read registervalue 
	CSN = 1; // CSN high, terminate SPI communication
	return(reg_val); // return register value
}

uint SPI_RW_Reg(uchar reg, uchar value)
{
	uint status;
	CSN = 0; // CSN low, init SPI transaction
	status = SPI_RW(reg); // select register
	SPI_RW(value); // ..and write value to it..
	CSN = 1; //CSN high again
	return(status); // return nRF24L01 status uchar
}

uint  SPI_Read_Buf(uchar  reg,  uchar  *pBuf,  uchar  uchars)  
{    
	uint  status,uchar_ctr;        
	CSN  =  0;     //  Set  CSN  low,  init  SPI  tranaction    
	status  =  SPI_RW(reg);     //  Select  register  to  write  to  and  read  status  uchar        
	for(uchar_ctr=0;uchar_ctr<uchars;uchar_ctr++)      
		pBuf[uchar_ctr]  =  SPI_RW(0);   // 
	CSN  =  1;                                                              
	return(status);   //  return  nRF24L01  status  uchar  
}  

uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars) 
{  
	uint status,uchar_ctr;    
	CSN = 0;            //SPI使能         
	status = SPI_RW(reg);     
	for(uchar_ctr=0; uchar_ctr<uchars; uchar_ctr++) //   
		SPI_RW(*pBuf++);  
	CSN = 1;        //关闭SPI  
	return(status);    //  
} 

void SetRX_Mode(void) 
{  
	CE=0;  
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // IRQ收发完成中断响应，16位CRC ，主接收
	CE = 1;   
	inerDelay_us(130); 
}

unsigned char nRF24L01_RxPacket(unsigned char* rx_buf) 
{      
	unsigned char revale=0;  
	sta=SPI_Read(STATUS); // 读取状态寄存其来判断数据接收状况  
	if(RX_DR)    // 判断是否接收到数据  
	{      
		CE = 0;    //SPI使能   
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer   
		revale =1;   //读取数据完成标志  
	}  
	SPI_RW_Reg(WRITE_REG+STATUS,sta);   //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志  
	return revale; 
} 

void nRF24L01_TxPacket(unsigned char * tx_buf) 
{  
	CE=0;   //StandBy I模式   
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址  
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);     // 装载数据   
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      // IRQ收发完成中断响应，16位CRC，主发送  
	CE=1;   //置高CE，激发数据发送  
	inerDelay_us(10); 
}


void main(void) 
{  
	unsigned char tf =0;  
	unsigned char TxBuf[20]={0};  //   
	unsigned char RxBuf[20]={0};      
	init_NRF24L01()   TxBuf[1] = 1   nRF24L01_TxPacket(TxBuf); // Transmit Tx buffer data  
	Delay(6000);  
	while(1)  
	{      
		if(KEY ==0 )      
		{    
			led=0;    
			Delay(200);    
			led=1;        
			TxBuf[1] = 1        
			tf = 1        
		}     
		if (tf==1)        
		{     
			nRF24L01_TxPacket(TxBuf); // Transmit Tx buffer data    
			TxBuf[1] = 0x00;    
			tf=0;    
			Delay(1000);     
		} 
		//***********************************************************************************************   
		SetRX_Mode();   
		nRF24L01_RxPacket(RxBuf);      
		if(RxBuf[1])   
		{         
			if( RxBuf[1]==1)    
			{        
				led=0;     
				Delay(200);     
				led=1;    
			}    
			Delay(1000);   
		}     
		RxBuf[1] = 0x00;  
	}
}