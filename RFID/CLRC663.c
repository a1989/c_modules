#include "CLRC663.h"
#include "gpio.h"
#include "Timer.h"
#include <stdio.h>
#include <string.h>
#include "CAN_Process.h"
#include "usart.h"

#define REG_VERSION		0x7F
#define REG_PRODUCT_ID	0x1


#define rRegCommand 					0x00 // Starts and stops command execution
#define rRegHostCtrl 				0x01 // Host control register
#define rRegFIFOControl 			0x02 // Control register of the FIFO
#define rRegWaterLevel 			0x03 // Level of the FIFO underflow and overflow warning
#define rRegFIFOLength 			0x04 // Length of the FIFO
#define rRegFIFOData 				0x05 // Data In/Out exchange register of FIFO buffer
#define rRegIRQ0 						0x06 // Interrupt register 0
#define rRegIRQ1 						0x07 // Interrupt register 1
#define rRegIRQ0En 					0x08 // Interrupt enable register 0
#define rRegIRQ1En 					0x09 // Interrupt enable register 1
#define rRegError 						0x0A // Error bits showing the error status of the last command execution
#define rRegStatus 					0x0B // Contains status of the communication
#define rRegRxBitCtrl 				0x0C // Control register for anticollision adjustments for bit oriented protocols
#define rRegRxColl 					0x0D // Collision position register
#define rRegTControl 				0x0E // Control of Timer 0..3
#define rRegT0Control 				0x0F // Control of Timer0
#define rRegT0ReloadHi 			0x10 // High register of the reload value of Timer0
#define rRegT0ReloadLo 			0x11 // Low register of the reload value of Timer0
#define rRegT0CounterValHi 	0x12 // Counter value high register of Timer0
#define rRegT0CounterValLo 	0x13 // Counter value low register of Timer0
#define rRegT1Control 				0x14 // Control of Timer1
#define rRegT1ReloadHi 			0x15 // High register of the reload value of Timer1
#define rRegT1ReloadLo 			0x16 // Low register of the reload value of Timer1
#define rRegT1CounterValHi 	0x17 // Counter value high register of Timer1
#define rRegT1CounterValLo 	0x18 // Counter value low register of Timer1
#define rRegT2Control 				0x19 // Control of Timer2
#define rRegT2ReloadHi 			0x1A // High byte of the reload value of Timer2
#define rRegT2ReloadLo 			0x1B // Low byte of the reload value of Timer2
#define rRegT2CounterValHi 	0x1C // Counter value high byte of Timer2
#define rRegT2CounterValLo 	0x1D // Counter value low byte of Timer2
#define rRegT3Control 				0x1E // Control of Timer3
#define rRegT3ReloadHi 			0x1F // High byte of the reload value of Timer3
#define rRegT3ReloadLo 			0x20 // Low byte of the reload value of Timer3
#define rRegT3CounterValHi 	0x21 // Counter value high byte of Timer3
#define rRegT3CounterValLo 	0x22 // Counter value low byte of Timer3
#define rRegT4Control 				0x23 // Control of Timer4
#define rRegT4ReloadHi 			0x24 // High byte of the reload value of Timer4
#define rRegT4ReloadLo 			0x25 // Low byte of the reload value of Timer4
#define rRegT4CounterValHi 	0x26 // Counter value high byte of Timer4
#define rRegT4CounterValLo 	0x27 // Counter value low byte of Timer4
#define rRegDrvMod 					0x28 // Driver mode register
#define rRegTxAmp 						0x29 // Transmitter amplifier register
#define rRegDrvCon 					0x2A // Driver configuration register
#define rRegTxl 							0x2B // Transmitter register
#define rRegTxCrcPreset 			0x2C // Transmitter CRC control register, preset value
#define rRegRxCrcPreset 			0x2D // Receiver CRC control register, preset value
#define rRegTxDataNum 				0x2E // Transmitter data number register
#define rRegTxModWidth 			0x2F // Transmitter modulation width register
#define rRegTxSym10BurstLen 	0x30 // Transmitter symbol 1 + symbol 0 burst length register
#define rRegTXWaitCtrl 			0x31 // Transmitter wait control
#define rRegTxWaitLo 				0x32 // Transmitter wait low
#define rRegFrameCon 				0x33 // Transmitter frame control
#define rRegRxSofD 					0x34 // Receiver start of frame detection
#define rRegRxCtrl 					0x35 // Receiver control register
#define rRegRxWait 					0x36 // Receiver wait register
#define rRegRxThreshold 			0x37 // Receiver threshold register
#define rRegRcv 							0x38 // Receiver register
#define rRegRxAna 						0x39 // Receiver analog register
#define rRegRFU_3A						0x3A // -
#define rRegSerialSpeed 			0x3B // Serial speed register
#define rRegLFO_Trimm 				0x3C // Low-power oscillator trimming register
#define rRegPLL_Ctrl 				0x3D // IntegerN PLL control register, for microcontroller clock output adjustment
#define rRegPLL_DivOut 			0x3E // IntegerN PLL control register, for microcontroller clock output adjustment
#define rRegLPCD_QMin 				0x3F // Low-power card detection Q channel minimum threshold
#define rRegLPCD_QMax 				0x40 // Low-power card detection Q channel maximum threshold
#define rRegLPCD_IMin 				0x41 // Low-power card detection I channel minimum threshold
#define rRegLPCD_I_Result 		0x42 // Low-power card detection I channel result register
#define rRegLPCD_Q_Result 		0x43 // Low-power card detection Q channel result register
#define rRegPadEn 						0x44 // PIN enable register
#define rRegPadOut 					0x45 // PIN out register
#define rRegPadIn 						0x46 // PIN in register
#define rRegSigOut 					0x47 // Enables and controls the SIGOUT Pin
#define rRegTxBitMod 				0x48 // Transmitter bit mode register
#define rRegRFU_49						0x49 // -
#define rRegTxDataCon 				0x4A // Transmitter data configuration register
#define rRegTxDataMod 				0x4B // Transmitter data modulation register
#define rRegTxSymFreq 				0x4C // Transmitter symbol frequency
#define rRegTxSym0H 					0x4D // Transmitter symbol 0 high register
#define rRegTxSym0L 					0x4E // Transmitter symbol 0 low register
#define rRegTxSym1H 					0x4F // Transmitter symbol 1 high register
#define rRegTxSym1L 					0x50 // Transmitter symbol 1 low register
#define rRegTxSym2 					0x51 // Transmitter symbol 2 register
#define rRegTxSym3 					0x52 // Transmitter symbol 3 register
#define rRegTxSym10Len 			0x53 // Transmitter symbol 1 + symbol 0 length register
#define rRegTxSym32Len 			0x54 // Transmitter symbol 3 + symbol 2 length register
#define rRegTxSym10BurstCtrl 0x55 // Transmitter symbol 1 + symbol 0 burst control register
#define rRegTxSym10Mod 			0x56 // Transmitter symbol 1 + symbol 0 modulation register
#define rRegTxSym32Mod 			0x57 // Transmitter symbol 3 + symbol 2 modulation register
#define rRegRxBitMod 				0x58 // Receiver bit modulation register
#define rRegRxEofSym 				0x59 // Receiver end of frame symbol register
#define rRegRxSyncValH 			0x5A // Receiver synchronisation value high register
#define rRegRxSyncValL 			0x5B // Receiver synchronisation value low register
#define rRegRxSyncMod 				0x5C // Receiver synchronisation mode register
#define rRegRxMod 						0x5D // Receiver modulation register
#define rRegRxCorr 					0x5E // Receiver correlation register
#define rRegFabCal 					0x5F // Calibration register of the receiver, calibration performed at production
#define rReg_60 							0x60 //
#define rReg_61 							0x61 //
#define rReg_66 							0x66 //
#define rReg_6A 							0x6A //
#define rReg_6B 							0x6B //
#define rReg_6C 							0x6C //
#define rReg_6D 							0x6D //
#define rReg_6E 							0x6E //
#define rReg_6F 							0x6F //
#define rRegVersion 					0x7F // Version and subversion register
//////////////////////////////////////////////////////////
//		Command 			No. Parameter (bytes) 	Short description
#define RC663_Idle 				0x00 //- 					no action, cancels current command execution
#define RC663_LPCD 				0x01 //- 					low-power card detection
#define RC663_LoadKey 		0x02 //(keybyte1..6); 		reads a MIFARE key (size of 6 bytes) from FIFO buffer and puts it into Key buffer
#define RC663_MFAuthent 	0x03 //60h or 61h,(block address),(card serial number byte0..3) 	performs the MIFARE standard authentication in MIFARE read/write mode only
#define RC663_AckReq 			0x04 //- 					performs a query, an Ack and a Req-Rn for ISO/IEC 18000-3 mode 3/ EPC Class-1 HF
#define RC663_Receive 		0x05 //- 					activates the receive circuit
#define RC663_Transmit 		0x06 //- 					transmits data from the FIFO buffer
#define RC663_Transceive 	0x07 //- 					transmits data from the FIFO buffer and automatically activates the receiver after transmission finished
#define RC663_WriteE2 		0x08 //addressH, addressL, data; 	gets one byte from FIFO buffer and writes it to the internal EEPROM, 
#define RC663_WriteE2Page 0x09 //(page Address), data0, [data1..data63]; 	gets up to 64 bytes (one EEPROM page) from the FIFO buffer and writes it to the EEPROM
#define RC663_ReadE2 			0x0A // address H, addressL,length; 	reads data from the EEPROM and copies it into the FIFO buffer
#define RC663_LoadReg 		0x0C //(EEPROM addressL), (EEPROM addressH), RegAdr, (number of Register to be copied);  reads data from the internal EEPROM and initializes the CLRC663 registers. EEPROM address needs to be within EEPROM sector 2
#define RC663_LoadProtocol 0x0D //(Protocol number RX), (Protocol number TX);		reads data from the internal EEPROM and initializes the CLRC663 registers needed for a Protocol change
#define RC663_LoadKeyE2 	0x0E //KeyNr; 				copies a key of the EEPROM into the key buffer
#define RC663_StoreKeyE2 	0x0F //KeyNr, byte1..6;	stores a MIFARE key (size of 6 bytes) into the EEPROM
#define RC663_ReadRNR 		0x1C //- 					Copies bytes from the Random Number generator into the FIFO until the FiFo is full
#define RC663_Soft_Reset 	0x1F //- 					resets the CLRC663


#define PICC_REQIDL           0x26               //Ñ°ÌìÏßÇøÄÚÎ´½øÈëÐÝÃß×´Ì¬
#define PICC_REQALL           0x52               //Ñ°ÌìÏßÇøÄÚÈ«²¿¿¨
#define PICC_ANTICOLL1        0x93               //·À³å×²
#define PICC_ANTICOLL2        0x95               //·À³å×²
#define PICC_AUTHENT1A        0x60               //ÑéÖ¤AÃÜÔ¿
#define PICC_AUTHENT1B        0x61               //ÑéÖ¤BÃÜÔ¿
#define PICC_READ             0x30               //¶Á¿é
#define PICC_WRITE            0xA0               //Ð´¿é
#define PICC_DECREMENT        0xC0               //¿Û¿î
#define PICC_INCREMENT        0xC1               //³äÖµ
#define PICC_RESTORE          0xC2               //µ÷¿éÊý¾Ýµ½»º³åÇø
#define PICC_TRANSFER         0xB0               //±£´æ»º³åÇøÖÐÊý¾Ý
#define PICC_HALT             0x50               //ÐÝÃß


#define PICC_REQIDL           0x26               //Ѱ̬ϟǸĚδ½øȫНß״̬
#define PICC_REQALL           0x52               //Ѱ̬ϟǸĚȫ²¿¿¨
#define PICC_ANTICOLL1        0x93               //·À³嗲
#define PICC_ANTICOLL2        0x95               //·À³嗲
#define PICC_AUTHENT1A        0x60               //ѩ֤AÜԿ
#define PICC_AUTHENT1B        0x61               //ѩ֤BÜԿ
#define PICC_READ             0x30               //¶Á¿鍊#define PICC_WRITE            0xA0               //д¿鍊#define PICC_DECREMENT        0xC0               //¿ۿ#define PICC_INCREMENT        0xC1               //³䖵
#define PICC_RESTORE          0xC2               //µ÷¿銽¾ݵ½»º³凸
#define PICC_TRANSFER         0xB0               //±£´滺³凸֐ʽ¾ݍ
#define PICC_HALT             0x50               //Нß

/////////////////////////////////////////////////////////////////////ISO14443B
#define PICC_ANTI        			0x05 
#define PICC_ATTRIB        		0x1D 
/////////////////////////////////////////////////////////////////////
#define MI_OK                          0
#define MI_CHK_OK                      0

#define MI_NOTAGERR                    (-1)
#define MI_CHK_FAILED                  (-1)
#define MI_CRCERR                      (-2)
#define MI_CHK_COMPERR                 (-2)
#define MI_EMPTY                       (-3)
#define MI_AUTHERR                     (-4)
#define MI_PARITYERR                   (-5)
#define MI_CODEERR                     (-6)
#define MI_SERNRERR                    (-8)
#define MI_KEYERR                      (-9)
#define MI_NOTAUTHERR                  (-10)
#define MI_BITCOUNTERR                 (-11)
#define MI_BYTECOUNTERR                (-12)
#define MI_IDLE                        (-13)
#define MI_TRANSERR                    (-14)
#define MI_WRITEERR                    (-15)
#define MI_INCRERR                     (-16)
#define MI_DECRERR                     (-17)
#define MI_READERR                     (-18)
#define MI_OVFLERR                     (-19)
#define MI_POLLING                     (-20)
#define MI_FRAMINGERR                  (-21)
#define MI_ACCESSERR                   (-22)
#define MI_UNKNOWN_COMMAND             (-23)
#define MI_COLLERR                     (-24)
#define MI_RESETERR                    (-25)
#define MI_INITERR                     (-25)
#define MI_INTERFACEERR                (-26)
#define MI_ACCESSTIMEOUT               (-27)
#define MI_NOBITWISEANTICOLL           (-28)
#define MI_QUIT                        (-30)
#define MI_RECBUF_OVERFLOW             (-50)
#define MI_SENDBYTENR                  (-51)
#define MI_SENDBUF_OVERFLOW            (-53)
#define MI_BAUDRATE_NOT_SUPPORTED      (-54)
#define MI_SAME_BAUDRATE_REQUIRED      (-55)
#define MI_WRONG_PARAMETER_VALUE       (-60)
#define MI_BREAK                       (-99)
#define MI_NY_IMPLEMENTED              (-100)
#define MI_NO_MFRC                     (-101)
#define MI_MFRC_NOTAUTH                (-102)
#define MI_WRONG_DES_MODE              (-103)
#define MI_HOST_AUTH_FAILED            (-104)
#define MI_WRONG_LOAD_MODE             (-106)
#define MI_WRONG_DESKEY                (-107)
#define MI_MKLOAD_FAILED               (-108)
#define MI_FIFOERR                     (-109)
#define MI_WRONG_ADDR                  (-110)
#define MI_DESKEYLOAD_FAILED           (-111)
#define MI_WRONG_SEL_CNT               (-114)
#define MI_WRONG_TEST_MODE             (-117)
#define MI_TEST_FAILED                 (-118)
#define MI_TOC_ERROR                   (-119)
#define MI_COMM_ABORT                  (-120)
#define MI_INVALID_BASE                (-121)
#define MI_MFRC_RESET                  (-122)
#define MI_WRONG_VALUE                 (-123)
#define MI_VALERR                      (-124)
#define MI_COM_ERR                     (-125)
#define MI_ERR						   					(-126)


#define PDWON_HIGH		HAL_GPIO_WritePin(PDOWN_GPIO_Port, PDOWN_Pin, GPIO_PIN_SET)
#define PDWON_LOW		HAL_GPIO_WritePin(PDOWN_GPIO_Port, PDOWN_Pin, GPIO_PIN_RESET)

#define SPI_CS_HIGH		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)
#define SPI_CS_LOW		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)


typedef enum
{
	RESULT_BUSY = 0,
	RESULT_NO_RESPOND,
	RESULT_ERROR,
	RESULT_SUCCESS,
}ExecuteState;


typedef ProcessState (*ProcessFuncType)(void);

typedef struct
{
	uint8_t u8Params[4];
	ProcessFuncType pConfigFunc;
	uint8_t u8Index;
}ProcessList;


void CLRC663_Init(void)
{
	//HAL_GPIO_WritePin(PDOWN_GPIO_Port, PDOWN_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
}


//uint8_t ReadVersion(void)
//{
//	return CLRC663_ReadReg(REG_PRODUCT_ID);
//}	


#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte


//struct TranSciveBuffer
//{
//	uint8_t Command;
//	uint16_t  Length;
//	uint8_t Data[DEF_FIFO_LENGTH];
//};


typedef struct
{
	uint8_t u8Command;
	uint8_t  u8Length;
	uint8_t u8Data[DEF_FIFO_LENGTH];
}CommandStruct;


#define EXECUTE_FUNCTION(ProcessIndex, FuncIndex, NextIndex, Function, ...)		\
do \
{\
	if (ProcessIndex == FuncIndex) \
	{\
		eState = Function(__VA_ARGS__);\
		if (eState != STATE_SUCCESS)\
		{\
			if (eState == STATE_FAILED || eState == STATE_TIMEOUT) \
			{\
				/*printf("%s failed,process:%d,state:%d\r\n", __FUNCTION__, FuncIndex, eState);\*/ \
				FuncIndex = 0;\
			}\
			return eState;\
		}\
		else \
		{\
			/*printf("%s success,process:%d\r\n", __FUNCTION__, FuncIndex);\*/ \
			FuncIndex = NextIndex;\
			return eState;\
		}\
	}\
}\
while(0)
	

#define EXECUTE_PROCESS_START(ProcessIndex, FuncIndex)	\
if (ProcessIndex == FuncIndex)\
{\
	
#define EXECUTE_PROCESS_END	\
}

#define DATA_BUFFER_GROUP	128


typedef enum
{
	EVENT_NONE = 0,
	EVENT_READ_DATA,
	EVENT_WRITE_DATA,
}RfidEvent;

struct
{
	bool bIntStatus;
	bool bSendFinish;
	bool bRecvFinish;	
	uint8_t u8Version;
	uint16_t u16Tag;
	uint32_t u32UID;
	uint8_t u8SAK;
	RfidEvent eEvent;
	uint16_t u16ReadRegDelay;
	uint16_t u16WriteRegDelay;
	struct
	{
		uint8_t u8Block;
		uint8_t u8M1_Data[7];
		uint8_t u8Key[6];
	}sDataHandler;
	struct
	{
		uint8_t u8Data[DATA_BUFFER_GROUP][8];
		uint8_t u8GroupIndex;
		uint16_t u16ReadByteNum;
		uint8_t u8WriteFrameNum;
		uint16_t u16WriteByteNum;
		uint16_t u16WriteCheckSum;
		uint32_t u32RecvTimer;
		uint32_t u32TimeUseTest;
		uint32_t u32TimeStepUse;
	}sSendRecvHandler;
}g_sRC663_Manager = {0};





void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	g_sRC663_Manager.bSendFinish = true;
}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	g_sRC663_Manager.bRecvFinish = true;
//}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	g_sRC663_Manager.bRecvFinish = true;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	g_sRC663_Manager.bIntStatus = true;
}


ProcessState CLRC663_ReadReg(const uint8_t u8RegAddr, uint8_t *u8RegVal)
{
	static uint8_t u8Step = 0;
	static uint8_t u8SendData[2] = {0};
	static uint8_t u8RecvData[2] = {0};
	static ProcessState eState = STATE_IDLE;
	HAL_StatusTypeDef eStatus;
	static uint32_t u32TimerCount = 0;
	static uint16_t u16UsTimerCount = 0;
	
	switch (u8Step)
	{
		case 0:
			g_sRC663_Manager.bSendFinish = false;
			eState = STATE_BUSY;
			SPI_CS_LOW;
			u8Step = 5;
			break;
		
		case 5:
			u8SendData[0] = (u8RegAddr << 1) | 0x1;
			u8SendData[1] = 0;
			u8Step = 10;
			break;
		
		case 10:
			eStatus = HAL_SPI_TransmitReceive_DMA(&hspi2, &u8SendData[0], &u8RecvData[0], 1);
			if (HAL_OK == eStatus)
			{
				u8Step = 15;
				ResetTimerCount(&u32TimerCount);
			}
			else
			{
				eState = STATE_FAILED;
				u8Step = 0;
			}
			break;
		
		case 15:
			if (g_sRC663_Manager.bRecvFinish)
			{
				g_sRC663_Manager.bRecvFinish = false;
				u8Step = 20;
				ResetTimerCount(&u32TimerCount);
			}
			else
			{
				if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 10)
				{
					eState = STATE_TIMEOUT;
					u8Step = 0;
				}
			}
			break;

			
		case 20:
			eStatus = HAL_SPI_TransmitReceive_DMA(&hspi2, &u8SendData[1], &u8RecvData[1], 1);
			if (HAL_OK == eStatus)
			{
				u8Step = 25;
				ResetTimerCount(&u32TimerCount);
			}
			else
			{
				eState = STATE_FAILED;
				u8Step = 0;
			}
			break;
			
		case 25:
			if (g_sRC663_Manager.bRecvFinish)
			{
				SPI_CS_HIGH;
				g_sRC663_Manager.bRecvFinish = false;
				u8Step = 30;
				ResetTimerCountUs(&u16UsTimerCount);
			}
			else
			{
				if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 10)
				{
					eState = STATE_TIMEOUT;
					u8Step = 0;
				}
			}
			break;
			
		case 30:
			if (GetTimerTickDeltaUs(u16UsTimerCount, GetCurTimerCountUs()) > 100)
			{
				*u8RegVal = u8RecvData[1];
				eState = STATE_SUCCESS;
				u8Step = 0;
			}
			break;
	}
	
	return eState;
}


ProcessState CLRC663_WriteReg(const uint8_t u8RegAddr, const uint8_t u8RegVal)
{  
	static uint8_t u8Step = 0;
	static uint8_t u8SendData[2] = {0};
	static uint8_t u8RecvData[2] = {0};
	uint8_t u8ReadValue = 0;
	static ProcessState eState = STATE_IDLE;
	HAL_StatusTypeDef eStatus;
	static uint32_t u32TimerCount = 0;
	static uint16_t u16UsTimerCount = 0;
	
	switch (u8Step)
	{
		case 0:
			g_sRC663_Manager.bRecvFinish = false;
			eState = STATE_BUSY;
			SPI_CS_LOW;
			u8Step = 5;
			break;
		
		case 5:
			u8SendData[0] = u8RegAddr << 1;
			u8SendData[1] = u8RegVal;
			u8Step = 10;
			break;
		
		case 10:
			eStatus = HAL_SPI_TransmitReceive_DMA(&hspi2, &u8SendData[0], &u8RecvData[0], 1);
			if (HAL_OK == eStatus)
			{
				u8Step = 15;
				ResetTimerCount(&u32TimerCount);
			}
			else
			{
				eState = STATE_FAILED;
				u8Step = 0;
			}
			break;
		
		case 15:
			if (g_sRC663_Manager.bRecvFinish)
			{
				g_sRC663_Manager.bRecvFinish = false;
				u8Step = 20;
				ResetTimerCount(&u32TimerCount);
			}
			else
			{
				if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 10)
				{
					eState = STATE_TIMEOUT;
					u8Step = 0;
				}
			}
			break;
			
		case 20:
			eStatus = HAL_SPI_TransmitReceive_DMA(&hspi2, &u8SendData[1], &u8RecvData[1], 1);
			if (HAL_OK == eStatus)
			{
				u8Step = 25;
				ResetTimerCount(&u32TimerCount);
			}
			else
			{
				eState = STATE_FAILED;
				u8Step = 0;
			}
			break;
			
		case 25:
			if (g_sRC663_Manager.bRecvFinish)
			{
				SPI_CS_HIGH;
				g_sRC663_Manager.bRecvFinish = false;
				u8Step = 30;
				ResetTimerCountUs(&u16UsTimerCount);
			}
			else
			{
				if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 10)
				{
					eState = STATE_TIMEOUT;
					u8Step = 0;
				}
			}
			break;
						
		case 30:
			if (GetTimerTickDeltaUs(u16UsTimerCount, GetCurTimerCountUs()) > 100)
			{
				eState = STATE_SUCCESS;
				u8Step = 0;
//				u8Step = 35;
			}
			break;
			
//		case 35:
//			if (STATE_SUCCESS == CLRC663_ReadReg(u8RegAddr, &u8ReadValue))
//			{
//				//printf("write value:%x,read value:%x\r\n", u8RegVal, u8ReadValue);
//				eState = STATE_SUCCESS;
//				u8Step = 0;
//			}
//			break;
	}
	
	return eState;
}


ProcessState CLRC663_SetBitMask(uint8_t u8Reg, uint8_t u8Mask)  
{
	ProcessState eState = STATE_IDLE;
	static ProcessState eFuncState = STATE_IDLE;
	static uint8_t u8Step = 0;
	static uint8_t u8ReadValue = 0;
	
	switch (u8Step)
	{
		case 0:
			eFuncState = STATE_BUSY;
			eState = CLRC663_ReadReg(u8Reg, &u8ReadValue);
			if (STATE_SUCCESS == eState)
			{
				u8Step = 5;
			}
			break;
		
		case 5:
			u8ReadValue = u8Mask | u8ReadValue;
			eState = CLRC663_WriteReg(u8Reg, u8ReadValue);
			if (STATE_SUCCESS == eState)
			{
				eFuncState = STATE_SUCCESS;
				u8Step = 0;
			}
			break;
	}
	
	if (STATE_FAILED == eState || STATE_TIMEOUT == eState)
	{
		eFuncState = eState;
		u8Step = 0;
	}	
	
	return eFuncState;
}


ProcessState CLRC663_ClearBitMask(uint8_t u8Reg, uint8_t u8Mask) 
{
	ProcessState eState = STATE_IDLE;
	static ProcessState eFuncState = STATE_IDLE;
	static uint8_t u8Step = 0;
	static uint8_t u8ReadValue = 0;
	
	switch (u8Step)
	{
		case 0:
			eFuncState = STATE_BUSY;
			eState = CLRC663_ReadReg(u8Reg, &u8ReadValue);
			if (STATE_SUCCESS == eState)
			{
				u8Step = 5;
			}
			break;
		
		case 5:
			u8ReadValue = ~u8Mask & u8ReadValue;
			eState = CLRC663_WriteReg(u8Reg, u8ReadValue);
			if (STATE_SUCCESS == eState)
			{
				eFuncState = STATE_SUCCESS;
				u8Step = 0;
			}
			break;
	}
	
	if (STATE_FAILED == eState || STATE_TIMEOUT == eState)
	{
		eFuncState = eState;
		u8Step = 0;
	}	
	
	return eFuncState;	
}


ProcessState CLRC663_FieldOn(void) 
{
	ProcessState eState = CLRC663_SetBitMask(rRegDrvMod,0x08);
	if (STATE_SUCCESS == eState)
	{
		eState = STATE_FINISH;
	}
	
	return eState;
}

ProcessState CLRC663_FieldOff(void) 
{
	ProcessState eState = CLRC663_ClearBitMask(rRegDrvMod,0x08);
	if (STATE_SUCCESS == eState)
	{
		eState = STATE_FINISH;
	}
	
	return eState;
}


uint8_t u8Mode = 0;
ProcessState RC663_Command_Int(CommandStruct *pCommand, int8_t *pStatus)
{
	static uint8_t u8FuncIndex = 0;	
	ProcessState eState = STATE_BUSY;
	static int8_t sStatus = 0;
	static uint16_t u16Count = 0;
	static uint8_t u8ReadValue = 0;
	static uint8_t u8ReadData = 0;
	static uint32_t u32TimerCount = 0;
	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegCommand, RC663_Idle);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_SetBitMask, rRegFIFOControl, 0x10);
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_WriteReg, rRegIRQ0, 0x7F);
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_WriteReg, rRegIRQ1, 0x7F);
	
	EXECUTE_PROCESS_START(4, u8FuncIndex)
		if (u16Count < pCommand->u8Length)
		{
			u8FuncIndex = 5;
		}
		else
		{
			u16Count = 0;
			u8FuncIndex = 7;
		}
	EXECUTE_PROCESS_END		
	EXECUTE_FUNCTION(5, u8FuncIndex, 6, CLRC663_WriteReg, rRegFIFOData, pCommand->u8Data[u16Count]);			
	EXECUTE_PROCESS_START(6, u8FuncIndex)
		u16Count++;
		u8FuncIndex = 4;
		return STATE_BUSY;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(7, u8FuncIndex)
		if (pCommand->u8Command & 0x80)
		{
			u8FuncIndex = 8;
		}
		else
		{
			u8FuncIndex = 21;
		}
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(8, u8FuncIndex, 9, CLRC663_WriteReg, rRegIRQ0En, 0x90);		
	
	EXECUTE_PROCESS_START(9, u8FuncIndex)
		if (u8Mode)
		{
			u8FuncIndex = 10;
		}
		else
		{
			u8FuncIndex = 11;
		}
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(10, u8FuncIndex, 12, CLRC663_WriteReg, rRegIRQ1En, 0xE0);					
	EXECUTE_FUNCTION(11, u8FuncIndex, 12, CLRC663_WriteReg, rRegIRQ1En, 0xE8);
		
		
	EXECUTE_PROCESS_START(12, u8FuncIndex)
		g_sRC663_Manager.bIntStatus = false;
		u8FuncIndex = 13;
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(14, u8FuncIndex, 15, CLRC663_WriteReg, rRegCommand, pCommand->u8Command);
		
	EXECUTE_PROCESS_START(15, u8FuncIndex)
		ResetTimerCount(&u32TimerCount);
		u8FuncIndex = 16;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(16, u8FuncIndex)
		if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 10)
		{
			// 超时处理
			u8FuncIndex = 17;
		}
		else
		{
			return STATE_BUSY;
		}
//		if (g_sRC663_Manager.bIntStatus)
//		{			
//			g_sRC663_Manager.bIntStatus = false;
//			u8FuncIndex = 17;
//		}
//		else
//		{
//			if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 3000)
//			{
//				// 超时处理
//				u8FuncIndex = 0;
//				return STATE_TIMEOUT;
//			}
//		}
//		return STATE_BUSY;
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(17, u8FuncIndex, 18, CLRC663_WriteReg, rRegIRQ0En, 0x10);
		
	EXECUTE_PROCESS_START(18, u8FuncIndex)
		if (u8Mode)
		{
			u8FuncIndex = 19;
		}
		else
		{
			u8FuncIndex = 20;
		}
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(19, u8FuncIndex, 22, CLRC663_WriteReg, rRegIRQ1En, 0x20);			
	EXECUTE_FUNCTION(20, u8FuncIndex, 22, CLRC663_WriteReg, rRegIRQ1En, 0x28);
				
	EXECUTE_FUNCTION(21, u8FuncIndex, 22, CLRC663_WriteReg, rRegCommand, pCommand->u8Command);
		
	EXECUTE_PROCESS_START(22, u8FuncIndex)
		if (u16Count < 2000)
		{
			u8FuncIndex = 23;
		}
		else
		{
			u8FuncIndex = 25;
		}
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(23, u8FuncIndex, 24, CLRC663_ReadReg, rRegIRQ0, &u8ReadValue);
	//printf("%s rRegIRQ0:%d\r\n", __FUNCTION__, u8ReadValue);
		
	EXECUTE_PROCESS_START(24, u8FuncIndex)
		if (u8ReadValue & 0x10)
		{
			u8FuncIndex++;
		}
		else
		{
			u16Count++;
			u8FuncIndex = 22;
		}
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(25, u8FuncIndex)
		if (u16Count >= 2000)
		{
			u8FuncIndex = 50;
			*pStatus = MI_ERR;
		}
		u16Count = 0;
		u8FuncIndex = 26;
	EXECUTE_PROCESS_END
	
		
	EXECUTE_FUNCTION(26, u8FuncIndex, 27, CLRC663_ReadReg, rRegFIFOLength, &u8ReadValue);	
		
	EXECUTE_PROCESS_START(27, u8FuncIndex)
		if (u16Count < u8ReadValue)
		{
			u8FuncIndex = 28;
		}
		else
		{
			u8FuncIndex = 50;
		}
	EXECUTE_PROCESS_END		
	EXECUTE_FUNCTION(28, u8FuncIndex, 29, CLRC663_ReadReg, rRegFIFOData, &u8ReadData);			
	EXECUTE_PROCESS_START(29, u8FuncIndex)
		pCommand->u8Data[u16Count] = u8ReadData;
		u16Count++;
		u8FuncIndex = 27;
		return STATE_BUSY;
	EXECUTE_PROCESS_END
		
	u8FuncIndex = 0;
	u16Count = 0;	
	*pStatus = MI_OK;
	return STATE_FINISH;
}


ProcessState CLRC663_LoadProtocol(const uint8_t u8Rx, const uint8_t u8Tx)
{
	static CommandStruct sCommand;
	sCommand.u8Command = RC663_LoadProtocol;
	sCommand.u8Length = 2;
	sCommand.u8Data[0] = u8Rx;
	sCommand.u8Data[1] = u8Tx;
	
	int8_t s8Status = MI_ERR;
	ProcessState eState = RC663_Command_Int(&sCommand, &s8Status);
	if (STATE_FINISH == eState && MI_OK == s8Status)
	{
		eState = STATE_SUCCESS;
	}
	else if (STATE_SUCCESS == eState)
	{
		eState = STATE_BUSY;
	}
	
	return eState;
}


ProcessState CLRC663_PcdConfigISOType(uint8_t u8Type, int8_t *pStatus)
{
	static uint8_t u8FuncIndex = 0;	
	static CommandStruct sCommand;
	ProcessState eState = STATE_BUSY;
	static int8_t sStatus = 0;

	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegT0Control, 0x98);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_WriteReg, rRegT1Control, 0x92);
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_WriteReg, rRegT2Control, 0x20);
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_WriteReg, rRegT2ReloadHi, 0x03);
	EXECUTE_FUNCTION(4, u8FuncIndex, 5, CLRC663_WriteReg, rRegT2ReloadLo, 0xFF);
	EXECUTE_FUNCTION(5, u8FuncIndex, 6, CLRC663_WriteReg, rRegT3Control, 0x00);
	
	EXECUTE_FUNCTION(6, u8FuncIndex, 7, CLRC663_WriteReg, rRegWaterLevel, 0x10);
	EXECUTE_FUNCTION(7, u8FuncIndex, 8, CLRC663_WriteReg, rRegRxBitCtrl, 0x80);
	EXECUTE_FUNCTION(8, u8FuncIndex, 9, CLRC663_WriteReg, rRegDrvMod, 0x80);
	EXECUTE_FUNCTION(9, u8FuncIndex, 10, CLRC663_WriteReg, rRegTxAmp, 0xC0);
	EXECUTE_FUNCTION(10, u8FuncIndex, 11, CLRC663_WriteReg, rRegDrvCon, 0x09);
	EXECUTE_FUNCTION(11, u8FuncIndex, 12, CLRC663_WriteReg, rRegTxl, 0x05);
	EXECUTE_FUNCTION(12, u8FuncIndex, 13, CLRC663_WriteReg, rRegRxSofD, 0x00);
	
	EXECUTE_FUNCTION(13, u8FuncIndex, 14, CLRC663_LoadProtocol, 0, 0);
	
	EXECUTE_FUNCTION(14, u8FuncIndex, 15, CLRC663_WriteReg, rRegIRQ0En, 0);
	EXECUTE_FUNCTION(15, u8FuncIndex, 16, CLRC663_WriteReg, rRegIRQ1En, 0);
	EXECUTE_FUNCTION(16, u8FuncIndex, 17, CLRC663_WriteReg, rRegFIFOControl, 0xB0);
	EXECUTE_FUNCTION(17, u8FuncIndex, 18,CLRC663_WriteReg, rRegTxModWidth, 0x20);
	EXECUTE_FUNCTION(18, u8FuncIndex, 19, CLRC663_WriteReg, rRegTxSym10BurstLen, 0);
	EXECUTE_FUNCTION(19, u8FuncIndex, 20, CLRC663_WriteReg, rRegFrameCon, 0xCF);
	EXECUTE_FUNCTION(20, u8FuncIndex, 21, CLRC663_WriteReg, rRegRxCtrl, 0x04);
	
	EXECUTE_FUNCTION(21, u8FuncIndex, 22, CLRC663_WriteReg, rRegRxThreshold, 0x55);
	EXECUTE_FUNCTION(22, u8FuncIndex, 23, CLRC663_WriteReg, rRegRcv, 0x12);
	EXECUTE_FUNCTION(23, u8FuncIndex, 24, CLRC663_WriteReg, rRegRxAna, 0x0A);
	EXECUTE_FUNCTION(24, u8FuncIndex, 25, CLRC663_WriteReg, rRegDrvMod, 0x81);
	EXECUTE_FUNCTION(25, u8FuncIndex, 26, CLRC663_WriteReg, rRegStatus, 0);
	EXECUTE_FUNCTION(26, u8FuncIndex, 27, CLRC663_WriteReg, rRegDrvMod, 0x89);
	
		
	*pStatus = MI_OK;
	u8FuncIndex = 0;
		
	return STATE_FINISH;
}


//ProcessState CLRC663_PcdConfigISOType(uint8_t u8Type, int8_t *pStatus)
//{
//	static uint8_t u8FuncIndex = 0;	
//	static CommandStruct sCommand;
//	ProcessState eState = STATE_BUSY;
//	static int8_t sStatus = 0;

//	
//	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegCommand, RC663_Idle);
//	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_WriteReg, rRegFIFOControl, 0xB0);
//	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_WriteReg, rRegFIFOData, 0x0);
//	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_WriteReg, rRegFIFOData, 0x0);
//	EXECUTE_FUNCTION(4, u8FuncIndex, 5, CLRC663_WriteReg, rRegCommand, RC663_LoadProtocol);
//	EXECUTE_FUNCTION(5, u8FuncIndex, 6, CLRC663_WriteReg, rRegFIFOControl, 0xB0);
//	
//	EXECUTE_FUNCTION(6, u8FuncIndex, 7, CLRC663_WriteReg, rRegDrvMod, 0x8E);
//	EXECUTE_FUNCTION(7, u8FuncIndex, 8, CLRC663_WriteReg, rRegIRQ0, 0x7F);
//	EXECUTE_FUNCTION(8, u8FuncIndex, 9, CLRC663_WriteReg, rRegTxCrcPreset, 0x18);
//	EXECUTE_FUNCTION(9, u8FuncIndex, 10, CLRC663_WriteReg, rRegRxCrcPreset, 0x18);
//	EXECUTE_FUNCTION(10, u8FuncIndex, 11, CLRC663_WriteReg, rRegTxDataNum, 0x0F);	
//	EXECUTE_FUNCTION(11, u8FuncIndex, 12, CLRC663_WriteReg, rRegFIFOData, PICC_REQIDL);
//	EXECUTE_FUNCTION(12, u8FuncIndex, 13, CLRC663_WriteReg, rRegCommand, RC663_Transceive);
//		
//	*pStatus = MI_OK;
//	u8FuncIndex = 0;
//		
//	return STATE_FINISH;
//}




volatile uint8_t u8IndexMonitor = 0;
ProcessState CLRC663_PcdComTransceive(CommandStruct *pCommand, int8_t *s8Result)
{
	static uint8_t u8FuncIndex = 0;	
	
	static uint8_t u8Count = 0;
	ProcessState eState = STATE_IDLE;
	static uint8_t u8Index = 0;
	static uint8_t u8RegValue = 0;
	static uint8_t u8ErrValue = 0;
	static uint8_t u8TempValue = 0;
	static uint8_t u8LastBits = 0;
	static bool bTimeout = false;
	static uint32_t u32TimeCount = 0;
	static uint8_t u8NoTagCount = 0;
	
	u8IndexMonitor = u8FuncIndex;
	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegCommand, RC663_Idle);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_SetBitMask, rRegFIFOControl, 0x10);
	
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_WriteReg, rRegIRQ0, 0x7F);
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_WriteReg, rRegIRQ1, 0x7F);
	
	EXECUTE_PROCESS_START(4, u8FuncIndex)
		if (u8Count < pCommand->u8Length)
		{
			u8FuncIndex = 5;
		}
		else
		{
			u8Count = 0;
			u8FuncIndex = 7;			
		}
	EXECUTE_PROCESS_END	
	EXECUTE_FUNCTION(5, u8FuncIndex, 6, CLRC663_WriteReg, rRegFIFOData, pCommand->u8Data[u8Count]);		
		
	EXECUTE_PROCESS_START(6, u8FuncIndex)
		u8Count++;
		u8FuncIndex = 4;
		return STATE_BUSY;
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(7, u8FuncIndex, 8, CLRC663_WriteReg, rRegIRQ0En, 0x18);
	EXECUTE_FUNCTION(8, u8FuncIndex, 9, CLRC663_WriteReg, rRegIRQ1En, 0x42);
	EXECUTE_FUNCTION(9, u8FuncIndex, 10, CLRC663_WriteReg, rRegCommand, RC663_Transceive);
				
	EXECUTE_PROCESS_START(10, u8FuncIndex)
		ResetTimerCount(&u32TimeCount);
		u8FuncIndex = 11;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(11, u8FuncIndex)
		u8IndexMonitor = u8FuncIndex;		
		if (u8RegValue & 0x40)
		{
			//printf("%s rRegIRQ1:%d\r\n", __FUNCTION__, u8RegValue);
			u8RegValue = 0;
			u8FuncIndex = 13;
		}
		else
		{
			if (GetTimerTickDelta(u32TimeCount, GetCurTimerCount()) > 3000)
			{
				u8FuncIndex = 0;
				return STATE_TIMEOUT;
			}
			else
			{
				u8FuncIndex = 12;
			}
		}
	EXECUTE_PROCESS_END		
	EXECUTE_FUNCTION(12, u8FuncIndex, 11, CLRC663_ReadReg, rRegIRQ1, &u8RegValue);
		
		
	EXECUTE_FUNCTION(13, u8FuncIndex, 14, CLRC663_WriteReg, rRegIRQ0En, 0x54);
	EXECUTE_FUNCTION(14, u8FuncIndex, 15, CLRC663_WriteReg, rRegIRQ1En, 0x42);
		
	EXECUTE_PROCESS_START(15, u8FuncIndex)
		ResetTimerCount(&u32TimeCount);
		u8FuncIndex = 16;
	EXECUTE_PROCESS_END	
	EXECUTE_PROCESS_START(16, u8FuncIndex)
		u8IndexMonitor = u8FuncIndex;
		if (u8RegValue & 0x40)
		{
			u8FuncIndex = 18;			
		}
		else
		{
			if (GetTimerTickDelta(u32TimeCount, GetCurTimerCount()) > 3000)
			{
				bTimeout = true;
				u8FuncIndex = 18;		
			}
			else
			{
				u8NoTagCount = 0;
				u8FuncIndex = 17;		
			}				
		}
	EXECUTE_PROCESS_END		
	EXECUTE_FUNCTION(17, u8FuncIndex, 16, CLRC663_ReadReg, rRegIRQ1, &u8RegValue);
		
		
	EXECUTE_FUNCTION(18, u8FuncIndex, 19, CLRC663_WriteReg, rRegIRQ0En, 0);
	EXECUTE_FUNCTION(19, u8FuncIndex, 20, CLRC663_WriteReg, rRegIRQ1En, 0);
		
	EXECUTE_FUNCTION(20, u8FuncIndex, 21, CLRC663_ReadReg, rRegError, &u8ErrValue);
		
	EXECUTE_PROCESS_START(21, u8FuncIndex)
		//printf("%s u8RegValue:%d\r\n", __FUNCTION__, u8RegValue);
		if (bTimeout)
		{
			bTimeout = false;
			*s8Result = MI_QUIT;
			u8FuncIndex = 30;
		}
		else if (u8RegValue & 0x02)
		{

			*s8Result = MI_NOTAGERR;
			u8FuncIndex = 30;
		}
		else if (u8ErrValue)
		{
			if (u8ErrValue & 0x04)
			{
				*s8Result = MI_COLLERR;
			}
			else if (u8ErrValue & 0x01)
			{
				*s8Result = MI_FRAMINGERR;
			}
			else
			{
				*s8Result = MI_ERR;
			}
			
			u8FuncIndex = 30;
		}
		else
		{
			*s8Result = MI_OK;
			if (pCommand->u8Command == RC663_Transceive)
			{
				u8FuncIndex = 22;
			}
		}
	EXECUTE_PROCESS_END
	
	EXECUTE_FUNCTION(22, u8FuncIndex, 23, CLRC663_ReadReg, rRegFIFOLength, &u8TempValue);
	EXECUTE_FUNCTION(23, u8FuncIndex, 24, CLRC663_ReadReg, rRegRxBitCtrl, &u8LastBits);
		
	EXECUTE_PROCESS_START(24, u8FuncIndex)
		u8LastBits = u8LastBits & 0x07;
		if (u8LastBits)
		{
			pCommand->u8Length = (u8TempValue - 1) * 8 + u8LastBits;
		}
		else
		{
			pCommand->u8Length = u8TempValue * 8;
		}
		
		if (0 == u8TempValue)
		{
			u8TempValue = 1;
		}
		
		if (u8TempValue > 250)
		{
			u8TempValue = 250;
		}
		
		u8FuncIndex = 25;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(25, u8FuncIndex)
		if (u8Count < u8TempValue)
		{
			u8FuncIndex = 26;			
		}
		else
		{
			u8FuncIndex = 30;		
		}
	EXECUTE_PROCESS_END		
	EXECUTE_FUNCTION(26, u8FuncIndex, 27, CLRC663_ReadReg, rRegFIFOData, &u8RegValue);		
	EXECUTE_PROCESS_START(27, u8FuncIndex)
		pCommand->u8Data[u8Count] = u8RegValue;
		u8FuncIndex = 25;
		u8Count++;
		return STATE_BUSY;
	EXECUTE_PROCESS_END
	
	u8Count = 0;	
	u8FuncIndex = 0;
	u8RegValue = 0;
	return STATE_FINISH;
}



ProcessState CLRC663_PcdRequestA(uint8_t u8ReqCode, uint8_t *pTagType, int8_t *pStatus)
{
	static uint8_t u8FuncIndex = 0;	
	static CommandStruct sCommand;
	ProcessState eState = STATE_IDLE;
	static int8_t sStatus = MI_ERR;
	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegTxCrcPreset, 0x18);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_WriteReg, rRegRxCrcPreset, 0x18);
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_WriteReg, rRegStatus, 0);
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_WriteReg, rRegTXWaitCtrl, 0xC0);
	EXECUTE_FUNCTION(4, u8FuncIndex, 5, CLRC663_WriteReg, rRegTxWaitLo, 0x0B);
	EXECUTE_FUNCTION(5, u8FuncIndex, 6, CLRC663_WriteReg, rRegT0ReloadHi, 0x08);
	EXECUTE_FUNCTION(6, u8FuncIndex, 7, CLRC663_WriteReg, rRegT0ReloadLo, 0x94);
	EXECUTE_FUNCTION(7, u8FuncIndex, 8, CLRC663_WriteReg, rRegT1ReloadHi, 0);
	EXECUTE_FUNCTION(8, u8FuncIndex, 9, CLRC663_WriteReg, rRegT1ReloadLo, 0x40);
	EXECUTE_FUNCTION(9, u8FuncIndex, 10, CLRC663_WriteReg, rRegIRQ0, 0x08);
	EXECUTE_FUNCTION(10, u8FuncIndex, 11, CLRC663_WriteReg, rRegRxWait, 0x90);
	EXECUTE_FUNCTION(11, u8FuncIndex, 12, CLRC663_WriteReg, rRegTxDataNum, 0x0F);
	
	EXECUTE_PROCESS_START(12, u8FuncIndex)
		sCommand.u8Command = RC663_Transceive;
		sCommand.u8Length = 1;
		sCommand.u8Data[0] = u8ReqCode;
		u8FuncIndex = 13;
	EXECUTE_PROCESS_END		
	
	EXECUTE_PROCESS_START(13, u8FuncIndex)
		eState = CLRC663_PcdComTransceive(&sCommand, &sStatus);
		if (STATE_FINISH == eState)
		{
			u8FuncIndex = 14;
		}
		else
		{
			return eState;
		}
	EXECUTE_PROCESS_END	
	
	EXECUTE_PROCESS_START(14, u8FuncIndex)
		if (sStatus == MI_OK)
		{
			if (sCommand.u8Length == 0x10)
			{
				*pTagType = sCommand.u8Data[0];
				*(pTagType + 1) = sCommand.u8Data[1];
			}
			else
			{
				sStatus = MI_VALERR;
			}
		}
	EXECUTE_PROCESS_END
		
	//printf("Status:%d,index:%d\r\n", sStatus, u8FuncIndex);
	*pStatus = sStatus;
	
	sStatus = MI_ERR;
	u8FuncIndex = 0;
		
	return STATE_FINISH;
}



ProcessState CLRC663_SetRawRC(uint8_t u8Reg, uint8_t u8Mask, uint8_t u8Set)
{
	ProcessState eState = STATE_BUSY;
	static uint8_t u8Step = 0;
	static uint8_t u8Value = 0;
	static uint8_t u8WriteValue = 0;
	
	switch (u8Step)
	{
		case 0:
			if (STATE_SUCCESS == CLRC663_ReadReg(u8Reg, &u8Value))
			{
				u8WriteValue = (u8Value & u8Mask) | u8Set;
				u8Step = 5;
			}
			break;
			
		case 5:
			if (STATE_SUCCESS == CLRC663_WriteReg(u8Reg, u8WriteValue))
			{
				eState = STATE_SUCCESS;
				u8Step = 0;
			}
			break;
	}
	
	return eState;
}




ProcessState CLRC663_PcdAnticoll(uint8_t *pSnr, int8_t *pStatus)
{
	static uint8_t u8FuncIndex = 0;	
	static uint8_t u8CollPosition = 0;
	static uint8_t u8Bytes = 0;
	static uint8_t u8Bits = 0;
	ProcessState eState = STATE_IDLE;
	
	static CommandStruct sCommand;
	uint8_t u8SNR[5] = {0, 0, 0, 0 ,0};
	static int8_t sStatus = 0;
	static uint8_t u8Temp = 0;
	uint8_t u8SnrCheck = 0;
	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegTxDataNum, 0x08);
	
	EXECUTE_PROCESS_START(1, u8FuncIndex)
		u8Bits = (u8CollPosition) % 8;
		if (u8Bits != 0)
		{
			u8Bytes = u8CollPosition / 8 + 1;
			u8FuncIndex++;
		}
		else
		{
			u8Bytes = u8CollPosition / 8;
			u8FuncIndex = 4;
		}
	EXECUTE_PROCESS_END
	
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_SetRawRC, rRegRxBitCtrl, 0x8f, (u8Bits << 4));
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_SetRawRC, rRegTxDataNum, 0xf8, u8Bits);
		
	EXECUTE_PROCESS_START(4, u8FuncIndex)
		sCommand.u8Command = RC663_Transceive;
		sCommand.u8Data[0] = PICC_ANTICOLL1;
		sCommand.u8Data[1] = 0x20 + ((u8CollPosition / 8) << 4) + (u8Bits & 0x0F);
		for (uint8_t i = 0; i < u8Bytes; i++)
		{
			sCommand.u8Data[i + 2] = u8SNR[i];			
		}
		sCommand.u8Length = u8Bytes + 2;
		u8FuncIndex = 5;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(5, u8FuncIndex)
		eState = CLRC663_PcdComTransceive(&sCommand, &sStatus);
		if (STATE_FINISH == eState)
		{
			u8FuncIndex = 6;
		}
		else
		{
			return eState;
		}
	EXECUTE_PROCESS_END	
	//EXECUTE_FUNCTION(5, u8FuncIndex, 6, CLRC663_PcdComTransceive, &sCommand, &sStatus);
		
	EXECUTE_PROCESS_START(6, u8FuncIndex)
		u8Temp = u8SNR[(u8CollPosition / 8)];
		if (sStatus == MI_COLLERR)
		{
			for (uint8_t i = 0; i < 5 - (u8CollPosition / 8); i++)
			{
				u8SNR[i + (u8CollPosition / 8)] = sCommand.u8Data[i+1];
			}
			u8SNR[(u8CollPosition / 8)] |= u8Temp;
			u8CollPosition = sCommand.u8Data[0];
		}
		else if (sStatus == MI_OK)
		{
			for (uint8_t i = 0; i < (sCommand.u8Length / 8); i++)
			{
				u8SNR[4 - i] = sCommand.u8Data[sCommand.u8Length / 8 - i - 1];
			}
			u8SNR[(u8CollPosition / 8)] |= u8Temp;
		}
		u8FuncIndex++;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(7, u8FuncIndex)
		if (sStatus == MI_COLLERR)
		{
			// 判断是否超时
			u8FuncIndex = 4;
		}
		else
		{
			u8FuncIndex++;
		}
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(8, u8FuncIndex)
		if (sStatus == MI_OK)
		{
			uint8_t i = 0;
			for (i = 0; i < 4; i++)
			{   
				*(pSnr + i) = u8SNR[i];
				u8SnrCheck ^= u8SNR[i];
			}
			if (u8SnrCheck != u8SNR[i])
			{
				sStatus = MI_COM_ERR;
			}
		}		
	EXECUTE_PROCESS_END
		
	*pStatus = sStatus;
	u8FuncIndex = 0;
		
	return STATE_FINISH;
}


ProcessState CLRC663_PcdSelect(const uint8_t *pSnr, uint8_t *pSize, int8_t *pStatus)
{
	static uint8_t u8FuncIndex = 0;	
	ProcessState eState = STATE_IDLE;
	static CommandStruct sCommand;
	static int8_t sStatus = 0;
	uint8_t u8SnrCheck = 0;

	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_SetRawRC, rRegTxCrcPreset, 0xfe, 0x01);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_SetRawRC, rRegRxCrcPreset, 0xfe, 0x01);
	
	EXECUTE_PROCESS_START(2, u8FuncIndex)
		sCommand.u8Command = RC663_Transceive;
		sCommand.u8Length  = 7;
		sCommand.u8Data[0] = PICC_ANTICOLL1;
		sCommand.u8Data[1] = 0x70;
		for (uint8_t i = 0; i < 4; i++)
		{
			u8SnrCheck ^= *(pSnr + i);
			sCommand.u8Data[i + 2] = *(pSnr + i);
		}
		sCommand.u8Data[6] = u8SnrCheck;	
		u8FuncIndex = 3;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(3, u8FuncIndex)
		eState = CLRC663_PcdComTransceive(&sCommand, &sStatus);
		if (STATE_FINISH == eState)
		{
			u8FuncIndex = 4;
		}
		else
		{
			return eState;
		}
	EXECUTE_PROCESS_END	
	//EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_PcdComTransceive, &sCommand, &sStatus);	
		
	EXECUTE_PROCESS_START(4, u8FuncIndex)
		if (sStatus == MI_OK)
		{
			if (sCommand.u8Length != 0x8)
			{
				sStatus = MI_BITCOUNTERR;
			}
			else
			{
				*pSize = sCommand.u8Data[0];
			}
		}
	EXECUTE_PROCESS_END
		
	*pStatus = sStatus;
	u8FuncIndex = 0;
		
	return STATE_FINISH;	
}




//s8 CLRC663_PcdRead(u8   addr,u8 *pReaddata)
//{
//	s8 status;
//	struct TranSciveBuffer ComData,*pi = &ComData;
//	
//	RC663_SetRawRC(rRegTxCrcPreset,0xfe,0x01);	//on
//	RC663_SetRawRC(rRegRxCrcPreset,0xfe,0x00);	//off
//	
//  ComData.Command = RC663_Transceive;
//	ComData.Length  = 2;
//	ComData.Data[0] = PICC_READ;
//	ComData.Data[1] = addr;
//	
//	status = RC663_PcdComTransceive(pi);
//  if (status == MI_OK)
//	{
//		if (ComData.Length != 0x90)
//			status = MI_BITCOUNTERR;
//		else
//			memcpy(pReaddata, &ComData.Data[0], 16);
//	}
//	return status;
//}

ProcessState MifareClassicConfig(void)
{
	int8_t s8Status = 0;
	ProcessState eState = CLRC663_PcdConfigISOType('A', &s8Status);
	if ( (STATE_FINISH == eState) && (MI_OK == s8Status) )
	{
		eState = STATE_SUCCESS;
	}
	else
	{
		eState = STATE_BUSY;
	}
	
	return eState;
}

uint8_t u8IRQ0Value = 0;
uint8_t u8Tag[2] = {0};
ProcessState GetCardInfo(void)
{
	static uint8_t u8FuncIndex = 0;	
	ProcessState eState = STATE_IDLE;
	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_ReadReg, rRegIRQ0, &u8IRQ0Value);	
	EXECUTE_PROCESS_START(1, u8FuncIndex)
		if (u8IRQ0Value & 0x04)
		{
			u8FuncIndex = 2;
		}
		else
		{
			u8FuncIndex = 0;
			return STATE_IDLE;
		}
	EXECUTE_PROCESS_END
	
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_ReadReg, rRegFIFOData, &u8Tag[0]);	
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_ReadReg, rRegFIFOData, &u8Tag[1]);	

	u8FuncIndex = 0;
	return STATE_FINISH;	
}

// 检查type A卡是否在射频场中
ProcessState MifareClassicReqA(uint8_t *pTag)
{
	static uint8_t u8FuncIndex = 0;	
	static CommandStruct sCommand;
	ProcessState eState = STATE_BUSY;
	static int8_t sStatus = 0;

	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegCommand, RC663_Idle);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_WriteReg, rRegFIFOControl, 0xB0);
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_WriteReg, rRegFIFOData, 0x0);
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_WriteReg, rRegFIFOData, 0x0);
	EXECUTE_FUNCTION(4, u8FuncIndex, 5, CLRC663_WriteReg, rRegCommand, RC663_LoadProtocol);
	EXECUTE_FUNCTION(5, u8FuncIndex, 6, CLRC663_WriteReg, rRegFIFOControl, 0xB0);
	
	EXECUTE_FUNCTION(6, u8FuncIndex, 7, CLRC663_WriteReg, rRegDrvMod, 0x8E);
	EXECUTE_FUNCTION(7, u8FuncIndex, 8, CLRC663_WriteReg, rRegIRQ0, 0x7F);
	EXECUTE_FUNCTION(8, u8FuncIndex, 9, CLRC663_WriteReg, rRegTxCrcPreset, 0x18);
	EXECUTE_FUNCTION(9, u8FuncIndex, 10, CLRC663_WriteReg, rRegRxCrcPreset, 0x18);
	EXECUTE_FUNCTION(10, u8FuncIndex, 11, CLRC663_WriteReg, rRegTxDataNum, 0x0F);	
	EXECUTE_FUNCTION(11, u8FuncIndex, 12, CLRC663_WriteReg, rRegFIFOData, PICC_REQIDL);
	EXECUTE_FUNCTION(12, u8FuncIndex, 13, CLRC663_WriteReg, rRegCommand, RC663_Transceive);
	
	EXECUTE_FUNCTION(13, u8FuncIndex, 14, CLRC663_ReadReg, rRegIRQ0, &u8IRQ0Value);	
	EXECUTE_PROCESS_START(14, u8FuncIndex)
		if (u8IRQ0Value & 0x04)
		{
			u8FuncIndex = 15;
		}
		else
		{
			u8FuncIndex = 0;
			return STATE_IDLE;
		}
	EXECUTE_PROCESS_END
	
	EXECUTE_FUNCTION(15, u8FuncIndex, 16, CLRC663_ReadReg, rRegFIFOData, &u8Tag[0]);	
	EXECUTE_FUNCTION(16, u8FuncIndex, 17, CLRC663_ReadReg, rRegFIFOData, &u8Tag[1]);	

		
	u8FuncIndex = 0;		
	return STATE_FINISH;
}


ProcessState MifareClassicGetUID(uint8_t *pTag)
{
	static uint8_t u8FuncIndex = 0;	
	static CommandStruct sCommand;
	ProcessState eState = STATE_BUSY;
	static int8_t sStatus = 0;
	
	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_WriteReg, rRegCommand, RC663_Idle);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_WriteReg, rRegFIFOControl, 0xB0);
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_WriteReg, rRegIRQ0, 0x7F);
	EXECUTE_FUNCTION(3, u8FuncIndex, 4, CLRC663_WriteReg, rRegTxCrcPreset, 0x18);
	EXECUTE_FUNCTION(4, u8FuncIndex, 5, CLRC663_WriteReg, rRegRxCrcPreset, 0x18);
	
	
	u8FuncIndex = 0;		
	return STATE_FINISH;
}


ProcessState CLRC663_LoadKey(uint8_t *pKey, int8_t *pStatus)
{
	static CommandStruct sCommand;
	static int8_t s8Status = 0;
	ProcessState eState = STATE_BUSY;
	static uint8_t u8FuncIndex = 0;	
	
	EXECUTE_PROCESS_START(0, u8FuncIndex)
		sCommand.u8Command = RC663_LoadKey;
		sCommand.u8Length = 6;
		memcpy(sCommand.u8Data, pKey, 6);
		u8FuncIndex = 1;
	EXECUTE_PROCESS_END

	EXECUTE_PROCESS_START(1, u8FuncIndex)
		eState = RC663_Command_Int(&sCommand, pStatus);
		if (STATE_FINISH == eState)
		{
			u8FuncIndex = 0;
		}
	EXECUTE_PROCESS_END
	
	return eState;
}


ProcessState CLRC663_CMD_MfcAuthenticate(uint8_t u8AuthMode, uint8_t u8Block, uint8_t *pSnr, int8_t *pStatus)
{
	static CommandStruct sCommand;
	static int8_t s8Status = -1;
	static uint8_t u8FuncIndex = 0;	
	ProcessState eState = STATE_BUSY;
	static uint8_t u8RegValue = 0;
	static uint8_t u8RetryCount = 0;
	
	EXECUTE_PROCESS_START(0, u8FuncIndex)
		sCommand.u8Command = RC663_MFAuthent;
		sCommand.u8Length = 6;
		sCommand.u8Data[0] = u8AuthMode;
		sCommand.u8Data[1] = u8Block;
		memcpy(&sCommand.u8Data[2], pSnr, 4);
		u8FuncIndex = 1;
	EXECUTE_PROCESS_END

	EXECUTE_PROCESS_START(1, u8FuncIndex)
		if (STATE_FINISH == RC663_Command_Int(&sCommand, &s8Status))
		{
			if (MI_OK == s8Status)
			{
				u8RetryCount = 0;
				u8FuncIndex = 2;
			}
		}
		else
		{
			return STATE_BUSY;
		}
	EXECUTE_PROCESS_END
		
	EXECUTE_FUNCTION(2, u8FuncIndex, 3, CLRC663_ReadReg, rRegStatus, &u8RegValue);	
		
	EXECUTE_PROCESS_START(3, u8FuncIndex)
		if (!(u8RegValue & 0x20))
		{
			if (u8RetryCount++ > 3)
			{
				*pStatus = MI_AUTHERR;
				u8FuncIndex++;
			}
			else
			{
				u8FuncIndex = 2;
				return STATE_BUSY;
			}
		}
		else
		{
			*pStatus = MI_OK;	
			u8FuncIndex++;			
		}
	EXECUTE_PROCESS_END
	
	u8FuncIndex = 0;

	return STATE_FINISH;
}


ProcessState GetDeviceVersion(void)
{
	uint8_t u8RegValue = 0;
	ProcessState eState = CLRC663_ReadReg(rRegVersion, &u8RegValue);
	if (eState == STATE_SUCCESS)
	{
		g_sRC663_Manager.u8Version = u8RegValue;
	}
	
	return eState;
}



ProcessState CLRC663_PcdRead(uint8_t u8Addr, uint8_t *pReadData, int8_t *pStatus)
{
	static CommandStruct sCommand;
	ProcessState eState = STATE_BUSY;
	static uint8_t u8FuncIndex = 0;	
	static int8_t s8Status = -1;
	static uint8_t u8RetryCount = 0;
	
	EXECUTE_FUNCTION(0, u8FuncIndex, 1, CLRC663_SetRawRC, rRegTxCrcPreset, 0xfe, 0x01);
	EXECUTE_FUNCTION(1, u8FuncIndex, 2, CLRC663_SetRawRC, rRegRxCrcPreset, 0xfe, 0x00);
		
	EXECUTE_PROCESS_START(2, u8FuncIndex)
		sCommand.u8Command = RC663_Transceive;
		sCommand.u8Length  = 2;
		sCommand.u8Data[0] = PICC_READ;
		sCommand.u8Data[1] = u8Addr;
		u8FuncIndex = 3;
	EXECUTE_PROCESS_END	
	
	EXECUTE_PROCESS_START(3, u8FuncIndex)
		eState = CLRC663_PcdComTransceive(&sCommand, &s8Status);
		if (STATE_FINISH == eState)
		{
			*pStatus = s8Status;
			//printf("read length:%d\r\n", sCommand.u8Length);
			if (s8Status == MI_OK)
			{
				if (sCommand.u8Length != 0x90)
				{
					*pStatus = MI_BITCOUNTERR;
				}
				else
				{
					memcpy(pReadData, &sCommand.u8Data[0], 16);
				}
			}
//			else
//			{
//				if (u8RetryCount++ < 3)
//				{
//					u8FuncIndex = 0;
//					return STATE_BUSY;
//				}
//			}
			
			u8FuncIndex = 4;
		}
		else
		{
			return eState;
		}
	EXECUTE_PROCESS_END	
	
	u8RetryCount = 0;
	u8FuncIndex = 0;
	s8Status = 0;
	return STATE_FINISH;
}

ProcessState CLRC663_PcdWrite(uint8_t u8Addr, uint8_t *pWriteData, int8_t *pStatus)
{
	static CommandStruct sCommand;
	ProcessState eState = STATE_BUSY;
	static int8_t s8Status = -1;
	static uint8_t u8FuncIndex = 0;
	
	EXECUTE_PROCESS_START(0, u8FuncIndex)
		sCommand.u8Command = RC663_Transceive;
		sCommand.u8Length  = 2;
		sCommand.u8Data[0] = PICC_WRITE;
		sCommand.u8Data[1] = u8Addr;
		u8FuncIndex = 1;
	EXECUTE_PROCESS_END	

	EXECUTE_PROCESS_START(1, u8FuncIndex)
		eState = CLRC663_PcdComTransceive(&sCommand, &s8Status);
		if (STATE_FINISH == eState)
		{
			u8FuncIndex = 2;
		}
		else
		{
			return eState;
		}
	EXECUTE_PROCESS_END	
	
	EXECUTE_PROCESS_START(2, u8FuncIndex)
		//printf("write length:%d,status:%d\r\n", sCommand.u8Length, s8Status);
		if (s8Status != MI_NOTAGERR)
		{
			
				if(sCommand.u8Length != 4)
				{					
					*pStatus = MI_BITCOUNTERR;
				}
				else
				{
					 sCommand.u8Data[0] &= 0x0F;
					 switch (sCommand.u8Data[0])
					 {
							case 0x00:
								 *pStatus = MI_NOTAUTHERR;
								 break;
							case 0x0A:
								 *pStatus = MI_OK;
								 break;
							default:
								 *pStatus = MI_CODEERR;
								 break;
					 }
				}
		}   
		
		if (s8Status == MI_OK)
		{
			u8FuncIndex = 3;
		}
		else
		{
			u8FuncIndex = 6;
		}
	EXECUTE_PROCESS_END
	
	EXECUTE_PROCESS_START(3, u8FuncIndex)
		sCommand.u8Command = RC663_Transceive;
		sCommand.u8Length  = 16;
		memcpy(sCommand.u8Data, pWriteData, 16);
		u8FuncIndex = 4;
	EXECUTE_PROCESS_END
		
	EXECUTE_PROCESS_START(4, u8FuncIndex)
		eState = CLRC663_PcdComTransceive(&sCommand, &s8Status);
		if (STATE_FINISH == eState)
		{
			if (s8Status != MI_NOTAGERR)
			{
					sCommand.u8Data[0] &= 0x0F;
					switch(sCommand.u8Data[0])
					{
						 case 0x00:
								*pStatus = MI_WRITEERR;
								break;
						 case 0x0A:
								*pStatus = MI_OK;
								break;
						 default:
								*pStatus = MI_CODEERR;
								break;
				 }
			}
		}
		else
		{
			return STATE_BUSY;
		}
	EXECUTE_PROCESS_END
	
	u8FuncIndex = 0;
	return STATE_FINISH;
}

uint8_t g_u8WriteAddr = 2;
uint8_t g_u8WriteData[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
bool g_bWriteSuccess = false;
bool g_bWriteFlag = false;
void WriteData(const uint8_t u8Address)
{
	static int8_t s8Status = 0;
	
	g_bWriteSuccess = false;
	if (STATE_FINISH == CLRC663_PcdWrite(u8Address, g_u8WriteData, &s8Status))
	{
		printf("write status:%d\r\n", s8Status);
		if (MI_OK == s8Status)
		{
			g_bWriteSuccess = true;
		}
		
		g_bWriteFlag = false;
	}
}

uint8_t g_u8ReadAddr = 2;
uint8_t g_u8ReadData[16] = {0};
bool g_bReadSuccess = false;
bool g_bReadFlag = false;
void ReadData(const uint8_t u8Address)
{
	static int8_t s8Status = 0;
	
	g_bReadSuccess = false;
	if (STATE_FINISH == CLRC663_PcdRead(u8Address, g_u8ReadData, &s8Status))
	{
		//printf("read status:%d\r\n", s8Status);
		if (MI_OK == s8Status)
		{
			g_bReadSuccess = true;
		}
		
		g_bReadFlag = false;
	}
}


static ExecuteState MifareDetectCard(void)
{
	ExecuteState eState = RESULT_BUSY;
	static int8_t s8Status = MI_ERR;
	static uint8_t u8Step = 0;
	static uint8_t u8RetryTimes = 0;
	ProcessState eProcessState = STATE_IDLE;
	
	switch (u8Step)
	{
		case 0:	
			eProcessState = GetDeviceVersion();
			if (STATE_SUCCESS == eProcessState)
			{
				memset(g_sRC663_Manager.sDataHandler.u8M1_Data, 0, 
						sizeof(g_sRC663_Manager.sDataHandler.u8M1_Data));
				memset(g_sRC663_Manager.sDataHandler.u8Key, 0xFF, 
						sizeof(g_sRC663_Manager.sDataHandler.u8Key));
				u8Step = 5;
			}
			break;
		
		case 5:
			if (STATE_SUCCESS == MifareClassicConfig())
			{
				//ResetTimerCount(&u32TimeoutCount);
				u8Step = 10;
			}
			break;
		
		case 10:
			if (STATE_FINISH == CLRC663_PcdRequestA(PICC_REQALL, 
								g_sRC663_Manager.sDataHandler.u8M1_Data, &s8Status))
			{
				if (MI_OK == s8Status)
				{
					g_sRC663_Manager.u16Tag = (g_sRC663_Manager.sDataHandler.u8M1_Data[0] << 8) | 
												g_sRC663_Manager.sDataHandler.u8M1_Data[1];
					if (0x0400 == g_sRC663_Manager.u16Tag)
					{
						u8Step = 15;
					}
					else
					{
						DEBUG_INFO("mifare requestA error\r\n");
						eState = RESULT_ERROR;
					}
				}
				else
				{
					eState = RESULT_ERROR;
				}
			}
			break;
			
		case 15:
			if (STATE_FINISH == CLRC663_PcdAnticoll(&g_sRC663_Manager.sDataHandler.u8M1_Data[2], 
													&s8Status))
			{
				if (MI_OK == s8Status)
				{
					g_sRC663_Manager.u32UID = (g_sRC663_Manager.sDataHandler.u8M1_Data[2] << 24) | 
											(g_sRC663_Manager.sDataHandler.u8M1_Data[3] << 16) | 
											(g_sRC663_Manager.sDataHandler.u8M1_Data[4] << 8) | 
											g_sRC663_Manager.sDataHandler.u8M1_Data[5];
					u8Step = 20;
				}
				else
				{
					DEBUG_INFO("mifare anticoll error\r\n");
					eState = RESULT_ERROR;
				}
			}
			break;
			
		case 20:
			if (STATE_FINISH == CLRC663_PcdSelect(&g_sRC663_Manager.sDataHandler.u8M1_Data[2], 
									&g_sRC663_Manager.sDataHandler.u8M1_Data[6], &s8Status))
			{
				if (MI_OK == s8Status)
				{
					// 获取IC卡的SAK
					g_sRC663_Manager.u8SAK = g_sRC663_Manager.sDataHandler.u8M1_Data[6];
					if (0x08 == g_sRC663_Manager.u8SAK)
					{
						s8Status = MI_ERR;
						u8Step = 25;
					}
					else
					{
						DEBUG_INFO("mifare sak error\r\n");
						eState = RESULT_ERROR;
					}
				}
				else
				{
					DEBUG_INFO("mifare select error\r\n");
					eState = RESULT_ERROR;
				}
			}
			break;
			
		case 25:
			if (STATE_FINISH == CLRC663_LoadKey(g_sRC663_Manager.sDataHandler.u8Key, &s8Status))
			{
				if (MI_OK == s8Status)
				{
					eState = RESULT_SUCCESS;
					u8Step = 0;
				}
				else
				{
					DEBUG_INFO("mifare load key error\r\n");
					eState = RESULT_ERROR;
				}
			}
			break;
	}
	
	if (STATE_FAILED == eProcessState)
	{
		DEBUG_INFO("mifare process error\r\n");
		eState = RESULT_ERROR;
	}
	else if (STATE_TIMEOUT == eProcessState)
	{
		DEBUG_INFO("mifare respond timeout\r\n");
		eState = RESULT_NO_RESPOND;
	}
	
	
	if (eState == RESULT_ERROR || eState == RESULT_NO_RESPOND)
	{
		u8Step = 0;
	}
	
	return eState;
}


const uint8_t g_u8RW_Address[] = 
{
	   1, 2,
	4, 5, 6,
	8, 9, 10,
	12, 13, 14,
	16, 17, 18,
	20, 21, 22,
	24, 25, 26,
	28, 29, 30,
	32, 33, 34,
	36, 37, 38,
	40, 41, 42,
	44, 45, 46,
	48, 49, 50,
	52, 53, 54,
	56, 57, 58,
	60, 61, 62,
};



bool MifareReadData(void)
{
	static uint8_t u8Step = 0;
	static int8_t s8Status = MI_ERR;
	uint8_t u8Message[8] = {0};
	CAN_TxHeaderTypeDef sTxHeader;
	ExecuteState eState = RESULT_BUSY;
	static uint8_t u8FrameCount = 0;
	static uint8_t u8BlockData[16] = {0};
	static uint8_t u8BlockIndex = 1;
	static uint32_t u32SendTimer = 0;
	uint16_t u16CheckSum = 0;
	bool bRet = false;
	static uint32_t u32TimeTest = 0;
	static uint32_t u32TimeStepTest = 0;
	static uint16_t u16UsTimeCount = 0;
	
	switch (u8Step)
	{
		case 0:
//			if (g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum > sizeof(g_u8RW_Address) * 16)
//			{
//				u8Message[0] = 0x4;
//				sTxHeader.StdId = 0x50;
//				sTxHeader.IDE = CAN_ID_STD;
//				sTxHeader.RTR = CAN_RTR_DATA;
//				sTxHeader.DLC = 1;
//				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
//				bRet = true;
//			}
//			else
//			{
//				ResetTimerCount(&u32TimeTest);
//				u8Step = 5;
//			}
			ResetTimerCount(&u32TimeTest);
			u8Step = 5;
			break;
		
		case 5:
			eState = MifareDetectCard();
			if (RESULT_SUCCESS == eState)
			{
				g_sRC663_Manager.sSendRecvHandler.u8GroupIndex = 0;
				u8BlockIndex = 0;
				g_sRC663_Manager.sDataHandler.u8Block = g_u8RW_Address[u8BlockIndex];
				u8Step = 10;
			}
			else if (eState == RESULT_ERROR || eState == RESULT_NO_RESPOND)
			{
				u8Message[0] = 0x3;
				if (eState == RESULT_NO_RESPOND)
				{
					u8Message[0] = 0x2;
				}
				sTxHeader.StdId = 0x50;
				sTxHeader.IDE = CAN_ID_STD;
				sTxHeader.RTR = CAN_RTR_DATA;
				sTxHeader.DLC = 1;
				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				u8Step = 0;
				bRet = true;
			}
			break;
			
		case 10:			
			g_sRC663_Manager.sDataHandler.u8Block = g_u8RW_Address[u8BlockIndex];
			u8Step = 15;
			break;
		
		case 15:
			if (STATE_FINISH == CLRC663_CMD_MfcAuthenticate(0x60, g_sRC663_Manager.sDataHandler.u8Block, 
								&g_sRC663_Manager.sDataHandler.u8M1_Data[2], &s8Status))
			{
				if (MI_OK == s8Status)
				{
					ResetTimerCount(&u32TimeStepTest);
					u8Step = 16;
					ResetTimerCountUs(&u16UsTimeCount);
				}
				else
				{
					// 读数据错误
					u8Message[0] = 0x3;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			break;
			
		case 16:
			if (GetTimerTickDeltaUs(u16UsTimeCount, GetCurTimerCountUs()) >= 1500)
			{
				u8Step = 20;
			}
			break;
		
		case 20:
			if (STATE_FINISH == CLRC663_PcdRead(g_sRC663_Manager.sDataHandler.u8Block, 
												u8BlockData, &s8Status))
			{
				if (MI_OK == s8Status)
				{	
					g_sRC663_Manager.sSendRecvHandler.u32TimeStepUse = 
						GetTimerTickDelta(u32TimeStepTest, GetCurTimerCount());
					u8Step = 25;				
				}
				else
				{
					// 读数据错误
					u8Message[0] = 0x3;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			break;
			
		case 25:
			if (0 == u8BlockIndex)
			{
				g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum = ((uint16_t)u8BlockData[0] << 8) | 
																	u8BlockData[1];
				if (g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum > (sizeof(g_u8RW_Address) - 2) * 16 || 
										0 == g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum)
				{
					u8Message[0] = 0x4;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
				else
				{
					u8BlockIndex++;
					u8Step = 26;
				}
			}
			else if (1 == u8BlockIndex)
			{
				u8Step = 26;
				u8BlockIndex++;
			}
			else
			{
				memcpy(g_sRC663_Manager.sSendRecvHandler.u8Data[g_sRC663_Manager.sSendRecvHandler.u8GroupIndex],
						&u8BlockData[0], 8);
				g_sRC663_Manager.sSendRecvHandler.u8GroupIndex++;
				memcpy(g_sRC663_Manager.sSendRecvHandler.u8Data[g_sRC663_Manager.sSendRecvHandler.u8GroupIndex],
						&u8BlockData[8], 8);
				g_sRC663_Manager.sSendRecvHandler.u8GroupIndex++;
			
//				if ((uint16_t)g_sRC663_Manager.sSendRecvHandler.u8GroupIndex * 8 >= 
//						((g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum / 16) * 16 + 
//					(g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum % 16) ? 16 : 0))
				if (u8BlockIndex >=
						(g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum / 16 + 1))
				{
					u8Step = 30;
				}
				else
				{
					u8BlockIndex++;
					ResetTimerCountUs(&u16UsTimeCount);
					u8Step = 26;
				}
			}
			break;
			
		case 26:
			if (GetTimerTickDeltaUs(u16UsTimeCount, GetCurTimerCountUs()) >= 1500)
			{
				u8Step = 10;
			}
			break;
			
		case 30:
			u16CheckSum = 0;
			for (uint8_t i = 0; i < g_sRC663_Manager.sSendRecvHandler.u8GroupIndex; i++)
			{
				for (uint8_t j = 0; j < 8; j++)
				{
					u16CheckSum = u16CheckSum + 
					g_sRC663_Manager.sSendRecvHandler.u8Data[i][j];
				}
			}
			
			u16CheckSum = ~u16CheckSum + 1;
			
			// 发送数据头
			u8Message[0] = 0x1;
			u8Message[1] = g_sRC663_Manager.sSendRecvHandler.u8GroupIndex;
			u8Message[2] = (u16CheckSum >> 8) & 0xFF;
			u8Message[3] = u16CheckSum & 0xFF;
			u8Message[4] = g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum >> 8;
			u8Message[5] = g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum & 0xFF;
			sTxHeader.StdId = 0x50;
			sTxHeader.IDE = CAN_ID_STD;
			sTxHeader.RTR = CAN_RTR_DATA;
			sTxHeader.DLC = 6;
			SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
			u8Step = 35;
			ResetTimerCount(&u32SendTimer);
			u8FrameCount = 0;
			break;
			
		case 35:
			if (GetTimerTickDelta(u32SendTimer, GetCurTimerCount()) >= 8)
			{
				ResetTimerCount(&u32SendTimer);
				memcpy(u8Message, g_sRC663_Manager.sSendRecvHandler.u8Data[u8FrameCount], 8);
				sTxHeader.StdId = 0x51;
				sTxHeader.IDE = CAN_ID_STD;
				sTxHeader.RTR = CAN_RTR_DATA;
				sTxHeader.DLC = 8;
				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				u8FrameCount++;
				
				if (u8FrameCount > g_sRC663_Manager.sSendRecvHandler.u8GroupIndex)
				{
					u8Step = 0;
					bRet = true;
					g_sRC663_Manager.sSendRecvHandler.u32TimeUseTest = 
						GetTimerTickDelta(u32TimeTest, GetCurTimerCount());
				}
			}
			break;
	}
	
	return bRet;
}



bool MifareWriteData(void)
{
	static uint8_t u8Step = 0;
	static int8_t s8Status = MI_ERR;
	uint8_t u8Message[8] = {0};
	CAN_TxHeaderTypeDef sTxHeader;
	ExecuteState eState = RESULT_BUSY;
	static uint8_t u8FrameCount = 0;
	static uint8_t u8WriteBlockData[16] = {0};
	static uint8_t u8ReadBlockData[16] = {0};
	static uint8_t u8WriteBlockIndex = 1;
	uint16_t u16CheckSum = 0;
	static uint16_t u16UsTimeCount = 0;
	bool bRet = false;
	
	switch (u8Step)
	{
		case 0:
			if ((g_sRC663_Manager.sSendRecvHandler.u8WriteFrameNum * 8) > 
				((sizeof(g_u8RW_Address) - 2) * 16))
			{
				// 数据量过多
				u8Message[0] = 0x14;
				sTxHeader.StdId = 0x50;
				sTxHeader.IDE = CAN_ID_STD;
				sTxHeader.RTR = CAN_RTR_DATA;
				sTxHeader.DLC = 1;
				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				u8Step = 0;
				bRet = true;
			}
			else
			{
				u8Step = 5;
				ResetTimerCount(&g_sRC663_Manager.sSendRecvHandler.u32RecvTimer);
			}
			break;
		
		case 5:
			if ((g_sRC663_Manager.sSendRecvHandler.u8GroupIndex - 4) >= 
				g_sRC663_Manager.sSendRecvHandler.u8WriteFrameNum)
			{
				// 接收完成				
				u16CheckSum = 0;
				for (uint8_t i = 4; i < g_sRC663_Manager.sSendRecvHandler.u8GroupIndex; i++)
				{
					for (uint8_t j = 0; j < 8; j++)
					{
						u16CheckSum = u16CheckSum + 
						g_sRC663_Manager.sSendRecvHandler.u8Data[i][j];
					}
				}
			
				u16CheckSum = ~u16CheckSum + 1;
				
				// 对比校验值
				if (g_sRC663_Manager.sSendRecvHandler.u16WriteCheckSum == u16CheckSum)
				{
					u8Step = 10;					
				}
				else
				{
					// 校验失败
					u8Message[0] = 0x12;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			else
			{
				if (GetTimerTickDelta(g_sRC663_Manager.sSendRecvHandler.u32RecvTimer, GetCurTimerCount()) >= 30000)
				{
					// 超时未接收足够数据
					u8Message[0] = 0x13;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			break;
		
		case 10:
			eState = MifareDetectCard();
			if (RESULT_SUCCESS == eState)
			{
				u8WriteBlockIndex = 0;
				g_sRC663_Manager.sDataHandler.u8Block = g_u8RW_Address[u8WriteBlockIndex];
				u8Step = 15;
			}
			else if (eState == RESULT_ERROR || eState == RESULT_NO_RESPOND)
			{
				u8Message[0] = 0x3;
				if (eState == RESULT_NO_RESPOND)
				{
					u8Message[0] = 0x2;
				}
				sTxHeader.StdId = 0x50;
				sTxHeader.IDE = CAN_ID_STD;
				sTxHeader.RTR = CAN_RTR_DATA;
				sTxHeader.DLC = 1;
				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				u8Step = 0;
				bRet = true;
			}
			break;
			
						
		case 15:			
			g_sRC663_Manager.sDataHandler.u8Block = g_u8RW_Address[u8WriteBlockIndex];
			u8Step = 20;
			break;
		
		case 20:
			if (STATE_FINISH == CLRC663_CMD_MfcAuthenticate(0x60, g_sRC663_Manager.sDataHandler.u8Block, 
								&g_sRC663_Manager.sDataHandler.u8M1_Data[2], &s8Status))
			{
				if (MI_OK == s8Status)
				{
					ResetTimerCountUs(&u16UsTimeCount);
					u8Step = 21;
				}
				else
				{
					// 读数据错误
					u8Message[0] = 0x3;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			break;
			
		case 21:
			if (GetTimerTickDeltaUs(u16UsTimeCount, GetCurTimerCountUs()) >= 1500)
			{
				u8Step = 22;
			}
			break;
			
		case 22:
			if (STATE_FINISH == CLRC663_PcdRead(g_sRC663_Manager.sDataHandler.u8Block, 
												u8ReadBlockData, &s8Status))
			{
				if (MI_OK == s8Status)
				{
					ResetTimerCountUs(&u16UsTimeCount);
					u8Step = 23;				
				}
				else
				{
					// 读数据错误
					u8Message[0] = 0x3;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			break;
			
			
		case 23:
			if (GetTimerTickDeltaUs(u16UsTimeCount, GetCurTimerCountUs()) >= 1500)
			{
				u8Step = 25;
			}
			break;
			
		case 25:
			// 写数据
			memcpy(&u8WriteBlockData[0], g_sRC663_Manager.sSendRecvHandler.u8Data[u8WriteBlockIndex * 2], 8);
			memcpy(&u8WriteBlockData[8], g_sRC663_Manager.sSendRecvHandler.u8Data[u8WriteBlockIndex * 2 + 1], 8);
			if (STATE_FINISH == CLRC663_PcdWrite(g_sRC663_Manager.sDataHandler.u8Block, 
												u8WriteBlockData, &s8Status))
			{
				if (MI_OK == s8Status)
				{
					ResetTimerCountUs(&u16UsTimeCount);
					u8Step = 26;
				}
				else
				{
					// 写数据错误
					u8Message[0] = 0x11;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			break;
			
		case 26:
			if (GetTimerTickDeltaUs(u16UsTimeCount, GetCurTimerCountUs()) >= 1500)
			{
				u8Step = 30;
			}
			break;
		
		case 30:
			// 将写入的数据读出来
			if (STATE_FINISH == CLRC663_PcdRead(g_sRC663_Manager.sDataHandler.u8Block, 
												u8ReadBlockData, &s8Status))
			{
				if (MI_OK == s8Status)
				{
					u8Step = 35;				
				}
				else
				{
					// 读数据错误
					u8Message[0] = 0x3;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
			}
			break;
			
		case 35:
			// 校验数据
			if (0 == memcmp(u8ReadBlockData, u8WriteBlockData, 16))
			{
				u8WriteBlockIndex++;
				if (u8WriteBlockIndex > (g_sRC663_Manager.sSendRecvHandler.u8WriteFrameNum / 2) + 1)
				{
					// 所有数据写入成功
					u8Message[0] = 0x10;
					sTxHeader.StdId = 0x50;
					sTxHeader.IDE = CAN_ID_STD;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 1;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					u8Step = 0;
					bRet = true;
				}
				else
				{
					ResetTimerCountUs(&u16UsTimeCount);
					u8Step = 36;
				}
			}
			else
			{
				// 写数据错误
				u8Message[0] = 0x11;
				sTxHeader.StdId = 0x50;
				sTxHeader.IDE = CAN_ID_STD;
				sTxHeader.RTR = CAN_RTR_DATA;
				sTxHeader.DLC = 1;
				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				u8Step = 0;
				bRet = true;
			}
			break;
			
		case 36:
			if (GetTimerTickDeltaUs(u16UsTimeCount, GetCurTimerCountUs()) >= 1500)
			{
				u8Step = 15;
			}
			break;
	}
	
	return bRet;
}



void cbRfidHandleRecvProcess(CAN_RxHeaderTypeDef *pRxHeader, uint8_t *pMessage)
{
	if (CAN_ID_STD == pRxHeader->IDE)
	{
		switch (pRxHeader->StdId)
		{
			case 0x40:
				if (1 == pRxHeader->DLC && (1 == pMessage[0]))
				{
					if (EVENT_NONE == g_sRC663_Manager.eEvent)
					{
//						g_sRC663_Manager.sSendRecvHandler.u16ReadByteNum = 
//							((uint16_t)pMessage[1] << 8) | pMessage[2];					
						g_sRC663_Manager.eEvent = EVENT_READ_DATA;
					}
				}
				else if (6 == pRxHeader->DLC && (2 == pMessage[0]))
				{
					if (EVENT_NONE == g_sRC663_Manager.eEvent)
					{	
						ResetTimerCount(&g_sRC663_Manager.sSendRecvHandler.u32RecvTimer);
						g_sRC663_Manager.sSendRecvHandler.u8WriteFrameNum = pMessage[1];
						g_sRC663_Manager.sSendRecvHandler.u16WriteCheckSum = ((uint16_t)pMessage[2] << 8) | pMessage[3];
						g_sRC663_Manager.sSendRecvHandler.u16WriteByteNum = ((uint16_t)pMessage[4] << 8) | pMessage[5];
						memset(g_sRC663_Manager.sSendRecvHandler.u8Data[0], 0, 8);
						memset(g_sRC663_Manager.sSendRecvHandler.u8Data[1], 0, 8);
						memset(g_sRC663_Manager.sSendRecvHandler.u8Data[2], 0, 8);
						memset(g_sRC663_Manager.sSendRecvHandler.u8Data[3], 0, 8);
						g_sRC663_Manager.sSendRecvHandler.u8Data[0][0] = pMessage[4];
						g_sRC663_Manager.sSendRecvHandler.u8Data[0][1] = pMessage[5];
						g_sRC663_Manager.sSendRecvHandler.u8GroupIndex = 4;						
						g_sRC663_Manager.eEvent = EVENT_WRITE_DATA;
					}
				}
				break;
				
			case 0x41:
				if (EVENT_WRITE_DATA == g_sRC663_Manager.eEvent)
				{
					ResetTimerCount(&g_sRC663_Manager.sSendRecvHandler.u32RecvTimer);
					memcpy(g_sRC663_Manager.sSendRecvHandler.u8Data[g_sRC663_Manager.sSendRecvHandler.u8GroupIndex],
							pMessage, 8);
					g_sRC663_Manager.sSendRecvHandler.u8GroupIndex++;
				}
				break;
		}
	}
}


void MifareClassicInit(void)
{
	RegisterRecvProcess(cbRfidHandleRecvProcess);
}


void MifareClassicProcess(void)
{
	if (EVENT_READ_DATA == g_sRC663_Manager.eEvent)
	{
		if (MifareReadData())
		{
			g_sRC663_Manager.eEvent = EVENT_NONE;
		}
	}
	else if (EVENT_WRITE_DATA == g_sRC663_Manager.eEvent)
	{
		if (MifareWriteData())
		{
			g_sRC663_Manager.eEvent = EVENT_NONE;
		}
	}
	
	
//	static uint32_t u32TestTime = 0;
//	
//	if (GetTimerTickDelta(u32TestTime, GetCurTimerCount()) >= 2000)
//	{
//		g_sRC663_Manager.eEvent = EVENT_READ_DATA;
//		ResetTimerCount(&u32TestTime);
//	}
}


//uint8_t g_u8Block = 0;
//uint8_t g_u8Auth = 0x60;
//void MifareClassicProcess(void)
//{
//	static uint8_t u8Step = 0;
//	static uint8_t u8M1_Data[7] = {0};
//	//static uint8_t RD_Data[16] = {0};
//	static int8_t s8Status = MI_ERR;
//	static uint32_t u32TimerCount = 0;
//	static uint8_t u8Key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//	ProcessState eState = STATE_IDLE;
//	
//	uint8_t u8Message[8] = {0};
//	CAN_TxHeaderTypeDef sTxHeader;
//	
//	switch (u8Step)
//	{
//		case 0:
//			//PDWON_LOW;
//			if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 2000)
//			{
//				u8Step = 5;
//			}
//			break;
//		
//		case 5:			
//			if (STATE_SUCCESS == GetDeviceVersion())
//			{
//				//if (g_sRC663_Manager.u8Version == 0x18)
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//		
//		case 10:
//			if (STATE_SUCCESS == MifareClassicConfig())
//			{
//				u8Step = 15;
//			}
//			break;
//		
//		case 15:
//			if (STATE_FINISH == CLRC663_PcdRequestA(PICC_REQALL, u8M1_Data, &s8Status))
//			{
//				if (MI_OK == s8Status)
//				{
//					u8Step = 10;
//					g_sRC663_Manager.u16Tag = (u8M1_Data[0] << 8) | u8M1_Data[1];
//					if (0x0400 == g_sRC663_Manager.u16Tag)
//					{
//						u8Step = 20;
//					}
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//			
//		case 20:
//			if (STATE_FINISH == CLRC663_PcdAnticoll(&u8M1_Data[2], &s8Status))
//			{
//				if (MI_OK == s8Status)
//				{
//					g_sRC663_Manager.u32UID = (u8M1_Data[2] << 24) | (u8M1_Data[3] << 16) | (u8M1_Data[4] << 8) | u8M1_Data[5];
//					u8Step = 25;
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//			
//		case 25:
//			if (STATE_FINISH == CLRC663_PcdSelect(&u8M1_Data[2], &u8M1_Data[6], &s8Status))
//			{
//				if (MI_OK == s8Status)
//				{
//					// 获取IC卡的SAK
//					g_sRC663_Manager.u8SAK = u8M1_Data[6];
//					if (0x08 == g_sRC663_Manager.u8SAK)
//					{
//						s8Status = MI_ERR;
//						u8Step = 30;
//					}
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//			
//		case 30:
//			if (STATE_FINISH == CLRC663_LoadKey(u8Key, &s8Status))
//			{
//				if (MI_OK == s8Status)
//				{
//					u8Step = 35;
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//			
//		case 35:
//			if (STATE_FINISH == CLRC663_CMD_MfcAuthenticate(g_u8Auth, g_u8Block, &u8M1_Data[2], &s8Status))
//			{
//				if (MI_OK == s8Status)
//				{
//					u8Step = 40;
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//			
//		case 40:	
//			if (STATE_FINISH == CLRC663_PcdRead(g_u8ReadAddr, g_u8ReadData, &s8Status))
//			{
//				//printf("read status:%d\r\n", s8Status);
//				if (MI_OK == s8Status)
//				{
//					u8Step = 45;
//					g_bReadSuccess = true;
//					
//					memcpy(g_u8WriteData, g_u8ReadData, 16);
//					for (uint8_t i = 0; i < 16; i++)
//					{
//						g_u8WriteData[i] += 2;
//					}
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//		
//			break;
//			
//			
//		case 45:
//			if (STATE_FINISH == CLRC663_PcdWrite(g_u8WriteAddr, g_u8WriteData, &s8Status))
//			{
//				//printf("write status:%d\r\n", s8Status);
//				if (MI_OK == s8Status)
//				{
//					u8Step = 50;
//					g_bWriteSuccess = true;
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//			
//			
//		case 50:
//			//HAL_Delay(10);
//			if (STATE_FINISH == CLRC663_PcdRead(g_u8ReadAddr, g_u8ReadData, &s8Status))
//			{
//				if (MI_OK == s8Status)
//				{
//					for (uint8_t i = 0; i < 16; i++)
//					{
//						printf("%d ", g_u8ReadData[i]);
//					}
//					printf("\r\n");
//					
//					memcpy(u8Message, g_u8ReadData, 8);
//					sTxHeader.StdId = 0x40;
//					sTxHeader.IDE = CAN_ID_STD;
//					sTxHeader.RTR = CAN_RTR_DATA;
//					sTxHeader.DLC = 8;
//					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
//					
//					
//					u8Step = 55;
//					g_bReadSuccess = true;
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//			
//		case 55:
//			if (STATE_FINISH == CLRC663_FieldOff())
//			{
//				//HAL_Delay(10);
//				ResetTimerCount(&u32TimerCount);
//				u8Step = 60;
//			}
//			break;
//			
//		case 60:
//			if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 10)
//			{
//				u8Step = 65;
//			}
//			break;
//			
//		case 65:
//			if (STATE_FINISH == CLRC663_FieldOn())
//			{
//				ResetTimerCount(&u32TimerCount);
//				u8Step = 70;
//			}
//			break;
//			
//		case 70:
//			if (GetTimerTickDelta(u32TimerCount, GetCurTimerCount()) > 10)
//			{
//				u8Step = 75;
//			}
//			break;
//			
//		case 75:
//			if (STATE_FINISH == CLRC663_PcdRequestA(PICC_REQALL, u8M1_Data, &s8Status))
//			{
//				if (MI_OK == s8Status)
//				{
//					u8Step = 55;
//				}
//				else
//				{
//					u8Step = 10;
//				}
//			}
//			break;
//	}
//}

