#include "CAN_IAP.h"
#include <string.h>
#include "Timer.h"
#include "stm32f4xx_hal_flash.h"
#include "CAN_Process.h"
#include "usart.h"
#include "flash_if.h"






// IAP版本
#define IAP_MAJOR_VERSION	1
#define IAP_MINOR_VERSION	0
#define IAP_PATCH_VERSION	0


/* 
	硬件索引
	电源板：0x1
	手柄转接板：0x2
	手柄：0x3
	抱闸控制板：0x4
	内镜检测板：0x5
*/
#define HARDWARE_INDEX		0x1



#define DATA_RECV_TIMEOUT	5000
#define FORCE_BOOT_TIMEOUT	5000
#define RETRY_MAX_TIMES		3

#define UPPER_COMPUTER_SCAN				0x18000000	//上位机扫描
#define SCAN_ALL						0x0
#define SCAN_ME							HARDWARE_INDEX
#define FORCE_BOOT						0x1
#define UPDATE_REQUEST_BASE				0x18100000	//上位机更新固件请求	
#define UPDATE_REQUEST					(UPDATE_REQUEST_BASE + HARDWARE_INDEX)	
#define PACKAGE_INFO_BASE				0x18200000	//数据包信息
#define PACKAGE_INFO					(PACKAGE_INFO_BASE + HARDWARE_INDEX)	
#define PACKAGE_DATA_BASE				0x18300000	//数据包信息
#define PACKAGE_DATA					(PACKAGE_DATA_BASE + HARDWARE_INDEX)	


#define EEPROM_READ_REQUEST_BASE		0x18A00000
#define EEPROM_READ_REQUEST				(EEPROM_READ_REQUEST_BASE + HARDWARE_INDEX)	
#define EEPROM_WRITE_REQUEST_BASE		0x18B00000
#define EEPROM_WRITE_REQUEST			(EEPROM_WRITE_REQUEST_BASE + HARDWARE_INDEX)	



#define RESPOND_FIRMWARE_INFO_BASE			0x18FF0000	//下位机程序信息ID
#define RESPOND_UPDATE_BASE					0x18FF1000	// 更新固件应答
#define RESPOND_UPDATE						(RESPOND_UPDATE_BASE + HARDWARE_INDEX)	



#define RESPOND_EEPROM_READ_REQUEST			(0x18FFA000 + HARDWARE_INDEX)
#define RESPOND_EEPROM_WRITE_REQUEST		(0x18FFB000 + HARDWARE_INDEX)




#define	FIRMWARE_READY_RUN  	1		//应用程序就绪, 可以运行
#define	LAST_UPDATE_FAILED  	2		//上次更新失败
#define	WAIT_FIRMWARE_UPDATE  	3		//等待应用程序更新



typedef void (*pFunction)(void);

static void SetFirmwareInfo(const FirmwareType eType, const uint32_t u32FirmwareVersion);
void SetUpdateIndex(const uint8_t u8Index);
static IAP_State GetIAP_CurrentState(void);
static void SetIAP_Busy(void);
static void IAP_IdleProcess(void);
static void IAP_PreUpdateProcess(void);	


IAP_ProcessManagerStruct g_sIAP_ProcessManager = 
{
	SetFirmwareInfo, 
	SetUpdateIndex,
	GetIAP_CurrentState,
	SetIAP_Busy,
};


typedef enum
{
	IAP_EVENT_NONE = 0,
	IAP_EVENT_SCANNING,
	IAP_EVENT_UPDATE_REQUEST,
	IAP_EVENT_PKG_INFO,
	IAP_EVENT_PKG_FRAME,
	IAP_EVENT_ROLLBACK,
	IAP_EVENT_READ_EEPROM,
	IAP_EVENT_WRITE_EEPROM,
}IAP_EventEnum;


struct
{
	FirmwareType eFirmwareType;
	uint32_t u32FirmwareVersion;
	IAP_State eCurrentState;
	void (*pFunc)(void);
	IAP_EventEnum eEvent;
	uint32_t u32TimerCount;
	uint32_t u32TimeoutCount;
	uint8_t u8UpdateIndex;
	uint32_t u32ProgramAddr;
	uint32_t u32ProgramSectorNum;
	uint32_t u32ForceBootCount;
	bool bForceBootFlag;
	bool bBusy;
	struct
	{
		uint16_t u16PkgTotalNum;
		uint16_t u16PkgIndex;
		uint16_t u16FrameNum;
		uint16_t u16CheckSum;
	}sPkgInfo;
	struct
	{
		uint16_t u16ReadAddress;
		uint16_t u16WriteAddress;
		uint8_t u8Data[8];
		uint8_t u8DataLen;
	}sEEPROM_Info;
}g_sIAP_StateManager = 
{
	TYPE_BOOTLOADER,
	0,
	IAP_IDLE,
	IAP_IdleProcess,
	IAP_EVENT_NONE,
	0,
	DATA_RECV_TIMEOUT,
};

struct
{
	uint16_t u16PackageIndex;
	uint16_t u16FrameNum;
	uint16_t u16CheckSum;
	uint8_t u8ReadIndex;
	uint8_t u8WriteIndex;
	uint8_t u8Data[64][8];
}g_sDataManager = {0};


static void SetFirmwareInfo(const FirmwareType eType, const uint32_t u32FirmwareVersion)
{
	g_sIAP_StateManager.eFirmwareType = eType;
	g_sIAP_StateManager.u32FirmwareVersion = u32FirmwareVersion;
}


void SetUpdateIndex(const uint8_t u8Index)
{
	g_sIAP_StateManager.u8UpdateIndex = u8Index;
}


static IAP_State GetIAP_CurrentState(void)
{
	return g_sIAP_StateManager.eCurrentState;
}


static void SetIAP_Busy(void)
{
	g_sIAP_StateManager.bBusy = true;
}


static void ChangeProcessFunc(void (*pFunc)(void))
{
	g_sIAP_StateManager.pFunc = pFunc;
	ResetTimerCount(&g_sIAP_StateManager.u32TimerCount);
}


__weak FirmwareEvent cbRollbackCheck(void)
{
	return EVENT_NONE;
}


__weak FirmwareEvent cbUpdateSuccessProcess(void)
{
	return EVENT_NONE;
}


__weak void cbRollbackProcess(void)
{
	return;
}


__weak FirmwareEvent cbWriteEEPROM(const uint16_t u16Address, const uint8_t u8Data)
{
	return EVENT_NONE;
}



__weak FirmwareEvent cbReadEEPROM(const uint16_t u16Address, uint8_t *pData, const uint8_t u8DataLen)
{
	return EVENT_NONE;
}



static void IAP_UpdateProcess(void)
{
	uint8_t i = 0;
	uint8_t u8Message[8] = {0};
	CAN_TxHeaderTypeDef sTxHeader;
	g_sIAP_StateManager.eCurrentState = IAP_UPDATE;
	
	if (GetTimerTickDelta(g_sIAP_StateManager.u32TimerCount, GetCurTimerCount()) >= g_sIAP_StateManager.u32TimeoutCount)
	{
		// 超时处理
		DEBUG_INFO("recv data frame timeout\r\n");
		FLASH_If_Erase(g_sIAP_StateManager.u32ProgramAddr, g_sIAP_StateManager.u32ProgramSectorNum);
		ChangeProcessFunc(IAP_IdleProcess);
	}
	else
	{
		if (IAP_EVENT_PKG_FRAME == g_sIAP_StateManager.eEvent)
		{
			for (uint8_t i = 0; i < 8; i++)
			{
				g_sDataManager.u16CheckSum = g_sDataManager.u16CheckSum + 
								g_sDataManager.u8Data[g_sDataManager.u8ReadIndex][i];						
			}
			g_sDataManager.u8ReadIndex++;		
			
			// 如果当前固件数据帧计数等于数据包中的数据帧数量信息, 就进入校验和写入环节
			if (g_sDataManager.u16FrameNum == g_sIAP_StateManager.sPkgInfo.u16FrameNum)
			{			
				if (g_sDataManager.u8ReadIndex < g_sDataManager.u16FrameNum)
				{	
					g_sDataManager.u16CheckSum = 0;
					DEBUG_INFO("recalc checksum\r\n");					
					for (uint8_t i = 0; i < g_sDataManager.u16FrameNum; i++)
					{				
						for (uint8_t j = 0; j < 8; j++)
						{
							g_sDataManager.u16CheckSum = g_sDataManager.u16CheckSum + 
									g_sDataManager.u8Data[i][j];	
						}
					}
				}
				g_sDataManager.u16CheckSum = ~g_sDataManager.u16CheckSum + 1;

				if (g_sIAP_StateManager.sPkgInfo.u16CheckSum == g_sDataManager.u16CheckSum)
				{
					DEBUG_INFO("checksum correct\r\n");				
					
										
					// 将一个包的数据写入FLASH
					uint32_t u32Ret = FLASH_If_Write(g_sIAP_StateManager.u32ProgramAddr + 
										(g_sIAP_StateManager.sPkgInfo.u16PkgIndex - 1) * 512, 
										(uint32_t *)g_sDataManager.u8Data, 
										g_sIAP_StateManager.sPkgInfo.u16FrameNum * 8 / sizeof(uint32_t));
					if (u32Ret)
					{
						DEBUG_INFO("write flash failed:%d\r\n", u32Ret);
						FLASH_If_Erase(g_sIAP_StateManager.u32ProgramAddr, g_sIAP_StateManager.u32ProgramSectorNum);
						
						// FLASH 写入有问题
						u8Message[0] = 0x2;		
						u8Message[1] = 0x5;							
						sTxHeader.ExtId = RESPOND_UPDATE_BASE + HARDWARE_INDEX;
						sTxHeader.IDE = CAN_ID_EXT;
						sTxHeader.RTR = CAN_RTR_DATA;
						sTxHeader.DLC = 2;
						SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
						ChangeProcessFunc(IAP_IdleProcess);		
						return;
					}
					else
					{
						DEBUG_INFO("write flash success\r\n");
						u8Message[0] = 0x2;				
						u8Message[1] = 0x2;						
						sTxHeader.ExtId = RESPOND_UPDATE_BASE + HARDWARE_INDEX;
						sTxHeader.IDE = CAN_ID_EXT;
						sTxHeader.RTR = CAN_RTR_DATA;
						sTxHeader.DLC = 2;
						SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);	
					}
					
					ChangeProcessFunc(IAP_PreUpdateProcess);
					
					// 检查固件数据包是否全部发送完成
					if (g_sIAP_StateManager.sPkgInfo.u16PkgIndex == g_sIAP_StateManager.sPkgInfo.u16PkgTotalNum)
					{
						DEBUG_INFO("recv all pkg\r\n");
						
						u8Message[1] = 0x10;
						switch (cbUpdateSuccessProcess())
						{
							case EVENT_WRITE_EEPROM_SUCCESS:
								u8Message[1] = 0x3;
								break;
							
							case EVENT_WRITE_EEPROM_FAILED:
								u8Message[1] = 0x10;
								break;
							
							case EVENT_READ_EEPROM_FAILED:
								u8Message[1] = 0x11;
								break;
							
							case EVENT_EEPROM_CHECK_FAILED:
								u8Message[1] = 0x12;
								break;
							
							default:
								break;
						}
						
						u8Message[0] = 0x2;								
						sTxHeader.ExtId = RESPOND_UPDATE_BASE + HARDWARE_INDEX;
						sTxHeader.IDE = CAN_ID_EXT;
						sTxHeader.RTR = CAN_RTR_DATA;
						sTxHeader.DLC = 2;
						SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
						
						ChangeProcessFunc(IAP_IdleProcess);
					}
				}
				else
				{
					DEBUG_INFO("checksum incorrect\r\n");
					FLASH_If_Erase(g_sIAP_StateManager.u32ProgramAddr, g_sIAP_StateManager.u32ProgramSectorNum);
					u8Message[0] = 0x2;				
					u8Message[1] = 0x6;						
					sTxHeader.ExtId = RESPOND_UPDATE_BASE + HARDWARE_INDEX;
					sTxHeader.IDE = CAN_ID_EXT;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 2;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					
					ChangeProcessFunc(IAP_IdleProcess);
				}
			}

			
			g_sIAP_StateManager.eEvent = IAP_EVENT_NONE;
			ResetTimerCount(&g_sIAP_StateManager.u32TimerCount);
		}
	}
}


/**
  * @brief  IAP 流程更新前的预处理, 针对固件数据包信息
  * @param  None
  * @retval None
  */
static void IAP_PreUpdateProcess(void)
{
	uint8_t u8Message[8] = {0};
	CAN_TxHeaderTypeDef sTxHeader;
	g_sIAP_StateManager.eCurrentState = IAP_PREUPDATE;
	
	if (GetTimerTickDelta(g_sIAP_StateManager.u32TimerCount, GetCurTimerCount()) >= g_sIAP_StateManager.u32TimeoutCount)
	{
		// 超时处理
		DEBUG_INFO("recv pack info timeout\r\n");
		FLASH_If_Erase(g_sIAP_StateManager.u32ProgramAddr, g_sIAP_StateManager.u32ProgramSectorNum);
		ChangeProcessFunc(IAP_IdleProcess);
	}
	else
	{
		if (IAP_EVENT_PKG_INFO == g_sIAP_StateManager.eEvent)
		{
			DEBUG_INFO("recv and respond pack %d info\r\n", g_sIAP_StateManager.sPkgInfo.u16PkgIndex);
			
			g_sIAP_StateManager.eEvent = IAP_EVENT_NONE;
			
			u8Message[0] = 0x2;				
			u8Message[1] = 0x1;
			
			sTxHeader.ExtId = RESPOND_UPDATE_BASE + HARDWARE_INDEX;
			sTxHeader.IDE = CAN_ID_EXT;
			sTxHeader.RTR = CAN_RTR_DATA;
			sTxHeader.DLC = 2;

			SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
			
			g_sDataManager.u16CheckSum = 0;
			g_sDataManager.u8WriteIndex = 0;
			g_sDataManager.u8ReadIndex = 0;
			g_sDataManager.u16FrameNum = 0;
			
			// 进入固件更新阶段
			ChangeProcessFunc(IAP_UpdateProcess);
			ResetTimerCount(&g_sIAP_StateManager.u32TimerCount);
		}		
	}
}


void SendBusyFlag(void)
{
	uint8_t u8Message[8] = {0};
	CAN_TxHeaderTypeDef sTxHeader;
	
	// 回应固件更新请求, 并指示不可以更新固件
	u8Message[0] = 0x1;
	u8Message[1] = 0x3;

	sTxHeader.ExtId = RESPOND_UPDATE;
	sTxHeader.IDE = CAN_ID_EXT;
	sTxHeader.RTR = CAN_RTR_DATA;
	sTxHeader.DLC = 2;
	SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
}


/**
  * @brief  IAP 流程空闲状态事务处理
  * @param  None
  * @retval None
  */
static void IAP_IdleProcess(void)
{
	uint8_t u8Message[8] = {0};
	CAN_TxHeaderTypeDef sTxHeader;
	
	if (g_sIAP_StateManager.bForceBootFlag)
	{
		g_sIAP_StateManager.eCurrentState = IAP_IDLE_FORCE_BOOT;
		if (GetTimerTickDelta(g_sIAP_StateManager.u32ForceBootCount, GetCurTimerCount()) >= FORCE_BOOT_TIMEOUT)
		{
			g_sIAP_StateManager.bForceBootFlag = false;
		}
	}
	else
	{
		g_sIAP_StateManager.eCurrentState = IAP_IDLE;
	}
	
	if (IAP_EVENT_NONE != g_sIAP_StateManager.eEvent)
	{
		switch (g_sIAP_StateManager.eEvent)
		{
			case IAP_EVENT_SCANNING:
				
				// 反馈当前固件类型和版本信息bootloader:0x1, app:0x2
				if (TYPE_BOOTLOADER == g_sIAP_StateManager.eFirmwareType ||
					TYPE_APPLICATION == g_sIAP_StateManager.eFirmwareType)
				{
					DEBUG_INFO("send firmware type\r\n");
					
					u8Message[0] = g_sIAP_StateManager.eFirmwareType;				
					u8Message[1] = g_sIAP_StateManager.u32FirmwareVersion >> 16;
					u8Message[2] = g_sIAP_StateManager.u32FirmwareVersion >> 8;
					u8Message[3] = g_sIAP_StateManager.u32FirmwareVersion & 0xFF;
					
					#if defined(BOOTLOADER)
						u8Message[4] = 0;
					#elif defined(APPLICATION1)
						u8Message[4] = 1;
					#elif defined(APPLICATION2)
						u8Message[4] = 2;
					#endif
					
					sTxHeader.ExtId = RESPOND_FIRMWARE_INFO_BASE + HARDWARE_INDEX;
					sTxHeader.IDE = CAN_ID_EXT;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 5;

					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				}
				break;
			
			case IAP_EVENT_UPDATE_REQUEST:	
				if (g_sIAP_StateManager.bBusy)
				{
					SendBusyFlag();
					break;
				}
				
				DEBUG_INFO("update request,pkg num:%d\r\n", g_sIAP_StateManager.sPkgInfo.u16PkgTotalNum);
				DEBUG_INFO("respond update request\r\n");
			
				// 擦除更新区域
				FLASH_If_Init();
				g_sIAP_StateManager.u32ProgramAddr = APPLICATION1_ADDRESS;
				g_sIAP_StateManager.u32ProgramSectorNum = APP1_SECTOR_NUM;
				if (2 == g_sIAP_StateManager.u8UpdateIndex)
				{
					g_sIAP_StateManager.u32ProgramAddr = APPLICATION2_ADDRESS;
					g_sIAP_StateManager.u32ProgramSectorNum = APP2_SECTOR_NUM;
				}				
				FLASH_If_Erase(g_sIAP_StateManager.u32ProgramAddr, g_sIAP_StateManager.u32ProgramSectorNum);
			
				// 回应固件更新请求, 并指示可以更新固件的区域即索引
				u8Message[0] = 0x1;
				u8Message[1] = g_sIAP_StateManager.u8UpdateIndex;
			
				sTxHeader.ExtId = RESPOND_UPDATE;
				sTxHeader.IDE = CAN_ID_EXT;
				sTxHeader.RTR = CAN_RTR_DATA;
				sTxHeader.DLC = 2;
			
				g_sIAP_StateManager.sPkgInfo.u16PkgIndex = 0;
				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
			
				// 任务指针指向更新流程
				ChangeProcessFunc(IAP_PreUpdateProcess);
				break;
				
			case IAP_EVENT_ROLLBACK:
				if (g_sIAP_StateManager.bBusy)
				{
					SendBusyFlag();
					break;
				}
				
				
				#if defined(BOOTLOADER)
					DEBUG_INFO("can not rollback\r\n");
					u8Message[0] = 0x3;
					u8Message[1] = 0x1;			
					sTxHeader.ExtId = RESPOND_UPDATE;
					sTxHeader.IDE = CAN_ID_EXT;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = 2;
				#else
					if (EVENT_WRITE_EEPROM_SUCCESS == cbRollbackCheck())
					{
						u8Message[0] = 0x3;
						u8Message[1] = 0x2;
						sTxHeader.ExtId = RESPOND_UPDATE;
						sTxHeader.IDE = CAN_ID_EXT;
						sTxHeader.RTR = CAN_RTR_DATA;
						sTxHeader.DLC = 2;
						SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
						
						HAL_Delay(10);
						cbRollbackProcess();
					}
					else
					{
						u8Message[0] = 0x3;
						u8Message[1] = 0x1;
						sTxHeader.ExtId = RESPOND_UPDATE;
						sTxHeader.IDE = CAN_ID_EXT;
						sTxHeader.RTR = CAN_RTR_DATA;
						sTxHeader.DLC = 2;
						SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
					}
				#endif
				break;
			
			case IAP_EVENT_READ_EEPROM:
				if (EVENT_READ_EEPROM_SUCCESS == cbReadEEPROM(g_sIAP_StateManager.sEEPROM_Info.u16ReadAddress, 
																g_sIAP_StateManager.sEEPROM_Info.u8Data,
																g_sIAP_StateManager.sEEPROM_Info.u8DataLen))
				{
					if (g_sIAP_StateManager.sEEPROM_Info.u8DataLen > 8)
					{
						g_sIAP_StateManager.sEEPROM_Info.u8DataLen = 8;
					}
					memcpy(u8Message, g_sIAP_StateManager.sEEPROM_Info.u8Data, g_sIAP_StateManager.sEEPROM_Info.u8DataLen);
					sTxHeader.ExtId = RESPOND_EEPROM_READ_REQUEST;
					sTxHeader.IDE = CAN_ID_EXT;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = g_sIAP_StateManager.sEEPROM_Info.u8DataLen;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				}
				else
				{
					u8Message[0] = 
					sTxHeader.ExtId = RESPOND_EEPROM_READ_REQUEST;
					sTxHeader.IDE = CAN_ID_EXT;
					sTxHeader.RTR = CAN_RTR_DATA;
					sTxHeader.DLC = g_sIAP_StateManager.sEEPROM_Info.u8DataLen;
					SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				}
				break;
				
			case IAP_EVENT_WRITE_EEPROM:
				u8Message[0] = 2;
				sTxHeader.ExtId = RESPOND_EEPROM_WRITE_REQUEST;
				sTxHeader.IDE = CAN_ID_EXT;
				sTxHeader.RTR = CAN_RTR_DATA;
				sTxHeader.DLC = 1;
				if (EVENT_WRITE_EEPROM_SUCCESS == cbWriteEEPROM(g_sIAP_StateManager.sEEPROM_Info.u16WriteAddress, 
																g_sIAP_StateManager.sEEPROM_Info.u8Data[0]))
				{
					u8Message[0] = 1;					
				}
				SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
				break;
					
			default:
				break;
		}
		
		g_sIAP_StateManager.eEvent = IAP_EVENT_NONE;
	}
}



void IAP_MessageHandler(CAN_RxHeaderTypeDef *pRxHeader, uint8_t *pMessage)
{
	// 检查是否是扩展帧
	if (CAN_ID_EXT == pRxHeader->IDE)
	{	
		switch (pRxHeader->ExtId)
		{
			case UPPER_COMPUTER_SCAN:
				// 判断上位机是否是扫描全部硬件或扫描此硬件
				if (pMessage[0] == SCAN_ALL || pMessage[0] == SCAN_ME)
				{
					g_sIAP_StateManager.eEvent = IAP_EVENT_SCANNING;
				
					// 如果当前处于bootloader阶段, 则要重新设置计时器
					if (TYPE_BOOTLOADER == g_sIAP_StateManager.eFirmwareType)
					{
						if (pMessage[1] == FORCE_BOOT)
						{
							ResetTimerCount(&g_sIAP_StateManager.u32ForceBootCount);
							g_sIAP_StateManager.bForceBootFlag = true;
						}
					}
				}
				break;
			
			case UPDATE_REQUEST:
				if (0x1 == pMessage[0])
				{
					// 获取固件数据包数量信息
					g_sIAP_StateManager.sPkgInfo.u16PkgTotalNum = (pMessage[1] << 8) | pMessage[2];
					g_sIAP_StateManager.eEvent = IAP_EVENT_UPDATE_REQUEST;
				}
				else if (0x2 == pMessage[0])
				{
					g_sIAP_StateManager.eEvent = IAP_EVENT_ROLLBACK;
				}
				break;
			
			case PACKAGE_INFO:
				// 获取固件数据包信息
				if (g_sIAP_StateManager.sPkgInfo.u16PkgIndex == (uint16_t)((pMessage[0] << 8) | pMessage[1]))
				{				
					g_sIAP_StateManager.sPkgInfo.u16FrameNum = (uint16_t)((pMessage[2] << 8) | pMessage[3]);
					g_sIAP_StateManager.sPkgInfo.u16CheckSum = (uint16_t)((pMessage[4] << 8) | pMessage[5]);
					g_sIAP_StateManager.eEvent = IAP_EVENT_PKG_INFO;
					g_sIAP_StateManager.sPkgInfo.u16PkgIndex++;
				}
				break;
			
			case PACKAGE_DATA:
				// 固件数据
				if (8 == pRxHeader->DLC)
				{
					memcpy(g_sDataManager.u8Data[g_sDataManager.u8WriteIndex], pMessage, pRxHeader->DLC);
					g_sDataManager.u8WriteIndex++;
					g_sDataManager.u16FrameNum++;
					g_sIAP_StateManager.eEvent = IAP_EVENT_PKG_FRAME;
				}
				break;
				
			case EEPROM_READ_REQUEST:
				g_sIAP_StateManager.sEEPROM_Info.u16ReadAddress = 
					(uint16_t)((pMessage[0] << 16) | pMessage[1]);
				g_sIAP_StateManager.sEEPROM_Info.u8DataLen = pMessage[2];
				g_sIAP_StateManager.eEvent = IAP_EVENT_READ_EEPROM;
				break;
			
			case EEPROM_WRITE_REQUEST:
				g_sIAP_StateManager.sEEPROM_Info.u16WriteAddress = 
					(uint16_t)((pMessage[0] << 16) | pMessage[1]);
				g_sIAP_StateManager.sEEPROM_Info.u8Data[0] = pMessage[2];
				g_sIAP_StateManager.eEvent = IAP_EVENT_WRITE_EEPROM;
				break;
			
		}
	}
}


void IAP_ManagerInit(void)
{
	// 获取固件信息
	RegisterRecvProcess(IAP_MessageHandler);
}


void IAP_ManagerRun(void)
{
	if (g_sIAP_StateManager.pFunc)
	{
		g_sIAP_StateManager.pFunc();
	}
	
	uint8_t u8Message[8] = {0};
	CAN_TxHeaderTypeDef sTxHeader;
		
//	u8Message[0] = 0x3;
//	u8Message[1] = 0x2;
//	sTxHeader.StdId = 0x100;
//	sTxHeader.IDE = CAN_ID_STD;
//	sTxHeader.RTR = CAN_RTR_DATA;
//	sTxHeader.DLC = 2;
//	SendMessageHandler(&sTxHeader, u8Message, DEFAULT_KEY_INDEX);
}


