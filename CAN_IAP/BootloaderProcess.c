#include "BootloaderProcess.h"
#include "i2c.h"
#include "can.h"
#include "Timer.h"
#include "CAN_IAP.h"
#include "ExternalStorage.h"
#include "usart.h"
#include "SwitchHandler.h"



// Bootloader 版本
#define BOOTLOADER_MAJOR_VERSION	0
#define BOOTLOADER_MINOR_VERSION	0
#define BOOTLOADER_PATCH_VERSION	1

#define PROGRAM_FLAG_ADDR			0x0
#define APP1_FLAG_ADDR				0x1
#define APP2_FLAG_ADDR				0x2
#define APP_INDEX_ADDR				0x3
#define LAST_FIRMWARE_ADDR_INDEX	0x4


#define AUTO_JUMP_AFTER_UPDATE		true



typedef void (*pFunction)(void);
static void ReadBootloaderParams(void);
static void CheckBootloaderParams(void);
static void SetIAP_Params(void);
static void CheckIAP_State(void);
static void WriteUpdateInfo(void);
static void StartApp(void);
static bool CheckJumpAddressData(const uint8_t u8Index);

struct
{
	uint32_t u32WaitCount;
	uint8_t u8ReadData[8];
	uint8_t u8WriteData[8];
	uint8_t u8AppIndex;		// 应用程序跳转索引
	uint8_t u8UpdateIndex;	// 固件更新区域索引
	bool bCurrentAppValid;
	bool bUpdateSuccess;
	pFunction pProcess;
	uint32_t u32BlinkCount;
	uint8_t u8Step;
}g_sBootloaderManager = 
{
	.pProcess = ReadBootloaderParams,
	.u32BlinkCount = 200,
};



static bool JumpToUserApplication(const uint8_t u8Index)
{
	uint32_t JumpAddress;
	uint32_t u32FirmwareAddress;
	pFunction JumpToApplication;
	
	if (u8Index == 1)
	{
		u32FirmwareAddress = APPLICATION1_ADDRESS;
	}
	else if (u8Index == 2)
	{
		u32FirmwareAddress = APPLICATION2_ADDRESS;
	}
	else
	{
		return false;
	}
	
	DEBUG_INFO("prepare to jump to area %d\r\n", u8Index);
	if (!CheckJumpAddressData(u8Index))
	{
		return false;
	}
	
	__disable_irq();
			
	JumpAddress = *(__IO uint32_t*) (u32FirmwareAddress + 4);
	JumpToApplication = (pFunction) JumpAddress;
	
	__set_MSP(*(__IO uint32_t*) u32FirmwareAddress);

	JumpToApplication();	
	
	return true;

}


/**
  * @brief  Run灯控制
  * @param  None
  * @retval None
  */
static void RunningLED_Process(void)
{
	static uint32_t u32BlinkCount = 0;
	
	
	if (GetTimerTickDelta(u32BlinkCount, GetCurTimerCount()) >= g_sBootloaderManager.u32BlinkCount)
	{
		ResetTimerCount(&u32BlinkCount);
		HAL_GPIO_TogglePin(RunningLED_GPIO_Port, RunningLED_Pin);
	}
}


/**
  * @brief  读取bootloader参数
  * @param  None
  * @retval None
  */
static void ReadBootloaderParams(void)
{
	if (GetTimerTickDelta(g_sBootloaderManager.u32WaitCount, GetCurTimerCount()) >= 10)
	{		
		DEBUG_INFO("start read eeprom\r\n");
		
		// 读取MCU烧录标志, 第一次烧录程序需要用烧录器将bootloader和app一起烧入
		if (HAL_OK == ReadMemory(PROGRAM_FLAG_ADDR, g_sBootloaderManager.u8ReadData, 4))
		{
			DEBUG_INFO("read eeprom success\r\n");
			g_sBootloaderManager.pProcess = CheckBootloaderParams;
		}
		else
		{
			DEBUG_INFO("read eeprom failed\r\n");
		}
		
		ResetTimerCount(&g_sBootloaderManager.u32WaitCount);		
	}
}


static bool CheckJumpAddressData(const uint8_t u8Index)
{
	bool bRet = false;
	uint32_t u32Address = 0;
	
	DEBUG_INFO("jump area is %d\r\n", u8Index);
	
	if (u8Index == 1)
	{
		u32Address = APPLICATION1_ADDRESS;
	}
	else if (u8Index == 2)
	{
		u32Address = APPLICATION2_ADDRESS;
	}
	else
	{
		DEBUG_INFO("index of jump area is not correct\r\n");
		return bRet;
	}
	
	if (u32Address != 0 && (((*(__IO uint32_t*)u32Address) & 0x2FFE0000 ) == 0x20000000))
	{
		DEBUG_INFO("data in jump area is correct\r\n");		
		bRet = true;
	}
	else
	{
		DEBUG_INFO("data in jump area is not correct\r\n");
		
	}

	return bRet;
}


/**
  * @brief  检查bootloader参数
  * @param  None
  * @retval None
  */
static void CheckBootloaderParams(void)
{
	if (GetTimerTickDelta(g_sBootloaderManager.u32WaitCount, GetCurTimerCount()) >= 10)
	{				
		if (1 != g_sBootloaderManager.u8ReadData[0])
		{
			/* 如果bootloader烧录标志不为1, 则是第一次烧录固件, 需要写入烧录标志
				addr0 = 0x1:bootloader已烧录
				addr1 = 0x1:app1正常
				addr2 = 0x0:app2无效
				addr3 = 0x1:跳转至app1
			*/
			g_sBootloaderManager.u8WriteData[0] = 1;
			g_sBootloaderManager.u8WriteData[1] = 1;
			g_sBootloaderManager.u8WriteData[2] = 0;
			g_sBootloaderManager.u8WriteData[3] = 1;
			
			if (HAL_OK == WriteMemory(PROGRAM_FLAG_ADDR, g_sBootloaderManager.u8WriteData, 4))
			{
				DEBUG_INFO("first time to set eeprom\r\n");
				g_sBootloaderManager.pProcess = ReadBootloaderParams;
			}
		}
		else
		{
			// bootloader已烧录, 检查跳转索引是否有效
			if (g_sBootloaderManager.u8ReadData[3] == 1 || g_sBootloaderManager.u8ReadData[3] == 2)
			{
				g_sBootloaderManager.u8AppIndex = g_sBootloaderManager.u8ReadData[3];
				
				DEBUG_INFO("jump index is %d\r\n", g_sBootloaderManager.u8AppIndex);
				
				/* 
					检查对应的固件区域标志是否正常, 1和2均为正常, 1是可以直接跳转, 
					2代表更新固件后未直接跳转运行, 而是从bootloader启动了, 需要把标志
					置为无效后再跳转
				*/
				if (1 == g_sBootloaderManager.u8ReadData[g_sBootloaderManager.u8AppIndex] || 
					2 == g_sBootloaderManager.u8ReadData[g_sBootloaderManager.u8AppIndex])
				{
					// 检查跳转的FLASH地址中的数据是否正常
					if (CheckJumpAddressData(g_sBootloaderManager.u8AppIndex))
					{
						if (2 == g_sBootloaderManager.u8ReadData[g_sBootloaderManager.u8AppIndex])
						{
							uint8_t u8FirmwareFlag = 3;
							if (HAL_OK == WriteMemory(PROGRAM_FLAG_ADDR + g_sBootloaderManager.u8AppIndex, &u8FirmwareFlag, 1))
							{
								g_sBootloaderManager.pProcess = SetIAP_Params;
							}
							else
							{
								g_sBootloaderManager.pProcess = ReadBootloaderParams;
							}
						}
						else
						{
							g_sBootloaderManager.pProcess = SetIAP_Params;
						}
					}
					else
					{
						// 如果跳转的FLASH地址中的数据不正常, 强行让该固件区域标志不正常										
						DEBUG_INFO("set firmware flag invalid\r\n");
						
						uint8_t u8FirmwareFlag = 0;
						WriteMemory(PROGRAM_FLAG_ADDR + g_sBootloaderManager.u8AppIndex, &u8FirmwareFlag, 1);
						g_sBootloaderManager.pProcess = ReadBootloaderParams;
					}
				}
				else
				{
					// 当前固件区域标志不正常时, 检查另一个区域标志是否正常
					if (1 == g_sBootloaderManager.u8ReadData[(g_sBootloaderManager.u8AppIndex % 2 + 1)] ||
						2 == g_sBootloaderManager.u8ReadData[(g_sBootloaderManager.u8AppIndex % 2 + 1)])
					{
						DEBUG_INFO("another firmware flag is valid\r\n");
						
						// 检查跳转的FLASH地址中的数据是否正常
						if (CheckJumpAddressData((g_sBootloaderManager.u8AppIndex % 2 + 1)))
						{
							g_sBootloaderManager.u8AppIndex = g_sBootloaderManager.u8AppIndex % 2 + 1;
							
							DEBUG_INFO("set jump index:%d\r\n", g_sBootloaderManager.u8AppIndex);
							
							WriteMemory(APP_INDEX_ADDR, &g_sBootloaderManager.u8AppIndex, 1);
							g_sBootloaderManager.pProcess = ReadBootloaderParams;
						}
						else
						{
							// 检查不正常就设置跳转索引为无效							
							g_sBootloaderManager.u8AppIndex = 0x0;
							
							DEBUG_INFO("set jump index:%d\r\n", g_sBootloaderManager.u8AppIndex);
							
							WriteMemory(APP_INDEX_ADDR, &g_sBootloaderManager.u8AppIndex, 1);
							g_sBootloaderManager.pProcess = ReadBootloaderParams;
						}
					}
					else
					{
						// 没有合适的固件区域可跳转
						g_sBootloaderManager.u8AppIndex = 0x0;
						
						DEBUG_INFO("another firmware flag is invalid, set jump index:%d\r\n", g_sBootloaderManager.u8AppIndex);
						
						WriteMemory(APP_INDEX_ADDR, &g_sBootloaderManager.u8AppIndex, 1);
						g_sBootloaderManager.pProcess = ReadBootloaderParams;
					}
				}
			}
			else	// 跳转索引无效
			{				
				g_sBootloaderManager.u8AppIndex = 0x0;
				
				DEBUG_INFO("jump index is invalid, set jump index:%d\r\n", g_sBootloaderManager.u8AppIndex);
				
				WriteMemory(APP_INDEX_ADDR, &g_sBootloaderManager.u8AppIndex, 1);
				g_sBootloaderManager.pProcess = SetIAP_Params;
			}
		}
		
		ResetTimerCount(&g_sBootloaderManager.u32WaitCount);
	}	
}


/**
  * @brief  设置IAP相关参数
  * @param  None
  * @retval None
  */
static void SetIAP_Params(void)
{	
	uint8_t u8Index;
	
	g_sBootloaderManager.u32BlinkCount  = 500;
	
	DEBUG_INFO("set iap params\r\n");
	
	// 告诉IAP当前bootloader的版本
	g_sIAP_ProcessManager.SetFimwareInfo(TYPE_BOOTLOADER, 
		(uint32_t)((BOOTLOADER_MAJOR_VERSION << 16) | 
					(BOOTLOADER_MINOR_VERSION << 8) |
					BOOTLOADER_PATCH_VERSION));
	
	
	if (1 == g_sBootloaderManager.u8AppIndex || 2 == g_sBootloaderManager.u8AppIndex)
	{
		g_sBootloaderManager.u8UpdateIndex = g_sBootloaderManager.u8AppIndex % 2 + 1;
	}
	else
	{
		// 如果app索引无效, 就把更新索引指向第一个区域待固件更新
		g_sBootloaderManager.u8UpdateIndex = 1;
	}
	
	DEBUG_INFO("update area index is:%d\r\n", g_sBootloaderManager.u8UpdateIndex);
	
	
	// 设置IAP可更新区域
	g_sIAP_ProcessManager.SetUpdateIndex(g_sBootloaderManager.u8UpdateIndex);	
	
	ResetTimerCount(&g_sBootloaderManager.u32WaitCount);
	g_sBootloaderManager.pProcess = CheckIAP_State;
}


static void IAP_Update(void)
{
	if (IAP_IDLE == g_sIAP_ProcessManager.GetIAP_CurrentState())
	{
		g_sBootloaderManager.pProcess = CheckIAP_State;
	}
}


/**
  * @brief  检查IAP流程当前的状态, 根据IAP状态处理相应的事务
  * @param  None
  * @retval None
  */
static void CheckIAP_State(void)
{	
	if (IAP_IDLE == g_sIAP_ProcessManager.GetIAP_CurrentState() || 
		IAP_IDLE_FORCE_BOOT == g_sIAP_ProcessManager.GetIAP_CurrentState())
	{
		if (GetTimerTickDelta(g_sBootloaderManager.u32WaitCount, GetCurTimerCount()) >= 2000)
		{
			if (IAP_IDLE_FORCE_BOOT == g_sIAP_ProcessManager.GetIAP_CurrentState())
			{
				DEBUG_INFO("force in bootloader\r\n");
				ResetTimerCount(&g_sBootloaderManager.u32WaitCount);
			}
			else
			{
				// 电源板比较特殊, 不可以主动跳转, 必须等待工控机关机才能跳转
				if (g_sBootloaderManager.bUpdateSuccess)
				{
					if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(PC_State_GPIO_Port, PC_State_Pin))
					{
						g_sBootloaderManager.pProcess = StartApp;
					}
				}
				else
				{
					g_sBootloaderManager.pProcess = StartApp;
				}
			}
		}
	}
	else
	{
		if (IAP_PREUPDATE == g_sIAP_ProcessManager.GetIAP_CurrentState())
		{
			g_sBootloaderManager.pProcess = IAP_Update;
		}
		
		ResetTimerCount(&g_sBootloaderManager.u32WaitCount);
	}
}


static void WriteUpdateInfo(void)
{
	uint8_t u8Data = 0;
	
	switch (g_sBootloaderManager.u8Step)
	{
		case 0:
			u8Data = 0x2;
			if (HAL_OK == WriteMemory(g_sBootloaderManager.u8UpdateIndex, &u8Data, 1))
			{				
				g_sBootloaderManager.u8Step = 5;
			}	
			break;
			
		case 5:
			if (HAL_OK == ReadMemory(g_sBootloaderManager.u8UpdateIndex, &u8Data, 1))
			{
				if (0x2 == u8Data)
				{
					g_sBootloaderManager.u8Step = 10;
				}
				else
				{
					g_sBootloaderManager.u8Step = 0;
				}
			}
			break;
			
		case 10:
			if (HAL_OK == WriteMemory(APP_INDEX_ADDR, &g_sBootloaderManager.u8UpdateIndex, 1))
			{				
				g_sBootloaderManager.u8Step = 15;
			}	
			break;
			
		case 15:
			if (HAL_OK == ReadMemory(APP_INDEX_ADDR, &u8Data, 1))
			{
				if (u8Data == g_sBootloaderManager.u8UpdateIndex)
				{
					g_sBootloaderManager.u8AppIndex = g_sBootloaderManager.u8UpdateIndex;
					g_sBootloaderManager.pProcess = StartApp;
					g_sBootloaderManager.u8Step = 0;
				}
				else
				{
					g_sBootloaderManager.u8Step = 10;
				}
			}
			break;
	}
}


static void StartApp(void)
{
	if (!JumpToUserApplication(g_sBootloaderManager.u8AppIndex))
	{
		g_sBootloaderManager.pProcess = CheckIAP_State;
	}
}


FirmwareEvent cbUpdateSuccessProcess(void)
{
	uint8_t u8Data = 0;
	g_sBootloaderManager.bUpdateSuccess = true;
	
	// 更新EEPROM中的固件信息
	u8Data = 2;
	if (HAL_OK != WriteMemory(g_sBootloaderManager.u8UpdateIndex, &u8Data, 1))
	{		
		DEBUG_INFO("update eeprom flag failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	if (HAL_OK != ReadMemory(g_sBootloaderManager.u8UpdateIndex, &u8Data, 1))
	{
		DEBUG_INFO("read eeprom flag failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	if (2 != u8Data)
	{
		DEBUG_INFO("check eeprom flag failed\r\n");
		return EVENT_EEPROM_CHECK_FAILED;
	}
	HAL_Delay(10);

	u8Data = g_sBootloaderManager.u8UpdateIndex;
	if (HAL_OK != WriteMemory(APP_INDEX_ADDR, &u8Data, 1))
	{		
		DEBUG_INFO("write eeprom index failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}	
	HAL_Delay(10);
	
	if (HAL_OK != ReadMemory(APP_INDEX_ADDR, &u8Data, 1))
	{
		DEBUG_INFO("read eeprom index failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	if (g_sBootloaderManager.u8UpdateIndex != u8Data)
	{
		DEBUG_INFO("check eeprom index failed\r\n");
		return EVENT_EEPROM_CHECK_FAILED;
	}
	HAL_Delay(10);
	
	
	// 记录本次固件信息
	u8Data = 0;
	if (HAL_OK != WriteMemory(LAST_FIRMWARE_ADDR_INDEX, &u8Data, 1))
	{		
		DEBUG_INFO("mark index failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}	
	HAL_Delay(10);
	
	if (HAL_OK != ReadMemory(LAST_FIRMWARE_ADDR_INDEX, &u8Data, 1))
	{
		DEBUG_INFO("read mark index failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	if (0 != u8Data)
	{
		DEBUG_INFO("check mark index failed\r\n");
		return EVENT_EEPROM_CHECK_FAILED;
	}
	
	g_sIAP_ProcessManager.pSetIAP_Busy();
	g_sBootloaderManager.u8AppIndex = g_sBootloaderManager.u8UpdateIndex;
	
	return EVENT_WRITE_EEPROM_SUCCESS;
}


FirmwareEvent cbWriteEEPROM(const uint16_t u16Address, const uint8_t u8Data)
{
	uint8_t u8DataRead = 0;
	
	if (HAL_OK != WriteMemory(u16Address, &u8Data, 1))
	{		
		DEBUG_INFO("write eeprom failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}	
	HAL_Delay(10);
	
	
	if (HAL_OK != ReadMemory(u16Address, &u8DataRead, 1))
	{
		DEBUG_INFO("read eeprom failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	HAL_Delay(10);	
	
	if (u8DataRead != u8Data)
	{
		DEBUG_INFO("check data failed\r\n");
		return EVENT_EEPROM_CHECK_FAILED;
	}
	
	DEBUG_INFO("write eeprom addr:%d success\r\n", u16Address);
	
	return EVENT_WRITE_EEPROM_SUCCESS;
}


FirmwareEvent cbReadEEPROM(const uint16_t u16Address, uint8_t *pData, const uint8_t u8DataLen)
{
	uint8_t u8DataLenCheck = u8DataLen;
	
	if (u8DataLenCheck > 8)
	{
		u8DataLenCheck = 8;
	}
	
	if (HAL_OK != ReadMemory(u16Address, pData, u8DataLen))
	{
		DEBUG_INFO("read eeprom data failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	
	DEBUG_INFO("read eeprom addr:%d,len:%d success\r\n", u16Address, u8DataLen);
	
	return EVENT_READ_EEPROM_SUCCESS;
}


void cbTripleClickHandler(void)
{
	HAL_GPIO_WritePin(AC_PowerControl_GPIO_Port, AC_PowerControl_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(Power24V_Control_GPIO_Port, Power24V_Control_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LiftingAxisDrive_GPIO_Port, LiftingAxisDrive_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RelayControl_GPIO_Port, RelayControl_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SW_Green_GPIO_Port, SW_Green_Pin, GPIO_PIN_SET);
}


void cbLongPressHandler(void)
{
	HAL_GPIO_WritePin(LiftingAxisDrive_GPIO_Port, LiftingAxisDrive_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Power24V_Control_GPIO_Port, Power24V_Control_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RelayControl_GPIO_Port, RelayControl_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AC_PowerControl_GPIO_Port, AC_PowerControl_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SW_Green_GPIO_Port, SW_Green_Pin, GPIO_PIN_RESET);
}


BtnStruct PowerSwitch = 
{
	.u8BtnID = 1,
	.pCbClickHandler = NULL,
	.pCbLongPressHandler = cbLongPressHandler,
	.pCbTripleClickHandler = cbTripleClickHandler,
};



bool cbSwitchTriggerDetect(BtnStruct *pBtn)
{
	GPIO_PinState bState = GPIO_PIN_RESET;
	
	switch (pBtn->u8BtnID)
	{
		case 1:
			bState = HAL_GPIO_ReadPin(PowerSwitch_GPIO_Port, PowerSwitch_Pin);
			break;
	}
	
	return bState;
}


void BootloaderProcessInit(void)
{
//	IAP_SetUpdateFinishProcess(cbUpdateSuccessProcess);
	IAP_ManagerInit();
	RegisterButton(&PowerSwitch);
}


void BootloaderProcess(void)
{
	if (g_sBootloaderManager.pProcess)
	{
		g_sBootloaderManager.pProcess();
	}
	
	// 运行IAP流程
	IAP_ManagerRun();
	
	RunningLED_Process();
	
	SwitchHandlerRun();
}

