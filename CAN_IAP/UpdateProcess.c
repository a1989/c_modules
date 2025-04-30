#include "UpdateProcess.h"
#include "CAN_IAP.h"
#include "usart.h"
#include "Timer.h"
#include "ExternalStorage.h"
#include "Version.h"


#define PROGRAM_FLAG_ADDR			0x0
#define APP1_FLAG_ADDR				0x1
#define APP2_FLAG_ADDR				0x2
#define APP_INDEX_ADDR				0x3
#define LAST_FIRMWARE_ADDR_INDEX	0x4


typedef void (*pFunction)(void);
static void ReadFirmwareFlag(void);
static void CheckFirmwareFlag(void);
static void SetIAP_Params(void);
static void CheckIAP_State(void);
static void WriteUpdateInfo(void);
static void StartNewApp(void);


struct
{
	uint32_t u32WaitCount;
	uint8_t u8ReadData[8];
	uint8_t u8WriteData[8];
	uint8_t u8FirmwareFlagAddr;
	uint8_t u8FirmwareFlag;
	uint8_t u8AppIndex;		// 应用程序跳转索引
	uint8_t u8UpdateIndex;	// 固件更新区域索引
	bool bCurrentAppValid;
	bool bUpdateSuccess;
	bool bPrepareRollback;
	uint32_t u32RollbackAddress;
	pFunction pProcess;
	uint8_t u8Step;
}g_sUpdateManager = 
{
	.pProcess = ReadFirmwareFlag,
};


#if defined(APPLICATION1)



#endif



/**
  * @brief  读取固件标志参数
  * @param  None
  * @retval None
  */
static void ReadFirmwareFlag(void)
{
	if (GetTimerTickDelta(g_sUpdateManager.u32WaitCount, GetCurTimerCount()) >= 10)
	{		
		DEBUG_INFO("start read eeprom\r\n");
		
		// 读取该固件对应的固件标志
		if (HAL_OK == ReadMemory(g_sUpdateManager.u8FirmwareFlagAddr, 
									&g_sUpdateManager.u8FirmwareFlag, 1))
		{
			DEBUG_INFO("read flag success\r\n");			
		}
		else
		{
			DEBUG_INFO("read flag failed\r\n");
		}
		
		g_sUpdateManager.pProcess = CheckFirmwareFlag;
		
		ResetTimerCount(&g_sUpdateManager.u32WaitCount);		
	}
}


/**
  * @brief  检查固件标志参数
  * @param  None
  * @retval None
  */
static void CheckFirmwareFlag(void)
{
	if (GetTimerTickDelta(g_sUpdateManager.u32WaitCount, GetCurTimerCount()) >= 10)
	{				
		if (1 != g_sUpdateManager.u8FirmwareFlag)
		{
			
			uint8_t u8Data = 0x1;
			if (HAL_OK == WriteMemory(g_sUpdateManager.u8FirmwareFlagAddr, &u8Data, 1))
			{
				DEBUG_INFO("set firmware flag success\r\n");	
			}
		}
		g_sUpdateManager.pProcess = CheckIAP_State;
		
		ResetTimerCount(&g_sUpdateManager.u32WaitCount);
	}	
}


static void IAP_Update(void)
{
	if (IAP_IDLE == g_sIAP_ProcessManager.GetIAP_CurrentState())
	{
		g_sUpdateManager.pProcess = CheckIAP_State;
	}
}


__weak void StopInt(void)
{
	return;
}


/**
  * @brief  检查IAP流程当前的状态, 根据IAP状态处理相应的事务
  * @param  None
  * @retval None
  */
static void CheckIAP_State(void)
{	
	if (IAP_IDLE == g_sIAP_ProcessManager.GetIAP_CurrentState())
	{
		if (g_sUpdateManager.bUpdateSuccess)
		{	
			// 电源板比较特殊, 不可以主动跳转, 必须等待工控机关机才能跳转
			if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(PC_State_GPIO_Port, PC_State_Pin))
			{
				StartNewApp();
			}
		}
		else if (g_sUpdateManager.bPrepareRollback)
		{
			if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(PC_State_GPIO_Port, PC_State_Pin))
			{
				uint32_t JumpAddress;
				pFunction JumpToApplication;
				
				StopInt();
				__disable_irq();
				
				JumpAddress = *(__IO uint32_t*) (g_sUpdateManager.u32RollbackAddress + 4);
				JumpToApplication = (pFunction) JumpAddress;
				
				__set_MSP(*(__IO uint32_t*) g_sUpdateManager.u32RollbackAddress);
				JumpToApplication();
//				__set_FAULTMASK(1);
//				NVIC_SystemReset();				
			}
		}
	}
	else
	{
		if (IAP_PREUPDATE == g_sIAP_ProcessManager.GetIAP_CurrentState())
		{
			if (g_sUpdateManager.bUpdateSuccess || g_sUpdateManager.bPrepareRollback)
			{
				
			}
			else
			{
				g_sUpdateManager.bUpdateSuccess = false;
				g_sUpdateManager.pProcess = IAP_Update;
			}
		}
		
		ResetTimerCount(&g_sUpdateManager.u32WaitCount);
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
	
	StopInt();
	__disable_irq();
	
			
	JumpAddress = *(__IO uint32_t*) (u32FirmwareAddress + 4);
	JumpToApplication = (pFunction) JumpAddress;
	
	__set_MSP(*(__IO uint32_t*) u32FirmwareAddress);

	JumpToApplication();	
	
	return true;

}


static void StartNewApp(void)
{
	if (!JumpToUserApplication(g_sUpdateManager.u8AppIndex))
	{
		g_sUpdateManager.pProcess = CheckIAP_State;
	}
}


FirmwareEvent cbUpdateSuccessProcess(void)
{
	uint8_t u8Data = 0;
	g_sUpdateManager.bUpdateSuccess = true;
	
	// 更新EEPROM中的固件信息
	u8Data = 2;
	if (HAL_OK != WriteMemory(g_sUpdateManager.u8UpdateIndex, &u8Data, 1))
	{		
		DEBUG_INFO("update eeprom flag failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	if (HAL_OK != ReadMemory(g_sUpdateManager.u8UpdateIndex, &u8Data, 1))
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

	u8Data = g_sUpdateManager.u8UpdateIndex;
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
	
	if (g_sUpdateManager.u8UpdateIndex != u8Data)
	{
		DEBUG_INFO("check eeprom index failed\r\n");
		return EVENT_EEPROM_CHECK_FAILED;
	}
	
	
	// 记录本次固件信息
	u8Data = 0;
	#if defined(APPLICATION1)
		u8Data = 1;
	#elif defined(APPLICATION2)
		u8Data = 2;
	#endif
	
	if (HAL_OK != WriteMemory(LAST_FIRMWARE_ADDR_INDEX, &u8Data, 1))
	{		
		DEBUG_INFO("mark index failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}	
	HAL_Delay(10);
	
	uint8_t u8ReadBack = 0xFF;
	if (HAL_OK != ReadMemory(LAST_FIRMWARE_ADDR_INDEX, &u8ReadBack, 1))
	{
		DEBUG_INFO("read mark index failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	if (u8ReadBack != u8Data)
	{
		DEBUG_INFO("check mark index failed\r\n");
		return EVENT_EEPROM_CHECK_FAILED;
	}
	
	g_sIAP_ProcessManager.pSetIAP_Busy();
	g_sUpdateManager.u8AppIndex = g_sUpdateManager.u8UpdateIndex;
	
	return EVENT_WRITE_EEPROM_SUCCESS;
}


FirmwareEvent cbRollbackCheck(void)
{
	uint8_t u8Data = 0;
	uint8_t u8CurrentIndex = 0;
		
	if (HAL_OK != ReadMemory(LAST_FIRMWARE_ADDR_INDEX, &u8Data, 1))
	{
		DEBUG_INFO("read mark index failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	uint8_t u8JumpArea = 0;
	
	/* 
		如果上次固件的标志和当前固件索引不同, 则回退至上次固件, 如果相同,
		就回退到bootloader
	*/
	#if defined(APPLICATION1)
		u8CurrentIndex = 1;
		if (2 == u8Data)
		{
			u8JumpArea = 2;
			DEBUG_INFO("prepare to rollback to app2\r\n");
		}
	#elif defined(APPLICATION2)
		u8CurrentIndex = 2;
		if (1 == u8Data)
		{
			u8JumpArea = 1;
			DEBUG_INFO("prepare to rollback to app1\r\n");
		}
	#endif
	
	// 将当前固件标志置为无效
	u8Data = 0;
	if (HAL_OK != WriteMemory(u8CurrentIndex, &u8Data, 1))
	{		
		DEBUG_INFO("mark index failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}	
	HAL_Delay(10);
	
	
	if (HAL_OK != ReadMemory(u8CurrentIndex, &u8Data, 1))
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
	
	
	
	// 设置固件跳转索引
	if (HAL_OK != WriteMemory(APP_INDEX_ADDR, &u8JumpArea, 1))
	{		
		DEBUG_INFO("mark index failed\r\n");
		return EVENT_WRITE_EEPROM_FAILED;
	}	
	HAL_Delay(10);
	
	
	if (HAL_OK != ReadMemory(APP_INDEX_ADDR, &u8Data, 1))
	{
		DEBUG_INFO("read mark index failed\r\n");
		return EVENT_READ_EEPROM_FAILED;
	}
	HAL_Delay(10);
	
	
	if (u8JumpArea != u8Data)
	{
		DEBUG_INFO("check mark index failed\r\n");
		return EVENT_EEPROM_CHECK_FAILED;
	}
	
	g_sUpdateManager.u8AppIndex = u8JumpArea;
	
	return EVENT_WRITE_EEPROM_SUCCESS;
}


void cbRollbackProcess(void)
{
	uint32_t JumpAddress;
	uint32_t u32FirmwareAddress;
	pFunction JumpToApplication;
	
	// 跳转位置包含了bootloader, app1, app2
	if (g_sUpdateManager.u8AppIndex == 0)
	{
		u32FirmwareAddress = BOOTLOADER_ADDRESS;
	}
	if (g_sUpdateManager.u8AppIndex == 1)
	{
		u32FirmwareAddress = APPLICATION1_ADDRESS;
	}
	else if (g_sUpdateManager.u8AppIndex == 2)
	{
		u32FirmwareAddress = APPLICATION2_ADDRESS;
	}

	g_sIAP_ProcessManager.pSetIAP_Busy();
	DEBUG_INFO("prepare to rollback to area %d\r\n", g_sUpdateManager.u8AppIndex);
	g_sUpdateManager.bPrepareRollback = true;
	g_sUpdateManager.u32RollbackAddress = u32FirmwareAddress;	
	
//	JumpAddress = *(__IO uint32_t*) (u32FirmwareAddress + 4);
//	JumpToApplication = (pFunction) JumpAddress;
//	
//	__set_MSP(*(__IO uint32_t*) u32FirmwareAddress);
//	JumpToApplication();		
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



void UpdateProcessInit(void)
{
	// 获取应用程序标志地址
	#if defined (APPLICATION1)
		g_sUpdateManager.u8FirmwareFlagAddr = APP1_FLAG_ADDR;
		g_sUpdateManager.u8UpdateIndex = 2;
	#elif defined (APPLICATION2)
		g_sUpdateManager.u8FirmwareFlagAddr = APP2_FLAG_ADDR;
		g_sUpdateManager.u8UpdateIndex = 1;
	#endif
	
	g_sIAP_ProcessManager.SetFimwareInfo(TYPE_APPLICATION, 
	(uint32_t)((MAJOR_VERSION << 16) | 
				(MINOR_VERSION << 8) |
				PATCH_VERSION));
	g_sIAP_ProcessManager.SetUpdateIndex(g_sUpdateManager.u8UpdateIndex);	
	
	IAP_ManagerInit();
}


void UpdateProcess(void)
{
	if (g_sUpdateManager.pProcess)
	{
		g_sUpdateManager.pProcess();
	}
	
	// 运行IAP流程
	IAP_ManagerRun();
}

