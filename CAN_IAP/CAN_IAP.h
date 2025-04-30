#ifndef CAN_IAP_H
#define CAN_IAP_H

#include "can.h"

#define BOOTLOADER_ADDRESS				0x8000000

// APP1和APP2的地址
#define APPLICATION1_ADDRESS			0x8020000
#define APP1_SECTOR_NUM		3
#define APPLICATION2_ADDRESS			0x8080000
#define APP2_SECTOR_NUM		3

typedef enum
{
	TYPE_ERROR = 0,
	TYPE_BOOTLOADER,
	TYPE_APPLICATION,
}FirmwareType;


typedef enum
{
	IAP_IDLE = 0,
	IAP_COMM,
	IAP_PREUPDATE,
	IAP_UPDATE,
	IAP_IDLE_FORCE_BOOT,
}IAP_State;


// 固件产生的事件, 供IAP执行流程时使用
typedef enum
{
	EVENT_NONE = 0,
	EVENT_WRITE_EEPROM_SUCCESS,
	EVENT_WRITE_EEPROM_FAILED,
	EVENT_READ_EEPROM_SUCCESS,
	EVENT_READ_EEPROM_FAILED,
	EVENT_EEPROM_CHECK_FAILED,
}FirmwareEvent;


typedef struct
{
	void (*SetFimwareInfo)(const FirmwareType eType, const uint32_t u32FirmwareVersion);
	void (*SetUpdateIndex)(const uint8_t u8Index);
	IAP_State (*GetIAP_CurrentState)(void);
	void (*pSetIAP_Busy)(void);
}IAP_ProcessManagerStruct;


extern IAP_ProcessManagerStruct g_sIAP_ProcessManager;
void IAP_ManagerInit(void);
void IAP_ManagerRun(void);

// weak函数, 需要外部重写
FirmwareEvent cbRollbackCheck(void);
FirmwareEvent cbUpdateSuccessProcess(void);
void cbRollbackProcess(void);
FirmwareEvent cbWriteEEPROM(const uint16_t u16Address, const uint8_t u8Data);
FirmwareEvent cbReadEEPROM(const uint16_t u16Address, uint8_t *pData, const uint8_t u8DataLen);


#endif
