#ifndef CLRC663_H
#define CLRC663_H

#include "spi.h"


typedef enum
{
	STATE_IDLE = 0,
	STATE_BUSY,
	STATE_SUCCESS,
	STATE_FINISH,
	STATE_FAILED,
	STATE_TIMEOUT,
}ProcessState;



//void CLRC663_Init(void); 
//uint8_t ReadVersion(void);
//void SetType(const uint8_t u8Type);

void MifareClassicInit(void);
void MifareClassicProcess(void);

#endif
