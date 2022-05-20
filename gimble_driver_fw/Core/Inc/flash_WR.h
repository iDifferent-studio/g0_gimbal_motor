#ifndef _FLASH_WR_H
#define _FLASH_WR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
	
#define FLASH_SECTOR30_START                      0x0800F000
#define FLASH_SECTOR30_END                        0x0800F7FF
#define FLASH_SECTOR31_START                      0x0800F800
#define FLASH_SECTOR31_END                        0x0800FFFF
 
#define FLASH_DATA_ADDR_MIN                         FLASH_SECTOR30_START
#define FLASH_DATA_ADDR_MAX                         FLASH_SECTOR31_END
 
#define FLASH_DATA_ADDR_BASE                        FLASH_DATA_ADDR_MIN
#define CALIBRATION_L_LR_VALUE_ADDR                 FLASH_DATA_ADDR_BASE+0
#define CALIBRATION_L_UD_VALUE_ADDR                 FLASH_DATA_ADDR_BASE+12
#define CALIBRATION_R_LR_VALUE_ADDR                 FLASH_DATA_ADDR_BASE+24
#define CALIBRATION_R_UD_VALUE_ADDR                 FLASH_DATA_ADDR_BASE+36
#define FLASH_WAITETIME                             1000
 
// bind addr长度为5字节  实际占用2个uint32
#define BIND_MSG_BASE                       FLASH_SECTOR15_START     
#define BIND_ADDR                           BIND_MSG_BASE+0

void STMFLASH_Read(uint32_t ReadAddr, uint64_t *pBuffer, uint32_t NumToRead);
void STMFLASH_Write(uint32_t WriteAddr, uint64_t *pBuffer, uint32_t NumToWrite);	

#ifdef __cplusplus
}
#endif

#endif
