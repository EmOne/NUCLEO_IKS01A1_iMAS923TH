/*
 * eeprom.c
 *
 *  Created on: Jul 15, 2018
 *      Author: anolp
 */
#include "stm32l0xx_hal.h"
#include "eeprom.h"

HAL_StatusTypeDef writeEEPROMByte(uint32_t address, uint8_t data)
 {
    HAL_StatusTypeDef  status;
    address = address + START_EEPROM_ADDR;
    HAL_FLASHEx_DATAEEPROM_Unlock();  //Unprotect the EEPROM to allow writing
    status = HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAMDATA_BYTE, address, data);
    HAL_FLASHEx_DATAEEPROM_Lock();  // Reprotect the EEPROM
    return status;
}

uint8_t readEEPROMByte(uint32_t address) {
    uint8_t tmp = 0;
    address = address + START_EEPROM_ADDR;
    tmp = *(__IO uint32_t*)address;

    return tmp;
}

void EEPROM_GetUserSetting(uint32_t addr, volatile uint32_t *buffer, uint32_t len)
{
	uint8_t i;

	for (i = 0; i < len; i += 4)
		*buffer++ = readEEPROMByte(addr + i);
}

void EEPROM_SetUserSetting(uint32_t addr, volatile uint32_t *buffer, uint32_t len)
{
	uint8_t i;
	volatile uint32_t data;

//	HAL_FLASHEx_DATAEEPROM_Unlock();
	for (i = 0; i < len; i += 4) {
		data = *buffer++;
		if (data != readEEPROMByte(addr + i))
			writeEEPROMByte(addr + i, data);
	}
//	HAL_FLASHEx_DATAEEPROM_Lock();
}
