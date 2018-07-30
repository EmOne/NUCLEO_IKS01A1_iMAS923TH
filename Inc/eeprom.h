/*
 * eeprom.h
 *
 *  Created on: Jul 15, 2018
 *      Author: anolp
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#define START_EEPROM_ADDR 0x08080000

void EEPROM_GetUserSetting(uint32_t addr, volatile uint32_t *buffer, uint32_t len);
void EEPROM_SetUserSetting(uint32_t addr, volatile uint32_t *buffer, uint32_t len);

#endif /* EEPROM_H_ */
