/*
 * EEPROM.h
 *
 *  Created on: Mar 26, 2025
 *      Author: dirtr
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

void EEPROM_init(I2C_HandleTypeDef i2c);
uint8_t EEPROM_read(uint32_t addr, uint8_t *data);
uint8_t EEPROM_write(uint32_t addr, uint8_t *data);

#endif /* INC_EEPROM_H_ */
