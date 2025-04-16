/*
 * EEPROM.c
 *
 *  Created on: Mar 26, 2025
 *      Author: dirtr
 */

#include "EEPROM.h"


#define READ_ADDR	0xA8
#define WRITE_ADDR	0xA9
#define ADDR_SIZE	(0x00000002U)
I2C_HandleTypeDef i2cHandle;

void EEPROM_init(I2C_HandleTypeDef i2c){
	i2cHandle = i2c;
}

uint8_t EEPROM_read(uint32_t addr, uint8_t *data){
	// Memory Write format:
	// 1: Handler
	// 2: Device Address
	// 3: Memory Address
	// 4: Memory Size
	// 5: Data
	// 6: Size
	// 7: Timeout
	return HAL_I2C_Mem_Read(&i2cHandle, READ_ADDR, addr, ADDR_SIZE, &data, 1, 100);
}
uint8_t EEPROM_write(uint32_t addr, uint8_t *data){
	// Memory Write format:
	// 1: Handler
	// 2: Device Address
	// 3: Memory Address
	// 4: Memory Size
	// 5: Data
	// 6: Size
	// 7: Timeout
	return HAL_I2C_Mem_Write(&i2cHandle, WRITE_ADDR, addr, ADDR_SIZE, &data, 1, 100);
}
