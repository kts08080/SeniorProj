/*
 * z_i2c_eeprom.h
 *
 *  Created on: 7 mar 2022
 *
 */

#ifndef INC_Z_I2C_EEPROM_H_
#define INC_Z_I2C_EEPROM_H_

#include "main.h"
#include <strings.h>

// Define STM32 I2C port connecting to EEPROM chip
#define EEPROM_PORT             hi2c1  // Using I2C1 port

// EEPROM I2C Address (7-bit addressing)
#define EEPROM_I2C_ADDR         (0xA8)  // EEPROM address (this might need to be shifted for 7-bit mode
//#define EEPROM_I2C_ADDR			(0xA8>> 1)  // Convert to 7-bit format if needed
//#define EEPROM_I2C_ADDR (0x50)  // Correct 7-bit address

// EEPROM Specifications
#define EE_PAGE_LEN 32  // Correct for 24AA32AF
#define UPPER_LIMIT_EEADDR      0x7FFF  // (24C256) 32 kBytes = 256 kbits


// HAL API parameters
#define EE_AVAIL_TRIALS         20  // Number of attempts testing EEPROM availability
#define EE_AVAIL_TIMEOUT        HAL_MAX_DELAY  // Timeout for EEPROM availability check
#define I2C_COMM_TIMEOUT        20  // Timeout for I2C communication (consider data length and speed)

// EEPROM memory addresses used in the project
#define PARAM_EEADDR            0x0000  // Save test string at EEPROM beginning

// Function Prototypes
void test_EEPROM();
uint8_t read_p_EEPROM(uint16_t addr, char *data);
uint8_t read_n_EEPROM(uint16_t addr, char *data, uint8_t size);
uint8_t write_s_EEPROM(uint16_t addr, char *data);
uint8_t write_n_EEPROM(uint16_t addr, char *data, uint8_t size);

#endif /* INC_Z_I2C_EEPROM_H_ */
