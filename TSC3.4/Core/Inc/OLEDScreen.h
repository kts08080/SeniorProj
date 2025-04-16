/*
 * 	File:	OLEDScreen.h
 * 	Author:	James Eastman
 */

#ifndef INC_OLEDSCREEN_H_
#define INC_OLEDSCREEN_H_

// including
#include "main.h"
#include <stdint.h>
// End including


// Defines
// Fundamental Commands
#define SET_CONTRASTAT		0x81	// Double byte command second byte sets contrast
#define DISPLAY_OFF			0xAE	// Display off (sleep mode) (RESET)
#define DISPLAY_ON			0xAF	// Display on in normal mode
#define RAM_CONTENT_ON		0xA4	// Resume to RAM content display (RESET)
#define RAM_ONTENT_OFF		0xA5	// Output ignores RAM content
#define NORMAL_DISPLAY		0xA6	// Normal Display (RESET)
#define INVERSE_DISPLAY		0xA7	// Inverse Display

// Hardware Configuration
#define MULTIPLEX_RATIO		0xA8
#define COM_SCAN_NORMAL		0xC0
#define COM_SCAN_REMAPPED	0xC8
#define DISPLAY_OFFSET		0xD3
#define SET_COM_PINS		0xDA

// Timing & driving scheme setting
// All commands other than NOP are double byte
#define CLOCK_DIVIDE		0xD5
#define PRECHARGE_PERIOD	0xD9
#define VCOMH_DESELECT		0xDB
#define NOP					0xE3

// Other
#define OLED_WIDTH		128
#define OLED_HEIGHT		64
#define SET_LOW_COLUMN	0x00
#define SET_HIGH_COLUMN	0x10
#define SET_START_LINE	0x40
#define MEMORY_MODE		0x20
#define COLUMN_ADDRESS	0x21
#define PAGE_ADDRESS	0x22
#define SEG_REMAP		0xA0
#define EXTERNAL_VCC	0x01
#define SWITCH_CAP_VCC	0x02
#define CHARGE_PUMP		0x8D
// End defines

// Function definitions
void OLED_I2C_Init(I2C_HandleTypeDef hi2c_input, uint16_t address_input);
void OLED_sendCommand(uint8_t data);
void OLED_sendData(uint8_t data);
void OLED_setColAddress(uint16_t xCol);
void OLED_setPageAddress(uint16_t yPage);
void OLED_setLocation(uint16_t xCol, uint16_t yPage);
void OLED_printChar(char character);
void OLED_writeString(char *c);
void OLED_clearScreen(void);
void OLED_clearLine(int x);
// End function definitions


#endif /* INC_OLEDSCREEN_H_ */
