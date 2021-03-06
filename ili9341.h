/**
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********
  * @file      :     ili9341.h
  * @author    :     Luyao Han
  * @email     :     luyaohan1001@gmail.com
  * @brief     :     C library for ILITEK ili9341 TFTLCD Controller.
  * @date      :     05-07-2022
  * Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal use / non-commercial use only.
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********  */

/* Macro to prevent recursive inclusion ----------------------------------------------------------------------------------------*/
#ifndef __ILI9341_H
#define __ILI9341_H

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "colors.h"

#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

/* TFT Screen Dimension --------------------------------------------------------------------------------------------------------*/
#define TFT_PIXEL_H_LENGTH 240
#define TFT_PIXEL_V_WIDTH  320

static const unsigned char font[] = {
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x3E, 0x5B, 0x4F, 0x5B, 0x3E,
	0x3E, 0x6B, 0x4F, 0x6B, 0x3E,
	0x1C, 0x3E, 0x7C, 0x3E, 0x1C,
	0x18, 0x3C, 0x7E, 0x3C, 0x18,
	0x1C, 0x57, 0x7D, 0x57, 0x1C,
	0x1C, 0x5E, 0x7F, 0x5E, 0x1C,
	0x00, 0x18, 0x3C, 0x18, 0x00,
	0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
	0x00, 0x18, 0x24, 0x18, 0x00,
	0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
	0x30, 0x48, 0x3A, 0x06, 0x0E,
	0x26, 0x29, 0x79, 0x29, 0x26,
	0x40, 0x7F, 0x05, 0x05, 0x07,
	0x40, 0x7F, 0x05, 0x25, 0x3F,
	0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
	0x7F, 0x3E, 0x1C, 0x1C, 0x08,
	0x08, 0x1C, 0x1C, 0x3E, 0x7F,
	0x14, 0x22, 0x7F, 0x22, 0x14,
	0x5F, 0x5F, 0x00, 0x5F, 0x5F,
	0x06, 0x09, 0x7F, 0x01, 0x7F,
	0x00, 0x66, 0x89, 0x95, 0x6A,
	0x60, 0x60, 0x60, 0x60, 0x60,
	0x94, 0xA2, 0xFF, 0xA2, 0x94,
	0x08, 0x04, 0x7E, 0x04, 0x08,
	0x10, 0x20, 0x7E, 0x20, 0x10,
	0x08, 0x08, 0x2A, 0x1C, 0x08,
	0x08, 0x1C, 0x2A, 0x08, 0x08,
	0x1E, 0x10, 0x10, 0x10, 0x10,
	0x0C, 0x1E, 0x0C, 0x1E, 0x0C,
	0x30, 0x38, 0x3E, 0x38, 0x30,
	0x06, 0x0E, 0x3E, 0x0E, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x5F, 0x00, 0x00,
	0x00, 0x07, 0x00, 0x07, 0x00,
	0x14, 0x7F, 0x14, 0x7F, 0x14,
	0x24, 0x2A, 0x7F, 0x2A, 0x12,
	0x23, 0x13, 0x08, 0x64, 0x62,
	0x36, 0x49, 0x56, 0x20, 0x50,
	0x00, 0x08, 0x07, 0x03, 0x00,
	0x00, 0x1C, 0x22, 0x41, 0x00,
	0x00, 0x41, 0x22, 0x1C, 0x00,
	0x2A, 0x1C, 0x7F, 0x1C, 0x2A,
	0x08, 0x08, 0x3E, 0x08, 0x08,
	0x00, 0x80, 0x70, 0x30, 0x00,
	0x08, 0x08, 0x08, 0x08, 0x08,
	0x00, 0x00, 0x60, 0x60, 0x00,
	0x20, 0x10, 0x08, 0x04, 0x02,
	0x3E, 0x51, 0x49, 0x45, 0x3E,
	0x00, 0x42, 0x7F, 0x40, 0x00,
	0x72, 0x49, 0x49, 0x49, 0x46,
	0x21, 0x41, 0x49, 0x4D, 0x33,
	0x18, 0x14, 0x12, 0x7F, 0x10,
	0x27, 0x45, 0x45, 0x45, 0x39,
	0x3C, 0x4A, 0x49, 0x49, 0x31,
	0x41, 0x21, 0x11, 0x09, 0x07,
	0x36, 0x49, 0x49, 0x49, 0x36,
	0x46, 0x49, 0x49, 0x29, 0x1E,
	0x00, 0x00, 0x14, 0x00, 0x00,
	0x00, 0x40, 0x34, 0x00, 0x00,
	0x00, 0x08, 0x14, 0x22, 0x41,
	0x14, 0x14, 0x14, 0x14, 0x14,
	0x00, 0x41, 0x22, 0x14, 0x08,
	0x02, 0x01, 0x59, 0x09, 0x06,
	0x3E, 0x41, 0x5D, 0x59, 0x4E,
	0x7C, 0x12, 0x11, 0x12, 0x7C,
	0x7F, 0x49, 0x49, 0x49, 0x36,
	0x3E, 0x41, 0x41, 0x41, 0x22,
	0x7F, 0x41, 0x41, 0x41, 0x3E,
	0x7F, 0x49, 0x49, 0x49, 0x41,
	0x7F, 0x09, 0x09, 0x09, 0x01,
	0x3E, 0x41, 0x41, 0x51, 0x73,
	0x7F, 0x08, 0x08, 0x08, 0x7F,
	0x00, 0x41, 0x7F, 0x41, 0x00,
	0x20, 0x40, 0x41, 0x3F, 0x01,
	0x7F, 0x08, 0x14, 0x22, 0x41,
	0x7F, 0x40, 0x40, 0x40, 0x40,
	0x7F, 0x02, 0x1C, 0x02, 0x7F,
	0x7F, 0x04, 0x08, 0x10, 0x7F,
	0x3E, 0x41, 0x41, 0x41, 0x3E,
	0x7F, 0x09, 0x09, 0x09, 0x06,
	0x3E, 0x41, 0x51, 0x21, 0x5E,
	0x7F, 0x09, 0x19, 0x29, 0x46,
	0x26, 0x49, 0x49, 0x49, 0x32,
	0x03, 0x01, 0x7F, 0x01, 0x03,
	0x3F, 0x40, 0x40, 0x40, 0x3F,
	0x1F, 0x20, 0x40, 0x20, 0x1F,
	0x3F, 0x40, 0x38, 0x40, 0x3F,
	0x63, 0x14, 0x08, 0x14, 0x63,
	0x03, 0x04, 0x78, 0x04, 0x03,
	0x61, 0x59, 0x49, 0x4D, 0x43,
	0x00, 0x7F, 0x41, 0x41, 0x41,
	0x02, 0x04, 0x08, 0x10, 0x20,
	0x00, 0x41, 0x41, 0x41, 0x7F,
	0x04, 0x02, 0x01, 0x02, 0x04,
	0x40, 0x40, 0x40, 0x40, 0x40,
	0x00, 0x03, 0x07, 0x08, 0x00,
	0x20, 0x54, 0x54, 0x78, 0x40,
	0x7F, 0x28, 0x44, 0x44, 0x38,
	0x38, 0x44, 0x44, 0x44, 0x28,
	0x38, 0x44, 0x44, 0x28, 0x7F,
	0x38, 0x54, 0x54, 0x54, 0x18,
	0x00, 0x08, 0x7E, 0x09, 0x02,
	0x18, 0xA4, 0xA4, 0x9C, 0x78,
	0x7F, 0x08, 0x04, 0x04, 0x78,
	0x00, 0x44, 0x7D, 0x40, 0x00,
	0x20, 0x40, 0x40, 0x3D, 0x00,
	0x7F, 0x10, 0x28, 0x44, 0x00,
	0x00, 0x41, 0x7F, 0x40, 0x00,
	0x7C, 0x04, 0x78, 0x04, 0x78,
	0x7C, 0x08, 0x04, 0x04, 0x78,
	0x38, 0x44, 0x44, 0x44, 0x38,
	0xFC, 0x18, 0x24, 0x24, 0x18,
	0x18, 0x24, 0x24, 0x18, 0xFC,
	0x7C, 0x08, 0x04, 0x04, 0x08,
	0x48, 0x54, 0x54, 0x54, 0x24,
	0x04, 0x04, 0x3F, 0x44, 0x24,
	0x3C, 0x40, 0x40, 0x20, 0x7C,
	0x1C, 0x20, 0x40, 0x20, 0x1C,
	0x3C, 0x40, 0x30, 0x40, 0x3C,
	0x44, 0x28, 0x10, 0x28, 0x44,
	0x4C, 0x90, 0x90, 0x90, 0x7C,
	0x44, 0x64, 0x54, 0x4C, 0x44,
	0x00, 0x08, 0x36, 0x41, 0x00,
	0x00, 0x00, 0x77, 0x00, 0x00,
	0x00, 0x41, 0x36, 0x08, 0x00,
	0x02, 0x01, 0x02, 0x04, 0x02,
	0x3C, 0x26, 0x23, 0x26, 0x3C,
	0x1E, 0xA1, 0xA1, 0x61, 0x12,
	0x3A, 0x40, 0x40, 0x20, 0x7A,
	0x38, 0x54, 0x54, 0x55, 0x59,
	0x21, 0x55, 0x55, 0x79, 0x41,
	0x22, 0x54, 0x54, 0x78, 0x42, 
	0x21, 0x55, 0x54, 0x78, 0x40,
	0x20, 0x54, 0x55, 0x79, 0x40,
	0x0C, 0x1E, 0x52, 0x72, 0x12,
	0x39, 0x55, 0x55, 0x55, 0x59,
	0x39, 0x54, 0x54, 0x54, 0x59,
	0x39, 0x55, 0x54, 0x54, 0x58,
	0x00, 0x00, 0x45, 0x7C, 0x41,
	0x00, 0x02, 0x45, 0x7D, 0x42,
	0x00, 0x01, 0x45, 0x7C, 0x40,
	0x7D, 0x12, 0x11, 0x12, 0x7D,
	0xF0, 0x28, 0x25, 0x28, 0xF0,
	0x7C, 0x54, 0x55, 0x45, 0x00,
	0x20, 0x54, 0x54, 0x7C, 0x54,
	0x7C, 0x0A, 0x09, 0x7F, 0x49,
	0x32, 0x49, 0x49, 0x49, 0x32,
	0x3A, 0x44, 0x44, 0x44, 0x3A, 
	0x32, 0x4A, 0x48, 0x48, 0x30,
	0x3A, 0x41, 0x41, 0x21, 0x7A,
	0x3A, 0x42, 0x40, 0x20, 0x78,
	0x00, 0x9D, 0xA0, 0xA0, 0x7D,
	0x3D, 0x42, 0x42, 0x42, 0x3D, 
	0x3D, 0x40, 0x40, 0x40, 0x3D,
	0x3C, 0x24, 0xFF, 0x24, 0x24,
	0x48, 0x7E, 0x49, 0x43, 0x66,
	0x2B, 0x2F, 0xFC, 0x2F, 0x2B,
	0xFF, 0x09, 0x29, 0xF6, 0x20,
	0xC0, 0x88, 0x7E, 0x09, 0x03,
	0x20, 0x54, 0x54, 0x79, 0x41,
	0x00, 0x00, 0x44, 0x7D, 0x41,
	0x30, 0x48, 0x48, 0x4A, 0x32,
	0x38, 0x40, 0x40, 0x22, 0x7A,
	0x00, 0x7A, 0x0A, 0x0A, 0x72,
	0x7D, 0x0D, 0x19, 0x31, 0x7D,
	0x26, 0x29, 0x29, 0x2F, 0x28,
	0x26, 0x29, 0x29, 0x29, 0x26,
	0x30, 0x48, 0x4D, 0x40, 0x20,
	0x38, 0x08, 0x08, 0x08, 0x08,
	0x08, 0x08, 0x08, 0x08, 0x38,
	0x2F, 0x10, 0xC8, 0xAC, 0xBA,
	0x2F, 0x10, 0x28, 0x34, 0xFA,
	0x00, 0x00, 0x7B, 0x00, 0x00,
	0x08, 0x14, 0x2A, 0x14, 0x22,
	0x22, 0x14, 0x2A, 0x14, 0x08,
	0xAA, 0x00, 0x55, 0x00, 0xAA,
	0xAA, 0x55, 0xAA, 0x55, 0xAA,
	0x00, 0x00, 0x00, 0xFF, 0x00,
	0x10, 0x10, 0x10, 0xFF, 0x00,
	0x14, 0x14, 0x14, 0xFF, 0x00,
	0x10, 0x10, 0xFF, 0x00, 0xFF,
	0x10, 0x10, 0xF0, 0x10, 0xF0,
	0x14, 0x14, 0x14, 0xFC, 0x00,
	0x14, 0x14, 0xF7, 0x00, 0xFF,
	0x00, 0x00, 0xFF, 0x00, 0xFF,
	0x14, 0x14, 0xF4, 0x04, 0xFC,
	0x14, 0x14, 0x17, 0x10, 0x1F,
	0x10, 0x10, 0x1F, 0x10, 0x1F,
	0x14, 0x14, 0x14, 0x1F, 0x00,
	0x10, 0x10, 0x10, 0xF0, 0x00,
	0x00, 0x00, 0x00, 0x1F, 0x10,
	0x10, 0x10, 0x10, 0x1F, 0x10,
	0x10, 0x10, 0x10, 0xF0, 0x10,
	0x00, 0x00, 0x00, 0xFF, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0xFF, 0x10,
	0x00, 0x00, 0x00, 0xFF, 0x14,
	0x00, 0x00, 0xFF, 0x00, 0xFF,
	0x00, 0x00, 0x1F, 0x10, 0x17,
	0x00, 0x00, 0xFC, 0x04, 0xF4,
	0x14, 0x14, 0x17, 0x10, 0x17,
	0x14, 0x14, 0xF4, 0x04, 0xF4,
	0x00, 0x00, 0xFF, 0x00, 0xF7,
	0x14, 0x14, 0x14, 0x14, 0x14,
	0x14, 0x14, 0xF7, 0x00, 0xF7,
	0x14, 0x14, 0x14, 0x17, 0x14,
	0x10, 0x10, 0x1F, 0x10, 0x1F,
	0x14, 0x14, 0x14, 0xF4, 0x14,
	0x10, 0x10, 0xF0, 0x10, 0xF0,
	0x00, 0x00, 0x1F, 0x10, 0x1F,
	0x00, 0x00, 0x00, 0x1F, 0x14,
	0x00, 0x00, 0x00, 0xFC, 0x14,
	0x00, 0x00, 0xF0, 0x10, 0xF0,
	0x10, 0x10, 0xFF, 0x10, 0xFF,
	0x14, 0x14, 0x14, 0xFF, 0x14,
	0x10, 0x10, 0x10, 0x1F, 0x00,
	0x00, 0x00, 0x00, 0xF0, 0x10,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0xFF,
	0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
	0x38, 0x44, 0x44, 0x38, 0x44,
	0xFC, 0x4A, 0x4A, 0x4A, 0x34, 
	0x7E, 0x02, 0x02, 0x06, 0x06,
	0x02, 0x7E, 0x02, 0x7E, 0x02,
	0x63, 0x55, 0x49, 0x41, 0x63,
	0x38, 0x44, 0x44, 0x3C, 0x04,
	0x40, 0x7E, 0x20, 0x1E, 0x20,
	0x06, 0x02, 0x7E, 0x02, 0x02,
	0x99, 0xA5, 0xE7, 0xA5, 0x99,
	0x1C, 0x2A, 0x49, 0x2A, 0x1C,
	0x4C, 0x72, 0x01, 0x72, 0x4C,
	0x30, 0x4A, 0x4D, 0x4D, 0x30,
	0x30, 0x48, 0x78, 0x48, 0x30,
	0xBC, 0x62, 0x5A, 0x46, 0x3D,
	0x3E, 0x49, 0x49, 0x49, 0x00,
	0x7E, 0x01, 0x01, 0x01, 0x7E,
	0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
	0x44, 0x44, 0x5F, 0x44, 0x44,
	0x40, 0x51, 0x4A, 0x44, 0x40,
	0x40, 0x44, 0x4A, 0x51, 0x40,
	0x00, 0x00, 0xFF, 0x01, 0x03,
	0xE0, 0x80, 0xFF, 0x00, 0x00,
	0x08, 0x08, 0x6B, 0x6B, 0x08,
	0x36, 0x12, 0x36, 0x24, 0x36,
	0x06, 0x0F, 0x09, 0x0F, 0x06,
	0x00, 0x00, 0x18, 0x18, 0x00,
	0x00, 0x00, 0x10, 0x10, 0x00,
	0x30, 0x40, 0xFF, 0x01, 0x01,
	0x00, 0x1F, 0x01, 0x01, 0x1E,
	0x00, 0x19, 0x1D, 0x17, 0x12,
	0x00, 0x3C, 0x3C, 0x3C, 0x3C,
	0x00, 0x00, 0x00, 0x00, 0x00
};

/* @brief utility used for printing debug message through serial UART. */
extern void serial_print(char* message);


/* Function Prototypes -------------------------------------------------------------------------------------------------------*/
void LCD_RST_1();
void LCD_RST_0();

void LCD_CS_1();
void LCD_CS_0();

void LCD_RS_1();
void LCD_RS_0();

void LCD_WR_1();
void LCD_WR_0();
void LCD_WR_RISING_EDGE();

void LCD_RD_1();
void LCD_RD_0();

/* GPIO Layer 8080 Bus ------------------------------------------------------------------------------------------------------*/
void gpio_configure_8080_datapins_input_mode();
void gpio_configure_8080_datapins_output_mode();
uint8_t bus_8080_read_parallel_datapins();
void bus_8080_write_parallel_datapins(uint8_t write_data);

void bus_8080_write_command(uint8_t cmd);
void bus_8080_write_data(uint8_t data);
void bus_8080_read_data(uint8_t* p_read_data, uint8_t length);

  
void bus_8080_write_register(uint8_t target_register, uint8_t length, uint8_t* p_param);
void bus_8080_read_register(uint8_t target_register, uint8_t length, uint8_t* p_read_data);



void ili9341_hard_reset();

/* ILI9341 Regulative Registers Access -------------------------------------------------------------------------------------------*/
/* 8.2.1. NOP (00h) */
void ili9341_nop();
/* 8.2.2. Software Reset (01h) */ 
void ili9341_soft_reset();
/* 8.2.3. Read display identification information (04h) */ 
void ili9341_get_display_identification(uint8_t* p_read_data);
/* 8.2.4. Read Display Status (09h) */ 
void ili9341_get_display_status(uint8_t* p_read_data);
/* 8.2.5. Read Display Power Mode (0Ah) */ 
void ili9341_get_display_power_mode(uint8_t* p_read_data);
/* 8.2.6. Read Display MADCTL (0Bh) */ 
void ili9341_get_display_MADCTL(uint8_t* p_read_data);
/* 8.2.7. Read Display Pixel Format (0Ch) */ 
void ili9341_get_display_pixel_format(uint8_t* p_read_data);
/* 8.2.8. Read Display Image Format (0Dh) */ 
void ili9341_get_display_image_format(uint8_t* p_read_data);
/* 8.2.9. Read Display Signal Mode (0Eh) */ 
void ili9341_get_display_signal_mode(uint8_t* p_read_data);
/* 8.2.10. Read Display Self-Diagnostic Result (0Fh) */ 
void ili9341_get_display_diagnostic_result(uint8_t* p_read_data);

/* 8.2.11. Enter Sleep Mode (10h) */ 
void ili9341_enter_sleep_mode();
/* 8.2.12. Sleep Out (11h) */ 
void ili9341_exit_sleep_mode();
/* 8.2.13. Partial Mode ON (12h)*/ 
void ili9341_set_partial_mode_on();
/* 8.2.14. Normal Display Mode ON (13h)*/ 
void ili9341_normal_mode_on();
/* 8.2.15. Display Inversion OFF (20h)*/ 
void ili9341_set_display_inversion_off();
/* 8.2.16. Display Inversion ON (21h) */
void ili9341_set_display_inversion_on();
/* 8.2.17. Gamma Set (26h) */ 
void ili9341_set_gamma();
/* 8.2.18. Display OFF (28h) */ 
void ili9341_set_display_off();
/* 8.2.19. Display ON (29h) */ 
void ili9341_set_display_on();
/* 8.2.20. Column Address Set (2Ah) */ 
void ili9341_set_column_address(uint16_t x1, uint16_t x2);
/* 8.2.21. Page Address Set (2Bh) */ 
void ili9341_set_page_address(uint16_t y1, uint16_t y2);
/* 8.2.22. Memory Write (2Ch) */ 
void ili9341_memory_write();
/* 8.2.23. Color Set (2Dh) */ 
void ili9341_set_color();
/* 8.2.24. Memory Read (2Eh) */ 
void ili9341_memory_read();
/* 8.2.25. Partial Area (30h) */ 
void ili9341_set_partial_area();
/* 8.2.26. Vertical Scrolling Definition (33h) */ 
void ili9341_set_vertical_scrolling_definition(uint16_t top_fixed_area_height, uint16_t vertical_scrolling_area_height, uint16_t bottom_fixed_area_height);
/* 8.2.27. Tearing Effect Line OFF (34h) */ 
void ili9341_set_tearing_effect_line_off();
/* 8.2.28. Tearing Effect Line ON (35h) */ 
void ili9341_set_tearing_effect_line_on();
/* 8.2.29. Memory Access Control (36h) */
void ili9341_set_memory_access_control(uint8_t setval);
/* 8.2.30. Vertical Scrolling Start Address (37h) */
void ili9341_vertical_scrolling_start_address(uint16_t VSP);
/* 8.2.31. Idle Mode OFF (38h) */
void ili9341_set_idle_mode_off();
/* 8.2.32. Idle Mode ON (39h) */
void ili9341_set_idle_mode_on();
/* 8.2.33. COLMOD: Pixel Format Set (3Ah) */
void ili9341_set_pixel_format_set();
/* 8.2.34. Write_Memory_Continue (3Ch) */
void ili9341_write_memory_continue();
/* 8.2.35. Read_Memory_Continue (3Eh)*/ 
void ili9341_read_memory_continue();
/* 8.2.36. Set_Tear_Scanline (44h)*/ 
void ili9341_set_tear_scanline();
/* 8.2.37. Get_Scanline (45h) */ 
void ili9341_get_scanline();
/* 8.2.38. Write Display Brightness (51h) */ 
void ili9341_set_display_brightness();
/* 8.2.39. Read Display Brightness (52h) */ 
void ili9341_get_display_brightness(uint8_t* p_read_data);
/* 8.2.40. Write CTRL Display (53h) */ 
void ili9341_set_CTRL_display();
/* 8.2.41. Read CTRL Display (54h) */ 
void ili9341_get_CTRL_display(uint8_t* p_read_data);
/* 8.2.42. Write Content Adaptive Brightness Control (55h) */ 
void ili9341_set_content_adaptive_brightness_control();
/* 8.2.43. Read Content Adaptive Brightness Control (56h) */ 
void ili9341_get_content_adaptive_brightness_control();
/* 8.2.44. Write CABC Minimum Brightness (5Eh) */ 
void ili9341_set_CABC_minimum_brightness();
/* 8.2.45. Read CABC Minimum Brightness (5Fh) */ 
void ili9341_get_CABC_minimum_brightness();
/* 8.2.46. Read ID1 (DAh) */ 
void ili9341_get_id1(uint8_t* p_read_data);
/* 8.2.47. Read ID2 (DBh) */ 
void ili9341_get_id2(uint8_t* p_read_data);
/* 8.2.48. Read ID3 (DCh) */ 
void ili9341_get_id3(uint8_t* p_read_data);


/* ILI9341 Extended Registers Access -------------------------------------------------------------------------------------------*/
/* 8.3.1. RGB Interface Signal Control (B0h) */
void ili9341_rgb_interface_signal_control();
/* 8.3.2. Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
void ili9341_frame_control_normal_mode();
/* 8.3.3. Frame Rate Control (In Idle Mode/8 colors) (B2h) */
void ili9341_frame_control_idle_mode();
/* 8.3.4. Frame Rate control (In Partial Mode/Full Colors) (B3h) */
void ili9341_frame_control_partial_mode();
/* 8.3.5. Display Inversion Control (B4h) */
void ili9341_display_inversion_control();
/* 8.3.6. Blanking Porch Control (B5h) */
void ili9341_blanking_porch_control();
/* 8.3.7. Display Function Control (B6h) */
void ili9341_display_function_control();
/* 8.3.8. Entry Mode Set (B7h) */
void ili9341_entry_mode_set();
/* 8.3.9. Backlight Control 1 (B8h) */
void ili9341_backlight_control_1();
/* 8.3.10. Backlight Control 2 (B9h) */
void ili9341_backlight_control_2();
/* 8.3.11. Backlight Control 3 (BAh) */
void ili9341_backlight_control_3();
/* 8.3.12. Backlight Control 4 (BBh) */
void ili9341_backlight_control_4();
/* 8.3.13. Backlight Control 5 (BCh) */
void ili9341_backlight_control_5();
/* 8.3.14. Backlight Control 7 (BEh) */
void ili9341_backlight_control_7();
/* 8.3.15. Backlight Control 8 (BFh)*/
void ili9341_backlight_control_8();
/* 8.3.16. Power Control 1 (C0h) */ 
void ili9341_set_power_control1();
/* 8.3.17. Power Control 2 (C1h) */ 
void ili9341_set_power_control2();
/* 8.3.18. VCOM Control 1(C5h) */ 
void ili9341_set_vcom_control1();
/* 8.3.19. VCOM Control 2(C7h) */ 
void ili9341_set_vcom_control2();

/* 8.3.20. NV Memory Write (D0h) */ 
void ili9341_nv_memory_write();
/* 8.3.21. NV Memory Protection Key (D1h) */
void ili9341_nv_memory_protection_key();
/* 8.3.22. NV Memory Status Read (D2h) */
void ili9341_nv_memory_status_read();
/* 8.3.23. Read ID4 (D3h) */
void ili9341_get_id4(uint8_t* p_read_data);
/* 8.3.24. Positive Gamma Correction (E0h) */
void ili9341_positive_gamma_correction();
/* 8.3.25. Negative Gamma Correction (E1h) */
void ili9341_negative_gamma_correction();
/* 8.3.26. Digital Gamma Control 1 (E2h) */
void ili9341_digital_gamma_control_1();
/* 8.3.27. Digital Gamma Control 2(E3h) */
void ili9341_digital_gamma_control_2();
/* 8.3.28. Interface Control (F6h)  */
void ili9341_interface_control();

/* 8.4.1 Power control A (CBh) */
void ili9341_set_power_control_a();
/* 8.4.2 Power control B (CFh) */ 
void ili9341_set_power_control_b();
/* 8.4.3 Driver timing control A (E8h) */ 
void ili9341_set_driver_timing_control_a();
/* 8.4.5 Driver timing control B (EAh) */ 
void ili9341_set_driver_timing_control_b();
/* 8.4.6 Power on sequence control (EDh) */ 
void ili9341_set_poweron_sequence_control();
/* 8.4.7 Enable 3G (F2h) */
void ili9341_enable_3G();





/* Abstraction Layer: LCD Drawing / Plotting Methods ------------------------------------------------------------*/
void ili9341_init();
void ili9341_set_frame_address(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

void lcd_draw_horizontal_line(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t length, uint16_t color);
void lcd_draw_vertical_line(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t length, uint16_t line_color);
void lcd_draw_rectangle_unfilled(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t rect_width, uint16_t rect_height, uint16_t frame_color);
void lcd_draw_rectangle_filled(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t rect_width, uint16_t rect_height, uint16_t rect_color);
void lcd_clear_all(uint16_t fill_color);
void lcd_plot_char(int16_t x, int16_t y, unsigned char c,uint16_t color, uint16_t bg, uint8_t size);
void lcd_draw_dot(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t dot_color);
void lcd_set_rotation(uint8_t orientation);
void lcd_write_message(char* message, uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint8_t size, uint16_t text_color, uint16_t text_bg_color);
void lcd_enter_vertical_scroll_mode();
void lcd_continuous_scroll();


#endif





