/**
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********
  * @file      :     ili9341.c
  * @author    :     Luyao Han
  * @email     :     luyaohan1001@gmail.com
  * @brief     :     C library for ILITEK ili9341 TFTLCD Controller.
  * @date      :     05-07-2022
  * Copyright (C) 2022-2122 Luyao Han. The following code may be shared or modified for personal use / non-commercial use only.
  ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ******** ********  */

/* Includes ---------------------------------------------------------------------------------------------------------------------------------------*/
#include "ili9341.h"

/* GPIO physical layer ----------------------------------------------------------------------------------------------------------------------------*/
/* GPIO Defined on STM32F401 */
/* Module Pin Name         Description                Assigned GPIO */
/* LCD_RST                 Reset                      PC1 */
/* LCD_CS                  8080 Chip Select           PB0 */
/* LCD_RS (D/CX)           Data / Command Select      PA4 */
/* LCD_WR                  Write Enable               PA1 */
/* LCD_RD                  Read Enable                PA0 */
/* LCD_D2                  8080 Data 2                PA10*/
/* LCD_D3                  8080 Data 3                PB3*/
/* LCD_D4                  8080 Data 4                PB5*/
/* LCD_D5                  8080 Data 5                PB4*/
/* LCD_D6                  8080 Data 6                PB10*/
/* LCD_D7                  8080 Data 7                PA8*/
/* LCD_D0                  8080 Data 0                PA9*/
/* LCD_D1                  8080 Data 1                PC7*/

/**
  * @brief   Set TFT LCD module RST (Reset) high.
  * @param   None.
  * @retval  None.
  */
void LCD_RST_1()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);  
}

/**
  * @brief   Set TFT LCD module RST (Reset) low.
  * @param   None.
  * @retval  None.
  */
void LCD_RST_0()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);  
}
 

/**
  * @brief   Set TFT LCD module CS (Chip Select) high.
  * @param   None.
  * @retval  None.
  */
void LCD_CS_1()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  
}

/**
  * @brief   Set TFT LCD module CS (Chip Select) low.
  * @param   None.
  * @retval  None.
  */
void LCD_CS_0()
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  
}

 
/**
  * @brief   Set TFT LCD module RS (Register Select) high.
  * @param   None.
  * @retval  None.
  * @note    RS pin is equvilent to D/C or DCX pin. 
  *          RS = 1 indicates D7-D0 sends data.
  *          RS = 0 indicates D7-D0 sends command.
  */
void LCD_RS_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  
}

/**
  * @brief   Set TFT LCD module RS (Register Select) low.
  * @param   None.
  * @retval  None.
  * @note    RS pin is equvilent to D/C or DCX pin. 
  *          RS = 1 indicates D7-D0 sends data.
  *          RS = 0 indicates D7-D0 sends command.
  */
void LCD_RS_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  
}

/**
  * @brief   Set TFT LCD module WR(Write) pin high.
  * @param   None.
  * @retval  None.
  */
void LCD_WR_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  
}

/**
  * @brief   Set TFT LCD module WR(Write) pin low.
  * @param   None.
  * @retval  None.
  */
void LCD_WR_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  
}

/**
  * @brief   Set rising edge on TFT LCD module WR(Write) pin.
  * @param   None.
  * @retval  None.
  */
void LCD_WR_RISING_EDGE()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  
}


/**
  * @brief   Set TFT LCD module RD (Read) high.
  * @param   None.
  * @retval  None.
  */
void LCD_RD_1()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  
}

/**
  * @brief   Set TFT LCD module RD (Read) low.
  * @param   None.
  * @retval  None.
  */
void LCD_RD_0()
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  
}

/**
  * @brief   Configure GPIO input mode for reading 8080 data pins D7-D0.
  * @param   None.
  * @retval  None.
  */
void gpio_configure_8080_datapins_input_mode()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief   Configure GPIO output mode for reading 8080 data pins D7-D0.
  * @param   None.
  * @retval  None.
  */
void gpio_configure_8080_datapins_output_mode()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief   Read 8080 data pins D7-D0 pins on GPIO.
  * @param   None.
  * @retval  Single byte concatenating pin level from D7-D0.
  */
uint8_t bus_8080_read_parallel_datapins() 
{
  uint8_t read_data;
  read_data = \
                HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 7   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) << 6   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  << 5   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  << 4   \
              | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)  << 3   \
              | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) << 2   \
              | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)  << 1   \
              | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
  return read_data;
}


/**
  * @brief   Reading 8080 data D7-D0 pins on GPIO.
  * @param   write_data Data written to 8080 bus D7-D0.
  * @retval  None.
  */
void bus_8080_write_parallel_datapins(uint8_t write_data) 
{
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  (write_data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (write_data & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET); 
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  (write_data & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);  
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  (write_data & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET); 
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  (write_data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET); 
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, (write_data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  (write_data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  (write_data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  GPIOA->ODR &= ~((1 << 8) | (1 << 10) | (1 << 9));
  GPIOA->ODR |= ( ((write_data & 0x80) << 1) | ((write_data & 0x04) << 8) | ( (write_data & 0x01) << 9));

  GPIOB->ODR &= ~((1 << 10) | (1 << 4) | (1 << 5) | (1 << 3));
  GPIOB->ODR |= ( ((write_data & 0x40) << 4) | ((write_data & 0x20) >> 1) | ( (write_data & 0x10) << 1) | ((write_data & 0x08) << 0));

  GPIOC->ODR &= ~(1 << 7);
  GPIOC->ODR |= ((write_data & 0x02) << 6);
}

/**
  *
  *
  */
void bus_8080_write_register(uint8_t target_register, uint8_t* p_param, uint8_t length)
{
  /* Chip Select */
  LCD_CS_0();

  LCD_RS_0();  
  LCD_RD_1();
  /* Write data to data bus D7-D0. */  
  gpio_configure_8080_datapins_output_mode();
  bus_8080_write_parallel_datapins(target_register);
  LCD_WR_RISING_EDGE();

  /* De-select Chip. */
  for (int i = 0; i < length; ++i)
  {
    LCD_RS_1();  
    /* LCD_RD_1(); */
    /* gpio_configure_8080_datapins_output_mode(); */
    bus_8080_write_parallel_datapins(p_param[i]);
    LCD_WR_RISING_EDGE();
    
  }
  LCD_CS_1();
}

void bus_8080_write_command(uint8_t cmd)
{
  /* Chip Select */
  LCD_CS_0();
  LCD_RS_0();  
  LCD_RD_1();

  /* Write data to data bus D7-D0. */  
  gpio_configure_8080_datapins_output_mode();
  bus_8080_write_parallel_datapins(cmd);
  LCD_WR_RISING_EDGE();

  /* De-select Chip. */
  LCD_CS_1();
}

void bus_8080_write_data(uint8_t data) 
{
  LCD_CS_0();
  LCD_RS_1();  
  LCD_RD_1();

  gpio_configure_8080_datapins_output_mode();
  bus_8080_write_parallel_datapins(data);
  LCD_WR_RISING_EDGE();
  LCD_CS_1();
}

/**
  * @brief   8080 Bus Read Data.
  * @param   None.
  * @retval  None.
  */
void bus_8080_read_data(uint8_t* p_read_data, uint8_t length) 
{
  LCD_CS_0();
  LCD_RS_1();  
  LCD_WR_1();

  gpio_configure_8080_datapins_input_mode();
  for (int i = 0; i < length; ++i)
  {
    LCD_RD_0();
    p_read_data[i] = bus_8080_read_parallel_datapins();
    LCD_RD_1();
  }

  LCD_CS_1();
}




/** 
  * @brief Set Power Control A. 
  */
void ili9341_set_power_control_a()
{
  /* Vcore = 1.6V DDVDH = 5.6V. */
  uint8_t p_param[5] = {0x39, 0x2C, 0x00, 0x34, 0x02};
  bus_8080_write_register(0xCB, p_param, 5);
}

void ili9341_set_power_control_b()
{
  uint8_t p_param[3] = {0x00, 0xC1, 0x30};
  bus_8080_write_register(0xCF, p_param, 3);
}


void ili9341_set_driver_timing_control_a()
{
  uint8_t p_param[3] = {0x85, 0x00, 0x78};
  bus_8080_write_register(0xE8, p_param, 3);
}

void ili9341_set_driver_timing_control_b()
{

  uint8_t p_param[2] = {0x00, 0x00};
  bus_8080_write_register(0xEA, p_param, 2);
}

void ili9341_set_poweron_sequence_control_b()
{
  uint8_t p_param[4] = {0x64, 0x03, 0x12, 0x81};
  bus_8080_write_register(0xED, p_param, 4);
}

/**/
void ili9341_set_pump_ratio_control()
{
  uint8_t p_param[1] = {0x20};
  bus_8080_write_register(0xF7, p_param, 1);
}

void ili9341_set_power_control1()
{
  uint8_t p_param[1] = {0x23};
  bus_8080_write_register(0xC0, p_param, 1);
}

void ili9341_set_power_control2()
{
  uint8_t p_param[1] = {0x10};
  bus_8080_write_register(0xC1, p_param, 1);
}


void ili9341_set_vcom_control1()
{
  uint8_t p_param[2] = {0x3E, 0x28};
  bus_8080_write_register(0xC5, p_param, 2);
}

void ili9341_set_vcom_control2()
{
  uint8_t p_param[1] = {0x86};
  bus_8080_write_register(0xC7, p_param, 1);
}

void ili9341_set_memory_access_control()
{
  uint8_t p_param[1] = {0x08};
  bus_8080_write_register(0x36, p_param, 1);
}

void ili9341_set_pixel_format_set()
{
  uint8_t p_param[1] = {0x55};
  bus_8080_write_register(0x3A, p_param, 1);
}

void ili9341_set_frame_rate_control()
{
  uint8_t p_param[2] = {0x00, 0x10};
  bus_8080_write_register(0xB1, p_param, 2);
}

void ili9341_set_display_function_control()
{
  uint8_t p_param[3] = {0x08, 0x82, 0x27};
  bus_8080_write_register(0xB6, p_param, 3);
}

void ili9341_exit_sleep_mode()
{
  bus_8080_write_register(0x11, NULL, 0);
  HAL_Delay(120);             //必须120ms的延迟
}


/** 
  * @brief  ILI9341 enters DISPLAY OFF mode. 
  *         Output of Frame Memory is disabled and blank paged is inserted. 
  * @param  None. 
  * @retval None.
  * @note   This command does not change current frame memory.
  */
void ili9341_set_display_off()
{
  bus_8080_write_register(0x28, NULL, 0);
}

/** 
  * @brief  ILI9341 enters DISPLAY ON mode. 
  *         This command recovers the IC from DISPLAY OFF mode.
  *         Output from frame memory is enabled.
  * @param  None. 
  * @retval None.
  I @note   This command does not change current frame memory.
  */
void ili9341_set_display_on()
{
  bus_8080_write_register(0x29, NULL, 0);
}


void ili9341_memory_write()
{
  bus_8080_write_register(0x2C, NULL, 0);
}

void ili9341_hard_reset() 
{
  LCD_RST_1();
  HAL_Delay(1);
  LCD_RST_0();
  HAL_Delay(1);
  LCD_RST_1();
  HAL_Delay(1);
}

/**
	* @brief Read LCD Controller Chip (ILI9341) ID. 
	* @note  It has been tested that some display modules has all manufacturer/version set to 0.
	*        Thus a more trustworthy way to test 8080 Read is to read the controller IC (ILI9341)'s ID through ID4 register.
	*/
void ili9341_get_id4(uint8_t* p_read_data) 
{
  bus_8080_write_command(0xD3);
  bus_8080_read_data(p_read_data, 4);

  char msg[64];
	sprintf(msg, "- Printing ID4 register value -\n");
	serial_print(msg);
	sprintf(msg, "ILI9341 IC Version: %d \nIC Model: 0x%02x%02x\n", p_read_data[1], p_read_data[2], p_read_data[3]); /* id4[0] is dummy byte. */
	serial_print(msg);
}


void ili9341_init()
{
  ili9341_hard_reset();
  ili9341_set_power_control_a();
  ili9341_set_power_control_b();
  ili9341_set_driver_timing_control_a();
  ili9341_set_driver_timing_control_b();
  ili9341_set_poweron_sequence_control_b();
  ili9341_set_pump_ratio_control();
  ili9341_set_power_control1();
  ili9341_set_power_control2();
  ili9341_set_vcom_control1();
  ili9341_set_vcom_control2();
  ili9341_set_memory_access_control();
  ili9341_set_pixel_format_set();
  ili9341_set_frame_rate_control();
  ili9341_set_display_function_control();
  ili9341_exit_sleep_mode();
  ili9341_set_display_on();
  ili9341_memory_write();
}



void ili9341_set_column_address(uint16_t x1, uint16_t x2)
{

  uint8_t p_param[4] = { (uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
  bus_8080_write_register(0x2A, p_param, 4);
}

void ili9341_set_page_address(uint16_t y1, uint16_t y2)
{

  uint8_t p_param[4] = { (uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
  bus_8080_write_register(0x2B, p_param, 4);
}


/**
	* @brief Send to ILI9341 the define area of frame memory where MCU can access.
  * @note  Refer to state diagram in ILI9341 Version V1.11 Section 8.8.20, Column Address Set.
	*/
void ili9341_set_frame_address(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
	ili9341_set_column_address(x1, x2);
	ili9341_set_page_address(y1, y2);
	ili9341_memory_write();
}




/**
	* @brief Draw a horizontal line.
	*/
void lcd_draw_horizontal_line(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t length, uint16_t line_color)                   
{ 
  bus_8080_write_command(0x2C);       //write_memory_start
	uint16_t end_coordinate_x = start_coordinate_x + length;
	uint16_t end_coordinate_y = start_coordinate_y;

	/* Sets the frame memory area. */
  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y);  

	/* Fill with line_color. */
  for(uint16_t i = 1; i<=length; ++i)
  {
    bus_8080_write_data(line_color >> 8);  /* MSB first. */
    bus_8080_write_data(line_color);      
  }
}

/**
	* Draw a vertical line. Fill color if necessary.
	*/
void lcd_draw_vertical_line(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t length, uint16_t line_color)                   
{ 
  
  bus_8080_write_command(0x2c); //write_memory_start
  
	uint16_t end_coordinate_x = start_coordinate_x;
	uint16_t end_coordinate_y = start_coordinate_y + length;
	/* Sets the frame memory area. */
  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y); 

	/* Fill with line_color. */
  for(uint16_t i=1; i<=length; ++i)
  { 
    bus_8080_write_data(line_color>>8);   
    bus_8080_write_data(line_color);       
  }
}

/**
	* @brief Draw an empty rectangle frame.
	* @note  This is equivalent to drawing two vertical lines plus two horizontal lines.
	*/
void lcd_draw_rectangle_unfilled(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t rect_width, uint16_t rect_height, uint16_t frame_color)
{
  lcd_draw_horizontal_line(start_coordinate_x, start_coordinate_y, rect_width, frame_color);
  lcd_draw_horizontal_line(start_coordinate_x, start_coordinate_y + rect_height, rect_width, frame_color);

  lcd_draw_vertical_line(start_coordinate_x, start_coordinate_y, rect_height, frame_color);
  lcd_draw_vertical_line(start_coordinate_x + rect_width, start_coordinate_y, rect_height, frame_color);
}

/**
	* @brief Draw a solid color-filled rectangle.
	*/
void lcd_draw_rectangle_filled(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t rect_width, uint16_t rect_height, uint16_t rect_color)
{
  for (uint16_t i = 0; i < rect_height; ++i)
  {
    lcd_draw_horizontal_line(start_coordinate_x, start_coordinate_y + i, rect_width, rect_color);
  }
}

/**
	* @brief Clear the entire screen and fill with color.
	*/
void lcd_clear_all(uint16_t fill_color)                   
{ 
	uint16_t start_coordinate_x = 0;
	uint16_t start_coordinate_y = 0;
	uint16_t end_coordinate_x = TFT_PIXEL_H_LENGTH - 1;
	uint16_t end_coordinate_y = TFT_PIXEL_V_WIDTH - 1;

  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y);
  for(uint16_t i = 0; i < TFT_PIXEL_V_WIDTH; ++i)  
    for(uint16_t m = 0; m < TFT_PIXEL_H_LENGTH; ++m) 
    {
			/* Write color to fill. */
      bus_8080_write_data(fill_color>>8);
      bus_8080_write_data(fill_color);
    }
}


void lcd_draw_dot(uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint16_t dot_color) 
{
	uint16_t end_coordinate_x = start_coordinate_x + 1;
	uint16_t end_coordinate_y = start_coordinate_y + 1;
  ili9341_set_frame_address(start_coordinate_x, start_coordinate_y, end_coordinate_x, end_coordinate_y);  
  bus_8080_write_data(dot_color >> 8); 
  bus_8080_write_data(dot_color);      
}


void lcd_plot_char(int16_t x, int16_t y, unsigned char c,uint16_t color, uint16_t bg, uint8_t size) 
{
	for (int8_t i=0; i<6; i++ )
	{
		uint8_t line;
		if (i == 5)
		{
			line = 0x0;
		}
		else 
		{
			line = *((uint8_t*)(font+(c*5)+i));
		}

		for (int8_t j = 0; j<8; j++)
		{
			if (line & 0x1)
			{
				if (size == 1) // default size
				{
					lcd_draw_dot(x+i, y+j, color);
				}
				else {  // big size
					// ili9341_plot_color_block(x+(i*size), y+(j*size), size, size, color);
				} 
			} else if (bg != color)
			{
				if (size == 1) // default size
				{
					lcd_draw_dot(x+i, y+j, bg);
				}
				else 
				{  // big size
					//ili9341_plot_color_block(x+i*size, y+j*size, size, size, bg);
				}
			}

			line >>= 1;
		}
	}
}

void lcd_set_rotation(uint8_t orientation) 
{
  bus_8080_write_command(0x36);    

  uint8_t p_param[1];

	switch (orientation) 
	{
		case 0:
      *p_param = 0x40 | 0x08;
      bus_8080_write_register(0x36, p_param, 1);
			break;
		case 1:
      *p_param = 0x20 | 0x08;
      bus_8080_write_register(0x36, p_param, 1);
			break;
		case 2:
      *p_param = 0x80 | 0x08;
      bus_8080_write_register(0x36, p_param, 1);
			break;
		case 3:
      *p_param = 0x40 | 0x80 | 0x20 | 0x08;
      bus_8080_write_register(0x36, p_param, 1);
			break;
	}
}


void lcd_write_message(char* message, uint16_t start_coordinate_x, uint16_t start_coordinate_y, uint8_t size, uint16_t text_color, uint16_t text_bg_color){
	for(uint8_t i = 0; i< strlen(message); i++){
		char single_char = message[i];
		lcd_plot_char(start_coordinate_x, start_coordinate_y, single_char, text_color, text_bg_color, 1);
		start_coordinate_x += 6;
	}
}
