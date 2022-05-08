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

/* Macros -----------------------------------------------------------------------------------------------------------------------------------------*/
#define ILI9341_DEBUG /* When defined, debug messages are logged through serial_print(). */

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
  * @brief  Write to devie register through 8080 bus. 
  * @param  target_register Command byte. Essentially the target register to write.
  * @param  p_param Pointer to an array containing parameters to write to the targert register.
  * @param  length Number of parameters in bytes.
  * @retval None.
  * @note   Parallel 8080 Bus Time Write Sequence:
  *         CS  ````\_________________________________________________________________/````
  *         RS  ``````\_______________/````````````````````````````````````````````````````
  *         WR  ____________________/_________________/________________/______________/````
  *         RD  ```````````````````````````````````````````````````````````````````````````
  *         D[7:0]       | COMMAND |    | PARAMETER |    | PARAMETER |   | PARAMETER |
  *         ILI9341                 read              read              read          read 
  *         MCU             write            write           write           write
  *        
  *         RS Stands for Register Select. It is equvilent to D/C (DCX) pin in ili9341 datasheet.
  *           When RS = 0, ili9341 interpret D[7:0] as command.
  *           When RS = 1, ili9341 interpret D[7:0] as data. 
  *         RD Read Line always high indicating read is inactive. 
  *         WR Write Line needs to show rising edge every time data is already sent on D[7:0],
  *           ILI9341 will read data on the WR rising edge. 
  */
  
void bus_8080_write_register(uint8_t target_register, uint8_t length, uint8_t* p_param)
{
  /* Write command byte (target register) */
  LCD_CS_0();  /* Chip Select */
  LCD_RS_0();  /* RS (Data / Command) = 0, selecting command*/
  LCD_RD_1();  /* READ = 1. Read line inactive. */
  gpio_configure_8080_datapins_output_mode(); /* Configure D7-D0 GPIO output mode */
  bus_8080_write_parallel_datapins(target_register); /* Write target register on D7-D0. */
  LCD_WR_RISING_EDGE(); /* Validate the byte on WRITE rising edge. */

  LCD_RS_1();   /* RS (Data / Command Selection) = 1, selecting data. */
  /* Iterate through each parameters. */ 
  for (int i = 0; i < length; ++i)
  {
    // LCD_RD_1(); /* already set above */
    // gpio_configure_8080_datapins_output_mode();  /* already set above */
    bus_8080_write_parallel_datapins(p_param[i]);
    LCD_WR_RISING_EDGE(); /* Validate the byte on WRITE rising edge. */
  }
  LCD_CS_1(); /* De-select Chip. */
}


/**
  * @brief  Read device register value from 8080 bus. 
  * @param  target_register Command byte. Essentially the target register to write.
  * @param  length Number of parameters in bytes.
  * @param  p_param Pointer to an array containing read data from target device.
  * @retval None.
  * @note   Parallel 8080 Bus Time Write Sequence:
  *         CS  ````\_________________________________________________________________/````
  *         RS  ``````\_______________/````````````````````````````````````````````````````
  *         WR  ____________________/``````````````````````````````````````````````````````
  *         RD  ``````````````````````````___________/````````````````__/``````````````_/``
  *         D[7:0]       | COMMAND |    | PARAMETER |    | PARAMETER |   | PARAMETER |
  *         ILI9341                 read  write            write           write
  *         MCU             write                read               read           read
  *         
  *         RS Stands for Register Select. It is equvilent to D/C (DCX) pin in ili9341 datasheet.
  *           When RS = 0, ili9341 interpret D[7:0] as command.
  *           When RS = 1, ili9341 interpret D[7:0] as data. 
  *         RD Read Line needs to pull rising edge by the MCU every time data prepared by ili9341
  *           has been consumed by the MCU.
  *         WR Write Line turns high once the MCU has written the command byte. 
  *           The MCU enters read mode and consumes data prepared by ili9341 on D[7:0].
  */
void bus_8080_read_register(uint8_t target_register, uint8_t length, uint8_t* p_read_data)
{
  
  /* Write command byte (target register) */
  LCD_CS_0(); /* Chip Select */
  LCD_RS_0();  
  LCD_RD_1();
  
  gpio_configure_8080_datapins_output_mode();
  bus_8080_write_parallel_datapins(target_register); /* Write data to data bus D7-D0. */  
  LCD_WR_RISING_EDGE();

  /* MCU Enter read mode. */
  LCD_RS_1();  
  LCD_WR_1();
  gpio_configure_8080_datapins_input_mode();
  /* Iterate through each data. */ 
  for (int i = 0; i < length; ++i)
  {
    LCD_RD_0();
    p_read_data[i] = bus_8080_read_parallel_datapins();
    LCD_RD_1();
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
  * @brief  Reset ili9341 through RESET pin.
  * @param  None.
  * @retval None.
  * @note   RST pin is active low. ili9341 is reset state at low level.
  */
void ili9341_hard_reset() 
{
  LCD_RST_1();
  HAL_Delay(1);
  LCD_RST_0();
  HAL_Delay(1);
  LCD_RST_1();
  HAL_Delay(1);
}


/* ILI9341 Regulative Registers Access -------------------------------------------------------------------------------------------*/

/**
  * @brief 8.2.1. NOP (00h) 
  * @param  None.
  * @retval None.
  */
void ili9341_nop()
{
  bus_8080_write_register(0x00, 0, NULL);
}

/**
  * @brief  8.2.2. Software Reset (01h) 
  * @param  None.
  * @retval None.
  */
void ili9341_soft_reset()
{
  bus_8080_write_register(0x01, 0, NULL);
}


/** 
  * @brief  TODO 8.2.3. Read display identification information (04h) 
  * @param  p_read_data Pointer to display identification data.
  * @retval None.
  */
void ili9341_get_display_identification(uint8_t* p_read_data)
{

}

/** 
  * @brief  TODO 8.2.4. Read Display Status (09h) 
  * @param  p_read_data Pointer to display status data.
  * @retval None.
  */
void ili9341_get_display_status(uint8_t* p_read_data)
{

}

/** 
  * @brief  TODO 8.2.5. Read Display Power Mode (0Ah) 
  * @param  p_read_data Pointer to display power mode.
  * @retval None.
  */
void ili9341_get_display_power_mode(uint8_t* p_read_data)
{

}


/** 
  * @brief  TODO 8.2.6. Read Display MADCTL (0Bh) 
  * @param  p_read_data Pointer to display MADCTL
  * @retval None.
  */
void ili9341_get_display_MADCTL(uint8_t* p_read_data)
{

}

/**
  * @brief       8.2.7. Read Display Pixel Format (0Ch) 
  * @param[OUT]  p_read_data Pointer to pixel format data read.
  * @retval      None.
  * @note        DBI[2:0]       #Bits/pixel      RGB Format 
  *              101            16 bits          RGB5-6-5 65K)    
  *              110            18 bits          RGB6-6-6 262K) 
  */
void ili9341_get_display_pixel_format(uint8_t* p_read_data)
{
  bus_8080_read_register(0x0C, 2, p_read_data);
 
  #if defined ILI9341_DEBUG
  char msg[64];
  sprintf(msg, "- Printing <Display Pixel Format> register value -\n");
  serial_print(msg);
  sprintf(msg, "Display pixel format: RIM:%d DPI: %d DBI: %d \n", p_read_data[1] & 0x80, p_read_data[1] & 0x70, p_read_data[1] & 0x07); /* p_read_data[0] is a dummy byte. */
  serial_print(msg);
  #endif
}


/** 
  * @brief  TODO 8.2.8. Read Display Image Format (0Dh)
  * @param  p_read_data Pointer to display format.
  * @retval None.
  */
void ili9341_get_display_image_format(uint8_t* p_read_data)
{

}

/**
  * @brief  8.2.9. Read Display Signal Mode (0Eh) 
  * @param  p_read_data Pointer to data read from RDDSM (Read Display Signal Mode).
  * @retval None. 
  * @note   Bit               Description                   
  *         7                 0 Tearing effect line OFF
  *                           1 Tearing effect line ON
  *         6                 0 Tearing effect line mode 1
  *                           1 Tearing effect line mode 2
  *         5                 0 Horizontal sync. (RGB interface) OFF
  *                           1 Horizontal sync. (RGB interface) ON
  *         4                 0 Vertical sync. (RGB interface) OFF
  *                           1 Vertical sync. (RGB interface) ON
  *         3                 0 Pixel clock (DOTCLK, RGB interface) OFF
  *                           1 Pixel clock (DOTCLK, RGB interface) ON
  *         2                 0 Data enable (DE, RGB interface) OFF  
  *                           1 Data enable (DE, RGB interface) ON
  *         1                 Don't Care
  *         0                 Don't Care
  */
void ili9341_get_display_signal_mode(uint8_t* p_read_data)
{
  bus_8080_read_register(0x0E, 2, p_read_data);
 
  #if defined ILI9341_DEBUG

  char msg[64];
  sprintf(msg, "- Printing <RDDSM (Read Display Signal Mode)> register value -\n"); serial_print(msg);
  sprintf(msg, "Tearing effect line: %d\n", (p_read_data[1] & 0x80) ? 1 : 0); 
  serial_print(msg);
  sprintf(msg, "Tearing effect line mode: %d\n", (p_read_data[1] & 0x40) ? 1 : 0); 
  serial_print(msg);
  sprintf(msg, "Horizontal sync: %d\n", (p_read_data[1] & 0x20) ? 1 : 0); 
  serial_print(msg);
  sprintf(msg, "Vertical sync: %d\n", (p_read_data[1] & 0x10) ? 1 : 0); 
  serial_print(msg);
  sprintf(msg, "Pixel clock: %d\n", (p_read_data[1] & 0x08) ? 1 : 0); 
  serial_print(msg);
  sprintf(msg, "Data enable: %d\n", (p_read_data[1] & 0x04) ? 1 : 0); 
  serial_print(msg);
  #endif
}



/** 
  * @brief  TODO 8.2.10. Read Display Self-Diagnostic Result (0Fh) 
  * @param  p_read_data Pointer to display self-diagnostic result.
  * @retval None.
  */
void ili9341_get_display_diagnostic_result(uint8_t* p_read_data)
{

}

/** 
  * @brief  8.2.11. Enter Sleep Mode (10h) 
  * @param  None.
  * @retval None.
  */
void ili9341_enter_sleep_mode() 
{
  bus_8080_write_register(0x10, 0, NULL);
}


/** 
  * @brief  8.2.12. Sleep Out (11h) 
  * @param  None.
  * @retval None.
  */
void ili9341_exit_sleep_mode()
{
  bus_8080_write_register(0x11, 0, NULL);
  HAL_Delay(120);             //必须120ms的延迟
}

/**
  * @brief  8.2.13. Partial Mode ON (12h)
  * @param  None.
  * @retval None. 
  * @note   This command turns on partial mode the partial mode window 
  *            is described by the Partial Area command (0x30). 
  *          To leave Partial mode, the Normal Display Mode On 
  *            command (0x13) should be written
  */ 
void ili9341_set_partial_mode_on()
{
  bus_8080_write_register(0x12, 0, NULL);
}


/** 
  * @brief  8.2.14. Normal Display Mode ON (13h)
  * @param  None.
  * @retval None.
  */
void ili9341_normal_mode_on()
{
  bus_8080_write_register(0x13, 0, NULL);
}

/** 
  * @brief  8.2.15. Display Inversion OFF (20h)
  * @param  None.
  * @retval None.
  */
void ili9341_set_display_inversion_off()
{
  bus_8080_write_register(0x20, 0, NULL);
}

/** 
  * @brief  8.2.16. Display Inversion ON (21h) 
  * @param  None.
  * @retval None.
  */
void ili9341_set_display_inversion_on()
{
  bus_8080_write_register(0x21, 0, NULL);
}


/** 
  * @brief  8.2.17. Gamma Set (26h) 
  * @param  None.
  * @retval None.
  */
void ili9341_set_gamma()
{
  uint8_t p_param[1] = {0x01};
  bus_8080_write_register(0x26, 4, p_param);
}


/** 
  * @brief  8.2.18. Display OFF (28h) 
  *         Output of Frame Memory is disabled and blank paged is inserted. 
  * @param  None. 
  * @retval None.
  * @note   This command does not change current frame memory.
  */
void ili9341_set_display_off()
{
  bus_8080_write_register(0x28, 0, NULL);
}


/** 
  * @brief  8.2.19. Display ON (29h) 
  *         This command recovers the IC from DISPLAY OFF mode.
  *         Output from frame memory is enabled.
  * @param  None. 
  * @retval None.
  I @note   This command does not change current frame memory.
  */
void ili9341_set_display_on()
{
  bus_8080_write_register(0x29, 0, NULL);
}


/** 
  * @brief  8.2.20. Column Address Set (2Ah) 
  * @param  x1 Starting column 
  * @param  x2 Ending column
  * @retval None.
  * @note   Imagine vertical pointers as columns. Counterpart to page addresses.
  */
void ili9341_set_column_address(uint16_t x1, uint16_t x2)
{
  uint8_t p_param[4] = { (uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
  bus_8080_write_register(0x2A, 4, p_param);
}


/** 
  * @brief  8.2.21. Page Address Set (2Bh)
  * @param  y1 Starting page
  * @param  y2 Ending page
  * @retval None.
  * @note   Imagine horizontal pointers as columns. Counterpart to column addresses.
  */
void ili9341_set_page_address(uint16_t y1, uint16_t y2)
{
  uint8_t p_param[4] = { (Vuint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
  bus_8080_write_register(0x2B, 4, p_param);
}


/** 
  * @brief  TODO 8.2.22. Memory Write (2Ch) 
  * @param  None.
  * @retval None.
  */
void ili9341_memory_write()
{
  bus_8080_write_register(0x2C, 0, NULL);
}

/* 8.2.23. Color Set (2Dh) */ 
void ili9341_set_color()
{

}
/* 8.2.24. Memory Read (2Eh) */ 
void ili9341_memory_read()
{

}
/* 8.2.25. Partial Area (30h) */ 
void ili9341_set_partial_area()
{
  
}
/* 8.2.26. Vertical Scrolling Definition (33h) */ 
void ili9341_set_vertical_scrolling_definition(uint16_t top_fixed_area_height, uint16_t vertical_scrolling_area_height, uint16_t bottom_fixed_area_height)
{
  uint8_t p_param[6] = {top_fixed_area_height >> 8, top_fixed_area_height & 0xFF,
                        vertical_scrolling_area_height >> 8, vertical_scrolling_area_height & 0xFF,
                        bottom_fixed_area_height >> 8, bottom_fixed_area_height & 0xFF
                       };
  bus_8080_write_register(0x35, 6, p_param);
}

/* 8.2.27. Tearing Effect Line OFF (34h) */ 
void ili9341_set_tearing_effect_line_off()
{

}
/**
  * @brief  8.2.28. Tearing Effect Line ON (35h) 
  * @param  None.
  * @retval None.
  * @note   When M=0:
  *   When M=1:
  *   The Tearing Effect Output line consists of V-Blanking information only:
  *   When M=0:
  *   The Tearing Effect Output Line consists of both V-Blanking and H-Blanking information.
  *   To confirm tearing effect mode on/off, read 0x0E, Read Display Signal Mode.
  */
void ili9341_set_tearing_effect_line_on()
{
  uint8_t p_param[1] = {1 << 0};
  bus_8080_write_register(0x35, 1, p_param);
}
/* 8.2.29. Memory Access Control (36h) */
void ili9341_set_memory_access_control()
{
  uint8_t p_param[1] = {0x08};
  bus_8080_write_register(0x36, 1, p_param);
}

/* 8.2.30. Vertical Scrolling Start Address (37h) */
void ili9341_vertical_scrolling_start_address()
{

}
/**
  * @brief  8.2.31. Idle Mode OFF (38h) 
  * @param  None.
  * @retval None.
  * @brief This command is used to enter into Idle mode on.
  *          In the idle on mode, color expression is reduced. 
  *        The primary and the secondary colors using MSB of each RGB 
  *          in the Frame Memory, 8 color depth data is displayed.
  */
void ili9341_set_idle_mode_off()
{
  bus_8080_write_register(0x38, 0, NULL);
}
/* 8.2.32. Idle Mode ON (39h) */
void ili9341_set_idle_mode_on()
{
  bus_8080_write_register(0x39, 0, NULL);
}
/* 8.2.33. COLMOD: Pixel Format Set (3Ah) */
void ili9341_set_pixel_format_set()
{
  uint8_t p_param[1] = {0x55};
  bus_8080_write_register(0x3A, 1, p_param);
}

/* 8.2.34. Write_Memory_Continue (3Ch) */
void ili9341_write_memory_continue()
{

}
/* 8.2.35. Read_Memory_Continue (3Eh)*/ 
void ili9341_read_memory_continue()
{

}
/* 8.2.36. Set_Tear_Scanline (44h)*/ 
void ili9341_set_tear_scanline()
{

}
/* 8.2.37. Get_Scanline (45h) */ 
void ili9341_get_scanline()
{

}
/**
  * @brief  8.2.38. Write Display Brightness (51h) 
  * @param  None.
  * @retval None.
  * @note   At the time of this program developed, 
  *           the brightness pinout is hardwired to VCC. 
  *         Thus through experiment I was not able to set brightness
  *           by writing to 0x51. 
  */ 
  
void ili9341_set_display_brightness()
{
  /* p_param pointes the brightness value. */
  uint8_t p_param[1] = {0x55};
  bus_8080_write_register(0x51, 1, p_param);
}

/**
  * @brief      8.2.39. Read Display Brightness (52h) 
  * @param[OUT] p_read_data Pointer to brightness data read.
  * @retval     None. 
  * @note   At the time of this program developed, 
  *           the brightness pinout is hardwired to VCC. 
  *         Thus through experiment I was not able to set brightness
  *           by writing to 0x51. 
  *         And brightness read from 0x52 always returns 0 for unknown reason.
  */
void ili9341_get_display_brightness(uint8_t* p_read_data)
{
  bus_8080_read_register(0x52, 2, p_read_data);
 
  #if defined ILI9341_DEBUG
  char msg[64];
  sprintf(msg, "- Printing ILI9341 <Display Brightness> register value -\n");
  serial_print(msg);
  sprintf(msg, "Brightness (0 ~ 255): %#02x\n", p_read_data[1]); /* id4[0] is dummy byte. */
  serial_print(msg);
  #endif

}
/** 
  * @brief  8.2.40. Write CTRL Display (53h) 
  * @note   At the time of this program developed, 
  *           the brightness pinout is hardwired to VCC. 
  *         Writing to CTRL_display was successful, however LCD brightness 
  *           did not adjust accordingly. 
  */
void ili9341_set_CTRL_display()
{
  uint8_t p_param[1] = {0x00};
  bus_8080_write_register(0x53, 1, p_param);
}

/**
  * @brief  8.2.41. Read CTRL Display (54h) 
  * @param  p_read_data Pointer to data read from CTRL Display register.
  * @retval None.
  * @note   This command is used to read brightness setting.
  *         BCTRL: Brightness Control Block On/Off,
  *         DD: Display Dimming
  *         BL: Backlight On/Off
  
  */
void ili9341_get_CTRL_display(uint8_t* p_read_data)
{
  bus_8080_read_register(0x54, 2, p_read_data);
 
  #if defined ILI9341_DEBUG
  char msg[64];
  sprintf(msg, "- Printing ILI9341 <CTRL Display> register value -\n");
  serial_print(msg);
  sprintf(msg, "BCTRL (Brightness Control on/off): %d\n", (p_read_data[1] & 0x20) ? 1 : 0); 
  serial_print(msg);
  sprintf(msg, "DD (Display Dimming on/off): %d\n", (p_read_data[1] & 0x08) ? 1 : 0); 
  serial_print(msg);
  sprintf(msg, "BL (Complete turn off backlight): %d\n", (p_read_data[1] & 0x04) ? 1 : 0); 
  serial_print(msg);
  #endif
}
/* 8.2.42. Write Content Adaptive Brightness Control (55h) */ 
void ili9341_set_content_adaptive_brightness_control()
{

}
/* 8.2.43. Read Content Adaptive Brightness Control (56h) */ 
void ili9341_get_content_adaptive_brightness_control()
{

}
/* 8.2.44. Write CABC Minimum Brightness (5Eh) */ 
void ili9341_set_CABC_minimum_brightness()
{

}
/* 8.2.45. Read CABC Minimum Brightness (5Fh) */ 
void ili9341_get_CABC_minimum_brightness()
{

}

/** 
  * @brief 8.2.46. Read ID1 (DAh) 
  * @param  p_read_data Pointer to the array storing data read from ID1 register.
  * @retval None.
  * @note.  p_read_data[0] is expected a dummy byte, ignore it.
  *         At the time of this code development, id1 returns 0x00
  *           even through other register read / write were tested successful.
  *         Since <id1> is defined as the LCD module's manufatuerer ID,
  *           some brand may use ILI9341 but never programmed its ID.
  *         In order to do sanity check on fresh connection, use ili9341_get_id4() instead, 
  *           which should return 0x9341 on its third and fourth bytes.
  */
void ili9341_get_id1(uint8_t* p_read_data)
{
  bus_8080_read_register(0xDA, 2, p_read_data);
 
  #if defined ILI9341_DEBUG
  char msg[64];
  sprintf(msg, "- Printing ILI9341 <id1> register value -\n");
  serial_print(msg);
  sprintf(msg, "ID1: %#02x\n", p_read_data[1]); 
  serial_print(msg);
  #endif
}


/** 
  * @brief  8.2.47. Read ID2 (DBh) 
  * @param  p_read_data Pointer to the array storing data read from ID2 register.
  * @retval None.
  * @note.  p_read_data[0] is expected a dummy byte, ignore it.
  *         At the time of this code development, id1 returns 0x00
  *           even through other register read / write were tested successful.
  *         Since <id2> is defined as the LCD module's manufatuerer ID,
  *           some brand may use ILI9341 but never programmed its ID.
  *         In order to do sanity check on fresh connection, use ili9341_get_id4() instead, 
  *           which should return 0x9341 on its third and fourth bytes.
  */
void ili9341_get_id2(uint8_t* p_read_data)
{
  bus_8080_read_register(0xDB, 2, p_read_data);
 
  #if defined ILI9341_DEBUG
  char msg[64];
  sprintf(msg, "- Printing ILI9341 <id2> register value -\n");
  serial_print(msg);
  sprintf(msg, "ID2: %#02x\n", p_read_data[1]); 
  serial_print(msg);
  #endif

}

/** 
  * @brief  8.2.48. Read ID3 (DCh)
  * @param  p_read_data Pointer to the array storing data read from ID3 register.
  * @retval None.
  * @note.  p_read_data[0] is expected a dummy byte, ignore it.
  *         At the time of this code development, id3 returns 0x00
  *           even through other register read / write were tested successful.
  *         Since <id3> is defined as the LCD module's driver ID.
  *           some brand may use ILI9341 but never programmed its ID.
  *         In order to do sanity check on fresh connection, use ili9341_get_id4() instead, 
  *           which should return 0x9341 on its third and fourth bytes.
  */
void ili9341_get_id3(uint8_t* p_read_data)
{
  bus_8080_read_register(0xDC, 2, p_read_data);
 
  #if defined ILI9341_DEBUG
  char msg[64];
  sprintf(msg, "- Printing ILI9341 <id3> register value -\n");
  serial_print(msg);
  sprintf(msg, "ID3: %#02x\n", p_read_data[1]); 
  serial_print(msg);
  #endif
}


/* ILI9341 Extended Registers Access -------------------------------------------------------------------------------------------*/
/* 8.3.1. RGB Interface Signal Control (B0h) */
void ili9341_rgb_interface_signal_control()
{

}
/* 8.3.2. Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
void ili9341_frame_control_normal_mode()
{

}
/* 8.3.3. Frame Rate Control (In Idle Mode/8 colors) (B2h) */
void ili9341_frame_control_idle_mode()
{

}
/* 8.3.4. Frame Rate control (In Partial Mode/Full Colors) (B3h) */
void ili9341_frame_control_partial_mode()
{

}
/* 8.3.5. Display Inversion Control (B4h) */
void ili9341_display_inversion_control()
{

}
/* 8.3.6. Blanking Porch Control (B5h) */
void ili9341_blanking_porch_control()
{

}
/* 8.3.7. Display Function Control (B6h) */
void ili9341_display_function_control()
{

}
/* 8.3.8. Entry Mode Set (B7h) */
void ili9341_entry_mode_set()
{

}
/* 8.3.9. Backlight Control 1 (B8h) */
void ili9341_backlight_control_1()
{

}
/* 8.3.10. Backlight Control 2 (B9h) */
void ili9341_backlight_control_2()
{

}
/* 8.3.11. Backlight Control 3 (BAh) */
void ili9341_backlight_control_3()
{

}
/* 8.3.12. Backlight Control 4 (BBh) */
void ili9341_backlight_control_4()
{

}
/* 8.3.13. Backlight Control 5 (BCh) */
void ili9341_backlight_control_5()
{

}
/* 8.3.14. Backlight Control 7 (BEh) */
void ili9341_backlight_control_7()
{

}
/* 8.3.15. Backlight Control 8 (BFh)*/
void ili9341_backlight_control_8()
{

}
/* 8.3.16. Power Control 1 (C0h) */ 
void ili9341_set_power_control1()
{
  uint8_t p_param[1] = {0x23};
  bus_8080_write_register(0xC0, 1, p_param);
}

/* 8.3.17. Power Control 2 (C1h) */ 
void ili9341_set_power_control2()
{
  uint8_t p_param[1] = {0x10};
  bus_8080_write_register(0xC1, 1, p_param);
}

/* 8.3.18. VCOM Control 1(C5h) */ 
void ili9341_set_vcom_control1()
{
  uint8_t p_param[2] = {0x3E, 0x28};
  bus_8080_write_register(0xC5, 2, p_param);
}

/* 8.3.19. VCOM Control 2(C7h) */ 
void ili9341_set_vcom_control2()
{
  uint8_t p_param[1] = {0x86};
  bus_8080_write_register(0xC7, 1, p_param);
}

/* 8.3.20. NV Memory Write (D0h) */ 
void ili9341_nv_memory_write()
{

}
/* 8.3.21. NV Memory Protection Key (D1h) */
void ili9341_nv_memory_protection_key()
{

}
/* 8.3.22. NV Memory Status Read (D2h) */
void ili9341_nv_memory_status_read()
{

}
/* 8.3.23. Read ID4 (D3h) */
/**
  * @brief Read LCD Controller Chip (ILI9341) ID. 
  * @note  It has been tested that some display modules has all manufacturer/version set to 0.
  *        Thus a more trustworthy way to test 8080 Read is to read the controller IC (ILI9341)'s ID through ID4 register.
  */
void ili9341_get_id4(uint8_t* p_read_data) 
{
  bus_8080_read_register(0xD3, 4, p_read_data);
 
  #if defined ILI9341_DEBUG
  char msg[64];
  sprintf(msg, "- Printing id4 register value -\n");
  serial_print(msg);
  sprintf(msg, "ili9341 ic version: %d \nIC model: 0x%02x%02x\n", p_read_data[1], p_read_data[2], p_read_data[3]); /* id4[0] is dummy byte. */
  serial_print(msg);
  #endif
}

/* 8.3.24. Positive Gamma Correction (E0h) */
void ili9341_positive_gamma_correction()
{

}
/* 8.3.25. Negative Gamma Correction (E1h) */
void ili9341_negative_gamma_correction()
{

}
/* 8.3.26. Digital Gamma Control 1 (E2h) */
void ili9341_digital_gamma_control_1()
{

}
/* 8.3.27. Digital Gamma Control 2(E3h) */
void ili9341_digital_gamma_control_2()
{

}
/* 8.3.28. Interface Control (F6h)  */
void ili9341_interface_control()
{

}

/* 8.4.1 Power control A (CBh) */
/** 
  * @brief Set Power Control A. 
  */
void ili9341_set_power_control_a()
{
  /* Vcore = 1.6V DDVDH = 5.6V. */
  uint8_t p_param[5] = {0x39, 0x2C, 0x00, 0x34, 0x02};
  bus_8080_write_register(0xCB, 5, p_param);
}

/* 8.4.2 Power control B (CFh) */ 
void ili9341_set_power_control_b()
{
  uint8_t p_param[3] = {0x00, 0xC1, 0x30};
  bus_8080_write_register(0xCF, 3, p_param);
}

/* 8.4.3 Driver timing control A (E8h) */ 
void ili9341_set_driver_timing_control_a()
{
  uint8_t p_param[3] = {0x85, 0x00, 0x78};
  bus_8080_write_register(0xE8, 3, p_param);
}


/* 8.4.5 Driver timing control B (EAh) */ 
void ili9341_set_driver_timing_control_b()
{

  uint8_t p_param[2] = {0x00, 0x00};
  bus_8080_write_register(0xEA, 2, p_param);
}

/* 8.4.6 Power on sequence control (EDh) */ 
void ili9341_set_poweron_sequence_control()
{

}
/* 8.4.7 Enable 3G (F2h) */
void ili9341_enable_3G()
{

}















void ili9341_set_frame_rate_control()
{
  uint8_t p_param[2] = {0x00, 0x10};
  bus_8080_write_register(0xB1, 2, p_param);
}

void ili9341_set_display_function_control()
{
  uint8_t p_param[3] = {0x08, 0x82, 0x27};
  bus_8080_write_register(0xB6, 3, p_param);
}



void ili9341_set_poweron_sequence_control_b()
{
  uint8_t p_param[4] = {0x64, 0x03, 0x12, 0x81};
  bus_8080_write_register(0xED, 4, p_param);
}

/**/
void ili9341_set_pump_ratio_control()
{
  uint8_t p_param[1] = {0x20};
  bus_8080_write_register(0xF7, 1, p_param);
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
      bus_8080_write_register(0x36, 1, p_param);
      break;
    case 1:
      *p_param = 0x20 | 0x08;
      bus_8080_write_register(0x36, 1, p_param);
      break;
    case 2:
      *p_param = 0x80 | 0x08;
      bus_8080_write_register(0x36, 1, p_param);
      break;
    case 3:
      *p_param = 0x40 | 0x80 | 0x20 | 0x08;
      bus_8080_write_register(0x36, 1, p_param);
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
