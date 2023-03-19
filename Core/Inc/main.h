/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4p5g_discovery.h"
#include "stm32l4p5g_discovery_io.h"
#include "stm32l4p5g_discovery_camera.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* OSPI commands */
#define READ_CMD_SRAM                           0x00
#define WRITE_CMD_SRAM                          0x80
#define READ_REG_CMD_SRAM                       0x40
#define WRITE_REG_CMD_SRAM                      0xC0

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_SRAM_READ               5
#define DUMMY_CLOCK_CYCLES_SRAM_WRITE              4

/* Octal PSRAM APS6408L-3OB-BA memory */
/* Size of the PSRAM */
#define OSPI_PSRAM_SIZE          23   /* 64 Mbits */
//#define OSPI_PSRAM_INCR_SIZE     256

#define OSPI_PSRAM_INCR_SIZE  524288

/* End address of the OSPI memory */
#define OSPI_PSRAM_END_ADDR      (1 << OSPI_PSRAM_SIZE)

/* Size of buffers */
#define BUFFERSIZE                  (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)         (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DCMI_D2_Pin GPIO_PIN_0
#define DCMI_D2_GPIO_Port GPIOE
#define ST7735_CS_Pin GPIO_PIN_0
#define ST7735_CS_GPIO_Port GPIOI
#define DCMI_D4_Pin GPIO_PIN_14
#define DCMI_D4_GPIO_Port GPIOH
#define DCMI_D7_Pin GPIO_PIN_7
#define DCMI_D7_GPIO_Port GPIOI
#define DCMI_D6_Pin GPIO_PIN_6
#define DCMI_D6_GPIO_Port GPIOI
#define ST7735_RES_Pin GPIO_PIN_2
#define ST7735_RES_GPIO_Port GPIOI
#define DCMI_D3_Pin GPIO_PIN_12
#define DCMI_D3_GPIO_Port GPIOH
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define OCTOSPIM_P1_IO4_Pin GPIO_PIN_4
#define OCTOSPIM_P1_IO4_GPIO_Port GPIOD
#define ST7735_DC_Pin GPIO_PIN_1
#define ST7735_DC_GPIO_Port GPIOD
#define DCMI_D10_Pin GPIO_PIN_3
#define DCMI_D10_GPIO_Port GPIOI
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define OCTOSPIM_P1_IO5_Pin GPIO_PIN_5
#define OCTOSPIM_P1_IO5_GPIO_Port GPIOD
#define DCMI_D5_Pin GPIO_PIN_4
#define DCMI_D5_GPIO_Port GPIOI
#define DCMI_D0_Pin GPIO_PIN_9
#define DCMI_D0_GPIO_Port GPIOH
#define DCMI_VSYNC_Pin GPIO_PIN_5
#define DCMI_VSYNC_GPIO_Port GPIOI
#define OCTOSPIM_P1_IO7_Pin GPIO_PIN_7
#define OCTOSPIM_P1_IO7_GPIO_Port GPIOD
#define MFX_WAKEUP_Pin GPIO_PIN_5
#define MFX_WAKEUP_GPIO_Port GPIOF
#define OCTOSPIM_P1_DQS_Pin GPIO_PIN_6
#define OCTOSPIM_P1_DQS_GPIO_Port GPIOG
#define OCTOSPIM_P1_CLK_Pin GPIO_PIN_10
#define OCTOSPIM_P1_CLK_GPIO_Port GPIOE
#define OCTOSPIM_P1_NCLK_Pin GPIO_PIN_9
#define OCTOSPIM_P1_NCLK_GPIO_Port GPIOE
#define OCTOSPIM_P1_IO6_Pin GPIO_PIN_3
#define OCTOSPIM_P1_IO6_GPIO_Port GPIOC
#define MFX_IRQ_OUT_Pin GPIO_PIN_0
#define MFX_IRQ_OUT_GPIO_Port GPIOA
#define I2C4_SDA_Pin GPIO_PIN_15
#define I2C4_SDA_GPIO_Port GPIOF
#define OCTOSPIM_P1_IO2_Pin GPIO_PIN_7
#define OCTOSPIM_P1_IO2_GPIO_Port GPIOA
#define I2C4_SCL_Pin GPIO_PIN_14
#define I2C4_SCL_GPIO_Port GPIOF
#define OCTOSPIM_P1_IO1_Pin GPIO_PIN_13
#define OCTOSPIM_P1_IO1_GPIO_Port GPIOE
#define DCMI_PIXCLK_Pin GPIO_PIN_5
#define DCMI_PIXCLK_GPIO_Port GPIOH
#define OCTOSPIM_P1_IO3_Pin GPIO_PIN_6
#define OCTOSPIM_P1_IO3_GPIO_Port GPIOA
#define OCTOSPIM_P1_IO0_Pin GPIO_PIN_12
#define OCTOSPIM_P1_IO0_GPIO_Port GPIOE
#define DCMI_D1_Pin GPIO_PIN_10
#define DCMI_D1_GPIO_Port GPIOH
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define OCTOSPIM_P1_NCS_Pin GPIO_PIN_11
#define OCTOSPIM_P1_NCS_GPIO_Port GPIOE
#define DCMI_HSYNC_Pin GPIO_PIN_8
#define DCMI_HSYNC_GPIO_Port GPIOH
#define SPI2_CLK_Pin GPIO_PIN_13
#define SPI2_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DISCO_SPIx_TIMEOUT_MAX  (10000)

/**
  * @brief  LCD Control Interface pins
  */
#define LCD_CS_PIN                                 GPIO_PIN_5
#define LCD_CS_GPIO_PORT                           GPIOG
#define LCD_CS_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOG_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOG_CLK_DISABLE()

/**
  * @brief  LCD Data/Command Interface pins
  */
#define LCD_DC_PIN                                 GPIO_PIN_4
#define LCD_DC_GPIO_PORT                           GPIOG
#define LCD_DC_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOG_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOG_CLK_DISABLE()

/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)

/* --------------------------------RGB565 -------------------------------------*/
#define LCD_RGB565_COLOR_BLUE          0x001F
#define LCD_RGB565_COLOR_GREEN         0x07E0
#define LCD_RGB565_COLOR_RED           0xF800
#define LCD_RGB565_COLOR_CYAN          0x07FF
#define LCD_RGB565_COLOR_MAGENTA       0xF81F
#define LCD_RGB565_COLOR_YELLOW        0xFFE0
#define LCD_RGB565_COLOR_LIGHTBLUE     0x841F
#define LCD_RGB565_COLOR_LIGHTGREEN    0x87F0
#define LCD_RGB565_COLOR_LIGHTRED      0xFC10
#define LCD_RGB565_COLOR_LIGHTCYAN     0x87FF
#define LCD_RGB565_COLOR_LIGHTMAGENTA  0xFC1F
#define LCD_RGB565_COLOR_LIGHTYELLOW   0xFFF0
#define LCD_RGB565_COLOR_DARKBLUE      0x0010
#define LCD_RGB565_COLOR_DARKGREEN     0x0400
#define LCD_RGB565_COLOR_DARKRED       0x8000
#define LCD_RGB565_COLOR_DARKCYAN      0x0410
#define LCD_RGB565_COLOR_DARKMAGENTA   0x8010
#define LCD_RGB565_COLOR_DARKYELLOW    0x8400
#define LCD_RGB565_COLOR_WHITE         0xFFFF
#define LCD_RGB565_COLOR_LIGHTGRAY     0xD69A
#define LCD_RGB565_COLOR_GRAY          0x8410
#define LCD_RGB565_COLOR_DARKGRAY      0x4208
#define LCD_RGB565_COLOR_BLACK         0x0000
#define LCD_RGB565_COLOR_BROWN         0xA145
#define LCD_RGB565_COLOR_ORANGE        0xFD20

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
