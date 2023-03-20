/* USER CODE BEGIN Header */
/*
 *
Table 8. CN1 STMod+ connector pinout
Pin number Description Pin number Description
1   SPI1_NSS/USART3_CTS (PG5/PD11)      11  INT (PD10)
2   SPI1_MOSI/ USART3_TX (PG4/PD8)      12  RESET (PF11)
3   SPI1_MISO/ USART3_RX (PG3/PD9)      13  ADC (PA5)
4   SPI1_SCK/ USART3_RTS (PG2/PD12)     14  PWM (PG11)
5   GND                                 15 5V
6   5V                                  16 GND
7   I2C4_SCL (PF14)                     17 DFSDM1-CKOUT (PF10)
8   SPI1_MOSIs (PE15)                   18 DFSDM1- DATIN1 (PB12)
9   SPI1_MISOs (PE14)                   19 GPIO3 (PD0)
10  I2C4_SDA (PF15)                     20 GPIO4 (PD1)

It is SPI but with a single bidirectional pin e.g.
LED backlight, sometimes called BL. (it might be LED anode or cathode)
SDA bidirectional data pin. Connect to MOSI
SCK clock. sometimes called SCLK, SCL, ...
A0 data/command GPIO OUTPUT pin. sometimes called DC, RS, ...
RESET reset pin. you must connect. (or pullup)
CS chip select

ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
Since the provided driver doesn't use read capability from LCD, only constraint
on write baudrate is considered.

 */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "st7735.h"
#include "fonts.h"
#include "testimg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_SRAM
//#define  CAMERA_WIDTH    ((uint16_t)320)
//#define  CAMERA_HEIGHT   ((uint16_t)240)

#define  CAMERA_WIDTH    ((uint16_t)160)
#define  CAMERA_HEIGHT   ((uint16_t)120)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

OSPI_HandleTypeDef hospi1;

SPI_HandleTypeDef hspi2;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
/* USER CODE BEGIN PV */
extern DCMI_HandleTypeDef hDcmiHandler;
static volatile uint32_t start_the_camera_capture = 0;
uint16_t pBuffer[CAMERA_WIDTH * CAMERA_HEIGHT];

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication**** ";

static __IO uint32_t transferErrorDetected;    /* Set to 1 if an error transfer is detected */
static __IO uint32_t transferCompleteDetected; /* Set to 1 if transfer is correctly completed */

HAL_StatusTypeDef hal_status = HAL_OK;

static SPI_HandleTypeDef hdisco_Spi;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI2_Init(void);
static void MX_DCMI_Init(void);
static void MX_OCTOSPI1_Init(void);
/* USER CODE BEGIN PFP */

static void TransferComplete(DMA_HandleTypeDef *DmaHandle);
static void TransferError(DMA_HandleTypeDef *DmaHandle);
static void OnError_Handler(uint32_t condition);

static void SPIx_Init(void);
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
static void SPIx_Write(uint8_t Value);
void LCD_Delay(uint32_t Delay);

static void BSP_OSPIM_Init(void);
static void BSP_OSPIM_RCC_delay(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART2 is connected to st-link
//int _write(int file, char *ptr, int len)
//{
//	HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
//	return len;
//}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_I2C4_Init();
	MX_SPI2_Init();
	MX_DCMI_Init();
	MX_OCTOSPI1_Init();
	/* USER CODE BEGIN 2 */
	MX_GPIO_Init();
	MX_SPI2_Init();
	ST7735_Init();

	ST7735_FillScreen(ST7735_BLACK);

	BSP_OSPIM_Init();
	BSP_OSPIM_RCC_delay();

	//  BSP_LCD_Init();
	//  uint32_t XSize= BSP_LCD_GetXSize();
	//  uint32_t YSize= BSP_LCD_GetYSize();
	//
	//  BSP_LCD_DrawPixel(0, 0, LCD_RGB565_COLOR_RED);
	//
	//  BSP_LCD_SetBackColor(LCD_RGB565_COLOR_LIGHTBLUE);
	//  BSP_LCD_SetTextColor(LCD_RGB565_COLOR_RED);
	//  BSP_LCD_FillRect(0, 0, 100, 100);


	/*##-2- Initialize the LCD #################################################*/
	/*##-1- LEDs initialization  #################################################*/
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED1);

	/* Initialize the IO functionalities */
	hal_status = BSP_IO_Init();
	OnError_Handler(hal_status != HAL_OK);

	hal_status = BSP_IO_ConfigPin(IO_PIN_3, IO_MODE_OUTPUT);
	OnError_Handler(hal_status != HAL_OK);

	BSP_IO_WritePin(IO_PIN_3, BSP_IO_PIN_SET);
	HAL_Delay(100);

	hal_status = BSP_IO_ConfigPin(IO_PIN_7, IO_MODE_INPUT);
	OnError_Handler(hal_status != HAL_OK);

	uint32_t val= BSP_IO_ReadPin(IO_PIN_7);
	printf("plugged %lu\n", val);

#define CAM
#ifdef CAM
	/*##-3- Camera Initialization ############################*/
	/* Initialize the Camera in QQVGA mode */
	hal_status = BSP_CAMERA_Init(CAMERA_R160x120);
	OnError_Handler(hal_status != HAL_OK);

	/* Wait 1s to let auto-loops in the camera module converge and lead to correct exposure */
	HAL_Delay(1000);

	/*##-4- Camera Continuous capture start in QVGA resolution ############################*/
	/* Disable unwanted HSYNC (IT_LINE)/VSYNC interrupts */
	__HAL_DCMI_DISABLE_IT(&hDcmiHandler, DCMI_IT_LINE | DCMI_IT_VSYNC);

#define OSPIM
#ifdef OSPIM
	hal_status = HAL_DCMI_Start_DMA(&hDcmiHandler, DCMI_MODE_SNAPSHOT,  (uint32_t)(OCTOSPI1_BASE) , CAMERA_WIDTH * CAMERA_HEIGHT/2 );
#else
	hal_status = HAL_DCMI_Start_DMA(&hDcmiHandler, DCMI_MODE_SNAPSHOT,  (uint32_t)(pBuffer) , CAMERA_WIDTH * CAMERA_HEIGHT/2 );
	/* ------------------------------------------------------------------------------------------- uint32_t size ---- */
#endif


	OnError_Handler(hal_status != HAL_OK);

	/*##-4- Line capture ############################*/
	printf("capture %lu\n", start_the_camera_capture);
	while(start_the_camera_capture == 0) {;}
	printf("capture %lu\n", start_the_camera_capture);

#ifdef OSPIM
	/* Read back data from the SRAM memory */
	for(uint32_t pos = 0; pos < CAMERA_WIDTH * CAMERA_HEIGHT; pos++)
	   {
		uint16_t val = * (__IO uint16_t *)(OCTOSPI1_BASE+2*pos);
		pBuffer[pos] = val;
	   }
#endif

#define noDRAWIMAGE
	for (uint16_t i=0; i<CAMERA_WIDTH; i++){
		for (uint16_t j=0; j<CAMERA_HEIGHT; j++){
			uint16_t color565 = pBuffer[j*CAMERA_WIDTH+i];
#ifdef DRAWIMAGE
			color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8);
			pBuffer[j*CAMERA_WIDTH+i]=color565;
			ST7735_DrawImage(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT, pBuffer);
#else
			ST7735_DrawPixel(i, j, color565);
#endif
		}
	}

#endif



	/* Set to 1 if an transfer error is detected */
	transferErrorDetected = 0;
	transferCompleteDetected = 0;

	/*##-5- Select Callbacks functions called after Transfer complete and Transfer error */
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel1, HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel1, HAL_DMA_XFER_ERROR_CB_ID, TransferError);

	if (HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel1, (uint32_t)&aTxBuffer, (uint32_t)(OCTOSPI1_BASE), BUFFERSIZE) != HAL_OK)
	{
		/* Transfer Error */
		Error_Handler();
	}


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1){
		if (transferErrorDetected == 1){
			/* Toggle LED2 with a period of 200 ms */
			BSP_LED_Toggle(LED2);
			HAL_Delay(200);
		}
		if (transferCompleteDetected == 1){
			/* Turn LED2 on*/
			BSP_LED_On(LED2);
			transferCompleteDetected = 0;

			/* Read back data from the SRAM memory */
			for(__IO uint32_t  uwIndex = 0; uwIndex < BUFFERSIZE; uwIndex++)
			{
				__IO uint16_t uval = aTxBuffer[uwIndex];
				if (uval != aTxBuffer[uwIndex]) Error_Handler();

				printf("%lu / %u %u\n", uwIndex, BUFFERSIZE, uval);
				//printf("%c", uval);
			}
			printf("\n");
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 55;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief DCMI Initialization Function
 * @param None
 * @retval None
 */
static void MX_DCMI_Init(void)
{

	/* USER CODE BEGIN DCMI_Init 0 */

	/* USER CODE END DCMI_Init 0 */

	/* USER CODE BEGIN DCMI_Init 1 */

	/* USER CODE END DCMI_Init 1 */
	hdcmi.Instance = DCMI;
	hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
	hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
	hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
	hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
	hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
	hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
	hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
	hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
	hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
	hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
	hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
	if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DCMI_Init 2 */

	/* USER CODE END DCMI_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x40505681;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void)
{

	/* USER CODE BEGIN I2C4_Init 0 */

	/* USER CODE END I2C4_Init 0 */

	/* USER CODE BEGIN I2C4_Init 1 */

	/* USER CODE END I2C4_Init 1 */
	hi2c4.Instance = I2C4;
	hi2c4.Init.Timing = 0x40505681;
	hi2c4.Init.OwnAddress1 = 0;
	hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c4.Init.OwnAddress2 = 0;
	hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c4) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C4_Init 2 */

	/* USER CODE END I2C4_Init 2 */

}

/**
 * @brief OCTOSPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OCTOSPI1_Init(void)
{

	/* USER CODE BEGIN OCTOSPI1_Init 0 */

	/* USER CODE END OCTOSPI1_Init 0 */

	OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

	/* USER CODE BEGIN OCTOSPI1_Init 1 */

	/* USER CODE END OCTOSPI1_Init 1 */
	/* OCTOSPI1 parameter configuration*/
	hospi1.Instance = OCTOSPI1;
	hospi1.Init.FifoThreshold = 2;
	hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
	hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_APMEMORY;
	hospi1.Init.DeviceSize = 23;
	hospi1.Init.ChipSelectHighTime = 1;
	hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
	hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
	hospi1.Init.ClockPrescaler = 0x03;
	hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
	hospi1.Init.ChipSelectBoundary = 4;
	hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
	hospi1.Init.Refresh = 0;
	if (HAL_OSPI_Init(&hospi1) != HAL_OK)
	{
		Error_Handler();
	}
	OSPIM_Cfg_Struct.ClkPort = 1;
	OSPIM_Cfg_Struct.DQSPort = 1;
	OSPIM_Cfg_Struct.NCSPort = 1;
	OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
	OSPIM_Cfg_Struct.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
	if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN OCTOSPI1_Init 2 */

	/* USER CODE END OCTOSPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_1LINE;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * Enable DMA controller clock
 * Configure DMA for memory to memory transfers
 *   hdma_memtomem_dma1_channel1
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
	hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
	hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_MEM2MEM;
	hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
	hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
	hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
	hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_HIGH;
	if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
	{
		Error_Handler( );
	}

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA2_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOI, ST7735_CS_Pin|ST7735_RES_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PI10 PI9 PI1 PI11
                           PI8 */
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_1|GPIO_PIN_11
			|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pins : PH2 PH15 PH13 PH7
                           PH3 PH6 PH0 PH1
                           PH4 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_7
			|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1
			|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : PB4 PB3 PB5 PB8
                           PB9 PB11 PB0 PB1
                           PB14 PB2 PB10 PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8
			|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_1
			|GPIO_PIN_14|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA15 PA12 PA11 PA10
                           PA9 PA8 PA5 PA4
                           PA3 PA2 PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10
			|GPIO_PIN_9|GPIO_PIN_8|GPIO_PIN_5|GPIO_PIN_4
			|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ST7735_CS_Pin ST7735_RES_Pin */
	GPIO_InitStruct.Pin = ST7735_CS_Pin|ST7735_RES_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pins : PE1 PE4 PE3 PE2
                           PE6 PE5 PE15 PE8
                           PE14 PE7 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2
			|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_15|GPIO_PIN_8
			|GPIO_PIN_14|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PG9 PG10 PG11 PG12
                           PG14 PG13 PG1 PG8
                           PG7 PG0 PG5 PG4
                           PG3 PG2 */
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
			|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_8
			|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_4
			|GPIO_PIN_3|GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : PD0 PD2 PD6 PD3
                           PD15 PD10 PD14 PD12
                           PD11 PD13 PD9 PD8 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_3
			|GPIO_PIN_15|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_12
			|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_9|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : ST7735_DC_Pin */
	GPIO_InitStruct.Pin = ST7735_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ST7735_DC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC10 PC13 PC11 PC14
                           PC12 PC6 PC15 PC9
                           PC8 PC7 PC4 PC0
                           PC1 PC2 PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14
			|GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_15|GPIO_PIN_9
			|GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_0
			|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PF2 PF1 PF0 PF3
                           PF4 PF10 PF11 PF13
                           PF12 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_3
			|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13
			|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : MFX_WAKEUP_Pin */
	GPIO_InitStruct.Pin = MFX_WAKEUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MFX_WAKEUP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OCTOSPIM_P1_NCLK_Pin */
	GPIO_InitStruct.Pin = OCTOSPIM_P1_NCLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
	HAL_GPIO_Init(OCTOSPIM_P1_NCLK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MFX_IRQ_OUT_Pin */
	GPIO_InitStruct.Pin = MFX_IRQ_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MFX_IRQ_OUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief  Initialize SPI HAL.
 * @retval None
 */
static void SPIx_Init(void)
{
	if (HAL_SPI_GetState(&hdisco_Spi) == HAL_SPI_STATE_RESET)
	{
		/* SPI Config */
		/* SPI baudrate is set to 13.75 MHz maximum (SPI_BaudRatePrescaler = 8)
     to verify these constraints:
        - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
          Since the provided driver doesn't use read capability from LCD, only constraint
          on write baudrate is considered.
		 */
		hdisco_Spi.Instance = SPI1;
		hdisco_Spi.Init.Mode = SPI_MODE_MASTER;
		hdisco_Spi.Init.Direction = SPI_DIRECTION_2LINES;
		hdisco_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
		hdisco_Spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
		hdisco_Spi.Init.CLKPhase = SPI_PHASE_2EDGE;
		hdisco_Spi.Init.NSS = SPI_NSS_SOFT;
		hdisco_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
		hdisco_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hdisco_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
		hdisco_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hdisco_Spi.Init.CRCPolynomial = 7;
		hdisco_Spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		hdisco_Spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		if (HAL_SPI_Init(&hdisco_Spi) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

/**
 * @brief  SPI Write byte(s) to device
 * @param  DataIn: Pointer to data buffer to write
 * @param  DataOut: Pointer to data buffer for read data
 * @param  DataLength: number of bytes to write
 * @retval None
 */
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_TransmitReceive(&hdisco_Spi, (uint8_t *) DataIn, DataOut, DataLength, DISCO_SPIx_TIMEOUT_MAX);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		Error_Handler();
	}
}

/**
 * @brief  SPI Write a byte to device
 * @param  Value: value to be written
 * @retval None
 */
static void SPIx_Write(uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t data;

	status = HAL_SPI_TransmitReceive(&hdisco_Spi, (uint8_t *) &Value, &data, 1, DISCO_SPIx_TIMEOUT_MAX);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		Error_Handler();
	}
}

/**
 * @brief  Initialize the LCD
 * @retval None
 */
void LCD_IO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct = {0};

	/* LCD_CS_GPIO and LCD_DC_GPIO Periph clock enable */
	LCD_CS_GPIO_CLK_ENABLE();
	LCD_DC_GPIO_CLK_ENABLE();

	/* Configure LCD_CS_PIN pin: LCD Card CS pin */
	GPIO_InitStruct.Pin    = LCD_CS_PIN;
	GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull   = GPIO_NOPULL;
	GPIO_InitStruct.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStruct);

	/* Configure LCD_DC_PIN pin: LCD Card DC pin */
	GPIO_InitStruct.Pin    = LCD_DC_PIN;
	HAL_GPIO_Init(LCD_DC_GPIO_PORT, &GPIO_InitStruct);

	/* LCD chip select high */
	LCD_CS_HIGH();

	/* LCD SPI Config */
	SPIx_Init();
}

/**
 * @brief  Write command to select the LCD register.
 * @param  LCDReg: Address of the selected register.
 * @retval None
 */
void LCD_IO_WriteReg(uint8_t LCDReg)
{
	/* Reset LCD control line CS */
	LCD_CS_LOW();

	/* Set LCD data/command line DC to Low */
	LCD_DC_LOW();

	/* Send Command */
	SPIx_Write(LCDReg);

	/* Deselect : Chip Select high */
	LCD_CS_HIGH();
}

/**
 * @brief  Write register value.
 * @param  pData Pointer on the register value
 * @param  Size Size of byte to transmit to the register
 * @retval None
 */
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
	uint32_t counter = 0;
	__IO uint32_t data = 0;

	/* Reset LCD control line CS */
	LCD_CS_LOW();

	/* Set LCD data/command line DC to High */
	LCD_DC_HIGH();

	if (Size == 1)
	{
		/* Only 1 byte to be sent to LCD - general interface can be used */
		/* Send Data */
		SPIx_Write(*pData);
	}
	else
	{
		/* Several data should be sent in a raw */
		/* Direct SPI accesses for optimization */
		for (counter = Size; counter != 0; counter--)
		{
			while (((hdisco_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
			{
			}
			/* Need to invert bytes for LCD*/
			*((__IO uint8_t *)&hdisco_Spi.Instance->DR) = *(pData + 1);

			while (((hdisco_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
			{
			}
			*((__IO uint8_t *)&hdisco_Spi.Instance->DR) = *pData;
			counter--;
			pData += 2;
		}

		/* Wait until the bus is ready before releasing Chip select */
		while (((hdisco_Spi.Instance->SR) & SPI_FLAG_BSY) != RESET)
		{
		}
	}

	/* Empty the Rx fifo */
	data = *(&hdisco_Spi.Instance->DR);
	UNUSED(data);  /* Remove GNU warning */

	/* Deselect : Chip Select high */
	LCD_CS_HIGH();
}

/**
 * @brief  Wait for loop in ms.
 * @param  Delay in ms.
 * @retval None
 */
void LCD_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

/**
 * @brief  Camera Frame Event callback.
 */
void BSP_CAMERA_FrameEventCallback(void)
{
	start_the_camera_capture = 1;
}

/**
 * @brief  Initialize OctoSPI and enable memory-mapped mode
 */
static void BSP_OSPIM_Init(void){
	OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};
	OSPI_RegularCmdTypeDef  sCommand = {0};
	OSPI_MemoryMappedTypeDef sMemMappedCfg = {0};
	uint8_t reg[2];

	OSPIM_Cfg_Struct.ClkPort    = 1;
	OSPIM_Cfg_Struct.DQSPort    = 1;
	OSPIM_Cfg_Struct.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
	OSPIM_Cfg_Struct.IOLowPort  = HAL_OSPIM_IOPORT_1_LOW;
	OSPIM_Cfg_Struct.NCSPort    = 1;

	hospi1.Instance = OCTOSPI1;
	HAL_OSPI_DeInit(&hospi1);

	__HAL_RCC_OSPIM_CLK_ENABLE();
	if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_OSPI_Init(&hospi1) != HAL_OK) {
		Error_Handler();
	}

	sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
	sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
	sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
	sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
	sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
	sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
	sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
	sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
	sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
	sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
	sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
	sCommand.Instruction        = READ_REG_CMD_SRAM;
	sCommand.Address            = 0;
	sCommand.NbData             = 2;
	sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_READ;

	if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_OSPI_Receive(&hospi1, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}

	sCommand.Instruction = WRITE_REG_CMD_SRAM;
	sCommand.DummyCycles = 0;
	MODIFY_REG(reg[0], 0x03, 0x00);

	if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_OSPI_Transmit(&hospi1, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}

	/****************************************************************************/
	/*                                                                          */
	/* Enable memory-mapped mode                                                */
	/*                                                                          */
	/****************************************************************************/
	sCommand.Instruction        = WRITE_CMD_SRAM;
	sCommand.OperationType      = HAL_OSPI_OPTYPE_WRITE_CFG;
	sCommand.NbData             = 10;
	sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_SRAM_WRITE;

	if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}

	sCommand.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
	sCommand.Instruction   = READ_CMD_SRAM;
	sCommand.DummyCycles   = DUMMY_CLOCK_CYCLES_SRAM_READ;

	if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}

	sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
	if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK) {
		Error_Handler();
	}


}

/**
 * @brief  RCC delay configuration register setting
 * May vary from board to be board. To be implemented by user application
 */
static void BSP_OSPIM_RCC_delay(void){

	__IO uint8_t *mem_addr=NULL;
	uint32_t delay=1, calibration_ongoing=1, test_failed=1;
	while (calibration_ongoing)
	{
		HAL_RCCEx_OCTOSPIDelayConfig(delay, 0);
		test_failed = 0;
		mem_addr = (__IO uint8_t *)(OCTOSPI1_BASE);
		for (uint16_t index = 0; index < BUFFERSIZE; index++)
		{
			*mem_addr = aTxBuffer[index];
			mem_addr++;
		}
		mem_addr = (__IO uint8_t *)(OCTOSPI1_BASE);
		for (uint16_t index = 0; index < BUFFERSIZE; index++)
		{
			if (*mem_addr != aTxBuffer[index])
			{
				test_failed = 1;
				break;
			}
			mem_addr++;
		}

		if (test_failed == 1)
		{
			if (delay < 15)
			{
				/* Increase delay and relaunch iteration */
				delay++;
			}
			else
			{
				/* If delay set to maximum and error still detected: can't use external PSRAM */
				Error_Handler();
			}
		}
		else
		{
			/* Proper delay found, exit calibration */
			calibration_ongoing = 0;
		}
	} /* while (calibration_ongoing) */
}

/**
 * @brief  DMA conversion complete callback
 * @note   This function is executed when the transfer complete interrupt
 *         is generated
 * @retval None
 */
static void TransferComplete(DMA_HandleTypeDef *DmaHandle)
{
	transferCompleteDetected = 1;
}

/**
 * @brief  DMA conversion error callback
 * @note   This function is executed when the transfer error interrupt
 *         is generated during DMA transfer
 * @retval None
 */
static void TransferError(DMA_HandleTypeDef *DmaHandle)
{
	transferErrorDetected = 1;
}

/**
 * @brief  On Error Handler on condition TRUE.
 * @param  condition : Can be TRUE or FALSE
 * @retval None
 */
static void OnError_Handler(uint32_t condition)
{
	if(condition)
	{
		BSP_LED_On(LED1);
		while(1) { ; } /* Blocking on error */
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
