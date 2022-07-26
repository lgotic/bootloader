/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SP_RANGE 0x2FFF0000u
#define SRAM_BASE_ADDR 0x20030000u
#define IS_VALID_SP(address) (((*(volatile uint32_t*)address) & SP_RANGE) == SRAM_BASE_ADDR)

#define APPLICATION_ADDRESS     (uint32_t)0x08004000u
#define APPLICATION_SECTOR_ADDRESS 1u
#define APPLICATION_SECTOR_LENGTH 1u
#define SECOND_FIMRWARE_ADDRESS (uint32_t)0x08008000u
#define SECOND_FIRMWARE_SECTOR_END (uint32_t)0x0800c000u
typedef void (*pFunction)(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void jumpToProgramFunction () {
pFunction Jump_To_Application_Function;
uint32_t JumpAddress;
JumpAddress = *(volatile uint32_t*) (APPLICATION_ADDRESS + 4);
Jump_To_Application_Function = (void*) JumpAddress;
__set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
Jump_To_Application_Function();
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void cleareSector () {
	FLASH_EraseInitTypeDef EraseInitStruct;
	volatile uint8_t flashEraseError;
	uint32_t volatile sectorError;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = APPLICATION_SECTOR_ADDRESS;
	EraseInitStruct.NbSectors = APPLICATION_SECTOR_LENGTH;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	if ( HAL_FLASHEx_Erase(&EraseInitStruct, &sectorError) != HAL_OK) {
		  flashEraseError=HAL_FLASH_GetError();
	  }

}

void flashNewFirmware () {
	HAL_StatusTypeDef flag;
	flag = HAL_FLASH_Unlock();
	volatile uint32_t flashData = 0;
	uint32_t firmwareAddress = APPLICATION_ADDRESS;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_BSY);
	cleareSector ();

	for (int positionCounter = SECOND_FIMRWARE_ADDRESS ; positionCounter < SECOND_FIRMWARE_SECTOR_END; positionCounter = positionCounter+4) {
		flashData = (*(uint8_t *)(positionCounter + 3)) << 24;
		flashData = flashData | (*(uint8_t *)(positionCounter + 2)) << 16;
		flashData = flashData | (*(uint8_t *)(positionCounter + 1)) << 8;
		flashData = flashData | (*(uint8_t *)(positionCounter));

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, firmwareAddress, flashData);
		for ( int I = 0 ; I <200 ; I ++) {};
		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET) {};
		firmwareAddress = firmwareAddress+4;
		}
	HAL_FLASH_Lock();
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//__disable_irq();
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
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if ( *(uint8_t*)(0x08003000) == 0x00) {
		  flashNewFirmware ();
	  }
	  else if ( IS_VALID_SP(APPLICATION_ADDRESS)) {

		  HAL_Delay(1000);
		  //__disable_irq();
		  HAL_SuspendTick();
		  HAL_DeInit();
		  __enable_irq();
		  jumpToProgramFunction ();
	  }
	  else {

		  flashNewFirmware ();
	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
