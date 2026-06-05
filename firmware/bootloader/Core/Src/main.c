/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "touchsensing.h"
#include "tsc.h"
#include "tsl_types.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "tusb.h"
#include "tsl_user.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_BASE_ADDR  0x08004000UL
#define DFU_BOOT_FLAG  0xB00710ADUL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t dfu_boot_flag __attribute__((section(".dfu_flag")));
volatile bool dfu_active = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void jump_to_app(uint32_t app_addr) {
    typedef void (*fn_t)(void);
    uint32_t sp = *(volatile uint32_t*)app_addr;
    uint32_t pc = *(volatile uint32_t*)(app_addr + 4UL);

    if ((sp & 0xFF000000UL) != 0x20000000UL) return;

    __disable_irq();
    SysTick->CTRL = 0UL;
    NVIC->ICER[0] = 0xFFFFFFFFUL;
    NVIC->ICPR[0] = 0xFFFFFFFFUL;

    USB->BCDR &= (uint16_t)(~USB_BCDR_DPPU);
    USB->CNTR = USB_CNTR_FRES;
    for (volatile int i = 0; i < 50000; i++) {}
    __HAL_RCC_USB_CLK_DISABLE();

    SCB->VTOR = app_addr;
    __set_MSP(sp);
    ((fn_t)pc)();
    while (1) {}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  bool stay_dfu = (dfu_boot_flag == DFU_BOOT_FLAG);
  if (stay_dfu) { dfu_boot_flag = 0UL; }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* GPIO short fallback: PA1-PA2 shorted = DFU */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);
  for (volatile int i = 0; i < 1000; i++) {}
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) {
    stay_dfu = true;
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* CRS: trim HSI48 from USB SOF (no crystal) */
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef crs = {
      .Prescaler             = RCC_CRS_SYNC_DIV1,
      .Source                = RCC_CRS_SYNC_SOURCE_USB,
      .Polarity              = RCC_CRS_SYNC_POLARITY_RISING,
      .ReloadValue           = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U),
      .ErrorLimitValue       = RCC_CRS_ERRORLIMIT_DEFAULT,
      .HSI48CalibrationValue = 0x20U,
  };
  HAL_RCCEx_CRSConfig(&crs);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  MX_TSC_Init();
  MX_TOUCHSENSING_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* TSC calibration */
  if (!stay_dfu) {
    uint32_t cal_end = HAL_GetTick() + 500UL;
    while (HAL_GetTick() < cal_end) {
      tsl_user_Exec();
      if (MyTKeys[0].p_Data->StateId != TSL_STATEID_CALIB) break;
    }
  }

  /* USB init */
  __HAL_RCC_USB_CLK_ENABLE();
  NVIC_SetPriority(USB_IRQn, 0U);
  NVIC_EnableIRQ(USB_IRQn);
  tusb_init();

  if (!stay_dfu) {
    uint32_t probe_end = HAL_GetTick() + 500UL;
    while (HAL_GetTick() < probe_end) {
      tud_task();
      tsl_user_Exec();
      /* if (tud_mounted()) break; */
    }

    if (!tud_mounted()) {
      jump_to_app(APP_BASE_ADDR);
    } else {
      uint16_t m0 = MyChannels_Data[0].Meas;
      uint16_t m5 = MyChannels_Data[5].Meas;
      uint32_t ref = ((uint32_t)MyChannels_Data[2].Meas +
                      MyChannels_Data[3].Meas) / 2UL;
      bool key0 = (ref > 0 && m0 < (ref * 80UL / 100UL));
      bool key5 = (ref > 0 && m5 < (ref * 80UL / 100UL));
      if (!(key0 && key5) && !dfu_active) {
        jump_to_app(APP_BASE_ADDR);
      }
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);

  uint32_t dfu_deadline = HAL_GetTick() + 120000UL;

  while (1)
  {
    tud_task();

    uint32_t now = HAL_GetTick();
    uint32_t phase = now % 3000UL;
    uint32_t brightness;
    if (phase < 1500UL) {
      brightness = phase;
    } else {
      brightness = 3000UL - phase;
    }
    uint32_t val_1023  = (brightness * 1023UL)  / 1500UL;
    uint32_t val_16000 = (brightness * 16000UL) / 1500UL;

    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_1, val_16000);
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_2, val_16000);
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_4, val_16000);
    __HAL_TIM_SET_COMPARE(&htim21, TIM_CHANNEL_1, val_1023);
    __HAL_TIM_SET_COMPARE(&htim21, TIM_CHANNEL_2, val_1023);
    __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_1, val_1023);
    __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, val_1023);

    if (!dfu_active && now >= dfu_deadline) {
      jump_to_app(APP_BASE_ADDR);
      dfu_deadline = UINT32_MAX;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
