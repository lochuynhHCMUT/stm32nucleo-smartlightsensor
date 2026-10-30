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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim1;

#define WS_LED_COUNT     16U
#define WS_LEAD_SLOTS    16U
#define WS_RESET_SLOTS   320U

#define WS_BUFFER_LEN \
    (WS_LEAD_SLOTS + WS_LED_COUNT * 24U + WS_RESET_SLOTS)

/* TIM1 clock 16 MHz, PSC=0, ARR=19 */
#define WS_BIT_0         5U
#define WS_BIT_1         14U

static uint32_t ws_buffer[WS_BUFFER_LEN]
    __attribute__((aligned(32)));

static volatile uint8_t ws_busy = 0;
static volatile uint8_t ws_error = 0;
#define ADC_BRIGHT       2600.0f
#define ADC_DARK         3950.0f
#define LIGHT_MAX_LEVEL  6U

typedef enum
{
    MODE_AUTO = 0,
    MODE_MANUAL_50,
    MODE_MANUAL_100,
    MODE_OFF
} LightMode;

static LightMode light_mode = MODE_AUTO;

static float filtered_adc = 0.0f;
static uint8_t filter_ready = 0;
static uint8_t current_level = 0;

static volatile uint8_t sample_due = 0;
static volatile uint32_t button_events = 0;
static volatile uint8_t button_armed = 1;

static const char *mode_names[] = {
    "AUTO",
    "MANUAL_50",
    "MANUAL_100",
    "OFF"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_SendText(const char *text)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)strlen(text), 100);
}
static void WS_DMA_Complete(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_tim1_up)
    {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
        ws_busy = 0;
    }
}

static void WS_DMA_Error(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_tim1_up)
    {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
        ws_error = 1;
        ws_busy = 0;
    }
}
/*
 * Gui cung mot mau toi tat ca 16 LED.
 * Tham so theo thu tu R, G, B.
 */
static HAL_StatusTypeDef WS_SetAll(uint8_t r,
                                 uint8_t g,
                                 uint8_t b)
{
    uint32_t index = WS_LEAD_SLOTS;
    if (r > LIGHT_MAX_LEVEL) r = LIGHT_MAX_LEVEL;
    if (g > LIGHT_MAX_LEVEL) g = LIGHT_MAX_LEVEL;
    if (b > LIGHT_MAX_LEVEL) b = LIGHT_MAX_LEVEL;
    uint32_t color = ((uint32_t)g << 16)
                   | ((uint32_t)r << 8)
                   |  (uint32_t)b;

    if (ws_busy)
    {
        return HAL_BUSY;
    }

    /* Dau va cuoi frame giu muc LOW */
    memset(ws_buffer, 0, sizeof(ws_buffer));

    for (uint32_t led = 0; led < WS_LED_COUNT; led++)
    {
        for (int bit = 23; bit >= 0; bit--)
        {
            ws_buffer[index++] =
                (color & (1UL << bit)) ? WS_BIT_1 : WS_BIT_0;
        }
    }

    /*
     * STM32F7: neu D-cache dang bat, dong bo buffer
     * truoc khi DMA doc. Buffer can hang va kich thuoc
     * deu la boi so cua 32 byte.
     */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr(
            (uint32_t *)ws_buffer,
            (int32_t)sizeof(ws_buffer));
    }

    /* Tat cac yeu cau DMA truoc khi chuan bi frame */
    __HAL_TIM_DISABLE_DMA(&htim1,
                          TIM_DMA_UPDATE | TIM_DMA_CC1);
    __HAL_TIM_DISABLE(&htim1);

    __HAL_TIM_SET_COUNTER(&htim1, 0);

    /* DMA ghi vao preload, ap dung o chu ky PWM tiep theo */
    htim1.Instance->CCMR1 |= TIM_CCMR1_OC1PE;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    htim1.Instance->EGR = TIM_EGR_UG;

    __HAL_TIM_CLEAR_FLAG(&htim1,
                         TIM_FLAG_UPDATE | TIM_FLAG_CC1);

    hdma_tim1_up.XferCpltCallback = WS_DMA_Complete;
    hdma_tim1_up.XferErrorCallback = WS_DMA_Error;
    hdma_tim1_up.XferHalfCpltCallback = NULL;

    ws_busy = 1;
    ws_error = 0;

    __DSB();

    /* DMA theo UPDATE, nhung du lieu van ghi vao CCR1 */
    HAL_StatusTypeDef status = HAL_DMA_Start_IT(
        &hdma_tim1_up,
        (uint32_t)ws_buffer,
        (uint32_t)&htim1.Instance->CCR1,
        WS_BUFFER_LEN);

    if (status != HAL_OK)
    {
        ws_busy = 0;
        return status;
    }

    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    if (status != HAL_OK)
    {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
        HAL_DMA_Abort(&hdma_tim1_up);
        ws_busy = 0;
        return status;
    }

    uint32_t start = HAL_GetTick();

    while (ws_busy)
    {
        if ((HAL_GetTick() - start) >= 20U)
        {
            __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
            HAL_DMA_Abort(&hdma_tim1_up);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            ws_busy = 0;
            return HAL_TIMEOUT;
        }
    }

    /* Cuoi buffer da co doan LOW de chot du lieu LED */
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim1,
                         TIM_FLAG_UPDATE | TIM_FLAG_CC1);

    return ws_error ? HAL_ERROR : HAL_OK;
}
/* TIM4 chi bao den thoi diem cap nhat */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        sample_due = 1;
    }
}

/* Nhan mot lan nhan; khoa cho den khi nut duoc tha on dinh */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == GPIO_PIN_13) && button_armed)
    {
        button_armed = 0;
        button_events++;
    }
}

/* Goi lien tuc trong main, khong dung HAL_Delay */
static void Button_Service(void)
{
    static uint8_t release_tracking = 0;
    static uint32_t release_start = 0;

    if (button_armed)
    {
        release_tracking = 0;
        return;
    }

    /* B1: tha = LOW, nhan = HIGH */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        if (!release_tracking)
        {
            release_tracking = 1;
            release_start = HAL_GetTick();
        }
        else if ((HAL_GetTick() - release_start) >= 50U)
        {
            button_armed = 1;
            release_tracking = 0;
        }
    }
    else
    {
        release_tracking = 0;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  UART_SendText("\r\nSmart light: AUTO / MANUAL_50 / MANUAL_100 / OFF\r\n");
  HAL_Delay(100);
  if (WS_SetAll(0, 0, 0) != HAL_OK)
  {
      UART_SendText("LED initialization failed\r\n");
      Error_Handler();
  }
  current_level = 0;
  light_mode = MODE_AUTO;
  filter_ready = 0;
  button_events = 0;
  /* Neu dang giu nut luc khoi dong, cho tha nut truoc */
  button_armed =
      (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);

  /* Xoa co update do qua trinh khoi tao timer */
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
      UART_SendText("TIM4 start failed\r\n");
      Error_Handler();
  }
  sample_due = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Button_Service();

	  /* Lay va xoa cac co an toan voi ngat */
	  uint32_t irq_state = __get_PRIMASK();
	  __disable_irq();

	  uint32_t presses = button_events;
	  button_events = 0;

	  uint8_t update = sample_due;
	  sample_due = 0;

	  __set_PRIMASK(irq_state);

	  /* Xu ly chuyen che do trong main */
	  if (presses > 0U)
	  {
	      light_mode = (LightMode)(
	          ((uint32_t)light_mode + (presses % 4U)) % 4U);

	      /* Cap nhat ngay sau khi nhan nut */
	      update = 1;

	      char mode_message[64];
	      snprintf(mode_message, sizeof(mode_message),
	               "MODE -> %s\r\n",
	               mode_names[(uint32_t)light_mode]);

	      UART_SendText(mode_message);
	  }

	  if (update)
	  {
	      static uint32_t last_raw = 0;
	      uint8_t adc_ok = 0;

	      /* Lay mau cam bien o ca bon che do de theo doi */
	      if (HAL_ADC_Start(&hadc1) == HAL_OK)
	      {
	          if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
	          {
	              last_raw = HAL_ADC_GetValue(&hadc1);
	              adc_ok = 1;

	              if (!filter_ready)
	              {
	                  filtered_adc = (float)last_raw;
	                  filter_ready = 1;
	              }
	              else
	              {
	                  filtered_adc += 0.15f
	                                * ((float)last_raw - filtered_adc);
	              }
	          }

	          HAL_ADC_Stop(&hadc1);
	      }

	      uint8_t next_level = current_level;

	      switch (light_mode)
	      {
	          case MODE_AUTO:
	          {
	              if (adc_ok)
	              {
	                  float ratio =
	                      (filtered_adc - ADC_BRIGHT)
	                      / (ADC_DARK - ADC_BRIGHT);

	                  if (ratio < 0.0f) ratio = 0.0f;
	                  if (ratio > 1.0f) ratio = 1.0f;

	                  float desired =
	                      ratio * (float)LIGHT_MAX_LEVEL;

	                  /* Hysteresis + moi chu ky thay doi mot muc */
	                  if ((desired > (float)current_level + 0.65f)
	                      && (current_level < LIGHT_MAX_LEVEL))
	                  {
	                      next_level++;
	                  }
	                  else if ((desired < (float)current_level - 0.65f)
	                           && (current_level > 0U))
	                  {
	                      next_level--;
	                  }
	              }
	              /* ADC loi: giu muc hien tai va bao trong log */
	              break;
	          }

	          case MODE_MANUAL_50:
	              next_level = LIGHT_MAX_LEVEL / 2U;  /* Muc 3 */
	              break;

	          case MODE_MANUAL_100:
	              next_level = LIGHT_MAX_LEVEL;       /* Muc 6 */
	              break;

	          case MODE_OFF:
	          default:
	              next_level = 0;
	              break;
	      }

	      const char *tx_result = "HOLD";

	      if (next_level != current_level)
	      {
	          HAL_StatusTypeDef status = WS_SetAll(
	              next_level, next_level, next_level);

	          if (status == HAL_OK)
	          {
	              current_level = next_level;
	              tx_result = "OK";
	          }
	          else
	          {
	              tx_result = "ERROR";
	          }
	      }

	      char message[160];
	      snprintf(message, sizeof(message),
	               "MODE=%s | ADC=%lu | Filter=%lu"
	               " | LED=%u/%u | ADC_OK=%u | TX=%s\r\n",
	               mode_names[(uint32_t)light_mode],
	               (unsigned long)last_raw,
	               (unsigned long)filtered_adc,
	               (unsigned int)current_level,
	               (unsigned int)LIGHT_MAX_LEVEL,
	               (unsigned int)adc_ok,
	               tx_result);

	      UART_SendText(message);
	  }
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1599;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
