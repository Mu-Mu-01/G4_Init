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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dpll_app.h"
#include "bsp_signal.h"
#include "bsp_signal_settings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* DPLL 波形参数：基频单位为 Hz，幅度和直流偏置采用 12 位 DAC 码。 */
#define NCO_INITIAL_FREQUENCY_HZ                 30000.0
#define DPLL_INITIAL_DAC_AMPLITUDE                1700U
#define DPLL_INITIAL_DAC_DC_OFFSET               2048U
#define DPLL_INITIAL_OUTPUT_WAVEFORM             NCO_OUTPUT_WAVEFORM_SINE
#define DPLL_INITIAL_OUTPUT_FREQUENCY_MULTIPLIER 1U /* 输出基频的整数倍。 */

/* 目标相位为 DAC 输出减 ADC 输入，单位为度。G474 补偿系数默认置零。 */
#define DPLL_INITIAL_LOCK_PHASE_DEG         0.0f

#define DPLL_PHASE_FIT_INTERCEPT_DEG        0.0f /* 单位：度。 */
#define DPLL_PHASE_FIT_SLOPE_DEG_PER_KHZ    0.0f /* 单位：度/kHz。 */

/* TIM8 更新率 = ADC 采样率 × 升采样倍数，TIM1 对 TIM8 更新事件分频。
 * 每半块处理截止时间 = ADC 块长 / 实际 ADC 采样率。 */
#define DPLL_DAC_OUTPUT_ENABLED             true

/* 比例、积分增益分别以 Hz/rad 和 Hz/(rad·s) 表示。 */
#define DPLL_INITIAL_PI_KP                  DPLL_PI_DEFAULT_KP_HZ_PER_RAD
#define DPLL_INITIAL_PI_KI                  DPLL_PI_DEFAULT_KI_HZ_PER_RAD_S
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 置位相应请求标志即可从主循环或调试器申请更新，处理结果由 App 回写。 */
DPLL_App_RuntimeControl g_dpll_runtime_control = {
    .signal_config = {
        .adc_sample_rate_hz = DPLL_ADC_SAMPLE_RATE_HZ,
        .dac_upsampling_factor = DPLL_DAC_UPSAMPLING_FACTOR,
        .dac_output_enabled = DPLL_DAC_OUTPUT_ENABLED,
        .adc_block_samples = DPLL_ADC_BLOCK_SAMPLES,
        .reference_sample_offset = UINT32_MAX, /* 由 G474 SignalPort 推导参考偏移。 */
        .output_buffers = {NULL, NULL},        /* 使用 BSP 提供的两个静态输出缓冲区。 */
        .output_capacity_samples = 0U,         /* 采用 BSP 的每块容量。 */
    },
    .pi_kp_hz_per_rad = DPLL_INITIAL_PI_KP,
    .pi_ki_hz_per_rad_s = DPLL_INITIAL_PI_KI,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  BSP_Signal_PreparePlatform();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  /* 启动和捕获期间保持灭灯。 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /* 将已初始化的板级信号端口传给 App，并按参考例程的顺序设置输出和闭环。 */
  const DPLL_App_Config dpll_app_config = {
      .port = BSP_Signal_Configure(&g_dpll_runtime_control.signal_config),
      .initial_frequency_hz = NCO_INITIAL_FREQUENCY_HZ,
      .dac_amplitude = DPLL_INITIAL_DAC_AMPLITUDE,
      .dac_dc_offset = DPLL_INITIAL_DAC_DC_OFFSET,
      .pi_kp_hz_per_rad = g_dpll_runtime_control.pi_kp_hz_per_rad,
      .pi_ki_hz_per_rad_s = g_dpll_runtime_control.pi_ki_hz_per_rad_s,
      .pi_max_correction_hz = DPLL_PI_DEFAULT_MAX_CORRECTION_HZ,
      .output_ready = NULL, /* 当前应用无需订阅内部输出块。 */
      .output_context = NULL,
  };

  if (!DPLL_App_Init(&dpll_app_config) ||
      !DPLL_App_SetPhaseCalibration(
          DPLL_PHASE_FIT_INTERCEPT_DEG, DPLL_PHASE_FIT_SLOPE_DEG_PER_KHZ) ||
      !DPLL_App_SetLockedPhaseDifferenceDegrees(DPLL_INITIAL_LOCK_PHASE_DEG) ||
      !DPLL_App_SetOutputWaveform(DPLL_INITIAL_OUTPUT_WAVEFORM) ||
      !DPLL_App_SetOutputFrequencyMultiplier(DPLL_INITIAL_OUTPUT_FREQUENCY_MULTIPLIER) ||
      !DPLL_App_Enable(true) ||
      !DPLL_App_Start())
  {
    Error_Handler();
  }

  /* 保存包含实际 ADC/DAC 更新率和处理块周期的配置结果。 */
  g_dpll_runtime_control.signal_config_success =
      DPLL_App_GetSignalInfo(&g_dpll_runtime_control.signal_info);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    DPLL_App_ProcessRuntimeControl(&g_dpll_runtime_control);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, DPLL_App_IsLocked() ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
