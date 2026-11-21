/* BSP 层：STM32G474CBT6 板级信号链与 DMA 存储。
 * PA0 / ADC1_IN1 由 TIM1_TRGO 触发，PA4 / DAC1_OUT1 由 TIM8_TRGO 触发。
 * TIM8 走内部时钟，按 RM0440 Rev 9 表 267，TIM1 用 ITR5 对 TIM8 更新事件整数分频。
 * ADC 使用 DMA1 Channel 1 半字环形传输，DAC 使用 Channel 2 字宽双数据环形传输。
 */
#include "bsp_signal.h"
#include "bsp_signal_settings.h"
#include "signal_port_stm32g4.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"

#define BOARD_INPUT_CAPACITY 512U /* 输入半块预留点数，兼容运行期切换块长。 */
/* 至少保留原有的 4096 点；启动请求较大时按照配置的输出点数分配。 */
#define BOARD_REQUESTED_OUTPUT_SAMPLES (1ULL * DPLL_ADC_BLOCK_SAMPLES * DPLL_DAC_UPSAMPLING_FACTOR)
#define BOARD_OUTPUT_CAPACITY ((BOARD_REQUESTED_OUTPUT_SAMPLES > 4096U) ? BOARD_REQUESTED_OUTPUT_SAMPLES : 4096U)
_Static_assert(DPLL_ADC_BLOCK_SAMPLES > 0 && DPLL_ADC_BLOCK_SAMPLES <= BOARD_INPUT_CAPACITY,
               "ADC block must fit the input buffer");
_Static_assert(DPLL_DAC_UPSAMPLING_FACTOR > 0 && BOARD_OUTPUT_CAPACITY <= UINT16_MAX,
               "DAC DMA transfer must fit the hardware counter");

/* 普通 .bss 位于 SRAM1 起始区，两个 DMA 环按 HAL 参数对齐到 4 字节。 */
static _Alignas(uint32_t) uint16_t adc_ring[2U * BOARD_INPUT_CAPACITY];
/* G4 小端双数据模式：两个连续的 12 位码组成一个 DMA 字。 */
static _Alignas(uint32_t) uint16_t dac_ring[2U * BOARD_OUTPUT_CAPACITY];
static const SignalPort *board_port; /* 停流配置成功后发布的板级单例。 */

/**
 * @brief 使能内核周期计数器，供 App 和 NCO 记录执行周期。
 * @param 无。
 * @retval 无。
 * @note HAL_Init 后调用。STM32G474 的 DMA SRAM 无需缓存维护。
 */
void BSP_Signal_PreparePlatform(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 依次调用 CubeMX 的信号外设初始化入口。
 * @param 无。
 * @retval 无。
 * @note 供自行装配的入口使用。使用生成的 main 时，由 main 完成同一组初始化。
 */
void BSP_Signal_InitPeripherals(void)
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_DAC1_Init();
    MX_TIM8_Init();
}

/**
 * @brief 填充板级默认的采样请求。
 * @param config 默认配置输出地址，允许为 NULL。
 * @retval 无。
 * @note 主循环调用，无硬件访问。参考样本偏移交给端口按 DAC 流水线计算。
 */
void BSP_Signal_GetDefaultConfig(SignalStreamConfig *config)
{
    if (config != NULL)
    {
        *config = (SignalStreamConfig){
            .adc_sample_rate_hz = DPLL_ADC_SAMPLE_RATE_HZ,
            .dac_upsampling_factor = DPLL_DAC_UPSAMPLING_FACTOR,
            .dac_output_enabled = true,
            .adc_block_samples = DPLL_ADC_BLOCK_SAMPLES,
            .reference_sample_offset = UINT32_MAX,
        };
    }
}

/**
 * @brief 核对生成的触发与 DMA 配置，再绑定和配置板级信号端口。
 * @param requested 采样请求，内含的可选输出地址须在使用期间有效。
 * @retval 板级端口地址，或 NULL 表示外设路由、参数或启停状态不符合要求。
 * @note CubeMX 初始化完成后在主循环停流时调用，应用工作分频和两路 DMA 中断抢占优先级。
 */
const SignalPort *BSP_Signal_Configure(const SignalStreamConfig *requested)
{
    /* G4 的 APB2 分频为 1 时定时器时钟等于 PCLK2，否则为 2 倍。 */
    const uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq() *
        ((RCC->CFGR & RCC_CFGR_PPRE2_2) == 0U ? 1U : 2U);
    const STM32G4_SignalConfig hardware = {
        .adc = &hadc1, .dac = &hdac1,
        .input_timer = &htim1, .output_timer = &htim8,
        .timer_clock_hz = timer_clock_hz,
        .adc_ring = adc_ring, .dac_ring = dac_ring,
        .input_capacity_samples = BOARD_INPUT_CAPACITY,
        .output_capacity_samples = BOARD_OUTPUT_CAPACITY,
    }; /* 板级实例绑定，平台适配层通过参数访问。 */
    const SignalPort *candidate = board_port; /* 成功应用后才对外发布。 */

    if (requested == NULL || hadc1.Instance != ADC1 || hdac1.Instance != DAC1 ||
        htim1.Instance != TIM1 || htim8.Instance != TIM8 ||
        hadc1.DMA_Handle == NULL || hdac1.DMA_Handle1 == NULL ||
        hadc1.DMA_Handle->Instance != DMA1_Channel1 ||
        hdac1.DMA_Handle1->Instance != DMA1_Channel2 ||
        hadc1.DMA_Handle->Init.Request != DMA_REQUEST_ADC1 ||
        hdac1.DMA_Handle1->Init.Request != DMA_REQUEST_DAC1_CHANNEL1 ||
        hadc1.Init.ExternalTrigConv != ADC_EXTERNALTRIG_T1_TRGO ||
        (DAC1->CR & (DAC_CR_TSEL1 | DAC_CR_TEN1)) != DAC_TRIGGER_T8_TRGO ||
        (TIM1->SMCR & (TIM_SMCR_SMS | TIM_SMCR_TS)) !=
            (TIM_SLAVEMODE_EXTERNAL1 | TIM_TS_ITR5) ||
        (TIM8->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U)
    {
        return NULL;
    }
    if (candidate == NULL)
    {
        candidate = STM32G4_SignalPort_Init(&hardware);
    }
    if (candidate == NULL ||
        !candidate->ops->configure(candidate->context, requested, false, NULL))
    {
        return NULL;
    }
    /* DAC 重填和打包有半块截止时间，允许它抢占 ADC 的测相和环路处理。
     * CubeMX 初始化负责使能两路 DMA IRQ 并生成中断入口。 */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0U, 0U);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1U, 0U);

    board_port = candidate;
    return board_port;
}

/**
 * @brief 获取已配置的端口，首次调用时应用板级默认值。
 * @param 无。
 * @retval 板级端口地址，或 NULL 表示默认配置未能应用。
 * @note 必须在 CubeMX 外设初始化之后调用。
 */
const SignalPort *BSP_Signal_GetPort(void)
{
    SignalStreamConfig defaults; /* 首次配置用的板级请求。 */
    if (board_port == NULL)
    {
        BSP_Signal_GetDefaultConfig(&defaults);
        return BSP_Signal_Configure(&defaults);
    }
    return board_port;
}
