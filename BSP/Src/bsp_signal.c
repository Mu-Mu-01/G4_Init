/* BSP 层：STM32H723ZGT6 的 ADC1/PA6、DAC1/PA4、TIM1/TIM8 绑定与 DMA 内存。 */
/*
 * 使用本 BSP 前需要准备的外设配置
 *
 * 1. RCC 与时钟
 *    - 系统时钟、APB2 和 ADC 内核时钟必须已经配置完成。
 *    - TIM1、TIM8 使用的定时器内核时钟必须与 timer_clock_hz 的计算一致。
 *    - 修改时钟树后，需要重新检查定时器实际频率和采样率误差。
 *
 * 2. GPIO
 *    - adc_dac引脚配置为模拟模式。
 *    - GPIOA、GPIOH 时钟必须打开。
 *
 * 3. DMA1
 *    - DMA1_Stream0 绑定 ADC1，方向为外设到存储器。
 *    - DMA1_Stream1 绑定 DAC1 Channel 1，方向为存储器到外设。
 *    - 两个 Stream 均使用半字对齐、存储器递增和 Circular 循环模式。
 *    - ADC 和 DAC DMA 优先级使用 Very High，并打开 Stream0、Stream1 中断。
 *
 * 4. ADC1
 *    - 分辨率为 16 bit，单通道，采样时间使用当前工程配置。
 *    - 禁止连续转换，外部触发源为 TIM8_TRGO，上升沿触发。
 *    - 数据管理使用 DMA Circular，ADC 采样块由 BSP 的 DMA 缓冲区接收。
 *
 * 5. DAC1 Channel 1
 *    - 输出引脚为 PA4，触发源为 TIM1_TRGO。
 *    - 打开输出缓冲，使用 DMA Circular 连续更新 DAC 数据。
 *    - DAC 输出采样率由 TIM1 主定时器和升采样倍数共同决定，不单独配置 DAC 定时器。
 *
 * 6. TIM1
 *    - 使用内部时钟，主触发输出选择 Update Event。
 *    - TIM1 是 ADC 与 DAC 共用的主定时器，运行时由 BSP 根据请求修改 PSC 和 ARR。
 *
 * 7. TIM8
 *    - 时钟源选择 ITR0，接收 TIM1_TRGO 的级联触发。
 *    - 主触发输出选择 Update Event，作为 ADC1 的外部触发源。
 *    - TIM8 的级联分频由 BSP 根据 ADC 采样率和 DAC 升采样倍数计算。
 *
 * 8. H7 内存与缓存
 *    - ADC、DAC DMA 缓冲区必须位于 DMA1 可访问的 D2 SRAM，且按 32 字节对齐。
 *    - .dma_buffer 段、MPU 非缓存属性和 D-Cache 一致性由工程平台代码维护。
 *    - BSP_Signal_PreparePlatform 应在外设初始化前调用，并保持 D2 SRAM 时钟开启。
 *
 * 9. 初始化调用约束
 *    - BSP_Signal_InitPeripherals 使用本文件中的默认实现初始化 GPIO、DMA、ADC、DAC、TIM1 和 TIM8。
 *    - 需要使用 CubeMX 修改硬件路由时，应同步修改对应 HAL MSP 回调、DMA 请求和中断向量。
 *    - 外设初始化完成后，才能调用 BSP_Signal_Configure 配置信号端口和运行时采样率。
 */
#include "bsp_signal.h"
#include "signal_port_stm32h7.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"

/* 对应 DPLL.ioc 的默认值：TIM1 = 275 MHz/(24+1)，TIM8 以 ITR0 再除以 11；
 * 启动前由 main 的 ADC 采样率与升采样倍数请求覆盖 PSC/ARR，路由仍由 CubeMX 保持。 */
#define BOARD_INPUT_RATE_HZ       1000000U  /* 默认 ADC 请求采样率，Hz */
#define BOARD_DAC_UPSAMPLING_FACTOR     11U  /* TIM8 对 TIM1 事件的严格整数分频 */
#define BOARD_OUTPUT_RATE_HZ (BOARD_INPUT_RATE_HZ * BOARD_DAC_UPSAMPLING_FACTOR) /* 由 ADC 率派生 */
#define BOARD_INPUT_BLOCK_SAMPLES     512U /* 每个输入数组容量，点数 */
#define BOARD_OUTPUT_BLOCK_SAMPLES   7168U /* 每个输出数组容量，可用点数另由配置确定 */
#define BOARD_REFERENCE_OFFSET          9U /* 默认 11 次触发及 DHR/DOR 一拍延迟的参考索引 */

/* 实际输入输出块覆盖相同时间，在运行时按 TIM8 分频计算并校验容量。 */
_Static_assert(BOARD_INPUT_BLOCK_SAMPLES % 16U == 0U &&
               BOARD_OUTPUT_BLOCK_SAMPLES % 16U == 0U, "DMA 块必须覆盖完整缓存行");

/* 链接脚本 .dma_buffer -> RAM_D2；MPU 将 0x30000000 的 32 KiB 设为不可缓存。
 * 每块独占缓存行，禁止放入 DMA1 无法访问的 DTCM；NOLOAD 输入块不预读。 */
static uint16_t adc_dma_buffer0[BOARD_INPUT_BLOCK_SAMPLES]
    __attribute__((section(".dma_buffer"), aligned(32))); /* ADC memory0 */
static uint16_t adc_dma_buffer1[BOARD_INPUT_BLOCK_SAMPLES]
    __attribute__((section(".dma_buffer"), aligned(32))); /* ADC memory1 */
static uint16_t nco_dma_buffer0[BOARD_OUTPUT_BLOCK_SAMPLES]
    __attribute__((section(".dma_buffer"), aligned(32))); /* DAC memory0 */
static uint16_t nco_dma_buffer1[BOARD_OUTPUT_BLOCK_SAMPLES]
    __attribute__((section(".dma_buffer"), aligned(32))); /* DAC memory1 */
static const SignalPort *board_port; /* 初始化一次，之后只返回同一端口 */
volatile uint32_t g_icache_enabled;   /* I-Cache 状态供调试器观察 */

/**
 * @brief 准备信号流需要的缓存、DMA 内存时钟与周期计数器。
 * @param 无。
 * @retval 无。
 * @note HAL_Init 后、MX 外设初始化前调用；不启动信号流。
 */
void BSP_Signal_PreparePlatform(void)
{
    SCB_EnableICache();
    g_icache_enabled = ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) ? 1U : 0U;
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 绑定板级资源并按项目请求配置共同主定时器的采样链。
 * @param requested 项目采样请求，复制参数并借用可选自定义地址。
 * @retval 静态端口地址；NULL 表示参数非法或端口仍在运行。
 * @note 所有 MX 初始化后主循环调用。后续运行中配置由 App 的停流接口完成。
 */
const SignalPort *BSP_Signal_Configure(const SignalStreamConfig *requested)
{
    /* 板级绑定集中于此，可复用驱动和算法中不出现这些外设实例。 */
    const STM32H7_SignalConfig config = {
        .adc = &hadc1, .dac = &hdac1,
        .input_timer = &htim8, .output_timer = &htim1,
        .dac_channel = DAC_CHANNEL_1,
        .input_buffers = {adc_dma_buffer0, adc_dma_buffer1},
        .output_buffers = {nco_dma_buffer0, nco_dma_buffer1},
        .input_block_samples = BOARD_INPUT_BLOCK_SAMPLES,
        .output_block_samples = BOARD_OUTPUT_BLOCK_SAMPLES,
        .input_rate_hz = BOARD_INPUT_RATE_HZ, .output_rate_hz = BOARD_OUTPUT_RATE_HZ,
        .reference_sample_offset = BOARD_REFERENCE_OFFSET,
        .timer_clock_hz = HAL_RCC_GetPCLK2Freq() *
            ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) == 0U ? 1U : 2U),
        .input_capacity_samples = BOARD_INPUT_BLOCK_SAMPLES,
        .output_capacity_samples = BOARD_OUTPUT_BLOCK_SAMPLES,
    };
    /* 本工程 TIMPRE=0；变更时钟树时必须重新核对 APB 定时器内核时钟。 */
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0U || requested == NULL) { return NULL; }
    /* 保留现有优先级：DAC 重填必须能抢占较长的 ADC IQ/PI 处理。 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0U, 0U);
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1U, 0U);
    if (board_port == NULL) { board_port = STM32H7_SignalPort_Init(&config); }
    if (board_port == NULL || !board_port->ops->configure(board_port->context,
        requested, false, NULL)) { return NULL; }
    return board_port;
}

/**
 * @brief 获取板级端口，首次调用时使用默认采样配置。
 * @param 无。
 * @retval 已绑定的静态端口；NULL 表示默认配置未能应用。
 * @note 与旧装配接口兼容；自定义采样参数应调用 BSP_Signal_Configure。
 */
const SignalPort *BSP_Signal_GetPort(void)
{
    SignalStreamConfig defaults; /* 首次调用的默认项目配置。 */
    if (board_port != NULL) { return board_port; }
    BSP_Signal_GetDefaultConfig(&defaults);
    return BSP_Signal_Configure(&defaults);
}

/**
 * @brief  开启信号链所需 GPIO 端口时钟。
 * @param  无。
 * @retval 无。
 * @note   只开启端口时钟；ADC/DAC 模拟引脚由对应 HAL 初始化回调配置。
 */
static void BSP_Signal_InitGPIO(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
 * @brief  开启信号链 DMA 控制器并配置 DMA 中断。
 * @param  无。
 * @retval 无。
 * @note   必须在 ADC/DAC HAL 初始化前完成；DMA 请求和数据流由外设 HAL 回调绑定。
 */
static void BSP_Signal_InitDMA(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
 * @brief  配置 ADC1 的采样分辨率、外部触发和规则通道。
 * @param  无。
 * @retval 无。
 * @note   PA6 使用 ADC1_INP3；采样由 TIM8_TRGO 触发，DMA 使用循环模式。
 */
static void BSP_Signal_InitADC1(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef channel = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.Oversampling.Ratio = 1U;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    channel.Channel = ADC_CHANNEL_3;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0;
    channel.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  配置 DAC1 通道 1 的触发源和输出缓冲。
 * @param  无。
 * @retval 无。
 * @note   PA4 使用 DAC1_OUT1；DAC 由 TIM1_TRGO 触发并通过 DMA 循环输出。
 */
static void BSP_Signal_InitDAC1(void)
{
    DAC_ChannelConfTypeDef channel = {0};

    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        Error_Handler();
    }

    channel.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    channel.DAC_Trigger = DAC_TRIGGER_T1_TRGO;
    channel.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    channel.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    channel.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &channel, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  配置 TIM1 作为 ADC/DAC 共享的主触发定时器。
 * @param  无。
 * @retval 无。
 * @note   运行时 BSP 会根据采样率重写 PSC/ARR；此处只建立基础时钟和 TRGO 路由。
 */
static void BSP_Signal_InitTIM1(void)
{
    TIM_ClockConfigTypeDef clock_source = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0U;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 25U - 1U;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &clock_source) != HAL_OK)
    {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  配置 TIM8 作为 TIM1 主触发的级联分频定时器。
 * @param  无。
 * @retval 无。
 * @note   TIM8 使用 ITR0 接收 TIM1_TRGO；运行时由 BSP 更新级联分频。
 */
static void BSP_Signal_InitTIM8(void)
{
    TIM_ClockConfigTypeDef clock_source = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0U;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 11U - 1U;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0U;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }

    clock_source.ClockSource = TIM_CLOCKSOURCE_ITR0;
    if (HAL_TIM_ConfigClockSource(&htim8, &clock_source) != HAL_OK)
    {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &master) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  使用 BSP 内置默认值初始化信号链所需片上外设。
 * @param  无。
 * @retval 无。
 * @note   应在 HAL_Init、SystemClock_Config 完成后调用；不调用 CubeMX 的
 *         MX_*_Init 函数。修改本函数中的默认值或外设路由时，应同步检查
 *         对应 HAL MSP 回调、DMA 请求和中断向量。调用后才允许配置信号端口。
 */
void BSP_Signal_InitPeripherals(void)
{
    BSP_Signal_InitGPIO();
    BSP_Signal_InitDMA();
    BSP_Signal_InitADC1();
    BSP_Signal_InitDAC1();
    BSP_Signal_InitTIM1();
    BSP_Signal_InitTIM8();
}

/**
 * @brief 为项目提供与原工程相同的默认采样配置。
 * @param config 非空输出地址。
 * @retval 无。
 * @note 主循环调用，填默认值但不访问硬件或启动采样。
 */
void BSP_Signal_GetDefaultConfig(SignalStreamConfig *config)
{
    if (config == NULL) { return; }
    *config = (SignalStreamConfig){
        .adc_sample_rate_hz = BOARD_INPUT_RATE_HZ,
        .dac_upsampling_factor = BOARD_DAC_UPSAMPLING_FACTOR,
        .dac_output_enabled = true, .adc_block_samples = BOARD_INPUT_BLOCK_SAMPLES,
        .reference_sample_offset = UINT32_MAX,
    };
}
