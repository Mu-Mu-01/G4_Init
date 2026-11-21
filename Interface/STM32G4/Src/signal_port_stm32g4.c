/* Interface 平台适配层：G4 循环 DMA 半传输/全传输映射为两个逻辑块。 */
#include "signal_port_stm32g4.h"
#include "timer_frequency.h"

/* 双数据模式先播放两个预装码，相位参考按两次触发的数字流水线计算。 */
#define DAC_PRELOAD_SAMPLES 2U

typedef struct
{
    STM32G4_SignalConfig hw;             /* 静态硬件资源和内存绑定。 */
    SignalOutputCallback refill;         /* NCO 重填入口。 */
    SignalInputCallback receive;          /* ADC 数据入口。 */
    SignalErrorCallback error;            /* 流错误通知入口。 */
    void *output_context;                /* 输出订阅者上下文。 */
    void *input_context;                 /* 输入及错误订阅者上下文。 */
    uint64_t sequence;                   /* 从 0 开始的 ADC 块号，仅由 ADC 中断递增。 */
    volatile bool output_armed;          /* 主循环发布、DMA 中断读取的装载状态。 */
    volatile bool input_running;         /* 主循环发布、DMA 中断读取的接收状态。 */
    bool adc_calibrated;                 /* HAL 校准成功后置位，跨停启复用。 */
} SignalState;

static SignalState stream;               /* 单实例，外设 IRQ 按 HAL 句柄过滤。 */
static SignalPort port;                  /* 平台无关端口和已应用的工作参数。 */

/**
 * @brief 保存中断屏蔽状态并进入临界区。
 * @param context 接口上下文，此操作无需访问。
 * @retval 进入前的 PRIMASK。
 * @note 可嵌套，调用方须用配对令牌退出。
 */
static uint32_t EnterCritical(void *context)
{
    (void)context;
    const uint32_t token = __get_PRIMASK(); /* 保留调用者原有屏蔽状态。 */
    __disable_irq();
    __DMB();
    return token;
}

/**
 * @brief 恢复进入临界区前的中断屏蔽状态。
 * @param context 接口上下文，此操作无需访问。
 * @param token 对应的 EnterCritical 返回值。
 * @retval 无。
 * @note 先发布临界区数据，再恢复中断。
 */
static void ExitCritical(void *context, uint32_t token)
{
    (void)context;
    __DMB();
    __set_PRIMASK(token);
}

/**
 * @brief 发布共享数据前执行内存屏障。
 * @param context 接口上下文，此操作无需访问。
 * @retval 无。
 * @note 可在主循环或中断中调用。
 */
static void MemoryBarrier(void *context)
{
    (void)context;
    __DMB();
}

/**
 * @brief 读取内核周期计数。
 * @param context 接口上下文，此操作无需访问。
 * @retval 32 位回绕的 CPU 周期数。
 * @note BSP 负责使能 DWT。
 */
static uint32_t Cycles(void *context)
{
    (void)context;
    return DWT->CYCCNT;
}

/**
 * @brief 把一个 NCO 渲染槽打包到对应 DMA 半块。
 * @param index 半块编号 0 或 1。
 * @retval 无。
 * @note 每字低半字先输出、高半字后输出。仅访问尚未启动或已经消费的 DMA 半块。
 */
static void PackOutput(uint32_t index)
{
    const uint32_t count = port.output_block_samples; /* 当前半块的输出点数。 */
    uint16_t *destination = stream.hw.dac_ring + index * count; /* DMA 已消费的半块。 */
    const uint16_t *source = port.output_buffers[index]; /* NCO 的 12 位右对齐输出。 */
    if (source != destination)
    {
        /* 自定义渲染槽保持原有的 12 位截取语义；默认槽已在 DMA 环内。 */
        for (uint32_t i = 0U; i < count; ++i)
        {
            destination[i] = source[i] & DAC_DHR12R1_DACC1DHR_Msk;
        }
    }
    __DMB();
}

/**
 * @brief 接收已完成的 DMA 半块，并向 App 发布递增序号。
 * @param adc HAL 回调所属的 ADC 句柄。
 * @param index 已完成半块，0 或 1。
 * @retval 无。
 * @note 中断上下文，只在当前块有效期间使用输入地址。
 */
static void ReceiveHalf(ADC_HandleTypeDef *adc, uint32_t index)
{
    if (adc != stream.hw.adc || !stream.input_running)
    {
        return;
    }
    const uint64_t sequence = stream.sequence++; /* 与同周期输出块对应。 */
    const uint16_t *samples = &stream.hw.adc_ring[index * port.input_block_samples];
    stream.receive(stream.input_context, samples, port.input_block_samples, sequence);

    /* 内部输出由 ADC 节拍推动，先使用当前参考，再重填已用槽。 */
    if (!port.stream_info.dac_output_enabled && stream.output_armed)
    {
        stream.refill(stream.output_context, port.output_buffers[index], port.output_block_samples);
    }
}

/**
 * @brief 发布 ADC 环的前半块。
 * @param adc HAL 回调所属的 ADC 句柄。
 * @retval 无。
 * @note 由 ADC DMA 半传输中断调用。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc)
{
    ReceiveHalf(adc, 0U);
}

/**
 * @brief 发布 ADC 环的后半块。
 * @param adc HAL 回调所属的 ADC 句柄。
 * @retval 无。
 * @note 由 ADC DMA 全传输中断调用。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    ReceiveHalf(adc, 1U);
}

/**
 * @brief 重填已经由 DAC DMA 读完的半块。
 * @param dac HAL 回调所属的 DAC 句柄。
 * @param index 已完成半块，0 或 1。
 * @retval 无。
 * @note 中断上下文。回填和打包须在 DMA 返回该半块前完成。
 */
static void RefillHalf(DAC_HandleTypeDef *dac, uint32_t index)
{
    if (dac == stream.hw.dac && stream.output_armed)
    {
        stream.refill(stream.output_context, port.output_buffers[index], port.output_block_samples);
        PackOutput(index);
    }
}

/**
 * @brief 重填 DAC 环的前半块。
 * @param dac HAL 回调所属的 DAC 句柄。
 * @retval 无。
 * @note 由 DAC DMA 半传输中断调用。
 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *dac)
{
    RefillHalf(dac, 0U);
}

/**
 * @brief 重填 DAC 环的后半块。
 * @param dac HAL 回调所属的 DAC 句柄。
 * @retval 无。
 * @note 由 DAC DMA 全传输中断调用。
 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *dac)
{
    RefillHalf(dac, 1U);
}

/**
 * @brief 将 ADC 溢出或 DMA 故障转发给输入订阅者。
 * @param adc HAL 回调所属的 ADC 句柄。
 * @retval 无。
 * @note 中断上下文，故障由应用决定何时恢复。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    if (adc == stream.hw.adc && stream.input_running)
    {
        stream.error(stream.input_context, SIGNAL_ERROR_INPUT, adc->ErrorCode);
    }
}

/**
 * @brief 将 DAC DMA 故障转发给输入订阅者。
 * @param dac HAL 回调所属的 DAC 句柄。
 * @retval 无。
 * @note 输入已装载后才启用主触发，此时错误回调有效。
 */
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *dac)
{
    if (dac == stream.hw.dac && stream.output_armed && stream.input_running)
    {
        stream.error(stream.input_context, SIGNAL_ERROR_OUTPUT_UNDERRUN, dac->ErrorCode);
    }
}

/**
 * @brief 转发 DAC DMA 下溢状态。
 * @param dac HAL 回调所属的 DAC 句柄。
 * @retval 无。
 * @note 由 CubeMX 生成的 TIM6/DAC 中断处理程序经 HAL 转发。
 */
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *dac)
{
    HAL_DAC_ErrorCallbackCh1(dac);
}

/**
 * @brief 先停止主触发，再释放输出 DMA。
 * @param context 信号端口私有状态。
 * @retval true 停止成功，false HAL 停止失败。
 * @note 主循环调用。允许在已停止时调用，便于故障回滚。
 */
static bool StopOutput(void *context)
{
    SignalState *state = context; /* 当前平台单例。 */
    const HAL_StatusTypeDef timer_status = HAL_TIM_Base_Stop(state->hw.output_timer);
    state->output_armed = false;
    if (!port.stream_info.dac_output_enabled)
    {
        return timer_status == HAL_OK;
    }
    /* DMA 出错后 HAL 可能已置为 READY，此时 Stop_DMA 的 abort 会报无传输。 */
    const bool dma_busy = HAL_DMA_GetState(state->hw.dac->DMA_Handle1) == HAL_DMA_STATE_BUSY;
    const HAL_StatusTypeDef dac_status = HAL_DAC_Stop_DMA(state->hw.dac, DAC_CHANNEL_1);
    __HAL_DAC_CLEAR_FLAG(state->hw.dac, DAC_FLAG_DMAUDR1);
    return timer_status == HAL_OK && (dac_status == HAL_OK || !dma_busy);
}

/**
 * @brief 复位停流定时器的计数器及预分频器相位。
 * @param timer 已停止的 HAL 定时器句柄。
 * @retval 无。
 * @note UG 会产生 TRGO，须在 ADC/DAC 装载前执行。
 */
static void ResetTimer(TIM_HandleTypeDef *timer)
{
    timer->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(timer, 0U);
    __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
}

/**
 * @brief 预装两个输出半块和 DAC 流水线首码。
 * @param context 信号端口私有状态。
 * @param preload 首个触发前的 12 位 DAC 码。
 * @param refill DMA 半块重填回调。
 * @param callback_context 输出订阅者上下文。
 * @retval true 装载成功，false 参数或 HAL 装载失败。
 * @note 定时器仍保持停止。双数据 DMA 的总传输字数等于单个逻辑块的样本数。
 */
static bool ArmOutput(void *context, uint16_t preload, SignalOutputCallback refill,
                      void *callback_context)
{
    SignalState *state = context; /* 当前平台单例。 */
    if (state->output_armed || refill == NULL || port.output_block_samples == 0U ||
        preload > DAC_DHR12R1_DACC1DHR_Msk)
    {
        return false;
    }
    /* 失败重试和停启都从相同相位起步。先触发主 UG，再复位从节拍。 */
    ResetTimer(state->hw.output_timer);
    ResetTimer(state->hw.input_timer);

    state->refill = refill;
    state->output_context = callback_context;
    if (port.stream_info.dac_output_enabled)
    {
        const uint32_t pair = preload | ((uint32_t)preload << DAC_DHR12R1_DACC1DHRB_Pos);
        PackOutput(0U);
        PackOutput(1U);
        __HAL_DAC_CLEAR_FLAG(state->hw.dac, DAC_FLAG_DMAUDR1);
        state->hw.dac->ErrorCode = HAL_DAC_ERROR_NONE;
        if (HAL_DAC_Start_DMA(state->hw.dac, DAC_CHANNEL_1, (uint32_t *)state->hw.dac_ring,
                              port.output_block_samples, DAC_ALIGN_12B_R) != HAL_OK ||
            HAL_DAC_SetValue(state->hw.dac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pair) != HAL_OK)
        {
            (void)StopOutput(context);
            return false;
        }
    }
    state->output_armed = true;
    return true;
}

/**
 * @brief 最后启动主定时器，开始共用触发时间轴。
 * @param context 信号端口私有状态。
 * @retval true 启动成功，false 未装载或 HAL 启动失败。
 * @note 主循环且中断开启时调用。低倍数先单步预装 DAC，再启动 TIM1 和 TIM8。
 */
static bool StartOutput(void *context)
{
    SignalState *state = context; /* 当前平台单例。 */
    if (!state->output_armed || !state->input_running)
    {
        return false;
    }
    if (port.stream_info.dac_output_enabled &&
        port.output_samples_per_input_sample <= DAC_PRELOAD_SAMPLES)
    {
        /* RM0440 22.4.8：第一次外部触发装入一对数据并请求一个 DMA 字，
         * 第二次播放高半字。TIM1 停止时用 TIM8 的 UG 单步走完预装码，
         * 使第一个周期性触发播放渲染样本 0。DORSTAT 同时用于停启后的半字对齐。 */
        DAC_HandleTypeDef *dac = state->hw.dac;
        const uint32_t expected_words = port.output_block_samples - 1U; /* 首字搬完后的 CNDTR。 */
        bool primed = false; /* 已搬运首字且高半字预装码已播放。 */
        for (unsigned pulse = 0U; pulse < DAC_PRELOAD_SAMPLES + 1U; ++pulse)
        {
            state->hw.output_timer->Instance->EGR = TIM_EGR_UG;
            HAL_Delay(1U); /* 仅在主循环启流时等待外设和 DMA 完成本次单步。 */
            if (__HAL_DMA_GET_COUNTER(dac->DMA_Handle1) == expected_words &&
                (dac->Instance->SR & DAC_SR_DORSTAT1) != 0U)
            {
                primed = true;
                break;
            }
        }
        if (!primed)
        {
            (void)StopOutput(context);
            return false;
        }
    }
    if (port.output_samples_per_input_sample == 1U)
    {
        /* RM0440 表 267 及 TIMx_CR2.MMS：在 Reset 从模式下，每个 ITR5
         * 上升沿产生一次复位，并经 Reset TRGO 触发 ADC。先预装 DAC，再
         * 接通转发，避免把软件预装脉冲计入 ADC 第一块。此处 TIM8 仍停表。 */
        TIM_TypeDef *input = state->hw.input_timer->Instance; /* ADC 触发转发定时器。 */
        MODIFY_REG(input->CR2, TIM_CR2_MMS, TIM_TRGO_RESET);
        MODIFY_REG(input->SMCR, TIM_SMCR_SMS, TIM_SLAVEMODE_RESET);
    }
    if (HAL_TIM_Base_Start(state->hw.input_timer) != HAL_OK ||
        HAL_TIM_Base_Start(state->hw.output_timer) != HAL_OK)
    {
        (void)StopOutput(context);
        return false;
    }
    return true;
}

/**
 * @brief 停止 ADC 从定时器并释放输入 DMA。
 * @param context 信号端口私有状态。
 * @retval true 停止成功，false HAL 停止失败。
 * @note 应先停主触发。关闭回调门控后清理 ADC 的溢出中断及状态。
 */
static bool StopInput(void *context)
{
    SignalState *state = context; /* 当前平台单例。 */
    const HAL_StatusTypeDef timer_status = HAL_TIM_Base_Stop(state->hw.input_timer);
    state->input_running = false;
    const HAL_StatusTypeDef adc_status = HAL_ADC_Stop_DMA(state->hw.adc);
    __HAL_ADC_DISABLE_IT(state->hw.adc, ADC_IT_OVR);
    __HAL_ADC_CLEAR_FLAG(state->hw.adc, ADC_FLAG_OVR);
    if (port.output_samples_per_input_sample == 1U)
    {
        /* 主触发已停止，恢复 CubeMX 的分频配置，供重新装载或切换倍数。 */
        TIM_TypeDef *input = state->hw.input_timer->Instance; /* 已停止的 ADC 触发定时器。 */
        MODIFY_REG(input->SMCR, TIM_SMCR_SMS, TIM_SLAVEMODE_EXTERNAL1);
        MODIFY_REG(input->CR2, TIM_CR2_MMS, TIM_TRGO_UPDATE);
    }
    return timer_status == HAL_OK && adc_status == HAL_OK;
}

/**
 * @brief 装载 ADC 环并使能 ADC 等待外部触发，两个定时器由 StartOutput 顺序启动。
 * @param context 信号端口私有状态。
 * @param receive 输入块回调。
 * @param error 输入或输出故障回调。
 * @param callback_context 输入及故障订阅者上下文。
 * @retval true 装载成功，false 参数、校准或 HAL 启动失败。
 * @note 主循环调用。单端 ADC 在首次启动前校准，主触发尚未启动。
 */
static bool StartInput(void *context, SignalInputCallback receive,
                       SignalErrorCallback error, void *callback_context)
{
    SignalState *state = context; /* 当前平台单例。 */
    if (state->input_running || !state->output_armed || receive == NULL || error == NULL)
    {
        return false;
    }
    if (!state->adc_calibrated)
    {
        if (HAL_ADCEx_Calibration_Start(state->hw.adc, ADC_SINGLE_ENDED) != HAL_OK)
        {
            return false;
        }
        state->adc_calibrated = true;
    }
    state->receive = receive;
    state->error = error;
    state->input_context = callback_context;
    state->sequence = 0U;
    if (HAL_ADC_Start_DMA(state->hw.adc, (uint32_t *)state->hw.adc_ring,
                          2U * port.input_block_samples) != HAL_OK)
    {
        (void)StopInput(context);
        return false;
    }
    state->input_running = true;
    return true;
}

/**
 * @brief 写入停流定时器分频，并初始化预分频器和计数器。
 * @param timer 已停止且未连接运行中数据流的 HAL 定时器句柄。
 * @param plan 已验证的 16 位 PSC/ARR 参数。
 * @retval 无。
 * @note UG 会触发 TRGO。应先应用主定时器，再清零从定时器，且 ADC/DAC 尚未装载。
 */
static void ApplyTimerPlan(TIM_HandleTypeDef *timer, const TimerFrequencyPlan *plan)
{
    timer->Init.Prescaler = plan->prescaler;
    timer->Init.Period = plan->autoreload;
    __HAL_TIM_SET_PRESCALER(timer, plan->prescaler);
    __HAL_TIM_SET_AUTORELOAD(timer, plan->autoreload);
    ResetTimer(timer);
}

/**
 * @brief 验证完整 DMA 环位于 G474 的 DMA 可访问普通 SRAM。
 * @param buffer 环形传输起始地址。
 * @param bytes 包含两个半块的长度，字节。
 * @retval true 地址按字对齐且全段位于 SRAM1/2，false 否则。
 * @note 使用普通 SRAM 的 0x20000000 映射。
 */
static bool DmaMemoryIsAccessible(const void *buffer, uintptr_t bytes)
{
    const uintptr_t start = (uintptr_t)buffer; /* 以整数校验整个地址范围。 */
    const uintptr_t end = SRAM2_BASE + 16U * 1024U; /* 16 KiB SRAM2 尾后地址。 */
    return start >= SRAM1_BASE && start < end && start % sizeof(uint32_t) == 0U &&
        bytes <= end - start;
}

/**
 * @brief 校验两块 NCO 输出缓冲区的边界和独占性。
 * @param buffers 两个候选逻辑块，至少长度为当前配置的样本数。
 * @param count 单个逻辑块样本数。
 * @retval true 地址可按半字访问且各有效区间互不重叠，false 否则。
 * @note 仅校验调用方提供的独立渲染槽。默认槽按本次块长直接划分 DMA 环。
 */
static bool OutputBuffersAreValid(uint16_t *const buffers[2], uint32_t count)
{
    const uintptr_t addresses[] = {(uintptr_t)buffers[0], (uintptr_t)buffers[1],
        (uintptr_t)stream.hw.adc_ring, (uintptr_t)stream.hw.dac_ring}; /* 各区间首地址。 */
    const uintptr_t sizes[] = {count * sizeof(uint16_t), count * sizeof(uint16_t),
        2U * stream.hw.input_capacity_samples * sizeof(uint16_t),
        stream.hw.output_capacity_samples * sizeof(uint32_t)}; /* 各区间字节数。 */
    for (size_t i = 0; i < 2U; ++i)
    {
        if (addresses[i] == 0U || addresses[i] % _Alignof(uint16_t) != 0U ||
            sizes[i] > UINTPTR_MAX - addresses[i])
        {
            return false;
        }
        for (size_t j = i + 1U; j < 4U; ++j)
        {
            if (addresses[j] <= UINTPTR_MAX - sizes[j] &&
                addresses[i] < addresses[j] + sizes[j] && addresses[j] < addresses[i] + sizes[i])
            {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief 计算同步采样率和块长，在停流时应用分频与缓冲区绑定。
 * @param context 信号端口私有状态。
 * @param config 请求采样率、倍数、块长和可选输出缓冲地址。
 * @param validate_only true 只校验，可在运行中调用；false 停流时应用。
 * @param info 可选输出，成功时返回实际速率和块周期。
 * @retval true 校验或配置成功，false 参数或外设状态不合规。
 * @note 主循环调用。先验证整个配置，再更新硬件和端口。
 */
static bool Configure(void *context, const SignalStreamConfig *config,
                      bool validate_only, SignalStreamInfo *info)
{
    SignalState *state = context; /* 当前平台单例。 */
    if (config == NULL || config->adc_block_samples == 0U ||
        config->adc_block_samples > state->hw.input_capacity_samples ||
        config->dac_upsampling_factor == 0U)
    {
        return false;
    }
    const uint32_t ratio = config->dac_upsampling_factor; /* TIM1 的整数分频。 */
    const uint64_t count = (uint64_t)config->adc_block_samples * ratio; /* 输出每块样本数。 */
    const double requested_dac_hz = config->adc_sample_rate_hz * ratio; /* 主触发目标率。 */
    const bool custom = config->output_buffers[0] != NULL || config->output_buffers[1] != NULL;
    const uint32_t offset = config->reference_sample_offset != UINT32_MAX ?
        config->reference_sample_offset :
        ratio - 1U - (config->dac_output_enabled && ratio > DAC_PRELOAD_SAMPLES ?
                      DAC_PRELOAD_SAMPLES : 0U); /* ADC 在每组的最后一个触发事件采样。 */
    if (count > state->hw.output_capacity_samples ||
        (custom && count > config->output_capacity_samples) || offset >= ratio ||
        (config->dac_output_enabled && count % 2U != 0U) ||
        (custom && !OutputBuffersAreValid(config->output_buffers, (uint32_t)count)))
    {
        return false;
    }

    TimerFrequencyPlan master, divider; /* TIM8 内部时钟分频与 TIM1 外部计数分频。 */
    if (!TimerFrequency_Find(state->hw.timer_clock_hz, requested_dac_hz, UINT16_MAX, &master) ||
        !TimerFrequency_FindExactDivider(master.actual_hz, config->adc_sample_rate_hz, ratio,
                                         UINT16_MAX, &divider) ||
        master.autoreload == 0U || (ratio > 1U && divider.autoreload == 0U))
    {
        return false;
    }

    const SignalStreamInfo actual = {
        .adc_sample_rate_hz = divider.actual_hz,
        .dac_base_sample_rate_hz = divider.actual_hz,
        .dac_output_sample_rate_hz = master.actual_hz,
        .adc_error_ppm = divider.error_ppm, .dac_error_ppm = master.error_ppm,
        .block_period_s = config->adc_block_samples / divider.actual_hz,
        .output_block_samples = (uint32_t)count,
        .output_samples_per_input_sample = ratio,
        .dac_output_enabled = config->dac_output_enabled,
    }; /* 把实际速率和误差交给上层。 */
    if (!validate_only)
    {
        if (state->input_running || state->output_armed ||
            ((state->hw.input_timer->Instance->CR1 | state->hw.output_timer->Instance->CR1) &
             TIM_CR1_CEN) != 0U)
        {
            return false;
        }
        ApplyTimerPlan(state->hw.output_timer, &master);
        if (ratio == 1U)
        {
            /* 同速模式由 ITR5 的复位事件直接转发触发，ARR 不参与分频。 */
            divider.autoreload = UINT16_MAX;
        }
        ApplyTimerPlan(state->hw.input_timer, &divider);
        port.input_rate_hz = actual.adc_sample_rate_hz;
        port.output_rate_hz = actual.dac_output_sample_rate_hz;
        port.input_block_samples = config->adc_block_samples;
        port.output_block_samples = actual.output_block_samples;
        port.output_samples_per_input_sample = ratio;
        port.reference_sample_offset = offset;
        port.output_buffers[0] = custom ? config->output_buffers[0] : state->hw.dac_ring;
        port.output_buffers[1] = custom ? config->output_buffers[1] : state->hw.dac_ring + (uint32_t)count;
        port.stream_info = actual;
    }
    if (info != NULL)
    {
        *info = actual;
    }
    return true;
}

static const SignalPortOps operations = { /* SignalPort 接口契约与 G4 实现绑定。 */
    .enter_critical = EnterCritical, .exit_critical = ExitCritical,
    .memory_barrier = MemoryBarrier, .cycles = Cycles,
    .arm_output = ArmOutput, .start_output = StartOutput, .stop_output = StopOutput,
    .start_input = StartInput, .stop_input = StopInput, .configure = Configure,
};

/**
 * @brief 校验已初始化的外设与 DMA 资源，并绑定 G4 静态端口。
 * @param config 平台外设和缓冲区绑定描述，结构体会复制，地址在端口使用期间有效。
 * @retval 静态 SignalPort，NULL 表示绑定非法或信号流已装载。
 * @note 只校验和绑定。启动前还须经过 configure 应用工作分频。
 */
const SignalPort *STM32G4_SignalPort_Init(const STM32G4_SignalConfig *config)
{
    if (config == NULL || stream.input_running || stream.output_armed ||
        config->adc == NULL || config->dac == NULL ||
        config->input_timer == NULL || config->output_timer == NULL ||
        config->adc->Instance == NULL || config->dac->Instance == NULL ||
        config->input_timer->Instance == NULL || config->output_timer->Instance == NULL ||
        config->adc->DMA_Handle == NULL || config->dac->DMA_Handle1 == NULL ||
        config->timer_clock_hz == 0U || config->input_capacity_samples == 0U ||
        config->input_capacity_samples > UINT16_MAX / 2U ||
        config->output_capacity_samples == 0U || config->output_capacity_samples > UINT16_MAX)
    {
        return NULL;
    }

    const DMA_InitTypeDef *rx = &config->adc->DMA_Handle->Init; /* 输入 DMA 参数。 */
    const DMA_InitTypeDef *tx = &config->dac->DMA_Handle1->Init; /* 输出 DMA 参数。 */
    if (rx->Direction != DMA_PERIPH_TO_MEMORY || tx->Direction != DMA_MEMORY_TO_PERIPH ||
        rx->Mode != DMA_CIRCULAR || tx->Mode != DMA_CIRCULAR ||
        rx->MemInc != DMA_MINC_ENABLE || tx->MemInc != DMA_MINC_ENABLE ||
        rx->PeriphInc != DMA_PINC_DISABLE || tx->PeriphInc != DMA_PINC_DISABLE ||
        rx->PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD ||
        rx->MemDataAlignment != DMA_MDATAALIGN_HALFWORD ||
        tx->PeriphDataAlignment != DMA_PDATAALIGN_WORD || tx->MemDataAlignment != DMA_MDATAALIGN_WORD ||
        config->adc->Init.Resolution != ADC_RESOLUTION_12B ||
        config->adc->Init.ClockPrescaler != ADC_CLOCK_SYNC_PCLK_DIV4 ||
        config->adc->Init.DataAlign != ADC_DATAALIGN_RIGHT ||
        config->adc->Init.OversamplingMode != DISABLE ||
        config->adc->Init.ContinuousConvMode != DISABLE ||
        config->adc->Init.NbrOfConversion != 1U ||
        config->adc->Init.DMAContinuousRequests != ENABLE ||
        config->adc->Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_RISING ||
        (config->dac->Instance->MCR & DAC_MCR_DMADOUBLE1) == 0U ||
        (config->input_timer->Instance->CR2 & TIM_CR2_MMS) != TIM_TRGO_UPDATE ||
        (config->output_timer->Instance->CR2 & TIM_CR2_MMS) != TIM_TRGO_UPDATE ||
        config->input_timer->Init.CounterMode != TIM_COUNTERMODE_UP ||
        config->output_timer->Init.CounterMode != TIM_COUNTERMODE_UP ||
        config->input_timer->Init.RepetitionCounter != 0U ||
        config->output_timer->Init.RepetitionCounter != 0U ||
        !DmaMemoryIsAccessible(config->adc_ring,
                               2U * config->input_capacity_samples * sizeof(uint16_t)) ||
        !DmaMemoryIsAccessible(config->dac_ring,
                               config->output_capacity_samples * sizeof(uint32_t)))
    {
        return NULL;
    }
    stream = (SignalState){.hw = *config};
    port = (SignalPort){.ops = &operations, .context = &stream};
    return &port;
}
