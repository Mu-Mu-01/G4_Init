/**
 * @file    signal_port_stm32h7.c
 * @brief   SignalPort 的 STM32H7 平台适配实现：把双缓冲、定时器与内核能力映射为通用流接口。
 *
 * 本文件把 SignalPort 契约落到 STM32H7 硬件上：ADC/DAC 双缓冲 + DMA、主从定时器
 * 触发、内核能力（PRIMASK 临界区、DMB 屏障、DWT 周期计数、D-Cache 维护）。
 *
 * 关键设计点：
 *  - HAL 弱回调没有用户参数，故用单实例 static stream 做路由，SignalPort.context
 *    即指向 &stream；操作表函数收到 context 后强转回 STM32H7_SignalState 使用。
 *  - 输入/输出块都遵循"先装载、后启动主触发"的时序，使 ADC 与 DAC 共享同一条
 *    触发时间轴，块序号才能一一对齐。
 *  - 缓存一致性：输出在交 DMA 前 Clean（写回），输入在 DMA 完成后 Invalidate
 *    （失效），均按完整 cache line 处理（BSP 保证 32 字节对齐）。
 *  - 输入块序号 next_sequence 每次启动归零，用于与 NCO 输出块序号对齐。
 */
#include "signal_port_stm32h7.h"
#include "adc_Doublebuffer.h"
#include "dac_Doublebuffer.h"
#include "timer_frequency_stm32h7.h"
#include <math.h>

/**
 * @brief  平台适配层的私有路由状态（单实例）。
 *
 * 由于 HAL 弱回调没有用户参数，无法把状态作为参数传入，因此用全局单例
 * stream 承载所有绑定信息，再由 SignalPort.context 指向它。
 */
typedef struct
{
    STM32H7_SignalConfig config;    /**< 绑定副本；指针目标由 BSP 持有 */
    SignalInputCallback receive;    /**< 借用输入事件处理函数 */
    SignalOutputCallback refill;    /**< 借用输出重填函数 */
    SignalErrorCallback error;      /**< 借用错误通知函数 */
    void *input_context;            /**< 输入与错误订阅者上下文 */
    void *output_context;           /**< 输出订阅者上下文 */
    const uint16_t *completed;      /**< HAL 状态更新期间暂存已完成输入块 */
    uint64_t next_sequence;         /**< 下一个输入块序号，每次启动归零 */
    uint64_t completed_sequence;    /**< 暂存完成块的时间轴序号 */
    bool input_running;             /**< 输入 DMA/从定时器已准备 */
    bool output_armed;              /**< 输出 DMA 已装载 */
    uint16_t *default_output[2];    /**< BSP 默认输出地址，切换自定义缓冲区后仍保留。 */
} STM32H7_SignalState;

static STM32H7_SignalState stream; /**< HAL 弱回调没有用户参数，单实例路由状态 */
static SignalPort port;            /**< 向上层发布的通用端口 */

/**
 * @brief  进入临界区：禁止中断并返回进入前的屏蔽状态。
 * @param  context 平台私有状态（本实现未使用）。
 * @retval 进入前的中断屏蔽状态（PRIMASK），供 ExitCritical 恢复。
 * @note   可嵌套，支持主循环/中断及嵌套调用。
 */
static uint32_t EnterCritical(void *context)
{
    uint32_t token = __get_PRIMASK(); /* 调用者原中断屏蔽状态 */
    (void)context;
    __disable_irq();
    return token;
}

/**
 * @brief  退出临界区：恢复进入前的中断屏蔽状态。
 * @param  context 平台私有状态（本实现未使用）。
 * @param  token   来自匹配的 EnterCritical 返回值。
 */
static void ExitCritical(void *context, uint32_t token)
{
    (void)context;
    __set_PRIMASK(token);
}

/**
 * @brief  发布数据前的数据内存屏障（DMB）。
 * @param  context 平台私有状态（本实现未使用）。
 */
static void MemoryBarrier(void *context)
{
    (void)context;
    __DMB();
}

/**
 * @brief  读取 CPU 周期计数器（DWT->CYCCNT）。
 * @param  context 平台私有状态（本实现未使用）。
 * @retval 当前周期计数值，按 uint32_t 回绕。
 */
static uint32_t Cycles(void *context)
{
    (void)context;
    return DWT->CYCCNT;
}

/**
 * @brief  把输出缓冲写回缓存，确保 DMA 读到最新渲染数据。
 * @param  buffer 输出缓冲首指针。
 * @note   BSP 保证完整 cache line 所有权；仅在 D-Cache 使能时执行。
 */
static void CleanOutput(uint16_t *buffer)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)buffer,
            (int32_t)(stream.config.output_block_samples * sizeof(uint16_t)));
    }
}

/**
 * @brief  使输入缓冲缓存失效，确保 CPU 读到 DMA 写入的最新数据。
 * @param  buffer 输入缓冲首指针。
 * @note   仅对已完成输入块失效；CPU 随后只读，禁止写回覆盖 DMA 接收数据。
 */
static void InvalidateInput(const uint16_t *buffer)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)buffer,
            (int32_t)(stream.config.input_block_samples * sizeof(uint16_t)));
    }
}

/**
 * @brief  输出 DMA 完成回调：把已消费的输出块交还给订阅者重填。
 * @param  dma   触发回调的 DMA 句柄。
 * @param  index 完成的双缓冲索引（0 或 1）。
 * @note   仅匹配绑定的 DAC DMA 句柄；重填完成后清理缓存写回。
 */
static void OutputComplete(DMA_HandleTypeDef *dma, uint32_t index)
{
    if (stream.output_armed && dma == stream.config.dac->DMA_Handle1)
    {
        uint16_t *buffer = stream.config.output_buffers[index]; /* 已消费块 */
        stream.refill(stream.output_context, buffer, stream.config.output_block_samples);
        CleanOutput(buffer);
    }
}
static void Output0Complete(DMA_HandleTypeDef *dma) { OutputComplete(dma, 0U); }
static void Output1Complete(DMA_HandleTypeDef *dma) { OutputComplete(dma, 1U); }

/**
 * @brief  输入 DMA 完成回调：登记完成块身份后，再走 HAL 原状态机。
 * @param  dma   触发回调的 DMA 句柄。
 * @param  index 完成的双缓冲索引（0 或 1）。
 * @note   HAL 最终会调用下方的 HAL_ADC_ConvCpltCallback，完成块在此期间
 *         通过 stream.completed / completed_sequence 暂存传递。
 */
static void InputComplete(DMA_HandleTypeDef *dma, uint32_t index)
{
    if (stream.input_running && dma == stream.config.adc->DMA_Handle)
    {
        stream.completed = stream.config.input_buffers[index];
        stream.completed_sequence = stream.next_sequence++;
        ADC_DMAConvCplt(dma);
        stream.completed = NULL;
    }
}
static void Input0Complete(DMA_HandleTypeDef *dma) { InputComplete(dma, 0U); }
static void Input1Complete(DMA_HandleTypeDef *dma) { InputComplete(dma, 1U); }

/**
 * @brief  ADC 转换完成入口（HAL 弱回调覆盖）。
 * @param  adc 触发回调的 ADC 句柄。
 * @note   保持原状态机调用顺序；不直接依赖任何 App/myMiddleware 符号。
 *         对已完成输入块做缓存失效后，把只读块与序号交给订阅者回调。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    if (stream.input_running && adc == stream.config.adc && stream.completed != NULL)
    {
        const uint16_t *samples = stream.completed; /* 本次回调内有效的只读块 */
        stream.completed = NULL;
        InvalidateInput(samples);
        stream.receive(stream.input_context, samples,
                       stream.config.input_block_samples, stream.completed_sequence);
        /* 内部模式没有 DAC DMA 完成事件；每个输入块消费同时间段输出块，
         * 在处理完对应参考后再重填该槽，保持双缓冲的两块前瞻不变。 */
        if (!stream.config.output_enabled && stream.output_armed)
        {
            uint32_t index = (uint32_t)(stream.completed_sequence % 2U); /* 已消费内部槽。 */
            stream.refill(stream.output_context, stream.config.output_buffers[index],
                          stream.config.output_block_samples);
        }
    }
}

/**
 * @brief  ADC 错误回调（HAL 弱回调覆盖）：转成输入错误通知。
 * @param  adc 触发回调的 ADC 句柄。
 * @note   外设错误只转成通知，由订阅者决定后续策略。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    if (stream.input_running && adc == stream.config.adc && stream.error != NULL)
    {
        stream.error(stream.input_context, SIGNAL_ERROR_INPUT, adc->ErrorCode);
    }
}

/**
 * @brief  DAC 通道 1 下溢回调（HAL 弱回调覆盖）：转成输出下溢通知。
 * @param  dac 触发回调的 DAC 句柄。
 */
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *dac)
{
    if (stream.output_armed && dac == stream.config.dac && stream.error != NULL)
    {
        stream.error(stream.input_context, SIGNAL_ERROR_OUTPUT_UNDERRUN, dac->ErrorCode);
    }
}

/**
 * @brief  停止输出：先停主定时器，再停 DAC DMA。
 * @param  context 平台私有状态（即 &stream）。
 * @retval true  定时器与 DMA 均停止成功；false 任一步骤失败。
 * @note   无论是否已产生触发，都先停主定时器；返回值包含停止外设的失败。
 */
static bool StopOutput(void *context)
{
    STM32H7_SignalState *state = context; /* 当前端口的私有状态 */
    HAL_StatusTypeDef timer_status = HAL_TIM_Base_Stop(state->config.output_timer);
    HAL_StatusTypeDef dma_status;
    state->output_armed = false;
    dma_status = state->config.output_enabled ?
        HAL_DAC_Stop_DMA(state->config.dac, state->config.dac_channel) : HAL_OK;
    return timer_status == HAL_OK && dma_status == HAL_OK;
}

/**
 * @brief  装载输出：接收已渲染双缓冲并设置首码，但主触发尚未启动。
 * @param  context          平台私有状态（即 &stream）。
 * @param  preload          首个触发沿前的预装码。
 * @param  refill           输出重填回调。
 * @param  callback_context 输出订阅者上下文。
 * @retval true  装载成功；false 已装载、回调为空或任一 HAL 步骤失败（失败时撤销）。
 * @note   配置回调时公共触发必须尚未启动。
 */
static bool ArmOutput(void *context, uint16_t preload,
                       SignalOutputCallback refill, void *callback_context)
{
    STM32H7_SignalState *state = context; /* 当前端口的私有状态 */
    DMA_HandleTypeDef *dma = state->config.dac->DMA_Handle1; /* 输出 DMA */
    if (state->output_armed || refill == NULL) { return false; }
    state->refill = refill;
    state->output_context = callback_context;
    if (!state->config.output_enabled)
    {
        /* 禁用通道而不产生模拟输出；TIM1 仍是 ADC 和软件 NCO 的共同时间源。 */
        if (HAL_DAC_Stop_DMA(state->config.dac, state->config.dac_channel) != HAL_OK)
        { return false; }
        __HAL_TIM_SET_COUNTER(state->config.output_timer, 0U);
        __HAL_TIM_CLEAR_FLAG(state->config.output_timer, TIM_FLAG_UPDATE);
        state->output_armed = true;
        return true;
    }
    CleanOutput(state->config.output_buffers[0]);
    CleanOutput(state->config.output_buffers[1]);
    if (HAL_DAC_Start_DMAEx_IT(state->config.dac, state->config.dac_channel,
            (const uint32_t *)state->config.output_buffers[0],
            (const uint32_t *)state->config.output_buffers[1],
            state->config.output_block_samples, DAC_ALIGN_12B_R) != HAL_OK)
    {
        (void)StopOutput(context);
        return false;
    }
    dma->XferCpltCallback = Output0Complete;
    dma->XferM1CpltCallback = Output1Complete;
    dma->XferHalfCpltCallback = NULL;
    dma->XferM1HalfCpltCallback = NULL;
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);
    if (HAL_DAC_SetValue(state->config.dac, state->config.dac_channel,
                         DAC_ALIGN_12B_R, preload) != HAL_OK)
    {
        (void)StopOutput(context);
        return false;
    }
    __HAL_TIM_SET_COUNTER(state->config.output_timer, 0U);
    __HAL_TIM_CLEAR_FLAG(state->config.output_timer, TIM_FLAG_UPDATE);
    state->output_armed = true;
    return true;
}

/**
 * @brief  启动输出主触发（启动时序的最后一步）。
 * @param  context 平台私有状态（即 &stream）。
 * @retval true  启动成功；false 未装载或启动失败（失败时撤销）。
 * @note   最后启动主触发，使 ADC 和 DAC 从同一条触发时间轴开始。
 */
static bool StartOutput(void *context)
{
    STM32H7_SignalState *state = context; /* 当前端口的私有状态 */
    if (!state->output_armed) { return false; }
    __HAL_TIM_SET_COUNTER(state->config.output_timer, 0U);
    __HAL_TIM_CLEAR_FLAG(state->config.output_timer, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start(state->config.output_timer) != HAL_OK)
    {
        (void)StopOutput(context);
        return false;
    }
    return true;
}

/**
 * @brief  停止输入：先停从定时器，再取消输入 DMA。
 * @param  context 平台私有状态（即 &stream）。
 * @retval true  定时器与 DMA 均停止成功；false 任一步骤失败。
 * @note   主触发停止后调用；停止后平台不得再触发输入回调。
 */
static bool StopInput(void *context)
{
    STM32H7_SignalState *state = context; /* 当前端口的私有状态 */
    HAL_StatusTypeDef timer_status = HAL_TIM_Base_Stop(state->config.input_timer);
    HAL_StatusTypeDef dma_status;
    state->input_running = false;
    state->completed = NULL;
    dma_status = HAL_ADC_Stop_DMA(state->config.adc);
    return timer_status == HAL_OK && dma_status == HAL_OK;
}

/**
 * @brief  启动输入：装载输入双缓冲、安装回调并启动从定时器。
 * @param  context          平台私有状态（即 &stream）。
 * @param  receive          输入回调。
 * @param  error            错误回调。
 * @param  callback_context 输入/错误订阅者上下文。
 * @retval true  启动成功；false 已在运行、回调为空或任一 HAL 步骤失败（失败时撤销）。
 * @note   主触发仍停止时装载；sequence 归零；失败撤销启动，支持应用重试。
 */
static bool StartInput(void *context, SignalInputCallback receive,
                        SignalErrorCallback error, void *callback_context)
{
    STM32H7_SignalState *state = context; /* 当前端口的私有状态 */
    DMA_HandleTypeDef *dma = state->config.adc->DMA_Handle; /* 输入 DMA */
    if (state->input_running || receive == NULL || error == NULL) { return false; }
    state->receive = receive;
    state->error = error;
    state->input_context = callback_context;
    state->completed = NULL;
    state->next_sequence = 0U;
    /* DMA 接管前先清理脏行；完成时再次失效，防止投机读取留下旧数据。 */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        for (uint32_t i = 0U; i < 2U; ++i) /* 两个互不重叠的完整缓存行区域 */
        {
            SCB_CleanInvalidateDCache_by_Addr((uint32_t *)state->config.input_buffers[i],
                (int32_t)(state->config.input_block_samples * sizeof(uint16_t)));
        }
    }
    if (HAL_ADC_Start_DMAEx_IT(state->config.adc,
            (uint32_t *)state->config.input_buffers[0],
            (uint32_t *)state->config.input_buffers[1],
            state->config.input_block_samples) != HAL_OK)
    {
        (void)StopInput(context);
        return false;
    }
    dma->XferCpltCallback = Input0Complete;
    dma->XferM1CpltCallback = Input1Complete;
    dma->XferHalfCpltCallback = NULL;
    dma->XferM1HalfCpltCallback = NULL;
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);
    __HAL_TIM_SET_COUNTER(state->config.input_timer, 0U);
    __HAL_TIM_CLEAR_FLAG(state->config.input_timer, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start(state->config.input_timer) != HAL_OK)
    {
        (void)StopInput(context);
        return false;
    }
    state->input_running = true;
    return true;
}

/**
 * @brief 验证一个完整输出缓冲范围是否可被 DMA1 访问。
 * @param address 缓冲地址，已经做过 32 字节对齐检查。
 * @param bytes 有效缓冲长度，字节。
 * @retval true 完整处于本工程 H723 的 AXI/D2/D3 SRAM；false 跨界或不可访问。
 * @note 内部模式无需 DMA 可访问，但仍校验地址不与其他缓冲区重叠。
 */
static bool OutputMemoryIsAccessible(uintptr_t address, uintptr_t bytes)
{
    return (address >= 0x24000000U && address < 0x24050000U && bytes <= 0x24050000U - address) ||
           (address >= 0x30000000U && address < 0x30008000U && bytes <= 0x30008000U - address) ||
           (address >= 0x38000000U && address < 0x38004000U && bytes <= 0x38004000U - address);
}

/**
 * @brief 求共同主时钟的最近分频，验证容量并在停流时提交到定时器。
 * @param context 平台私有状态。
 * @param config 请求的 ADC 采样率、升采样倍数、输出开关和存储地址。
 * @param validate_only true 仅计算与检查，可在运行中预检。
 * @param info 可选输出，取得实际速率和误差，失败时不改写。
 * @retval true 配置可用或已提交；false 参数、地址、容量、时钟或启停状态不合法。
 * @note 主循环调用；先完整预检，再修改 PSC/ARR。触发路由和参考流水线不变。
 */
static bool Configure(void *context, const SignalStreamConfig *config,
                      bool validate_only, SignalStreamInfo *info)
{
    STM32H7_SignalState *state = context; /* 当前平台及静态内存绑定。 */
    TimerFrequencyPlan output_plan, input_plan; /* TIM1 内部时钟、TIM8 外部事件时钟计划。 */
    if (config == NULL || !isfinite(config->adc_sample_rate_hz) ||
        config->adc_sample_rate_hz <= 0.0 || config->dac_upsampling_factor == 0U ||
        config->adc_block_samples == 0U || config->adc_block_samples % 16U != 0U ||
        config->adc_block_samples > state->config.input_capacity_samples ||
        !TimerFrequency_Find(state->config.timer_clock_hz,
            config->adc_sample_rate_hz * (double)config->dac_upsampling_factor,
            UINT16_MAX, &output_plan) ||
        !TimerFrequency_FindExactDivider(output_plan.actual_hz,
            config->adc_sample_rate_hz, config->dac_upsampling_factor,
            UINT16_MAX, &input_plan)) { return false; }
    const uint64_t output_count = (uint64_t)config->adc_block_samples *
        input_plan.period_ticks; /* 块覆盖同样时间，先用 64 位防止溢出。 */
    const bool custom = config->output_buffers[0] != NULL ||
                        config->output_buffers[1] != NULL; /* 是否使用调用者存储。 */
    uint16_t *buffers[2] = {custom ? config->output_buffers[0] : state->default_output[0],
                            custom ? config->output_buffers[1] : state->default_output[1]};
    const uint32_t capacity = custom ? config->output_capacity_samples :
        state->config.output_capacity_samples; /* 每块可写容量，点数。 */
    const uint32_t ratio = (uint32_t)input_plan.period_ticks; /* 容量检查后可表示的速率比。 */
    const uint32_t offset = config->reference_sample_offset == UINT32_MAX ?
        (ratio > 1U ? ratio - 2U : 0U) : config->reference_sample_offset; /* 原有 DHR/DOR 一拍。 */
    if (output_count == 0U || output_count > capacity || output_count > UINT16_MAX ||
        input_plan.period_ticks > UINT32_MAX || offset >= ratio) { return false; }
    uintptr_t addresses[4] = {(uintptr_t)state->config.input_buffers[0],
        (uintptr_t)state->config.input_buffers[1], (uintptr_t)buffers[0], (uintptr_t)buffers[1]};
    uintptr_t lengths[4] = {config->adc_block_samples * sizeof(uint16_t),
        config->adc_block_samples * sizeof(uint16_t), (uintptr_t)output_count * sizeof(uint16_t),
        (uintptr_t)output_count * sizeof(uint16_t)}; /* 所有权范围，用于缓存/重叠检查。 */
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        if (addresses[i] == 0U || addresses[i] % 32U != 0U ||
            addresses[i] > UINTPTR_MAX - lengths[i] ||
            (i >= 2U && config->dac_output_enabled &&
             !OutputMemoryIsAccessible(addresses[i], lengths[i]))) { return false; }
        for (uint32_t j = 0U; j < i; ++j)
        {
            if (addresses[i] < addresses[j] + lengths[j] &&
                addresses[j] < addresses[i] + lengths[i]) { return false; }
        }
    }
    SignalStreamInfo result = {
        .adc_sample_rate_hz = input_plan.actual_hz,
        .dac_base_sample_rate_hz = output_plan.actual_hz / config->dac_upsampling_factor,
        .dac_output_sample_rate_hz = output_plan.actual_hz,
        .adc_error_ppm = input_plan.error_ppm, .dac_error_ppm = output_plan.error_ppm,
        .block_period_s = config->adc_block_samples / input_plan.actual_hz,
        .output_block_samples = (uint32_t)output_count,
        .output_samples_per_input_sample = ratio, .dac_output_enabled = config->dac_output_enabled,
    }; /* 请求与实际的差异显式返回，上层算法只使用实际值。 */
    if (!validate_only)
    {
        if (state->input_running || state->output_armed ||
            (state->config.input_timer->Instance->CR1 & TIM_CR1_CEN) != 0U ||
            (state->config.output_timer->Instance->CR1 & TIM_CR1_CEN) != 0U ||
            !TimerFrequency_Apply(state->config.output_timer, &output_plan) ||
            !TimerFrequency_Apply(state->config.input_timer, &input_plan)) { return false; }
        /* UG 会传播到从定时器，所以两份 PSC/ARR 都应用后再次归零。 */
        __HAL_TIM_SET_COUNTER(state->config.input_timer, 0U);
        __HAL_TIM_CLEAR_FLAG(state->config.input_timer, TIM_FLAG_UPDATE);
        state->config.input_block_samples = config->adc_block_samples;
        state->config.output_block_samples = (uint32_t)output_count;
        state->config.output_enabled = config->dac_output_enabled;
        state->config.output_buffers[0] = buffers[0];
        state->config.output_buffers[1] = buffers[1];
        port.input_rate_hz = input_plan.actual_hz;
        port.output_rate_hz = output_plan.actual_hz;
        port.input_block_samples = config->adc_block_samples;
        port.output_block_samples = (uint32_t)output_count;
        port.output_samples_per_input_sample = ratio;
        port.reference_sample_offset = offset;
        port.reference_uses_previous_sample = ratio == 1U;
        port.output_buffers[0] = buffers[0];
        port.output_buffers[1] = buffers[1];
        port.stream_info = result;
    }
    if (info != NULL) { *info = result; }
    return true;
}

/** 稳定接口与平台实现的唯一操作绑定（只读，运行期有效）。 */
static const SignalPortOps operations = {
    .enter_critical = EnterCritical, .exit_critical = ExitCritical,
    .memory_barrier = MemoryBarrier, .cycles = Cycles,
    .arm_output = ArmOutput, .start_output = StartOutput, .stop_output = StopOutput,
    .start_input = StartInput, .stop_input = StopInput,
    .configure = Configure,
};

/**
 * @brief  初始化平台端口：校验配置并填充 SignalPort 描述符。
 * @param  config 平台配置（含 ADC/DAC 句柄、定时器、速率、双缓冲指针等）。
 * @retval 非 NULL 指向静态 SignalPort（运行期有效）；NULL 配置非法或端口已被占用。
 * @note   校验项：句柄/定时器非空、DAC 通道为 CH1、速率与块长非 0 且 ≤ UINT16_MAX、
 *         块长为 16 的倍数、双缓冲 32 字节对齐。初始化只填充描述符，不启动外设。
 */
const SignalPort *STM32H7_SignalPort_Init(const STM32H7_SignalConfig *config)
{
    if (config == NULL || stream.input_running || stream.output_armed ||
        config->adc == NULL || config->dac == NULL ||
        config->adc->DMA_Handle == NULL || config->dac->DMA_Handle1 == NULL ||
        config->input_timer == NULL || config->output_timer == NULL ||
        config->dac_channel != DAC_CHANNEL_1 || config->input_rate_hz == 0U ||
        config->output_rate_hz == 0U || config->input_block_samples == 0U ||
        config->output_block_samples == 0U || config->input_block_samples > UINT16_MAX ||
        config->output_block_samples > UINT16_MAX ||
        config->input_block_samples % 16U != 0U ||
        config->output_block_samples % 16U != 0U)
    {
        return NULL;
    }
    for (uint32_t i = 0U; i < 2U; ++i) /* 检查 32 字节缓存行对齐 */
    {
        if (config->input_buffers[i] == NULL || config->output_buffers[i] == NULL ||
            (uintptr_t)config->input_buffers[i] % 32U != 0U ||
            (uintptr_t)config->output_buffers[i] % 32U != 0U)
        {
            return NULL;
        }
    }
    stream = (STM32H7_SignalState){.config = *config,
        .default_output = {config->output_buffers[0], config->output_buffers[1]}};
    port = (SignalPort){
        .ops = &operations, .context = &stream,
        .input_rate_hz = config->input_rate_hz,
        .output_rate_hz = config->output_rate_hz,
        .input_block_samples = config->input_block_samples,
        .output_block_samples = config->output_block_samples,
        .reference_sample_offset = config->reference_sample_offset,
        .output_buffers = {config->output_buffers[0], config->output_buffers[1]},
    };
    return &port;
}
