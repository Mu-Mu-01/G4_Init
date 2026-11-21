/**
 * @file    nco_dac.c
 * @brief   可移植 NCO 输出服务：把频率控制字渲染成 12 位 DAC 码并通过端口输出。
 *
 * 本模块是 DPLL 的"输出半环"：维护一个 64 位相位累加器（NCO），按频率控制字
 * （FCW）逐采样步进，查预计算波形表渲染成右对齐 12 位 DAC 码，再经由 SignalPort
 * 的双缓冲输出流送往 DAC。模块不直接依赖 HAL，同步流、临界区和缓存一致性均由
 * 注入的 SignalPort 提供，因此本层保持芯片无关。
 *
 * 关键设计点：
 *  - FCW 定义在实际 ADC 采样率（NCO 域），按端口实际输出/输入速率比插值为输出
 *    采样，用"商 + 余数"方式在块内精确分摊，
 *    避免逐点除法。
 *  - 参考历史环：保留最近 8 个已渲染块的起始相位/FCW 元数据，供输入回调按
 *    block_sequence 反查"DAC 在该输入采样时刻的真实相位"，实现 ADC/DAC 时间轴对齐。
 *  - FCW 写入权：闭环使能后 FCW 归 PI 控制，手动写入会被忽略，避免两处争写。
 *  - 所有跨中断共享变量（64 位 FCW、电平、波形、倍频）都经临界区读写，
 *    用内存屏障保证发布顺序，不依赖 volatile 替代同步。
 *  - 渲染循环保持纯整数运算（查表 + 定点插值 + 缩放），不分配、不等待，
 *    以便安全地在 DAC 重填中断内执行。
 */
#include "nco_dac.h"

#include "nco_frequency_multiplier.h"
#include "nco_phase_interp.h"
#include "nco_waveform.h"

#include <math.h>
#include <stddef.h>

/** 输出接口的右对齐码宽：DAC 为 12 位。 */
#define NCO_DAC_BITS                12U
/** 12 位码的饱和上限（4095），用于校验幅度+偏置是否越界。 */
#define NCO_DAC_MAX_CODE            ((1U << NCO_DAC_BITS) - 1U)
/** 保留最近 8 个已渲染块的参考元数据，供输入侧按序号对齐。 */
#define NCO_REFERENCE_HISTORY_SIZE  8U

/** uint64 相位累加器满量程对应的 2^64，换算频率时使用。 */
#define NCO_TWO_POW_64_D            18446744073709551616.0

/* 插值比例来自端口实际速率，不再要求固定 11 倍。 */

/* ---- 模块内部状态 ------------------------------------------------------- */

static const SignalPort *nco_port; /* 借用运行期有效的平台端口 */

static volatile uint64_t nco_frequency_control_word; /* 每输入采样步进，2^64/周 */
static volatile uint32_t nco_output_level = (2048U << 16) | 1700U; /* 高16位偏置，低16位幅度 */
static volatile uint32_t nco_output_waveform = NCO_OUTPUT_WAVEFORM_SINE; /* 输出波形类型 */
static volatile uint32_t nco_output_frequency_multiplier = 1U; /* 仅输出相位的整数倍数 */
static uint64_t nco_phase_accumulator;       /* 下一个渲染块的基频相位 */
static uint64_t nco_initial_preload_phase;   /* 首个触发预装码的相位 */
static uint32_t nco_phase_remainder_accumulator; /* 分数余数，范围 0..速率比-1 */
static uint32_t nco_output_samples_per_input_sample; /* 兼容旧端口的速率比副本。 */
static uint64_t nco_previous_sample_phase; /* 上一块末点，用于同速 DAC 一拍延迟。 */
static SignalOutputReadyCallback nco_output_ready; /* 可选内部输出订阅者。 */
static void *nco_output_context; /* 借用订阅者上下文。 */
static uint64_t nco_buffer_sequence[2]; /* 两块存储当前对应的渲染序号。 */
static bool nco_buffer_valid[2]; /* 存储是否已经完成渲染。 */
static uint64_t nco_next_block_sequence;     /* 下一个渲染块的时间轴序号 */
static bool nco_initialized;                /* 端口和波形表已初始化 */
static bool nco_armed;                      /* 输出流已装载，可接收重填回调 */
static bool nco_trigger_running;            /* 主触发已启动 */
static bool nco_closed_loop_enabled;        /* FCW 写入权归 PI，而非手动控制 */
static volatile uint32_t nco_last_fill_cycles;    /* 最近一次渲染 CPU 周期，不含平台缓存维护 */
static volatile uint32_t nco_maximum_fill_cycles; /* 周期性渲染最坏 CPU 周期 */

/** 参考历史环的单项：元数据 + 发布标记。 */
typedef struct
{
    NCO_DAC_ReferenceMetadata metadata; /* 完整输出块与基频参考的映射 */
    bool valid;                        /* 发布标记，临界区内最后写入 */
} NCO_DAC_ReferenceHistoryEntry;

static NCO_DAC_ReferenceHistoryEntry
    nco_reference_history[NCO_REFERENCE_HISTORY_SIZE]; /* 固定长度参考历史环 */

static NCO_WaveformLut nco_waveform_lut; /* 初始化时预计算，实时路径只查询 */

/* ---- 运行环境与临界区 --------------------------------------------------- */

/* 统一通过接口保护 64 位 FCW 和跨中断元数据，不假定 CPU 指令集。 */
static uint32_t NCO_DAC_EnterCritical(void)
{
    return SignalPort_EnterCritical(nco_port);
}

/* 恢复进入前的执行环境，可与应用层临界区嵌套。 */
static void NCO_DAC_ExitCritical(uint32_t token)
{
    SignalPort_ExitCritical(nco_port, token);
}

/* ---- 内部辅助函数 ------------------------------------------------------- */

/**
 * @brief  发布一个渲染块的参考元数据到历史环（环形覆盖写入）。
 * @param  block_sequence              该块的序号，用于选择环槽及供输入侧匹配。
 * @param  start_phase                 输入首样本对应的参考相位（2^64/周）。
 * @param  first_frequency_control_word 第一个参考步长（2^64/周）。
 * @param  frequency_control_word      后续参考步长（2^64/周）。
 * @note   先写字段、发内存屏障、最后置 valid，保证读者只看到完整数据。
 */
static void NCO_DAC_RecordReference(uint64_t block_sequence,
                                    uint64_t start_phase,
                                    uint64_t first_frequency_control_word,
                                    uint64_t frequency_control_word)
{
    /* history_index 选择环槽；primask 为不透明的临界区恢复令牌。 */
    uint32_t history_index =
        (uint32_t)(block_sequence % NCO_REFERENCE_HISTORY_SIZE);
    uint32_t primask = NCO_DAC_EnterCritical();

    nco_reference_history[history_index].valid = false;
    nco_reference_history[history_index].metadata.block_sequence = block_sequence;
    nco_reference_history[history_index].metadata.start_phase = start_phase;
    nco_reference_history[history_index].metadata.first_frequency_control_word =
        first_frequency_control_word;
    nco_reference_history[history_index].metadata.frequency_control_word =
        frequency_control_word;
    SignalPort_MemoryBarrier(nco_port);
    nco_reference_history[history_index].valid = true;
    NCO_DAC_ExitCritical(primask);
}

/**
 * @brief  用只读波形表和打包电平渲染一个 12 位 DAC 码。
 * @param  waveform_table 波形 Q15 表首指针。
 * @param  phase          相位，单位为 2^64/周。
 * @param  output_level   打包的"偏置<<16 | 幅度"码值。
 * @retval 右对齐的 12 位 DAC 输出码。
 */
static uint16_t NCO_DAC_RenderSample(const int16_t *waveform_table,
                                     uint64_t phase,
                                     uint32_t output_level)
{
    /* 解包峰值幅度和直流偏置，单位为 DAC 码。 */
    const uint16_t amplitude = (uint16_t)(output_level & 0xFFFFU);
    const uint16_t dc_offset = (uint16_t)(output_level >> 16U);

    return NCO_Waveform_RenderTable(waveform_table, phase,
                                    amplitude, dc_offset);
}

/**
 * @brief  根据当前写入权更新 FCW；写入来源与当前控制权不一致时拒绝。
 * @param  frequency_control_word 新的频率控制字（2^64/周）。
 * @param  closed_loop_write      true 表示闭环（PI）写入，false 表示手动写入。
 * @retval true  写入成功；false 写入者当前不拥有 FCW 控制权，写入被忽略。
 */
static bool NCO_DAC_WriteFrequencyControlWord(
    uint64_t frequency_control_word,
    bool closed_loop_write)
{
    bool accepted; /* 当前写入者是否拥有 FCW 控制权 */
    uint32_t primask = NCO_DAC_EnterCritical();

    accepted = (closed_loop_write == nco_closed_loop_enabled);
    if (accepted)
    {
        nco_frequency_control_word = frequency_control_word;
        SignalPort_MemoryBarrier(nco_port);
    }
    NCO_DAC_ExitCritical(primask);
    return accepted;
}

/* 填满一个已由平台释放的配置长度块；buffer 只在本次调用内可写。
 * 启动时由 Arm 调用，运行时由输出回调调用，两种上下文禁止并发。 */
/**
 * @brief 渲染配置长度的输出块，并发布对应 ADC 时间轴参考和只读输出事件。
 * @param buffer 平台释放的输出地址，容量至少为 output_block_samples。
 * @retval 无。
 * @note 主循环预填或中断重填，不可重入；循环有界，无动态分配。
 */
static void NCO_DAC_FillBuffer(uint16_t* buffer)
{
    /* 计时、基础/倍频相位、当前块身份及 FCW 快照；相位以 2^64/周为单位。 */
    uint32_t start_cycles = SignalPort_Cycles(nco_port);
    uint64_t phase = nco_phase_accumulator;
    uint64_t dac_phase;
    uint64_t dac_frequency_control_word;
    uint64_t generated_start_phase = phase;
    uint64_t block_sequence = nco_next_block_sequence;
    uint64_t frequency_control_word;
    /* 插值商、余数和组内偏移；reference_phase_offset 是输入首样本的相位偏移。 */
    uint64_t phase_step_quotient;
    uint64_t reference_phase_offset;
    uint32_t phase_step_remainder;
    uint32_t remainder_accumulator;
    uint32_t ratio = nco_output_samples_per_input_sample; /* 定时器确定的整数速率比。 */
    uint64_t output_step; /* 倍频后每输出点的整数相位步长。 */
    uint64_t previous_sample = nco_previous_sample_phase; /* 前块末点相位。 */
    /* 本块不可变的波形/倍频/电平码值快照，避免重填过程中参数撕裂。 */
    uint32_t output_level;
    uint32_t output_frequency_multiplier;
    NCO_OutputWaveform output_waveform;
    const int16_t *waveform_table;
    uint16_t amplitude;
    uint16_t dc_offset;
    uint32_t primask = NCO_DAC_EnterCritical();

    /* 临界区内一次性快照 FCW/电平/波形/倍频，之后渲染不触碰共享变量。 */
    frequency_control_word = nco_frequency_control_word;
    output_level = nco_output_level;
    output_waveform = (NCO_OutputWaveform)nco_output_waveform;
    output_frequency_multiplier = nco_output_frequency_multiplier;
    NCO_DAC_ExitCritical(primask);

    /* FCW 按运行时速率比拆成商与余数：商每步累加，余数按组进位，保证
     * 一组输出步长之和精确等于一个 FCW，长期不漂移。 */
    phase_step_quotient = frequency_control_word / ratio;
    phase_step_remainder = (uint32_t)(frequency_control_word % ratio);
    remainder_accumulator = nco_phase_remainder_accumulator;
    amplitude = (uint16_t)(output_level & 0xFFFFU);
    dc_offset = (uint16_t)(output_level >> 16U);
    waveform_table = NCO_Waveform_GetTable(&nco_waveform_lut,
                                           output_waveform);
    if (waveform_table == NULL)
    {
        waveform_table = nco_waveform_lut.sine_q15;
    }

    /* 一个完整的 DMA 块包含 input_block_samples 组、每组 ratio 个采样，
     * 小数余数会回到起始值。商与余数预先计算，输出循环不执行逐点除法。 */
    reference_phase_offset = NCO_PhaseInterp_GetOffset(frequency_control_word,
        ratio, nco_port->reference_sample_offset, 0U);

    /* 保持基础累加器不变以供 IQ/PI 使用。每块预计算倍频步长；
     * 配置长度的渲染循环保持仅加运算。 */
    dac_phase = phase;
    dac_frequency_control_word = frequency_control_word;
    if (output_frequency_multiplier != 1U)
    {
        dac_phase = NCO_FrequencyMultiplier_ApplyPhase(
            phase, output_frequency_multiplier);
        dac_frequency_control_word = NCO_FrequencyMultiplier_ApplyPhase(
            frequency_control_word, output_frequency_multiplier);
    }
    output_step = phase_step_quotient * output_frequency_multiplier;

    if (ratio == 1U)
    {
        /* 同速时相位步长恒等于完整 FCW，无需逐点分配余数。 */
        for (uint32_t i = 0; i < nco_port->output_block_samples; ++i)
        {
            buffer[i] = NCO_Waveform_RenderTable(waveform_table, dac_phase, amplitude, dc_offset);
            dac_phase += dac_frequency_control_word;
        }
        phase += frequency_control_word * nco_port->input_block_samples;
    }
    else
    {
        /* 逐组（每组 ratio 个采样）有界渲染，支持运行时改变速率比。 */
        for (uint32_t group = 0U; group < nco_port->input_block_samples; ++group)
        {
            uint16_t *destination = &buffer[group * ratio]; /* 本组目标。 */
            uint64_t sample_phase = dac_phase; /* 当前输出点相位。 */
            for (uint32_t sample = 0U; sample < ratio; ++sample)
            {
                destination[sample] = NCO_Waveform_RenderTable(
                    waveform_table, sample_phase, amplitude, dc_offset);
                sample_phase += output_step;
                remainder_accumulator += phase_step_remainder;
                if (remainder_accumulator >= ratio)
                {
                    remainder_accumulator -= ratio;
                    sample_phase += output_frequency_multiplier;
                }
            }
            phase += frequency_control_word;
            dac_phase += dac_frequency_control_word;
        }
    }

    /* 首个输入样本对应的输出相位由端口描述；后续每个输入样本前进一个 FCW。 */
    nco_phase_accumulator = phase;
    nco_next_block_sequence = block_sequence + 1U;
    NCO_DAC_RecordReference(block_sequence,
                            nco_port->reference_uses_previous_sample ? previous_sample :
                                generated_start_phase + reference_phase_offset,
                            nco_port->reference_uses_previous_sample ?
                                generated_start_phase - previous_sample : frequency_control_word,
                            frequency_control_word);
    nco_previous_sample_phase = phase - frequency_control_word +
        NCO_PhaseInterp_GetOffset(frequency_control_word, ratio, ratio - 1U, 0U);
    {
        uint32_t index = buffer == nco_port->output_buffers[0] ? 0U : 1U; /* 存储槽。 */
        SignalOutputBlock block = {buffer, nco_port->output_block_samples,
            nco_port->output_rate_hz, block_sequence}; /* 回调期间有效的视图。 */
        uint32_t token = NCO_DAC_EnterCritical(); /* 原子发布块身份。 */
        nco_buffer_sequence[index] = block_sequence;
        nco_buffer_valid[index] = true;
        NCO_DAC_ExitCritical(token);
        if (nco_output_ready != NULL) { nco_output_ready(nco_output_context, &block); }
    }

    nco_last_fill_cycles = SignalPort_Cycles(nco_port) - start_cycles;
    if (nco_last_fill_cycles > nco_maximum_fill_cycles)
    {
        nco_maximum_fill_cycles = nco_last_fill_cycles;
    }
}

/* 平台只交付已消费的输出块；中断内有界渲染，不分配、不等待。 */
/**
 * @brief 响应平台交付的空闲输出块。
 * @param context 未使用。
 * @param buffer 空闲输出缓冲区。
 * @param count 本块有效点数。
 * @retval 无。
 * @note 在输出或内部模式的输入中断执行，禁止重入。
 */
static void NCO_DAC_Refill(void *context, uint16_t *buffer, uint32_t count)
{
    (void)context;
    if (nco_armed && count == nco_port->output_block_samples &&
        (buffer == nco_port->output_buffers[0] ||
         buffer == nco_port->output_buffers[1]))
    {
        NCO_DAC_FillBuffer(buffer);
    }
}

/* ---- 生命周期与启停 ----------------------------------------------------- */

/**
 * @brief  初始化单实例 NCO 输出服务，借用运行期有效的端口。
 * @param  port 平台端口；最终输出率为输入率整数倍，输入输出块时长相等。
 * @retval true  初始化成功；false 已装载、端口非法或契约不匹配。
 * @note   仅做静态配置并预计算波形表，不启动任何外设；运行中禁止重复初始化。
 */
bool NCO_DAC_Init(const SignalPort *port)
{
    const double ratio_real = port != NULL && port->input_rate_hz > 0.0 ?
        port->output_rate_hz / port->input_rate_hz : 0.0;
    const uint32_t ratio = port != NULL && port->output_samples_per_input_sample != 0U ?
        port->output_samples_per_input_sample : (uint32_t)llround(ratio_real);
    if (nco_armed || !SignalPort_IsValid(port) ||
        !isfinite(port->input_rate_hz) || !isfinite(port->output_rate_hz) ||
        ratio == 0U || fabs(ratio_real - ratio) > 1.0e-8 ||
        (uint64_t)port->input_block_samples * ratio !=
            port->output_block_samples ||
        port->output_block_samples > UINT16_MAX ||
        port->reference_sample_offset >= ratio)
    {
        return false;
    }

    NCO_Waveform_Init(&nco_waveform_lut);
    nco_port = port;
    nco_output_samples_per_input_sample = ratio;
    nco_output_ready = NULL;
    nco_output_context = NULL;
    nco_buffer_valid[0] = nco_buffer_valid[1] = false;
    nco_phase_accumulator = 0U;
    nco_initial_preload_phase = 0U;
    nco_phase_remainder_accumulator = 0U;
    nco_next_block_sequence = 0U;
    nco_initialized = true;
    nco_armed = false;
    nco_trigger_running = false;
    nco_closed_loop_enabled = false;
    nco_output_waveform = NCO_OUTPUT_WAVEFORM_SINE;
    nco_output_frequency_multiplier = 1U;
    nco_last_fill_cycles = 0U;
    nco_maximum_fill_cycles = 0U;
    return true;
}

/**
 * @brief  装载输出：填充两块输出缓冲与首个预装码，交给平台但触发仍停止。
 * @retval true  装载成功并进入 armed 状态；false 未初始化、已装载或平台装载失败。
 * @note   首次填表不属于周期性中断负载，其周期计数单独清零，不计入最坏情况统计。
 */
bool NCO_DAC_Arm(void)
{
    /* 首个预装码紧邻第 0 个 DMA 样本之前，保持已有启动相位。 */
    uint16_t preload;
    if ((!nco_initialized) || nco_armed) { return false; }

    nco_phase_accumulator = 0U;
    nco_phase_remainder_accumulator = 0U;
    {
        /* 单位为 2^64/周；余数插值确保一组输出步长精确等于一个 FCW。 */
        uint64_t initial_fcw = NCO_DAC_GetFrequencyControlWord();
        uint32_t ratio = nco_output_samples_per_input_sample; /* 实际整数速率比。 */
        uint64_t last_step =
            NCO_PhaseInterp_GetOffset(initial_fcw, ratio, ratio, 0U) -
            NCO_PhaseInterp_GetOffset(initial_fcw, ratio, ratio - 1U, 0U);
        nco_initial_preload_phase = 0U - last_step;
    }
    nco_previous_sample_phase = nco_initial_preload_phase;
    nco_next_block_sequence = 0U;
    for (uint32_t i = 0U; i < NCO_REFERENCE_HISTORY_SIZE; ++i)
    {
        nco_reference_history[i].valid = false;
    }
    NCO_DAC_FillBuffer(nco_port->output_buffers[0]);
    NCO_DAC_FillBuffer(nco_port->output_buffers[1]);
    /* 首次填表不属于周期性中断负载，单独排除。 */
    nco_last_fill_cycles = 0U;
    nco_maximum_fill_cycles = 0U;
    {
        /* 原子取得输出参数，临界区不包含渲染循环。 */
        uint32_t token = NCO_DAC_EnterCritical();
        uint32_t level = nco_output_level;
        uint32_t multiplier = nco_output_frequency_multiplier;
        const int16_t *table = NCO_Waveform_GetTable(
            &nco_waveform_lut, (NCO_OutputWaveform)nco_output_waveform);
        NCO_DAC_ExitCritical(token);
        if (table == NULL) { table = nco_waveform_lut.sine_q15; }
        preload = NCO_DAC_RenderSample(table,
            NCO_FrequencyMultiplier_ApplyPhase(nco_initial_preload_phase, multiplier),
            level);
    }
    if (!nco_port->ops->arm_output(nco_port->context, preload,
                                   NCO_DAC_Refill, NULL))
    {
        return false;
    }
    nco_armed = true;
    return true;
}

/**
 * @brief  便捷启动：先装载输出，再启动主触发。
 * @retval true  装载并启动成功；false 任一环节失败。
 * @note   同步输入的应用不应使用本函数，而应手动 Arm → 启动输入 → StartTrigger，
 *         以保证输入输出共享同一触发时间轴。
 */
bool NCO_DAC_Start(void)
{
    return NCO_DAC_Arm() && NCO_DAC_StartTrigger();
}

/**
 * @brief  启动已装载输出的主触发（最后一步）。
 * @retval true  触发已启动；false 未初始化、未装载、已在运行或平台启动失败。
 */
bool NCO_DAC_StartTrigger(void)
{
    if (!nco_initialized || !nco_armed || nco_trigger_running) { return false; }
    if (!nco_port->ops->start_output(nco_port->context))
    {
        /* 端口在失败返回前已停止输出；允许重新装载并重试。 */
        nco_armed = false;
        return false;
    }
    nco_trigger_running = true;
    return true;
}

/**
 * @brief  停止输出：先停触发再停输出 DMA。
 * @retval true  停止成功；false 未初始化或未装载。
 */
bool NCO_DAC_Stop(void)
{
    if (!nco_initialized || !nco_armed) { return false; }
    nco_armed = false;
    nco_trigger_running = false;
    return nco_port->ops->stop_output(nco_port->context);
}

/* ---- 参考元数据与诊断 --------------------------------------------------- */

/**
 * @brief 安装内部输出订阅者，DAC 关闭时也会通知。
 * @param callback 可选回调，NULL 取消订阅。
 * @param context 借用回调上下文。
 * @retval true 安装成功；false 未初始化或输出已装载。
 * @note 主循环停流时调用；回调契约见 signal_config.h。
 */
bool NCO_DAC_SetOutputCallback(SignalOutputReadyCallback callback, void *context)
{
    if (!nco_initialized || nco_armed) { return false; }
    nco_output_ready = callback;
    nco_output_context = context;
    return true;
}

/**
 * @brief 查询内部输出缓冲区地址及该地址当前对应的块序号。
 * @param index 双缓冲索引，0 或 1。
 * @param block 输出只读地址、有效长度、实际采样率和序号。
 * @retval true 已复制视图；false 尚未渲染或参数非法。
 * @note 仅描述符在临界区复制；样本运行时会覆盖，稳定消费应使用回调或停流。
 */
bool NCO_DAC_GetOutputBuffer(uint32_t index, SignalOutputBlock *block)
{
    if (!nco_initialized || index > 1U || block == NULL) { return false; }
    uint32_t token = NCO_DAC_EnterCritical(); /* 保护地址对应的序号。 */
    bool valid = nco_buffer_valid[index]; /* 是否有可用样本。 */
    if (valid)
    {
        *block = (SignalOutputBlock){nco_port->output_buffers[index],
            nco_port->output_block_samples, nco_port->output_rate_hz,
            nco_buffer_sequence[index]};
    }
    NCO_DAC_ExitCritical(token);
    return valid;
}

/**
 * @brief  按序号复制最近 8 块之一的参考元数据，不转移内部所有权。
 * @param  block_sequence 目标块序号。
 * @param  metadata       输出参数，写入匹配的元数据副本。
 * @retval true  命中且复制成功；false 未初始化、指针为空或记录已过期。
 * @note   可从输入回调调用；用于把 ADC 采样时刻对齐到 DAC 的相位时间轴。
 */
bool NCO_DAC_GetReferenceMetadata(uint64_t block_sequence,
                                  NCO_DAC_ReferenceMetadata* metadata)
{
    uint32_t history_index;
    uint32_t primask;
    bool found;

    if ((!nco_initialized) || (metadata == NULL))
    {
        return false;
    }

    history_index = (uint32_t)(block_sequence % NCO_REFERENCE_HISTORY_SIZE);
    primask = NCO_DAC_EnterCritical();
    found = nco_reference_history[history_index].valid &&
            (nco_reference_history[history_index].metadata.block_sequence ==
             block_sequence);
    if (found)
    {
        *metadata = nco_reference_history[history_index].metadata;
    }
    NCO_DAC_ExitCritical(primask);
    return found;
}

/**
 * @brief  获取最近一次渲染消耗的 CPU 周期数。
 * @retval 周期计数，仅含中间层渲染，不含平台缓存维护及中断分发。
 */
uint32_t NCO_DAC_GetLastFillCycles(void)
{
    return nco_last_fill_cycles;
}

/**
 * @brief  获取周期性渲染的历史最坏 CPU 周期数。
 * @retval 启动以来单块渲染的最大周期计数。
 */
uint32_t NCO_DAC_GetMaximumFillCycles(void)
{
    return nco_maximum_fill_cycles;
}

/* ---- FCW 与频率管理 ----------------------------------------------------- */

/**
 * @brief  手动设置频率控制字（闭环禁用时生效）。
 * @param  frequency_control_word 新的 FCW，单位 2^64/周。
 * @note   闭环使能时本写入会被忽略（返回值为 void，静默拒绝）。
 */
void NCO_DAC_SetFrequencyControlWord(uint64_t frequency_control_word)
{
    (void)NCO_DAC_WriteFrequencyControlWord(frequency_control_word, false);
}

/**
 * @brief  读取当前频率控制字。
 * @retval 当前 FCW（2^64/周），临界区内原子读取。
 */
uint64_t NCO_DAC_GetFrequencyControlWord(void)
{
    uint64_t frequency_control_word;
    uint32_t primask = NCO_DAC_EnterCritical();
    frequency_control_word = nco_frequency_control_word;
    NCO_DAC_ExitCritical(primask);
    return frequency_control_word;
}

/**
 * @brief  使能或禁用闭环：使能后 FCW 写入权归 PI，手动写入被忽略。
 * @param  enable true 进入闭环，false 交还手动控制。
 * @retval true  切换成功；false 未初始化。
 */
bool NCO_DAC_SetClosedLoopEnabled(bool enable)
{
    uint32_t primask;

    if (!nco_initialized)
    {
        return false;
    }

    primask = NCO_DAC_EnterCritical();
    nco_closed_loop_enabled = enable;
    SignalPort_MemoryBarrier(nco_port);
    NCO_DAC_ExitCritical(primask);
    return true;
}

/**
 * @brief  查询闭环当前是否使能。
 * @retval true  闭环已使能；false 未使能。
 */
bool NCO_DAC_IsClosedLoopEnabled(void)
{
    bool enabled;
    uint32_t primask = NCO_DAC_EnterCritical();

    enabled = nco_closed_loop_enabled;
    NCO_DAC_ExitCritical(primask);
    return enabled;
}

/**
 * @brief  由闭环（PI）写入频率控制字，仅在闭环使能时被接受。
 * @param  frequency_control_word 新的 FCW（2^64/周）。
 * @retval true  写入成功；false 闭环未使能，写入被忽略。
 */
bool NCO_DAC_SetClosedLoopFrequencyControlWord(
    uint64_t frequency_control_word)
{
    return NCO_DAC_WriteFrequencyControlWord(frequency_control_word, true);
}

/**
 * @brief  手动设置输出频率（Hz），范围 0..实际 ADC 采样率/2。
 * @param  frequency_hz 目标频率。
 * @retval true  设置成功；false 未初始化、闭环占用或参数非法。
 */
bool NCO_DAC_SetFrequencyHz(double frequency_hz)
{
    if ((!nco_initialized) || (!isfinite(frequency_hz)) ||
        (frequency_hz < 0.0) ||
        (frequency_hz > nco_port->input_rate_hz * 0.5))
    {
        return false;
    }

    return NCO_DAC_WriteFrequencyControlWord(
        NCO_DAC_FrequencyHzToControlWord(frequency_hz), false);
}

/**
 * @brief  获取当前输出频率（Hz）。
 * @retval 当前 FCW 换算得到的频率。
 */
double NCO_DAC_GetFrequencyHz(void)
{
    return NCO_DAC_ControlWordToFrequencyHz(NCO_DAC_GetFrequencyControlWord());
}

/**
 * @brief  把频率（Hz）换算为频率控制字（2^64/周），四舍五入。
 * @param  frequency_hz 目标频率，须 > 0。
 * @retval 对应 FCW；未初始化或参数非法返回 0。
 */
uint64_t NCO_DAC_FrequencyHzToControlWord(double frequency_hz)
{
    double scaled_word;

    if ((!nco_initialized) || (!isfinite(frequency_hz)) || (frequency_hz <= 0.0) ||
        frequency_hz > nco_port->input_rate_hz * 0.5)
    {
        return 0U;
    }

    scaled_word = (frequency_hz / nco_port->input_rate_hz) *
                  NCO_TWO_POW_64_D;
    return (uint64_t)(scaled_word + 0.5);
}

/**
 * @brief  把频率控制字（2^64/周）换算为频率（Hz）。
 * @param  frequency_control_word 目标 FCW。
 * @retval 对应频率；未初始化返回 0.0。
 */
double NCO_DAC_ControlWordToFrequencyHz(uint64_t frequency_control_word)
{
    if (!nco_initialized)
    {
        return 0.0;
    }

    return ((double)frequency_control_word / NCO_TWO_POW_64_D) *
           nco_port->input_rate_hz;
}

/* ---- 输出参数管理 ------------------------------------------------------- */

/**
 * @brief  设置输出峰值幅度与直流偏置（均为 12 位 DAC 码）。
 * @param  amplitude 峰值幅度码。
 * @param  dc_offset 直流偏置码。
 * @retval true  设置成功；false 参数非法（幅度 > 偏置 或 幅度+偏置 > 4095）。
 */
bool NCO_DAC_SetOutputLevel(uint16_t amplitude, uint16_t dc_offset)
{
    uint32_t primask;

    if ((amplitude > dc_offset) ||
        ((uint32_t)amplitude + (uint32_t)dc_offset > NCO_DAC_MAX_CODE))
    {
        return false;
    }

    primask = NCO_DAC_EnterCritical();
    nco_output_level = ((uint32_t)dc_offset << 16) | amplitude;
    SignalPort_MemoryBarrier(nco_port);
    NCO_DAC_ExitCritical(primask);
    return true;
}

/**
 * @brief  设置输出波形类型（正弦/方波/三角波）。
 * @param  waveform 目标波形枚举值。
 * @retval true  设置成功；false 未初始化或波形非法。
 * @note   运行时修改在随后重填的块上生效，不影响当前已装载块。
 */
bool NCO_DAC_SetOutputWaveform(NCO_OutputWaveform waveform)
{
    uint32_t primask;

    if ((!nco_initialized) ||
        !NCO_Waveform_IsOutputWaveformValid(waveform))
    {
        return false;
    }

    primask = NCO_DAC_EnterCritical();
    nco_output_waveform = (uint32_t)waveform;
    SignalPort_MemoryBarrier(nco_port);
    NCO_DAC_ExitCritical(primask);
    return true;
}

/**
 * @brief  获取当前输出波形类型。
 * @retval 当前波形枚举值。
 */
NCO_OutputWaveform NCO_DAC_GetOutputWaveform(void)
{
    NCO_OutputWaveform waveform;
    uint32_t primask = NCO_DAC_EnterCritical();

    waveform = (NCO_OutputWaveform)nco_output_waveform;
    NCO_DAC_ExitCritical(primask);
    return waveform;
}

/**
 * @brief  设置仅作用于 DAC 输出的整数倍频系数。
 * @param  multiplier 倍频系数，须满足基频 × 系数 ≤ 1 MHz 上限。
 * @retval true  设置成功；false 未初始化或系数超限。
 * @note   IQ 检测与 PI 环仍跟踪基频，倍频只影响渲染相位，可能引起相位跳变。
 */
bool NCO_DAC_SetOutputFrequencyMultiplier(uint32_t multiplier)
{
    double base_frequency_hz;
    bool accepted;
    uint32_t primask;

    if (!nco_initialized)
    {
        return false;
    }

    primask = NCO_DAC_EnterCritical();
    base_frequency_hz =
        ((double)nco_frequency_control_word / NCO_TWO_POW_64_D) *
        nco_port->input_rate_hz;
    accepted = NCO_FrequencyMultiplier_IsValid(multiplier,
                                                base_frequency_hz);
    if (accepted)
    {
        nco_output_frequency_multiplier = multiplier;
        SignalPort_MemoryBarrier(nco_port);
    }
    NCO_DAC_ExitCritical(primask);
    return accepted;
}

/**
 * @brief  获取当前输出倍频系数。
 * @retval 当前倍频系数。
 */
uint32_t NCO_DAC_GetOutputFrequencyMultiplier(void)
{
    uint32_t multiplier;
    uint32_t primask = NCO_DAC_EnterCritical();

    multiplier = nco_output_frequency_multiplier;
    NCO_DAC_ExitCritical(primask);
    return multiplier;
}

/**
 * @brief  获取当前基频下允许的最大输出倍频系数。
 * @retval 最大倍频系数（由 1 MHz 输出上限决定）。
 */
uint32_t NCO_DAC_GetMaximumOutputFrequencyMultiplier(void)
{
    return NCO_FrequencyMultiplier_GetMaximum(NCO_DAC_GetFrequencyHz());
}

/**
 * @brief  获取当前实际输出频率（基频 × 倍频系数）。
 * @retval 输出频率（Hz）。
 */
double NCO_DAC_GetOutputFrequencyHz(void)
{
    uint64_t frequency_control_word;
    uint32_t multiplier;
    uint32_t primask = NCO_DAC_EnterCritical();

    frequency_control_word = nco_frequency_control_word;
    multiplier = nco_output_frequency_multiplier;
    NCO_DAC_ExitCritical(primask);

    return NCO_FrequencyMultiplier_GetOutputHz(
        NCO_DAC_ControlWordToFrequencyHz(frequency_control_word),
        multiplier);
}
