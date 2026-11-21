/* 主机集成测试：真实 App/NCO/算法搭配内存端口，不包含任何 STM32 头文件。 */
#include "dpll_app.h"
#include "nco_dac.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    uint32_t critical_depth; /* 模拟可嵌套临界区，退出后必须回到原深度 */
    uint32_t cycles;         /* 模拟 uint32_t CPU 计数器回绕 */
    uint32_t failure;        /* 0 成功，1 装载失败，2 输入失败，3 主触发失败 */
    bool armed;             /* 输出已装载 */
    bool input_ready;       /* 输入已装载 */
    bool running;           /* 公共触发运行中 */
    char trace[32];         /* 启停调用序列，用于验证顺序与回滚 */
    size_t trace_length;    /* trace 已写字符数 */
    uint16_t preload;       /* 端口首个输出码 */
    SignalInputCallback receive; /* App 注册的真实回调 */
    SignalOutputCallback refill; /* NCO 注册的真实回调 */
    SignalErrorCallback error;   /* App 注册的错误回调 */
    void *input_context;    /* 输入订阅者上下文 */
    void *output_context;   /* 输出订阅者上下文 */
} FakePort;

static uint16_t output[2][NCO_DAC_BLOCK_SAMPLE_COUNT]; /* 替代 DMA 的输出存储 */
static uint16_t input[IQ_PHASE_MAX_BLOCK_SAMPLES];     /* 合成 ADC 数据块 */
static NCO_WaveformLut reference_lut; /* 标量逐点对照使用的波形表 */
static uint64_t input_frequency_control_word; /* 固定外部输入频率控制字。 */
static uint64_t input_frequency_control_word_offset; /* 低速场景的输入频率偏差。 */
static bool input_frequency_is_fixed; /* 低速场景模拟独立的外部时钟。 */
static uint64_t input_phase_continuous; /* 连续外部输入相位，跨块保持。 */
static bool input_phase_initialized; /* 是否已从首块参考建立外部输入相位。 */

/* 记录平台动作；不模拟具体芯片寄存器。 */
static void Record(FakePort *fake, char action)
{
    assert(fake->trace_length + 1U < sizeof(fake->trace));
    fake->trace[fake->trace_length++] = action;
    fake->trace[fake->trace_length] = '\0';
}
static uint32_t Enter(void *context)
{
    FakePort *fake = context; /* 当前替身端口 */
    return fake->critical_depth++;
}
static void Exit(void *context, uint32_t token)
{
    FakePort *fake = context; /* 当前替身端口 */
    assert(fake->critical_depth == token + 1U);
    fake->critical_depth = token;
}
static void Barrier(void *context) { (void)context; }
static uint32_t Cycles(void *context)
{
    FakePort *fake = context; /* 合成周期只测试回绕计算，不作性能证据 */
    fake->cycles += 100U;
    return fake->cycles;
}
static bool Arm(void *context, uint16_t preload,
                SignalOutputCallback refill, void *callback_context)
{
    FakePort *fake = context; /* 当前替身端口 */
    Record(fake, 'A');
    assert(!fake->running && !fake->armed);
    if (fake->failure == 1U) { return false; }
    fake->preload = preload;
    fake->refill = refill;
    fake->output_context = callback_context;
    fake->armed = true;
    return true;
}
static bool StartInput(void *context, SignalInputCallback receive,
                       SignalErrorCallback error, void *callback_context)
{
    FakePort *fake = context; /* 当前替身端口 */
    Record(fake, 'I');
    assert(fake->armed && !fake->running);
    if (fake->failure == 2U) { return false; }
    fake->receive = receive;
    fake->error = error;
    fake->input_context = callback_context;
    fake->input_ready = true;
    return true;
}
static bool StartOutput(void *context)
{
    FakePort *fake = context; /* 当前替身端口 */
    Record(fake, 'T');
    assert(fake->armed && fake->input_ready);
    if (fake->failure == 3U)
    {
        fake->armed = false; /* 遵守失败已撤销输出的接口约定 */
        return false;
    }
    fake->running = true;
    return true;
}
static bool StopOutput(void *context)
{
    FakePort *fake = context; /* 当前替身端口 */
    Record(fake, 'D');
    fake->armed = false;
    fake->running = false;
    return true;
}
static bool StopInput(void *context)
{
    FakePort *fake = context; /* 当前替身端口 */
    Record(fake, 'S');
    assert(!fake->running);
    fake->input_ready = false;
    return true;
}

static const SignalPortOps fake_ops = { /* 只替代硬件服务，不替代业务或数学模块 */
    .enter_critical = Enter, .exit_critical = Exit,
    .memory_barrier = Barrier, .cycles = Cycles,
    .arm_output = Arm, .start_output = StartOutput, .stop_output = StopOutput,
    .start_input = StartInput, .stop_input = StopInput,
};

/* 用逐点整数公式检查服务的整块展开渲染；精确匹配码值和 64 位参考相位。
 * start/fcw 单位均为 2^64/周；返回下一块起点，不依赖 NCO 私有状态。 */
static uint64_t CheckBlock(const SignalPort *port, uint32_t index,
                           uint64_t sequence, uint64_t start, uint64_t fcw)
{
    NCO_DAC_ReferenceMetadata metadata; /* 服务公开的参考元数据副本 */
    uint32_t multiplier = NCO_DAC_GetOutputFrequencyMultiplier(); /* 输出倍频 */
    NCO_OutputWaveform waveform = NCO_DAC_GetOutputWaveform(); /* 当前输出表 */
    assert(NCO_DAC_GetReferenceMetadata(sequence, &metadata));
    assert(metadata.start_phase == start +
           (fcw / 1U) * port->reference_sample_offset +
           ((fcw % 1U) * port->reference_sample_offset) / 1U);
    assert(metadata.first_frequency_control_word == fcw);
    assert(metadata.frequency_control_word == fcw);
    for (uint32_t sample = 0U; sample < port->output_block_samples; ++sample)
    {
        /* 标量参考每点计算整组与组内余数；不使用固件的 11 路展开循环。 */
        uint64_t phase = start + (sample / 1U) * fcw +
                         (fcw / 1U) * (sample % 1U) +
                         ((fcw % 1U) * (sample % 1U)) / 1U;
        assert(output[index][sample] == NCO_Waveform_RenderTable(
            NCO_Waveform_GetTable(&reference_lut, waveform),
            phase * multiplier, 1700U, 2048U));
    }
    return start + NCO_DAC_BASE_BLOCK_SAMPLE_COUNT * fcw;
}

/* 以独立 double 正弦生成 ADC 码，规定输入超前参考 0.25 rad、幅度 900 码。 */
static void GenerateInput(uint64_t sequence)
{
    NCO_DAC_ReferenceMetadata metadata; /* 对应采样时刻的参考 */
    uint64_t phase; /* ADC 当前采样的参考相位，2^64/周 */
    assert(NCO_DAC_GetReferenceMetadata(sequence, &metadata));
    if (!input_phase_initialized)
    {
        input_phase_continuous = metadata.start_phase;
        input_phase_initialized = true;
    }
    phase = input_phase_continuous;
    for (uint32_t sample = 0U; sample < IQ_PHASE_MAX_BLOCK_SAMPLES; ++sample)
    {
        double radians = (double)phase / 18446744073709551616.0 *
                         6.28318530717958647692; /* 转为弧度用于主机正弦 */
        input[sample] = (uint16_t)lround(2048.0 + 900.0 * sin(radians + 0.25));
        phase += input_frequency_is_fixed ? input_frequency_control_word :
                 metadata.frequency_control_word;
    }
    input_phase_continuous = phase;
}

/* 一个进程执行一个场景，保留生产固件的单实例生命周期限制。 */
int main(int argc, char **argv)
{
    FakePort fake = {.cycles = UINT32_MAX - 50U}; /* 覆盖周期计数回绕 */
    const bool low_rate = argc == 2 &&
        (strcmp(argv[1], "low_rate") == 0 ||
         strcmp(argv[1], "low_rate_offset") == 0);
    SignalPort port = {
        .ops = &fake_ops, .context = &fake,
        .input_rate_hz = low_rate ? 100000U : 1000000U,
        .output_rate_hz = low_rate ? 100000U : 1000000U,
        .input_block_samples = 512U, .output_block_samples = 512U,
        .reference_sample_offset = 0U, .output_buffers = {output[0], output[1]},
    }; /* 本进程整个测试期间有效的可移植端口 */
    DPLL_App_Config config = {
        .port = &port, .initial_frequency_hz = low_rate ? 2500.0 : 20000.0,
        .dac_amplitude = 1700U, .dac_dc_offset = 2048U,
        .pi_kp_hz_per_rad = DPLL_PI_DEFAULT_KP_HZ_PER_RAD,
        .pi_ki_hz_per_rad_s = DPLL_PI_DEFAULT_KI_HZ_PER_RAD_S,
        .pi_max_correction_hz = DPLL_PI_DEFAULT_MAX_CORRECTION_HZ,
    }; /* 业务配置 */
    uint64_t fcw, next_phase; /* 标量渲染参考的频率控制字和下一块起点 */
    assert(argc == 2);
    if (strcmp(argv[1], "offset") == 0) { port.reference_sample_offset = 3U; }
    if (strcmp(argv[1], "arm_failure") == 0) { fake.failure = 1U; }
    if (strcmp(argv[1], "input_failure") == 0) { fake.failure = 2U; }
    if (strcmp(argv[1], "trigger_failure") == 0) { fake.failure = 3U; }
    if (strcmp(argv[1], "low_rate_offset") == 0)
    {
        input_frequency_control_word_offset =
            (uint64_t)((5.0 / 100000.0) * 18446744073709551616.0 + 0.5);
    }

    assert(!DPLL_App_Start());
    assert(!DPLL_App_Init(NULL));
    config.initial_frequency_hz = NAN;
    assert(!DPLL_App_Init(&config));
    config.initial_frequency_hz = low_rate ? 2500.0 : 20000.0;
    port.input_rate_hz = low_rate ? 99999U : 999999U;
    assert(!DPLL_App_Init(&config));
    port.input_rate_hz = low_rate ? 100000U : 1000000U;
    config.dac_amplitude = 3000U;
    assert(!DPLL_App_Init(&config));
    config.dac_amplitude = 1700U;
    assert(fake.trace_length == 0U); /* 无效配置不得产生平台启动副作用 */
    assert(DPLL_App_Init(&config));
    assert(!DPLL_App_Init(&config));
    assert(DPLL_App_Enable(true));
    if (fake.failure != 0U)
    {
        const char *expected[] = {"", "A", "AID", "AITS"}; /* 各失败阶段回滚序列 */
        assert(!DPLL_App_Start());
        assert(strcmp(fake.trace, expected[fake.failure]) == 0);
        assert(!fake.armed && !fake.running && !fake.input_ready);
        fake.failure = 0U;
        fake.trace_length = 0U;
        fake.trace[0] = '\0';
    }
    assert(DPLL_App_Start());
    assert(strcmp(fake.trace, "AIT") == 0);
    assert(!DPLL_App_Start());
    assert(!NCO_DAC_Init(&port)); /* 运行中不得重置输出状态 */
    NCO_Waveform_Init(&reference_lut);
    fcw = NCO_DAC_GetFrequencyControlWord();
    input_frequency_is_fixed = low_rate;
    input_frequency_control_word = fcw + input_frequency_control_word_offset;
    next_phase = CheckBlock(&port, 0U, 0U, 0U, fcw);
    next_phase = CheckBlock(&port, 1U, 1U, next_phase, fcw);
    /* 首码等于第 0 样本之前最后一个插值步长对应的正弦码。 */
    assert(fake.preload == NCO_Waveform_Render(&reference_lut,
        0U - (fcw - ((fcw / 1U) * 0U +
        ((fcw % 1U) * 0U) / 1U)), 1700U, 2048U));

    const uint64_t block_count = low_rate ? 160U : 40U;
    for (uint64_t sequence = 0U; sequence < block_count; ++sequence)
    {
        uint32_t index = (uint32_t)(sequence % 2U); /* 交替完成两个输出块 */
        GenerateInput(sequence);
        if ((sequence % 2U) == 0U)
        {
            fake.receive(fake.input_context, input, 512U, sequence);
        }
        fcw = NCO_DAC_GetFrequencyControlWord();
        fake.refill(fake.output_context, output[index], 512U);
        next_phase = CheckBlock(&port, index, sequence + 2U, next_phase, fcw);
        if ((sequence % 2U) != 0U)
        {
            fake.receive(fake.input_context, input, 512U, sequence);
        }
        assert(g_adc_blocks_processed == sequence + 1U);
        if (low_rate || ((sequence + 1U) % 8U == 0U))
        {
            if (low_rate)
            {
                assert(isfinite(g_iq_phase_result.phase_rad));
            }
            else
            {
                assert(fabsf(g_iq_phase_result.phase_rad - 0.25f) < 0.001f);
            }
            assert(fabsf(g_iq_phase_result.magnitude - 900.0f) <
                   (low_rate ? 3.0f : 1.0f));
            assert(g_iq_phase_result.sample_count == (low_rate ? 512U : 4096U));
        }
    }
    assert(g_dpll_pi_snapshot.signal_valid);
    assert(g_dpll_pi_snapshot.update_applied);
    if (strcmp(argv[1], "low_rate_offset") == 0)
    {
        assert(fabsf(g_dpll_pi_snapshot.frequency_correction_hz - 5.0f) < 0.2f);
    }
    else if (strcmp(argv[1], "low_rate") == 0)
    {
        assert(fabsf(g_dpll_pi_snapshot.frequency_correction_hz) < 0.2f);
    }
    assert(g_dpll_pi_snapshot.reference_mismatch_count == 0U);
    assert(g_adc_processing_cycles_max > 0U);
    assert(g_nco_fill_cycles_max == 100U);
    fake.error(fake.input_context, SIGNAL_ERROR_INPUT, 123U);
    fake.error(fake.input_context, SIGNAL_ERROR_OUTPUT_UNDERRUN, 456U);
    assert(g_adc_error_count == 1U && g_adc_last_error_code == 123U);
    assert(g_dac_underrun_count == 1U);

    /* 错误块长被拒绝，过期参考必须进入参考丢失路径，不能混入累积器。 */
    fake.receive(fake.input_context, input, 51U, block_count);
    assert(g_adc_blocks_processed == block_count);
    fake.receive(fake.input_context, input, 512U, 1000U);
    assert(g_dpll_pi_snapshot.reference_mismatch_count == 1U);
    assert(DPLL_App_SetPhaseCalibration(0.0f, 0.0f));
    assert(DPLL_App_SetLockedPhaseDifferenceDegrees(45.0f));
    assert(DPLL_App_GetLockedPhaseDifferenceDegrees() == 45.0f);
    assert(DPLL_App_Enable(false));
    assert(NCO_DAC_SetFrequencyHz(12345.0));
    assert(DPLL_App_SetOutputWaveform(NCO_OUTPUT_WAVEFORM_TRIANGLE));
    assert(DPLL_App_SetOutputFrequencyMultiplier(3U));
    fcw = NCO_DAC_GetFrequencyControlWord();
    fake.refill(fake.output_context, output[0], 512U);
    (void)CheckBlock(&port, 0U, block_count + 2U, next_phase, fcw);
    assert(fabs(DPLL_App_GetOutputFrequencyHz() - 37035.0) < 1.0e-6);
    assert(fake.critical_depth == 0U);
    assert(NCO_DAC_Stop());
    assert(!fake.running && !fake.armed);
    assert(StopInput(&fake));
    puts("App/port integration passed: exact output and dynamic-rate IQ/PI checks");
    return 0;
}

