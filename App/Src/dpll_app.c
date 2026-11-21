/**
 * @file    dpll_app.c
 * @brief   App 层：组合 IQ、PI 与 NCO 服务，构成完整 DPLL 闭环；不持有外设句柄或 DMA 内存。
 *
 * 本层是 DPLL 的"胶水层"：它不直接操作 ADC/DAC/DMA，而是通过 SignalPort 注入的
 * 输入/输出流与运行环境（临界区、内存屏障、周期计数）把三个子模块串起来：
 *  - iq_phase：对输入块做 IQ 相位检测（运行时 1～8 块最小二乘累积，默认 8 块）；
 *  - dpll_pi：PI 环路控制器，输出频率修正量；
 *  - nco_dac：NCO 输出服务，渲染 DAC 波形并接受闭环 FCW。
 *
 * 关键设计点：
 *  - 输入回调 DPLL_App_ProcessInput 是闭环的"心跳"：每个 ADC 块触发一次
 *    "测相 → 更新 PI 目标 → PI 校正 → 回写 NCO FCW"。
 *  - 共享参数（相位校准、锁定相位差、PI 状态）统一通过临界区保护，主循环与
 *    中断回调互不竞争；volatile 只用于调试观测变量，不替代同步。
 *  - 错误回调只累计诊断计数，不承担硬件恢复策略，恢复决策留给上层。
 */
/*
 * 在 main.c 中启动锁相环的推荐流程
 *
 * 1. 调用 HAL_Init，完成 HAL 库基础初始化。
 * 2. 调用 BSP_Signal_PreparePlatform，开启 H7 的 I-Cache、D2 SRAM 时钟和 DWT 计时。
 * 3. 调用 SystemClock_Config，完成系统、总线、ADC 和定时器时钟配置。
 * 4. 调用 BSP_Signal_InitPeripherals，初始化 GPIO、DMA、ADC1、DAC1、TIM1 和 TIM8。
 * 5. 准备 SignalStreamConfig，填写 ADC 采样率、DAC 升采样倍数、输出开关、块长度和可选缓冲区。
 * 6. 调用 BSP_Signal_Configure，绑定 ADC、DAC、TIM1、TIM8 和 DMA 缓冲区，并计算实际采样率。
 * 7. 填充 DPLL_App_Config，将 BSP_Signal_Configure 返回的 SignalPort 写入 port，
 *    同时填写初始频率、DAC 幅值、直流偏置、PI 比例增益、PI 积分增益和最大修正范围。
 * 8. 调用 DPLL_App_Init，创建 IQ、PI 和 NCO 的应用状态。该函数只保存配置，不启动 ADC、DAC 和定时器。
 * 9. 按项目需要调用 DPLL_App_SetPhaseCalibration、DPLL_App_SetLockedPhaseDifferenceDegrees、
 *    DPLL_App_SetOutputWaveform 和 DPLL_App_SetOutputFrequencyMultiplier。
 * 10. 调用 DPLL_App_Enable(true) 使能闭环。
 * 11. 调用 DPLL_App_Start，启动 DAC DMA、ADC DMA 以及输入和输出定时器。
 * 12. 主循环中持续调用 DPLL_App_ProcessRuntimeControl，处理运行时采样配置和 PI 增益更新请求。
 *
 * 启动失败时，必须检查每一步的返回值并进入工程的错误处理流程。
 * DPLL_App_Start 成功后，ADC DMA 完成回调会调用 DPLL_App_ProcessInput，依次完成 IQ 测相、
 * PI 频率修正和 NCO 频率控制字更新。DAC DMA 回调负责输出缓冲区的连续填充。
 * 停止或修改采样配置时，应通过 DPLL_App_ConfigureSignal 处理，避免运行中的 DMA 和定时器状态不一致。
 */
#include "dpll_app.h"

#include "nco_dac.h"

#include <math.h>
#include <stddef.h>

/** 每次输入回调样本数由端口提供，不再锁定 512 点和固定采样率。 */
#define ADC_DMA_BLOCK_SAMPLES (app_config.port->input_block_samples)
_Static_assert(IQ_PHASE_ACCUMULATION_BLOCKS == 8U,
               "当前 IQ 累积器接口的最大观测窗口必须为 8 块");
/* PI 周期在初始化时按实际速率和动态 IQ 窗口计算，保留分频误差。 */

/* ---- 应用内部状态 ------------------------------------------------------- */
static DPLL_App_Config app_config;              /* 应用配置副本（Init 时保存） */
static IQ_PhaseState iq_phase_state;            /* IQ 相位检测器状态 */
static IQ_PhaseAccumulator iq_phase_accumulator;/* IQ 跨块累积器（运行时 1～8 块） */
static uint16_t iq_accumulation_blocks = IQ_PHASE_ACCUMULATION_BLOCKS; /* 当前窗口块数 */
static DPLL_PI_State dpll_pi_state;             /* PI 环路控制器状态 */
static DPLL_App_PhaseCalibration phase_calibration; /* 频率相关相位校准系数 */
static float locked_phase_difference_deg;       /* 锁定相位差（度，DAC-ADC） */
static bool app_initialized;                    /* 应用是否已初始化 */
static bool app_started;                        /* 应用是否已启动（外设运行中） */
static bool app_enabled;                        /* 闭环是否已使能 */

/**
 * @brief  清空 IQ 累积器并恢复当前采样配置对应的观测窗口长度。
 * @param  无。
 * @retval 无。
 * @note   仅在初始化、启动或输入回调的窗口边界调用；窗口块数由初始化阶段计算，
 *         取值范围为 1～IQ_PHASE_ACCUMULATION_BLOCKS。
 */
static void DPLL_App_ResetIQAccumulator(void)
{
    IQ_Phase_AccumulatorReset(&iq_phase_accumulator);
    (void)IQ_Phase_AccumulatorSetBlockCount(
        &iq_phase_accumulator, iq_accumulation_blocks);
}

/**
 * @brief  根据 IQ 观测窗口长度换算实际使用的 PI 增益。
 * @param  nominal_kp_hz_per_rad 标称比例增益，单位 Hz/rad。
 * @param  nominal_ki_hz_per_rad_s 标称积分增益，单位 Hz/(rad*s)。
 * @param  effective_kp_hz_per_rad 非空输出地址，写入当前窗口对应的比例增益。
 * @param  effective_ki_hz_per_rad_s 非空输出地址，写入当前窗口对应的积分增益。
 * @retval 无。
 * @note   标称值以默认 8 块窗口为基准；窗口缩短后按块数比缩放比例项、按其平方
 *         缩放积分项，降低短观测窗的噪声和额外相位延迟对环路稳定性的影响。
 */
static void DPLL_App_SchedulePIGains(float nominal_kp_hz_per_rad,
                                     float nominal_ki_hz_per_rad_s,
                                     float *effective_kp_hz_per_rad,
                                     float *effective_ki_hz_per_rad_s)
{
    const float window_ratio = (float)iq_accumulation_blocks /
                               (float)IQ_PHASE_ACCUMULATION_BLOCKS;

    *effective_kp_hz_per_rad = nominal_kp_hz_per_rad * window_ratio;
    *effective_ki_hz_per_rad_s = nominal_ki_hz_per_rad_s *
                                 window_ratio * window_ratio;
}

/* ---- 调试观测全局变量（volatile，调试器可直接读取） --------------------- */
volatile IQ_PhaseResult g_iq_phase_result;      /* 最近一次 IQ 相位检测结果 */
volatile DPLL_PI_Snapshot g_dpll_pi_snapshot;   /* PI 控制器最新快照 */
volatile uint32_t g_dpll_snapshot_sequence;     /* 快照序号（奇偶递增检测中断读） */
volatile uint64_t g_adc_last_block_sequence;    /* 最近处理的 ADC 块序号 */
volatile uint32_t g_adc_blocks_processed;       /* 已成功处理的 ADC 块累计数 */
volatile uint32_t g_adc_error_count;            /* ADC 错误累计次数 */
volatile uint32_t g_adc_last_error_code;        /* 最近一次 ADC 错误码 */
volatile uint32_t g_dac_underrun_count;         /* DAC 下溢累计次数 */
volatile uint32_t g_nco_fill_cycles_last;       /* 最近一次 NCO 重填耗时（周期） */
volatile uint32_t g_nco_fill_cycles_max;        /* NCO 重填耗时历史最大值（周期） */
volatile uint32_t g_adc_processing_cycles_last; /* 最近一次 ADC 处理耗时（周期） */
volatile uint32_t g_adc_processing_cycles_max;  /* ADC 处理耗时历史最大值（周期） */

/* ---- 回调与运行环境的前置声明 ----------------------------------------- */

/**
 * @brief  输入事件回调（SignalInputCallback）：执行动态窗口 IQ 累积与 PI 更新。
 * @param  context  未使用（本应用以 NULL 注册）。
 * @param  samples  本块 ADC 样本（未缩放码值），仅在回调内只读有效。
 * @param  count    本块样本数。
 * @param  sequence 本块序号，对应输入时间轴。
 * @note   回调不能重入、不能留存 samples；实现见文件末尾。
 */
static void DPLL_App_ProcessInput(void *context, const uint16_t *samples,
                                  uint32_t count, uint64_t sequence);

/**
 * @brief  流错误回调（SignalErrorCallback）：只累计诊断，不恢复硬件。
 * @param  context 未使用（本应用以 NULL 注册）。
 * @param  error   错误类型。
 * @param  code    平台诊断码。
 * @note   实现见文件末尾；恢复策略由上层决定。
 */
static void DPLL_App_OnStreamError(void *context, SignalError error, uint32_t code);

/**
 * @brief  进入临界区：通过注入的运行环境保护共享参数。
 * @retval 进入前的中断屏蔽状态（token），供 DPLL_App_ExitCritical 恢复。
 * @note   初始化前只允许单线程读取默认值；初始化后保护与输入回调共享的状态。
 */
static uint32_t DPLL_App_EnterCritical(void)
{
    return SignalPort_EnterCritical(app_config.port);
}

/**
 * @brief  退出临界区：恢复进入前的中断屏蔽状态。
 * @param  token 来自匹配的 DPLL_App_EnterCritical 返回值。
 * @note   必须与进入调用严格配对，避免误开中断。
 */
static void DPLL_App_ExitCritical(uint32_t token)
{
    SignalPort_ExitCritical(app_config.port, token);
}

/**
 * @brief  校验应用配置结构是否完整且参数合法。
 * @param  config 待校验的配置指针。
 * @retval true  端口合法、初始频率低于实际 ADC 奈奎斯特率且 DAC 幅值不削波；
 *         false 任一条件不满足。
 * @note   先验证全部边界，避免在输出模块初始化后才发现幅度不合法。
 */
static bool DPLL_App_ConfigIsValid(const DPLL_App_Config *config)
{
    return config != NULL && SignalPort_IsValid(config->port) &&
           isfinite(config->initial_frequency_hz) &&
           config->initial_frequency_hz > 0.0 &&
           config->initial_frequency_hz < config->port->input_rate_hz * 0.5 &&
           config->port->input_block_samples <= IQ_PHASE_MAX_BLOCK_SAMPLES &&
           isfinite(config->pi_kp_hz_per_rad) && config->pi_kp_hz_per_rad >= 0.0f &&
           isfinite(config->pi_ki_hz_per_rad_s) && config->pi_ki_hz_per_rad_s >= 0.0f &&
           isfinite(config->pi_max_correction_hz) && config->pi_max_correction_hz >= 0.0f &&
           config->dac_amplitude <= config->dac_dc_offset &&
           (uint32_t)config->dac_amplitude + config->dac_dc_offset <= 4095U;
}

/**
 * @brief  在临界区内原子读取相位校准参数与当前锁定相位差。
 * @param  output_minus_input_phase_deg 输出参数（可为 NULL），返回锁定的
 *                                     输出减输入相位差（单位：度）。
 * @retval 当前的相位校准配置副本。
 */
static DPLL_App_PhaseCalibration DPLL_App_ReadPhaseConfiguration(
    float *output_minus_input_phase_deg)
{
    DPLL_App_PhaseCalibration calibration;
    uint32_t primask = DPLL_App_EnterCritical();

    calibration = phase_calibration;
    if (output_minus_input_phase_deg != NULL)
    {
        *output_minus_input_phase_deg = locked_phase_difference_deg;
    }
    DPLL_App_ExitCritical(primask);
    return calibration;
}

/**
 * @brief  根据参考频率控制字计算 PI 目标相位并应用（无扰更新）。
 * @param  reference_frequency_control_word 实际到达 DAC 的块对应的 FCW，
 *                                          用于换算当前频率后套用相位校准。
 * @retval true  目标相位成功写入 PI 控制器；
 *         false 相位校准换算失败或写入目标相位失败。
 */
static bool DPLL_App_ApplyPhaseTarget(
    uint64_t reference_frequency_control_word)
{
    DPLL_App_PhaseCalibration calibration;
    float output_minus_input_phase_deg;
    float target_phase_rad;
    uint32_t primask;
    bool success;

    calibration = DPLL_App_ReadPhaseConfiguration(
        &output_minus_input_phase_deg);

    if (!DPLL_PhaseCalibration_CalculateLockTargetRadians(
            &calibration,
            NCO_DAC_ControlWordToFrequencyHz(
                reference_frequency_control_word),
            output_minus_input_phase_deg,
            &target_phase_rad))
    {
        return false;
    }

    primask = DPLL_App_EnterCritical();
    success = DPLL_PI_SetTargetPhase(&dpll_pi_state, target_phase_rad);
    DPLL_App_ExitCritical(primask);
    return success;
}

/**
 * @brief  初始化 DPLL 应用：绑定端口、初始化 NCO/IQ/PI，并复位内部状态。
 * @param  config 指向应用配置结构的指针，须通过 DPLL_App_ConfigIsValid 校验。
 * @retval true  初始化成功；false 已初始化、参数非法或
 *               任一子模块初始化失败。
 * @note   仅做静态配置，不启动任何外设；启动动作在 DPLL_App_Start 中完成。
 *         PI 周期由实际采样窗口计算，防止分频量化误差造成环路增益偏差。
 */
bool DPLL_App_Init(const DPLL_App_Config *config)
{
    DPLL_PI_Config dpll_config;
    uint64_t nominal_frequency_control_word;
    float effective_kp_hz_per_rad;
    float effective_ki_hz_per_rad_s;

    /* 防重入并校验配置；采样率和 PI 周期不再与固定宏比较。 */
    if (app_initialized || !DPLL_App_ConfigIsValid(config))
    {
        return false;
    }

    app_config = *config;

    if (!NCO_DAC_Init(app_config.port) ||
        !NCO_DAC_SetOutputCallback(config->output_ready, config->output_context))
    {
        return false;
    }

    /* 先配置 DAC 幅值/偏置与初始频率，二者失败即视为初始化失败。 */
    if ((!NCO_DAC_SetOutputLevel(app_config.dac_amplitude,
                                 app_config.dac_dc_offset)) ||
        (!NCO_DAC_SetFrequencyHz(app_config.initial_frequency_hz)))
    {
        return false;
    }

    /* 以 NCO 当前 FCW 为准初始化 IQ 相位检测器与 PI 控制器，保证三者同频。 */
    nominal_frequency_control_word = NCO_DAC_GetFrequencyControlWord();
    IQ_Phase_Init(&iq_phase_state, 0U, nominal_frequency_control_word);
    {
        const double desired_blocks =
            (double)DPLL_PI_DEFAULT_UPDATE_PERIOD_S *
            app_config.port->input_rate_hz /
            (double)ADC_DMA_BLOCK_SAMPLES;
        uint32_t blocks = (uint32_t)lround(desired_blocks);

        if (blocks < 1U)
        {
            blocks = 1U;
        }
        if (blocks > IQ_PHASE_ACCUMULATION_BLOCKS)
        {
            blocks = IQ_PHASE_ACCUMULATION_BLOCKS;
        }
        iq_accumulation_blocks = (uint16_t)blocks;
    }
    DPLL_App_ResetIQAccumulator();

    DPLL_PI_GetDefaultConfig(&dpll_config,
                             nominal_frequency_control_word,
                             app_config.port->input_rate_hz);
    DPLL_App_SchedulePIGains(config->pi_kp_hz_per_rad,
                             config->pi_ki_hz_per_rad_s,
                             &effective_kp_hz_per_rad,
                             &effective_ki_hz_per_rad_s);
    dpll_config.update_period_s = (float)((double)ADC_DMA_BLOCK_SAMPLES *
        iq_accumulation_blocks / app_config.port->input_rate_hz);
    dpll_config.kp_hz_per_rad = effective_kp_hz_per_rad;
    dpll_config.ki_hz_per_rad_s = effective_ki_hz_per_rad_s;
    if (config->pi_max_correction_hz > 0.0f)
    { dpll_config.max_frequency_correction_hz = config->pi_max_correction_hz; }
    if (!DPLL_PI_Init(&dpll_pi_state, &dpll_config))
    {
        return false;
    }

    phase_calibration.intercept_deg = 0.0f;
    phase_calibration.slope_deg_per_khz = 0.0f;
    locked_phase_difference_deg = 0.0f;
    app_enabled = false;
    app_started = false;
    app_initialized = true;
    return true;
}

/**
 * @brief  启动信号采集与输出：装载 NCO、启动输入，最后启动主触发。
 * @retval true  全部启动成功；false 任一环节失败（失败时回滚已启动的外设）。
 * @note   顺序固定为 Arm 输出 → 启动输入 → 启动主触发，让输入首块等待公共触发，
 *         避免首块输入与输出元数据在时间轴上错位。
 */
bool DPLL_App_Start(void)
{
    /* 先让输入等待公共触发，避免首块输入与输出元数据错位。 */
    const SignalPort *port = app_config.port;
    if (!app_initialized || app_started) { return false; }
    /* 每次启动重新从块序号 0 建立参考，不能混入停止前的 IQ 窗口。 */
    DPLL_App_ResetIQAccumulator();
    DPLL_PI_Reset(&dpll_pi_state);
    DPLL_PI_Enable(&dpll_pi_state, app_enabled);
    if (app_enabled)
    {
        (void)NCO_DAC_SetClosedLoopFrequencyControlWord(
            dpll_pi_state.config.nominal_frequency_control_word);
    }
    if (!NCO_DAC_Arm()) { return false; }
    if (!port->ops->start_input(port->context, DPLL_App_ProcessInput,
                                DPLL_App_OnStreamError, NULL))
    {
        (void)NCO_DAC_Stop();
        return false;
    }
    if (!NCO_DAC_StartTrigger())
    {
        (void)port->ops->stop_input(port->context);
        return false;
    }
    app_started = true;
    return true;
}

/**
 * @brief 停止共同主触发和双 DMA，为重新配置采样参数留出安全窗口。
 * @param 无。
 * @retval true 已停止；false 未启动或至少一个平台停止操作失败。
 * @note 仅在主循环调用，失败也不自动重新启动；保留闭环使能意图和项目参数。
 */
bool DPLL_App_Stop(void)
{
    if (!app_initialized || !app_started) { return false; }
    bool output_ok = NCO_DAC_Stop(); /* 先停止共同主时钟，避免 DMA 停止期间继续触发。 */
    bool input_ok = app_config.port->ops->stop_input(app_config.port->context); /* 再撤销输入。 */
    app_started = false;
    return output_ok && input_ok;
}

/**
 * @brief 在运行中或启动前修改信号参数，按原状态停流重配并重新锁相。
 * @param config 项目采样请求，借用的缓冲区在运行期间必须一直有效。
 * @retval true 配置成功且按原运行状态重启；false 预检或硬件配置/启动失败。
 * @note 主循环调用；预检失败不停止原信号，硬件阶段失败保持停止供调用者检查。
 */
bool DPLL_App_ConfigureSignal(const SignalStreamConfig *config)
{
    if (!app_initialized || config == NULL || app_config.port->ops->configure == NULL)
    { return false; }
    const SignalPort *port = app_config.port; /* 端口单例，重配置后地址不变。 */
    SignalStreamInfo candidate; /* 先获取实际分频结果，不能只验证请求值。 */
    if (!port->ops->configure(port->context, config, true, &candidate) ||
        config->adc_block_samples > IQ_PHASE_MAX_BLOCK_SAMPLES) { return false; }
    const double limit = app_config.pi_max_correction_hz > 0.0f ?
        app_config.pi_max_correction_hz : DPLL_PI_DEFAULT_MAX_CORRECTION_HZ; /* 保留用户限幅。 */
    if (app_config.initial_frequency_hz - limit < 0.0 ||
        app_config.initial_frequency_hz + limit >= candidate.adc_sample_rate_hz * 0.5)
    { return false; }
    const bool restart = app_started; /* 修改前的运行/业务状态。 */
    const bool enabled = app_enabled;
    const DPLL_App_PhaseCalibration calibration = phase_calibration;
    const float phase = locked_phase_difference_deg;
    const NCO_OutputWaveform waveform = NCO_DAC_GetOutputWaveform();
    const uint32_t multiplier = NCO_DAC_GetOutputFrequencyMultiplier();
    DPLL_App_Config saved = app_config; /* 参数地址不能直接别名 app_config，避免重初始化覆盖。 */
    if (restart && !DPLL_App_Stop()) { return false; }
    if (!port->ops->configure(port->context, config, false, NULL)) { return false; }
    app_initialized = false;
    if (!DPLL_App_Init(&saved) ||
        !DPLL_App_SetPhaseCalibration(calibration.intercept_deg, calibration.slope_deg_per_khz) ||
        !DPLL_App_SetLockedPhaseDifferenceDegrees(phase) ||
        !DPLL_App_SetOutputWaveform(waveform) ||
        !DPLL_App_SetOutputFrequencyMultiplier(multiplier) ||
        !DPLL_App_Enable(enabled)) { return false; }
    return !restart || DPLL_App_Start();
}

/**
 * @brief 运行时更新 PI 标称比例和积分增益，不停止采样。
 * @param kp_hz_per_rad 标称比例增益，Hz/rad，有限非负数。
 * @param ki_hz_per_rad_s 标称积分增益，Hz/(rad*s)，有限非负数。
 * @retval true 已更新；false 未初始化、非法参数或无扰补偿超出积分限幅。
 * @note 主循环调用，在短临界区内更新；短 IQ 窗口沿用初始化时的增益调度规则。
 */
bool DPLL_App_SetPIGains(float kp_hz_per_rad, float ki_hz_per_rad_s)
{
    float effective_kp_hz_per_rad;
    float effective_ki_hz_per_rad_s;

    if (!app_initialized) { return false; }
    if (!isfinite(kp_hz_per_rad) || !isfinite(ki_hz_per_rad_s) ||
        kp_hz_per_rad < 0.0f || ki_hz_per_rad_s < 0.0f)
    { return false; }
    DPLL_App_SchedulePIGains(kp_hz_per_rad, ki_hz_per_rad_s,
                             &effective_kp_hz_per_rad,
                             &effective_ki_hz_per_rad_s);
    uint32_t token = DPLL_App_EnterCritical(); /* 排除输入处理与参数写入竞争。 */
    bool success = DPLL_PI_SetGains(&dpll_pi_state,
                                    effective_kp_hz_per_rad,
                                    effective_ki_hz_per_rad_s);
    if (success)
    {
        app_config.pi_kp_hz_per_rad = kp_hz_per_rad;
        app_config.pi_ki_hz_per_rad_s = ki_hz_per_rad_s;
    }
    DPLL_App_ExitCritical(token);
    return success;
}

/**
 * @brief 读取 PI 当前增益。
 * @param kp_hz_per_rad 非空输出地址，Hz/rad。
 * @param ki_hz_per_rad_s 非空输出地址，Hz/(rad*s)。
 * @retval true 已复制；false 未初始化或空指针。
 * @note 在短临界区内复制，调用者拥有输出存储。
 */
bool DPLL_App_GetPIGains(float *kp_hz_per_rad, float *ki_hz_per_rad_s)
{
    if (!app_initialized || kp_hz_per_rad == NULL || ki_hz_per_rad_s == NULL) { return false; }
    uint32_t token = DPLL_App_EnterCritical(); /* 原子读取两个相关参数。 */
    *kp_hz_per_rad = dpll_pi_state.config.kp_hz_per_rad;
    *ki_hz_per_rad_s = dpll_pi_state.config.ki_hz_per_rad_s;
    DPLL_App_ExitCritical(token);
    return true;
}

/**
 * @brief 取得锁相输出内部存储地址、有效长度和采样率。
 * @param index 缓冲索引，0 或 1。
 * @param block 非空输出描述符地址。
 * @retval true 已渲染且复制成功；false 未初始化或数据不可用。
 * @note 运行中地址会循环覆盖，稳定读取请使用输出回调或先停止信号流。
 */
bool DPLL_App_GetOutputBuffer(uint32_t index, SignalOutputBlock *block)
{
    return app_initialized && NCO_DAC_GetOutputBuffer(index, block);
}

/**
 * @brief 查询实际采样率、频率误差、输出开关及处理截止时间。
 * @param info 非空输出地址，成功时复制实际配置。
 * @retval true 复制成功；false 未初始化或空指针。
 * @note 主循环调用；配置只能由同一主循环修改。
 */
bool DPLL_App_GetSignalInfo(SignalStreamInfo *info)
{
    if (!app_initialized || info == NULL) { return false; }
    *info = app_config.port->stream_info;
    return true;
}

/**
 * @brief  处理运行时采样配置和 PI 增益请求。
 * @param  control 运行时控制对象，调用期间必须保持有效。
 * @retval 无。
 * @note   仅由主循环调用；采样配置请求可能停流、重配定时器并重新锁相，
 *         PI 增益请求在短临界区内完成且不停止采样。
 */
void DPLL_App_ProcessRuntimeControl(DPLL_App_RuntimeControl *control)
{
    if (control == NULL) { return; }

    if (control->apply_signal_config)
    {
        control->apply_signal_config = false;
        control->signal_config_success =
            DPLL_App_ConfigureSignal(&control->signal_config);
        (void)DPLL_App_GetSignalInfo(&control->signal_info);
    }
    if (control->apply_pi_gains)
    {
        control->apply_pi_gains = false;
        control->pi_gains_success = DPLL_App_SetPIGains(
            control->pi_kp_hz_per_rad, control->pi_ki_hz_per_rad_s);
    }
}

/**
 * @brief  使能或禁用闭环：使能时让 PI 接管 NCO FCW，禁用时交还手动控制。
 * @param  enable true 进入闭环，false 退出闭环。
 * @retval true  状态切换成功；false 应用未初始化或底层切换失败。
 * @note   退出闭环会保留最后一个 FCW 不变。所有切换在临界区内完成。
 */
bool DPLL_App_Enable(bool enable)
{
    const DPLL_PI_Snapshot *snapshot;
    uint32_t primask;
    bool success = true;

    if (!app_initialized)
    {
        return false;
    }

    primask = DPLL_App_EnterCritical();
    if (enable)
    {
        /* 先复位 PI 运行时状态并取快照，再依次移交闭环写入权与当前 FCW；
         * 任一步失败则回滚到禁用态。 */
        DPLL_PI_Enable(&dpll_pi_state, true);
        snapshot = DPLL_PI_GetSnapshot(&dpll_pi_state);
        success = (snapshot != NULL) &&
                  NCO_DAC_SetClosedLoopEnabled(true) &&
                  NCO_DAC_SetClosedLoopFrequencyControlWord(
                      snapshot->output_frequency_control_word);
        if (!success)
        {
            (void)NCO_DAC_SetClosedLoopEnabled(false);
            DPLL_PI_Enable(&dpll_pi_state, false);
        }
    }
    else
    {
        success = NCO_DAC_SetClosedLoopEnabled(false);
        if (success)
        {
            DPLL_PI_Enable(&dpll_pi_state, false);
        }
    }
    if (success)
    {
        app_enabled = enable;
    }
    DPLL_App_ExitCritical(primask);
    return success;
}

/**
 * @brief  查询闭环当前是否处于使能状态。
 * @retval true  闭环已使能；false 未使能。
 */
bool DPLL_App_IsEnabled(void)
{
    bool enabled;
    uint32_t primask = DPLL_App_EnterCritical();

    enabled = app_enabled;
    DPLL_App_ExitCritical(primask);
    return enabled;
}

/**
 * @brief  设置频率相关相位校准系数（截距 + 斜率），并据此更新 PI 目标相位。
 * @param  intercept_deg      拟合截距（单位：度）。
 * @param  slope_deg_per_khz  拟合斜率（单位：度/千赫兹）。
 * @retval true  系数合法且目标相位更新成功；false 应用未初始化或参数非法。
 */
bool DPLL_App_SetPhaseCalibration(float intercept_deg,
                                  float slope_deg_per_khz)
{
    const DPLL_App_PhaseCalibration calibration = {
        .intercept_deg = intercept_deg,
        .slope_deg_per_khz = slope_deg_per_khz,
    };
    float output_minus_input_phase_deg;
    float target_phase_rad;
    uint32_t primask;
    bool success;

    (void)DPLL_App_ReadPhaseConfiguration(
        &output_minus_input_phase_deg);
    if ((!app_initialized) ||
        !DPLL_PhaseCalibration_IsValid(&calibration) ||
        !DPLL_PhaseCalibration_CalculateLockTargetRadians(
            &calibration,
            NCO_DAC_GetFrequencyHz(),
            output_minus_input_phase_deg,
            &target_phase_rad))
    {
        return false;
    }

    primask = DPLL_App_EnterCritical();
    success = DPLL_PI_SetTargetPhase(&dpll_pi_state, target_phase_rad);
    if (success)
    {
        phase_calibration = calibration;
    }
    DPLL_App_ExitCritical(primask);
    return success;
}

/**
 * @brief  获取当前的相位校准配置。
 * @retval 当前的相位校准配置副本。
 */
DPLL_App_PhaseCalibration DPLL_App_GetPhaseCalibration(void)
{
    return DPLL_App_ReadPhaseConfiguration(NULL);
}

/**
 * @brief  设置最终锁定时 DAC 输出相对 ADC 输入的相位差（单位：度）。
 * @param  output_minus_input_phase_deg 输出减输入相位差，正值表示 DAC 超前 ADC，
 *                                      负值表示 DAC 滞后 ADC。
 * @retval true  设置成功；false 应用未初始化、参数非有限值或目标更新失败。
 * @note   该相位在所有合法频率下始终生效，且会叠加到频率相关校准上。
 */
bool DPLL_App_SetLockedPhaseDifferenceDegrees(
    float output_minus_input_phase_deg)
{
    DPLL_App_PhaseCalibration calibration;
    float target_phase_rad;
    uint32_t primask;
    bool success;

    if ((!app_initialized) || !isfinite(output_minus_input_phase_deg))
    {
        return false;
    }

    calibration = DPLL_App_ReadPhaseConfiguration(NULL);
    if (!DPLL_PhaseCalibration_CalculateLockTargetRadians(
            &calibration,
            NCO_DAC_GetFrequencyHz(),
            output_minus_input_phase_deg,
            &target_phase_rad))
    {
        return false;
    }

    primask = DPLL_App_EnterCritical();
    success = DPLL_PI_SetTargetPhase(&dpll_pi_state, target_phase_rad);
    if (success)
    {
        locked_phase_difference_deg = output_minus_input_phase_deg;
    }
    DPLL_App_ExitCritical(primask);
    return success;
}

/**
 * @brief  获取当前设定的锁定相位差（单位：度）。
 * @retval 输出减输入的锁定相位差。
 */
float DPLL_App_GetLockedPhaseDifferenceDegrees(void)
{
    float phase_difference_deg;
    uint32_t primask = DPLL_App_EnterCritical();

    phase_difference_deg = locked_phase_difference_deg;
    DPLL_App_ExitCritical(primask);
    return phase_difference_deg;
}

/**
 * @brief  设置 DAC 输出波形类型（正弦/方波/三角波）。
 * @param  waveform 目标波形枚举值。
 * @retval true  设置成功；false 应用未初始化或波形参数非法。
 * @note   不改变 NCO 相位与频率，IQ 相位检测始终使用正弦参考。
 */
bool DPLL_App_SetOutputWaveform(NCO_OutputWaveform waveform)
{
    return app_initialized && NCO_DAC_SetOutputWaveform(waveform);
}

/**
 * @brief  获取当前 DAC 输出波形类型。
 * @retval 当前波形枚举值。
 */
NCO_OutputWaveform DPLL_App_GetOutputWaveform(void)
{
    return NCO_DAC_GetOutputWaveform();
}

/**
 * @brief  设置仅作用于 DAC 输出的整数倍频系数。
 * @param  multiplier 倍频系数，须 <= floor(1 MHz / 当前基频)。
 * @retval true  设置成功；false 应用未初始化或系数超限。
 * @note   IQ 检测与 PI 环仍跟踪基频，设置可能引起相位跳变。
 */
bool DPLL_App_SetOutputFrequencyMultiplier(uint32_t multiplier)
{
    return app_initialized &&
           NCO_DAC_SetOutputFrequencyMultiplier(multiplier);
}

/**
 * @brief  获取当前 DAC 输出倍频系数。
 * @retval 当前倍频系数。
 */
uint32_t DPLL_App_GetOutputFrequencyMultiplier(void)
{
    return NCO_DAC_GetOutputFrequencyMultiplier();
}

/**
 * @brief  获取当前基频下允许的最大输出倍频系数。
 * @retval 最大倍频系数（由 1 MHz 输出上限决定）。
 */
uint32_t DPLL_App_GetMaximumOutputFrequencyMultiplier(void)
{
    return NCO_DAC_GetMaximumOutputFrequencyMultiplier();
}

/**
 * @brief  获取当前实际输出频率（基频 x 倍频系数）。
 * @retval 输出频率（单位：Hz）。
 */
double DPLL_App_GetOutputFrequencyHz(void)
{
    return NCO_DAC_GetOutputFrequencyHz();
}

/**
 * @brief  输入事件回调实现：执行动态窗口 IQ 累积与一次 PI 更新。
 * @param  context  未使用。
 * @param  samples  本块 ADC 样本（未缩放码值），仅在回调内只读有效。
 * @param  count    本块样本数。
 * @param  sequence 本块序号，对应输入时间轴。
 * @note   这是 DPLL 闭环的实时入口：每块做一次 IQ 累积，攒满当前 1～8 块窗口后
 *         解出相位/幅度，再跑一次 PI 并回写 NCO FCW。回调不能重入、不能留存 samples；
 *         sequence 用实际采样时刻，不用主机到达时间替代。
 */
static void DPLL_App_ProcessInput(void *context, const uint16_t *samples,
                                  uint32_t count, uint64_t sequence)
{
    /* 下列局部状态均只在本次输入回调内有效；计时包含抢占开销。 */
    uint32_t start_cycles = SignalPort_Cycles(app_config.port);
    IQ_PhaseResult result;
    NCO_DAC_ReferenceMetadata reference;
    const DPLL_PI_Snapshot *snapshot;
    uint64_t output_frequency_control_word;
    IQ_PhaseAccumulationStatus iq_status = IQ_PHASE_ACCUMULATION_ERROR;
    bool reference_match;
    bool pi_input_ready = false;

    (void)context;
    if (!app_initialized || samples == NULL || count != ADC_DMA_BLOCK_SAMPLES)
    {
        return;
    }

    /* 采样 NCO 重填耗时的诊断值，供调试器观测。 */
    g_nco_fill_cycles_last = NCO_DAC_GetLastFillCycles();
    g_nco_fill_cycles_max = NCO_DAC_GetMaximumFillCycles();

    /* 按序号反查该输入块对应的 DAC 参考元数据（起始相位/FCW）。 */
    reference_match = NCO_DAC_GetReferenceMetadata(sequence,
                                                   &reference);
    if (reference_match)
    {
        /* 命中：把 IQ 相位检测器复位到 DAC 该块的起始相位，再做跨块累积。 */
        IQ_Phase_Reset(&iq_phase_state, reference.start_phase);
        iq_status = IQ_Phase_AccumulateU16Reference(
            &iq_phase_state,
            &iq_phase_accumulator,
            samples,
            ADC_DMA_BLOCK_SAMPLES,
            reference.first_frequency_control_word,
            reference.frequency_control_word,
            &result);
        if (iq_status != IQ_PHASE_ACCUMULATION_ERROR)
        {
            g_adc_blocks_processed++;
        }
        if (iq_status == IQ_PHASE_ACCUMULATION_READY)
        {
            /* 攒满当前观测窗口：更新观测结果，按实际 FCW 应用相位目标，
             * 为 PI 更新准备输入，然后重置累积器开始下一窗口。 */
            g_iq_phase_result = result;
            /* 使用实际到达 DAC 的那个块的 FCW。 */
            if (!DPLL_App_ApplyPhaseTarget(
                    reference.frequency_control_word))
            {
                iq_status = IQ_PHASE_ACCUMULATION_ERROR;
            }
            else
            {
                pi_input_ready = true;
            }
            DPLL_App_ResetIQAccumulator();
        }
        else if (iq_status == IQ_PHASE_ACCUMULATION_ERROR)
        {
            DPLL_App_ResetIQAccumulator();
        }
    }
    else
    {
        /* 未命中参考（记录过期或序号错位）：丢弃累积，避免用错误相位继续。 */
        DPLL_App_ResetIQAccumulator();
    }

    /* 满足条件时跑一次 PI：有就绪输入、参考不匹配或累积出错都要推进状态，
     * 让 PI 内部据此更新 signal_valid/reference_match 等诊断。 */
    if ((pi_input_ready || !reference_match ||
         (iq_status == IQ_PHASE_ACCUMULATION_ERROR)) &&
        DPLL_PI_Update(&dpll_pi_state,
                       pi_input_ready ? result.phase_rad : 0.0f,
                       pi_input_ready ? result.magnitude : 0.0f,
                       pi_input_ready,
                       reference_match,
                       &output_frequency_control_word))
    {
        /* PI 产生了新 FCW：回写 NCO 闭环（仅闭环使能时会被接受）。 */
        (void)NCO_DAC_SetClosedLoopFrequencyControlWord(
            output_frequency_control_word);
    }

    /* 把 PI 快照发布到观测变量；奇偶序号让调试器能检测到被中断的结构体读取。 */
    snapshot = DPLL_PI_GetSnapshot(&dpll_pi_state);
    if (snapshot != NULL)
    {
        /* 奇偶值让调试器能检测到被中断的结构体读取。 */
        g_dpll_snapshot_sequence++;
        SignalPort_MemoryBarrier(app_config.port);
        g_dpll_pi_snapshot = *snapshot;
        g_adc_last_block_sequence = sequence;
        SignalPort_MemoryBarrier(app_config.port);
        g_dpll_snapshot_sequence++;
    }

    g_adc_processing_cycles_last = SignalPort_Cycles(app_config.port) - start_cycles;
    if (g_adc_processing_cycles_last > g_adc_processing_cycles_max)
    {
        g_adc_processing_cycles_max = g_adc_processing_cycles_last;
    }
}

/**
 * @brief  流错误回调实现：只累计诊断计数，不承担硬件恢复。
 * @param  context 未使用。
 * @param  error   错误类型。
 * @param  code    平台诊断码（仅输入错误时记录）。
 * @note   端口已完成设备筛选，应用不依赖平台错误码的具体含义，仅做观测统计。
 */
static void DPLL_App_OnStreamError(void *context, SignalError error, uint32_t code)
{
    (void)context;
    if (!app_initialized) { return; }
    if (error == SIGNAL_ERROR_INPUT)
    {
        g_adc_error_count++;
        g_adc_last_error_code = code;
    }
    else if (error == SIGNAL_ERROR_OUTPUT_UNDERRUN)
    {
        g_dac_underrun_count++;
    }
}
