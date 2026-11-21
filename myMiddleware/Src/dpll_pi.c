/**
 * @file    dpll_pi.c
 * @brief   PI 环路滤波器、信号有效性与锁相状态；不依赖板级资源。
 *
 * 本模块实现 DPLL 的数字环路滤波器（PI 控制器）：把 IQ 鉴相得到的相位误差转换成
 * 频率修正量（Hz），再折算成 NCO 频率控制字（FCW）回写，从而完成闭环锁定。
 *
 * 关键设计点：
 *  - PI 结构：输出修正 = kp * 相位误差 + ki * ∫相位误差。kp 单位为 Hz/rad，
 *    ki 单位为 Hz/rad/s，修正量最终叠加到标称频率上。
 *  - 相位解卷绕：atan2 得到的原始相位落在 (-π, π]，需累加成"连续相位"，
 *    避免跨越 ±π 时出现 2π 跳变；发生整周滑移时同步修正 I 项，保证输出无阶跃。
 *  - 无扰更新（bumpless transfer）：运行时改目标相位（相位校准、锁定相位差）时，
 *    把 P 项的变化转移到 I 项，避免 NCO 频率突跳。
 *  - 抗积分饱和（anti-windup）：仅拒绝会"加深饱和"的积分，允许反向退饱和，
 *    既防止积分器无限累计又不至于过度保守。
 *  - 信号有效性滞回：幅度高于 acquire 阈值连续若干块后判有效，低于 release
 *    阈值连续若干块后判失锁，避免阈值附近反复翻转。
 */
#include "dpll_pi.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/** π 的 float32 常量。 */
#define DPLL_PI_PI_F          3.14159265358979323846f
/** 2π 的 float32 常量。 */
#define DPLL_PI_TWO_PI_F      6.28318530717958647692f
/** 4π 的 float32 常量，用于滑移检测的误差门限。 */
#define DPLL_PI_FOUR_PI_F     12.56637061435917295384f
/** 2^64 的 double 常量，64 位相位/FCW 的满量程。 */
#define DPLL_PI_TWO_POW_64_D  18446744073709551616.0

/* ---- 内部辅助函数 ------------------------------------------------------- */

/**
 * @brief  把相位折叠到 (-π, π] 区间。
 * @param  phase_rad 待折叠的相位（弧度）。
 * @retval 折叠后的相位。
 */
static float DPLL_PI_WrapPhase(float phase_rad)
{
    float wrapped = remainderf(phase_rad, DPLL_PI_TWO_PI_F);

    if (wrapped <= -DPLL_PI_PI_F)
    {
        wrapped += DPLL_PI_TWO_PI_F;
    }
    else if (wrapped > DPLL_PI_PI_F)
    {
        wrapped -= DPLL_PI_TWO_PI_F;
    }

    return wrapped;
}

/**
 * @brief  把相位误差限幅到 [-π, π]。
 * @param  error_rad 原始相位误差（弧度）。
 * @retval 限幅后的相位误差。
 */
static float DPLL_PI_ClampPhaseError(float error_rad)
{
    if (error_rad > DPLL_PI_PI_F)
    {
        return DPLL_PI_PI_F;
    }
    if (error_rad < -DPLL_PI_PI_F)
    {
        return -DPLL_PI_PI_F;
    }
    return error_rad;
}

/**
 * @brief  通用数值限幅。
 * @param  value 待限幅值。
 * @param  lower 下限。
 * @param  upper 上限。
 * @retval 限幅后的值。
 */
static float DPLL_PI_Clamp(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

/**
 * @brief  把频率控制字换算为频率（Hz）。
 * @param  control_word    频率控制字（2^64/周）。
 * @param  sample_rate_hz  采样率（Hz）。
 * @retval 对应频率。
 */
static double DPLL_PI_ControlWordToFrequency(uint64_t control_word,
                                              double sample_rate_hz)
{
    return ((double)control_word / DPLL_PI_TWO_POW_64_D) * sample_rate_hz;
}

/**
 * @brief  把频率（Hz）换算为频率控制字（2^64/周），四舍五入。
 * @param  frequency_hz    频率。
 * @param  sample_rate_hz  采样率（Hz）。
 * @retval 对应 FCW；频率非正返回 0，超出可表示范围返回 UINT64_MAX。
 */
static uint64_t DPLL_PI_FrequencyToControlWord(double frequency_hz,
                                               double sample_rate_hz)
{
    double scaled_word;

    if (frequency_hz <= 0.0)
    {
        return 0U;
    }

    scaled_word = (frequency_hz / sample_rate_hz) * DPLL_PI_TWO_POW_64_D;
    if (scaled_word >= DPLL_PI_TWO_POW_64_D)
    {
        return UINT64_MAX;
    }

    return (uint64_t)(scaled_word + 0.5);
}

/**
 * @brief  校验 PI 配置是否合法。
 * @param  config 待校验的配置。
 * @retval true  全部参数有限且满足约束（采样率>0、更新周期>0、kp/ki≥0、
 *               acquire>release、块数非 0，且标称频率 ± 最大修正落在 (0, 奈奎斯特)）；
 *         false 任一条件不满足。
 */
static bool DPLL_PI_ConfigIsValid(const DPLL_PI_Config *config)
{
    double nominal_frequency_hz;

    if ((config == NULL) || !isfinite(config->sample_rate_hz) ||
        (config->sample_rate_hz <= 0.0) ||
        !isfinite(config->update_period_s) || (config->update_period_s <= 0.0f) ||
        !isfinite(config->target_phase_rad) ||
        !isfinite(config->kp_hz_per_rad) || (config->kp_hz_per_rad < 0.0f) ||
        !isfinite(config->ki_hz_per_rad_s) || (config->ki_hz_per_rad_s < 0.0f) ||
        !isfinite(config->max_frequency_correction_hz) ||
        (config->max_frequency_correction_hz <= 0.0f) ||
        !isfinite(config->acquire_magnitude) ||
        !isfinite(config->release_magnitude) ||
        (config->release_magnitude < 0.0f) ||
        (config->acquire_magnitude <= config->release_magnitude) ||
        (config->acquire_blocks == 0U) || (config->release_blocks == 0U))
    {
        return false;
    }

    nominal_frequency_hz = DPLL_PI_ControlWordToFrequency(
        config->nominal_frequency_control_word, config->sample_rate_hz);

    /* 标称频率 ± 最大修正必须落在合法范围（下限 0，上限奈奎斯特）。 */
    return ((nominal_frequency_hz - config->max_frequency_correction_hz) >= 0.0) &&
           ((nominal_frequency_hz + config->max_frequency_correction_hz) <
            config->sample_rate_hz * 0.5);
}

/**
 * @brief  复位 PI 运行时状态，保留 config 与 initialized 标记。
 * @param  state 待复位的 PI 状态。
 * @note   复位后输出 FCW 回到标称值，reference_match 初始为 true。
 */
static void DPLL_PI_ResetRuntime(DPLL_PI_State *state)
{
    const bool initialized = state->initialized;
    const DPLL_PI_Config config = state->config;
    const double nominal_frequency_hz = DPLL_PI_ControlWordToFrequency(
        config.nominal_frequency_control_word, config.sample_rate_hz);

    memset(state, 0, sizeof(*state));
    state->config = config;
    state->initialized = initialized;
    state->snapshot.output_frequency_control_word =
        config.nominal_frequency_control_word;
    state->snapshot.output_frequency_hz = nominal_frequency_hz;
    state->snapshot.reference_match = true;
}

/**
 * @brief  相位解卷绕并检测整周滑移（cycle slip）。
 * @param  state        PI 状态。
 * @param  raw_phase_rad 本次的原始（折叠）相位。
 * @note   原始相位相邻两次的差值折叠后累加，得到连续相位；当连续误差超过 4π
 *         时认为发生整周滑移，折叠回整周期并把 P 项变化转移到 I 项，
 *         避免输出阶跃，同时累计 cycle_slip_count。
 */
static void DPLL_PI_TrackPhase(DPLL_PI_State *state, float raw_phase_rad)
{
    if (!state->phase_initialized)
    {
        /* 首次：直接用原始相位作为连续相位起点。 */
        state->previous_raw_phase_rad = raw_phase_rad;
        state->continuous_phase_rad = raw_phase_rad;
        state->phase_initialized = true;
    }
    else
    {
        /* 相邻原始相位差折叠到 (-π,π]，累加避免 2π 跳变。 */
        const float phase_delta = DPLL_PI_WrapPhase(
            raw_phase_rad - state->previous_raw_phase_rad);

        state->continuous_phase_rad += phase_delta;
        state->previous_raw_phase_rad = raw_phase_rad;
    }

    {
        const float continuous_error = state->continuous_phase_rad -
                                       state->config.target_phase_rad;

        if (fabsf(continuous_error) > DPLL_PI_FOUR_PI_F)
        {
            const float whole_cycles = roundf(continuous_error /
                                              DPLL_PI_TWO_PI_F);
            const uint32_t slipped_cycles = (uint32_t)fabsf(whole_cycles);
            const float old_error = DPLL_PI_ClampPhaseError(continuous_error);
            float new_error;

            state->continuous_phase_rad -= whole_cycles * DPLL_PI_TWO_PI_F;
            new_error = DPLL_PI_ClampPhaseError(
                state->continuous_phase_rad - state->config.target_phase_rad);

            /* 将未折叠相位移动整周期必须不造成控制器输出阶跃。
             * 将由此产生的 P 变化转移到 I 项。 */
            state->integrator_hz += state->config.kp_hz_per_rad *
                                    (old_error - new_error);
            state->integrator_hz = DPLL_PI_Clamp(
                state->integrator_hz,
                -state->config.max_frequency_correction_hz,
                state->config.max_frequency_correction_hz);
            if (state->error_initialized)
            {
                state->previous_error_rad = new_error;
            }

            if ((UINT32_MAX - state->snapshot.cycle_slip_count) < slipped_cycles)
            {
                state->snapshot.cycle_slip_count = UINT32_MAX;
            }
            else
            {
                state->snapshot.cycle_slip_count += slipped_cycles;
            }
        }
    }
}

/**
 * @brief  根据幅度更新信号有效性状态（带滞回的 acquire/release 计数）。
 * @param  state     PI 状态。
 * @param  magnitude 本次输入信号的幅度。
 * @note   幅度 ≥ acquire 阈值时递增 acquire 计数，攒满 acquire_blocks 判有效；
 *         幅度 < release 阈值时递增 release 计数，攒满 release_blocks 判失锁；
 *         介于两者之间时清零计数，保持当前状态（滞回）。
 */
static void DPLL_PI_UpdateSignalState(DPLL_PI_State *state, float magnitude)
{
    if (magnitude >= state->config.acquire_magnitude)
    {
        state->release_count = 0U;
        if (!state->signal_valid && (state->acquire_count < UINT16_MAX))
        {
            ++state->acquire_count;
        }

        if (state->acquire_count >= state->config.acquire_blocks)
        {
            state->signal_valid = true;
            state->acquire_count = state->config.acquire_blocks;
        }
    }
    else if (magnitude < state->config.release_magnitude)
    {
        state->acquire_count = 0U;
        if (state->signal_valid && (state->release_count < UINT16_MAX))
        {
            ++state->release_count;
        }

        if (state->release_count >= state->config.release_blocks)
        {
            state->signal_valid = false;
            state->release_count = state->config.release_blocks;
        }
    }
    else
    {
        /* 滞回区：清计数、保持现状，避免抖动。 */
        state->acquire_count = 0U;
        state->release_count = 0U;
    }
}

/* ---- 公开 API ------------------------------------------------------------ */

/**
 * @brief  用默认参数填充 PI 配置（kp/ki/修正上限/信号阈值等）。
 * @param  config                        输出参数，写入默认配置。
 * @param  nominal_frequency_control_word 标称频率控制字（2^64/周）。
 * @param  sample_rate_hz                 采样率（Hz）。
 */
void DPLL_PI_GetDefaultConfig(DPLL_PI_Config *config,
                              uint64_t nominal_frequency_control_word,
                              double sample_rate_hz)
{
    if (config == NULL)
    {
        return;
    }

    config->nominal_frequency_control_word = nominal_frequency_control_word;
    config->sample_rate_hz = sample_rate_hz;
    config->update_period_s = DPLL_PI_DEFAULT_UPDATE_PERIOD_S;
    config->target_phase_rad = 0.0f;
    config->kp_hz_per_rad = DPLL_PI_DEFAULT_KP_HZ_PER_RAD;
    config->ki_hz_per_rad_s = DPLL_PI_DEFAULT_KI_HZ_PER_RAD_S;
    config->max_frequency_correction_hz = DPLL_PI_DEFAULT_MAX_CORRECTION_HZ;
    config->acquire_magnitude = DPLL_PI_DEFAULT_ACQUIRE_MAGNITUDE;
    config->release_magnitude = DPLL_PI_DEFAULT_RELEASE_MAGNITUDE;
    config->acquire_blocks = DPLL_PI_DEFAULT_ACQUIRE_BLOCKS;
    config->release_blocks = DPLL_PI_DEFAULT_RELEASE_BLOCKS;
}

/**
 * @brief  初始化 PI 状态。
 * @param  state  待初始化的状态。
 * @param  config 配置，须通过 DPLL_PI_ConfigIsValid 校验。
 * @retval true  初始化成功；false state 为空或配置非法。
 * @note   初始化后目标相位折叠到 (-π,π]，并复位运行时状态。
 */
bool DPLL_PI_Init(DPLL_PI_State *state, const DPLL_PI_Config *config)
{
    if ((state == NULL) || !DPLL_PI_ConfigIsValid(config))
    {
        return false;
    }

    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->config.target_phase_rad = DPLL_PI_WrapPhase(config->target_phase_rad);
    state->initialized = true;
    DPLL_PI_ResetRuntime(state);
    return true;
}

/**
 * @brief  复位 PI 运行时状态（保留 config 与 initialized）。
 * @param  state 待复位的状态。
 */
void DPLL_PI_Reset(DPLL_PI_State *state)
{
    if ((state != NULL) && state->initialized)
    {
        DPLL_PI_ResetRuntime(state);
    }
}

/**
 * @brief  使能或禁用 PI 环。
 * @param  state  PI 状态。
 * @param  enable true 使能（首次使能时复位运行时状态），false 禁用。
 * @note   禁用后不产生 FCW 更新（update_applied 置 false）。
 */
void DPLL_PI_Enable(DPLL_PI_State *state, bool enable)
{
    if ((state == NULL) || !state->initialized)
    {
        return;
    }

    if (enable && !state->enabled)
    {
        DPLL_PI_ResetRuntime(state);
        state->enabled = true;
    }
    else if (!enable)
    {
        state->enabled = false;
        state->snapshot.update_applied = false;
    }

    state->snapshot.enabled = state->enabled;
}

/**
 * @brief  设置目标相位并做无扰更新（bumpless transfer）。
 * @param  state           PI 状态。
 * @param  target_phase_rad 新的目标相位（弧度）。
 * @retval true  设置成功；false 状态非法或目标相位非有限。
 * @note   若相位已初始化，把"旧误差与新误差之差 × kp"转移到 I 项，
 *         使 P 项变化被 I 项抵消，运行时改目标不会造成 NCO 频率突跳。
 */
bool DPLL_PI_SetTargetPhase(DPLL_PI_State *state, float target_phase_rad)
{
    float old_error;
    float new_error;
    float wrapped_target;

    if ((state == NULL) || !state->initialized ||
        !isfinite(target_phase_rad))
    {
        return false;
    }

    wrapped_target = DPLL_PI_WrapPhase(target_phase_rad);
    if (state->phase_initialized)
    {
        old_error = DPLL_PI_ClampPhaseError(
            state->continuous_phase_rad - state->config.target_phase_rad);
        new_error = DPLL_PI_ClampPhaseError(
            state->continuous_phase_rad - wrapped_target);

        /* 将 P 项的变化转移到 I 项，实现无扰目标更新。 */
        state->integrator_hz += state->config.kp_hz_per_rad *
                                (old_error - new_error);
        state->integrator_hz = DPLL_PI_Clamp(
            state->integrator_hz,
            -state->config.max_frequency_correction_hz,
            state->config.max_frequency_correction_hz);
        if (state->error_initialized)
        {
            state->previous_error_rad = new_error;
        }

        state->snapshot.phase_error_rad = new_error;
        state->snapshot.p_term_hz = state->config.kp_hz_per_rad * new_error;
        state->snapshot.i_term_hz = state->integrator_hz;
    }

    state->config.target_phase_rad = wrapped_target;
    return true;
}

/**
 * @brief 原子逻辑更新 PI 增益，并通过积分项抵消比例项的突变。
 * @param state 已初始化且与 Update 互斥的状态。
 * @param kp_hz_per_rad 新比例增益，Hz/rad，有限非负数。
 * @param ki_hz_per_rad_s 新积分增益，Hz/(rad*s)，有限非负数。
 * @retval true 更新成功；false 参数非法或无扰补偿超出积分限幅。
 * @note 本函数本身不关中断，调用者负责互斥；失败保持原状态不变。
 */
bool DPLL_PI_SetGains(DPLL_PI_State *state, float kp_hz_per_rad,
                      float ki_hz_per_rad_s)
{
    float integral;
    if (state == NULL || !state->initialized || !isfinite(kp_hz_per_rad) ||
        !isfinite(ki_hz_per_rad_s) || kp_hz_per_rad < 0.0f ||
        ki_hz_per_rad_s < 0.0f)
    {
        return false;
    }
    integral = state->integrator_hz;
    if (state->phase_initialized)
    {
        const float error = DPLL_PI_ClampPhaseError(
            state->continuous_phase_rad - state->config.target_phase_rad);
        integral += (state->config.kp_hz_per_rad - kp_hz_per_rad) * error;
        if (!isfinite(integral) ||
            fabsf(integral) > state->config.max_frequency_correction_hz)
        {
            return false;
        }
        state->snapshot.p_term_hz = kp_hz_per_rad * error;
        state->snapshot.i_term_hz = integral;
    }
    state->integrator_hz = integral;
    state->config.kp_hz_per_rad = kp_hz_per_rad;
    state->config.ki_hz_per_rad_s = ki_hz_per_rad_s;
    return true;
}

/**
 * @brief  执行一次 PI 更新：由相位误差/幅度计算频率修正并输出 FCW。
 * @param  state                         PI 状态。
 * @param  raw_phase_rad                 本次原始相位（弧度，来自 IQ 鉴相）。
 * @param  magnitude                     本次信号幅度（ADC 码）。
 * @param  iq_valid                      本次 IQ 解算是否有效。
 * @param  reference_match               参考元数据是否匹配。
 * @param  output_frequency_control_word 输出参数，写入更新后的 FCW。
 * @retval true  产生了一次 FCW 更新；false 各种原因未更新（未使能、信号无效、
 *               参考不匹配、IQ 无效等），此时输出 FCW 保持不变。
 * @note   算法流程：更新信号有效性 → 相位解卷绕 → P = kp*误差；
 *         I 用梯形积分 I += ki*T*(e+e_prev)/2，抗积分饱和后限幅，
 *         最终输出频率 = 标称频率 + 修正，再折算成 FCW。
 */
bool DPLL_PI_Update(DPLL_PI_State *state,
                    float raw_phase_rad,
                    float magnitude,
                    bool iq_valid,
                    bool reference_match,
                    uint64_t *output_frequency_control_word)
{
    float error_rad;
    float p_term_hz;
    float candidate_integrator_hz;
    float candidate_correction_hz;
    float correction_hz;
    double nominal_frequency_hz;
    double output_frequency_hz;
    bool integration_rejected;
    bool saturated;

    if ((state == NULL) || !state->initialized)
    {
        return false;
    }

    state->snapshot.enabled = state->enabled;
    state->snapshot.iq_valid = iq_valid;
    state->snapshot.reference_match = reference_match;
    state->snapshot.update_applied = false;
    state->snapshot.saturated = false;

    /* 参考不匹配：仅累计计数并返回，不更新环路。 */
    if (!reference_match)
    {
        if (state->snapshot.reference_mismatch_count < UINT32_MAX)
        {
            ++state->snapshot.reference_mismatch_count;
        }
        return false;
    }

    /* IQ 无效或输入非有限：直接返回。 */
    if (!iq_valid || !isfinite(raw_phase_rad) || !isfinite(magnitude) ||
        (magnitude < 0.0f))
    {
        return false;
    }

    state->snapshot.raw_phase_rad = raw_phase_rad;
    state->snapshot.magnitude = magnitude;
    DPLL_PI_UpdateSignalState(state, magnitude);

    /* 信号有效（或至少达到 acquire 阈值）才推进相位跟踪。 */
    if (state->signal_valid || (magnitude >= state->config.acquire_magnitude))
    {
        DPLL_PI_TrackPhase(state, raw_phase_rad);
        state->snapshot.continuous_phase_rad = state->continuous_phase_rad;
        state->snapshot.phase_error_rad = DPLL_PI_ClampPhaseError(
            state->continuous_phase_rad - state->config.target_phase_rad);
    }

    state->snapshot.signal_valid = state->signal_valid;
    if (!state->enabled || !state->signal_valid)
    {
        return false;
    }

    /* ---- PI 计算 ---- */
    error_rad = state->snapshot.phase_error_rad;
    p_term_hz = state->config.kp_hz_per_rad * error_rad;
    if (!state->error_initialized)
    {
        state->previous_error_rad = error_rad;
        state->error_initialized = true;
    }

    /* 梯形积分：I += ki * T * (e + e_prev) / 2。 */
    candidate_integrator_hz = state->integrator_hz +
        (0.5f * state->config.ki_hz_per_rad_s * state->config.update_period_s *
         (error_rad + state->previous_error_rad));
    candidate_correction_hz = p_term_hz + candidate_integrator_hz;
    saturated = fabsf(candidate_correction_hz) >
                state->config.max_frequency_correction_hz;

    /* 仅拒绝那些会使已饱和输出进一步加深饱和的积分；
     * 反向退饱和的积分仍然保持有效。 */
    integration_rejected = saturated &&
        (((candidate_correction_hz > 0.0f) && (error_rad > 0.0f)) ||
         ((candidate_correction_hz < 0.0f) && (error_rad < 0.0f)));
    if (!integration_rejected)
    {
        state->integrator_hz = DPLL_PI_Clamp(
            candidate_integrator_hz,
            -state->config.max_frequency_correction_hz,
            state->config.max_frequency_correction_hz);
    }

    correction_hz = integration_rejected ? candidate_correction_hz :
                    (p_term_hz + state->integrator_hz);
    if (correction_hz > state->config.max_frequency_correction_hz)
    {
        correction_hz = state->config.max_frequency_correction_hz;
        saturated = true;
    }
    else if (correction_hz < -state->config.max_frequency_correction_hz)
    {
        correction_hz = -state->config.max_frequency_correction_hz;
        saturated = true;
    }
    else
    {
        saturated = false;
    }

    /* 输出频率 = 标称频率 + 修正，再折算成 FCW。 */
    nominal_frequency_hz = DPLL_PI_ControlWordToFrequency(
        state->config.nominal_frequency_control_word,
        state->config.sample_rate_hz);
    output_frequency_hz = nominal_frequency_hz + (double)correction_hz;

    state->previous_error_rad = error_rad;
    state->snapshot.p_term_hz = p_term_hz;
    state->snapshot.i_term_hz = state->integrator_hz;
    state->snapshot.frequency_correction_hz = correction_hz;
    state->snapshot.output_frequency_hz = output_frequency_hz;
    state->snapshot.output_frequency_control_word =
        DPLL_PI_FrequencyToControlWord(output_frequency_hz,
                                       state->config.sample_rate_hz);
    state->snapshot.saturated = saturated;
    state->snapshot.update_applied = true;

    if (output_frequency_control_word != NULL)
    {
        *output_frequency_control_word =
            state->snapshot.output_frequency_control_word;
    }

    return true;
}

/**
 * @brief  获取 PI 状态快照指针（只读）。
 * @param  state PI 状态。
 * @retval 非 NULL 指向内部快照；NULL 表示状态未初始化。
 * @note   返回指针指向内部结构，调用方只读；运行期有效。
 */
const DPLL_PI_Snapshot *DPLL_PI_GetSnapshot(const DPLL_PI_State *state)
{
    if ((state == NULL) || !state->initialized)
    {
        return NULL;
    }

    return &state->snapshot;
}
