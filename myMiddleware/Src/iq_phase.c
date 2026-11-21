/**
 * @file    iq_phase.c
 * @brief   IQ 参考生成与跨块最小二乘鉴相；不依赖板级资源。
 *
 * 本模块负责从 ADC 输入块中解出"信号在 NCO 频率上的复包络"，即直流分量 DC、
 * 同相分量 I 和正交分量 Q，进而得到幅度与相位。这是 DPLL 的"输入半环"测量端。
 *
 * 工作原理：
 *  - 用与 DAC NCO 相同的频率控制字（FCW）驱动一个相位累加器，生成 sin/cos 参考；
 *    输入信号被建模为 x[n] = DC + I*sin(theta[n]) + Q*cos(theta[n])。
 *  - 单块处理用去均值后直接相关（I/Q = 2*Σ(x·sin)/N、2*Σ(x·cos)/N）；
 *  - 跨块累积（默认 8 块 = 4096 点，运行时可缩短）用最小二乘同时解出 DC、I、Q，即便单个块不足
 *    整周期也能保证精度（见 IQ_Phase_AccumulateU16Reference）。
 *
 * 关键设计点：
 *  - 参考相位与 NCO 相位同源（同一 FCW、同一 64 位累加），保证测得的相位差
 *    就是 DAC 输出相对 ADC 输入的真实相位差。
 *  - sin/cos 递推采用旋转矩阵，每 128 点用 IQ_Phase_GetSinCos 精确重同步一次，
 *    抑制浮点递推的累积误差。
 *  - 正弦参考支持 CMSIS DSP（arm_sin_q31，Q31 定点）与软件（sinf/cosf）双路径。
 *  - 在生成参考的同一循环中累加两个相关和，避免保存整块参考数组。
 */
#include "iq_phase.h"

#include <math.h>
#include <stddef.h>

#if defined(IQ_PHASE_USE_CMSIS_DSP)
#include "arm_math.h"
#endif

/** 2π 的 float32 常量，用于把 64 位相位换算成弧度。 */
#define IQ_TWO_PI_F      6.28318530717958647692f
/** 2^64 的 double 常量，64 位相位累加器的满量程。 */
#define IQ_TWO_POW_64_D 18446744073709551616.0
/** 弧度转角度系数（180/π）。 */
#define IQ_RAD_TO_DEG_F 57.2957795130823208768f
/** Q31 定点转 float 的缩放系数（2^-31）。 */
#define IQ_Q31_TO_FLOAT 4.656612873077392578125e-10f
/** 最小二乘行列式下限，低于此值视为参考退化解不出 I/Q。 */
#define IQ_LEAST_SQUARES_MIN_DETERMINANT 1.0e-6

/**
 * @brief  单块的统计量集合，用于最小二乘正规方程的系数累加。
 */
typedef struct
{
    double sample_sum;          /**< 样本和 Σx */
    double sine_sum;            /**< sin 参考和 Σsin */
    double cosine_sum;          /**< cos 参考和 Σcos */
    double sine_squared_sum;    /**< sin² 和 Σsin² */
    double cosine_squared_sum;  /**< cos² 和 Σcos² */
    double sine_cosine_sum;     /**< sin·cos 和 Σ(sin·cos) */
    float centered_sine_sum;    /**< 去均值输入与正弦参考的点积。 */
    float centered_cosine_sum;  /**< 去均值输入与余弦参考的点积。 */
    float mean;                 /**< 本块样本均值（直流估计） */
} IQ_PhaseBlockStatistics;

/**
 * @brief  根据当前 FCW 计算单步相位的 sin/cos（即旋转矩阵的 cosΔ 与 sinΔ）。
 * @param  state IQ 相位检测器状态。
 * @note   结果存于 state->sin_step / state->cos_step，供 IQ_Phase_PrepareBlock
 *         做 sin/cos 递推；FCW 变化时须调用本函数刷新。
 */
static void IQ_Phase_UpdateStep(IQ_PhaseState *state)
{
#if defined(IQ_PHASE_USE_CMSIS_DSP)
    /* 取 FCW 高 31 位作为 Q31 相位步长，用定点正弦计算 sinΔ/cosΔ。 */
    uint32_t phase_q31 = (uint32_t)(state->frequency_control_word >> 33U);
    state->sin_step = (float)arm_sin_q31((q31_t)phase_q31) * IQ_Q31_TO_FLOAT;
    state->cos_step = (float)arm_sin_q31(
        (q31_t)((phase_q31 + UINT32_C(0x20000000)) &
                UINT32_C(0x7fffffff))) * IQ_Q31_TO_FLOAT;
#else
    /* 把 FCW 换算成弧度步长 Δ = FCW/2^64 * 2π，用 sinf/cosf 直接计算。 */
    float step_rad = (float)(((double)state->frequency_control_word /
                              IQ_TWO_POW_64_D) * (double)IQ_TWO_PI_F);
    state->sin_step = sinf(step_rad);
    state->cos_step = cosf(step_rad);
#endif
}

/**
 * @brief  由 64 位相位精确计算 sin/cos（用于递推的重同步点）。
 * @param  phase      相位，单位 2^64/周。
 * @param  sin_phase  输出参数，写入 sin(phase)。
 * @param  cos_phase  输出参数，写入 cos(phase)。
 */
static void IQ_Phase_GetSinCos(uint64_t phase,
                               float *sin_phase,
                               float *cos_phase)
{
#if defined(IQ_PHASE_USE_CMSIS_DSP)
    /* 取相位高 31 位为 Q31 相位；cos 用 sin(phase + π/2) 得到（Q31 加 0x20000000）。 */
    uint32_t phase_q31 = (uint32_t)(phase >> 33U);
    *sin_phase = (float)arm_sin_q31((q31_t)phase_q31) * IQ_Q31_TO_FLOAT;
    *cos_phase = (float)arm_sin_q31(
        (q31_t)((phase_q31 + UINT32_C(0x20000000)) &
                UINT32_C(0x7fffffff))) * IQ_Q31_TO_FLOAT;
#else
    float phase_rad = (float)(((double)phase / IQ_TWO_POW_64_D) *
                              (double)IQ_TWO_PI_F);
    *sin_phase = sinf(phase_rad);
    *cos_phase = cosf(phase_rad);
#endif
}

/**
 * @brief  准备一个块：去均值、生成 sin/cos 参考并累加最小二乘各阶统计量。
 * @param  state                      IQ 相位检测器状态（含相位累加器）。
 * @param  samples                    输入样本（无符号 ADC 码）。
 * @param  sample_count               样本数。
 * @param  first_frequency_control_word 第一个相位步长（2^64/周）。
 * @param  frequency_control_word     后续相位步长（2^64/周）。
 * @param  statistics                 输出参数，写入本块的统计量集合。
 * @retval true  准备成功；false 参数非法或状态未初始化。
 * @note   本函数更新 state->phase_accumulator；参考相位与 NCO 同源，保证相位差
 *         测量正确。第一个步长可单独指定，用于补偿首样本可能来自前一 DAC 块。
 */
static bool IQ_Phase_PrepareBlock(IQ_PhaseState *state,
                                  const uint16_t *samples,
                                  uint32_t sample_count,
                                  uint64_t first_frequency_control_word,
                                  uint64_t frequency_control_word,
                                  IQ_PhaseBlockStatistics *statistics)
{
    uint64_t phase;
    uint32_t sample_sum = 0U; /* 512 个 16 位 ADC 码的和可用 32 位保存。 */
    float centered_sine_sum = 0.0f, centered_cosine_sum = 0.0f; /* 两个逐点相关和。 */
    float sin_phase;
    float cos_phase;
    float sine_sum = 0.0f;
    float cosine_sum = 0.0f;
    float sine_squared_sum = 0.0f;
    float cosine_squared_sum = 0.0f;
    float sine_cosine_sum = 0.0f;

    if ((state == NULL) || (samples == NULL) || (statistics == NULL) ||
        (sample_count == 0U) ||
        (sample_count > IQ_PHASE_MAX_BLOCK_SAMPLES) ||
        (!state->initialized))
    {
        return false;
    }

    /* FCW 变化时刷新递推用的单步 sin/cos。 */
    if (state->frequency_control_word != frequency_control_word)
    {
        IQ_Phase_SetFrequencyControlWord(state, frequency_control_word);
    }

    /* 先求和得到本块均值（直流估计）。 */
    for (uint32_t index = 0U; index < sample_count; ++index)
    {
        sample_sum += samples[index];
    }
    statistics->mean = (float)sample_sum / (float)sample_count;

    phase = state->phase_accumulator;
    IQ_Phase_GetSinCos(phase, &sin_phase, &cos_phase);

    for (uint32_t index = 0U; index < sample_count; ++index)
    {
        /* 参考值已在寄存器内，直接累加相关和，省去三个数组及后续两遍点积。 */
        const float centered = (float)samples[index] - statistics->mean;
        centered_sine_sum += centered * sin_phase;
        centered_cosine_sum += centered * cos_phase;

        /* 累加最小二乘正规方程所需的各阶和。 */
        sine_sum += sin_phase;
        cosine_sum += cos_phase;
        sine_squared_sum += sin_phase * sin_phase;
        cosine_squared_sum += cos_phase * cos_phase;
        sine_cosine_sum += sin_phase * cos_phase;

        if (index == 0U)
        {
            /* 首样本用 first_frequency_control_word 步进，并精确重算 sin/cos。 */
            phase += first_frequency_control_word;
            IQ_Phase_GetSinCos(phase, &sin_phase, &cos_phase);
        }
        else
        {
            /* 后续样本用旋转矩阵递推，避免每点调用三角函数。 */
            phase += frequency_control_word;
            const float old_sin_phase = sin_phase;
            const float old_cos_phase = cos_phase;

            sin_phase = old_sin_phase * state->cos_step +
                        old_cos_phase * state->sin_step;
            cos_phase = old_cos_phase * state->cos_step -
                        old_sin_phase * state->sin_step;
        }

        /* 每 128 点用 IQ_Phase_GetSinCos 精确重同步一次，抑制浮点递推漂移。 */
        if (((index & 0x7FU) == 0x7FU) && ((index + 1U) < sample_count))
        {
            IQ_Phase_GetSinCos(phase, &sin_phase, &cos_phase);
        }
    }

    state->phase_accumulator = phase;
    statistics->centered_sine_sum = centered_sine_sum;
    statistics->centered_cosine_sum = centered_cosine_sum;
    statistics->sample_sum = (double)sample_sum;
    statistics->sine_sum = (double)sine_sum;
    statistics->cosine_sum = (double)cosine_sum;
    statistics->sine_squared_sum = (double)sine_squared_sum;
    statistics->cosine_squared_sum = (double)cosine_squared_sum;
    statistics->sine_cosine_sum = (double)sine_cosine_sum;
    return true;
}

/**
 * @brief  初始化 IQ 相位检测器状态。
 * @param  state                  待初始化的状态。
 * @param  initial_phase          初始相位（2^64/周）。
 * @param  frequency_control_word 频率控制字（2^64/周）。
 * @note   初始化后置 initialized，并按 FCW 预计算单步 sin/cos。
 */
void IQ_Phase_Init(IQ_PhaseState *state,
                   uint64_t initial_phase,
                   uint64_t frequency_control_word)
{
    if (state == NULL)
    {
        return;
    }

    state->phase_accumulator = initial_phase;
    state->frequency_control_word = frequency_control_word;
    state->initialized = true;
    IQ_Phase_UpdateStep(state);
}

/**
 * @brief  复位相位累加器到指定相位。
 * @param  state IQ 相位检测器状态。
 * @param  phase 新的相位累加值（2^64/周）。
 * @note   通常用于把参考相位对齐到某个 DAC 块的起始相位。
 */
void IQ_Phase_Reset(IQ_PhaseState *state, uint64_t phase)
{
    if (state != NULL)
    {
        state->phase_accumulator = phase;
    }
}

/**
 * @brief  设置频率控制字并刷新递推用的单步 sin/cos。
 * @param  state                   IQ 相位检测器状态。
 * @param  frequency_control_word  新的 FCW（2^64/周）。
 */
void IQ_Phase_SetFrequencyControlWord(IQ_PhaseState *state,
                                      uint64_t frequency_control_word)
{
    if (state == NULL)
    {
        return;
    }

    state->frequency_control_word = frequency_control_word;
    IQ_Phase_UpdateStep(state);
}

/**
 * @brief  对单个块做 IQ 鉴相（便捷入口，首步长与后续步长相同）。
 * @param  state                  IQ 相位检测器状态。
 * @param  samples                输入样本。
 * @param  sample_count           样本数。
 * @param  frequency_control_word 相位步长（2^64/周）。
 * @param  result                 输出参数，写入 I/Q/幅度/相位/直流。
 * @retval true  成功；false 参数非法或状态未初始化。
 * @note   等价于 IQ_Phase_ProcessU16Reference 且首步长等于后续步长。
 */
bool IQ_Phase_ProcessU16(IQ_PhaseState *state,
                         const uint16_t *samples,
                         uint32_t sample_count,
                         uint64_t frequency_control_word,
                         IQ_PhaseResult *result)
{
    return IQ_Phase_ProcessU16Reference(state, samples, sample_count,
                                        frequency_control_word,
                                        frequency_control_word, result);
}

/**
 * @brief  对单个块做 IQ 鉴相，支持首步长与后续步长不同。
 * @param  state                      IQ 相位检测器状态。
 * @param  samples                    输入样本。
 * @param  sample_count               样本数。
 * @param  first_frequency_control_word 第一个相位步长（2^64/周）。
 * @param  frequency_control_word     后续相位步长（2^64/周）。
 * @param  result                     输出参数，写入 I/Q/幅度/相位/直流。
 * @retval true  成功；false 参数非法或状态未初始化。
 * @note   单块算法：去均值后用直接相关得到 I = 2*Σ(x·sin)/N、
 *         Q = 2*Σ(x·cos)/N，相位约定 x[n] = A*sin(theta+phi) -> phase ≈ phi。
 */
bool IQ_Phase_ProcessU16Reference(IQ_PhaseState *state,
                                  const uint16_t *samples,
                                  uint32_t sample_count,
                                  uint64_t first_frequency_control_word,
                                  uint64_t frequency_control_word,
                                  IQ_PhaseResult *result)
{
    IQ_PhaseBlockStatistics statistics;

    if ((result == NULL) ||
        !IQ_Phase_PrepareBlock(state, samples, sample_count,
                               first_frequency_control_word,
                               frequency_control_word, &statistics))
    {
        return false;
    }

    /* 归一化得到复包络 I/Q（因子 2 补偿 sin² 的平均值 1/2）。 */
    result->i = (2.0f * statistics.centered_sine_sum) / (float)sample_count;
    result->q = (2.0f * statistics.centered_cosine_sum) / (float)sample_count;
    result->magnitude = sqrtf((result->i * result->i) + (result->q * result->q));
    result->phase_rad = atan2f(result->q, result->i);
    result->phase_deg = result->phase_rad * IQ_RAD_TO_DEG_F;
    result->dc_level = statistics.mean;
    result->valid = (result->magnitude > 1.0e-6f);
    result->sample_count = sample_count;

    return true;
}

/**
 * @brief  清零跨块累积器，开始一个新的观测窗口。
 * @param  accumulator 待复位的累积器。
 */
void IQ_Phase_AccumulatorReset(IQ_PhaseAccumulator *accumulator)
{
    if (accumulator != NULL)
    {
        *accumulator = (IQ_PhaseAccumulator){
            .target_block_count = IQ_PHASE_ACCUMULATION_BLOCKS};
    }
}

/**
 * @brief  设置跨块 IQ 观测窗口的块数。
 * @param  accumulator 跨块累积器。
 * @param  block_count 目标块数，范围 1..IQ_PHASE_ACCUMULATION_BLOCKS。
 * @retval true 设置成功；false 参数为空或块数超出编译期最大值。
 * @note   设置会清零当前统计量，避免不同窗口长度的数据混合。
 */
bool IQ_Phase_AccumulatorSetBlockCount(IQ_PhaseAccumulator *accumulator,
                                       uint16_t block_count)
{
    if (accumulator == NULL || block_count == 0U ||
        block_count > IQ_PHASE_ACCUMULATION_BLOCKS)
    {
        return false;
    }
    IQ_Phase_AccumulatorReset(accumulator);
    accumulator->target_block_count = block_count;
    return true;
}

/**
 * @brief  累积一个块到跨块最小二乘累积器，攒满后解出 I/Q/相位/直流。
 * @param  state                      IQ 相位检测器状态。
 * @param  accumulator                跨块累积器。
 * @param  samples                    输入样本。
 * @param  sample_count               样本数。
 * @param  first_frequency_control_word 第一个相位步长（2^64/周）。
 * @param  frequency_control_word     后续相位步长（2^64/周）。
 * @param  result                     输出参数，攒满后写入解算结果。
 * @retval IQ_PHASE_ACCUMULATION_ERROR       参数非法、已满或解算失败；
 *         IQ_PHASE_ACCUMULATION_IN_PROGRESS 尚未攒满当前目标块数；
 *         IQ_PHASE_ACCUMULATION_READY       攒满当前目标块数并成功解出结果。
 * @note   算法：对 DC + I*sin + Q*cos 模型做最小二乘，通过归一化正规方程
 *         解 2x2 线性方程组得到 I、Q，再反解直流。即使单个块不足整周期也保持
 *         准确；窗口块数由 IQ_Phase_AccumulatorSetBlockCount 设置，默认 8 块。
 */
IQ_PhaseAccumulationStatus IQ_Phase_AccumulateU16Reference(
    IQ_PhaseState *state,
    IQ_PhaseAccumulator *accumulator,
    const uint16_t *samples,
    uint32_t sample_count,
    uint64_t first_frequency_control_word,
    uint64_t frequency_control_word,
    IQ_PhaseResult *result)
{
    IQ_PhaseBlockStatistics statistics;

    if ((accumulator == NULL) || (result == NULL) ||
        (accumulator->target_block_count == 0U) ||
        (accumulator->target_block_count > IQ_PHASE_ACCUMULATION_BLOCKS) ||
        (accumulator->block_count >= accumulator->target_block_count) ||
        !IQ_Phase_PrepareBlock(state, samples, sample_count,
                               first_frequency_control_word,
                               frequency_control_word, &statistics))
    {
        return IQ_PHASE_ACCUMULATION_ERROR;
    }

    /* 累加最小二乘正规方程的各阶和（跨块）。 */
    accumulator->sample_sum += statistics.sample_sum;
    accumulator->sine_sum += statistics.sine_sum;
    accumulator->cosine_sum += statistics.cosine_sum;
    accumulator->sine_squared_sum += statistics.sine_squared_sum;
    accumulator->cosine_squared_sum += statistics.cosine_squared_sum;
    accumulator->sine_cosine_sum += statistics.sine_cosine_sum;
    /* 用"去均值相关 + 均值×参考和"还原 Σ(x·sin) 与 Σ(x·cos)。 */
    accumulator->sample_sine_sum += (double)statistics.centered_sine_sum +
        ((double)statistics.mean * statistics.sine_sum);
    accumulator->sample_cosine_sum += (double)statistics.centered_cosine_sum +
        ((double)statistics.mean * statistics.cosine_sum);
    accumulator->sample_count += sample_count;
    accumulator->block_count++;

    if (accumulator->block_count < accumulator->target_block_count)
    {
        return IQ_PHASE_ACCUMULATION_IN_PROGRESS;
    }

    {
        const double count = (double)accumulator->sample_count;
        /* 各统计量减去均值项，得到中心化后的正规方程系数。 */
        const double sine_normal = accumulator->sine_squared_sum -
            ((accumulator->sine_sum * accumulator->sine_sum) / count);
        const double cosine_normal = accumulator->cosine_squared_sum -
            ((accumulator->cosine_sum * accumulator->cosine_sum) / count);
        const double cross_normal = accumulator->sine_cosine_sum -
            ((accumulator->sine_sum * accumulator->cosine_sum) / count);
        const double sample_sine_normal = accumulator->sample_sine_sum -
            ((accumulator->sample_sum * accumulator->sine_sum) / count);
        const double sample_cosine_normal = accumulator->sample_cosine_sum -
            ((accumulator->sample_sum * accumulator->cosine_sum) / count);
        const double determinant = (sine_normal * cosine_normal) -
                                   (cross_normal * cross_normal);
        double i_component;
        double q_component;

        /* 行列式过小或非有限，说明参考退化（如幅度为零），无法解出 I/Q。 */
        if ((!isfinite(determinant)) ||
            (determinant <= IQ_LEAST_SQUARES_MIN_DETERMINANT))
        {
            return IQ_PHASE_ACCUMULATION_ERROR;
        }

        /* 用克拉默法则解 2x2 线性方程组得到 I、Q。 */
        i_component = ((sample_sine_normal * cosine_normal) -
                       (sample_cosine_normal * cross_normal)) / determinant;
        q_component = ((sample_cosine_normal * sine_normal) -
                       (sample_sine_normal * cross_normal)) / determinant;

        result->i = (float)i_component;
        result->q = (float)q_component;
        result->magnitude = hypotf(result->i, result->q);
        result->phase_rad = atan2f(result->q, result->i);
        result->phase_deg = result->phase_rad * IQ_RAD_TO_DEG_F;
        /* 由 DC + I*sin + Q*cos 模型反解直流分量。 */
        result->dc_level = (float)((accumulator->sample_sum -
            (i_component * accumulator->sine_sum) -
            (q_component * accumulator->cosine_sum)) / count);
        result->valid = isfinite(result->magnitude) &&
                        isfinite(result->phase_rad) &&
                        (result->magnitude > 1.0e-6f);
        result->sample_count = accumulator->sample_count;
    }

    return result->valid ? IQ_PHASE_ACCUMULATION_READY :
                           IQ_PHASE_ACCUMULATION_ERROR;
}
