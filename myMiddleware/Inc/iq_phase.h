/**
 * @file    iq_phase.h
 * @brief   IQ 参考生成与跨块最小二乘鉴相；不依赖板级资源。
 *
 * 本头文件定义 IQ 相位检测器的公开接口：用与 NCO 相同的频率控制字（FCW）驱动
 * 相位累加器生成 sin/cos 参考，把输入 ADC 块建模为 DC + I*sin + Q*cos，
 * 通过相关（单块）或最小二乘（跨块累积）解出复包络 I/Q，进而得到幅度与相位。
 *
 * 关键约定：
 *  - 采样率由端口实际值确定，FCW 每个实际 ADC 样本步进；本算法无需固定 Hz 常量。
 *  - 相位/FCW 均以 64 位满量程（2^64）表示一个周期。
 *  - 参考相位与 NCO 同源，保证测得的相位差即 DAC 输出相对 ADC 输入的真实相位差。
 *  - 本模块单实例，工作缓冲区仅在输入回调内使用，不可重入。
 */
#ifndef IQ_PHASE_H
#define IQ_PHASE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 旧接口保留的默认采样率（1 MS/s）；实际值由端口提供，不参与鉴相计算。 */
#define IQ_PHASE_SAMPLE_RATE_HZ 1000000U

/** 单块最大样本数；一个 512 采样的块在 1 MS/s 下提供 512 us 的处理时间预算。 */
#define IQ_PHASE_MAX_BLOCK_SAMPLES 512U

/** 跨块累积块数；默认 1 MHz、512 点/块时窗口 4.096 ms，约覆盖 500 Hz 两周期。
 *  改采样率/块长后窗口随实际速率变化，须重新评估低频观测长度。 */
#define IQ_PHASE_ACCUMULATION_BLOCKS 8U

/**
 * @brief  IQ 相位检测器状态，保存参考相位累加器与递推所需的单步 sin/cos。
 */
typedef struct
{
    uint64_t phase_accumulator;     /**< 参考相位累加器（2^64/周） */
    uint64_t frequency_control_word;/**< 频率控制字（2^64/周），每采样步进 */
    float sin_step;                 /**< 单步相位的 sin(Δ)，用于递推 */
    float cos_step;                 /**< 单步相位的 cos(Δ)，用于递推 */
    bool initialized;               /**< 是否已初始化 */
} IQ_PhaseState;

/**
 * @brief  一次鉴相的结果，包含复包络、幅度、相位与直流分量。
 */
typedef struct
{
    float i;            /**< 同相分量 I（ADC 码） */
    float q;            /**< 正交分量 Q（ADC 码） */
    float magnitude;    /**< 幅度 = sqrt(I² + Q²)（ADC 码） */
    float phase_rad;    /**< 相位（弧度），约定 x[n]=A*sin(theta+phi) -> phase≈phi */
    float phase_deg;    /**< 相位（度） */
    float dc_level;     /**< 直流分量（ADC 码） */
    bool valid;         /**< 结果是否有效（幅度大于门限且各值有限） */
    uint32_t sample_count; /**< 参与解算的样本总数 */
} IQ_PhaseResult;

/**
 * @brief  跨块累积器，累加最小二乘正规方程所需的各阶和。
 */
typedef struct
{
    double sample_sum;         /**< Σx */
    double sine_sum;           /**< Σsin */
    double cosine_sum;         /**< Σcos */
    double sine_squared_sum;   /**< Σsin² */
    double cosine_squared_sum; /**< Σcos² */
    double sine_cosine_sum;    /**< Σ(sin·cos) */
    double sample_sine_sum;    /**< Σ(x·sin) */
    double sample_cosine_sum;  /**< Σ(x·cos) */
    uint32_t sample_count;     /**< 累计样本总数 */
    uint16_t block_count;      /**< 已累计块数 */
    uint16_t target_block_count; /**< 本窗口目标块数，范围 1..IQ_PHASE_ACCUMULATION_BLOCKS。 */
} IQ_PhaseAccumulator;

/**
 * @brief  跨块累积的状态。
 */
typedef enum
{
    IQ_PHASE_ACCUMULATION_ERROR = 0,       /**< 参数非法、已满或解算失败 */
    IQ_PHASE_ACCUMULATION_IN_PROGRESS,     /**< 尚未攒满当前目标块数 */
    IQ_PHASE_ACCUMULATION_READY            /**< 攒满并成功解出结果 */
} IQ_PhaseAccumulationStatus;

/**
 * @brief  初始化 IQ 相位检测器状态。
 * @param  state                  待初始化的状态。
 * @param  initial_phase          初始相位（2^64/周）。
 * @param  frequency_control_word 频率控制字（2^64/周）。
 */
void IQ_Phase_Init(IQ_PhaseState *state,
                   uint64_t initial_phase,
                   uint64_t frequency_control_word);

/**
 * @brief  复位相位累加器到指定相位。
 * @param  state IQ 相位检测器状态。
 * @param  phase 新的相位累加值（2^64/周）。
 */
void IQ_Phase_Reset(IQ_PhaseState *state, uint64_t phase);

/**
 * @brief  设置频率控制字并刷新递推用的单步 sin/cos。
 * @param  state                   IQ 相位检测器状态。
 * @param  frequency_control_word  新的 FCW（2^64/周）。
 */
void IQ_Phase_SetFrequencyControlWord(IQ_PhaseState *state,
                                      uint64_t frequency_control_word);

/**
 * @brief  对单个块做 IQ 鉴相（便捷入口，首步长与后续步长相同）。
 * @param  state                  IQ 相位检测器状态。
 * @param  samples                输入样本（无符号 ADC 码）。
 * @param  sample_count           样本数。
 * @param  frequency_control_word 相位步长（2^64/周）。
 * @param  result                 输出参数，写入 I/Q/幅度/相位/直流。
 * @retval true  成功；false 参数非法或状态未初始化。
 * @note   将一个 ADC 块与 sin/cos 参考做相关，theta 由与 NCO 相同的 FCW 生成。
 *         相位约定：x[n] = A*sin(theta[n]+phi) -> phase_rad ≈ phi。
 */
bool IQ_Phase_ProcessU16(IQ_PhaseState *state,
                         const uint16_t *samples,
                         uint32_t sample_count,
                         uint64_t frequency_control_word,
                         IQ_PhaseResult *result);

/**
 * @brief  对单个块做 IQ 鉴相，支持首步长与后续步长不同。
 * @param  state                      IQ 相位检测器状态。
 * @param  samples                    输入样本。
 * @param  sample_count               样本数。
 * @param  first_frequency_control_word 第一个相位步长（2^64/周）。
 * @param  frequency_control_word     后续相位步长（2^64/周）。
 * @param  result                     输出参数，写入 I/Q/幅度/相位/直流。
 * @retval true  成功；false 参数非法或状态未初始化。
 * @note   第一个步长可单独指定：因为 DAC 在 DMA 装载下一个采样之前就把 DHR
 *         传输到 DOR，首个相位步进可能属于前一个 DAC DMA 缓冲区。
 */
bool IQ_Phase_ProcessU16Reference(IQ_PhaseState *state,
                                  const uint16_t *samples,
                                  uint32_t sample_count,
                                  uint64_t first_frequency_control_word,
                                  uint64_t frequency_control_word,
                                  IQ_PhaseResult *result);

/**
 * @brief  清零跨块累积器，开始一个新的观测窗口。
 * @param  accumulator 待复位的累积器。
 */
void IQ_Phase_AccumulatorReset(IQ_PhaseAccumulator *accumulator);

/**
 * @brief  设置跨块 IQ 观测窗口的块数。
 * @param  accumulator 跨块累积器。
 * @param  block_count 目标块数，范围 1..IQ_PHASE_ACCUMULATION_BLOCKS。
 * @retval true 设置成功；false 参数为空或块数超出编译期最大值。
 * @note   设置会清零当前统计量，下一窗口使用新的块数。
 */
bool IQ_Phase_AccumulatorSetBlockCount(IQ_PhaseAccumulator *accumulator,
                                       uint16_t block_count);

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
 *         IQ_PHASE_ACCUMULATION_READY       攒满并成功解出结果。
 * @note   按设置的目标块数累积连续采样块，用最小二乘法求解 DC + sin + cos；
 *         即使单个块包含的载波周期不足一个，该方法仍保持准确。
 */
IQ_PhaseAccumulationStatus IQ_Phase_AccumulateU16Reference(
    IQ_PhaseState *state,
    IQ_PhaseAccumulator *accumulator,
    const uint16_t *samples,
    uint32_t sample_count,
    uint64_t first_frequency_control_word,
    uint64_t frequency_control_word,
    IQ_PhaseResult *result);

#ifdef __cplusplus
}
#endif

#endif /* IQ_PHASE_H */
