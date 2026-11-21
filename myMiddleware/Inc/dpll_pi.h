/**
 * @file    dpll_pi.h
 * @brief   PI 环路滤波、信号有效性与锁相状态；不依赖板级资源。
 *
 * 本头文件定义 DPLL 数字环路滤波器（PI 控制器）的公开接口：把 IQ 鉴相得到的
 * 相位误差转换成频率修正量（Hz），再折算成 NCO 频率控制字（FCW）回写，完成闭环
 * 锁定。同时管理信号有效性（带滞回的 acquire/release 判定）与锁相状态观测。
 *
 * 关键约定：
 *  - 相位/FCW 均以 64 位满量程（2^64）表示一个周期。
 *  - kp 单位为 Hz/rad，ki 单位为 Hz/rad/s，修正量叠加到标称频率上。
 *  - 输出频率 = 标称频率 + 修正，修正量被限幅在 ±max_frequency_correction_hz。
 */
#ifndef DPLL_PI_H
#define DPLL_PI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 默认 PI 更新周期（秒）：8 块 × 512 采样 ÷ 1 MHz = 4.096 ms。 */
#define DPLL_PI_DEFAULT_UPDATE_PERIOD_S       0.004096f
/** 默认比例增益 kp，单位 Hz/rad。 */
#define DPLL_PI_DEFAULT_KP_HZ_PER_RAD          14.1421356f
/** 默认积分增益 ki，单位 Hz/rad/s。 */
#define DPLL_PI_DEFAULT_KI_HZ_PER_RAD_S        628.318531f
/** 默认最大频率修正量，单位 Hz（环路的捕获/牵引范围上限）。 */
#define DPLL_PI_DEFAULT_MAX_CORRECTION_HZ      1000.0f
/** 默认信号捕获幅度阈值（ADC 码），高于此值开始计入 acquire 计数。 */
#define DPLL_PI_DEFAULT_ACQUIRE_MAGNITUDE      100.0f
/** 默认信号释放幅度阈值（ADC 码），低于此值开始计入 release 计数。 */
#define DPLL_PI_DEFAULT_RELEASE_MAGNITUDE      80.0f
/** 默认连续高于 acquire 阈值的块数，攒满即判信号有效。 */
#define DPLL_PI_DEFAULT_ACQUIRE_BLOCKS         4U
/** 默认连续低于 release 阈值的块数，攒满即判信号失锁。 */
#define DPLL_PI_DEFAULT_RELEASE_BLOCKS         2U

/**
 * @brief  PI 控制器配置，所有参数在初始化后视为只读。
 */
typedef struct
{
    uint64_t nominal_frequency_control_word; /**< 标称频率控制字（2^64/周） */
    double sample_rate_hz;                   /**< 采样率（Hz） */
    float update_period_s;                   /**< PI 更新周期（秒） */
    float target_phase_rad;                  /**< 目标相位（弧度，折叠到 (-π,π]） */
    float kp_hz_per_rad;                     /**< 比例增益（Hz/rad） */
    float ki_hz_per_rad_s;                   /**< 积分增益（Hz/rad/s） */
    float max_frequency_correction_hz;       /**< 频率修正限幅（Hz） */
    float acquire_magnitude;                 /**< 信号捕获幅度阈值（ADC 码） */
    float release_magnitude;                 /**< 信号释放幅度阈值（ADC 码） */
    uint16_t acquire_blocks;                 /**< 判有效所需连续块数 */
    uint16_t release_blocks;                 /**< 判失锁所需连续块数 */
} DPLL_PI_Config;

/**
 * @brief  PI 控制器快照，供调试/观测（由 DPLL_PI_GetSnapshot 只读返回）。
 */
typedef struct
{
    float raw_phase_rad;                 /**< 本次原始（折叠）相位 */
    float continuous_phase_rad;          /**< 连续（解卷绕后）相位 */
    float phase_error_rad;               /**< 相位误差（限幅到 ±π） */
    float magnitude;                     /**< 本次信号幅度（ADC 码） */
    float p_term_hz;                     /**< P 项输出（Hz） */
    float i_term_hz;                     /**< I 项输出（Hz） */
    float frequency_correction_hz;       /**< 本次频率修正量（Hz） */
    double output_frequency_hz;          /**< 输出频率（标称 + 修正，Hz） */
    uint64_t output_frequency_control_word; /**< 输出频率控制字（2^64/周） */
    uint32_t cycle_slip_count;           /**< 整周滑移累计次数 */
    uint32_t reference_mismatch_count;   /**< 参考元数据不匹配累计次数 */
    bool enabled;                        /**< 环是否使能 */
    bool signal_valid;                   /**< 信号是否有效（已锁定） */
    bool iq_valid;                       /**< 本次 IQ 解算是否有效 */
    bool reference_match;                /**< 本次参考元数据是否匹配 */
    bool saturated;                      /**< 本次修正是否饱和 */
    bool update_applied;                 /**< 本次是否产生了 FCW 更新 */
} DPLL_PI_Snapshot;

/**
 * @brief  PI 控制器完整状态（配置 + 快照 + 运行时内部量）。
 */
typedef struct
{
    DPLL_PI_Config config;               /**< 配置副本 */
    DPLL_PI_Snapshot snapshot;           /**< 最近一次快照 */
    float previous_raw_phase_rad;        /**< 上一原始相位（用于解卷绕） */
    float continuous_phase_rad;          /**< 连续相位（累加） */
    float previous_error_rad;            /**< 上一相位误差（用于梯形积分） */
    float integrator_hz;                 /**< 积分器累加值（Hz） */
    uint16_t acquire_count;              /**< 当前连续满足 acquire 的块数 */
    uint16_t release_count;              /**< 当前连续满足 release 的块数 */
    bool initialized;                    /**< 是否已初始化 */
    bool enabled;                        /**< 环是否使能 */
    bool phase_initialized;              /**< 相位跟踪是否已启动 */
    bool error_initialized;              /**< 误差是否已初始化（用于积分） */
    bool signal_valid;                   /**< 信号是否有效（已锁定） */
} DPLL_PI_State;

/**
 * @brief  用默认参数填充 PI 配置。
 * @param  config                        输出参数，写入默认配置。
 * @param  nominal_frequency_control_word 标称频率控制字（2^64/周）。
 * @param  sample_rate_hz                 采样率（Hz）。
 */
void DPLL_PI_GetDefaultConfig(DPLL_PI_Config *config,
                              uint64_t nominal_frequency_control_word,
                              double sample_rate_hz);

/**
 * @brief  初始化 PI 状态。
 * @param  state  待初始化的状态。
 * @param  config 配置，须通过内部校验。
 * @retval true  初始化成功；false state 为空或配置非法。
 */
bool DPLL_PI_Init(DPLL_PI_State *state, const DPLL_PI_Config *config);

/**
 * @brief  复位 PI 运行时状态（保留 config 与 initialized）。
 * @param  state 待复位的状态。
 */
void DPLL_PI_Reset(DPLL_PI_State *state);

/**
 * @brief  使能或禁用 PI 环。
 * @param  state  PI 状态。
 * @param  enable true 使能（首次使能时复位运行时状态），false 禁用。
 */
void DPLL_PI_Enable(DPLL_PI_State *state, bool enable);

/**
 * @brief  设置目标相位并做无扰更新（bumpless transfer）。
 * @param  state           PI 状态。
 * @param  target_phase_rad 新的目标相位（弧度）。
 * @retval true  设置成功；false 状态非法或目标相位非有限。
 * @note   在不复位环路的条件下改变相位目标；积分项会被调整，使运行时的校准更新
 *         不会造成 NCO 频率阶跃。
 */
bool DPLL_PI_SetTargetPhase(DPLL_PI_State *state, float target_phase_rad);
/* 更新增益并补偿 P 项变化；补偿超出积分限幅时拒绝，保持原状态。
 * 调用方负责与 Update 互斥；kp、ki 均为有限非负数。 */
bool DPLL_PI_SetGains(DPLL_PI_State *state, float kp_hz_per_rad, float ki_hz_per_rad_s);

/**
 * @brief  执行一次 PI 更新：由相位误差/幅度计算频率修正并输出 FCW。
 * @param  state                         PI 状态。
 * @param  raw_phase_rad                 本次原始相位（弧度）。
 * @param  magnitude                     本次信号幅度（ADC 码）。
 * @param  iq_valid                      本次 IQ 解算是否有效。
 * @param  reference_match               参考元数据是否匹配。
 * @param  output_frequency_control_word 输出参数，写入更新后的 FCW。
 * @retval true  产生了一次 FCW 更新；false 未更新（未使能、信号无效、参考不匹配、
 *               IQ 无效等），此时应保持 NCO FCW 不变。
 */
bool DPLL_PI_Update(DPLL_PI_State *state,
                    float raw_phase_rad,
                    float magnitude,
                    bool iq_valid,
                    bool reference_match,
                    uint64_t *output_frequency_control_word);

/**
 * @brief  获取 PI 状态快照指针（只读）。
 * @param  state PI 状态。
 * @retval 非 NULL 指向内部快照；NULL 表示状态未初始化。
 */
const DPLL_PI_Snapshot *DPLL_PI_GetSnapshot(const DPLL_PI_State *state);

#ifdef __cplusplus
}
#endif

#endif /* DPLL_PI_H */
