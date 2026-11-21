/* App 层：锁相业务编排；外设事件通过初始化时注册的接口回调进入。 */
#ifndef DPLL_APP_H
#define DPLL_APP_H

#include "dpll_phase_calibration.h"
#include "dpll_pi.h"
#include "iq_phase.h"
#include "signal_port.h"
#include "nco_waveform.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    const SignalPort *port; /* 借用平台端口，须在整个运行期间有效 */
    double initial_frequency_hz; /* 初始基频，Hz，范围 (0, 实际 ADC 采样率/2)。 */
    uint16_t dac_amplitude; /* 峰值幅度，12 位 DAC 码；不得导致削顶 */
    uint16_t dac_dc_offset; /* 直流偏置，12 位 DAC 码，0..4095 */
    float pi_kp_hz_per_rad; /* 标称比例增益，Hz/rad；短 IQ 窗口会按窗口比例调度。 */
    float pi_ki_hz_per_rad_s; /* 标称积分增益，Hz/(rad*s)；短窗口按比例平方调度。 */
    float pi_max_correction_hz; /* 修正限幅，Hz；0 使用默认值。 */
    SignalOutputReadyCallback output_ready; /* 可选内部输出块订阅者。 */
    void *output_context; /* 订阅者上下文，运行期间保持有效。 */
} DPLL_App_Config;

/**
 * @brief  主循环运行时控制对象，集中保存采样配置、PI 参数和处理结果。
 * @note   请求字段由主循环或调试器修改后置位对应标志；App 处理完成后自动清除标志，
 *         结果字段保留最近一次处理状态。
 */
typedef struct
{
    SignalStreamConfig signal_config; /**< 待应用的 ADC/DAC 共同定时配置。 */
    SignalStreamInfo signal_info; /**< 最近一次实际采样配置及误差。 */
    volatile bool apply_signal_config; /**< 请求停流重配并重新锁相。 */
    volatile bool signal_config_success; /**< 最近一次采样配置结果。 */
    float pi_kp_hz_per_rad; /**< 待应用的标称比例增益，Hz/rad。 */
    float pi_ki_hz_per_rad_s; /**< 待应用的标称积分增益，Hz/(rad*s)。 */
    volatile bool apply_pi_gains; /**< 请求更新 PI 增益，不停止采样。 */
    volatile bool pi_gains_success; /**< 最近一次 PI 增益更新结果。 */
} DPLL_App_RuntimeControl;

typedef DPLL_PhaseCalibration DPLL_App_PhaseCalibration;

/* 单实例应用；Init 复制配置但借用 port。成功返回 true，重复初始化返回 false。
 * 初始化和启动在主循环调用；启动顺序为装载输出、准备输入、启动公共触发。
 * 参数设置可在主循环与输入回调之间调用，端口临界区必须支持嵌套。 */
bool DPLL_App_Init(const DPLL_App_Config *config);
bool DPLL_App_Start(void);
bool DPLL_App_Stop(void);
bool DPLL_App_ConfigureSignal(const SignalStreamConfig *config);
bool DPLL_App_SetPIGains(float kp_hz_per_rad, float ki_hz_per_rad_s);
bool DPLL_App_GetPIGains(float *kp_hz_per_rad, float *ki_hz_per_rad_s);
bool DPLL_App_GetOutputBuffer(uint32_t index, SignalOutputBlock *block);
bool DPLL_App_GetSignalInfo(SignalStreamInfo *info);
/**
 * @brief  处理运行时采样配置和 PI 增益请求。
 * @param  control 运行时控制对象，调用期间必须保持有效。
 * @retval 无。
 * @note   仅由主循环调用；采样配置请求可能停流、重配定时器并重新锁相，
 *         PI 增益请求在短临界区内完成且不停止采样。
 */
void DPLL_App_ProcessRuntimeControl(DPLL_App_RuntimeControl *control);

/* 使能 PI 对 NCO FCW 的接管。禁用时保留最后一个 FCW，
 * 并将控制权交还给现有的手动 NCO API。 */
bool DPLL_App_Enable(bool enable);
bool DPLL_App_IsEnabled(void);

/* 示波器相位校准模型：
 *   measured_phase_deg = intercept_deg +
 *                        slope_deg_per_khz * frequency_khz
 * 假定正的实测相位表示 DAC 滞后于 ADC，因此 PI 目标使用拟合相位的相反数。
 * 当前测量结果的示例：
 *   DPLL_App_SetPhaseCalibration(2.566667f, 0.15824242f);
 * 将两个系数都设为 0.0f 可禁用补偿。如果示波器的符号约定相反，
 * 则将两个系数同时取反。若要校正分别在 f1/f2 (kHz) 处测得的残差
 * r1/r2（单位：度，DAC 滞后为正），使用：
 *   delta_slope = (r2 - r1) / (f2 - f1)
 *   delta_intercept = r1 - delta_slope * f1
 * 然后将两个增量都加到当前系数上。 */
bool DPLL_App_SetPhaseCalibration(float intercept_deg,
                                  float slope_deg_per_khz);
DPLL_App_PhaseCalibration DPLL_App_GetPhaseCalibration(void);

/* 设置 DAC 输出相对 ADC 输入的最终锁定相位（单位：度）。
 * 正值表示 DAC 超前 ADC；负值表示 DAC 滞后 ADC。与频率相关的相位校准
 * 在所有合法频率下自动叠加到此处请求的相位。
 * 相位角按 360 度取模等价。运行时修改使用 PI 的无扰目标更新。 */
bool DPLL_App_SetLockedPhaseDifferenceDegrees(
    float output_minus_input_phase_deg);
float DPLL_App_GetLockedPhaseDifferenceDegrees(void);

/* 选择预计算的正弦、方波或三角波 DAC 表，而不改变 NCO 相位或频率。
 * IQ 相位检测始终保留其正弦参考。 */
bool DPLL_App_SetOutputWaveform(NCO_OutputWaveform waveform);
NCO_OutputWaveform DPLL_App_GetOutputWaveform(void);

/* 设置一个仅作用于 DAC 的整数倍频系数。IQ 检测和 PI 环仍然跟踪基频。
 * 实际的输出上限为 1 MHz：
 *   maximum = floor(1 MHz / 当前基频)
 * 在运行时修改倍频系数前应先查询当前最大值。该设置在稍后安全的
 * DMA 重填时生效，并可能引起相位跳变。 */
bool DPLL_App_SetOutputFrequencyMultiplier(uint32_t multiplier);
uint32_t DPLL_App_GetOutputFrequencyMultiplier(void);
uint32_t DPLL_App_GetMaximumOutputFrequencyMultiplier(void);
double DPLL_App_GetOutputFrequencyHz(void);

/* 在 main.c 重构后保留的稳定调试器符号。 */
extern volatile IQ_PhaseResult g_iq_phase_result;
extern volatile DPLL_PI_Snapshot g_dpll_pi_snapshot;
extern volatile uint32_t g_dpll_snapshot_sequence;
extern volatile uint64_t g_adc_last_block_sequence;
extern volatile uint32_t g_adc_blocks_processed;
extern volatile uint32_t g_adc_error_count;
extern volatile uint32_t g_adc_last_error_code;
extern volatile uint32_t g_dac_underrun_count;
extern volatile uint32_t g_nco_fill_cycles_last;
extern volatile uint32_t g_nco_fill_cycles_max;
extern volatile uint32_t g_adc_processing_cycles_last;
extern volatile uint32_t g_adc_processing_cycles_max;

#ifdef __cplusplus
}
#endif

#endif /* DPLL_APP_H */
