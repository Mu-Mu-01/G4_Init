/**
 * @file    dpll_phase_calibration.c
 * @brief   频率相关相位校准模型及目标相位计算；不依赖板级资源。
 *
 * 本模块把示波器实测得到的"DAC 滞后随频率变化"的经验拟合（一次线性模型）换算成
 * PI 控制器的目标相位，用于补偿模拟链路的频率相关相位误差。
 *
 * 模型：
 *   measured_phase_deg = intercept_deg + slope_deg_per_khz * frequency_khz
 *   target_phase_rad  = -(measured_phase_deg + output_minus_input_phase_deg) * π/180
 *
 * 关键设计点：
 *  - 所有合法频率统一应用拟合，并叠加用户请求的锁定相位差。
 *  - 取相反数：拟合的 measured_phase 约定"正值表示 DAC 滞后"，而 PI 目标需要
 *    输出相对输入的相位，所以符号相反。
 *  - output_minus_input_phase_deg（用户请求的锁定相位差）在所有合法频率下始终生效。
 */
#include "dpll_phase_calibration.h"

#include <math.h>
#include <stddef.h>

/** 角度转弧度系数（π/180）。 */
#define DPLL_PHASE_CAL_DEG_TO_RAD_D \
    (3.14159265358979323846 / 180.0)

/** 频率拟合不设分段下限，所有合法频率（Hz）均使用同一线性模型。 */

/**
 * @brief  校验相位校准参数是否合法。
 * @param  calibration 待校验的校准参数。
 * @retval true  非空且截距/斜率均为有限值；false 否则。
 */
bool DPLL_PhaseCalibration_IsValid(
    const DPLL_PhaseCalibration *calibration)
{
    return (calibration != NULL) &&
           isfinite(calibration->intercept_deg) &&
           isfinite(calibration->slope_deg_per_khz);
}

/**
 * @brief  计算目标相位（便捷入口，不含用户请求的锁定相位差）。
 * @param  calibration      相位校准参数。
 * @param  frequency_hz     当前频率（Hz）。
 * @param  target_phase_rad 输出参数，写入目标相位（弧度）。
 * @retval true  计算成功；false 参数非法或结果非有限。
 * @note   等价于 DPLL_PhaseCalibration_CalculateLockTargetRadians 且
 *         output_minus_input_phase_deg = 0。
 */
bool DPLL_PhaseCalibration_CalculateTargetRadians(
    const DPLL_PhaseCalibration *calibration,
    double frequency_hz,
    float *target_phase_rad)
{
    return DPLL_PhaseCalibration_CalculateLockTargetRadians(
        calibration, frequency_hz, 0.0f, target_phase_rad);
}

/**
 * @brief  计算目标相位（含用户请求的锁定相位差）。
 * @param  calibration                  相位校准参数。
 * @param  frequency_hz                 当前频率（Hz）。
 * @param  output_minus_input_phase_deg 输出减输入的锁定相位差（度），
 *                                      正值表示 DAC 超前 ADC，负值表示滞后。
 * @param  target_phase_rad             输出参数，写入目标相位（弧度）。
 * @retval true  计算成功；false 参数非法或结果非有限。
 * @note   所有合法频率统一应用实测的 DAC 滞后拟合：
 *           measured_phase_deg = intercept_deg + slope_deg_per_khz * frequency_khz
 *         然后取相反数作为 PI 目标：
 *           target = -(measured + output_minus_input) * π/180。
 *         拟合贡献与 output_minus_input_phase_deg 均始终生效；计算不阻塞、不修改共享状态。
 */
bool DPLL_PhaseCalibration_CalculateLockTargetRadians(
    const DPLL_PhaseCalibration *calibration,
    double frequency_hz,
    float output_minus_input_phase_deg,
    float *target_phase_rad)
{
    double measured_phase_deg;
    double target_phase;

    if (!DPLL_PhaseCalibration_IsValid(calibration) ||
        !isfinite(frequency_hz) || (frequency_hz < 0.0) ||
        !isfinite(output_minus_input_phase_deg) ||
        (target_phase_rad == NULL))
    {
        return false;
    }

    /* 所有合法频率均套用频率拟合，并保留用户请求的锁定相位差。 */
    /* 一次线性拟合：intercept + slope * frequency_kHz。 */
    measured_phase_deg = (double)calibration->intercept_deg +
        ((double)calibration->slope_deg_per_khz * frequency_hz * 0.001);
    /* 取相反数（实测约定正值=滞后，目标需输出相对输入），并换算成弧度。 */
    target_phase = -(measured_phase_deg +
                     (double)output_minus_input_phase_deg) *
                   DPLL_PHASE_CAL_DEG_TO_RAD_D;
    if (!isfinite(target_phase))
    {
        return false;
    }

    *target_phase_rad = (float)target_phase;
    return isfinite(*target_phase_rad);
}
