/* myMiddleware 层：频率相关相位校准模型及目标相位计算；不依赖板级资源。 */
#ifndef DPLL_PHASE_CALIBRATION_H
#define DPLL_PHASE_CALIBRATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float intercept_deg;
    float slope_deg_per_khz;
} DPLL_PhaseCalibration;

bool DPLL_PhaseCalibration_IsValid(
    const DPLL_PhaseCalibration *calibration);

/* 所有合法频率统一应用实测的 DAC 滞后拟合，然后将其转换为相反的
 * PI 相位目标，不再按频率分段：
 * measured_phase_deg = intercept_deg + slope_deg_per_khz * frequency_khz
 * target_phase_rad = -measured_phase_deg * pi / 180。 */
bool DPLL_PhaseCalibration_CalculateTargetRadians(
    const DPLL_PhaseCalibration *calibration,
    double frequency_hz,
    float *target_phase_rad);

/* output_minus_input_phase_deg 在所有合法频率下始终与频率拟合叠加。
 * 正值表示 DAC 超前 ADC；负值表示 DAC 滞后 ADC。 */
bool DPLL_PhaseCalibration_CalculateLockTargetRadians(
    const DPLL_PhaseCalibration *calibration,
    double frequency_hz,
    float output_minus_input_phase_deg,
    float *target_phase_rad);

#ifdef __cplusplus
}
#endif

#endif /* DPLL_PHASE_CALIBRATION_H */
