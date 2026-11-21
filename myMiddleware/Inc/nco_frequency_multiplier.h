/* myMiddleware 层：输出倍频边界与相位倍乘；不依赖板级资源。 */
#ifndef NCO_FREQUENCY_MULTIPLIER_H
#define NCO_FREQUENCY_MULTIPLIER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 1 MHz 时每个输出周期 11 个 DAC 采样是实际可行的上限。
 * 方波和三角波输出在 500 kHz 或更低时更干净。 */
#define NCO_FREQUENCY_MULTIPLIER_MAX_OUTPUT_HZ 1000000.0

bool NCO_FrequencyMultiplier_IsValid(uint32_t multiplier,
                                     double base_frequency_hz);
uint32_t NCO_FrequencyMultiplier_GetMaximum(double base_frequency_hz);
double NCO_FrequencyMultiplier_GetOutputHz(double base_frequency_hz,
                                            uint32_t multiplier);

/* 无符号溢出有意按 2^64 取模回绕，与 NCO 相位累加器完全一致。 */
static inline uint64_t NCO_FrequencyMultiplier_ApplyPhase(
    uint64_t base_phase,
    uint32_t multiplier)
{
    return base_phase * (uint64_t)multiplier;
}

#ifdef __cplusplus
}
#endif

#endif /* NCO_FREQUENCY_MULTIPLIER_H */
