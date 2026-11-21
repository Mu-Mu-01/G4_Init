/* myMiddleware 层：整数余数相位插值；不依赖板级资源。 */
#ifndef NCO_PHASE_INTERP_H
#define NCO_PHASE_INTERP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint64_t phase;
    uint64_t phase_step_quotient;
    uint32_t phase_step_remainder;
    uint32_t remainder_accumulator;
    uint32_t interpolation_factor;
} NCO_PhaseInterpolator;

bool NCO_PhaseInterp_Init(NCO_PhaseInterpolator *state,
                          uint64_t initial_phase,
                          uint64_t base_frequency_control_word,
                          uint32_t interpolation_factor,
                          uint32_t initial_remainder);
uint64_t NCO_PhaseInterp_GetPhase(const NCO_PhaseInterpolator *state);
void NCO_PhaseInterp_Advance(NCO_PhaseInterpolator *state);

/* 经过 output_sample_index 个插值步进后的精确模 2^64 相位偏移。 */
uint64_t NCO_PhaseInterp_GetOffset(uint64_t base_frequency_control_word,
                                   uint32_t interpolation_factor,
                                   uint32_t output_sample_index,
                                   uint32_t initial_remainder);

#ifdef __cplusplus
}
#endif

#endif /* NCO_PHASE_INTERP_H */
