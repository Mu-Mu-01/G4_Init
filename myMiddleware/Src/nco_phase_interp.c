/* myMiddleware 层：整数余数相位插值；不依赖板级资源。 */
#include "nco_phase_interp.h"

#include <stddef.h>

bool NCO_PhaseInterp_Init(NCO_PhaseInterpolator *state,
                          uint64_t initial_phase,
                          uint64_t base_frequency_control_word,
                          uint32_t interpolation_factor,
                          uint32_t initial_remainder)
{
    if ((state == NULL) || (interpolation_factor == 0U) ||
        (initial_remainder >= interpolation_factor))
    {
        return false;
    }

    state->phase = initial_phase;
    state->phase_step_quotient =
        base_frequency_control_word / interpolation_factor;
    state->phase_step_remainder =
        (uint32_t)(base_frequency_control_word % interpolation_factor);
    state->remainder_accumulator = initial_remainder;
    state->interpolation_factor = interpolation_factor;
    return true;
}

uint64_t NCO_PhaseInterp_GetPhase(const NCO_PhaseInterpolator *state)
{
    return (state != NULL) ? state->phase : 0U;
}

void NCO_PhaseInterp_Advance(NCO_PhaseInterpolator *state)
{
    if ((state == NULL) || (state->interpolation_factor == 0U))
    {
        return;
    }

    state->phase += state->phase_step_quotient;
    state->remainder_accumulator += state->phase_step_remainder;
    if (state->remainder_accumulator >= state->interpolation_factor)
    {
        state->phase++;
        state->remainder_accumulator -= state->interpolation_factor;
    }
}

uint64_t NCO_PhaseInterp_GetOffset(uint64_t base_frequency_control_word,
                                   uint32_t interpolation_factor,
                                   uint32_t output_sample_index,
                                   uint32_t initial_remainder)
{
    uint64_t quotient;
    uint32_t remainder;

    if ((interpolation_factor == 0U) ||
        (initial_remainder >= interpolation_factor))
    {
        return 0U;
    }

    quotient = base_frequency_control_word / interpolation_factor;
    remainder = (uint32_t)(base_frequency_control_word % interpolation_factor);

    /* output_sample_index 受单个 DMA 缓冲区限制，因此只有商乘积
     * 有意按 2^64 取模回绕。 */
    return ((uint64_t)output_sample_index * quotient) +
           ((((uint64_t)initial_remainder +
              ((uint64_t)output_sample_index * remainder)) /
             interpolation_factor));
}
