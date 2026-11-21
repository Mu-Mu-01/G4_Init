#include "dpll_pi.h"
#include "iq_phase.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_PI_D          3.1415926535897932384626433832795
#define TEST_TWO_PI_D      (2.0 * TEST_PI_D)
#define TEST_TWO_POW_64    18446744073709551616.0
#define TEST_SAMPLE_RATE   1000000.0
#define TEST_BLOCK_SAMPLES 512U
#define TEST_TOTAL_BLOCKS  3000U

static uint64_t frequency_to_fcw(double frequency_hz)
{
    return (uint64_t)(((frequency_hz / TEST_SAMPLE_RATE) * TEST_TWO_POW_64) +
                      0.5);
}

static double phase_to_rad(uint64_t phase)
{
    return ((double)phase / TEST_TWO_POW_64) * TEST_TWO_PI_D;
}

static uint16_t render_input(uint64_t phase)
{
    const double sample = 2200.0 + (650.0 * sin(phase_to_rad(phase)));
    return (uint16_t)(sample + 0.5);
}

static void require_close(double actual,
                          double expected,
                          double tolerance,
                          const char *message)
{
    if (fabs(actual - expected) > tolerance)
    {
        fprintf(stderr, "%s: actual=%g expected=%g\n",
                message, actual, expected);
        abort();
    }
}

static void test_low_frequency_lock(double nominal_hz, double input_hz)
{
    const uint64_t nominal_fcw = frequency_to_fcw(nominal_hz);
    const uint64_t input_fcw = frequency_to_fcw(input_hz);
    uint64_t applied_nco_fcw = nominal_fcw;
    uint64_t input_phase = UINT64_C(0x23456789abcdef01);
    uint64_t nco_phase = 0U;
    uint16_t samples[TEST_BLOCK_SAMPLES];
    IQ_PhaseState iq_state;
    IQ_PhaseAccumulator accumulator;
    IQ_PhaseResult iq_result = {0};
    DPLL_PI_Config pi_config;
    DPLL_PI_State pi_state;

    IQ_Phase_Init(&iq_state, nco_phase, applied_nco_fcw);
    IQ_Phase_AccumulatorReset(&accumulator);
    DPLL_PI_GetDefaultConfig(&pi_config, nominal_fcw, TEST_SAMPLE_RATE);
    pi_config.max_frequency_correction_hz = 100.0f; assert(DPLL_PI_Init(&pi_state, &pi_config));
    DPLL_PI_Enable(&pi_state, true);

    for (uint32_t block = 0U; block < TEST_TOTAL_BLOCKS; ++block)
    {
        for (uint32_t index = 0U; index < TEST_BLOCK_SAMPLES; ++index)
        {
            samples[index] = render_input(input_phase);
            input_phase += input_fcw;
        }

        IQ_Phase_Reset(&iq_state, nco_phase);
        const IQ_PhaseAccumulationStatus status =
            IQ_Phase_AccumulateU16Reference(
                &iq_state, &accumulator, samples, TEST_BLOCK_SAMPLES,
                applied_nco_fcw, applied_nco_fcw, &iq_result);
        nco_phase += applied_nco_fcw * TEST_BLOCK_SAMPLES;

        assert(status != IQ_PHASE_ACCUMULATION_ERROR);
        if (status == IQ_PHASE_ACCUMULATION_READY)
        {
            uint64_t commanded_fcw = applied_nco_fcw;

            if (DPLL_PI_Update(&pi_state,
                               iq_result.phase_rad,
                               iq_result.magnitude,
                               iq_result.valid,
                               true,
                               &commanded_fcw))
            {
                applied_nco_fcw = commanded_fcw;
            }
            IQ_Phase_AccumulatorReset(&accumulator);
        }
    }

    assert(pi_state.snapshot.signal_valid);
    assert(!pi_state.snapshot.saturated);
    require_close(pi_state.snapshot.output_frequency_hz, input_hz, 0.20,
                  "low-frequency DPLL did not converge");
    require_close(pi_state.snapshot.phase_error_rad, 0.0, 0.02,
                  "low-frequency phase error remained too large");
}

int main(void)
{
    test_low_frequency_lock(500.0, 500.0);
    test_low_frequency_lock(500.0, 599.0);
    test_low_frequency_lock(600.0, 501.0);
    puts("low-frequency DPLL tests passed");
    return 0;
}

