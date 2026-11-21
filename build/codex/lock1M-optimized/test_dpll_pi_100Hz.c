#include "dpll_pi.h"
#include "dpll_phase_calibration.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_PI_F         3.14159265358979323846f
#define TEST_TWO_PI_F     6.28318530717958647692f
#define TEST_SAMPLE_RATE  1000000.0
#define TEST_NOMINAL_HZ   10000.0

static void require_condition(bool condition, const char *message)
{
    if (!condition)
    {
        fputs(message, stderr);
        fputc('\n', stderr);
        abort();
    }
}

static uint64_t frequency_to_fcw(double frequency_hz)
{
    return (uint64_t)((frequency_hz / TEST_SAMPLE_RATE) *
                      18446744073709551616.0 + 0.5);
}

static float wrap_phase(float phase_rad)
{
    float wrapped = remainderf(phase_rad, TEST_TWO_PI_F);
    if (wrapped <= -TEST_PI_F)
    {
        wrapped += TEST_TWO_PI_F;
    }
    return wrapped;
}

static DPLL_PI_State make_enabled_controller(void)
{
    DPLL_PI_Config config;
    DPLL_PI_State state;

    DPLL_PI_GetDefaultConfig(&config, frequency_to_fcw(TEST_NOMINAL_HZ),
                             TEST_SAMPLE_RATE);
    config.max_frequency_correction_hz = 100.0f; assert(DPLL_PI_Init(&state, &config));
    DPLL_PI_Enable(&state, true);
    return state;
}

static void acquire_signal(DPLL_PI_State *state, float phase_rad)
{
    uint64_t fcw = 0U;

    for (unsigned int index = 0U;
         index < DPLL_PI_DEFAULT_ACQUIRE_BLOCKS;
         ++index)
    {
        (void)DPLL_PI_Update(state, phase_rad, 120.0f, true, true, &fcw);
    }
    assert(state->snapshot.signal_valid);
    assert(state->snapshot.update_applied);
}

static void test_atan2_wrap_has_no_two_pi_impulse(void)
{
    DPLL_PI_State state = make_enabled_controller();
    uint64_t fcw;
    float before;

    acquire_signal(&state, TEST_PI_F - 0.01f);
    before = state.snapshot.continuous_phase_rad;
    const float p_before = state.snapshot.p_term_hz;
    assert(DPLL_PI_Update(&state, -TEST_PI_F + 0.01f, 120.0f,
                          true, true, &fcw));
    assert(fabsf((state.snapshot.continuous_phase_rad - before) - 0.02f) < 1.0e-4f);
    assert(fabsf(state.snapshot.phase_error_rad) <= TEST_PI_F);
    assert(state.snapshot.phase_error_rad > 3.0f);
    assert(fabsf(state.snapshot.p_term_hz - p_before) < 1.0f);
    assert(fabsf(state.snapshot.frequency_correction_hz) <= 100.0f);
}

static void test_runtime_target_update_is_bumpless(void)
{
    DPLL_PI_State state = make_enabled_controller();
    float controller_output_before;
    float controller_output_after;

    acquire_signal(&state, 0.3f);
    controller_output_before = state.snapshot.p_term_hz +
                               state.snapshot.i_term_hz;

    assert(DPLL_PI_SetTargetPhase(&state, -0.2f));
    controller_output_after = state.snapshot.p_term_hz +
                              state.snapshot.i_term_hz;

    assert(fabsf(controller_output_after - controller_output_before) < 1.0e-4f);
    assert(fabsf(state.snapshot.phase_error_rad - 0.5f) < 1.0e-5f);
    assert(!DPLL_PI_SetTargetPhase(&state, NAN));
}

static void test_phase_calibration_fit(void)
{
    static const struct
    {
        double frequency_hz;
        float expected_phase_deg;
    } cases[] = {
        {10000.0, 4.149091f},
        {50000.0, 10.478788f},
        {80000.0, 15.226061f},
        {100000.0, 18.390909f},
    };
    const DPLL_PhaseCalibration calibration = {
        .intercept_deg = 2.566667f,
        .slope_deg_per_khz = 0.15824242f,
    };
    const DPLL_PhaseCalibration disabled = {0.0f, 0.0f};
    DPLL_PhaseCalibration invalid = {NAN, 0.0f};
    float target_phase_rad;

    assert(DPLL_PhaseCalibration_IsValid(&calibration));
    for (size_t index = 0U; index < (sizeof(cases) / sizeof(cases[0])); ++index)
    {
        float measured_phase_deg;

        assert(DPLL_PhaseCalibration_CalculateTargetRadians(
            &calibration, cases[index].frequency_hz, &target_phase_rad));
        measured_phase_deg = -target_phase_rad * (180.0f / TEST_PI_F);
        assert(fabsf(measured_phase_deg - cases[index].expected_phase_deg) <
               1.0e-3f);
    }

    assert(DPLL_PhaseCalibration_CalculateTargetRadians(
        &disabled, 100000.0, &target_phase_rad));
    assert(target_phase_rad == 0.0f);
    assert(!DPLL_PhaseCalibration_CalculateTargetRadians(
        &calibration, NAN, &target_phase_rad));
    assert(!DPLL_PhaseCalibration_IsValid(&invalid));
    invalid.intercept_deg = 0.0f;
    invalid.slope_deg_per_khz = INFINITY;
    assert(!DPLL_PhaseCalibration_IsValid(&invalid));
}

/**
 * @brief  验证低频和原 10 kHz 边界两侧统一应用校准，边界没有相位阶跃。
 * @param  无。
 * @retval 无；断言失败时测试进程退出。
 * @note   主机测试，误差容限 0.001 度；零频率沿用接口允许的非负频率契约。
 */
static void test_phase_calibration_uniform_across_frequencies(void)
{
    /* 频率单位 Hz；预期校准量单位度，包含原门限两侧及高频回归点。 */
    static const struct {
        double frequency_hz; /* 被测频率，Hz */
        float expected_phase_deg; /* 未取反的校准量，度 */
    } cases[] = {
        {0.0, 2.566667f}, {800.0, 2.693260936f},
        {9999.999, 4.149091042f}, {10000.0, 4.1490912f},
        {10000.001, 4.149091358f}, {100000.0, 18.390909f},
    };
    const DPLL_PhaseCalibration calibration = { /* 截距为度，斜率为度/kHz */
        .intercept_deg = 2.566667f,
        .slope_deg_per_khz = 0.15824242f,
    };
    float target_phase_rad; /* 接口输出，弧度 */
    float previous_phase_rad = 0.0f; /* 保存原门限下方的目标，用于连续性比较 */
    for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index)
    {
        assert(DPLL_PhaseCalibration_CalculateTargetRadians(
            &calibration, cases[index].frequency_hz, &target_phase_rad));
        assert(fabsf(target_phase_rad * (180.0f / TEST_PI_F) +
                     cases[index].expected_phase_deg) < 1.0e-3f);
        if (index == 2U) { previous_phase_rad = target_phase_rad; }
        if (index == 4U) { assert(fabsf(target_phase_rad - previous_phase_rad) < 1.0e-6f); }
    }
    assert(!DPLL_PhaseCalibration_CalculateTargetRadians(
        &calibration, -1.0, &target_phase_rad));
    assert(!DPLL_PhaseCalibration_CalculateTargetRadians(
        &calibration, INFINITY, &target_phase_rad));
}

static void test_requested_locked_phase_is_added_to_calibration(void)
{
    const DPLL_PhaseCalibration calibration = {
        .intercept_deg = 2.566667f,
        .slope_deg_per_khz = 0.15824242f,
    };
    float target_phase_rad;
    float target_phase_deg;

    assert(DPLL_PhaseCalibration_CalculateLockTargetRadians(
        &calibration, 50000.0, 30.0f, &target_phase_rad));
    target_phase_deg = target_phase_rad * (180.0f / TEST_PI_F);
    assert(fabsf(target_phase_deg - (-40.478788f)) < 1.0e-3f);

    assert(DPLL_PhaseCalibration_CalculateLockTargetRadians(
        &calibration, 50000.0, -30.0f, &target_phase_rad));
    target_phase_deg = target_phase_rad * (180.0f / TEST_PI_F);
    assert(fabsf(target_phase_deg - 19.521212f) < 1.0e-3f);
    assert(!DPLL_PhaseCalibration_CalculateLockTargetRadians(
        &calibration, 50000.0, NAN, &target_phase_rad));
}

/**
 * @brief  验证低频请求的正负相位差均与非零校准量叠加。
 * @param  无。
 * @retval 无；断言失败时测试进程退出。
 * @note   主机测试，800 Hz 下校准量为 2.693260936 度，容限 0.0001 度。
 */
static void test_requested_locked_phase_adds_calibration_at_low_frequency(void)
{
    const DPLL_PhaseCalibration calibration = {
        .intercept_deg = 2.566667f,
        .slope_deg_per_khz = 0.15824242f,
    };
    float target_phase_rad;
    float target_phase_deg;

    assert(DPLL_PhaseCalibration_CalculateLockTargetRadians(
        &calibration, 800.0, 30.0f, &target_phase_rad));
    target_phase_deg = target_phase_rad * (180.0f / TEST_PI_F);
    assert(fabsf(target_phase_deg - (-32.693260936f)) < 1.0e-4f);

    assert(DPLL_PhaseCalibration_CalculateLockTargetRadians(
        &calibration, 800.0, -30.0f, &target_phase_rad));
    target_phase_deg = target_phase_rad * (180.0f / TEST_PI_F);
    assert(fabsf(target_phase_deg - 27.306739064f) < 1.0e-4f);
}

static void test_magnitude_hysteresis_and_freeze(void)
{
    DPLL_PI_State state = make_enabled_controller();
    uint64_t fcw = 123U;
    uint64_t held_fcw;

    for (unsigned int index = 0U; index < 3U; ++index)
    {
        assert(!DPLL_PI_Update(&state, 0.2f, 100.0f, true, true, &fcw));
    }
    assert(!state.snapshot.signal_valid);
    assert(DPLL_PI_Update(&state, 0.2f, 100.0f, true, true, &fcw));
    held_fcw = fcw;

    assert(DPLL_PI_Update(&state, 0.2f, 79.0f, true, true, &fcw));
    assert(state.snapshot.signal_valid);
    assert(!DPLL_PI_Update(&state, 0.2f, 79.0f, true, true, &fcw));
    assert(!state.snapshot.signal_valid);

    fcw = held_fcw;
    assert(!DPLL_PI_Update(&state, 1.0f, 120.0f, false, true, &fcw));
    assert(fcw == held_fcw);
    assert(!DPLL_PI_Update(&state, 1.0f, 120.0f, true, false, &fcw));
    assert(fcw == held_fcw);
    assert(state.snapshot.reference_mismatch_count == 1U);
}

static void test_saturation_and_recovery(void)
{
    DPLL_PI_State state = make_enabled_controller();
    uint64_t fcw;

    acquire_signal(&state, 3.0f);
    for (unsigned int index = 0U; index < 20000U; ++index)
    {
        assert(DPLL_PI_Update(&state, 3.0f, 120.0f, true, true, &fcw));
    }
    assert(state.snapshot.saturated);
    assert(fabsf(state.snapshot.frequency_correction_hz - 100.0f) < 1.0e-3f);
    assert(state.snapshot.i_term_hz < 60.0f);

    bool recovered = false;
    float recovery_phase = 3.0f;
    for (unsigned int index = 0U; index < 3000U; ++index)
    {
        recovery_phase -= 0.01f;
        assert(DPLL_PI_Update(&state, wrap_phase(recovery_phase), 120.0f,
                              true, true, &fcw));
        if ((state.snapshot.frequency_correction_hz < 0.0f) &&
            !state.snapshot.saturated)
        {
            recovered = true;
            break;
        }
    }
    assert(recovered);
}

static void test_frequency_lock(float input_offset_hz)
{
    DPLL_PI_State state = make_enabled_controller();
    float phase_error = 0.0f;
    uint64_t fcw = 0U;

    for (unsigned int index = 0U; index < 20000U; ++index)
    {
        const bool updated = DPLL_PI_Update(&state, wrap_phase(phase_error),
                                            120.0f, true, true, &fcw);
        const float nco_offset_hz = updated ?
            state.snapshot.frequency_correction_hz : 0.0f;

        phase_error += TEST_TWO_PI_F *
            (input_offset_hz - nco_offset_hz) *
            DPLL_PI_DEFAULT_UPDATE_PERIOD_S;
    }

    assert(fabsf(state.snapshot.frequency_correction_hz - input_offset_hz) < 0.05f);
    assert(fabsf(state.snapshot.phase_error_rad) < 0.01f);
}

static void test_frequency_lock_with_command_delay(float input_offset_hz,
                                                   unsigned int delay_blocks)
{
    DPLL_PI_State state = make_enabled_controller();
    float correction_delay[4] = {0.0f};
    float phase_error = 0.0f;
    uint64_t fcw = 0U;
    unsigned int delay_index = 0U;

    assert((delay_blocks > 0U) && (delay_blocks <= 4U));
    for (unsigned int index = 0U; index < 30000U; ++index)
    {
        const float applied_correction = correction_delay[delay_index];

        phase_error += TEST_TWO_PI_F *
            (input_offset_hz - applied_correction) *
            DPLL_PI_DEFAULT_UPDATE_PERIOD_S;
        if (DPLL_PI_Update(&state, wrap_phase(phase_error), 120.0f,
                           true, true, &fcw))
        {
            correction_delay[delay_index] =
                state.snapshot.frequency_correction_hz;
        }
        delay_index = (delay_index + 1U) % delay_blocks;
    }

    if ((fabsf(state.snapshot.frequency_correction_hz - input_offset_hz) >= 0.05f) ||
        (fabsf(state.snapshot.phase_error_rad) >= 0.01f))
    {
        fprintf(stderr, "delayed lock failed: input=%g delay=%u correction=%g phase=%g\n",
                (double)input_offset_hz, delay_blocks,
                (double)state.snapshot.frequency_correction_hz,
                (double)state.snapshot.phase_error_rad);
        abort();
    }
}

static void test_out_of_range_frequency_recovers(void)
{
    DPLL_PI_State state = make_enabled_controller();
    float phase_error = 0.0f;
    float applied_correction_hz = 0.0f;
    float maximum_abs_i_term_hz = 0.0f;
    uint64_t fcw = 0U;

    for (unsigned int index = 0U; index < 30000U; ++index)
    {
        /* 110 Hz 超出 +/-100 Hz 的执行器限制，但仍低于
         * 4.096 ms 处 +/-122.07 Hz 的相位增量歧义限制。 */
        phase_error += TEST_TWO_PI_F * (110.0f - applied_correction_hz) *
                       DPLL_PI_DEFAULT_UPDATE_PERIOD_S;
        if (DPLL_PI_Update(&state, wrap_phase(phase_error), 120.0f,
                           true, true, &fcw))
        {
            applied_correction_hz = state.snapshot.frequency_correction_hz;
        }
        if (fabsf(state.snapshot.i_term_hz) > maximum_abs_i_term_hz)
        {
            maximum_abs_i_term_hz = fabsf(state.snapshot.i_term_hz);
        }
    }
    assert(state.snapshot.saturated);
    assert(fabsf(state.snapshot.frequency_correction_hz - 100.0f) < 1.0e-3f);
    require_condition(maximum_abs_i_term_hz < 101.0f,
                      "integrator wound up outside the output limit");

    for (unsigned int index = 0U; index < 30000U; ++index)
    {
        phase_error += TEST_TWO_PI_F * (50.0f - applied_correction_hz) *
                       DPLL_PI_DEFAULT_UPDATE_PERIOD_S;
        assert(DPLL_PI_Update(&state, wrap_phase(phase_error), 120.0f,
                              true, true, &fcw));
        applied_correction_hz = state.snapshot.frequency_correction_hz;
    }
    require_condition(!state.snapshot.saturated,
                      "controller remained saturated after returning in range");
    require_condition(fabsf(state.snapshot.frequency_correction_hz - 50.0f) <
                          0.05f,
                      "controller did not recover to the in-range frequency");
    require_condition(fabsf(state.snapshot.phase_error_rad) < 0.01f,
                      "phase error did not recover after saturation");
}

static void test_signal_recovery_uses_nearest_phase_branch(void)
{
    DPLL_PI_State state = make_enabled_controller();
    uint64_t fcw;
    float continuous_before;

    acquire_signal(&state, TEST_PI_F - 0.04f);
    continuous_before = state.snapshot.continuous_phase_rad;
    assert(!DPLL_PI_Update(&state, 0.0f, 120.0f, false, true, &fcw));
    assert(DPLL_PI_Update(&state, -TEST_PI_F + 0.04f, 120.0f,
                          true, true, &fcw));
    assert(fabsf((state.snapshot.continuous_phase_rad - continuous_before) -
                 0.08f) < 1.0e-4f);
}

static void test_cycle_slip_rebase(void)
{
    DPLL_PI_Config config;
    DPLL_PI_State state;
    uint64_t fcw;
    float phase = 0.0f;
    float previous_correction = 0.0f;
    bool rebase_was_continuous = false;

    DPLL_PI_GetDefaultConfig(&config, frequency_to_fcw(TEST_NOMINAL_HZ),
                             TEST_SAMPLE_RATE);
    config.kp_hz_per_rad = 1.0f;
    config.ki_hz_per_rad_s = 0.0f;
    config.max_frequency_correction_hz = 100.0f; assert(DPLL_PI_Init(&state, &config));
    DPLL_PI_Enable(&state, true);

    acquire_signal(&state, phase);
    for (unsigned int index = 0U; index < 12U; ++index)
    {
        phase += 1.2f;
        const uint32_t slips_before = state.snapshot.cycle_slip_count;
        assert(DPLL_PI_Update(&state, wrap_phase(phase), 120.0f,
                              true, true, &fcw));
        if (state.snapshot.cycle_slip_count != slips_before)
        {
            rebase_was_continuous =
                fabsf(state.snapshot.frequency_correction_hz -
                      previous_correction) < 1.0e-4f;
        }
        previous_correction = state.snapshot.frequency_correction_hz;
    }
    assert(state.snapshot.cycle_slip_count == 2U);
    assert(fabsf(state.snapshot.continuous_phase_rad) < TEST_PI_F);
    assert(rebase_was_continuous);
}

int main(void)
{
    test_atan2_wrap_has_no_two_pi_impulse();
    test_runtime_target_update_is_bumpless();
    test_phase_calibration_fit();
    test_phase_calibration_uniform_across_frequencies();
    test_requested_locked_phase_is_added_to_calibration();
    test_requested_locked_phase_adds_calibration_at_low_frequency();
    test_magnitude_hysteresis_and_freeze();
    test_saturation_and_recovery();
    test_frequency_lock(5.0f);
    test_frequency_lock(-5.0f);
    test_frequency_lock(50.0f);
    test_frequency_lock(-50.0f);
    test_frequency_lock(99.0f);
    test_frequency_lock(-99.0f);
    test_frequency_lock_with_command_delay(99.0f, 2U);
    test_frequency_lock_with_command_delay(-99.0f, 2U);
    test_frequency_lock_with_command_delay(99.0f, 3U);
    test_frequency_lock_with_command_delay(-99.0f, 3U);
    test_out_of_range_frequency_recovers();
    test_signal_recovery_uses_nearest_phase_branch();
    test_cycle_slip_rebase();
    puts("dpll_pi tests passed");
    return 0;
}

