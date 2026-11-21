/* myMiddleware 层：输出倍频边界与相位倍乘；不依赖板级资源。 */
#include "nco_frequency_multiplier.h"

#include <float.h>
#include <limits.h>
#include <math.h>

uint32_t NCO_FrequencyMultiplier_GetMaximum(double base_frequency_hz)
{
    double maximum_multiplier;

    if (!isfinite(base_frequency_hz) || (base_frequency_hz <= 0.0))
    {
        return 0U;
    }

    maximum_multiplier =
        NCO_FREQUENCY_MULTIPLIER_MAX_OUTPUT_HZ / base_frequency_hz;
    if (maximum_multiplier < 1.0)
    {
        return 0U;
    }
    if (maximum_multiplier >= (double)UINT32_MAX)
    {
        return UINT32_MAX;
    }

    /* FCW 转 Hz 可能把精确的名义边界放在其上方几个 double ULP 处
     * （例如 1 kHz x 1000）。不要仅仅因为这种往返量化就丢掉一个
     * 有效的整数倍。 */
    maximum_multiplier +=
        fmax(1.0, maximum_multiplier) * (16.0 * DBL_EPSILON);
    if (maximum_multiplier >= (double)UINT32_MAX)
    {
        return UINT32_MAX;
    }
    return (uint32_t)maximum_multiplier;
}

bool NCO_FrequencyMultiplier_IsValid(uint32_t multiplier,
                                     double base_frequency_hz)
{
    return (multiplier != 0U) &&
           (multiplier <=
            NCO_FrequencyMultiplier_GetMaximum(base_frequency_hz));
}

double NCO_FrequencyMultiplier_GetOutputHz(double base_frequency_hz,
                                            uint32_t multiplier)
{
    double output_frequency_hz;

    if (!isfinite(base_frequency_hz) || (base_frequency_hz < 0.0) ||
        (multiplier == 0U))
    {
        return 0.0;
    }

    output_frequency_hz = base_frequency_hz * (double)multiplier;
    return isfinite(output_frequency_hz) ? output_frequency_hz : 0.0;
}
