/* myMiddleware 层：预计算波形表及码值渲染；不依赖板级资源。 */
#ifndef NCO_WAVEFORM_H
#define NCO_WAVEFORM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NCO_WAVEFORM_LUT_BITS 11U
#define NCO_WAVEFORM_LUT_SIZE (1U << NCO_WAVEFORM_LUT_BITS)

typedef enum
{
    NCO_OUTPUT_WAVEFORM_SINE = 0,
    NCO_OUTPUT_WAVEFORM_SQUARE,
    NCO_OUTPUT_WAVEFORM_TRIANGLE
} NCO_OutputWaveform;

typedef struct
{
    int16_t sine_q15[NCO_WAVEFORM_LUT_SIZE + 1U];
    int16_t square_q15[NCO_WAVEFORM_LUT_SIZE + 1U];
    int16_t triangle_q15[NCO_WAVEFORM_LUT_SIZE + 1U];
    bool initialized;
} NCO_WaveformLut;

void NCO_Waveform_Init(NCO_WaveformLut *lut);
bool NCO_Waveform_IsOutputWaveformValid(NCO_OutputWaveform waveform);
const int16_t *NCO_Waveform_GetTable(const NCO_WaveformLut *lut,
                                     NCO_OutputWaveform waveform);

/* 定点插值避免了 11 MHz 环中的浮点运算。 */
static inline uint16_t NCO_Waveform_RenderTable(const int16_t *table,
                                                uint64_t phase,
                                                uint16_t amplitude,
                                                uint16_t dc_offset)
{
    const uint32_t fraction_bits = 16U;
    const uint32_t fraction_shift =
        64U - NCO_WAVEFORM_LUT_BITS - fraction_bits;
    const uint32_t index =
        (uint32_t)(phase >> (64U - NCO_WAVEFORM_LUT_BITS));
    const uint32_t fraction =
        ((uint32_t)(phase >> fraction_shift)) & 0xFFFFU;
    const int32_t y0 = table[index];
    const int32_t delta = (int32_t)table[index + 1U] - y0;
    /* 方波表的边沿几乎跨越整个 Q15 范围，这会使 32 位插值乘积溢出，
     * 并且还会把边沿四舍五入掉。保持当前码值不变，
     * 直到相位索引越过精确的边界。 */
    const int32_t waveform_q15 = ((delta > INT16_MAX) ||
                                  (delta < INT16_MIN)) ? y0 :
        y0 + ((delta * (int32_t)fraction + 0x8000) >> fraction_bits);
    const int32_t scaled = (int32_t)amplitude * waveform_q15;
    const int32_t rounded = (scaled >= 0) ?
        ((scaled + 0x4000) >> 15U) :
        -(((-scaled) + 0x4000) >> 15U);

    return (uint16_t)((int32_t)dc_offset + rounded);
}

/* 相位检测器始终使用正弦参考。保持这个兼容渲染器显式绑定到
 * 正弦表，而不管 DAC 选择了哪种波形。 */
static inline uint16_t NCO_Waveform_Render(const NCO_WaveformLut *lut,
                                           uint64_t phase,
                                           uint16_t amplitude,
                                           uint16_t dc_offset)
{
    return NCO_Waveform_RenderTable(lut->sine_q15, phase,
                                    amplitude, dc_offset);
}

#ifdef __cplusplus
}
#endif

#endif /* NCO_WAVEFORM_H */
