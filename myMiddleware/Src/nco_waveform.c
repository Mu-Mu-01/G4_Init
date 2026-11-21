/**
 * @file    nco_waveform.c
 * @brief   NCO 波形查找表（LUT）模块：预计算正弦/方波/三角波表并提供选择接口。
 *
 * 本模块在初始化阶段一次性生成三张 Q15 定点波形表，供 NCO DAC 在配置的
 * 输出采样率实时重填中断中做"查表 + 线性插值"渲染。这样就把耗时的三角函数运算从
 * 热路径中剥离出来，运行期只保留纯整数运算，避免在中断里调用 sinf。
 *
 * 关键设计点：
 *  - Q15 定点格式：1.0 用 32767 表示。int16 存不下 32768（会溢出），
 *    所以正负幅值都取 ±32767，保持正负对称，便于后续乘幅度、加偏置。
 *  - 表长 2048（NCO_WAVEFORM_LUT_SIZE = 2^11）覆盖一个完整周期，
 *    折点（1/4、1/2、3/4 周期）都是精确的 2 的幂，折线边界无舍入误差。
 *  - 每张表多存一个"回绕元素"（索引 2048 号 == 0 号），让插值器在
 *    周期末尾能安全读取 index+1，无需在渲染热路径里做取模或边界判断。
 *  - 正弦支持两条路径：定义 NCO_WAVEFORM_USE_CMSIS_DSP 时用
 *    arm_sin_q31（Q31 定点），否则用软件 sinf（浮点），产物一致。
 */
#include "nco_waveform.h"

#include <math.h>
#include <stddef.h>

#if defined(NCO_WAVEFORM_USE_CMSIS_DSP)
#include "arm_math.h"
#endif

/** 2π 的 float32 常量，用于把表索引换算成弧度。 */
#define NCO_TWO_PI_F 6.28318530717958647692f

/** 1/4 周期对应的表点数，三角波折线的每一段长度。 */
#define NCO_WAVEFORM_QUARTER_SIZE (NCO_WAVEFORM_LUT_SIZE / 4U)

/**
 * @brief  判断给定的波形枚举值是否合法（正弦/方波/三角波之一）。
 * @param  waveform 待校验的波形枚举值。
 * @retval true  波形合法；false 波形非法。
 */
bool NCO_Waveform_IsOutputWaveformValid(NCO_OutputWaveform waveform)
{
    return (waveform == NCO_OUTPUT_WAVEFORM_SINE) ||
           (waveform == NCO_OUTPUT_WAVEFORM_SQUARE) ||
           (waveform == NCO_OUTPUT_WAVEFORM_TRIANGLE);
}

/**
 * @brief  取出指定波形对应的 Q15 查找表首指针。
 * @param  lut      已初始化的波形表结构（须先调用 NCO_Waveform_Init）。
 * @param  waveform 目标波形枚举值。
 * @retval 非 NULL 指向对应波形表；NULL 表示表未初始化、lut 为空或波形非法。
 * @note   返回的指针指向结构内部数组，调用方只读、不释放；整个运行期有效。
 */
const int16_t *NCO_Waveform_GetTable(const NCO_WaveformLut *lut,
                                     NCO_OutputWaveform waveform)
{
    /* 表未就绪、指针为空或波形非法时直接返回空，让调用方回退到正弦表。 */
    if ((lut == NULL) || !lut->initialized ||
        !NCO_Waveform_IsOutputWaveformValid(waveform))
    {
        return NULL;
    }

    switch (waveform)
    {
        case NCO_OUTPUT_WAVEFORM_SQUARE:
            return lut->square_q15;
        case NCO_OUTPUT_WAVEFORM_TRIANGLE:
            return lut->triangle_q15;
        case NCO_OUTPUT_WAVEFORM_SINE:
        default:
            return lut->sine_q15;
    }
}

/**
 * @brief  预计算三张波形查找表（正弦/方波/三角波），并设置就绪标记。
 * @param  lut 待填充的波形表结构；为空时直接返回、不做任何事。
 * @note   每张表生成 NCO_WAVEFORM_LUT_SIZE 个点，最后多写一个回绕点
 *         （索引 == 表长，值复制自 0 号），供插值器在周期末尾安全读取
 *          index+1。该函数应在启动前调用一次，运行期不改动表内容。
 */
void NCO_Waveform_Init(NCO_WaveformLut *lut)
{
    if (lut == NULL)
    {
        return;
    }

    /* 遍历一个完整周期（0~2π），每个索引同时生成三种波形。 */
    for (uint32_t index = 0U; index < NCO_WAVEFORM_LUT_SIZE; ++index)
    {
        /* 三角波当前点的 Q15 值，由下方四段折线计算。 */
        int32_t triangle_q15;

#if defined(NCO_WAVEFORM_USE_CMSIS_DSP)
        /* CMSIS 路径：index 左移 20 位得到 Q31 相位（0~2π 覆盖 Q31 全范围），
         * arm_sin_q31 返回 Q31 正弦，再 >>16 转成 Q15（int16）。 */
        const uint32_t phase = index << (31U - NCO_WAVEFORM_LUT_BITS);
        lut->sine_q15[index] = (int16_t)(arm_sin_q31((q31_t)phase) >> 16U);
#else
        /* 软件路径：把 index 换算成弧度，用 sinf 计算后乘 32767 缩放到 Q15。 */
        const float phase = NCO_TWO_PI_F * (float)index /
                            (float)NCO_WAVEFORM_LUT_SIZE;
        lut->sine_q15[index] = (int16_t)(sinf(phase) * 32767.0f);
#endif

        /* 方波：前半周期 +32767，后半周期 -32767。用 -INT16_MAX 而非
         * INT16_MIN，保持正负幅值对称，避免后续幅度缩放产生不对称舍入。 */
        lut->square_q15[index] =
            (index < (NCO_WAVEFORM_LUT_SIZE / 2U)) ? INT16_MAX : -INT16_MAX;

        /* 三角波按四段折线生成，走势为 0→+max→-max→0。所有乘法先升到
         * int32 再除，避免中间结果溢出 int16。 */
        if (index < NCO_WAVEFORM_QUARTER_SIZE)
        {
            /* 第 1 段（0~1/4 周期）：从 0 线性上升到 +32767。 */
            triangle_q15 = (int32_t)index * INT16_MAX /
                           (int32_t)NCO_WAVEFORM_QUARTER_SIZE;
        }
        else if (index < (2U * NCO_WAVEFORM_QUARTER_SIZE))
        {
            /* 第 2 段（1/4~1/2 周期）：从 +32767 线性下降到 0。 */
            triangle_q15 =
                (int32_t)(2U * NCO_WAVEFORM_QUARTER_SIZE - index) *
                INT16_MAX / (int32_t)NCO_WAVEFORM_QUARTER_SIZE;
        }
        else if (index < (3U * NCO_WAVEFORM_QUARTER_SIZE))
        {
            /* 第 3 段（1/2~3/4 周期）：从 0 线性下降到 -32767。 */
            triangle_q15 =
                -((int32_t)(index - 2U * NCO_WAVEFORM_QUARTER_SIZE) *
                  INT16_MAX / (int32_t)NCO_WAVEFORM_QUARTER_SIZE);
        }
        else
        {
            /* 第 4 段（3/4~1 周期）：从 -32767 线性上升到 0，回到起点。 */
            triangle_q15 =
                -((int32_t)(NCO_WAVEFORM_LUT_SIZE - index) * INT16_MAX /
                  (int32_t)NCO_WAVEFORM_QUARTER_SIZE);
        }
        lut->triangle_q15[index] = (int16_t)triangle_q15;
    }

    /* 回绕元素：索引 2048 号复制 0 号，使插值器读取 table[2047] 的
     * table[index+1] 时能无缝回到周期起点，避免边界判断。 */
    lut->sine_q15[NCO_WAVEFORM_LUT_SIZE] = lut->sine_q15[0];
    lut->square_q15[NCO_WAVEFORM_LUT_SIZE] = lut->square_q15[0];
    lut->triangle_q15[NCO_WAVEFORM_LUT_SIZE] = lut->triangle_q15[0];
    lut->initialized = true;
}
