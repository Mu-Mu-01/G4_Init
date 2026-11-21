#include "arm_math.h"

/* 保存 8 个窗系数和单精度正弦结果。 */
static float32_t window[8];
static volatile float32_t result;

/**
 * @brief  引用单精度 DSP 函数以验证链接。
 * @param  无。
 * @retval 无。
 * @note   仅用于链接检查。
 */
void link_check(void)
{
    arm_hanning_f32(window, 8);
    result = arm_sin_f32(window[1]);
}
