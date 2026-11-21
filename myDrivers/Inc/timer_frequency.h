/** @file timer_frequency.h
 * @brief Driver 层：无硬件依赖的 PSC/ARR 最近频率求解器。
 */
#ifndef TIMER_FREQUENCY_H
#define TIMER_FREQUENCY_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    uint32_t prescaler; /**< PSC 寄存器值，实际分频为值加一。 */
    uint32_t autoreload; /**< ARR 寄存器值，实际周期为值加一。 */
    uint64_t period_ticks; /**< 总分频；上计数、RCR=0 时每次更新的输入时钟数。 */
    double actual_hz; /**< 实际可实现的更新率，Hz。 */
    double error_hz; /**< actual_hz - requested_hz，带符号误差。 */
    double error_ppm; /**< 相对请求频率误差，百万分比。 */
} TimerFrequencyPlan;
/* max_arr 为寄存器最大值：16 位填 UINT16_MAX，32 位填 UINT32_MAX。
 * 全覆盖 PSC=0..65535，检查每个 PSC 下目标 ARR 两侧候选；无全局可变状态。
 * 不可表示的目标返回 false，不静默截断到边界。主循环调用，最多 131072 次候选。 */
bool TimerFrequency_Find(double clock_hz, double requested_hz,
                         uint32_t max_arr, TimerFrequencyPlan *plan);

/**
 * @brief  查找严格等于指定整数倍数的 PSC/ARR 级联分频。
 * @param  clock_hz 输入定时器时钟，Hz，有限正数。
 * @param  requested_hz 分频后的请求更新率，Hz，用于计算误差字段。
 * @param  divider 需要保持的总整数分频倍数，必须大于 0。
 * @param  max_arr ARR 最大寄存器值，由定时器位宽决定。
 * @param  plan 成功时写入精确分频计划；失败时不改写。
 * @retval true 找到 PSC/ARR 组合；false 参数非法或倍数超出定时器能力。
 * @note   仅做数学计算；优先使用较小 PSC。适用于主定时器事件级联，
 *         可保证 ADC/DAC 使用同一主时钟时保持固定整数速率比。
 */
bool TimerFrequency_FindExactDivider(double clock_hz, double requested_hz,
                                     uint32_t divider, uint32_t max_arr,
                                     TimerFrequencyPlan *plan);
#endif
