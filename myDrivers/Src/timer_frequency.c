/** @file timer_frequency.c
 * @brief Driver 层：参考 Set_Fre.c 思路，完整搜索最近可实现的定时器更新率。
 */
#include "timer_frequency.h"
#include <math.h>
#include <stddef.h>
#include <float.h>

/**
 * @brief 计算最近的 PSC/ARR 组合和带符号频率误差。
 * @param clock_hz 实际定时器输入时钟，Hz，有限正数。
 * @param requested_hz 请求更新率，Hz，有限正数且在可表示范围内。
 * @param max_arr ARR 最大寄存器值，由定时器位宽决定。
 * @param plan 成功时写入计划；失败不改写。
 * @retval true 找到计划；false 参数非法或目标超出分频范围。
 * @note 仅做数学计算；同等误差优先较小 PSC。无阻塞 I/O，不用于中断。
 */
bool TimerFrequency_Find(double clock_hz, double requested_hz,
                         uint32_t max_arr, TimerFrequencyPlan *plan)
{
    const uint64_t arr_limit = (uint64_t)max_arr + 1U; /* ARR+1，避免 32 位溢出。 */
    TimerFrequencyPlan best = {0}; /* 尚未提交的最优候选。 */
    double best_error = DBL_MAX; /* 当前最小绝对误差，Hz。 */
    if (plan == NULL || !isfinite(clock_hz) || !isfinite(requested_hz) ||
        clock_hz <= 0.0 || requested_hz <= 0.0 || requested_hz > clock_hz ||
        requested_hz < clock_hz / (65536.0 * (double)arr_limit)) { return false; }
    for (uint32_t p = 1U; p <= 65536U; ++p)
    {
        const double ideal = clock_hz / requested_hz / p; /* 理想 ARR+1。 */
        uint64_t lower = ideal < 1.0 ? 1U :
            (ideal >= (double)arr_limit ? arr_limit : (uint64_t)ideal); /* 下邻值。 */
        for (uint32_t side = 0U; side < 2U; ++side)
        {
            const uint64_t a = lower + side; /* 上下两个邻值足以覆盖此 PSC 最优解。 */
            if (a > arr_limit) { continue; }
            const uint64_t ticks = (uint64_t)p * a; /* 总分频，不用 int 乘法。 */
            const double actual = clock_hz / (double)ticks; /* 实际更新率。 */
            const double error = fabs(actual - requested_hz); /* 绝对频差。 */
            if (error < best_error)
            {
                best_error = error;
                best = (TimerFrequencyPlan){p - 1U, (uint32_t)(a - 1U), ticks,
                    actual, actual - requested_hz, (actual / requested_hz - 1.0) * 1.0e6};
            }
        }
        if (best_error == 0.0) { break; }
    }
    *plan = best;
    return true;
}

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
                                     TimerFrequencyPlan *plan)
{
    const uint64_t arr_limit = (uint64_t)max_arr + 1U; /* ARR+1，避免上限溢出。 */
    if (plan == NULL || !isfinite(clock_hz) || !isfinite(requested_hz) ||
        clock_hz <= 0.0 || requested_hz <= 0.0 || divider == 0U ||
        (uint64_t)divider > 65536ULL * arr_limit) {
        return false;
    }
    for (uint32_t prescaler_plus_one = 1U;
         prescaler_plus_one <= 65536U; ++prescaler_plus_one)
    {
        if (divider % prescaler_plus_one != 0U) { continue; }
        const uint32_t autoreload_plus_one = divider / prescaler_plus_one;
        if ((uint64_t)autoreload_plus_one > arr_limit) { continue; }
        const double actual = clock_hz / (double)divider;
        *plan = (TimerFrequencyPlan){
            .prescaler = prescaler_plus_one - 1U,
            .autoreload = autoreload_plus_one - 1U,
            .period_ticks = divider,
            .actual_hz = actual,
            .error_hz = actual - requested_hz,
            .error_ppm = (actual / requested_hz - 1.0) * 1.0e6,
        };
        return true;
    }
    return false;
}
