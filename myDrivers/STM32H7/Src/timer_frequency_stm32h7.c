/** @file timer_frequency_stm32h7.c
 * @brief Driver 层：定时器频率应用，板卡绑定由 BSP/Interface 提供。
 */
#include "timer_frequency_stm32h7.h"
#include <stddef.h>

/**
 * @brief 写入已停止定时器的 PSC/ARR，并让预装载寄存器立即生效。
 * @param timer 已初始化、停止的定时器句柄。
 * @param plan 与定时器位宽相符的求解计划。
 * @retval true 已应用；false 参数或运行状态非法。
 * @note 主循环调用；不启动计数。生成 UG 前下游采集和输出必须停止。
 */
bool TimerFrequency_Apply(TIM_HandleTypeDef *timer, const TimerFrequencyPlan *plan)
{
    if (timer == NULL || timer->Instance == NULL || plan == NULL ||
        (timer->Instance->CR1 & TIM_CR1_CEN) != 0U ||
        timer->Init.CounterMode != TIM_COUNTERMODE_UP || plan->prescaler > UINT16_MAX ||
        (!IS_TIM_32B_COUNTER_INSTANCE(timer->Instance) && plan->autoreload > UINT16_MAX))
    { return false; }
    timer->Init.Prescaler = plan->prescaler;
    timer->Init.Period = plan->autoreload;
    timer->Init.RepetitionCounter = 0U;
    __HAL_TIM_SET_PRESCALER(timer, plan->prescaler);
    __HAL_TIM_SET_AUTORELOAD(timer, plan->autoreload);
    if (IS_TIM_REPETITION_COUNTER_INSTANCE(timer->Instance)) { timer->Instance->RCR = 0U; }
    timer->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(timer, 0U);
    __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
    return true;
}

/**
 * @brief 求最近更新率并应用到停止的定时器，返回实际频率和误差。
 * @param timer 已初始化的上计数定时器。
 * @param input_clock_hz 该时基实际输入时钟，Hz。
 * @param requested_hz 目标更新率，Hz。
 * @param actual 可选输出，成功时获得实际分频计划。
 * @retval true 成功；false 目标不可表示或定时器仍在运行。
 * @note 主循环调用，无全局状态；不代替 PWM 占空比配置或外设启动。
 */
bool TimerFrequency_Set(TIM_HandleTypeDef *timer, double input_clock_hz,
                        double requested_hz, TimerFrequencyPlan *actual)
{
    TimerFrequencyPlan plan; /* 局部候选，失败不发布。 */
    if (timer == NULL || timer->Instance == NULL ||
        !TimerFrequency_Find(input_clock_hz, requested_hz,
            IS_TIM_32B_COUNTER_INSTANCE(timer->Instance) ? UINT32_MAX : UINT16_MAX, &plan) ||
        !TimerFrequency_Apply(timer, &plan)) { return false; }
    if (actual != NULL) { *actual = plan; }
    return true;
}
