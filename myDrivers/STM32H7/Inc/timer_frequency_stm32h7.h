/** @file timer_frequency_stm32h7.h
 * @brief Driver 层：将已计算的更新频率计划应用到停止的 STM32H7 定时器。
 */
#ifndef TIMER_FREQUENCY_STM32H7_H
#define TIMER_FREQUENCY_STM32H7_H
#include "timer_frequency.h"
#include "stm32h7xx_hal.h"
/* 上计数、RCR=0；必须停止并且 ADC/DAC DMA 未装载，UG 可能产生 TRGO。
 * input_clock_hz 支持内部时钟或外部计数时钟；不在驱动内写死板级频率。 */
bool TimerFrequency_Set(TIM_HandleTypeDef *timer, double input_clock_hz,
                        double requested_hz, TimerFrequencyPlan *actual);
bool TimerFrequency_Apply(TIM_HandleTypeDef *timer, const TimerFrequencyPlan *plan);
#endif
