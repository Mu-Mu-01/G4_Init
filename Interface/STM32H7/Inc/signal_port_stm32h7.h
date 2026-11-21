/** @file signal_port_stm32h7.h
 * @brief Interface 平台适配私有入口：仅 BSP 可依赖本头文件中的 HAL 类型。
 */
#ifndef SIGNAL_PORT_STM32H7_H
#define SIGNAL_PORT_STM32H7_H

#include "signal_port.h"
#include "stm32h7xx_hal.h"

typedef struct
{
    ADC_HandleTypeDef *adc;       /* 已由 CubeMX 初始化的单路输入 */
    DAC_HandleTypeDef *dac;       /* 已由 CubeMX 初始化的输出，仅支持通道 1 */
    TIM_HandleTypeDef *input_timer;  /* 从定时器，计数公共输出触发 */
    TIM_HandleTypeDef *output_timer; /* 主定时器，启动前必须停止 */
    uint32_t dac_channel;         /* 必须为 DAC_CHANNEL_1 */
    uint16_t *input_buffers[2];   /* BSP 持有的独立输入块，32 字节对齐 */
    uint16_t *output_buffers[2];  /* BSP 持有的独立输出块，32 字节对齐 */
    uint32_t input_block_samples; /* 每块输入样本数，字节长度为 32 的整数倍 */
    uint32_t output_block_samples;/* 每块输出样本数，字节长度为 32 的整数倍 */
    uint32_t input_rate_hz;       /* 端口初始 ADC 速率占位，运行时由共同主时钟计划覆盖，Hz */
    uint32_t output_rate_hz;      /* 端口初始 TIM1 速率占位，由 ADC 速率×升采样倍数派生，Hz */
    uint32_t reference_sample_offset; /* 输入首样本在输出插值组中的索引 */
    uint32_t timer_clock_hz; /* TIM1 实际内核时钟，Hz，由 BSP 从 RCC 读取。 */
    uint32_t input_capacity_samples; /* 每个输入数组的静态容量。 */
    uint32_t output_capacity_samples; /* 每个默认输出数组的静态容量。 */
    double actual_input_rate_hz; /* 当前 TIM8 实际更新率，Hz。 */
    double actual_output_rate_hz; /* 当前 TIM1 实际更新率，Hz。 */
    uint32_t output_samples_per_input_sample; /* TIM8 总分频。 */
    bool output_enabled; /* false 不启动 DAC DMA，保留同步触发时间轴。 */
} STM32H7_SignalConfig;

/* 主循环中调用，复制 config，借用所有 HAL 句柄/缓冲区。
 * BSP 保证缓冲区在 DMA 可访问 SRAM，互不重叠且不共享 cache line。
 * 本适配仅支持单个端口，独占相应 HAL 弱回调；运行中拒绝重新绑定。
 * 返回运行期有效端口；NULL 表示配置或状态不合法，不启动任何外设。 */
const SignalPort *STM32H7_SignalPort_Init(const STM32H7_SignalConfig *config);

#endif /* SIGNAL_PORT_STM32H7_H */
