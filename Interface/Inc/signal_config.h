/** @file signal_config.h
 * @brief Interface 层：项目采样配置与内部输出块契约，不暴露 STM32 类型。
 */
#ifndef SIGNAL_CONFIG_H
#define SIGNAL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* ADC 是项目采样率入口；DAC 最终更新率固定由同一 TIM1 主触发及
 * dac_upsampling_factor 派生，接口不提供第二个独立 DAC 频率。 */
typedef struct
{
    double adc_sample_rate_hz; /**< 请求 ADC 采样率，Hz；实际值由定时器返回。 */
    uint32_t dac_upsampling_factor; /**< 正整数倍数；DAC 最终率由 ADC 率 × 此值派生。 */
    bool dac_output_enabled; /**< true 输出到 DAC 引脚；false 仅生成内部数据。 */
    uint32_t adc_block_samples; /**< 每个输入块的点数，同时决定处理截止时间。 */
    uint32_t reference_sample_offset; /**< UINT32_MAX 按当前硬件流水线自动设置。 */
    uint16_t *output_buffers[2]; /**< 可选双缓冲地址；同时为 NULL 时使用 BSP 默认存储。 */
    uint32_t output_capacity_samples; /**< 自定义缓冲区各自容量，单位 uint16_t 点数，NULL 使用 BSP 默认。 */
} SignalStreamConfig;

typedef struct
{
    double adc_sample_rate_hz; /**< 实际 ADC 采样率，Hz。 */
    double dac_base_sample_rate_hz; /**< 实际 DAC 基率，等于实际 ADC 采样率，Hz。 */
    double dac_output_sample_rate_hz; /**< 实际 DAC 最终更新率，主定时器 TIM1，Hz。 */
    double adc_error_ppm; /**< ADC 相对请求采样率的误差，ppm。 */
    double dac_error_ppm; /**< DAC 主定时器相对派生目标率的误差，ppm。 */
    double block_period_s; /**< 输入输出共同块周期，秒。 */
    uint32_t output_block_samples; /**< 实际每个输出块的点数。 */
    uint32_t output_samples_per_input_sample; /**< TIM8 对 TIM1 事件的总分频。 */
    bool dac_output_enabled; /**< 当前是否输出到 DAC 引脚。 */
} SignalStreamInfo;

typedef struct
{
    const uint16_t *samples; /**< 右对齐 12 位码，只读；回调内有效，禁止写入。 */
    uint32_t sample_count; /**< 本块有效点数，不是缓冲区总容量。 */
    double sample_rate_hz; /**< 升采样后的实际最终采样率，Hz。 */
    uint64_t sequence; /**< 渲染块序号；每次启动重新从 0 开始。 */
} SignalOutputBlock;

/* 回调在主循环预填或输出/输入中断重填时执行，必须有界、非阻塞。
 * block 及 samples 仅在回调期间有效；需要长期保存时复制到调用者存储。
 * 回调不得调用启停、重配置或修改 NCO 参数；context 生命周期覆盖整个运行期。 */
typedef void (*SignalOutputReadyCallback)(void *context, const SignalOutputBlock *block);

#endif /* SIGNAL_CONFIG_H */
