/* Interface 平台适配层：绑定 G4 HAL 句柄与循环 DMA 使用的静态存储。 */
#ifndef SIGNAL_PORT_STM32G4_H
#define SIGNAL_PORT_STM32G4_H

#include "signal_port.h"
#include "stm32g4xx_hal.h"

typedef struct
{
    ADC_HandleTypeDef *adc;               /* 外部上升沿触发的单通道 ADC。 */
    DAC_HandleTypeDef *dac;               /* 通道 1 使用 12 位右对齐双数据模式。 */
    TIM_HandleTypeDef *input_timer;       /* 外部时钟模式，整数分频后触发 ADC。 */
    TIM_HandleTypeDef *output_timer;      /* 内部时钟模式，触发 DAC 和输入定时器。 */
    uint32_t timer_clock_hz;              /* 主定时器内核时钟，Hz。 */
    uint16_t *adc_ring;                   /* 两个输入半块，可容纳 2 * input_capacity_samples。 */
    uint16_t *dac_ring;                   /* 字对齐的输出环，共 2 * output_capacity_samples 个半字。 */
    uint32_t input_capacity_samples;     /* ADC 每半块容量，点。 */
    uint32_t output_capacity_samples;    /* DAC 每半块容量，点。 */
} STM32G4_SignalConfig;

const SignalPort *STM32G4_SignalPort_Init(const STM32G4_SignalConfig *config);

#endif
