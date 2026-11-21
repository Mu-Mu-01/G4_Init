/* G474 板级采样配置：应用启动参数与 BSP 静态缓冲区共用同一组值。 */
#ifndef BSP_SIGNAL_SETTINGS_H
#define BSP_SIGNAL_SETTINGS_H

#define DPLL_ADC_SAMPLE_RATE_HZ       100000.0 /* ADC 目标采样率，Hz。 */
#define DPLL_DAC_UPSAMPLING_FACTOR    20U      /* 每个输入样本对应的 DAC 更新次数。 */
#define DPLL_ADC_BLOCK_SAMPLES        512U     /* 每个输入半块的 ADC 样本数。 */

#endif
