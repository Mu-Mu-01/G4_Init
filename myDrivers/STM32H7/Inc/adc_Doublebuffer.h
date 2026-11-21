/* HAL/Driver 层：STM32H7 双缓冲扩展，通过句柄接收板级绑定。 */
/**
 ******************************************************************************
 * @file    adc_Doublebuffer.h
 * @brief   ADC DMA 双缓冲启动函数头文件
 ******************************************************************************
 * @attention
 * 此文件为标准的 HAL ADC DMA API 扩展双缓冲支持。提供两种启动模式：
 *   - 中断模式：使用 DMA 中断
 *   - 轮询模式：使用无中断的 DMA
 *
 * 两个函数内部都使用 HAL_DMAEx_MultiBufferStart(_IT)，
 * 使能两个内存缓冲区之间的自动乒乓切换，用于 ADC 数据接收。
 *
 * 与 DAC 双缓冲的关键区别：ADC DMA 方向为 PERIPH_TO_MEMORY，
 * 因此第二个缓冲区是目标 memory1。
 ******************************************************************************
 */

#ifndef ADC_DOUBLEBUFFER_H
#define ADC_DOUBLEBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 ----------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* 导出函数 ----------------------------------------------------------------*/

/**
 * @brief  以中断模式启动带双缓冲的 ADC DMA 传输。
 * @note   使用 HAL_DMAEx_MultiBufferStart_IT() 实现自动乒乓切换。
 *         ADC 持续采样，DMA 在 pData（memory0）和 pSecondData（memory1）
 *         之间交替。
 * @param  hadc:       指向 ADC_HandleTypeDef 结构的指针
 * @param  pData:      主缓冲区（memory0）数据指针，作为 DMA 目标
 * @param  pSecondData: 次缓冲区（memory1）数据指针，作为 DMA 目标
 * @param  Length:     每个缓冲区的数据项数（每个缓冲区接收 Length 个采样）
 * @retval HAL 状态
 */
HAL_StatusTypeDef HAL_ADC_Start_DMAEx_IT(ADC_HandleTypeDef *hadc,
                                         uint32_t *pData,
                                         uint32_t *pSecondData,
                                         uint32_t Length);

/**
 * @brief  以轮询模式（无中断）启动带双缓冲的 ADC DMA 传输。
 * @note   使用 HAL_DMAEx_MultiBufferStart() 实现无中断回调的自动乒乓切换。
 * @param  hadc:       指向 ADC_HandleTypeDef 结构的指针
 * @param  pData:      主缓冲区（memory0）数据指针，作为 DMA 目标
 * @param  pSecondData: 次缓冲区（memory1）数据指针，作为 DMA 目标
 * @param  Length:     每个缓冲区的数据项数（每个缓冲区接收 Length 个采样）
 * @retval HAL 状态
 */
HAL_StatusTypeDef HAL_ADC_Start_DMAEx(ADC_HandleTypeDef *hadc,
                                      uint32_t *pData,
                                      uint32_t *pSecondData,
                                      uint32_t Length);

#ifdef __cplusplus
}
#endif

#endif /* ADC_DOUBLEBUFFER_H */
