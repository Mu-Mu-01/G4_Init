/* HAL/Driver 层：STM32H7 双缓冲扩展，通过句柄接收板级绑定。 */
/**
 ******************************************************************************
 * @file    dac_Doublebuffer.h
 * @brief   DAC DMA 双缓冲启动函数头文件
 ******************************************************************************
 * @attention
 * 此文件为标准的 HAL DAC DMA API 扩展双缓冲支持。提供两种启动模式：
 *   - 中断模式：使用 DMA 中断 + DAC 下溢中断
 *   - 轮询模式：使用无中断的 DMA
 *
 * 两个函数内部都使用 HAL_DMAEx_MultiBufferStart(_IT)，
 * 使能两个内存缓冲区之间的自动乒乓切换。
 ******************************************************************************
 */

#ifndef DAC_DOUBLEBUFFER_H
#define DAC_DOUBLEBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 ----------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* 导出函数 ----------------------------------------------------------------*/

/**
 * @brief  以中断模式启动带双缓冲的 DAC DMA 传输。
 * @param  hdac:      指向 DAC_HandleTypeDef 结构的指针
 * @param  Channel:   DAC 通道（DAC_CHANNEL_1 或 DAC_CHANNEL_2）
 * @param  pData:     主缓冲区（memory0）数据指针
 * @param  pSecondData: 次缓冲区（memory1）数据指针
 * @param  Length:    每个缓冲区的数据项数
 * @param  Alignment: DAC 数据对齐（DAC_ALIGN_8B_R / DAC_ALIGN_12B_L / DAC_ALIGN_12B_R）
 * @retval HAL 状态
 */
HAL_StatusTypeDef HAL_DAC_Start_DMAEx_IT(DAC_HandleTypeDef *hdac, uint32_t Channel,
                                         const uint32_t *pData, const uint32_t *pSecondData,
                                         uint32_t Length, uint32_t Alignment);

/**
 * @brief  以轮询模式（无中断）启动带双缓冲的 DAC DMA 传输。
 * @param  hdac:      指向 DAC_HandleTypeDef 结构的指针
 * @param  Channel:   DAC 通道（DAC_CHANNEL_1 或 DAC_CHANNEL_2）
 * @param  pData:     主缓冲区（memory0）数据指针
 * @param  pSecondData: 次缓冲区（memory1）数据指针
 * @param  Length:    每个缓冲区的数据项数
 * @param  Alignment: DAC 数据对齐（DAC_ALIGN_8B_R / DAC_ALIGN_12B_L / DAC_ALIGN_12B_R）
 * @retval HAL 状态
 */
HAL_StatusTypeDef HAL_DAC_Start_DMAEx(DAC_HandleTypeDef *hdac, uint32_t Channel,
                                      const uint32_t *pData, const uint32_t *pSecondData,
                                      uint32_t Length, uint32_t Alignment);

#ifdef __cplusplus
}
#endif

#endif /* DAC_DOUBLEBUFFER_H */
