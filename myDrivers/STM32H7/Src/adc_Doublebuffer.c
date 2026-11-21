/* HAL/Driver 层：STM32H7 双缓冲扩展，通过句柄接收板级绑定。 */
/**
 ******************************************************************************
 * @file    adc_Doublebuffer.c
 * @brief   ADC DMA 双缓冲启动函数实现
 ******************************************************************************
 * @attention
 * 此文件提供两个函数，为标准的 HAL ADC DMA API 扩展双缓冲（乒乓）支持：
 *   - HAL_ADC_Start_DMAEx_IT():  中断模式，注册 DMA 回调
 *   - HAL_ADC_Start_DMAEx():     轮询模式，无中断
 *
 * 与原始 HAL_ADC_Start_DMA() 的核心区别在于，将 HAL_DMA_Start_IT()
 * 替换为 HAL_DMAEx_MultiBufferStart_IT()（轮询版本则为
 * HAL_DMAEx_MultiBufferStart），从而配置 DMA 在两个内存缓冲区之间自动切换。
 *
 * ADC DMA 方向为 PERIPH_TO_MEMORY：
 *   - SrcAddress  = &hadc->Instance->DR  （ADC 数据寄存器，固定）
 *   - DstAddress  = pData                 （memory0，主缓冲区）
 *   - SecondMemAddress = pSecondData      （memory1，次缓冲区）
 *
 * Created by Rui on 2026/7/26.
 ******************************************************************************
 */

#include "adc_Doublebuffer.h"

/**
 * @brief  以中断模式启动带双缓冲的 ADC DMA 传输。
 * @note   此函数在结构上与原始的 HAL_ADC_Start_DMA() 相同，
 *         只是使用 HAL_DMAEx_MultiBufferStart_IT() 来使能乒乓缓冲。
 *         DMA 中断和 ADC 溢出中断均被使能。
 * @param  hadc:       ADC 句柄指针
 * @param  pData:      主缓冲区（memory0）
 * @param  pSecondData: 次缓冲区（memory1）
 * @param  Length:     每个缓冲区的采样数
 * @retval HAL 状态
 */
HAL_StatusTypeDef HAL_ADC_Start_DMAEx_IT(ADC_HandleTypeDef *hadc,
                                         uint32_t *pData,
                                         uint32_t *pSecondData,
                                         uint32_t Length)
{
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));

  /* 检查参数 */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* 若无正在进行的转换，则执行 ADC 使能和转换启动 */
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
    /* 锁定处理 */
    __HAL_LOCK(hadc);

    /* 确保未使能多模式规则转换。   */
    /* 否则必须使用专用 API HAL_ADCEx_MultiModeStart_DMA()。  */
    if ((tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
       )
    {
      /* 使能 ADC 外设 */
      tmp_hal_status = ADC_Enable(hadc);

      /* 若 ADC 已实际使能则启动转换 */
      if (tmp_hal_status == HAL_OK)
      {
        /* 设置 ADC 状态                                                       */
        /* - 清除与规则组转换结果相关的状态位域                                */
        /* - 设置与规则操作相关的状态位域                                      */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
                          HAL_ADC_STATE_REG_BUSY);

        /* 复位 HAL_ADC_STATE_MULTIMODE_SLAVE 位
          - 若 ADC 实例为主机，或多模式特性不可用
          - 若多模式设置被禁用（ADC 实例为独立模式的从机） */
        if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
            || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
           )
        {
          CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
        }

        /* 检查 ADC 注入组是否正在进行转换 */
        if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) != 0UL)
        {
          /* 仅复位与规则转换相关的 ADC 错误码字段 */
          CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
        }
        else
        {
          /* 复位所有 ADC 错误码字段 */
          ADC_CLEAR_ERRORCODE(hadc);
        }

        /* 设置 DMA 传输完成回调 */
        hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

        /* DMA 双缓冲引擎通过其专用回调槽上报 Memory 1 完成。
         * 在应用程序用其缓冲区专用回调替换之前，
         * 为两个内存目标注册相同的 HAL 状态更新。 */
        hadc->DMA_Handle->XferM1CpltCallback = ADC_DMAConvCplt;

        /* 设置 DMA 半传输完成回调 */
        hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

        hadc->DMA_Handle->XferM1HalfCpltCallback = ADC_DMAHalfConvCplt;

        /* 设置 DMA 错误回调 */
        hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;

        /* 管理 ADC 和 DMA 启动：ADC 溢出中断、DMA 启动、   */
        /* ADC 启动（软件启动时）：                         */

        /* 清除规则组转换标志和溢出标志                      */
        /* （确保不会因之前可能的 ADC 操作而处于未知状态）   */
        __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

        /* 解锁处理 */
        /* 在启动 ADC 转换之前解锁：以防可能的中断，        */
        /* 让处理交由 ADC IRQ Handler 完成。               */
        __HAL_UNLOCK(hadc);

        /* 使用 DMA 时，即使 hadc->Init.Overrun 被设置为
           ADC_OVR_DATA_OVERWRITTEN，溢出事件也始终被视为错误。
           因此，使能 ADC_IT_OVR。 */
        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

        /* 使能 ADC DMA 模式*/
#if defined(ADC_VER_V5_V90)
        if (hadc->Instance == ADC3)
        {
          LL_ADC_REG_SetDMATransferMode(hadc->Instance, ADC3_CFGR_DMACONTREQ((uint32_t)hadc->Init.DMAContinuousRequests));
          LL_ADC_EnableDMAReq(hadc->Instance);
        }
        else
        {
          LL_ADC_REG_SetDataTransferMode(hadc->Instance, ADC_CFGR_DMACONTREQ((uint32_t)hadc->Init.ConversionDataManagement));
        }

#else
        LL_ADC_REG_SetDataTransferMode(hadc->Instance, (uint32_t)hadc->Init.ConversionDataManagement);
#endif

        /* 以双缓冲模式（中断）启动 DMA 通道
         * ADC DMA 方向：PERIPH_TO_MEMORY
         *   SrcAddress       = &hadc->Instance->DR  （ADC 数据寄存器）
         *   DstAddress       = pData                 （memory0）
         *   SecondMemAddress = pSecondData           （memory1）
         */
        tmp_hal_status = HAL_DMAEx_MultiBufferStart_IT(hadc->DMA_Handle,
                                                       (uint32_t)&hadc->Instance->DR,
                                                       (uint32_t)pData,
                                                       (uint32_t)pSecondData,
                                                       Length);

        /* 使能规则组转换。                                              */
        /* 若选择了软件启动，转换立即开始。                              */
        /* 若选择了外部触发，转换将在下一个触发事件时开始。              */
        /* 启动 ADC 组规则转换 */
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
        /* 解锁处理 */
        __HAL_UNLOCK(hadc);
      }

    }
    else
    {
      tmp_hal_status = HAL_ERROR;
      /* 解锁处理 */
      __HAL_UNLOCK(hadc);
    }
  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

  /* 返回函数状态 */
  return tmp_hal_status;
}

/**
 * @brief  以轮询模式启动带双缓冲的 ADC DMA 传输。
 * @note   此函数在结构上与原始的 HAL_ADC_Start_DMA() 相同，
 *         只是使用 HAL_DMAEx_MultiBufferStart()（无中断）来使能乒乓缓冲。
 *         不注册 DMA 回调。
 * @param  hadc:       ADC 句柄指针
 * @param  pData:      主缓冲区（memory0）
 * @param  pSecondData: 次缓冲区（memory1）
 * @param  Length:     每个缓冲区的采样数
 * @retval HAL 状态
 */
HAL_StatusTypeDef HAL_ADC_Start_DMAEx(ADC_HandleTypeDef *hadc,
                                      uint32_t *pData,
                                      uint32_t *pSecondData,
                                      uint32_t Length)
{
  HAL_StatusTypeDef tmp_hal_status;
  uint32_t tmp_multimode_config = LL_ADC_GetMultimode(__LL_ADC_COMMON_INSTANCE(hadc->Instance));

  /* 检查参数 */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* 若无正在进行的转换，则执行 ADC 使能和转换启动 */
  if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
  {
    /* 锁定处理 */
    __HAL_LOCK(hadc);

    /* 确保未使能多模式规则转换。   */
    /* 否则必须使用专用 API HAL_ADCEx_MultiModeStart_DMA()。  */
    if ((tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_SIMULT)
        || (tmp_multimode_config == LL_ADC_MULTI_DUAL_INJ_ALTERN)
       )
    {
      /* 使能 ADC 外设 */
      tmp_hal_status = ADC_Enable(hadc);

      /* 若 ADC 已实际使能则启动转换 */
      if (tmp_hal_status == HAL_OK)
      {
        /* 设置 ADC 状态                                                       */
        /* - 清除与规则组转换结果相关的状态位域                                */
        /* - 设置与规则操作相关的状态位域                                      */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
                          HAL_ADC_STATE_REG_BUSY);

        /* 复位 HAL_ADC_STATE_MULTIMODE_SLAVE 位
          - 若 ADC 实例为主机，或多模式特性不可用
          - 若多模式设置被禁用（ADC 实例为独立模式的从机） */
        if ((__LL_ADC_MULTI_INSTANCE_MASTER(hadc->Instance) == hadc->Instance)
            || (tmp_multimode_config == LL_ADC_MULTI_INDEPENDENT)
           )
        {
          CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
        }

        /* 检查 ADC 注入组是否正在进行转换 */
        if ((hadc->State & HAL_ADC_STATE_INJ_BUSY) != 0UL)
        {
          /* 仅复位与规则转换相关的 ADC 错误码字段 */
          CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
        }
        else
        {
          /* 复位所有 ADC 错误码字段 */
          ADC_CLEAR_ERRORCODE(hadc);
        }

        /* 管理 ADC 和 DMA 启动：ADC 溢出中断、DMA 启动、   */
        /* ADC 启动（软件启动时）：                         */

        /* 清除规则组转换标志和溢出标志                      */
        /* （确保不会因之前可能的 ADC 操作而处于未知状态）   */
        __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

        /* 解锁处理 */
        /* 在启动 ADC 转换之前解锁：以防可能的中断，        */
        /* 让处理交由 ADC IRQ Handler 完成。               */
        __HAL_UNLOCK(hadc);

        /* 使用 DMA 时，即使 hadc->Init.Overrun 被设置为
           ADC_OVR_DATA_OVERWRITTEN，溢出事件也始终被视为错误。
           因此，使能 ADC_IT_OVR。 */
        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

        /* 使能 ADC DMA 模式*/
#if defined(ADC_VER_V5_V90)
        if (hadc->Instance == ADC3)
        {
          LL_ADC_REG_SetDMATransferMode(hadc->Instance, ADC3_CFGR_DMACONTREQ((uint32_t)hadc->Init.DMAContinuousRequests));
          LL_ADC_EnableDMAReq(hadc->Instance);
        }
        else
        {
          LL_ADC_REG_SetDataTransferMode(hadc->Instance, ADC_CFGR_DMACONTREQ((uint32_t)hadc->Init.ConversionDataManagement));
        }

#else
        LL_ADC_REG_SetDataTransferMode(hadc->Instance, (uint32_t)hadc->Init.ConversionDataManagement);
#endif

        /* 以双缓冲模式（轮询，无中断）启动 DMA 通道
         * ADC DMA 方向：PERIPH_TO_MEMORY
         *   SrcAddress       = &hadc->Instance->DR  （ADC 数据寄存器）
         *   DstAddress       = pData                 （memory0）
         *   SecondMemAddress = pSecondData           （memory1）
         */
        tmp_hal_status = HAL_DMAEx_MultiBufferStart(hadc->DMA_Handle,
                                                    (uint32_t)&hadc->Instance->DR,
                                                    (uint32_t)pData,
                                                    (uint32_t)pSecondData,
                                                    Length);

        /* 使能规则组转换。                                              */
        /* 若选择了软件启动，转换立即开始。                              */
        /* 若选择了外部触发，转换将在下一个触发事件时开始。              */
        /* 启动 ADC 组规则转换 */
        LL_ADC_REG_StartConversion(hadc->Instance);
      }
      else
      {
        /* 解锁处理 */
        __HAL_UNLOCK(hadc);
      }

    }
    else
    {
      tmp_hal_status = HAL_ERROR;
      /* 解锁处理 */
      __HAL_UNLOCK(hadc);
    }
  }
  else
  {
    tmp_hal_status = HAL_BUSY;
  }

  /* 返回函数状态 */
  return tmp_hal_status;
}
