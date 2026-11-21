/* HAL/Driver 层：STM32H7 双缓冲扩展，通过句柄接收板级绑定。 */
//
// Created by Rui on 2026/7/26.
//
#include "dac_Doublebuffer.h"

HAL_StatusTypeDef HAL_DAC_Start_DMAEx_IT(DAC_HandleTypeDef *hdac, uint32_t Channel, const uint32_t *pData, const uint32_t *pSecondData, uint32_t Length,
                                    uint32_t Alignment)
{
  HAL_StatusTypeDef status;
  uint32_t tmpreg;

  /* 检查 DAC 外设句柄 */
  if (hdac == NULL)
  {
    return HAL_ERROR;
  }

  /* 检查参数 */
  assert_param(IS_DAC_CHANNEL(Channel));
  assert_param(IS_DAC_ALIGN(Alignment));

  /* 锁定处理 */
  __HAL_LOCK(hdac);

  /* 改变 DAC 状态 */
  hdac->State = HAL_DAC_STATE_BUSY;

  if (Channel == DAC_CHANNEL_1)
  {
    /* 设置通道 1 的 DMA 传输完成回调 */
    hdac->DMA_Handle1->XferCpltCallback = DAC_DMAConvCpltCh1;

    /* 设置通道 1 的 DMA 半传输完成回调 */
    hdac->DMA_Handle1->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh1;

    /* 设置通道 1 的 DMA 错误回调 */
    hdac->DMA_Handle1->XferErrorCallback = DAC_DMAErrorCh1;

    /* 使能所选 DAC 通道 1 的 DMA 请求 */
    SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN1);

    /* 使用通道 1 的情况 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* 获取 DHR12R1 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12R1;
        break;
      case DAC_ALIGN_12B_L:
        /* 获取 DHR12L1 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12L1;
        break;
      default: /* case DAC_ALIGN_8B_R */
        /* 获取 DHR8R1 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR8R1;
        break;
    }
  }

  else
  {
    /* 设置通道 2 的 DMA 传输完成回调 */
    hdac->DMA_Handle2->XferCpltCallback = DAC_DMAConvCpltCh2;

    /* 设置通道 2 的 DMA 半传输完成回调 */
    hdac->DMA_Handle2->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh2;

    /* 设置通道 2 的 DMA 错误回调 */
    hdac->DMA_Handle2->XferErrorCallback = DAC_DMAErrorCh2;

    /* 使能所选 DAC 通道 2 的 DMA 请求 */
    SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN2);

    /* 使用通道 2 的情况 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* 获取 DHR12R2 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12R2;
        break;
      case DAC_ALIGN_12B_L:
        /* 获取 DHR12L2 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12L2;
        break;
      default: /* case DAC_ALIGN_8B_R */
        /* 获取 DHR8R2 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR8R2;
        break;
    }
  }

  if (Channel == DAC_CHANNEL_1)
  {
    /* 使能 DAC DMA 下溢中断 */
    __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR1);

    /* 以双缓冲模式使能 DMA 流 */
    status = HAL_DMAEx_MultiBufferStart_IT(hdac->DMA_Handle1, (uint32_t)pData, tmpreg, (uint32_t)pSecondData, Length);
  }

  else
  {
    /* 使能 DAC DMA 下溢中断 */
    __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR2);

    /* 以双缓冲模式使能 DMA 流 */
    status = HAL_DMAEx_MultiBufferStart_IT(hdac->DMA_Handle2, (uint32_t)pData, tmpreg, (uint32_t)pSecondData, Length);
  }


  /* 解锁处理 */
  __HAL_UNLOCK(hdac);

  if (status == HAL_OK)
  {
    /* 使能外设 */
    __HAL_DAC_ENABLE(hdac, Channel);
  }
  else
  {
    hdac->ErrorCode |= HAL_DAC_ERROR_DMA;
  }

  /* 返回函数状态 */
  return status;
}

HAL_StatusTypeDef HAL_DAC_Start_DMAEx(DAC_HandleTypeDef *hdac, uint32_t Channel, const uint32_t *pData, const uint32_t *pSecondData, uint32_t Length,
                                    uint32_t Alignment)
{
  HAL_StatusTypeDef status;
  uint32_t tmpreg;

  /* 检查 DAC 外设句柄 */
  if (hdac == NULL)
  {
    return HAL_ERROR;
  }

  /* 检查参数 */
  assert_param(IS_DAC_CHANNEL(Channel));
  assert_param(IS_DAC_ALIGN(Alignment));

  /* 锁定处理 */
  __HAL_LOCK(hdac);

  /* 改变 DAC 状态 */
  hdac->State = HAL_DAC_STATE_BUSY;

  if (Channel == DAC_CHANNEL_1)
  {
    /* 使能所选 DAC 通道 1 的 DMA 请求 */
    SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN1);

    /* 使用通道 1 的情况 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* 获取 DHR12R1 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12R1;
        break;
      case DAC_ALIGN_12B_L:
        /* 获取 DHR12L1 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12L1;
        break;
      default: /* case DAC_ALIGN_8B_R */
        /* 获取 DHR8R1 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR8R1;
        break;
    }
  }

  else
  {
    /* 使能所选 DAC 通道 2 的 DMA 请求 */
    SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN2);

    /* 使用通道 2 的情况 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* 获取 DHR12R2 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12R2;
        break;
      case DAC_ALIGN_12B_L:
        /* 获取 DHR12L2 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR12L2;
        break;
      default: /* case DAC_ALIGN_8B_R */
        /* 获取 DHR8R2 地址 */
        tmpreg = (uint32_t)&hdac->Instance->DHR8R2;
        break;
    }
  }

  if (Channel == DAC_CHANNEL_1)
  {
    /* 以双缓冲模式（轮询，无中断）使能 DMA 流 */
    status = HAL_DMAEx_MultiBufferStart(hdac->DMA_Handle1, (uint32_t)pData, tmpreg, (uint32_t)pSecondData, Length);
  }

  else
  {
    /* 以双缓冲模式（轮询，无中断）使能 DMA 流 */
    status = HAL_DMAEx_MultiBufferStart(hdac->DMA_Handle2, (uint32_t)pData, tmpreg, (uint32_t)pSecondData, Length);
  }


  /* 解锁处理 */
  __HAL_UNLOCK(hdac);

  if (status == HAL_OK)
  {
    /* 使能外设 */
    __HAL_DAC_ENABLE(hdac, Channel);
  }
  else
  {
    hdac->ErrorCode |= HAL_DAC_ERROR_DMA;
  }

  /* 返回函数状态 */
  return status;
}
