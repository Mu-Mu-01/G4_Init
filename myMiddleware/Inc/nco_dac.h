/* myMiddleware 层：波形合成与参考元数据服务，公开接口不暴露 HAL。 */
#ifndef NCO_DAC_H
#define NCO_DAC_H

#include "signal_port.h"
#include "nco_waveform.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NCO_DAC_NCO_SAMPLE_RATE_HZ       1000000U  /* 默认 ADC/NCO 基准率，Hz；运行值来自端口。 */
#define NCO_DAC_SAMPLE_RATE_HZ           11000000U /* 默认 ADC 率×升采样倍数得到的最终率，Hz。 */
#define NCO_DAC_INTERPOLATION_FACTOR     11U      /* 默认输出/输入速率比，不限制运行配置。 */
#define NCO_DAC_BASE_BLOCK_SAMPLE_COUNT  512U     /* 默认每块基础采样点数。 */
/* 默认单个输出缓冲区长度，单位 uint16_t 样本；运行时长度来自端口。 */
#define NCO_DAC_BLOCK_SAMPLE_COUNT       \
    (NCO_DAC_BASE_BLOCK_SAMPLE_COUNT * NCO_DAC_INTERPOLATION_FACTOR)

_Static_assert(NCO_DAC_SAMPLE_RATE_HZ ==
               (NCO_DAC_NCO_SAMPLE_RATE_HZ * NCO_DAC_INTERPOLATION_FACTOR),
               "DAC 速率必须是整数插值后的 NCO 速率");

typedef struct
{
    uint64_t block_sequence;               /* 与输入块对齐的序号，从 0 开始 */
    uint64_t start_phase;                  /* 输入首样本参考相位，2^64/周 */
    uint64_t first_frequency_control_word; /* 第一个参考步长，2^64/周 */
    uint64_t frequency_control_word;       /* 后续参考步长，2^64/周 */
} NCO_DAC_ReferenceMetadata;

/* 初始化单实例输出服务；借用 port，禁止运行中重新初始化。
 * 要求共同主时钟、整数输出/输入速率比、输入输出块覆盖相同时间。
 * 所有 bool 启停函数 true 为成功；false 为参数/状态或平台操作失败。
 * Init/Arm/Start/Stop 仅在主循环调用，不阻塞等待数据。 */
bool NCO_DAC_Init(const SignalPort *port);
/* 填充两块并装载输出，触发仍停止；失败不会启动采样时钟。 */
bool NCO_DAC_Arm(void);
/* 便捷启动输出；同步输入应用应先 Arm，再启动输入，最后 StartTrigger。 */
bool NCO_DAC_Start(void);
bool NCO_DAC_StartTrigger(void);
bool NCO_DAC_Stop(void);
bool NCO_DAC_SetOutputCallback(SignalOutputReadyCallback callback, void *context);
bool NCO_DAC_GetOutputBuffer(uint32_t index, SignalOutputBlock *block);

/* 按序号复制最近 8 块之一的元数据到非 NULL 指针，不转移内部所有权。
 * 返回 false 表示未初始化、参数无效或记录已过期。可从输入回调调用。 */
bool NCO_DAC_GetReferenceMetadata(uint64_t block_sequence,
                                  NCO_DAC_ReferenceMetadata *metadata);
/* 仅含中间层渲染，单位 CPU 周期；不含平台缓存维护及外层中断分发。 */
uint32_t NCO_DAC_GetLastFillCycles(void);
uint32_t NCO_DAC_GetMaximumFillCycles(void);

/* FCW 按实际 ADC 采样率定义。渲染器在最终输出率的 DMA 传输之前，
 * 按共同主时钟的整数速率比对每个 NCO 区间做余数插值。 */
void NCO_DAC_SetFrequencyControlWord(uint64_t frequency_control_word);
uint64_t NCO_DAC_GetFrequencyControlWord(void);

/* 当闭环接管 NCO 时，手动 FCW 写入会被忽略。 */
bool NCO_DAC_SetClosedLoopEnabled(bool enable);
bool NCO_DAC_IsClosedLoopEnabled(void);
bool NCO_DAC_SetClosedLoopFrequencyControlWord(
    uint64_t frequency_control_word);

/* 手动设置频率 0..实际 ADC 采样率/2；未初始化、闭环占用或参数非法返回 false。 */
bool NCO_DAC_SetFrequencyHz(double frequency_hz);
double NCO_DAC_GetFrequencyHz(void);
/* 频率/FCW 换算要求已初始化；frequency_hz 应在 0..实际 ADC 采样率/2。 */
uint64_t NCO_DAC_FrequencyHzToControlWord(double frequency_hz);
double NCO_DAC_ControlWordToFrequencyHz(uint64_t frequency_control_word);

/* amplitude 为峰值幅度码，dc_offset 为偏置码；必须满足
 * amplitude <= dc_offset 且 amplitude + dc_offset <= 4095，非法返回 false。 */
bool NCO_DAC_SetOutputLevel(uint16_t amplitude, uint16_t dc_offset);

/* 所选的预计算相位表会直接渲染到每个安全的 DMA 缓冲区中。
 * 运行时修改在随后重填的块上生效。 */
bool NCO_DAC_SetOutputWaveform(NCO_OutputWaveform waveform);
NCO_OutputWaveform NCO_DAC_GetOutputWaveform(void);

/* 仅对用于渲染 DAC 波形的相位进行倍频。保留用于 IQ 检测和 PI 控制的
 * 基础 NCO 相位/FCW 保持不变。整数倍频系数应用于随后重填的 DMA 块。 */
bool NCO_DAC_SetOutputFrequencyMultiplier(uint32_t multiplier);
uint32_t NCO_DAC_GetOutputFrequencyMultiplier(void);
uint32_t NCO_DAC_GetMaximumOutputFrequencyMultiplier(void);
double NCO_DAC_GetOutputFrequencyHz(void);

#ifdef __cplusplus
}
#endif

#endif /* NCO_DAC_H */
