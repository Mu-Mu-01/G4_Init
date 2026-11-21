/* BSP 层：本板信号资源装配入口；上层只接收平台无关端口。 */
#ifndef BSP_SIGNAL_H
#define BSP_SIGNAL_H
#include "signal_port.h"

/* HAL_Init 后、MX 外设初始化前调用；准备平台所需的计时和内存资源。 */
void BSP_Signal_PreparePlatform(void);
/* 使用 CubeMX 当前生成的配置初始化信号链外设；调用顺序由 BSP 统一维护。 */
void BSP_Signal_InitPeripherals(void);
/* BSP_Signal_InitPeripherals 完成后、主循环启动信号前调用。返回板级单例端口；
 * NULL 表示绑定失败。不启动外设；端口及缓冲区具有静态生命周期。 */
const SignalPort *BSP_Signal_GetPort(void);
/* 填入当前板级默认采样率、升采样倍数和输入块长。 */
void BSP_Signal_GetDefaultConfig(SignalStreamConfig *config);
/* 停流时绑定并配置采样定时器分频；NULL 表示配置无法应用。 */
const SignalPort *BSP_Signal_Configure(const SignalStreamConfig *config);

extern volatile uint32_t g_icache_enabled; /* 保留调试符号：I-Cache 实际开启状态 */
#endif /* BSP_SIGNAL_H */
