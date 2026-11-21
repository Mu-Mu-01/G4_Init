/**
 * @file    signal_port.h
 * @brief   Interface 层：同步采样/输出流及运行环境契约，不包含芯片类型。
 *
 * 本头文件定义了"一条同步输入流 + 一条同步输出流"的抽象接口，是 DPLL 算法层
 * 与具体硬件（ADC/DAC/DMA/定时器）之间的"合同"。算法层只依赖这里的函数指针
 * 类型，不 include 任何 HAL 头，从而保持芯片无关、便于移植到其他平台。
 *
 * 核心约定：
 *  - 单个端口（SignalPort）对应一条流；描述符、操作表、context 和缓冲区须在
 *    整个运行期有效，不随启动/停止重新分配。
 *  - 启停操作（arm/start/stop）仅在主循环、触发停止时调用；输入/输出/错误回调
 *    在中断上下文同步执行，必须非阻塞且不得重入同一回调。
 *  - 平台负责缓存一致性（清理/失效），回调方不感知 cache 细节。
 *
 * 启动时序固定为：先 arm 输出（装载但不触发）→ 启动输入（装 DMA + 从触发）→
 * 最后启动主触发，使输入与输出的块序号共享同一条时间轴。
 */
#ifndef SIGNAL_PORT_H
#define SIGNAL_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "signal_config.h"

/**
 * @brief  输入回调：ADC 采满一个块后，平台把数据同步交给订阅者。
 * @param  context  订阅者上下文，由 start_input 时传入，平台不解释其含义。
 * @param  samples  本块样本首指针，为未缩放的无符号 ADC 码。
 * @param  count    本块样本数。
 * @param  sequence 本块序号，每次启动从 0 递增，并与输出块时间轴对齐。
 * @note   samples 仅在回调执行期间只读有效，不能留存指针（DMA 会循环覆盖）；
 *         回调必须在一个块周期内返回；平台不得重入同一输入回调。
 */
typedef void (*SignalInputCallback)(void *context, const uint16_t *samples,
                                    uint32_t count, uint64_t sequence);

/**
 * @brief  输出回调：DAC 消费完一个块后，平台把该块交还给订阅者重填。
 * @param  context 订阅者上下文。
 * @param  buffer  待重填的缓冲区首指针，写入右对齐的 12 位 DAC 码。
 * @param  count   可写样本数。
 * @note   仅将已消费的 buffer 交给回调重填；回调返回前平台不得让 DMA 重新读取
 *         该块。平台负责缓存一致性；同一个输出回调不得重入，但可抢占输入回调。
 */
typedef void (*SignalOutputCallback)(void *context, uint16_t *buffer,
                                     uint32_t count);

/**
 * @brief  流错误类型，仅用于通知，不隐式触发重启。
 */
typedef enum
{
    SIGNAL_ERROR_INPUT,          /**< 采集或输入 DMA 错误，code 为平台诊断码 */
    SIGNAL_ERROR_OUTPUT_UNDERRUN /**< 输出数据供给不足，code 为平台诊断码 */
} SignalError;

/**
 * @brief  错误回调：平台检测到流错误时通知订阅者。
 * @param  context 订阅者上下文。
 * @param  error   错误类型。
 * @param  code    平台诊断码，不作为跨平台业务条件使用。
 * @note   错误仅通知、不隐式重启；恢复策略由订阅者自行决定。
 */
typedef void (*SignalErrorCallback)(void *context, SignalError error,
                                    uint32_t code);

/**
 * @brief  平台操作表：抽象运行环境与启停流程，由平台适配层实现。
 *
 * 这是整个契约的核心——算法层通过这组函数指针与平台交互，不依赖具体芯片。
 * 所有成员在运行期有效且只读；context 均为平台私有状态，可由无状态实现置 NULL。
 */
typedef struct
{
    /**
     * @brief 进入临界区：禁止中断并返回进入前的屏蔽状态作为恢复令牌。
     * @param  context 平台私有状态。
     * @retval token 进入前的中断屏蔽状态（PRIMASK），供 exit_critical 恢复。
     * @note  临界区须可嵌套，且保护输入/输出回调与主循环之间的共享数据；
     *        token 是恢复状态而非锁编号。
     */
    uint32_t (*enter_critical)(void *context);
    /**
     * @brief 退出临界区：用进入时返回的 token 恢复原中断屏蔽状态。
     * @param  context 平台私有状态。
     * @param  token   由匹配的 enter_critical 返回。
     */
    void (*exit_critical)(void *context, uint32_t token);
    /**
     * @brief 发布数据前的编译器及内存屏障。
     * @param  context 平台私有状态。
     */
    void (*memory_barrier)(void *context);
    /**
     * @brief 读取 CPU 周期计数器。
     * @param  context 平台私有状态。
     * @retval 周期计数值，按 uint32_t 回绕。
     */
    uint32_t (*cycles)(void *context);
    /**
     * @brief 装载两个已填好的输出块和首个预装码，但不产生触发。
     * @param  context          平台私有状态。
     * @param  preload          首个触发沿前的预装码。
     * @param  refill           输出重填回调，之后每消费完一块调用一次。
     * @param  callback_context 传递给 refill 的上下文。
     * @retval true  装载成功；false 失败且已撤销本次启动。
     * @note   回调/context 只借用，不释放；公共触发此时必须尚未启动。
     */
    bool (*arm_output)(void *context, uint16_t preload,
                       SignalOutputCallback refill, void *callback_context);
    /**
     * @brief 最后启动主触发，使输入输出共享同一条触发时间轴。
     * @param  context 平台私有状态。
     * @retval true 启动成功；false 失败，此时已停止并撤销输出 DMA。
     */
    bool (*start_output)(void *context);
    /**
     * @brief 停止输出：先停止触发，再停止输出 DMA。
     * @param  context 平台私有状态。
     * @retval true 停止成功；false 任一步骤失败。
     */
    bool (*stop_output)(void *context);
    /**
     * @brief 安装输入及错误回调、启动输入 DMA 和从触发。
     * @param  context          平台私有状态。
     * @param  receive          输入回调。
     * @param  error            错误回调。
     * @param  callback_context 传递给 receive/error 的上下文。
     * @retval true  启动成功；false 失败并已撤销输入启动。
     * @note   主触发仍须保持停止；sequence 从 0 开始，不能在成功返回前回调。
     */
    bool (*start_input)(void *context, SignalInputCallback receive,
                        SignalErrorCallback error, void *callback_context);
    /**
     * @brief 停止输入：释放输入 DMA 与从触发。
     * @param  context 平台私有状态。
     * @retval true 停止成功；false 任一步骤失败。
     * @note   须在主触发停止后调用；停止后平台不得再触发输入/错误回调。
     */
    bool (*stop_input)(void *context);
    /**
     * @brief 应用或校验采样配置：计算最近分频、验证容量并（可选）提交到定时器。
     * @param  context       平台私有状态。
     * @param  config        请求的 ADC 采样率、升采样倍数、输出开关与存储地址。
     * @param  validate_only true 仅计算与检查，可在运行中预检；false 实际提交到硬件。
     * @param  info          可选输出，取得实际速率/误差/块周期；失败时不改写。
     * @retval true  配置可用（validate_only）或已提交；false 参数、地址、容量、时钟或启停状态不合法。
     * @note   主循环调用；先完整预检再修改 PSC/ARR，触发路由与参考流水线不变。
     */
    bool (*configure)(void *context, const SignalStreamConfig *config,
                      bool validate_only, SignalStreamInfo *info);
} SignalPortOps;

/**
 * @brief  端口描述符：把操作表、私有状态与流参数打包成一个对外对象。
 */
typedef struct
{
    const SignalPortOps *ops;       /**< 平台操作表，只读且运行期有效 */
    void *context;                  /**< 平台私有状态，可由无状态实现置 NULL */
    double input_rate_hz;           /**< 实际 ADC 采样率，Hz */
    double output_rate_hz;          /**< 实际 DAC 更新率，Hz */
    uint32_t input_block_samples;   /**< 每次输入回调的样本数 */
    uint32_t output_block_samples;  /**< 每个输出缓冲区的样本数 */
    uint32_t reference_sample_offset; /**< 第一个输入样本对应插值组内的输出索引 */
    uint32_t output_samples_per_input_sample; /**< TIM8 对 TIM1 事件的总分频 */
    bool reference_uses_previous_sample; /**< 同速 DAC 时首参考来自前一块末点 */
    SignalStreamInfo stream_info; /**< 实际速率、误差与块周期 */
    uint16_t *output_buffers[2];    /**< 平台拥有的两个互不重叠的输出缓冲区 */
} SignalPort;

/**
 * @brief  检查端口结构的基本完整性。
 * @param  port 待校验的端口。
 * @retval true  结构合法（操作表各成员非空、速率/块长非 0、双缓冲非空且不重叠）；
 *         false 任一条件不满足。
 * @note   采样率、块长和参考相位的兼容性由使用方另行验证，本函数不检查。
 */
static inline bool SignalPort_IsValid(const SignalPort *port)
{
    return port != NULL && port->ops != NULL &&
           port->ops->enter_critical != NULL && port->ops->exit_critical != NULL &&
           port->ops->memory_barrier != NULL && port->ops->cycles != NULL &&
           port->ops->arm_output != NULL && port->ops->start_output != NULL &&
           port->ops->stop_output != NULL && port->ops->start_input != NULL &&
           port->ops->stop_input != NULL && port->input_rate_hz > 0.0 &&
           port->output_rate_hz > 0.0 &&
           port->input_block_samples != 0U &&
           port->output_block_samples != 0U && port->output_buffers[0] != NULL &&
           port->output_buffers[1] != NULL &&
           port->output_buffers[0] != port->output_buffers[1];
}

/**
 * @brief  进入临界区的内联包装，统一处理 port 为空的情况。
 * @param  port 端口；初始化前可为 NULL（此时仅用于单线程默认值查询）。
 * @retval 进入前的屏蔽状态；port 为空返回 0。
 */
static inline uint32_t SignalPort_EnterCritical(const SignalPort *port)
{
    return port != NULL ? port->ops->enter_critical(port->context) : 0U;
}

/**
 * @brief  退出临界区的内联包装，用进入时的 token 恢复屏蔽状态。
 * @param  port  端口；可为 NULL。
 * @param  token 匹配的 enter_critical 返回值。
 */
static inline void SignalPort_ExitCritical(const SignalPort *port, uint32_t token)
{
    if (port != NULL) { port->ops->exit_critical(port->context, token); }
}

/**
 * @brief  发布数据前执行内存屏障的内联包装。
 * @param  port 端口；可为 NULL。
 */
static inline void SignalPort_MemoryBarrier(const SignalPort *port)
{
    if (port != NULL) { port->ops->memory_barrier(port->context); }
}

/**
 * @brief  读取 CPU 周期计数的内联包装。
 * @param  port 端口；可为 NULL。
 * @retval 周期计数值；port 为空返回 0。
 */
static inline uint32_t SignalPort_Cycles(const SignalPort *port)
{
    return port != NULL ? port->ops->cycles(port->context) : 0U;
}

#endif /* SIGNAL_PORT_H */
