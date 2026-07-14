/******************************************************************************
 * @file    motor_driver.h
 * @brief   NiMotion SDK电机驱动 - 工业级CANopen主站控制
 * @author  System Architect
 * @date    2026-05-12
 * @version 2.0.0
 * 
 * @description
 * 基于NiMotion NimServoSDK的电机驱动实现
 * - 支持100Hz PDO实时控制
 * - 支持CSV/CSP/CST等同步模式
 * - 工业级错误处理和状态管理
 ******************************************************************************/

#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 电机状态定义 (CiA 402标准)
 ******************************************************************************/
typedef enum {
    MOTOR_STATE_INIT = 0,       /* 初始化 */
    MOTOR_STATE_NOT_READY,      /* 未就绪 */
    MOTOR_STATE_READY,          /* 就绪 */
    MOTOR_STATE_ENABLED,        /* 已使能 */
    MOTOR_STATE_FAULT,          /* 故障 */
    MOTOR_STATE_UNKNOWN         /* 未知 */
} MotorState_t;

/******************************************************************************
 * 电机工作模式
 ******************************************************************************/
typedef enum {
    MOTOR_MODE_PP = 1,          /* 轮廓位置模式 */
    MOTOR_MODE_VM = 2,          /* 速度模式 */
    MOTOR_MODE_PV = 3,          /* 轮廓速度模式 */
    MOTOR_MODE_PT = 4,          /* 轮廓转矩模式 */
    MOTOR_MODE_HM = 6,          /* 原点回归模式 */
    MOTOR_MODE_IP = 7,          /* 位置插补模式 */
    MOTOR_MODE_CSP = 8,         /* 循环同步位置模式 */
    MOTOR_MODE_CSV = 9,         /* 循环同步速度模式 */
    MOTOR_MODE_CST = 10         /* 循环同步转矩模式 */
} MotorMode_t;

/******************************************************************************
 * 电机数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    uint8_t node_id;            /* CAN节点ID */
    char can_interface[16];     /* CAN接口名称 */
    
    /* SDK句柄 */
    unsigned int sdk_master;    /* SDK主站句柄 */
    int sdk_initialized;        /* SDK初始化标志 */
    
    /* 状态信息 */
    MotorState_t state;         /* 电机状态 */
    MotorMode_t mode;           /* 当前模式 */
    uint16_t status_word;       /* 状态字 */
    
    /* 实时数据 (通过PDO 100Hz更新) */
    volatile double actual_position;    /* 实际位置 (用户单位) */
    volatile double actual_velocity;    /* 实际速度 (用户单位/s) */
    volatile int actual_speed_rpm;      /* 实际速度 (rpm) */
    volatile int actual_torque;         /* 实际转矩 (0.001倍额定) */
    volatile double target_velocity;    /* 目标速度 */
    volatile double target_position;    /* 目标位置 */
    volatile int target_torque;         /* 目标转矩 (0.001倍额定) */
    
    /* 控制参数 */
    double max_velocity;        /* 最大速度 */
    double max_acceleration;    /* 最大加速度 */
    double unit_factor;         /* 用户单位转换系数 */
    
    /* 机械参数 */
    double gear_ratio;          /* 减速比 (电机转数:轮子转数) */
    double wheel_radius_mm;     /* 轮子半径 (mm) */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    pthread_mutex_t data_mutex; /* 数据访问互斥锁 */
    
    /* 状态标志 */
    int initialized;
    int enabled;
    int fault_reset_needed;     /* 需要故障复位 */
    
    /* 统计信息 */
    uint64_t pdo_tx_count;      /* PDO发送计数 */
    uint64_t pdo_rx_count;      /* PDO接收计数 */
    uint64_t pdo_rx_fail_count; /* PDO接收失败计数 */
    uint64_t error_count;       /* 错误计数 */
} MotorDriver_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化电机驱动和SDK
 * @param motor 电机驱动结构指针
 * @param node_id CAN节点ID
 * @param can_if CAN接口名称 (如 "can0")
 * @return ErrorCode_t
 */
ErrorCode_t motor_init(MotorDriver_t *motor, uint8_t node_id, const char *can_if);

/**
 * @brief 反初始化电机驱动和SDK
 * @param motor 电机驱动结构指针
 */
void motor_deinit(MotorDriver_t *motor);

/**
 * @brief 使能电机 (CSV模式)
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_enable(MotorDriver_t *motor);

/**
 * @brief 失能电机
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_disable(MotorDriver_t *motor);

/**
 * @brief 设置工作模式
 * @param motor 电机驱动结构指针
 * @param mode 工作模式
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_mode(MotorDriver_t *motor, MotorMode_t mode);

/**
 * @brief 设置目标速度 (PDO方式, 100Hz实时)
 * @param motor 电机驱动结构指针
 * @param velocity 目标速度 (rpm)，支持浮点数
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_velocity(MotorDriver_t *motor, float velocity);

/**
 * @brief 设置目标位置 (PDO方式)
 * @param motor 电机驱动结构指针
 * @param position 目标位置
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_position(MotorDriver_t *motor, int32_t position);

/**
 * @brief 读取实际速度
 * @param motor 电机驱动结构指针
 * @param velocity 速度输出指针 (rpm)
 * @return ErrorCode_t
 */
ErrorCode_t motor_get_velocity(MotorDriver_t *motor, int32_t *velocity);

/**
 * @brief 读取实际位置
 * @param motor 电机驱动结构指针
 * @param position 位置输出指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_get_position(MotorDriver_t *motor, int32_t *position);

/**
 * @brief 获取电机实际位置（以米为单位）
 * @param motor 电机驱动结构指针
 * @return 位置值（米）
 */
float motor_get_position_m(MotorDriver_t *motor);

/**
 * @brief 计算线速度（m/s）
 * @param motor 电机驱动结构指针
 * @param motor_rpm 电机转速（rpm）
 * @return 线速度（m/s）
 */
float motor_calculate_linear_velocity(MotorDriver_t *motor, float motor_rpm);

/**
 * @brief 获取电机实际速度（以rpm为单位）
 * @param motor 电机驱动结构指针
 * @return 速度值（rpm）
 */
int32_t motor_get_velocity_rpm(MotorDriver_t *motor);

/**
 * @brief 从缓存读取实际速度（非阻塞，工业级实时控制使用）
 * @param motor 电机驱动结构指针
 * @param velocity 速度输出指针 (rpm)
 * @return ErrorCode_t
 * @note 此函数直接从缓存读取，不执行CANopen通信，适用于100Hz实时控制循环
 */
ErrorCode_t motor_get_velocity_cached(MotorDriver_t *motor, int32_t *velocity);

/**
 * @brief 更新电机状态 (100Hz周期性调用, 通过PDO读取)
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_update_state(MotorDriver_t *motor);

/**
 * @brief 清除故障
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_clear_fault(MotorDriver_t *motor);

/**
 * @brief 快速停止
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_fast_stop(MotorDriver_t *motor);

/**
 * @brief 获取电机状态字符串
 * @param state 电机状态
 * @return 状态字符串
 */
const char* motor_get_state_string(MotorState_t state);

/**
 * @brief 获取模式字符串
 * @param mode 工作模式
 * @return 模式字符串
 */
const char* motor_get_mode_string(MotorMode_t mode);

/**
 * @brief 获取电机统计信息
 * @param motor 电机驱动结构指针
 * @param tx_count PDO发送计数输出
 * @param rx_count PDO接收计数输出
 * @param err_count 错误计数输出
 */
void motor_get_statistics(MotorDriver_t *motor, uint64_t *tx_count, 
                          uint64_t *rx_count, uint64_t *err_count);

/**
 * @brief 设置目标转矩 (CST模式, PDO方式, 100Hz实时)
 * @param motor 电机驱动结构指针
 * @param torque 目标转矩 (0.001倍额定转矩), 范围: -1000 ~ 1000
 * @return ErrorCode_t
 * @note 电机需在CST模式下使能
 */
ErrorCode_t motor_set_torque(MotorDriver_t *motor, int torque);

/**
 * @brief 读取实际转矩
 * @param motor 电机驱动结构指针
 * @param torque 转矩输出指针 (0.001倍额定转矩)
 * @return ErrorCode_t
 */
ErrorCode_t motor_get_torque(MotorDriver_t *motor, int *torque);

/**
 * @brief 从缓存读取实际转矩（非阻塞，工业级实时控制使用）
 * @param motor 电机驱动结构指针
 * @param torque 转矩输出指针 (0.001倍额定转矩)
 * @return ErrorCode_t
 * @note 此函数直接从缓存读取，不执行CANopen通信，适用于100Hz实时控制循环
 */
ErrorCode_t motor_get_torque_cached(MotorDriver_t *motor, int *torque);

/**
 * @brief 设置目标转矩缓存（非阻塞，仅更新缓存值）
 * @param motor 电机驱动结构指针
 * @param torque 目标转矩 (0.001倍额定转矩), 范围: -1000 ~ 1000
 * @return ErrorCode_t
 * @note 此函数仅更新缓存值，不执行CANopen通信，适用于100Hz实时控制循环
 *       实际的PDO发送由专门的PDO线程处理
 */
ErrorCode_t motor_set_torque_cached(MotorDriver_t *motor, int torque);

/**
 * @brief 使能电机转矩控制模式 (CST模式)
 * @param motor 电机驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t motor_enable_torque_mode(MotorDriver_t *motor);

/**
 * @brief 设置转矩斜坡
 * @param motor 电机驱动结构指针
 * @param torque_ramp 转矩斜坡: 0 无斜坡; >0 每秒钟增加的转矩值 (0.001倍额定转矩)
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_torque_ramp(MotorDriver_t *motor, uint32_t torque_ramp);

/**
 * @brief 设置轮廓转矩模式下速度限制
 * @param motor 电机驱动结构指针
 * @param fwd_limit 正向速度限制 (rpm)
 * @param bwd_limit 反向速度限制 (rpm)
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_torque_speed_limit(MotorDriver_t *motor, uint16_t fwd_limit, uint16_t bwd_limit);

/**
 * @brief 设置电机速度环PID参数
 * @param motor 电机驱动结构指针
 * @param kp 速度环比例增益 (I2008-1, 单位: 0.1Hz, 范围: 1-20000)
 * @param ki 速度环积分时间常数 (I2008-2, 单位: 0.01ms, 范围: 0-51200, 0=禁用积分)
 * @param kff 速度前馈增益 (I2008-16, 单位: 0.001, 范围: 0-65535)
 * @return ErrorCode_t
 */
ErrorCode_t motor_set_velocity_pid(MotorDriver_t *motor, uint16_t kp, uint16_t ki, uint16_t kff);

/**
 * @brief 获取电机速度环PID参数
 * @param motor 电机驱动结构指针
 * @param kp 速度环比例增益输出
 * @param ki 速度环积分时间常数输出
 * @param kff 速度前馈增益输出
 * @return ErrorCode_t
 */
ErrorCode_t motor_get_velocity_pid(MotorDriver_t *motor, uint16_t *kp, uint16_t *ki, uint16_t *kff);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H__ */
