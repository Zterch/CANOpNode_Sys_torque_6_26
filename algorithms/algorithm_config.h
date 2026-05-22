/******************************************************************************
 * @file    algorithm_config.h
 * @brief   算法参数配置文件 - 重力卸载控制系统
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 * 
 * @description
 * 本文件包含重力卸载控制系统的所有可调参数。
 * 所有物理参数、控制参数、安全阈值集中在此配置，便于维护和调整。
 * 
 * 系统描述：
 * - 滑轮1（电机侧）：半径 R1 = 100mm，连接磁粉离合器
 * - 滑轮2（编码器侧）：半径 R2 = 50mm，下方有压力传感器
 * - 磁粉离合器：额定电流 0.88A，额定转矩 5Nm
 ******************************************************************************/

#ifndef __ALGORITHM_CONFIG_H__
#define __ALGORITHM_CONFIG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 物理系统参数 - 根据实际硬件修改
 ******************************************************************************/

/* 滑轮半径 (单位：米) */
#define PULLEY_R1_MOTOR_RADIUS_M        0.100f      /* 电机侧滑轮半径 R1 = 100mm */
#define PULLEY_R2_ENCODER_RADIUS_M      0.050f      /* 编码器侧滑轮半径 R2 = 50mm */

/* 磁粉离合器参数 */
#define CLUTCH_RATED_CURRENT_MA         880         /* 额定电流 0.88A = 880mA */
#define CLUTCH_RATED_TORQUE_NM          5.0f        /* 额定转矩 5Nm */
#define CLUTCH_RATED_POWER_W            21.12f      /* 额定功率 21.12W */

/* 离合器电流-转矩转换系数 (mA per Nm) */
/* 假设线性关系：转矩 = 电流 / (额定电流/额定转矩) */
#define CLUTCH_CURRENT_PER_TORQUE_MA_NM 176.0f      /* 880mA / 5Nm = 176 mA/Nm */

/******************************************************************************
 * 控制算法参数
 ******************************************************************************/

/* 控制周期 - 100Hz严格实时控制 */
#define ALGO_CONTROL_PERIOD_MS          10          /* 控制周期 10ms = 100Hz */
#define ALGO_CONTROL_PERIOD_S           0.01f       /* 控制周期 0.01s */

/* 速度计算滤波参数 */
/* 编码器分辨率限制导致速度呈现0.0038m/s的阶梯状，需要更大的滤波窗口来平滑 */
#define SPEED_FILTER_WINDOW_SIZE        10          /* 0.2s滑动平均 = 10个采样点（20ms周期） */
/* 总延迟 = 20ms(测速) + 200ms(滤波) = 220ms，工业可接受 */

/* 低通滤波器参数 - 用于进一步平滑速度信号 */
#define SPEED_LPF_ALPHA                 0.15f       /* 低通滤波系数 0-1，越小越平滑 */
#define SPEED_FILTER_SAMPLE_TIME_MS     10          /* 采样时间 10ms */

/* 基于摩擦力的电机速度控制参数 */
#define PRESSURE_FILTER_WINDOW_SIZE     10          /* 压力传感器平均值滤波窗口大小 */
#define FRICTION_ANGLE_COS              0.861f      /* cos(30.5°) ≈ 0.861 */
#define FRICTION_SPEED_OFFSET_C         0.0f       /* 速度偏移常量 C (rpm) */
#define PRESSURE_F0_DEFAULT_KG          0.0f        /* 静止时压力默认值，运行时会自动校准 */
#define MOTOR_SPEED_COMPENSATION_C      0.0f        /* 电机速度补偿系数 - 新算法中不再使用，保留兼容性 */

/* 编码器到距离的转换系数 (米/脉冲) */
/* 距离 = 脉冲数 / 分辨率 * 2π * R2 */
#define ENCODER_TO_DISTANCE_M           7.6699e-5f  /* 2π * 0.05 / 4096 */

/******************************************************************************
 * PID控制器参数 - 电机速度控制
 ******************************************************************************/

/* PID参数 - 保守设置以消除振荡 */
#define PID_KP                          10.0f        /* 比例系数 - 降低消除振荡 */
#define PID_KI                          0.1f       /* 积分系数 - 低值避免饱和 */
#define PID_KD                          0.0f        /* 微分系数 - 禁用，消除超前 */

#define PID_OUTPUT_MIN                  -10000      /* 电机最小速度指令 */
#define PID_OUTPUT_MAX                  10000       /* 电机最大速度指令 */
#define PID_INTEGRAL_LIMIT              1000.0f     /* 积分限幅 */

/******************************************************************************
 * 传感器校准参数
 ******************************************************************************/

/* 压力传感器 */
#define PRESSURE_ZERO_OFFSET_KG         0.0f        /* 零点偏移（去皮后） */
#define PRESSURE_FILTER_ALPHA           0.8f        /* 低通滤波系数 0-1 */

/* 编码器 */
#define ENCODER_ZERO_POSITION           0           /* 零位位置 */
#define ENCODER_DIRECTION               -1          /* 方向：1=正向，-1=反向。重物上升时位置增加，速度为正 */

/******************************************************************************
 * 安全保护参数 - 重要！防止系统失控
 ******************************************************************************/

/* 压力传感器安全阈值 */
#define SAFETY_PRESSURE_MIN_KG          -5.0f       /* 最小允许压力 - 放宽到-5kg允许去皮误差 */
#define SAFETY_PRESSURE_MAX_KG          40.0f       /* 最大允许压力 - 增加到25kg，适应实际工况 */
#define SAFETY_PRESSURE_RATE_MAX_KG_S   10.0f       /* 压力变化率限制 kg/s - 放宽到10kg/s */

/* 电机安全限制 */
#define SAFETY_MOTOR_SPEED_MAX          8000        /* 电机最大速度限制 */
#define SAFETY_MOTOR_ACCEL_MAX          5000        /* 电机最大加速度限制 */
#define SAFETY_MOTOR_TIMEOUT_MS         100         /* 电机通信超时 ms */

/* 离合器电流安全限制 */
#define SAFETY_CLUTCH_CURRENT_MIN_MA    0           /* 最小电流 */
#define SAFETY_CLUTCH_CURRENT_MAX_MA    900         /* 最大电流（略高于额定值） */
#define SAFETY_CLUTCH_CURRENT_RATE_MAX_MA_S 500     /* 电流变化率限制 mA/s */

/* 位置安全限制 - 放宽以适应实际测试需求 */
#define SAFETY_POSITION_MIN_M           -3.0f       /* 最小位置 -3m */
#define SAFETY_POSITION_MAX_M           5.0f        /* 最大位置 5m */
#define SAFETY_SPEED_MAX_M_S            2.0f        /* 最大速度 2m/s */

/* 系统健康检查参数 */
#define HEALTH_CHECK_SENSOR_MIN_RATE_HZ 45          /* 传感器最小更新频率 */
#define HEALTH_CHECK_TIMEOUT_MS         1000        /* 健康检查超时 */

/******************************************************************************
 * 算法状态枚举
 ******************************************************************************/

typedef enum {
    ALGO_STATE_INIT = 0,        /* 初始化 */
    ALGO_STATE_CHECKING,        /* 系统检测中 */
    ALGO_STATE_READY,           /* 就绪等待确认 */
    ALGO_STATE_RUNNING,         /* 运行中 */
    ALGO_STATE_PAUSED,          /* 暂停 */
    ALGO_STATE_ERROR,           /* 错误状态 */
    ALGO_STATE_EMERGENCY_STOP,  /* 紧急停止 */
    ALGO_STATE_SHUTDOWN         /* 关闭中 */
} AlgoState_t;

typedef enum {
    ALGO_ERR_NONE = 0,
    ALGO_ERR_INVALID_PARAM,     /* 无效参数 */
    ALGO_ERR_SENSOR_FAIL,       /* 传感器故障 */
    ALGO_ERR_MOTOR_FAIL,        /* 电机故障 */
    ALGO_ERR_CLUTCH_FAIL,       /* 离合器故障 */
    ALGO_ERR_SAFETY_VIOLATION,  /* 安全限制违反 */
    ALGO_ERR_COMM_TIMEOUT,      /* 通信超时 */
    ALGO_ERR_USER_ABORT         /* 用户中止 */
} AlgoError_t;

/******************************************************************************
 * 数据结构定义
 ******************************************************************************/

/* 传感器原始数据 */
typedef struct {
    float pressure_kg;          /* 压力传感器读数 kg */
    float encoder_position_m;   /* 编码器位置 m */
    int32_t encoder_pulse_delta; /* 编码器脉冲变化量（M/T法测速） */
    uint32_t encoder_time_delta_us; /* 编码器时间变化量（微秒，M/T法测速） */
    uint32_t timestamp_ms;      /* 时间戳 ms */
    int data_valid;             /* 数据有效标志 */
} SensorDataRaw_t;

/* 滤波后的传感器数据 */
typedef struct {
    float pressure_kg;          /* 滤波后压力 */
    float pressure_derivative;  /* 压力变化率 kg/s */
    float position_m;           /* 位置 m */
    float velocity_m_s;         /* 速度 m/s (滤波后) */
    float velocity_raw_m_s;     /* 原始速度 m/s */
    uint32_t timestamp_ms;
} SensorDataFiltered_t;

/* 控制输出 */
typedef struct {
    float clutch_current_mA;    /* 离合器目标电流 mA */
    float clutch_torque_nm;     /* 离合器目标转矩 Nm */
    float motor_velocity_cmd;   /* 电机速度指令（PID输出） */
    float motor_velocity_target;/* 电机目标速度（新算法计算值，用于数据记录） */
    float motor_velocity_actual;/* 电机实际速度 */
    uint32_t timestamp_ms;
} ControlOutput_t;

/* 系统状态 */
typedef struct {
    AlgoState_t state;
    AlgoError_t error;
    uint32_t cycle_count;
    uint32_t error_count;
    float running_time_s;
    int emergency_stop;
} AlgoStatus_t;

#ifdef __cplusplus
}
#endif

#endif /* __ALGORITHM_CONFIG_H__ */
