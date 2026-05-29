/******************************************************************************
 * @file    gravity_unload.h
 * @brief   重力卸载控制算法 - 核心算法模块
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 * 
 * @description
 * 实现重力卸载控制系统的核心算法：
 * 1. 读取压力传感器，计算所需离合器力矩
 * 2. 读取编码器，计算重物速度和位置
 * 3. PID控制电机速度
 ******************************************************************************/

#ifndef __GRAVITY_UNLOAD_H__
#define __GRAVITY_UNLOAD_H__

#include <stdint.h>
#include <pthread.h>
#include "algorithm_config.h"
#include "signal_filter.h"
#include "pid_controller.h"
#include "safety_monitor.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 重力卸载控制器结构体
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    float pulley_r1_m;          /* 电机侧滑轮半径 m */
    float pulley_r2_m;          /* 编码器侧滑轮半径 m */
    float clutch_current_per_torque; /* 离合器电流-转矩系数 mA/Nm */
    float motor_speed_compensation;  /* 电机速度补偿系数 */
    
    /* 滤波器 */
    MovingAverageFilter_t velocity_filter;  /* 速度滑动窗口平均滤波 - 更好地平滑阶梯效应 */
    LowPassFilter_t velocity_lpf;           /* 速度低通滤波 - 进一步平滑编码器分辨率导致的阶梯 */
    LowPassFilter_t pressure_filter;        /* 压力低通滤波 */
    Differentiator_t pressure_diff;         /* 压力微分 */
    Differentiator_t position_diff;         /* 位置微分（速度计算） */
    
    /* PID控制器 */
    PID_Controller_t pid;
    
    /* 安全监控 */
    SafetyMonitor_t safety;
    
    /* 状态 */
    AlgoStatus_t status;
    
    /* 上次状态（用于计算） */
    float last_position_m;
    uint32_t last_timestamp_ms;
    int first_run;
    
    /* 统计 */
    uint32_t cycle_count;
    float max_pressure_kg;
    float min_pressure_kg;
    float max_velocity_m_s;
    
    /* 压力平均值滤波缓冲区 */
    float pressure_buffer[PRESSURE_FILTER_WINDOW_SIZE];
    int pressure_buffer_index;
    int pressure_buffer_count;
    float pressure_f0_kg;           /* 静止时压力值F0 */
    int pressure_f0_calibrated;     /* F0是否已校准 */
    float pressure_f0_sum;          /* F0校准期间压力累加和 */
    int pressure_f0_sample_count;   /* F0校准期间采样计数 */
    int pressure_stabilize_count;   /* 压力稳定延迟计数（算法启动后先稳定一段时间再校准） */
    
    /* 离合器电流PID控制 - 增量式PID */
    float clutch_pi_integral;       /* PID积分项 */
    float clutch_pi_derivative;     /* PID微分项 */
    float last_deltaf;              /* 上一次的deltaf值（用于计算微分） */
    float last_last_deltaf;         /* 上上一次的deltaf值（用于增量式微分计算） */
    float last_current_mA;          /* 上一时刻的电流值（增量式PID需要） */
    
    /* 线程控制 */
    pthread_t thread_id;
    int running;
    int paused;
    pthread_mutex_t mutex;
    
} GravityUnloadController_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化重力卸载控制器
 * @param ctrl 控制器实例
 * @return 0=成功, -1=失败
 */
int gravity_unload_init(GravityUnloadController_t *ctrl);

/**
 * @brief 反初始化重力卸载控制器
 * @param ctrl 控制器实例
 */
void gravity_unload_deinit(GravityUnloadController_t *ctrl);

/**
 * @brief 启动控制算法线程
 * @param ctrl 控制器实例
 * @return 0=成功, -1=失败
 */
int gravity_unload_start(GravityUnloadController_t *ctrl);

/**
 * @brief 停止控制算法线程
 * @param ctrl 控制器实例
 */
void gravity_unload_stop(GravityUnloadController_t *ctrl);

/**
 * @brief 暂停控制算法
 * @param ctrl 控制器实例
 */
void gravity_unload_pause(GravityUnloadController_t *ctrl);

/**
 * @brief 恢复控制算法
 * @param ctrl 控制器实例
 */
void gravity_unload_resume(GravityUnloadController_t *ctrl);

/**
 * @brief 紧急停止
 * @param ctrl 控制器实例
 * @param reason 停止原因
 */
void gravity_unload_emergency_stop(GravityUnloadController_t *ctrl, const char *reason);

/**
 * @brief 重置控制器状态
 * @param ctrl 控制器实例
 */
void gravity_unload_reset(GravityUnloadController_t *ctrl);

/**
 * @brief 执行一次控制周期（用于外部调用）
 * @param ctrl 控制器实例
 * @param raw_data 原始传感器数据
 * @param filtered_data 输出滤波后的数据
 * @param control_output 输出控制量
 * @return 状态码
 */
AlgoError_t gravity_unload_control_cycle(GravityUnloadController_t *ctrl,
                                          const SensorDataRaw_t *raw_data,
                                          SensorDataFiltered_t *filtered_data,
                                          ControlOutput_t *control_output);

/**
 * @brief 获取当前状态
 * @param ctrl 控制器实例
 * @param status 输出状态
 */
void gravity_unload_get_status(const GravityUnloadController_t *ctrl, AlgoStatus_t *status);

/**
 * @brief 打印控制器状态（调试）
 * @param ctrl 控制器实例
 */
void gravity_unload_print_status(const GravityUnloadController_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __GRAVITY_UNLOAD_H__ */
