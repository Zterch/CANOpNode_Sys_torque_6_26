/******************************************************************************
 * @file    sensor_manager.h
 * @brief   统一传感器管理器 - RS485传感器轮询管理
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 * 
 * 功能说明：
 * - 统一管理RS485总线上的多个传感器设备
 * - 通过时间片轮询方式读取设备数据
 * - 支持Modbus RTU协议，通过地址区分设备
 * - 50Hz统一更新频率，避免总线竞争
 ******************************************************************************/

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include <pthread.h>
#include "rs485_bus.h"
#include "../utils/logger.h"

/******************************************************************************
 * 宏定义
 ******************************************************************************/
#define PRESSURE_MA_WINDOW_SIZE         1         /* 压力传感器移动平均滤波窗口大小 - 200Hz采样，100点=500ms窗口 */
/* 传感器设备类型 */
typedef enum {
    SENSOR_TYPE_ENCODER = 0,    /* 编码器 */
    SENSOR_TYPE_PRESSURE,       /* 压力传感器 */
    SENSOR_TYPE_COUNT           /* 传感器类型数量 */
} SensorType_t;

/* 传感器配置 */
typedef struct {
    SensorType_t type;          /* 传感器类型 */
    uint8_t slave_addr;         /* Modbus设备地址 */
    uint16_t reg_addr;          /* 寄存器起始地址 */
    uint16_t reg_count;         /* 寄存器数量 */
    uint8_t func_code;          /* 功能码 (0x03或0x04) */
    uint8_t decimal_places;     /* 小数点位数 (用于压力传感器) */
    uint32_t read_timeout_ms;   /* 读取超时时间 */
    const char* name;           /* 设备名称 */
} SensorConfig_t;

/* 传感器数据 */
typedef struct {
    union {
        /* 编码器数据 */
        struct {
            uint32_t multi_turn_value;      /* 多圈值 - 无符号32位整数 (0~2147483647) */
            float angle_deg;                /* 角度值 */
            float rope_length_mm;           /* 绳子长度 */
            int32_t pulse_delta;            /* 脉冲变化量（用于M/T法测速） */
            uint32_t time_delta_us;         /* 时间变化量（微秒，用于M/T法测速） */
        } encoder;
        
        /* 压力传感器数据 */
        struct {
            float pressure_kg;              /* 压力值 (kg) - 原始值 */
            float pressure_filtered_kg;     /* 压力值 (kg) - 低通滤波后 */
            int16_t raw_value;              /* 原始有符号值 */
            int16_t zero_offset;            /* 去皮偏移值 */
        } pressure;
    } data;
    
    uint32_t read_count;        /* 成功读取次数 */
    uint32_t error_count;       /* 错误次数 */
    uint64_t last_read_us;      /* 上次读取时间 (微秒) */
    int data_valid;             /* 数据是否有效 */
} SensorData_t;

/* 传感器管理器 */
typedef struct {
    /* 设备配置 */
    SensorConfig_t configs[SENSOR_TYPE_COUNT];
    SensorData_t datas[SENSOR_TYPE_COUNT];
    
    /* RS485总线 */
    int rs485_fd;
    pthread_mutex_t bus_mutex;  /* 总线互斥锁 */
    
    /* 运行状态 */
    int initialized;
    int running;
    
    /* 线程 */
    pthread_t manager_thread;
    
    /* 统计 */
    uint64_t cycle_count;       /* 轮询周期计数 */
    uint64_t total_errors;      /* 总错误数 */
} SensorManager_t;

/******************************************************************************
 * 公共接口
 ******************************************************************************/

/**
 * @brief 初始化传感器管理器
 * @param manager 管理器实例指针
 * @param device RS485设备路径
 * @param baudrate 波特率
 * @return ErrorCode_t
 */
ErrorCode_t sensor_mgr_init(SensorManager_t *manager, const char *device, int baudrate);

/**
 * @brief 反初始化传感器管理器
 * @param manager 管理器实例指针
 */
void sensor_mgr_deinit(SensorManager_t *manager);

/**
 * @brief 启动传感器管理线程
 * @param manager 管理器实例指针
 * @return ErrorCode_t
 */
ErrorCode_t sensor_mgr_start(SensorManager_t *manager);

/**
 * @brief 停止传感器管理线程
 * @param manager 管理器实例指针
 */
void sensor_mgr_stop(SensorManager_t *manager);

/**
 * @brief 读取传感器数据（线程安全）
 * @param manager 管理器实例指针
 * @param type 传感器类型
 * @param data 数据输出指针
 * @return ErrorCode_t
 */
ErrorCode_t sensor_mgr_get_data(SensorManager_t *manager, SensorType_t type, SensorData_t *data);

/**
 * @brief 设置编码器绳子长度参数
 * @param manager 管理器实例指针
 * @param drum_diameter 卷筒直径(mm)
 * @param resolution 编码器分辨率
 * @return ErrorCode_t
 */
ErrorCode_t sensor_mgr_set_encoder_rope_params(SensorManager_t *manager, 
                                                float drum_diameter, 
                                                uint32_t resolution);

/**
 * @brief 执行编码器零点校准
 * @param manager 管理器实例指针
 * @return ErrorCode_t
 */
ErrorCode_t sensor_mgr_encoder_zero_calibration(SensorManager_t *manager);

/**
 * @brief 执行压力传感器去皮/清零
 * @param manager 管理器实例指针
 * @return ErrorCode_t
 */
ErrorCode_t sensor_mgr_pressure_tare(SensorManager_t *manager);

/**
 * @brief 设置编码器基准长度
 * @param manager 管理器实例指针
 * @param base_length_mm 基准长度(mm)
 * @return ErrorCode_t
 */
ErrorCode_t sensor_mgr_set_encoder_base_length(SensorManager_t *manager, float base_length_mm);

/**
 * @brief 获取传感器读取成功率
 * @param manager 管理器实例指针
 * @param type 传感器类型
 * @return 成功率 (0.0 - 100.0)
 */
float sensor_mgr_get_success_rate(SensorManager_t *manager, SensorType_t type);

/**
 * @brief 打印所有传感器状态（调试用）
 * @param manager 管理器实例指针
 */
void sensor_mgr_print_status(SensorManager_t *manager);

/******************************************************************************
 * 全局变量声明 - 压力传感器陷波滤波器控制
 ******************************************************************************/

/**
 * @brief 压力传感器11Hz陷波滤波器开关
 * @details
 * 0: 关闭陷波滤波器（默认）
 * 1: 开启陷波滤波器
 *
 * 陷波滤波器参数（采样频率200Hz）：
 * - 中心频率: 11Hz
 * - 品质因数 Q = 1.7
 * - 10-12Hz衰减 > 85%
 * - 11Hz中心频率衰减 > 95%
 * - 5Hz相移 < 8°
 */
extern int g_pressure_notch_filter_enabled;

/**
 * @brief 编码器使能开关（用于将RS485带宽全部让给压力传感器）
 * @details
 * 0: 禁用编码器读取，压力传感器独占RS485总线，可运行200Hz
 * 1: 启用编码器读取，编码器+压力传感器共享RS485总线，各运行100Hz
 *
 * 修改方式：直接在代码中修改此全局变量的值，或在 sensor_manager.c 初始化前设置
 */
extern int g_encoder_enabled;

#endif /* SENSOR_MANAGER_H */
