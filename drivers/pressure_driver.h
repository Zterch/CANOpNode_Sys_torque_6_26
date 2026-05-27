/******************************************************************************
 * @file    pressure_driver.h
 * @brief   压力计驱动 - RS485接口压力传感器 (Modbus RTU)
 * @author  System Architect
 * @date    2026-04-20
 * @version 1.1.0
 * 
 * 协议说明：
 * - Modbus RTU协议，功能码03（读取）、06（写入）
 * - 寄存器0x00：压力值
 * - 寄存器0x01：小数点设置（3=保留3位小数）
 ******************************************************************************/

#ifndef __PRESSURE_DRIVER_H__
#define __PRESSURE_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 压力计数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    uint8_t slave_addr;         /* RS485设备地址 */
    int fd;                     /* 串口文件描述符 */
    uint8_t decimal_places;     /* 小数点位数 */
    
    /* 数据 */
    float pressure;             /* 压力值 (kg) */
    float pressure_filtered;    /* 滤波后的压力值 (kg) */
    float zero_offset;          /* 零点偏移 */
    
    /* 低通滤波器 - 截止频率9Hz，采样率50Hz */
    float lpf_alpha;            /* 滤波系数 */
    int lpf_initialized;        /* 滤波器是否已初始化 */
    
    /* 统计 */
    uint32_t read_count;        /* 读取次数 */
    uint32_t error_count;       /* 错误次数 */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
} PressureDriver_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化压力计驱动
 * @param pressure 压力计驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @param slave_addr 设备地址
 * @return ErrorCode_t
 */
ErrorCode_t pressure_init(PressureDriver_t *pressure, const char *device, 
                          int baudrate, uint8_t slave_addr);

/**
 * @brief 反初始化压力计驱动
 * @param pressure 压力计驱动结构指针
 */
void pressure_deinit(PressureDriver_t *pressure);

/**
 * @brief 设置小数点位数（功能码06）
 * @param pressure 压力计驱动结构指针
 * @param decimal_places 小数点位数 (0-3)
 * @return ErrorCode_t
 */
ErrorCode_t pressure_set_decimal(PressureDriver_t *pressure, uint8_t decimal_places);

/**
 * @brief 读取压力值
 * @param pressure 压力计驱动结构指针
 * @param value 压力输出指针 (kg)
 * @return ErrorCode_t
 */
ErrorCode_t pressure_read(PressureDriver_t *pressure, float *value);

/**
 * @brief 零点校准
 * @param pressure 压力计驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t pressure_zero_calibration(PressureDriver_t *pressure);

/**
 * @brief 读取滤波后的压力值（低通滤波，截止频率9Hz）
 * @param pressure 压力计驱动结构指针
 * @param value 压力输出指针 (kg)
 * @return ErrorCode_t
 */
ErrorCode_t pressure_read_filtered(PressureDriver_t *pressure, float *value);

/**
 * @brief 重置压力低通滤波器
 * @param pressure 压力计驱动结构指针
 */
void pressure_filter_reset(PressureDriver_t *pressure);

#ifdef __cplusplus
}
#endif

#endif /* __PRESSURE_DRIVER_H__ */
