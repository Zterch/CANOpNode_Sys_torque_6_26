/******************************************************************************
 * @file    weight_driver.h
 * @brief   重量采集驱动 - UART/TTL串口控制，与电源板协议兼容
 * @author  System Architect
 * @date    2026-06-03
 * @version 1.0.0
 * 
 * 协议说明：
 * - 设备地址：0xAA
 * - 功能码：0x01（读取），0x06（写入）
 * - 数据格式：高字节在前
 * - CRC16：高字节在前
 * 
 * 示例帧：
 * 读取重量：AA 01 01 02 00 02 CRC_H CRC_L
 * 返回数据：AA 01 01 02 XX XX CRC_H CRC_L (重量值，单位0.01kg)
 ******************************************************************************/

#ifndef __WEIGHT_DRIVER_H__
#define __WEIGHT_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 寄存器地址定义
 ******************************************************************************/
#define WEIGHT_REG_WEIGHT_FILTERED  0x0102  /* 读取滤波后重量 (0.01kg单位) */
#define WEIGHT_REG_VOLTAGE_FILTERED 0x0304  /* 读取滤波后电压 (mV) */
#define WEIGHT_REG_WEIGHT_RAW       0x0506  /* 读取原始重量 (0.01kg单位) */
#define WEIGHT_REG_VOLTAGE_RAW      0x0708  /* 读取原始电压 (mV) */
#define WEIGHT_REG_ADC_RAW          0x090A  /* 读取原始ADC值 */
#define WEIGHT_REG_ADC_FILTERED     0x0B0C  /* 读取滤波后ADC值 */

/******************************************************************************
 * 重量采集状态定义
 ******************************************************************************/
typedef enum {
    WEIGHT_STATE_INIT = 0,      /* 初始化 */
    WEIGHT_STATE_READY,         /* 就绪 */
    WEIGHT_STATE_ERROR,         /* 错误 */
    WEIGHT_STATE_UNKNOWN        /* 未知 */
} WeightState_t;

/******************************************************************************
 * 重量数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    int fd;                     /* 串口文件描述符 */
    
    /* 状态信息 */
    WeightState_t state;        /* 设备状态 */
    
    /* 采集数据 */
    float weight_filtered;      /* 滤波后重量 (kg) */
    float weight_raw;           /* 原始重量 (kg) */
    float voltage_filtered;     /* 滤波后电压 (V) */
    float voltage_raw;          /* 原始电压 (V) */
    uint16_t adc_raw;           /* 原始ADC值 */
    uint16_t adc_filtered;      /* 滤波后ADC值 */
    uint32_t timestamp_ms;      /* 时间戳 */
    
    /* 统计 */
    uint32_t read_count;        /* 读取次数 */
    uint32_t error_count;       /* 错误次数 */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
    
    /* 后台采集线程 */
    pthread_t collect_thread;       /* 采集线程ID */
    int thread_running;             /* 线程运行标志 */
    uint32_t sample_count;          /* 采样计数 */
} WeightDriver_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化重量采集驱动
 * @param weight 重量驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @return ErrorCode_t
 */
ErrorCode_t weight_init(WeightDriver_t *weight, const char *device, int baudrate);

/**
 * @brief 反初始化重量采集驱动
 * @param weight 重量驱动结构指针
 */
void weight_deinit(WeightDriver_t *weight);

/**
 * @brief 读取重量数据（滤波后）
 * @param weight 重量驱动结构指针
 * @param weight_kg 重量输出指针 (kg)
 * @return ErrorCode_t
 */
ErrorCode_t weight_get_weight(WeightDriver_t *weight, float *weight_kg);

/**
 * @brief 读取原始重量数据
 * @param weight 重量驱动结构指针
 * @param weight_kg 重量输出指针 (kg)
 * @return ErrorCode_t
 */
ErrorCode_t weight_get_weight_raw(WeightDriver_t *weight, float *weight_kg);

/**
 * @brief 读取电压数据
 * @param weight 重量驱动结构指针
 * @param voltage_v 电压输出指针 (V)
 * @return ErrorCode_t
 */
ErrorCode_t weight_get_voltage(WeightDriver_t *weight, float *voltage_v);

/**
 * @brief 读取所有数据
 * @param weight 重量驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t weight_get_all(WeightDriver_t *weight);

/**
 * @brief 获取重量数据（线程安全）
 * @param weight 重量驱动结构指针
 * @param weight_kg 重量输出指针 (kg)
 * @param is_filtered 是否获取滤波后数据 (1=滤波后, 0=原始)
 * @return ErrorCode_t
 */
ErrorCode_t weight_get_data(WeightDriver_t *weight, float *weight_kg, int is_filtered);

/**
 * @brief 启动后台采集线程（100Hz）
 * @param weight 重量驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t weight_start_collection(WeightDriver_t *weight);

/**
 * @brief 停止后台采集线程
 * @param weight 重量驱动结构指针
 */
void weight_stop_collection(WeightDriver_t *weight);

/**
 * @brief CRC16计算（重量采集协议 - 与电源板相同）
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return CRC16值（需要交换高低字节后发送）
 */
uint16_t weight_crc16(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __WEIGHT_DRIVER_H__ */
