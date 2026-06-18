/******************************************************************************
 * @file    power_driver_v2.h
 * @brief   电源板驱动V2 - 适配固定帧长协议（支持100Hz通信）
 * @author  System Architect
 * @date    2026-06-12
 * @version 2.0.0
 * 
 * @description
 * 工业级100Hz双向通信驱动
 * - 固定8字节帧长，无字符超时等待
 * - 单次通信约1.5ms，支持100Hz控制周期
 * - 新增批量读写功能（0x10），单次完成设置+读取
 ******************************************************************************/

#ifndef __POWER_DRIVER_V2_H__
#define __POWER_DRIVER_V2_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 协议定义
 ******************************************************************************/
#define POWER_V2_FRAME_LENGTH   8       /* 固定帧长8字节 */
#define POWER_V2_ADDR           0xAA    /* 设备地址 */

/* 功能码定义 */
#define POWER_FUNC_READ_IOUT    0x01    /* 读取电流 */
#define POWER_FUNC_READ_VOUT    0x02    /* 读取电压 */
#define POWER_FUNC_SET_CURRENT  0x06    /* 设置电流 */
#define POWER_FUNC_BATCH_RW     0x10    /* 批量读写（设置+读取） */

/******************************************************************************
 * 寄存器地址定义（兼容旧协议）
 ******************************************************************************/
#define POWER_REG_VOUT          0x09A0  /* 读取输出电压 */
#define POWER_REG_IOUT          0xA1A2  /* 读取输出电流 */
#define POWER_REG_VSET          0x0102  /* 设置目标电压 */
#define POWER_REG_ISET          0x0304  /* 设置目标电流 */
#define POWER_REG_OVP_SET       0x0506  /* 设置过压保护 */
#define POWER_REG_OCP_SET       0x0708  /* 设置过流保护 */
#define POWER_REG_OVP_STATE     0xA3A4  /* 读取OVP状态 */
#define POWER_REG_OCP_STATE     0xA5A6  /* 读取OCP状态 */

/******************************************************************************
 * 电源板状态定义
 ******************************************************************************/
typedef enum {
    POWER_STATE_INIT = 0,       /* 初始化 */
    POWER_STATE_OFF,            /* 关闭 */
    POWER_STATE_ON,             /* 开启 */
    POWER_STATE_FAULT,          /* 故障 */
    POWER_STATE_UNKNOWN         /* 未知 */
} PowerState_t;

/******************************************************************************
 * 电源板数据结构
 ******************************************************************************/
typedef struct {
    /* 配置参数 */
    char device[32];            /* 串口设备 */
    int baudrate;               /* 波特率 */
    int fd;                     /* 串口文件描述符 */
    
    /* 状态信息 */
    PowerState_t state;         /* 电源状态 */
    uint16_t current_setpoint;  /* 设定电流 (mA) */
    uint16_t actual_current;    /* 实际电流 (mA) */
    uint16_t actual_voltage;    /* 实际电压 (mV) */
    
    /* 统计 */
    uint32_t read_count;        /* 读取次数 */
    uint32_t error_count;       /* 错误次数 */
    
    /* 线程安全 */
    pthread_mutex_t mutex;
    int initialized;
} PowerDriver_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化电源板驱动V2
 * @param power 电源驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @return ErrorCode_t
 */
ErrorCode_t power_init(PowerDriver_t *power, const char *device, int baudrate);

/**
 * @brief 反初始化电源板驱动V2
 * @param power 电源驱动结构指针
 */
void power_deinit(PowerDriver_t *power);

/**
 * @brief 设置输出电流
 * @param power 电源驱动结构指针
 * @param current_ma 电流值 (mA), 范围 50-4000
 * @return ErrorCode_t
 */
ErrorCode_t power_set_current(PowerDriver_t *power, uint16_t current_ma);

/**
 * @brief 读取实际电流
 * @param power 电源驱动结构指针
 * @param current_ma 电流输出指针 (mA)
 * @return ErrorCode_t
 */
ErrorCode_t power_get_current(PowerDriver_t *power, uint16_t *current_ma);

/**
 * @brief 读取实际电压
 * @param power 电源驱动结构指针
 * @param voltage_mv 电压输出指针 (mV)
 * @return ErrorCode_t
 */
ErrorCode_t power_get_voltage(PowerDriver_t *power, uint16_t *voltage_mv);

/**
 * @brief 同时读取电流和电压（实际通信）
 * @param power 电源驱动结构指针
 * @param current_ma 电流输出指针 (mA)
 * @param voltage_mv 电压输出指针 (mV)
 * @return ErrorCode_t
 */
ErrorCode_t power_get_status(PowerDriver_t *power, uint16_t *current_ma, uint16_t *voltage_mv);

/**
 * @brief 从缓存读取电流和电压（非阻塞）
 * @param power 电源驱动结构指针
 * @param current_ma 电流输出指针 (mA), 可为NULL
 * @param voltage_mv 电压输出指针 (mV), 可为NULL
 * @return ErrorCode_t
 * @note 此函数只读取缓存值，不执行实际通信，适用于实时控制循环
 */
ErrorCode_t power_get_status_cached(PowerDriver_t *power, uint16_t *current_ma, uint16_t *voltage_mv);

/**
 * @brief 【V2新增】批量控制 - 单次通信完成设置电流+读取电流电压
 * @param power 电源驱动结构指针
 * @param target_current_ma 目标电流 (mA), 范围 50-4000
 * @param actual_current_ma 实际电流输出指针 (mA), 可为NULL
 * @param voltage_mv 实际电压输出指针 (mV), 可为NULL
 * @return ErrorCode_t
 * 
 * @note 这是100Hz控制周期推荐使用的接口，单次通信约1.5ms
 */
ErrorCode_t power_batch_control(PowerDriver_t *power, uint16_t target_current_ma,
                                 uint16_t *actual_current_ma, uint16_t *voltage_mv);

/**
 * @brief 开启电源输出
 * @param power 电源驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t power_on(PowerDriver_t *power);

/**
 * @brief 关闭电源输出
 * @param power 电源驱动结构指针
 * @return ErrorCode_t
 */
ErrorCode_t power_off(PowerDriver_t *power);

/**
 * @brief CRC16计算（电源板协议 - 高字节在前）
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return CRC16值
 */
uint16_t power_crc16(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_DRIVER_V2_H__ */
