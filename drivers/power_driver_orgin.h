/******************************************************************************
 * @file    power_driver.h
 * @brief   电源板驱动 - UART/TTL串口控制电流输出
 * @author  System Architect
 * @date    2026-04-20
 * @version 1.0.0
 * 
 * 协议说明：
 * - 自定义协议，类似Modbus RTU
 * - 帧头：0xAA（设备地址）
 * - 功能码：0x01（读取），0x06（写入）
 * - CRC16校验，高字节在前
 ******************************************************************************/

#ifndef __POWER_DRIVER_H__
#define __POWER_DRIVER_H__

#include <stdint.h>
#include <pthread.h>
#include "../config/system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 寄存器地址定义
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
 * @brief 初始化电源板驱动
 * @param power 电源驱动结构指针
 * @param device 串口设备
 * @param baudrate 波特率
 * @return ErrorCode_t
 */
ErrorCode_t power_init(PowerDriver_t *power, const char *device, int baudrate);

/**
 * @brief 反初始化电源板驱动
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
 * @brief 同时读取电流和电压
 * @param power 电源驱动结构指针
 * @param current_ma 电流输出指针 (mA)
 * @param voltage_mv 电压输出指针 (mV)
 * @return ErrorCode_t
 */
ErrorCode_t power_get_status(PowerDriver_t *power, uint16_t *current_ma, uint16_t *voltage_mv);

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
 * @brief CRC16计算（电源板协议）
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return CRC16值（需要交换高低字节后发送）
 */
uint16_t power_crc16(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_DRIVER_H__ */
