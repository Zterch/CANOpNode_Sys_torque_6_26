/******************************************************************************
 * @file    pressure_driver.c
 * @brief   压力计驱动实现 - RS485接口压力传感器 (Modbus RTU)
 * @author  System Architect
 * @date    2026-04-20
 * @version 1.1.0
 * 
 * 协议说明：
 * - Modbus RTU协议
 * - 支持功能码03（读取）、06（写入）
 * - 寄存器0x00：压力值（带符号整数，需除以小数点位数对应的倍数）
 * - 寄存器0x01：小数点设置（0-3，表示保留几位小数）
 * 
 * 指令示例：
 * 读取压力：  02 03 00 00 00 01 CRC_L CRC_H
 * 设置小数点：02 06 00 01 00 03 CRC_L CRC_H （设为3位小数）
 ******************************************************************************/

#include "pressure_driver.h"
#include "rs485_bus.h"
#include "../utils/logger.h"
#include "../config/system_config.h"

#include <string.h>
#include <unistd.h>
#include <math.h>

/******************************************************************************
 * 内部函数：发送Modbus命令并接收响应
 ******************************************************************************/
static ErrorCode_t pressure_send_command(PressureDriver_t *pressure, uint8_t func_code,
                                          uint16_t reg_addr, uint16_t data,
                                          uint8_t *response, int resp_len, int *rx_len)
{
    uint8_t frame[8];
    uint16_t crc;
    
    /* 构建帧 */
    frame[0] = pressure->slave_addr;        /* 设备地址 */
    frame[1] = func_code;                   /* 功能码 */
    frame[2] = (reg_addr >> 8) & 0xFF;      /* 寄存器地址高字节 */
    frame[3] = reg_addr & 0xFF;             /* 寄存器地址低字节 */
    frame[4] = (data >> 8) & 0xFF;          /* 数据高字节 */
    frame[5] = data & 0xFF;                 /* 数据低字节 */
    
    /* 计算CRC */
    crc = crc16_modbus(frame, 6);
    frame[6] = crc & 0xFF;                  /* CRC低字节 */
    frame[7] = (crc >> 8) & 0xFF;           /* CRC高字节 */
    
    /* 获取RS485总线互斥锁 */
    pthread_mutex_t *bus_mutex = rs485_bus_get_mutex();
    pthread_mutex_lock(bus_mutex);
    
    /* 清空缓冲区 */
    rs485_bus_flush();
    
    /* Modbus RTU帧间隔 */
    usleep(5000);
    
    /* 发送请求 */
    ErrorCode_t ret = rs485_bus_send(frame, 8, 50);
    if (ret != ERR_OK) {
        pthread_mutex_unlock(bus_mutex);
        return ERR_COMM_FAIL;
    }
    
    /* 等待设备响应 */
    usleep(15000);
    
    /* 接收响应 */
    int received = 0;
    ret = rs485_bus_receive(response, resp_len, &received, 50);
    
    pthread_mutex_unlock(bus_mutex);
    
    if (ret != ERR_OK || received < 5) {
        return ERR_TIMEOUT;
    }
    
    if (rx_len) {
        *rx_len = received;
    }
    
    return ERR_OK;
}

/******************************************************************************
 * 公共接口实现
 ******************************************************************************/

ErrorCode_t pressure_init(PressureDriver_t *pressure, const char *device, 
                          int baudrate, uint8_t slave_addr)
{
    (void)device;   /* 串口由RS485总线管理器统一管理 */
    (void)baudrate;
    
    if (pressure == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(pressure, 0, sizeof(PressureDriver_t));
    
    pressure->slave_addr = slave_addr;
    pressure->decimal_places = PRESSURE_DECIMAL_PLACES;
    pressure->zero_offset = 0.0f;
    
    /* 初始化低通滤波器参数 - 截止频率9Hz，采样率50Hz
     * 使用一阶低通滤波器: alpha = 1 / (1 + 2*pi*fc/fs)
     * fc = 9Hz, fs = 50Hz
     * alpha = 1 / (1 + 2*pi*9/50) ≈ 0.469
     */
    pressure->lpf_alpha = 0.469f;
    pressure->lpf_initialized = 0;
    pressure->pressure_filtered = 0.0f;
    
    if (pthread_mutex_init(&pressure->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    /* 串口由RS485总线管理器统一管理 */
    pressure->fd = -1;
    pressure->initialized = 1;
    pressure->pressure = 0.0f;
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure driver initialized (addr=%d)", slave_addr);
    
    /* 设置小数点为3位 */
    if (pressure_set_decimal(pressure, PRESSURE_DECIMAL_PLACES) != ERR_OK) {
        LOG_WARN(LOG_MODULE_PRESSURE, "Failed to set decimal places to %d", PRESSURE_DECIMAL_PLACES);
    } else {
        LOG_INFO(LOG_MODULE_PRESSURE, "Decimal places set to %d", PRESSURE_DECIMAL_PLACES);
    }
    
    return ERR_OK;
}

void pressure_deinit(PressureDriver_t *pressure)
{
    if (pressure == NULL || !pressure->initialized) {
        return;
    }
    
    pthread_mutex_lock(&pressure->mutex);
    pressure->initialized = 0;
    pthread_mutex_unlock(&pressure->mutex);
    
    pthread_mutex_destroy(&pressure->mutex);
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure driver deinitialized");
}

ErrorCode_t pressure_set_decimal(PressureDriver_t *pressure, uint8_t decimal_places)
{
    if (pressure == NULL || !pressure->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    if (decimal_places > 3) {
        decimal_places = 3;
    }
    
    uint8_t response[16];
    int rx_len = 0;
    
    /* 发送设置小数点命令 */
    ErrorCode_t ret = pressure_send_command(pressure, 0x06, 
                                            PRESSURE_DECIMAL_REG_ADDR, 
                                            decimal_places, 
                                            response, sizeof(response), &rx_len);
    
    if (ret != ERR_OK) {
        LOG_WARN(LOG_MODULE_PRESSURE, "Failed to send decimal setting command");
        return ret;
    }
    
    /* 验证响应 */
    if (rx_len < 8) {
        return ERR_COMM_FAIL;
    }
    
    if (response[0] != pressure->slave_addr) {
        return ERR_COMM_FAIL;
    }
    
    if (response[1] != 0x06) {
        return ERR_COMM_FAIL;
    }
    
    /* 检查设置是否成功 */
    uint16_t set_decimal = ((uint16_t)response[4] << 8) | response[5];
    if (set_decimal != decimal_places) {
        LOG_WARN(LOG_MODULE_PRESSURE, "Decimal setting mismatch: expected %d, got %d", 
                 decimal_places, set_decimal);
        return ERR_COMM_FAIL;
    }
    
    pthread_mutex_lock(&pressure->mutex);
    pressure->decimal_places = decimal_places;
    pthread_mutex_unlock(&pressure->mutex);
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Decimal places set to %d successfully", decimal_places);
    return ERR_OK;
}

ErrorCode_t pressure_read(PressureDriver_t *pressure, float *value)
{
    if (pressure == NULL || !pressure->initialized || value == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t response[16];
    int rx_len = 0;
    
    /* 发送读取命令 */
    ErrorCode_t ret = pressure_send_command(pressure, 0x03,
                                            PRESSURE_MODBUS_REG_ADDR,
                                            0x0001,  /* 读取1个寄存器 */
                                            response, sizeof(response), &rx_len);
    
    if (ret != ERR_OK) {
        pressure->error_count++;
        return ret;
    }
    
    /* 验证响应 */
    if (rx_len < 7) {
        pressure->error_count++;
        return ERR_COMM_FAIL;
    }
    
    if (response[0] != pressure->slave_addr) {
        pressure->error_count++;
        return ERR_COMM_FAIL;
    }
    
    if (response[1] != 0x03) {
        pressure->error_count++;
        return ERR_COMM_FAIL;
    }
    
    if (response[2] != 2) {
        pressure->error_count++;
        return ERR_COMM_FAIL;
    }
    
    /* 解析压力值（16位有符号整数） */
    int16_t raw_value = (int16_t)(((uint16_t)response[3] << 8) | (uint16_t)response[4]);
    
    pthread_mutex_lock(&pressure->mutex);
    
    /* 根据小数点位数计算实际压力值 */
    float divisor = 1.0f;
    for (uint8_t i = 0; i < pressure->decimal_places; i++) {
        divisor *= 10.0f;
    }
    
    pressure->pressure = (raw_value / divisor) - pressure->zero_offset;
    *value = pressure->pressure;
    pressure->read_count++;
    
    pthread_mutex_unlock(&pressure->mutex);
    
    return ERR_OK;
}

ErrorCode_t pressure_zero_calibration(PressureDriver_t *pressure)
{
    if (pressure == NULL || !pressure->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    float current_value;
    
    /* 读取当前压力值作为零点偏移 */
    if (pressure_read(pressure, &current_value) != ERR_OK) {
        return ERR_COMM_FAIL;
    }
    
    pthread_mutex_lock(&pressure->mutex);
    pressure->zero_offset = current_value;
    pthread_mutex_unlock(&pressure->mutex);
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Zero calibration completed: offset=%.3f %s", 
             pressure->zero_offset, PRESSURE_UNIT);
    
    return ERR_OK;
}

ErrorCode_t pressure_read_filtered(PressureDriver_t *pressure, float *value)
{
    if (pressure == NULL || !pressure->initialized || value == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    /* 先读取原始压力值 */
    float raw_value;
    ErrorCode_t ret = pressure_read(pressure, &raw_value);
    if (ret != ERR_OK) {
        return ret;
    }
    
    pthread_mutex_lock(&pressure->mutex);
    
    /* 应用低通滤波器 - 一阶IIR滤波
     * y[n] = alpha * x[n] + (1-alpha) * y[n-1]
     * alpha = 0.469, 截止频率约9Hz @ 50Hz采样率
     */
    if (!pressure->lpf_initialized) {
        /* 第一次读取，直接赋值 */
        pressure->pressure_filtered = raw_value;
        pressure->lpf_initialized = 1;
    } else {
        /* 应用滤波器 */
        pressure->pressure_filtered = pressure->lpf_alpha * raw_value + 
                                      (1.0f - pressure->lpf_alpha) * pressure->pressure_filtered;
    }
    
    *value = pressure->pressure_filtered;
    
    pthread_mutex_unlock(&pressure->mutex);
    
    return ERR_OK;
}

void pressure_filter_reset(PressureDriver_t *pressure)
{
    if (pressure == NULL || !pressure->initialized) {
        return;
    }
    
    pthread_mutex_lock(&pressure->mutex);
    pressure->lpf_initialized = 0;
    pressure->pressure_filtered = 0.0f;
    pthread_mutex_unlock(&pressure->mutex);
    
    LOG_INFO(LOG_MODULE_PRESSURE, "Pressure filter reset");
}
