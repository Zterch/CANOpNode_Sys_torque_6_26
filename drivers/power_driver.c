/******************************************************************************
 * @file    power_driver_v2.c
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
 * 
 * 协议说明：
 * - 设备地址：0xAA
 * - 功能码：0x01（读电流），0x02（读电压），0x06（写电流），0x10（批量读写）
 * - 帧格式：固定8字节 [地址|功能码|数据1H|数据1L|数据2H|数据2L|CRC_H|CRC_L]
 * - CRC16：高字节在前（标准Modbus）
 ******************************************************************************/

#include "power_driver.h"
#include "../utils/logger.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <pthread.h>

static pthread_mutex_t g_power_mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************
 * 配置宏定义
 ******************************************************************************/
#define FRAME_FIXED_LENGTH      8
#define FRAME_ADDR              0xAA
#define FUNC_READ_IOUT          0x01
#define FUNC_READ_VOUT          0x02
#define FUNC_SET_CURRENT        0x06
#define FUNC_BATCH_RW           0x10    /* 批量读写：设置电流+读取电流电压 */

#define COMM_TIMEOUT_MS         5       /* 通信超时5ms */
#define FRAME_SYNC_TIMEOUT_US   1000    /* 帧同步等待1ms */

/******************************************************************************
 * CRC16计算（电源板协议 - 高字节在前）
 ******************************************************************************/
uint16_t power_crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    
    for (j = 0; j < len; j++) {
        crc = crc ^ data[j];
        for (i = 0; i < 8; i++) {
            if ((crc & 0x0001) > 0) {
                crc = crc >> 1;
                crc = crc ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    /* 电源板协议要求CRC高字节在前 */
    return (crc << 8) | (crc >> 8);
}

/******************************************************************************
 * 串口操作函数
 ******************************************************************************/
static int serial_open(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        LOG_ERROR(LOG_MODULE_POWER, "Failed to open %s: %s", device, strerror(errno));
        return -1;
    }
    
    /* 清除非阻塞标志 */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0) {
        LOG_ERROR(LOG_MODULE_POWER, "tcgetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    
    /* 设置波特率 */
    speed_t baud = B115200;
    switch (baudrate) {
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        default:     baud = B115200; break;
    }
    
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    /* 8N1 */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    
    /* 原始模式 */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    
    /* 最小读取字节数和超时 - 关键配置 */
    tty.c_cc[VMIN] = 0;     /* 非阻塞 */
    tty.c_cc[VTIME] = 0;    /* 无超时 */
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        LOG_ERROR(LOG_MODULE_POWER, "tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    LOG_INFO(LOG_MODULE_POWER, "Serial port %s opened at %d baud (V2 protocol)", device, baudrate);
    return fd;
}

static void serial_close(int fd) {
    if (fd >= 0) {
        tcflush(fd, TCIOFLUSH);
        close(fd);
    }
}

// static int serial_send(int fd, const uint8_t *data, int len) {
//     if (fd < 0 || data == NULL || len <= 0) {
//         return -1;
//     }
    
//     int ret = write(fd, data, len);
//     if (ret != len) {
//         return -1;
//     }
    
//     /* 等待数据发送完成 */
//     tcdrain(fd);
//     return ret;
// }
static int serial_send(int fd, const uint8_t *data, int len) {
    int ret = -1;
    
    pthread_mutex_lock(&g_power_mutex);
    
    if (fd >= 0 && data != NULL && len > 0) {
        ret = write(fd, data, len);
        if (ret == len) {
            tcdrain(fd);  /* 等待数据发送完成 */
        } else {
            ret = -1;
        }
    }
    
    pthread_mutex_unlock(&g_power_mutex);
    return ret;
}

/******************************************************************************
 * 接收固定帧（8字节）
 ******************************************************************************/
// static int serial_receive_frame(int fd, uint8_t *frame, int timeout_ms) {
//     if (fd < 0 || frame == NULL) {
//         return -1;
//     }
    
//     fd_set readfds;
//     struct timeval tv;
//     int total_received = 0;
//     int ret;
    
//     /* 阶段1：等待帧同步（等待0xAA） */
//     int sync_timeout_us = timeout_ms * 1000;
//     int elapsed_us = 0;
    
//     while (elapsed_us < sync_timeout_us) {
//         FD_ZERO(&readfds);
//         FD_SET(fd, &readfds);
        
//         tv.tv_sec = 0;
//         tv.tv_usec = 100;  /* 100us轮询 */
        
//         ret = select(fd + 1, &readfds, NULL, NULL, &tv);
//         if (ret > 0) {
//             ret = read(fd, &frame[0], 1);
//             if (ret == 1 && frame[0] == FRAME_ADDR) {
//                 total_received = 1;
//                 break;
//             }
//         }
//         elapsed_us += 100;
//     }
    
//     if (total_received == 0) {
//         return -1;  /* 同步超时 */
//     }
    
//     /* 阶段2：接收剩余7字节 */
//     int remaining_timeout_ms = timeout_ms - (elapsed_us / 1000);
//     if (remaining_timeout_ms < 1) remaining_timeout_ms = 1;
    
//     while (total_received < FRAME_FIXED_LENGTH) {
//         FD_ZERO(&readfds);
//         FD_SET(fd, &readfds);
        
//         tv.tv_sec = remaining_timeout_ms / 1000;
//         tv.tv_usec = (remaining_timeout_ms % 1000) * 1000;
        
//         ret = select(fd + 1, &readfds, NULL, NULL, &tv);
//         if (ret <= 0) {
//             return -1;  /* 接收超时 */
//         }
        
//         ret = read(fd, &frame[total_received], FRAME_FIXED_LENGTH - total_received);
//         if (ret > 0) {
//             total_received += ret;
//         } else if (ret < 0 && errno != EAGAIN) {
//             return -1;
//         }
//     }
    
//     return total_received;
// }
static int serial_receive_frame(int fd, uint8_t *frame, int timeout_ms) {
    if (fd < 0 || frame == NULL) return -1;
    
    pthread_mutex_lock(&g_power_mutex);
    
    fd_set readfds;
    struct timeval tv;
    int total_received = 0;
    int ret;
    
    /* 阶段1：等待帧同步（等待0xAA） */
    int sync_timeout_us = timeout_ms * 1000;
    int elapsed_us = 0;
    
    while (elapsed_us < sync_timeout_us) {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 100;
        
        ret = select(fd + 1, &readfds, NULL, NULL, &tv);
        if (ret > 0) {
            ret = read(fd, &frame[0], 1);
            if (ret == 1 && frame[0] == 0xAA) {  /* 0xAA 是您的设备地址 */
                total_received = 1;
                break;
            }
        }
        elapsed_us += 100;
    }
    
    if (total_received == 0) {
        pthread_mutex_unlock(&g_power_mutex);
        return -1;
    }
    
    /* 阶段2：接收剩余7字节 */
    int remaining_timeout_ms = timeout_ms - (elapsed_us / 1000);
    if (remaining_timeout_ms < 1) remaining_timeout_ms = 1;
    
    while (total_received < 8) {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        tv.tv_sec = remaining_timeout_ms / 1000;
        tv.tv_usec = (remaining_timeout_ms % 1000) * 1000;
        
        ret = select(fd + 1, &readfds, NULL, NULL, &tv);
        if (ret <= 0) {
            pthread_mutex_unlock(&g_power_mutex);
            return -1;
        }
        
        ret = read(fd, &frame[total_received], 8 - total_received);
        if (ret > 0) {
            total_received += ret;
        } else if (ret < 0 && errno != EAGAIN) {
            pthread_mutex_unlock(&g_power_mutex);
            return -1;
        }
    }
    
    pthread_mutex_unlock(&g_power_mutex);
    return total_received;
}

// static void serial_flush(int fd) {
//     if (fd >= 0) {
//         tcflush(fd, TCIOFLUSH);
//     }
// }
static void serial_flush(int fd) {
    if (fd >= 0) {
        pthread_mutex_lock(&g_power_mutex);
        tcflush(fd, TCIOFLUSH);
        pthread_mutex_unlock(&g_power_mutex);
    }
}

/******************************************************************************
 * 构建命令帧
 ******************************************************************************/
static void build_frame(uint8_t *frame, uint8_t func_code, 
                        uint16_t data1, uint16_t data2) {
    frame[0] = FRAME_ADDR;
    frame[1] = func_code;
    frame[2] = (data1 >> 8) & 0xFF;
    frame[3] = data1 & 0xFF;
    frame[4] = (data2 >> 8) & 0xFF;
    frame[5] = data2 & 0xFF;
    
    uint16_t crc = power_crc16(frame, 6);
    frame[6] = (crc >> 8) & 0xFF;
    frame[7] = crc & 0xFF;
}

/******************************************************************************
 * 验证响应帧
 ******************************************************************************/
static int verify_response(const uint8_t *response, uint8_t expected_func) {
    /* 检查地址 */
    if (response[0] != FRAME_ADDR) {
        return -1;
    }
    
    /* 检查功能码（支持错误码0x8X） */
    if (response[1] != expected_func && response[1] != (expected_func | 0x80)) {
        return -1;
    }
    
    /* 检查CRC */
    uint16_t crc_received = ((uint16_t)response[6] << 8) | response[7];
    uint16_t crc_calculated = power_crc16(response, 6);
    if (crc_received != crc_calculated) {
        return -1;
    }
    
    return 0;
}

/******************************************************************************
 * 公共接口实现
 ******************************************************************************/

ErrorCode_t power_init(PowerDriver_t *power, const char *device, int baudrate) {
    if (power == NULL || device == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(power, 0, sizeof(PowerDriver_t));
    
    strncpy(power->device, device, sizeof(power->device) - 1);
    power->baudrate = baudrate;
    power->state = POWER_STATE_INIT;
    power->current_setpoint = 0.0;  /* 默认0.44A */
    
    if (pthread_mutex_init(&power->mutex, NULL) != 0) {
        return ERR_GENERAL;
    }
    
    /* 打开串口 */
    power->fd = serial_open(device, baudrate);
    if (power->fd < 0) {
        pthread_mutex_destroy(&power->mutex);
        return ERR_DEVICE_NOT_FOUND;
    }
    
    power->initialized = 1;
    power->state = POWER_STATE_ON;
    
    LOG_INFO(LOG_MODULE_POWER, "Power driver V2 initialized (device=%s, baud=%d)", 
             device, baudrate);
    
    return ERR_OK;
}

void power_deinit(PowerDriver_t *power) {
    if (power == NULL || !power->initialized) {
        return;
    }
    
    pthread_mutex_lock(&power->mutex);
    
    serial_close(power->fd);
    power->fd = -1;
    power->initialized = 0;
    power->state = POWER_STATE_OFF;
    
    pthread_mutex_unlock(&power->mutex);
    pthread_mutex_destroy(&power->mutex);
    
    LOG_INFO(LOG_MODULE_POWER, "Power driver V2 deinitialized");
}

/******************************************************************************
 * 设置电流（兼容旧协议）
 ******************************************************************************/
ErrorCode_t power_set_current(PowerDriver_t *power, uint16_t current_ma) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 限制范围 50-4000mA */
    if (current_ma < 50) current_ma = 50;
    if (current_ma > 1000) current_ma = 1000;
    
    uint8_t request[FRAME_FIXED_LENGTH];
    uint8_t response[FRAME_FIXED_LENGTH];
    
    pthread_mutex_lock(&power->mutex);
    
    /* 清空缓冲区 */
    serial_flush(power->fd);
    
    /* 构建设置电流帧 */
    build_frame(request, FUNC_SET_CURRENT, 0x0304, current_ma);
    
    /* 发送 */
    if (serial_send(power->fd, request, FRAME_FIXED_LENGTH) != FRAME_FIXED_LENGTH) {
        pthread_mutex_unlock(&power->mutex);
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    /* 接收响应 */
    int rx_len = serial_receive_frame(power->fd, response, COMM_TIMEOUT_MS);
    
    pthread_mutex_unlock(&power->mutex);
    
    if (rx_len != FRAME_FIXED_LENGTH) {
        power->error_count++;
        return ERR_TIMEOUT;
    }
    
    /* 验证响应 */
    if (verify_response(response, FUNC_SET_CURRENT) != 0) {
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    /* 更新状态 */
    pthread_mutex_lock(&power->mutex);
    uint16_t old_setpoint = power->current_setpoint;
    power->current_setpoint = current_ma;
    pthread_mutex_unlock(&power->mutex);
    
    /* 日志 */
    static uint32_t last_log_time = 0;
    uint32_t now = get_timestamp_ms();
    if (abs((int)current_ma - (int)old_setpoint) > 5 || (now - last_log_time) > 1000) {
        LOG_INFO(LOG_MODULE_POWER, "Current set to %d mA (%.2f A)", current_ma, current_ma / 1000.0f);
        last_log_time = now;
    }
    
    return ERR_OK;
}

/******************************************************************************
 * 读取电流（V2优化版）
 ******************************************************************************/
ErrorCode_t power_get_current(PowerDriver_t *power, uint16_t *current_ma) {
    if (power == NULL || !power->initialized || current_ma == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t request[FRAME_FIXED_LENGTH];
    uint8_t response[FRAME_FIXED_LENGTH];
    
    pthread_mutex_lock(&power->mutex);
    
    serial_flush(power->fd);
    
    /* 构建读取电流帧 */
    build_frame(request, FUNC_READ_IOUT, 0x09A0, 0x0002);
    
    if (serial_send(power->fd, request, FRAME_FIXED_LENGTH) != FRAME_FIXED_LENGTH) {
        pthread_mutex_unlock(&power->mutex);
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    int rx_len = serial_receive_frame(power->fd, response, COMM_TIMEOUT_MS);
    
    pthread_mutex_unlock(&power->mutex);
    
    if (rx_len != FRAME_FIXED_LENGTH) {
        power->error_count++;
        return ERR_TIMEOUT;
    }
    
    if (verify_response(response, FUNC_READ_IOUT) != 0) {
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    /* 解析电流值 */
    uint16_t current = ((uint16_t)response[2] << 8) | response[3];
    
    pthread_mutex_lock(&power->mutex);
    power->actual_current = current;
    power->read_count++;
    pthread_mutex_unlock(&power->mutex);
    
    *current_ma = current;
    
    return ERR_OK;
}

/******************************************************************************
 * 读取电压（V2优化版）
 ******************************************************************************/
ErrorCode_t power_get_voltage(PowerDriver_t *power, uint16_t *voltage_mv) {
    if (power == NULL || !power->initialized || voltage_mv == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t request[FRAME_FIXED_LENGTH];
    uint8_t response[FRAME_FIXED_LENGTH];
    
    pthread_mutex_lock(&power->mutex);
    
    serial_flush(power->fd);
    
    /* 构建读取电压帧 */
    build_frame(request, FUNC_READ_VOUT, 0x09A0, 0x0002);
    
    if (serial_send(power->fd, request, FRAME_FIXED_LENGTH) != FRAME_FIXED_LENGTH) {
        pthread_mutex_unlock(&power->mutex);
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    int rx_len = serial_receive_frame(power->fd, response, COMM_TIMEOUT_MS);
    
    pthread_mutex_unlock(&power->mutex);
    
    if (rx_len != FRAME_FIXED_LENGTH) {
        power->error_count++;
        return ERR_TIMEOUT;
    }
    
    if (verify_response(response, FUNC_READ_VOUT) != 0) {
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    /* 解析电压值 */
    uint16_t voltage = ((uint16_t)response[2] << 8) | response[3];
    
    pthread_mutex_lock(&power->mutex);
    power->actual_voltage = voltage;
    pthread_mutex_unlock(&power->mutex);
    
    *voltage_mv = voltage;
    
    return ERR_OK;
}

/******************************************************************************
 * 【新增】批量读写 - 单次通信完成设置电流+读取电流电压（100Hz推荐）
 ******************************************************************************/
ErrorCode_t power_batch_control(PowerDriver_t *power, uint16_t target_current_ma,
                                 uint16_t *actual_current_ma, uint16_t *voltage_mv) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 限制电流范围 */
    if (target_current_ma < 50) target_current_ma = 50;
    if (target_current_ma > 1000) target_current_ma = 1000;
    
    uint8_t request[FRAME_FIXED_LENGTH];
    uint8_t response[FRAME_FIXED_LENGTH];
    
    pthread_mutex_lock(&power->mutex);
    
    serial_flush(power->fd);
    
    /* 构建批量读写帧 */
    /* 数据1：目标电流，数据2：保留 */
    build_frame(request, FUNC_BATCH_RW, target_current_ma, 0);
    
    if (serial_send(power->fd, request, FRAME_FIXED_LENGTH) != FRAME_FIXED_LENGTH) {
        pthread_mutex_unlock(&power->mutex);
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    int rx_len = serial_receive_frame(power->fd, response, COMM_TIMEOUT_MS);
    
    pthread_mutex_unlock(&power->mutex);
    
    if (rx_len != FRAME_FIXED_LENGTH) {
        power->error_count++;
        return ERR_TIMEOUT;
    }
    
    if (verify_response(response, FUNC_BATCH_RW) != 0) {
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    /* 解析响应 */
    uint16_t current = ((uint16_t)response[2] << 8) | response[3];
    uint16_t voltage = ((uint16_t)response[4] << 8) | response[5];
    
    pthread_mutex_lock(&power->mutex);
    power->current_setpoint = target_current_ma;
    power->actual_current = current;
    power->actual_voltage = voltage;
    power->read_count++;
    pthread_mutex_unlock(&power->mutex);
    
    if (actual_current_ma) *actual_current_ma = current;
    if (voltage_mv) *voltage_mv = voltage;
    
    return ERR_OK;
}

/******************************************************************************
 * 获取状态（V2优化版）
 ******************************************************************************/
ErrorCode_t power_get_status(PowerDriver_t *power, uint16_t *current_ma, uint16_t *voltage_mv) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }

    /* 使用批量命令同时获取电流和电压 */
    return power_batch_control(power, power->current_setpoint, current_ma, voltage_mv);
}

ErrorCode_t power_get_status_cached(PowerDriver_t *power, uint16_t *current_ma, uint16_t *voltage_mv) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }

    pthread_mutex_lock(&power->mutex);
    if (current_ma) *current_ma = power->actual_current;
    if (voltage_mv) *voltage_mv = power->actual_voltage;
    pthread_mutex_unlock(&power->mutex);

    return ERR_OK;
}

ErrorCode_t power_on(PowerDriver_t *power) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    ErrorCode_t ret = power_set_current(power, power->current_setpoint);
    
    if (ret == ERR_OK) {
        pthread_mutex_lock(&power->mutex);
        power->state = POWER_STATE_ON;
        pthread_mutex_unlock(&power->mutex);
        
        LOG_INFO(LOG_MODULE_POWER, "Power output enabled");
    }
    
    return ret;
}

ErrorCode_t power_off(PowerDriver_t *power) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    ErrorCode_t ret = power_set_current(power, 50);
    
    if (ret == ERR_OK) {
        pthread_mutex_lock(&power->mutex);
        power->state = POWER_STATE_OFF;
        pthread_mutex_unlock(&power->mutex);
        
        LOG_INFO(LOG_MODULE_POWER, "Power output disabled (set to minimum)");
    }
    
    return ret;
}
