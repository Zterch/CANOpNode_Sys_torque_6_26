/******************************************************************************
 * @file    power_driver.c
 * @brief   电源板驱动实现 - UART/TTL串口控制
 * @author  System Architect
 * @date    2026-04-20
 * @version 1.0.0
 * 
 * 协议说明：
 * - 设备地址：0xAA
 * - 功能码：0x01（读取），0x06（写入）
 * - 数据格式：高字节在前
 * - CRC16：高字节在前
 * 
 * 示例帧：
 * 读取电压：AA 01 09 A0 00 02 CRC_H CRC_L
 * 设置电流：AA 06 03 04 02 58 CRC_H CRC_L (设置600mA)
 ******************************************************************************/

#include "power_driver.h"
#include "../utils/logger.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>

/******************************************************************************
 * CRC16计算（电源板协议 - 与RS485协议相同）
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
    
    /* 最小读取字节数和超时 */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        LOG_ERROR(LOG_MODULE_POWER, "tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    LOG_INFO(LOG_MODULE_POWER, "Serial port %s opened at %d baud", device, baudrate);
    return fd;
}

static void serial_close(int fd) {
    if (fd >= 0) {
        tcflush(fd, TCIOFLUSH);
        close(fd);
    }
}

static int serial_send(int fd, const uint8_t *data, int len, int timeout_ms) {
    if (fd < 0 || data == NULL || len <= 0) {
        return -1;
    }
    
    fd_set writefds;
    struct timeval tv;
    
    FD_ZERO(&writefds);
    FD_SET(fd, &writefds);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(fd + 1, NULL, &writefds, NULL, &tv);
    if (ret <= 0) {
        return -1;
    }
    
    ret = write(fd, data, len);
    if (ret != len) {
        return -1;
    }
    
    return ret;
}

static int serial_receive(int fd, uint8_t *data, int max_len, int timeout_ms) {
    if (fd < 0 || data == NULL || max_len <= 0) {
        return -1;
    }
    
    fd_set readfds;
    struct timeval tv;
    
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    if (ret <= 0) {
        return -1;
    }
    
    ret = read(fd, data, max_len);
    return ret;
}

static void serial_flush(int fd) {
    if (fd >= 0) {
        tcflush(fd, TCIOFLUSH);
    }
}

/******************************************************************************
 * 内部函数：构建并发送命令帧
 ******************************************************************************/
static ErrorCode_t power_send_command(PowerDriver_t *power, uint8_t func_code, 
                                       uint16_t reg_addr, uint16_t data,
                                       uint8_t *response, int resp_len) {
    uint8_t frame[8];
    uint16_t crc;
    
    /* 构建帧 */
    frame[0] = 0xAA;                    /* 设备地址 */
    frame[1] = func_code;               /* 功能码 */
    frame[2] = (reg_addr >> 8) & 0xFF;  /* 寄存器地址高字节 */
    frame[3] = reg_addr & 0xFF;         /* 寄存器地址低字节 */
    
    if (func_code == 0x06) {
        /* 写入命令 */
        frame[4] = (data >> 8) & 0xFF;  /* 数据高字节 */
        frame[5] = data & 0xFF;         /* 数据低字节 */
    } else {
        /* 读取命令 */
        frame[4] = 0x00;
        frame[5] = 0x02;                /* 读取2个字节 */
    }
    
    /* 计算CRC */
    crc = power_crc16(frame, 6);
    frame[6] = (crc >> 8) & 0xFF;       /* CRC高字节 */
    frame[7] = crc & 0xFF;              /* CRC低字节 */
    
    pthread_mutex_lock(&power->mutex);
    
    /* 清空缓冲区 */
    serial_flush(power->fd);
    
    /* 发送 */
    if (serial_send(power->fd, frame, 8, 20) != 8) {
        pthread_mutex_unlock(&power->mutex);
        return ERR_COMM_FAIL;
    }

    /* 接收响应 - 使用更短的超时，避免阻塞控制线程 */
    int rx_len = serial_receive(power->fd, response, resp_len, 20);
    
    pthread_mutex_unlock(&power->mutex);
    
    if (rx_len < 5) {
        return ERR_TIMEOUT;
    }
    
    /* 验证响应 */
    if (response[0] != 0xAA) {
        return ERR_COMM_FAIL;
    }
    
    if (response[1] != func_code) {
        return ERR_COMM_FAIL;
    }
    
    return ERR_OK;
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
    power->current_setpoint = 440;  /* 默认0.44A */
    
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
    
    LOG_INFO(LOG_MODULE_POWER, "Power driver initialized (device=%s, baud=%d)", 
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
    
    LOG_INFO(LOG_MODULE_POWER, "Power driver deinitialized");
}

ErrorCode_t power_set_current(PowerDriver_t *power, uint16_t current_ma) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 限制范围 50-4000mA */
    if (current_ma < 50) current_ma = 50;
    if (current_ma > 4000) current_ma = 4000;
    
    uint8_t response[16];
    
    ErrorCode_t ret = power_send_command(power, 0x06, POWER_REG_ISET, current_ma, response, 16);
    
    if (ret == ERR_OK) {
        pthread_mutex_lock(&power->mutex);
        uint16_t old_setpoint = power->current_setpoint;
        power->current_setpoint = current_ma;
        pthread_mutex_unlock(&power->mutex);
        
        /* 只有当电流设置值发生明显变化时才打印日志（避免频繁输出） */
        static uint32_t last_log_time = 0;
        uint32_t now = get_timestamp_ms();
        if (abs((int)current_ma - (int)old_setpoint) > 5 || (now - last_log_time) > 1000) {
            LOG_INFO(LOG_MODULE_POWER, "Current set to %d mA (%.2f A)", current_ma, current_ma / 1000.0f);
            last_log_time = now;
        }
    } else {
        LOG_WARN(LOG_MODULE_POWER, "Failed to set current to %d mA", current_ma);
    }
    
    return ret;
}

ErrorCode_t power_get_current(PowerDriver_t *power, uint16_t *current_ma) {
    if (power == NULL || !power->initialized || current_ma == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t response[16];
    
    ErrorCode_t ret = power_send_command(power, 0x01, POWER_REG_IOUT, 0, response, 16);
    
    if (ret != ERR_OK) {
        power->error_count++;
        return ret;
    }
    
    /* 解析响应：AA 01 02 XX XX CRC_H CRC_L */
    if (response[2] != 0x02) {
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    uint16_t current = ((uint16_t)response[3] << 8) | response[4];
    
    pthread_mutex_lock(&power->mutex);
    power->actual_current = current;
    power->read_count++;
    pthread_mutex_unlock(&power->mutex);
    
    *current_ma = current;
    
    return ERR_OK;
}

ErrorCode_t power_get_voltage(PowerDriver_t *power, uint16_t *voltage_mv) {
    if (power == NULL || !power->initialized || voltage_mv == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    uint8_t response[16];
    
    ErrorCode_t ret = power_send_command(power, 0x01, POWER_REG_VOUT, 0, response, 16);
    
    if (ret != ERR_OK) {
        power->error_count++;
        return ret;
    }
    
    /* 解析响应：AA 01 02 XX XX CRC_H CRC_L */
    if (response[2] != 0x02) {
        power->error_count++;
        return ERR_COMM_FAIL;
    }
    
    uint16_t voltage = ((uint16_t)response[3] << 8) | response[4];
    
    pthread_mutex_lock(&power->mutex);
    power->actual_voltage = voltage;
    pthread_mutex_unlock(&power->mutex);
    
    *voltage_mv = voltage;
    
    return ERR_OK;
}

ErrorCode_t power_get_status(PowerDriver_t *power, uint16_t *current_ma, uint16_t *voltage_mv) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    ErrorCode_t ret;
    
    /* 读取电流 */
    if (current_ma != NULL) {
        ret = power_get_current(power, current_ma);
        if (ret != ERR_OK) {
            return ret;
        }
    }
    
    /* 读取电压 */
    if (voltage_mv != NULL) {
        ret = power_get_voltage(power, voltage_mv);
        if (ret != ERR_OK) {
            return ret;
        }
    }
    
    return ERR_OK;
}

ErrorCode_t power_on(PowerDriver_t *power) {
    if (power == NULL || !power->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 电源板上电即工作，设置默认电流 */
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
    
    /* 设置电流为最小值 */
    ErrorCode_t ret = power_set_current(power, 50);  /* 最小50mA */
    
    if (ret == ERR_OK) {
        pthread_mutex_lock(&power->mutex);
        power->state = POWER_STATE_OFF;
        pthread_mutex_unlock(&power->mutex);
        
        LOG_INFO(LOG_MODULE_POWER, "Power output disabled (set to minimum)");
    }
    
    return ret;
}
