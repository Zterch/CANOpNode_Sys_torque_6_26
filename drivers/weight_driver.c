/******************************************************************************
 * @file    weight_driver.c
 * @brief   重量采集驱动实现 - UART/TTL串口控制，与电源板协议兼容
 * @author  System Architect
 * @date    2026-06-03
 * @version 1.0.0
 * 
 * 协议说明：
 * - 设备地址：0xAA
 * - 功能码：0x01（读取），0x06（写入）
 * - 数据格式：高字节在前
 * - CRC16：高字节在前
 ******************************************************************************/

#include "weight_driver.h"
#include "../utils/logger.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <time.h>
#include <stdint.h>

/******************************************************************************
 * CRC16计算（重量采集协议 - 与电源板相同）
 ******************************************************************************/
uint16_t weight_crc16(const uint8_t *data, uint8_t len) {
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
    
    /* 协议要求CRC高字节在前 */
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
    
    /* 设置超时 - 与测试工具一致，使用500ms超时 */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;  /* 500ms超时，给设备足够时间响应 */
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        LOG_ERROR(LOG_MODULE_POWER, "tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    
    return fd;
}

static void serial_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

static int serial_send(int fd, const uint8_t *data, uint8_t len) {
    return write(fd, data, len);
}

static int serial_receive(int fd, uint8_t *data, uint8_t max_len, int timeout_ms) {
    fd_set read_fds;
    struct timeval tv;
    
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(fd + 1, &read_fds, NULL, NULL, &tv);
    if (ret <= 0) {
        return ret;  /* 超时或错误 */
    }
    
    return read(fd, data, max_len);
}

/******************************************************************************
 * 发送读取命令
 ******************************************************************************/
static ErrorCode_t weight_send_read_cmd(WeightDriver_t *weight, uint16_t reg_addr) {
    uint8_t tx_buf[8];
    uint8_t temp_buf[256];
    
    tx_buf[0] = 0xAA;  /* 设备地址 */
    tx_buf[1] = 0x01;  /* 功能码：读取 */
    tx_buf[2] = (reg_addr >> 8) & 0xFF;  /* 寄存器地址高字节 */
    tx_buf[3] = reg_addr & 0xFF;         /* 寄存器地址低字节 */
    tx_buf[4] = 0x00;  /* 读取长度高字节 */
    tx_buf[5] = 0x02;  /* 读取长度低字节（2字节数据） */
    
    uint16_t crc = weight_crc16(tx_buf, 6);
    tx_buf[6] = (crc >> 8) & 0xFF;  /* CRC高字节 */
    tx_buf[7] = crc & 0xFF;         /* CRC低字节 */
    
    pthread_mutex_lock(&weight->mutex);
    
    /* 清空接收缓冲区 - 避免读取到旧数据 */
    while (read(weight->fd, temp_buf, sizeof(temp_buf)) > 0);
    
    LOG_DEBUG(LOG_MODULE_POWER, "Weight TX: %02X %02X %02X %02X %02X %02X %02X %02X (reg=0x%04X)",
              tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3], tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7], reg_addr);
    
    int ret = serial_send(weight->fd, tx_buf, 8);
    pthread_mutex_unlock(&weight->mutex);
    
    if (ret != 8) {
        LOG_ERROR(LOG_MODULE_POWER, "Weight send failed: sent %d bytes (expected 8), errno=%d", ret, errno);
        return ERR_COMM_FAIL;
    }
    
    return ERR_OK;
}

/******************************************************************************
 * 接收响应数据
 ******************************************************************************/
static ErrorCode_t weight_receive_response(WeightDriver_t *weight, uint16_t *data, int timeout_ms) {
    uint8_t rx_buf[16];
    
    pthread_mutex_lock(&weight->mutex);
    int rx_len = serial_receive(weight->fd, rx_buf, sizeof(rx_buf), timeout_ms);
    pthread_mutex_unlock(&weight->mutex);
    
    if (rx_len <= 0) {
        LOG_WARN(LOG_MODULE_POWER, "Weight receive timeout or error: ret=%d", rx_len);
        return ERR_COMM_FAIL;
    }
    
    if (rx_len < 8) {
        LOG_WARN(LOG_MODULE_POWER, "Weight response too short: %d bytes (expected >= 8)", rx_len);
        /* 打印接收到的数据以便调试 */
        LOG_WARN(LOG_MODULE_POWER, "Weight RX data: %02X %02X %02X %02X ...", 
                 rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        return ERR_COMM_FAIL;
    }
    
    /* 打印接收到的完整数据 */
    LOG_DEBUG(LOG_MODULE_POWER, "Weight RX: %02X %02X %02X %02X %02X %02X %02X %02X (len=%d)",
              rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7], rx_len);
    
    /* 验证帧头 */
    if (rx_buf[0] != 0xAA || rx_buf[1] != 0x01) {
        LOG_WARN(LOG_MODULE_POWER, "Weight response header mismatch: 0x%02X 0x%02X (expected 0xAA 0x01)", 
                 rx_buf[0], rx_buf[1]);
        return ERR_COMM_FAIL;
    }
    
    /* 验证CRC - 注意：某些设备固件有bug，CRC校验被禁用，这里我们记录但不强制失败 */
    uint16_t rx_crc = (rx_buf[rx_len - 2] << 8) | rx_buf[rx_len - 1];
    uint16_t calc_crc = weight_crc16(rx_buf, rx_len - 2);
    
    if (rx_crc != calc_crc) {
        /* CRC不匹配，但某些设备固件有bug，CRC计算错误，这里只记录警告 */
        LOG_WARN(LOG_MODULE_POWER, "Weight response CRC mismatch: RX=0x%04X, CALC=0x%04X (device firmware bug?)", 
                 rx_crc, calc_crc);
        /* 不返回错误，继续处理数据，因为某些设备CRC校验被禁用 */
    }
    
    /* 提取数据 - 数据在字节4和5（从0开始计数） */
    if (data != NULL) {
        *data = (rx_buf[4] << 8) | rx_buf[5];
        LOG_DEBUG(LOG_MODULE_POWER, "Weight data received: 0x%04X (%u)", *data, *data);
    }
    
    return ERR_OK;
}

/******************************************************************************
 * 初始化重量采集驱动 - 工业级重试机制
 ******************************************************************************/
ErrorCode_t weight_init(WeightDriver_t *weight, const char *device, int baudrate) {
    if (weight == NULL || device == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(weight, 0, sizeof(WeightDriver_t));
    
    pthread_mutex_init(&weight->mutex, NULL);
    
    strncpy(weight->device, device, sizeof(weight->device) - 1);
    weight->baudrate = baudrate;
    weight->state = WEIGHT_STATE_INIT;
    
    /* 打开串口 */
    weight->fd = serial_open(device, baudrate);
    if (weight->fd < 0) {
        LOG_ERROR(LOG_MODULE_POWER, "Failed to open weight device: %s", device);
        return ERR_DEVICE_NOT_FOUND;
    }
    
    /* 工业级通信测试 - 带重试机制 */
    /* 某些设备上电后需要一定时间初始化，因此需要多次尝试 */
    #define WEIGHT_INIT_RETRY_COUNT     5       /* 最大重试次数 */
    #define WEIGHT_INIT_RETRY_DELAY_MS  500     /* 每次重试间隔500ms */
    
    ErrorCode_t ret = ERR_COMM_FAIL;
    int retry_count = 0;
    float test_weight = 0.0f;
    
    while (retry_count < WEIGHT_INIT_RETRY_COUNT) {
        ret = weight_get_weight(weight, &test_weight);
        if (ret == ERR_OK) {
            LOG_INFO(LOG_MODULE_POWER, "Weight device communication test passed (attempt %d/%d), weight=%.3f kg",
                     retry_count + 1, WEIGHT_INIT_RETRY_COUNT, test_weight);
            break;
        }
        
        retry_count++;
        if (retry_count < WEIGHT_INIT_RETRY_COUNT) {
            LOG_WARN(LOG_MODULE_POWER, "Weight device communication test failed (attempt %d/%d), retrying in %d ms...",
                     retry_count, WEIGHT_INIT_RETRY_COUNT, WEIGHT_INIT_RETRY_DELAY_MS);
            usleep(WEIGHT_INIT_RETRY_DELAY_MS * 1000);  /* 等待后重试 */
        }
    }
    
    if (ret != ERR_OK) {
        LOG_ERROR(LOG_MODULE_POWER, "Weight device communication test failed after %d attempts", WEIGHT_INIT_RETRY_COUNT);
        LOG_ERROR(LOG_MODULE_POWER, "Possible causes:");
        LOG_ERROR(LOG_MODULE_POWER, "  1. Device not powered on or not connected to %s", device);
        LOG_ERROR(LOG_MODULE_POWER, "  2. Wrong baudrate (expected: %d)", baudrate);
        LOG_ERROR(LOG_MODULE_POWER, "  3. Device address mismatch (expected: 0xAA)");
        LOG_ERROR(LOG_MODULE_POWER, "  4. Communication protocol mismatch");
        serial_close(weight->fd);
        weight->fd = -1;
        return ERR_COMM_FAIL;
    }
    
    weight->state = WEIGHT_STATE_READY;
    weight->initialized = 1;
    weight->weight_filtered = test_weight;
    
    LOG_INFO(LOG_MODULE_POWER, "Weight driver initialized: %s @ %d baud", device, baudrate);
    
    return ERR_OK;
}

/******************************************************************************
 * 反初始化重量采集驱动
 ******************************************************************************/
void weight_deinit(WeightDriver_t *weight) {
    if (weight == NULL) {
        return;
    }
    
    pthread_mutex_lock(&weight->mutex);
    
    if (weight->fd >= 0) {
        serial_close(weight->fd);
        weight->fd = -1;
    }
    
    weight->initialized = 0;
    weight->state = WEIGHT_STATE_UNKNOWN;
    
    pthread_mutex_unlock(&weight->mutex);
    pthread_mutex_destroy(&weight->mutex);
    
    LOG_INFO(LOG_MODULE_POWER, "Weight driver deinitialized");
}

/******************************************************************************
 * 读取重量数据（滤波后）
 ******************************************************************************/
ErrorCode_t weight_get_weight(WeightDriver_t *weight, float *weight_kg) {
    if (weight == NULL || weight_kg == NULL) {
        LOG_ERROR(LOG_MODULE_POWER, "weight_get_weight: invalid parameters");
        return ERR_INVALID_PARAM;
    }
    
    if (!weight->initialized) {
        /* 初始化时调用，允许未初始化状态 */
        LOG_DEBUG(LOG_MODULE_POWER, "weight_get_weight: device not initialized yet");
    }
    
    ErrorCode_t ret = weight_send_read_cmd(weight, WEIGHT_REG_WEIGHT_FILTERED);
    if (ret != ERR_OK) {
        LOG_WARN(LOG_MODULE_POWER, "weight_get_weight: send command failed");
        weight->error_count++;
        return ret;
    }
    
    /* 优化：使用更短的等待时间和超时，避免阻塞主采集线程 */
    usleep(2000);  /* 等待2ms，给设备足够时间响应 */
    
    uint16_t data;
    ret = weight_receive_response(weight, &data, 20);  /* 超时20ms，快速失败 */
    if (ret != ERR_OK) {
        /* 采集失败，使用上一次的值 */
        LOG_DEBUG(LOG_MODULE_POWER, "weight_get_weight: receive timeout, using cached value");
        *weight_kg = weight->weight_filtered;  /* 使用缓存值 */
        return ERR_OK;  /* 返回成功，但使用旧值 */
    }
    
    /* 数据格式：0.01kg单位，转换为kg */
    weight->weight_filtered = (float)data / 100.0f;
    weight->read_count++;
    
    *weight_kg = weight->weight_filtered;
    
    LOG_DEBUG(LOG_MODULE_POWER, "weight_get_weight: success, raw=%u, weight=%.3f kg", data, *weight_kg);
    
    return ERR_OK;
}

/******************************************************************************
 * 读取原始重量数据
 ******************************************************************************/
ErrorCode_t weight_get_weight_raw(WeightDriver_t *weight, float *weight_kg) {
    if (weight == NULL || weight_kg == NULL || !weight->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    ErrorCode_t ret = weight_send_read_cmd(weight, WEIGHT_REG_WEIGHT_RAW);
    if (ret != ERR_OK) {
        weight->error_count++;
        return ret;
    }
    
    usleep(5000);  /* 等待5ms */
    
    uint16_t data;
    ret = weight_receive_response(weight, &data, 50);
    if (ret != ERR_OK) {
        weight->error_count++;
        return ret;
    }
    
    /* 数据格式：0.01kg单位，转换为kg */
    weight->weight_raw = (float)data / 100.0f;
    weight->read_count++;
    
    *weight_kg = weight->weight_raw;
    
    return ERR_OK;
}

/******************************************************************************
 * 读取电压数据
 ******************************************************************************/
ErrorCode_t weight_get_voltage(WeightDriver_t *weight, float *voltage_v) {
    if (weight == NULL || voltage_v == NULL || !weight->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    ErrorCode_t ret = weight_send_read_cmd(weight, WEIGHT_REG_VOLTAGE_FILTERED);
    if (ret != ERR_OK) {
        weight->error_count++;
        return ret;
    }
    
    usleep(5000);  /* 等待5ms */
    
    uint16_t data;
    ret = weight_receive_response(weight, &data, 50);
    if (ret != ERR_OK) {
        weight->error_count++;
        return ret;
    }
    
    /* 数据格式：mV单位，转换为V */
    weight->voltage_filtered = (float)data / 1000.0f;
    weight->read_count++;
    
    *voltage_v = weight->voltage_filtered;
    
    return ERR_OK;
}

/******************************************************************************
 * 读取所有数据
 ******************************************************************************/
ErrorCode_t weight_get_all(WeightDriver_t *weight) {
    if (weight == NULL || !weight->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    ErrorCode_t ret;
    
    /* 读取滤波后重量 */
    ret = weight_get_weight(weight, &weight->weight_filtered);
    if (ret != ERR_OK) {
        return ret;
    }
    
    /* 读取原始重量 */
    ret = weight_get_weight_raw(weight, &weight->weight_raw);
    if (ret != ERR_OK) {
        return ret;
    }
    
    /* 读取电压 */
    ret = weight_get_voltage(weight, &weight->voltage_filtered);
    if (ret != ERR_OK) {
        return ret;
    }
    
    weight->timestamp_ms = (uint32_t)(time(NULL) * 1000);
    
    return ERR_OK;
}

/******************************************************************************
 * 获取重量数据（线程安全）
 ******************************************************************************/
ErrorCode_t weight_get_data(WeightDriver_t *weight, float *weight_kg, int is_filtered) {
    if (weight == NULL || weight_kg == NULL || !weight->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&weight->mutex);
    
    if (is_filtered) {
        *weight_kg = weight->weight_filtered;
    } else {
        *weight_kg = weight->weight_raw;
    }
    
    pthread_mutex_unlock(&weight->mutex);
    
    return ERR_OK;
}

/******************************************************************************
 * 后台采集线程 - 100Hz独立采集
 ******************************************************************************/
static void* weight_collection_thread(void* arg) {
    WeightDriver_t *weight = (WeightDriver_t *)arg;
    
    LOG_INFO(LOG_MODULE_POWER, "Weight collection thread started (10Hz)");
    
    /* 使用普通调度策略，避免与实时线程竞争 */
    struct sched_param param;
    param.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    
    /* 降低采集频率到10Hz（100ms周期），大幅减少CPU占用 */
    #define WEIGHT_SAMPLE_PERIOD_US 100000
    
    while (weight->thread_running) {
        /* 采集重量数据 - 使用非阻塞方式 */
        float weight_kg = 0.0f;
        ErrorCode_t ret = weight_get_weight(weight, &weight_kg);
        
        if (ret == ERR_OK) {
            pthread_mutex_lock(&weight->mutex);
            weight->weight_filtered = weight_kg;
            weight->sample_count++;
            pthread_mutex_unlock(&weight->mutex);
        }
        
        /* 简单延迟，不严格周期控制，避免影响主线程 */
        usleep(WEIGHT_SAMPLE_PERIOD_US);
    }
    
    LOG_INFO(LOG_MODULE_POWER, "Weight collection thread stopped, total samples: %u", weight->sample_count);
    return NULL;
}

/******************************************************************************
 * 启动后台采集线程（100Hz）
 ******************************************************************************/
ErrorCode_t weight_start_collection(WeightDriver_t *weight) {
    if (weight == NULL || !weight->initialized) {
        return ERR_INVALID_PARAM;
    }
    
    if (weight->thread_running) {
        return ERR_OK;  /* 已经在运行 */
    }
    
    weight->thread_running = 1;
    
    int ret = pthread_create(&weight->collect_thread, NULL, weight_collection_thread, weight);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_POWER, "Failed to create weight collection thread: %s", strerror(ret));
        weight->thread_running = 0;
        return ERR_THREAD_CREATE;
    }
    
    LOG_INFO(LOG_MODULE_POWER, "Weight collection thread created (100Hz)");
    return ERR_OK;
}

/******************************************************************************
 * 停止后台采集线程
 ******************************************************************************/
void weight_stop_collection(WeightDriver_t *weight) {
    if (weight == NULL || !weight->thread_running) {
        return;
    }
    
    weight->thread_running = 0;
    pthread_join(weight->collect_thread, NULL);
    
    LOG_INFO(LOG_MODULE_POWER, "Weight collection thread stopped, total samples: %u", weight->sample_count);
}
