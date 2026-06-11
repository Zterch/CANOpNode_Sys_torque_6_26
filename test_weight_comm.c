/******************************************************************************
 * @file    test_weight_comm.c
 * @brief   重量采集模块通信测试工具
 * @author  System Architect
 * @date    2026-06-11
 * @version 1.0.0
 * 
 * 用法: ./test_weight_comm [device] [baudrate]
 * 示例: ./test_weight_comm /dev/ttyUSB2 115200
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <time.h>

#define WEIGHT_REG_WEIGHT_FILTERED  0x0102  /* 读取滤波后重量 (0.01kg单位) */

/* CRC16计算 */
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
    
    return (crc << 8) | (crc >> 8);
}

/* 打开串口 */
int serial_open(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf("Failed to open %s: %s\n", device, strerror(errno));
        return -1;
    }
    
    /* 清除非阻塞标志 */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0) {
        printf("tcgetattr failed: %s\n", strerror(errno));
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
    
    /* 设置超时 */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;  /* 500ms超时 */
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    
    printf("Serial port %s opened at %d baud\n", device, baudrate);
    return fd;
}

/* 发送数据 */
int serial_send(int fd, const uint8_t *data, uint8_t len) {
    int ret = write(fd, data, len);
    printf("TX (%d bytes): ", ret);
    for (int i = 0; i < ret; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    return ret;
}

/* 接收数据 */
int serial_receive(int fd, uint8_t *data, uint8_t max_len, int timeout_ms) {
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
    
    int rx_len = read(fd, data, max_len);
    if (rx_len > 0) {
        printf("RX (%d bytes): ", rx_len);
        for (int i = 0; i < rx_len && i < 16; i++) {
            printf("%02X ", data[i]);
        }
        if (rx_len > 16) printf("...");
        printf("\n");
    }
    return rx_len;
}

/* 测试通信 */
int test_communication(int fd, uint16_t reg_addr) {
    uint8_t tx_buf[8];
    
    tx_buf[0] = 0xAA;  /* 设备地址 */
    tx_buf[1] = 0x01;  /* 功能码：读取 */
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = 0x00;
    tx_buf[5] = 0x02;
    
    uint16_t crc = weight_crc16(tx_buf, 6);
    tx_buf[6] = (crc >> 8) & 0xFF;
    tx_buf[7] = crc & 0xFF;
    
    printf("\n--- Test Register 0x%04X ---\n", reg_addr);
    
    /* 清空接收缓冲区 */
    uint8_t temp_buf[256];
    while (read(fd, temp_buf, sizeof(temp_buf)) > 0);
    
    /* 发送命令 */
    if (serial_send(fd, tx_buf, 8) != 8) {
        printf("Send failed\n");
        return -1;
    }
    
    /* 等待响应 */
    usleep(10000);  /* 10ms */
    
    /* 接收响应 */
    uint8_t rx_buf[16];
    int rx_len = serial_receive(fd, rx_buf, sizeof(rx_buf), 500);
    
    if (rx_len < 8) {
        printf("Response too short or timeout\n");
        return -1;
    }
    
    /* 验证帧头 */
    if (rx_buf[0] != 0xAA || rx_buf[1] != 0x01) {
        printf("Header mismatch: expected 0xAA 0x01, got 0x%02X 0x%02X\n", 
               rx_buf[0], rx_buf[1]);
        return -1;
    }
    
    /* 提取数据 */
    uint16_t data = (rx_buf[4] << 8) | rx_buf[5];
    float weight_kg = (float)data / 100.0f;
    
    printf("Data: 0x%04X (%u) -> %.2f kg\n", data, data, weight_kg);
    
    /* 验证CRC */
    uint16_t rx_crc = (rx_buf[6] << 8) | rx_buf[7];
    uint16_t calc_crc = weight_crc16(rx_buf, 6);
    printf("CRC: RX=0x%04X, CALC=0x%04X, %s\n", 
           rx_crc, calc_crc, (rx_crc == calc_crc) ? "OK" : "MISMATCH");
    
    return 0;
}

int main(int argc, char *argv[]) {
    const char *device = (argc > 1) ? argv[1] : "/dev/ttyUSB2";
    int baudrate = (argc > 2) ? atoi(argv[2]) : 115200;
    
    printf("=======================================\n");
    printf("Weight Sensor Communication Test Tool\n");
    printf("=======================================\n");
    printf("Device: %s\n", device);
    printf("Baudrate: %d\n", baudrate);
    printf("=======================================\n\n");
    
    int fd = serial_open(device, baudrate);
    if (fd < 0) {
        return 1;
    }
    
    /* 测试多次 */
    for (int i = 0; i < 3; i++) {
        printf("\n========== Test %d/3 ==========\n", i + 1);
        test_communication(fd, WEIGHT_REG_WEIGHT_FILTERED);
        sleep(1);
    }
    
    close(fd);
    printf("\nTest completed.\n");
    
    return 0;
}
