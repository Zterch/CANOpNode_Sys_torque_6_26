#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>

unsigned short modbus_crc16(const unsigned char *buffer, unsigned int length) {
    unsigned short crc = 0xFFFF;
    for (unsigned int i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

int configure_serial(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;   // 1秒超时（单位0.1秒）

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);  // 无校验
    tty.c_cflag &= ~CSTOPB;             // 1停止位
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

void print_hex(const unsigned char *data, int len, const char *prefix) {
    printf("%s", prefix);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

int send_modbus_request(int fd, unsigned char addr, unsigned char func,
                        unsigned short reg, unsigned short value,
                        unsigned char *response, int timeout_sec, int debug) {
    unsigned char request[8];
    request[0] = addr;
    request[1] = func;
    request[2] = (reg >> 8) & 0xFF;
    request[3] = reg & 0xFF;
    request[4] = (value >> 8) & 0xFF;
    request[5] = value & 0xFF;

    unsigned short crc = modbus_crc16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    if (debug) {
        printf("Sending %d bytes: ", 8);
        print_hex(request, 8, "");
    }

    ssize_t written = write(fd, request, 8);
    if (written != 8) {
        fprintf(stderr, "Write error: wrote %ld bytes\n", written);
        return -1;
    }
    tcdrain(fd);

    fd_set read_fds;
    struct timeval tv;
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;

    int ret = select(fd + 1, &read_fds, NULL, NULL, &tv);
    if (ret < 0) {
        perror("select");
        return -1;
    } else if (ret == 0) {
        fprintf(stderr, "Response timeout (%d seconds)\n", timeout_sec);
        return -1;
    }

    int total = 0;
    while (total < 8) {
        ssize_t len = read(fd, response + total, 8 - total);
        if (len <= 0) {
            perror("read");
            return -1;
        }
        total += len;
    }

    if (debug) {
        printf("Received %d bytes: ", total);
        print_hex(response, total, "");
    }

    return total;
}

int main(int argc, char *argv[]) {
    const char *device = "/dev/ttyUSB1";
    int baudrate = B1152000;          // 默认 9600
    int slave_addr = 0x01;         // 从机地址
    int reg = 0x0004;              // 采样速率寄存器
    int value = 0x0003;            // 3:150pcs
    int timeout = 2;
    int debug = 1;                 // 默认输出调试信息

    // 简单命令行解析
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i+1 < argc) {
            device = argv[++i];
        } else if (strcmp(argv[i], "-b") == 0 && i+1 < argc) {
            int b = atoi(argv[++i]);
            switch (b) {
                case 4800:  baudrate = B4800;  break;
                case 9600:  baudrate = B9600;  break;
                case 19200: baudrate = B19200; break;
                case 38400: baudrate = B38400; break;
                case 115200: baudrate = B115200; break;
                default:
                    fprintf(stderr, "Unsupported baudrate %d. Use 4800/9600/19200/38400/1152000\n", b);
                    return EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "-a") == 0 && i+1 < argc) {
            slave_addr = strtol(argv[++i], NULL, 16);
        } else if (strcmp(argv[i], "-v") == 0 && i+1 < argc) {
            value = strtol(argv[++i], NULL, 16);
        } else if (strcmp(argv[i], "-t") == 0 && i+1 < argc) {
            timeout = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-q") == 0) {
            debug = 0;
        } else {
            printf("Usage: %s [-d device] [-b baud] [-a addr] [-v value] [-t timeout] [-q]\n", argv[0]);
            printf("  -d device   : serial device, default /dev/ttyUSB1\n");
            printf("  -b baud     : 4800/9600/19200/38400, default 9600\n");
            printf("  -a addr     : slave address in hex, default 0x01\n");
            printf("  -v value    : register value in hex, default 0x03\n");
            printf("  -t timeout  : response timeout seconds, default 2\n");
            printf("  -q          : quiet mode (no debug output)\n");
            return EXIT_FAILURE;
        }
    }

    if (debug) {
        printf("Device: %s\n", device);
        printf("Baudrate: %d\n", baudrate == B4800 ? 4800 : baudrate == B9600 ? 9600 : baudrate == B19200 ? 19200 : 38400);
        printf("Slave Address: 0x%02X\n", slave_addr);
        printf("Register: 0x%04X\n", reg);
        printf("Value: 0x%04X\n", value);
        printf("Timeout: %d seconds\n", timeout);
    }

    int fd = configure_serial(device, baudrate);
    if (fd < 0) {
        return EXIT_FAILURE;
    }

    unsigned char response[8];
    int len = send_modbus_request(fd, slave_addr, 0x06, reg, value, response, timeout, debug);

    if (len == 8) {
        unsigned short recv_crc = response[6] | (response[7] << 8);
        unsigned short calc_crc = modbus_crc16(response, 6);
        if (recv_crc == calc_crc &&
            response[0] == slave_addr &&
            response[1] == 0x06 &&
            response[2] == ((reg >> 8) & 0xFF) &&
            response[3] == (reg & 0xFF) &&
            response[4] == ((value >> 8) & 0xFF) &&
            response[5] == (value & 0xFF)) {
            printf("SUCCESS: Sampling rate set to %d\n", value);
            close(fd);
            return EXIT_SUCCESS;
        } else {
            fprintf(stderr, "ERROR: Invalid response CRC or content.\n");
            if (debug) {
                printf("Expected response: ");
                unsigned char expected[8] = {slave_addr, 0x06, (reg>>8)&0xFF, reg&0xFF, (value>>8)&0xFF, value&0xFF, 0,0};
                unsigned short ecrc = modbus_crc16(expected, 6);
                expected[6] = ecrc & 0xFF;
                expected[7] = (ecrc >> 8) & 0xFF;
                print_hex(expected, 8, "");
            }
            close(fd);
            return EXIT_FAILURE;
        }
    } else {
        fprintf(stderr, "ERROR: Failed to read valid response (got %d bytes)\n", len);
        close(fd);
        return EXIT_FAILURE;
    }
}