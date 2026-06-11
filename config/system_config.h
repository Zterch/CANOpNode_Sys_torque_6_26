
/******************************************************************************
 * @file    system_config.h
 * @brief   系统配置文件 - 包含所有设备的配置参数
 * @author  System Architect
 * @date    2026-04-16
 * @version 1.0.0
 ******************************************************************************/

#ifndef __SYSTEM_CONFIG_H__
#define __SYSTEM_CONFIG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 系统通用配置
 ******************************************************************************/
#define SYSTEM_NAME             "CANOpNode_Sys"
#define SYSTEM_VERSION          "1.0.0"
#define SYSTEM_LOG_LEVEL        3           /* 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG */

/******************************************************************************
 * CANopen电机配置 (Nimotion Servo)
 ******************************************************************************/
#define MOTOR_CAN_INTERFACE     "can0"
#define MOTOR_NODE_ID           1
#define MOTOR_CAN_BITRATE       1000000     /* 1 Mbps */

/* 电机控制参数 */
#define MOTOR_MAX_VELOCITY      10000       /* 最大速度 (编码器单位/秒) */
#define MOTOR_MAX_ACCELERATION  5000        /* 最大加速度 */
#define MOTOR_MAX_TORQUE        1000        /* 最大扭矩 (0.1%额定) */

/* CSV模式默认参数 */
#define MOTOR_CSV_AMPLITUDE     5000        /* 正弦波幅度 */
#define MOTOR_CSV_FREQUENCY     0.5         /* 正弦波频率 (Hz) */
#define MOTOR_CSV_PERIOD_MS     10          /* 控制周期 (ms) */

/******************************************************************************
 * 电源板配置 (UART - TTL)
 ******************************************************************************/
#define POWER_UART_DEVICE       "/dev/ttyUSB0"  /* 根据实际设备修改 */
#define POWER_UART_BAUDRATE     115200
#define POWER_UART_DATA_BITS    8
#define POWER_UART_STOP_BITS    1
#define POWER_UART_PARITY       'N'         /* N=None, O=Odd, E=Even */

/* 电源板控制参数 */
#define POWER_MAX_CURRENT       4000        /* 最大电流 (mA) - 4A */
#define POWER_MIN_CURRENT       50          /* 最小电流 (mA) */
#define POWER_DEFAULT_CURRENT   440         /* 默认电流 0.44A */

/* 电源板读取频率配置 */
#define POWER_READ_PERIOD_MS    20          /* 电源板读取周期 50Hz */
#define POWER_PRINT_PERIOD_MS   1000        /* 电源板打印周期 1Hz */

/******************************************************************************
 * 编码器配置 (RS485)
 ******************************************************************************/
#define ENCODER_UART_DEVICE     "/dev/ttyUSB1"  /* USB转RS485设备 - 编码器 */
#define ENCODER_UART_BAUDRATE   115200
#define ENCODER_UART_DATA_BITS  8
#define ENCODER_UART_STOP_BITS  1
#define ENCODER_UART_PARITY     'N'

#define ENCODER_SLAVE_ADDR      2           /* RS485设备地址 */
#define ENCODER_RESOLUTION      4096        /* 编码器分辨率 */

/* 编码器读取频率配置 */
#define ENCODER_READ_PERIOD_MS      10      /* 编码器读取周期 100Hz */
#define ENCODER_PRINT_PERIOD_MS     500     /* 编码器打印周期 2Hz */
#define ENCODER_MODBUS_REG_ADDR     0x0000  /* 位置数据寄存器地址 */
#define ENCODER_MODBUS_FUNC_CODE    0x03    /* 读取保持寄存器功能码 */

/* 编码器调试选项 */
#define ENCODER_DEBUG_RAW_DATA      0       /* 打印原始Modbus数据帧用于调试 */

/* 压力计配置 (RS485) */
#define PRESSURE_UART_DEVICE    "/dev/ttyUSB1"  /* 与编码器共用USB转485 */
#define PRESSURE_UART_BAUDRATE  115200
#define PRESSURE_UART_DATA_BITS 8
#define PRESSURE_UART_STOP_BITS 1
#define PRESSURE_UART_PARITY    'N'

#define PRESSURE_SLAVE_ADDR     1           /* RS485设备地址 (已修改为01) */
#define PRESSURE_DECIMAL_PLACES 3           /* 小数点位数 (3位 = 0.001分辨率) */
#define PRESSURE_UNIT           "kg"        /* 单位 */

/* 压力计读取频率配置 */
#define PRESSURE_READ_PERIOD_MS     10      /* 压力计读取周期 100Hz */
#define PRESSURE_PRINT_PERIOD_MS    1000    /* 压力计打印周期 1Hz */
#define PRESSURE_MODBUS_REG_ADDR    0x0000  /* 压力值寄存器地址 */
#define PRESSURE_MODBUS_FUNC_CODE   0x03    /* 读取保持寄存器功能码 */
#define PRESSURE_DECIMAL_REG_ADDR   0x0001  /* 小数点设置寄存器地址 */

/******************************************************************************
 * 新增重量采集模块配置 (UART - TTL, /dev/ttyUSB2)
 * 协议与电源板兼容，使用相同的通信协议
 ******************************************************************************/
#define WEIGHT_UART_DEVICE      "/dev/ttyUSB2"  /* 重量采集模块串口设备 */
#define WEIGHT_UART_BAUDRATE    115200
#define WEIGHT_UART_DATA_BITS   8
#define WEIGHT_UART_STOP_BITS   1
#define WEIGHT_UART_PARITY      'N'         /* N=None, O=Odd, E=Even */

/* 重量采集模块读取频率配置 */
#define WEIGHT_READ_PERIOD_MS   10          /* 重量采集读取周期 100Hz */
#define WEIGHT_PRINT_PERIOD_MS  1000        /* 重量采集打印周期 1Hz */

/******************************************************************************
 * 系统运行参数
 ******************************************************************************/
#define SYS_CONTROL_PERIOD_MS           10          /* 主控制周期 (ms) */
#define SYS_LOG_PERIOD_MS               100         /* 日志输出周期 (ms) */
#define SYS_DATA_RECORD_PERIOD_MS       10          /* 数据记录周期 (ms) */

/* 共享内存通信参数 */
#define SHM_DATA_OUTPUT_PERIOD_MS       50          /* 共享内存数据输出周期 (ms) - 20Hz */
#define SHM_DATA_COLLECTION_PERIOD_MS   10          /* 数据采集周期 (ms) - 100Hz */

/* 速度滤波参数 */
#define SPEED_FILTER_WINDOW_SIZE        5           /* 速度移动平均滤波窗口大小 */
#define SPEED_FILTER_ALPHA              0.8f        /* 速度低通滤波系数 0-1 */

/* 演示模式参数 */
#define DEMO_RUN_TIME_S         10          /* 演示运行时间 (秒) */
#define DEMO_ENABLE_SINE_WAVE   1           /* 启用正弦波运动 */

/******************************************************************************
 * 线程优先级配置 (Linux实时调度)
 ******************************************************************************/
#define THREAD_PRIORITY_HIGH    80          /* 控制线程优先级 */
#define THREAD_PRIORITY_NORMAL  50          /* 数据处理线程优先级 */
#define THREAD_PRIORITY_LOW     20          /* 日志线程优先级 */

/******************************************************************************
 * 缓冲区大小配置
 ******************************************************************************/
#define BUFFER_SIZE_SMALL       256
#define BUFFER_SIZE_MEDIUM      1024
#define BUFFER_SIZE_LARGE       4096
#define BUFFER_SIZE_HUGE        65536

/******************************************************************************
 * 错误码定义
 ******************************************************************************/
typedef enum {
    ERR_OK = 0,                 /* 成功 */
    ERR_GENERAL = -1,           /* 通用错误 */
    ERR_INVALID_PARAM = -2,     /* 无效参数 */
    ERR_INVALID_STATE = -3,     /* 无效状态 */
    ERR_NO_MEMORY = -4,         /* 内存不足 */
    ERR_TIMEOUT = -5,           /* 超时 */
    ERR_COMM_FAIL = -6,         /* 通信失败 */
    ERR_DEVICE_NOT_FOUND = -7,  /* 设备未找到 */
    ERR_DEVICE_BUSY = -8,       /* 设备忙 */
    ERR_NOT_INITIALIZED = -9,   /* 未初始化 */
    ERR_ALREADY_INIT = -10,     /* 已初始化 */
    ERR_THREAD_CREATE = -11,    /* 线程创建失败 */
    ERR_INIT_FAIL = -12,        /* 初始化失败 */
    ERR_OPERATION_NOT_ALLOWED = -13, /* 操作不允许 */
} ErrorCode_t;

/******************************************************************************
 * 系统状态定义
 ******************************************************************************/
typedef enum {
    SYS_STATE_INIT = 0,         /* 初始化状态 */
    SYS_STATE_READY,            /* 就绪状态 */
    SYS_STATE_RUNNING,          /* 运行状态 */
    SYS_STATE_PAUSED,           /* 暂停状态 */
    SYS_STATE_ERROR,            /* 错误状态 */
    SYS_STATE_SHUTDOWN,         /* 关闭状态 */
} SystemState_t;

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_CONFIG_H__ */
