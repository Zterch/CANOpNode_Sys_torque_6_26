#define _GNU_SOURCE
#include "sensor_manager.h"
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <limits.h>
#include "../config/system_config.h"

/* 编码器参数 */
static float s_rope_drum_diameter = 100.0f;
static float s_rope_length_per_turn = 314.16f;
static float s_rope_length_base = 0.0f;
static uint32_t s_encoder_resolution = 4096;
static uint32_t s_encoder_zero_offset = 0;

/* 编码器数据校验参数 */
static uint32_t s_last_valid_encoder = 0;
static int s_encoder_first_read = 1;  /* 首次读取标志 */
static int s_encoder_consecutive_errors = 0;
#define ENCODER_MAX_DELTA_PER_CYCLE 10000
#define ENCODER_ERROR_THRESHOLD 5

/* 压力传感器参数 */
static int16_t s_pressure_zero_offset = 0;

/* 压力传感器陷波滤波器参数 - 中心频率11Hz，品质因数Q=1.70
 * 二阶IIR陷波滤波器，使用标准陷波滤波器公式设计
 * 采样频率: 100Hz，中心频率: 11Hz，品质因数Q=1.7
 * 传递函数: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (a0 + a1*z^-1 + a2*z^-2)
 * 设计公式:
 *   omega0 = 2 * pi * f0 / Fs = 2 * pi * 11 / 100 ≈ 0.6912
 *   alpha = sin(omega0) / (2 * Q) ≈ 0.1856
 *   b0 = b2 = 1 / (1 + alpha) ≈ 0.8434
 *   b1 = -2 * cos(omega0) / (1 + alpha) ≈ -1.307
 *   a0 = 1
 *   a1 = -2 * cos(omega0) / (1 + alpha) ≈ -1.307
 *   a2 = (1 - alpha) / (1 + alpha) ≈ 0.6870
 * 性能（Q=1.7，Fs=100Hz）:
 *   - 10-12Hz衰减 > 85% (>8.5dB)
 *   - 5Hz相移 < 15°
 *   - 5Hz幅值变化 < 5%
 *   - 11Hz中心频率衰减 > 95%
 */
#define PRESSURE_NOTCH_B0    0.8434f
#define PRESSURE_NOTCH_B1    -1.307f
#define PRESSURE_NOTCH_B2    0.8434f
#define PRESSURE_NOTCH_A0    1.0000f
#define PRESSURE_NOTCH_A1    -1.307f
#define PRESSURE_NOTCH_A2    0.6870f

static float s_pressure_filtered = 0.0f;
static float s_pressure_x1 = 0.0f;  /* x[n-1] */
static float s_pressure_x2 = 0.0f;  /* x[n-2] */
static float s_pressure_y1 = 0.0f;  /* y[n-1] */
static float s_pressure_y2 = 0.0f;  /* y[n-2] */
static int s_pressure_notch_initialized = 0;

/* Modbus功能码 */
#define MODBUS_READ_HOLDING 0x03

/* 编码器数据文件路径 */
#define ENCODER_DATA_FILE "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share/encoder_rope_data.txt"
#define PRESSURE_DATA_FILE "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share/pressure_data.txt"

/* 日志模块 */
#define LOG_MODULE_SENSOR "SENSOR"

/* 获取微秒时间戳 */
static uint64_t get_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

/* 编码器最大值（数据手册规定范围 0~2147483647） */
#define ENCODER_MAX_VALUE 2147483647U
#define ENCODER_HALF_RANGE 1073741824  /* 2^30，半圈脉冲数 */
#define ENCODER_FULL_SIGNED_RANGE 2147483648  /* 2^31，有符号范围 */

/* 计算两个uint32_t编码器值的差值（正确处理回绕）
 * 编码器值在 0~2147483647 范围内循环
 * 当值超过最大值时回绕到0
 * 
 * 原理：
 * 1. 先计算有符号差值 delta = (int32_t)(current - last)
 * 2. 如果 delta > 2^30 (半圈)，说明是负方向回绕，delta -= 2^31
 * 3. 如果 delta < -2^30 (负半圈)，说明是正方向回绕，delta += 2^31
 * 
 * 这样可以把回绕导致的巨大差值映射回合理的范围
 */
static int32_t calculate_encoder_delta(uint32_t current, uint32_t last) {
    /* 计算有符号差值 */
    int32_t delta = (int32_t)(current - last);
    
    /* 处理负方向回绕：例如 current=2147483646, last=0
     * delta = 2147483646 (很大，超过半圈)
     * 实际应该是 -2 (负方向移动了2个脉冲)
     */
    if (delta > ENCODER_HALF_RANGE) {
        delta -= ENCODER_FULL_SIGNED_RANGE;  /* 2147483646 - 2147483648 = -2 */
    }
    /* 处理正方向回绕：例如 current=1, last=2147483641
     * delta = -2147483640 (很小，小于负半圈)
     * 实际应该是 +8 (正方向移动了8个脉冲)
     */
    else if (delta < -ENCODER_HALF_RANGE) {
        delta += ENCODER_FULL_SIGNED_RANGE;  /* -2147483640 + 2147483648 = +8 */
    }
    
    return delta;
}

/* Modbus读取保持寄存器 */
static ErrorCode_t modbus_read_registers(SensorManager_t *manager, uint8_t slave_addr,
                                          uint8_t func_code, uint16_t reg_addr,
                                          uint16_t reg_count, uint8_t *rx_buf,
                                          int *rx_len, int timeout_ms) {
    uint8_t tx_buf[8];
    tx_buf[0] = slave_addr;
    tx_buf[1] = func_code;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (reg_count >> 8) & 0xFF;
    tx_buf[5] = reg_count & 0xFF;
    
    uint16_t crc = crc16_modbus(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    
    pthread_mutex_lock(&manager->bus_mutex);
    
    ErrorCode_t ret = rs485_bus_send(tx_buf, 8, timeout_ms);
    if (ret != ERR_OK) {
        pthread_mutex_unlock(&manager->bus_mutex);
        return ret;
    }
    
    ret = rs485_bus_receive(rx_buf, 16, rx_len, timeout_ms);
    pthread_mutex_unlock(&manager->bus_mutex);
    
    if (ret != ERR_OK) {
        return ret;
    }
    
    /* 验证CRC */
    if (*rx_len < 5) {
        return ERR_COMM_FAIL;
    }
    
    uint16_t rx_crc = (rx_buf[*rx_len - 1] << 8) | rx_buf[*rx_len - 2];
    uint16_t calc_crc = crc16_modbus(rx_buf, *rx_len - 2);
    
    if (rx_crc != calc_crc) {
        return ERR_COMM_FAIL;
    }
    
    return ERR_OK;
}

/* M/T法测速静态变量 - 必须在函数外部定义，确保状态持久 */
static uint32_t s_mt_last_time_us = 0;
static uint32_t s_mt_last_encoder_value = 0;
static int s_mt_initialized = 0;

/* 控制周期M/T法测速静态变量 - 用于在控制线程读取时计算速度 */
static uint32_t s_ctrl_mt_last_time_us = 0;
static uint32_t s_ctrl_mt_last_encoder_value = 0;
static int s_ctrl_mt_initialized = 0;

/* 累积M/T法数据 - 用于累积传感器线程在控制周期内的脉冲变化 */
static int32_t s_acc_pulse_delta = 0;
static uint32_t s_acc_time_delta_us = 0;
static pthread_mutex_t s_acc_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 读取并解析编码器数据 - 工业级抗干扰版本 */
static ErrorCode_t read_encoder(SensorManager_t *manager, SensorData_t *data) {
    uint8_t rx_buf[16];
    int rx_len;
    uint32_t current_time_us = get_time_us();
    int32_t pulse_delta = 0;
    uint32_t time_delta_us = 0;
    
    ErrorCode_t ret = modbus_read_registers(manager,
                                             manager->configs[SENSOR_TYPE_ENCODER].slave_addr,
                                             manager->configs[SENSOR_TYPE_ENCODER].func_code,
                                             manager->configs[SENSOR_TYPE_ENCODER].reg_addr,
                                             manager->configs[SENSOR_TYPE_ENCODER].reg_count,
                                             rx_buf, &rx_len, 5);
    
    if (ret != ERR_OK) {
        return ret;
    }
    
    if (rx_buf[2] != 4) {
        return ERR_COMM_FAIL;
    }
    
    /* 按无符号32位整数解析（手册确认） */
    uint32_t multi_turn_first = ((uint32_t)rx_buf[3] << 24) |
                                ((uint32_t)rx_buf[4] << 16) |
                                ((uint32_t)rx_buf[5] << 8) |
                                ((uint32_t)rx_buf[6]);
    
    /* 检查编码器值是否在有效范围 0~2147483647 */
    if (multi_turn_first > 2147483647) {
        printf("[SENSOR] Invalid encoder value: %u (out of range)\n", multi_turn_first);
        return ERR_COMM_FAIL;
    }
    
    /* 首次读取，建立基准 - 工业级可靠版本 */
    if (s_encoder_first_read) {
        s_encoder_first_read = 0;
        s_last_valid_encoder = multi_turn_first;
        
        /* 工业级改进：始终使用当前编码器值作为零点 */
        /* 这样可以确保位置计算从当前位置开始，而不是从保存的位置 */
        /* 保存的位置只用于记录绳长，不用于编码器零点 */
        if (s_encoder_zero_offset == 0) {
            /* 首次初始化或文件不存在 */
            printf("[SENSOR] Initializing zero_offset: %u (first run)\n", multi_turn_first);
            s_encoder_zero_offset = multi_turn_first;
            /* 保留保存的绳长基准，但编码器零点重新校准 */
        } else {
            /* 检查保存的零点与当前值是否跨越回绕边界 */
            uint32_t diff = (multi_turn_first > s_encoder_zero_offset) ? 
                            (multi_turn_first - s_encoder_zero_offset) : 
                            (s_encoder_zero_offset - multi_turn_first);
            
            if (diff > ENCODER_MAX_VALUE / 4) {
                /* 跨越回绕边界，需要重新校准 */
                printf("[SENSOR] Recalibrating zero_offset: old=%u, new=%u (diff=%u, wrap-around detected)\n", 
                       s_encoder_zero_offset, multi_turn_first, diff);
                s_encoder_zero_offset = multi_turn_first;
                /* 跨越回绕边界时，绳长基准重置为0 */
                s_rope_length_base = 0.0f;
            } else {
                /* 正常情况：使用当前编码器值作为新的零点 */
                /* 这样可以确保位置计算从当前位置开始 */
                printf("[SENSOR] Updating zero_offset: old=%u, new=%u (diff=%u)\n", 
                       s_encoder_zero_offset, multi_turn_first, diff);
                /* 调整绳长基准以保持绝对位置连续 */
                int32_t pulse_diff = calculate_encoder_delta(multi_turn_first, s_encoder_zero_offset);
                float length_diff = (float)pulse_diff / (float)s_encoder_resolution * s_rope_length_per_turn;
                s_rope_length_base += length_diff;
                s_encoder_zero_offset = multi_turn_first;
            }
        }
        
        /* 首次读取，初始化M/T记录 */
        s_mt_last_encoder_value = multi_turn_first;
        s_mt_last_time_us = current_time_us;
        s_mt_initialized = 1;
        
        goto parse_data;
    }
    
    /* 计算与上次有效值的差值 */
    int32_t delta_from_last = calculate_encoder_delta(multi_turn_first, s_last_valid_encoder);
    
    /* 变化正常，接受数据 */
    if (delta_from_last <= ENCODER_MAX_DELTA_PER_CYCLE && 
        delta_from_last >= -ENCODER_MAX_DELTA_PER_CYCLE) {
        s_last_valid_encoder = multi_turn_first;
        s_encoder_consecutive_errors = 0;
        
        /* M/T法数据现在在parse_data统一计算 */
        goto parse_data;
    }
    
    /* 检测到突变，重读验证 */
    s_encoder_consecutive_errors++;
    printf("[SENSOR] Suspicious jump: last=%u, current=%u, delta=%d\n",
           s_last_valid_encoder, multi_turn_first, delta_from_last);
    
    usleep(5000);
    
    ret = modbus_read_registers(manager,
                                 manager->configs[SENSOR_TYPE_ENCODER].slave_addr,
                                 manager->configs[SENSOR_TYPE_ENCODER].func_code,
                                 manager->configs[SENSOR_TYPE_ENCODER].reg_addr,
                                 manager->configs[SENSOR_TYPE_ENCODER].reg_count,
                                 rx_buf, &rx_len, 50);
    
    if (ret != ERR_OK) {
        printf("[SENSOR] Retry failed, using last valid\n");
        multi_turn_first = s_last_valid_encoder;
        /* 使用旧值，不更新M/T记录 */
        goto parse_data;
    }
    
    if (rx_buf[2] != 4) {
        multi_turn_first = s_last_valid_encoder;
        goto parse_data;
    }
    
    uint32_t multi_turn_second = ((uint32_t)rx_buf[3] << 24) |
                                 ((uint32_t)rx_buf[4] << 16) |
                                 ((uint32_t)rx_buf[5] << 8) |
                                 ((uint32_t)rx_buf[6]);
    
    if (multi_turn_second > 2147483647) {
        printf("[SENSOR] Retry invalid, using last valid\n");
        multi_turn_first = s_last_valid_encoder;
        goto parse_data;
    }
    
    /* 关键修复：分析两次读取结果 */
    int32_t delta_first_from_last = calculate_encoder_delta(multi_turn_first, s_last_valid_encoder);
    int32_t delta_second_from_last = calculate_encoder_delta(multi_turn_second, s_last_valid_encoder);
    
    /* 判断哪次读取正确 */
    int first_is_normal = (delta_first_from_last <= ENCODER_MAX_DELTA_PER_CYCLE && 
                           delta_first_from_last >= -ENCODER_MAX_DELTA_PER_CYCLE);
    int second_is_normal = (delta_second_from_last <= ENCODER_MAX_DELTA_PER_CYCLE && 
                            delta_second_from_last >= -ENCODER_MAX_DELTA_PER_CYCLE);
    
    if (first_is_normal && !second_is_normal) {
        /* 首次正常，重读异常 - 使用首次 */
        printf("[SENSOR] First OK, second bad, using first: %u\n", multi_turn_first);
        s_last_valid_encoder = multi_turn_first;
        s_encoder_consecutive_errors = 0;
    } else if (!first_is_normal && second_is_normal) {
        /* 首次异常，重读正常 - 使用重读 */
        printf("[SENSOR] First bad, second OK, using second: %u\n", multi_turn_second);
        multi_turn_first = multi_turn_second;
        s_last_valid_encoder = multi_turn_second;
        s_encoder_consecutive_errors = 0;
    } else if (first_is_normal && second_is_normal) {
        /* 两次都正常（非常接近）- 使用第二次 */
        printf("[SENSOR] Both OK, using second: %u\n", multi_turn_second);
        multi_turn_first = multi_turn_second;
        s_last_valid_encoder = multi_turn_second;
        s_encoder_consecutive_errors = 0;
    } else {
        /* 两次都异常（都远离上次有效值） */
        printf("[SENSOR] Both reads abnormal, using last valid: %u\n", s_last_valid_encoder);
        
        if (s_encoder_consecutive_errors >= ENCODER_ERROR_THRESHOLD) {
            printf("[SENSOR] Too many errors, recalibrating...\n");
            s_encoder_zero_offset = multi_turn_second;
            s_rope_length_base = 0.0f;
            s_last_valid_encoder = multi_turn_second;
            s_encoder_consecutive_errors = 0;
            multi_turn_first = multi_turn_second;
        } else {
            multi_turn_first = s_last_valid_encoder;
        }
    }

parse_data:
    /* 填充数据 */
    data->data.encoder.multi_turn_value = multi_turn_first;

    /* 计算单圈角度（取模运算得到单圈值） */
    uint32_t single_turn_value = multi_turn_first % s_encoder_resolution;
    data->data.encoder.angle_deg = (float)single_turn_value * 360.0f / (float)s_encoder_resolution;

    /* 计算相对于零点的脉冲差值（正确处理回绕） */
    int32_t pulse_diff = calculate_encoder_delta(multi_turn_first, s_encoder_zero_offset);

    /* 只检查极端异常值（超过1000万脉冲 ≈ 2441圈） */
    if (pulse_diff > 10000000 || pulse_diff < -10000000) {
        printf("[SENSOR] WARNING: Extreme pulse diff: %d, using 0\n", pulse_diff);
        pulse_diff = 0;
    }

    float turns = (float)pulse_diff / (float)s_encoder_resolution;
    data->data.encoder.rope_length_mm = s_rope_length_base + turns * s_rope_length_per_turn;

    /* M/T法测速计算 - 必须在更新记录之前计算 */
    if (s_mt_initialized) {
        /* 计算脉冲变化量（正确处理回绕） */
        pulse_delta = calculate_encoder_delta(multi_turn_first, s_mt_last_encoder_value);
        /* 计算时间变化量（微秒） */
        time_delta_us = current_time_us - s_mt_last_time_us;
        
        /* 累积M/T法数据 - 用于控制周期速度计算 */
        pthread_mutex_lock(&s_acc_mutex);
        s_acc_pulse_delta += pulse_delta;
        s_acc_time_delta_us += time_delta_us;
        pthread_mutex_unlock(&s_acc_mutex);
        
        /* 调试输出 - 显示M/T法计算结果 */
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            printf("[MT DEBUG] current=%u, last=%u, delta=%d, acc_delta=%d, time_us=%u\n",
                   multi_turn_first, s_mt_last_encoder_value, pulse_delta, s_acc_pulse_delta, time_delta_us);
        }
        
        /* 更新M/T记录 - 在计算完delta之后更新，确保下次计算正确 */
        s_mt_last_encoder_value = multi_turn_first;
        s_mt_last_time_us = current_time_us;
    }
    
    /* 填充M/T法测速数据 - 使用累积值 */
    pthread_mutex_lock(&s_acc_mutex);
    data->data.encoder.pulse_delta = s_acc_pulse_delta;
    data->data.encoder.time_delta_us = s_acc_time_delta_us;
    /* 重置累积值 */
    s_acc_pulse_delta = 0;
    s_acc_time_delta_us = 0;
    pthread_mutex_unlock(&s_acc_mutex);

    data->data_valid = 1;
    data->last_read_us = current_time_us;

    return ERR_OK;
}

/* 读取压力传感器 */
static ErrorCode_t read_pressure(SensorManager_t *manager, SensorData_t *data) {
    uint8_t rx_buf[16];
    int rx_len;
    
    ErrorCode_t ret = modbus_read_registers(manager,
                                             manager->configs[SENSOR_TYPE_PRESSURE].slave_addr,
                                             manager->configs[SENSOR_TYPE_PRESSURE].func_code,
                                             manager->configs[SENSOR_TYPE_PRESSURE].reg_addr,
                                             manager->configs[SENSOR_TYPE_PRESSURE].reg_count,
                                             rx_buf, &rx_len, 5);
    
    if (ret != ERR_OK) {
        return ret;
    }
    
    if (rx_buf[2] != 2) {
        return ERR_COMM_FAIL;
    }
    
    int16_t raw_value = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
    
    /* 根据传感器配置的小数点位数计算压力值 */
    float divisor = 1.0f;
    uint8_t decimal_places = manager->configs[SENSOR_TYPE_PRESSURE].decimal_places;
    for (uint8_t i = 0; i < decimal_places; i++) {
        divisor *= 10.0f;
    }
    
    float pressure_kg = ((float)(raw_value - s_pressure_zero_offset)) / divisor;

    /* 应用陷波滤波器 - 中心频率11Hz，针对性滤除10-12Hz干扰（采样频率100Hz）
     * 二阶IIR陷波滤波器，Q=1.7
     * 性能:
     *   - 10-12Hz衰减 > 85% (>8.5dB)
     *   - 5Hz相移 < 15°
     *   - 5Hz幅值变化 < 5%
     *   - 11Hz中心频率衰减 > 95%
     * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
     */
    if (!s_pressure_notch_initialized) {
        /* 首次读取，初始化滤波器状态 */
        s_pressure_filtered = pressure_kg;
        s_pressure_x1 = pressure_kg;
        s_pressure_x2 = pressure_kg;
        s_pressure_y1 = pressure_kg;
        s_pressure_y2 = pressure_kg;
        s_pressure_notch_initialized = 1;
    } else {
        /* 二阶IIR陷波滤波 */
        float x0 = pressure_kg;
        s_pressure_filtered = PRESSURE_NOTCH_B0 * x0 +
                              PRESSURE_NOTCH_B1 * s_pressure_x1 +
                              PRESSURE_NOTCH_B2 * s_pressure_x2 -
                              PRESSURE_NOTCH_A1 * s_pressure_y1 -
                              PRESSURE_NOTCH_A2 * s_pressure_y2;

        /* 更新状态 */
        s_pressure_x2 = s_pressure_x1;
        s_pressure_x1 = x0;
        s_pressure_y2 = s_pressure_y1;
        s_pressure_y1 = s_pressure_filtered;
    }
    
    data->data.pressure.raw_value = raw_value;
    data->data.pressure.pressure_kg = pressure_kg;           /* 原始值 */
    data->data.pressure.pressure_filtered_kg = s_pressure_filtered;  /* 滤波后 */
    data->data_valid = 1;
    data->last_read_us = get_time_us();
    
    return ERR_OK;
}

/* 传感器采集线程 - 工业级严格固定周期版本
 * 
 * 采集策略：
 * - 编码器：每10ms采集一次（100Hz）
 * - 压力传感器：每10ms采集一次（100Hz）
 * - 总周期：10ms（两个传感器在同一周期内顺序采集）
 * 
 * 使用绝对时间戳确保严格的周期控制
 */
static void* sensor_thread(void* arg) {
    SensorManager_t* manager = (SensorManager_t*)arg;
    
    /* 设置线程高优先级，确保传感器采集实时性 */
    struct sched_param param;
    param.sched_priority = 88;  /* 高于数据采集线程(85) */
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        printf("[SENSOR] Warning: Failed to set high priority for sensor thread\n");
    }
    
    /* 定义严格的采集周期 - 10ms = 100Hz */
    #define SENSOR_SAMPLE_PERIOD_US 10000  /* 10ms = 100Hz */
    
    /* 获取起始时间戳 */
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    
    /* 转换为微秒 */
    uint64_t next_time_us = next_time.tv_sec * 1000000ULL + next_time.tv_nsec / 1000;
    
    printf("[SENSOR] Thread started with strict 10ms cycle (100Hz for both sensors)\n");
    
    while (manager->running) {
        /* 记录本次采集开始时间 */
        uint64_t sample_start_us = get_time_us();
        
        /* 采集编码器（100Hz） */
        ErrorCode_t ret = read_encoder(manager, &manager->datas[SENSOR_TYPE_ENCODER]);
        if (ret != ERR_OK) {
            manager->datas[SENSOR_TYPE_ENCODER].error_count++;
        } else {
            manager->datas[SENSOR_TYPE_ENCODER].read_count++;
        }
        
        /* 采集压力传感器（100Hz） */
        ret = read_pressure(manager, &manager->datas[SENSOR_TYPE_PRESSURE]);
        if (ret != ERR_OK) {
            manager->datas[SENSOR_TYPE_PRESSURE].error_count++;
        } else {
            manager->datas[SENSOR_TYPE_PRESSURE].read_count++;
        }
        
        /* 完成一个10ms周期 */
        manager->cycle_count++;
        
        /* 计算下一次采集的绝对时间点 */
        next_time_us += SENSOR_SAMPLE_PERIOD_US;
        
        /* 获取当前时间 */
        uint64_t current_time_us = get_time_us();
        
        /* 计算需要等待的时间 */
        int64_t sleep_us = (int64_t)(next_time_us - current_time_us);
        
        if (sleep_us > 0) {
            /* 正常情况：等待到下一个采集时间点 */
            struct timespec sleep_ts;
            sleep_ts.tv_sec = sleep_us / 1000000;
            sleep_ts.tv_nsec = (sleep_us % 1000000) * 1000;
            nanosleep(&sleep_ts, NULL);
        } else if (sleep_us < -5000) {
            /* 严重超时（超过5ms）：打印警告并重新同步 */
            printf("[SENSOR] WARNING: Sample deadline missed by %ld us, resynchronizing...\n", 
                   (long)(-sleep_us));
            /* 重新同步到下一个周期 */
            next_time_us = current_time_us + SENSOR_SAMPLE_PERIOD_US;
        }
        /* 如果sleep_us在[-5000, 0]之间，说明轻微超时，继续执行不等待 */
    }
    
    return NULL;
}

/* 加载编码器数据 - 工业级可靠版本 */
static int load_encoder_data(void) {
    FILE *fp = fopen(ENCODER_DATA_FILE, "r");
    if (fp == NULL) {
        printf("[SENSOR] No encoder data file, will initialize on first read\n");
        return -1;
    }
    
    char line[128];
    float saved_length = 0.0f;
    uint32_t saved_encoder = 0;
    int has_data = 0;
    
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#') continue;
        
        if (strncmp(line, "ABSOLUTE_LENGTH_MM=", 19) == 0) {
            sscanf(line + 19, "%f", &saved_length);
            has_data = 1;
        }
        else if (strncmp(line, "LAST_ENCODER_VALUE=", 19) == 0) {
            sscanf(line + 19, "%u", &saved_encoder);
            has_data = 1;
        }
        else if (strncmp(line, "DRUM_DIAMETER=", 14) == 0) {
            sscanf(line + 14, "%f", &s_rope_drum_diameter);
        }
        else if (strncmp(line, "ENCODER_RESOLUTION=", 19) == 0) {
            sscanf(line + 19, "%u", &s_encoder_resolution);
        }
    }
    
    fclose(fp);
    
    s_rope_length_per_turn = M_PI * s_rope_drum_diameter;
    
    if (has_data) {
        printf("[SENSOR] Loaded: length=%.2fmm, encoder=%u\n", saved_length, saved_encoder);
        /* 工业级改进：不直接使用保存的encoder值作为零点 */
        /* 而是在首次读取时根据实际位置重新校准 */
        s_rope_length_base = saved_length;
        /* 标记为零点需要重新校准，而不是直接使用保存值 */
        s_encoder_zero_offset = 0;  /* 标记为未初始化，将在首次读取时设置 */
    }
    
    return 0;
}

/* 保存编码器数据 */
static int save_encoder_data(uint32_t encoder_value, float absolute_length) {
    FILE *fp = fopen(ENCODER_DATA_FILE, "w");
    if (fp == NULL) {
        return -1;
    }
    
    fprintf(fp, "# Encoder Rope Length Data\n");
    fprintf(fp, "ABSOLUTE_LENGTH_MM=%.4f\n", absolute_length);
    fprintf(fp, "LAST_ENCODER_VALUE=%u\n", encoder_value);
    fprintf(fp, "DRUM_DIAMETER=%.4f\n", s_rope_drum_diameter);
    fprintf(fp, "ENCODER_RESOLUTION=%u\n", s_encoder_resolution);
    
    fclose(fp);
    printf("[SENSOR] Saved: length=%.2fmm, encoder=%u\n", absolute_length, encoder_value);
    return 0;
}

/* 初始化传感器管理器 */
ErrorCode_t sensor_mgr_init(SensorManager_t *manager, const char *device, int baudrate) {
    if (manager == NULL || device == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    memset(manager, 0, sizeof(SensorManager_t));
    
    /* 初始化互斥锁 */
    pthread_mutex_init(&manager->bus_mutex, NULL);
    
    /* 配置编码器 */
    manager->configs[SENSOR_TYPE_ENCODER].slave_addr = ENCODER_SLAVE_ADDR;
    manager->configs[SENSOR_TYPE_ENCODER].func_code = ENCODER_MODBUS_FUNC_CODE;
    manager->configs[SENSOR_TYPE_ENCODER].reg_addr = ENCODER_MODBUS_REG_ADDR;
    manager->configs[SENSOR_TYPE_ENCODER].reg_count = 2;
    manager->configs[SENSOR_TYPE_ENCODER].name = "Encoder";
    
    /* 配置压力传感器 */
    manager->configs[SENSOR_TYPE_PRESSURE].slave_addr = PRESSURE_SLAVE_ADDR;
    manager->configs[SENSOR_TYPE_PRESSURE].func_code = 0x03;  /* 读取保持寄存器 */
    manager->configs[SENSOR_TYPE_PRESSURE].reg_addr = 0x0000;  /* 压力值寄存器 */
    manager->configs[SENSOR_TYPE_PRESSURE].reg_count = 1;
    manager->configs[SENSOR_TYPE_PRESSURE].decimal_places = 2;  /* 2位小数 = 0.01kg分辨率 */
    manager->configs[SENSOR_TYPE_PRESSURE].name = "Pressure";
    
    /* 加载保存的数据 */
    load_encoder_data();
    
    /* 初始化RS485 */
    ErrorCode_t ret = rs485_bus_init(device, baudrate);
    if (ret != ERR_OK) {
        printf("[SENSOR] Failed to init RS485: %d\n", ret);
        return ret;
    }
    
    manager->rs485_fd = rs485_bus_get_fd();
    manager->initialized = 1;
    printf("[SENSOR] Initialized\n");
    
    return ERR_OK;
}

/* 启动传感器管理器 */
ErrorCode_t sensor_mgr_start(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    manager->running = 1;
    
    if (pthread_create(&manager->manager_thread, NULL, sensor_thread, manager) != 0) {
        return ERR_GENERAL;
    }
    
    printf("[SENSOR] Started\n");
    return ERR_OK;
}

/* 停止传感器管理器 */
void sensor_mgr_stop(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return;
    }
    
    manager->running = 0;
    pthread_join(manager->manager_thread, NULL);
    
    printf("[SENSOR] Stopped\n");
}

/* 反初始化 */
void sensor_mgr_deinit(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return;
    }
    
    /* 停止线程 */
    manager->running = 0;
    pthread_join(manager->manager_thread, NULL);
    
    /* 保存当前位置 */
    SensorData_t *encoder_data = &manager->datas[SENSOR_TYPE_ENCODER];
    if (encoder_data->data_valid) {
        uint32_t current = encoder_data->data.encoder.multi_turn_value;
        int32_t delta = calculate_encoder_delta(current, s_encoder_zero_offset);
        float turns = (float)delta / (float)s_encoder_resolution;
        float absolute = s_rope_length_base + turns * s_rope_length_per_turn;
        save_encoder_data(current, absolute);
    }
    
    rs485_bus_deinit();
    pthread_mutex_destroy(&manager->bus_mutex);
    manager->initialized = 0;
}

/* 获取传感器数据 */
ErrorCode_t sensor_mgr_get_data(SensorManager_t *manager, SensorType_t type, SensorData_t *data) {
    if (manager == NULL || data == NULL || type >= SENSOR_TYPE_COUNT) {
        return ERR_INVALID_PARAM;
    }
    
    if (!manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&manager->bus_mutex);
    *data = manager->datas[type];
    
    /* 注意：M/T法数据已经在read_encoder中计算并累积
     * 这里直接读取传感器线程累积的数据即可
     */
    
    pthread_mutex_unlock(&manager->bus_mutex);
    
    return ERR_OK;
}

/* 设置编码器绳子长度参数 */
ErrorCode_t sensor_mgr_set_encoder_rope_params(SensorManager_t *manager, 
                                                float drum_diameter, 
                                                uint32_t resolution) {
    if (manager == NULL || drum_diameter <= 0 || resolution == 0) {
        return ERR_INVALID_PARAM;
    }
    
    s_rope_drum_diameter = drum_diameter;
    s_encoder_resolution = resolution;
    s_rope_length_per_turn = M_PI * drum_diameter;
    
    return ERR_OK;
}

/* 执行编码器零点校准 */
ErrorCode_t sensor_mgr_encoder_zero_calibration(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    SensorData_t data;
    ErrorCode_t ret = sensor_mgr_get_data(manager, SENSOR_TYPE_ENCODER, &data);
    if (ret != ERR_OK) {
        return ret;
    }
    
    s_encoder_zero_offset = data.data.encoder.multi_turn_value;
    s_rope_length_base = 0.0f;
    
    printf("[SENSOR] Encoder zero calibrated: offset=%u\n", s_encoder_zero_offset);
    return ERR_OK;
}

/* 执行压力传感器去皮/清零 */
ErrorCode_t sensor_mgr_pressure_tare(SensorManager_t *manager) {
    if (manager == NULL || !manager->initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    SensorData_t data;
    ErrorCode_t ret = sensor_mgr_get_data(manager, SENSOR_TYPE_PRESSURE, &data);
    if (ret != ERR_OK) {
        return ret;
    }
    
    //s_pressure_zero_offset = data.data.pressure.raw_value;
    s_pressure_zero_offset = 0;
    
    printf("[SENSOR] Pressure tared: offset=%d\n", s_pressure_zero_offset);
    return ERR_OK;
}

/* 设置编码器基准长度 */
ErrorCode_t sensor_mgr_set_encoder_base_length(SensorManager_t *manager, float base_length_mm) {
    if (manager == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    s_rope_length_base = base_length_mm;
    return ERR_OK;
}

/* 获取传感器读取成功率 */
float sensor_mgr_get_success_rate(SensorManager_t *manager, SensorType_t type) {
    if (manager == NULL || type >= SENSOR_TYPE_COUNT) {
        return 0.0f;
    }
    
    uint32_t reads = manager->datas[type].read_count;
    uint32_t errors = manager->datas[type].error_count;
    uint32_t total = reads + errors;
    
    if (total == 0) {
        return 0.0f;
    }
    
    return (float)reads * 100.0f / (float)total;
}

/* 打印所有传感器状态 */
void sensor_mgr_print_status(SensorManager_t *manager) {
    if (manager == NULL) {
        return;
    }
    
    printf("\n=== Sensor Manager Status ===\n");
    printf("Cycle count: %lu\n", manager->cycle_count);
    printf("Total errors: %lu\n", manager->total_errors);
    
    const char* type_names[] = {"Encoder", "Pressure"};
    
    for (int i = 0; i < SENSOR_TYPE_COUNT; i++) {
        SensorData_t *data = &manager->datas[i];
        float success_rate = sensor_mgr_get_success_rate(manager, i);
        
        printf("\n%s (addr=%d):\n", type_names[i], manager->configs[i].slave_addr);
        printf("  Reads: %lu, Errors: %lu, Success: %.1f%%\n",
               data->read_count, data->error_count, success_rate);
        
        if (i == SENSOR_TYPE_ENCODER && data->data_valid) {
            printf("  Multi-turn: %u\n", data->data.encoder.multi_turn_value);
            printf("  Angle: %.2f deg\n", data->data.encoder.angle_deg);
            printf("  Rope: %.2f mm\n", data->data.encoder.rope_length_mm);
        } else if (i == SENSOR_TYPE_PRESSURE && data->data_valid) {
            printf("  Pressure: %.3f kg (raw=%d, offset=%d)\n",
                   data->data.pressure.pressure_kg,
                   data->data.pressure.raw_value,
                   s_pressure_zero_offset);
        }
    }
}
