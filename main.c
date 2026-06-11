/******************************************************************************
 * @file    main.c
 * @brief   CANOpNode_Sys 主程序 - 重力卸载控制系统 v3.0
 * @author  System Architect
 * @date    2026-05-11
 * @version 3.0.0
 * 
 * @description
 * 重力卸载控制系统主程序
 * - 共享内存通信：20Hz实时数据输出
 * - 算法独立监控：无论算法是否运行都可监控和控制
 * - 双向通信：数据上传 + 命令接收
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "config/system_config.h"
#include "utils/logger.h"
#include "include/shared_memory.h"
#include "drivers/sensor_manager.h"
#include "drivers/power_driver.h"
#include "drivers/motor_driver.h"
#include "drivers/weight_driver.h"  /* 新增重量采集驱动 */
#include "algorithms/gravity_unload.h"
#include "algorithms/system_check.h"

/******************************************************************************
 * 全局变量
 ******************************************************************************/
static volatile int g_running = 1;
static volatile int g_algorithm_enabled = 0;
static volatile int g_shm_initialized = 0;
static volatile uint32_t g_last_cmd_id = 0;

static SensorManager_t g_sensor_mgr;
static PowerDriver_t g_power;
static MotorDriver_t g_motor;
static WeightDriver_t g_weight;  /* 新增重量采集驱动 */
static GravityUnloadController_t g_gravity_ctrl;
static ShmManager_t g_shm_mgr;

static int g_motor_enabled = 1;  /* 电机使能标志 */
static int g_algorithm_mode = 0; /* 0=手动模式, 1=算法模式 */
static volatile int g_logging_enabled = 0; /* 数据记录标志 */

/* 手动模式下的F0计算 - 使用滑动窗口平均 */
#define MANUAL_F0_SAMPLE_COUNT  50      /* 手动模式下采集50个点计算F0 (约1秒) */
static float g_manual_f0_kg = 0.0f;     /* 手动模式下的F0值 */
static int g_manual_f0_sample_count = 0; /* 已采集的样本数 */
static float g_manual_f0_sum = 0.0f;    /* 压力累积和 */

/* 线程ID */
static pthread_t g_data_thread_tid;
static pthread_t g_command_thread_tid;
static pthread_t g_monitor_thread_tid;
static pthread_t g_collection_thread_tid;
static pthread_t g_motor_state_thread_tid;
static pthread_t g_power_state_thread_tid;
static pthread_t g_actuator_thread_tid;  /* 异步执行器控制线程 */

/* 函数前置声明 */
void start_logging(void);
void stop_logging(void);
void update_pi_terms(float p_term_mA, float i_term_mA, float d_term_mA);

/******************************************************************************
 * 异步执行器控制 - 工业级严格周期控制关键
 * 控制线程只更新目标值，实际通信由独立线程处理
 ******************************************************************************/
typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    
    /* 电机目标值 */
    float target_velocity_rpm;
    int motor_target_updated;
    
    /* 离合器目标值 */
    uint16_t target_clutch_current_ma;
    int clutch_target_updated;
    
    /* 统计 */
    uint32_t motor_update_count;
    uint32_t clutch_update_count;
    uint32_t motor_fail_count;
    uint32_t clutch_fail_count;
} ActuatorTargetBuffer_t;

static ActuatorTargetBuffer_t g_actuator_target;

/* 初始化异步执行器目标缓冲区 */
static void actuator_target_init(void) {
    pthread_mutex_init(&g_actuator_target.mutex, NULL);
    pthread_cond_init(&g_actuator_target.cond, NULL);
    g_actuator_target.target_velocity_rpm = 0.0f;
    g_actuator_target.target_clutch_current_ma = 0;
    g_actuator_target.motor_target_updated = 0;
    g_actuator_target.clutch_target_updated = 0;
    g_actuator_target.motor_update_count = 0;
    g_actuator_target.clutch_update_count = 0;
    g_actuator_target.motor_fail_count = 0;
    g_actuator_target.clutch_fail_count = 0;
}

/* 异步执行器控制线程 - 100Hz严格周期 */
static void* actuator_control_thread(void* arg) {
    (void)arg;
    
    /* 设置线程高优先级 */
    struct sched_param param;
    param.sched_priority = 85;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        printf("[ACTUATOR] Warning: Failed to set priority\n");
    }
    
    printf("[ACTUATOR] Asynchronous actuator control thread started (100Hz)\n");
    
    /* 工业级严格周期控制 */
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    uint64_t next_time_us = next_time.tv_sec * 1000000ULL + next_time.tv_nsec / 1000;
    const uint64_t period_us = 10000; /* 10ms = 100Hz */
    
    while (g_running) {
        /* 读取目标值 */
        pthread_mutex_lock(&g_actuator_target.mutex);
        float motor_target = g_actuator_target.target_velocity_rpm;
        int motor_updated = g_actuator_target.motor_target_updated;
        uint16_t clutch_target = g_actuator_target.target_clutch_current_ma;
        int clutch_updated = g_actuator_target.clutch_target_updated;
        /* 清除更新标志 */
        g_actuator_target.motor_target_updated = 0;
        g_actuator_target.clutch_target_updated = 0;
        pthread_mutex_unlock(&g_actuator_target.mutex);
        
        /* 执行电机控制（5ms超时） */
        if (motor_updated && g_motor.initialized && g_motor.enabled) {
            ErrorCode_t ret = motor_set_velocity(&g_motor, motor_target);
            if (ret == ERR_OK) {
                g_actuator_target.motor_update_count++;
            } else {
                g_actuator_target.motor_fail_count++;
            }
        }
        
        /* 执行离合器控制（5ms超时） */
        if (clutch_updated && g_power.initialized && g_power.state == POWER_STATE_ON) {
            ErrorCode_t ret = power_set_current(&g_power, clutch_target);
            if (ret == ERR_OK) {
                g_actuator_target.clutch_update_count++;
            } else {
                g_actuator_target.clutch_fail_count++;
            }
        }
        
        /* 严格周期控制 */
        next_time_us += period_us;
        uint64_t current_time_us = (uint64_t)(next_time.tv_sec * 1000000ULL + next_time.tv_nsec / 1000);
        int64_t sleep_us = (int64_t)(next_time_us - current_time_us);
        
        if (sleep_us > 0) {
            struct timespec sleep_ts;
            sleep_ts.tv_sec = sleep_us / 1000000;
            sleep_ts.tv_nsec = (sleep_us % 1000000) * 1000;
            nanosleep(&sleep_ts, NULL);
        }
        
        clock_gettime(CLOCK_MONOTONIC, &next_time);
    }
    
    printf("[ACTUATOR] Actuator control thread stopped\n");
    return NULL;
}

/* 共享状态缓冲区 - 用于线程间数据共享 */
typedef struct {
    pthread_mutex_t mutex;
    /* 传感器数据 */
    float pressure_kg;
    float rope_length_m;
    uint32_t encoder_value;
    float encoder_angle_deg;
    /* 新增重量采集模块数据 (UART/TTL, 100Hz) */
    float weight_raw_kg;
    float weight_filtered_kg;
    /* 电源数据 */
    float current_a;
    float voltage_v;
    float target_current_a;         /* 目标电流（算法计算值） */
    /* 电机数据 */
    float motor_speed_rpm;
    float motor_position_m;
    int32_t motor_status;
    float motor_linear_velocity_m_s;       /* 电机实际线速度（电机轴侧） */
    float motor_theory_linear_velocity_m_s; /* 电机理论线速度（根据重物速度计算） */
    /* 绳子速度数据 */
    float rope_velocity_raw_m_s;
    float rope_velocity_filtered_m_s;
    /* 压力传感器数据 */
    float pressure_f0_kg;           /* 静止时压力值F0 */
    float pressure_deltaf;          /* 摩擦力变化量dF */
    /* PID控制数据 */
    float pi_p_term_mA;             /* PID控制P项（比例项） */
    float pi_i_term_mA;             /* PID控制I项（积分项） */
    float pi_d_term_mA;             /* PID控制D项（微分项） */
    /* 前馈控制数据 */
    float feedforward_current_mA;   /* 前馈电流值 */
    /* 算法内部数据 */
    float algo_deltaf_kg;           /* 算法实际使用的DeltaF（用于数据一致性验证） */
    float algo_pressure_kg;         /* 算法实际使用的压力值（用于数据一致性验证） */
    /* PI累积电流 */
    float pi_last_current_mA;       /* PI部分累积电流last_current_mA（用于调试增量式PID） */
} SharedStateBuffer_t;

static SharedStateBuffer_t g_shared_state;

/* 电机转速滑动平均滤波器 - 用于平滑电机转速波动 */
#define MOTOR_SPEED_FILTER_WINDOW_SIZE  5
static float s_motor_speed_buffer[MOTOR_SPEED_FILTER_WINDOW_SIZE] = {0};
static int s_motor_speed_index = 0;
static int s_motor_speed_count = 0;

/* 电机转速滤波函数 */
static float filter_motor_speed(float raw_speed) {
    // 更新缓冲区
    s_motor_speed_buffer[s_motor_speed_index] = raw_speed;
    s_motor_speed_index = (s_motor_speed_index + 1) % MOTOR_SPEED_FILTER_WINDOW_SIZE;
    if (s_motor_speed_count < MOTOR_SPEED_FILTER_WINDOW_SIZE) {
        s_motor_speed_count++;
    }
    
    // 计算平均值
    float sum = 0.0f;
    for (int i = 0; i < s_motor_speed_count; i++) {
        sum += s_motor_speed_buffer[i];
    }
    return sum / s_motor_speed_count;
}

/* 初始化共享状态缓冲区 */
static void shared_state_init(void) {
    pthread_mutex_init(&g_shared_state.mutex, NULL);
    memset((void*)&g_shared_state + sizeof(pthread_mutex_t), 0, 
           sizeof(SharedStateBuffer_t) - sizeof(pthread_mutex_t));
    // 重置电机转速滤波器
    memset(s_motor_speed_buffer, 0, sizeof(s_motor_speed_buffer));
    s_motor_speed_index = 0;
    s_motor_speed_count = 0;
}

/* 更新传感器数据到共享缓冲区 */
static void update_sensor_to_buffer(SensorData_t *encoder, SensorData_t *pressure) {
    pthread_mutex_lock(&g_shared_state.mutex);
    if (encoder && encoder->data_valid) {
        g_shared_state.encoder_value = encoder->data.encoder.multi_turn_value;
        g_shared_state.encoder_angle_deg = encoder->data.encoder.angle_deg;
        g_shared_state.rope_length_m = encoder->data.encoder.rope_length_mm * 0.001f; // 转换为米
    }
    if (pressure && pressure->data_valid) {
        g_shared_state.pressure_kg = pressure->data.pressure.pressure_filtered_kg;  /* 使用低通滤波后的压力值 */
    }
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 更新电源数据到共享缓冲区 */
static void update_power_to_buffer(PowerDriver_t *power) {
    pthread_mutex_lock(&g_shared_state.mutex);
    if (power && power->state == POWER_STATE_ON) {
        // 直接读取预缓存的数据，不调用串口
        g_shared_state.current_a = (float)power->actual_current / 1000.0f; // mA -> A
        g_shared_state.voltage_v = (float)power->actual_voltage / 1000.0f; // mV -> V
    }
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 更新电机数据到共享缓冲区 */
static void update_motor_to_buffer(void) {
    pthread_mutex_lock(&g_shared_state.mutex);
    
    // 使用缓存读取电机速度，避免SDO阻塞
    int32_t motor_speed_raw;
    if (motor_get_velocity_cached(&g_motor, &motor_speed_raw) == ERR_OK) {
        // 对电机转速进行滑动平均滤波，平滑波动
        float motor_speed_filtered = filter_motor_speed((float)motor_speed_raw);
        g_shared_state.motor_speed_rpm = motor_speed_filtered;
    }
    g_shared_state.motor_position_m = (float)motor_get_position_m(&g_motor);
    g_shared_state.motor_status = g_motor.state;
    
    // 计算电机线速度（滑轮侧，与重物速度对应）
    // 线速度 = π * D * (电机转速(rpm) / 减速比3) / 60
    // 存入文件时需要除以减速比3，与重物速度在同一参考系
    float pulley_diameter_m = 0.2f; // 滑轮直径200mm
    float motor_speed_rpm = g_shared_state.motor_speed_rpm;
    float pulley_speed_rpm = motor_speed_rpm / 3.0f; // 减速比3:1
    g_shared_state.motor_linear_velocity_m_s = -3.14159f * pulley_diameter_m * pulley_speed_rpm / 60.0f;
    
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 电机状态更新线程 - 独立线程处理CANopen SDO读取 */
static void* motor_state_update_thread(void* arg) {
    (void)arg;
    
    /* 设置线程中等优先级 */
    struct sched_param param;
    param.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        printf("[MOTOR] Warning: Failed to set priority for motor state thread\n");
    }
    
    printf("[MOTOR] Motor state update thread started\n");
    
    while (g_running) {
        if (g_motor.initialized) {
            // 通过SDK更新电机状态（PDO方式，在独立线程中执行）
            motor_update_state(&g_motor);
        }

        // 电机状态更新频率100Hz（10ms周期，与控制循环同步）
        struct timespec delay = {0, 10000000}; /* 10ms */
        nanosleep(&delay, NULL);
    }
    
    return NULL;
}

/* 电源状态更新线程 - 独立线程处理串口读取 */
static void* power_state_update_thread(void* arg) {
    (void)arg;
    
    /* 设置线程中等优先级 */
    struct sched_param param;
    param.sched_priority = 78;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        printf("[POWER] Warning: Failed to set priority for power state thread\n");
    }
    
    printf("[POWER] Power state update thread started\n");
    
    while (g_running) {
        if (g_power.initialized && g_power.state == POWER_STATE_ON) {
            // 通过串口读取电源状态（耗时操作，在独立线程中执行）
            uint16_t current_ma, voltage_mv;
            power_get_status(&g_power, &current_ma, &voltage_mv);
        }
        
        // 电源状态更新频率50Hz（20ms周期，与数据记录同步）
        struct timespec delay = {0, 20000000}; /* 20ms */
        nanosleep(&delay, NULL);
    }
    
    return NULL;
}

/******************************************************************************
 * 外部接口实现（供算法模块调用）
 ******************************************************************************/

/**
 * @brief 更新绳子速度数据到共享状态缓冲区
 * @param raw_velocity 原始速度 (m/s)
 * @param filtered_velocity 滤波后速度 (m/s)
 */
void update_rope_velocity(float raw_velocity, float filtered_velocity) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.rope_velocity_raw_m_s = raw_velocity;
    g_shared_state.rope_velocity_filtered_m_s = filtered_velocity;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_motor_theory_velocity(float theory_velocity) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.motor_theory_linear_velocity_m_s = theory_velocity;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_pressure_f0_deltaf(float f0_kg, float deltaf) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.pressure_f0_kg = f0_kg;
    g_shared_state.pressure_deltaf = deltaf;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_pi_terms(float p_term_mA, float i_term_mA, float d_term_mA) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.pi_p_term_mA = p_term_mA;
    g_shared_state.pi_i_term_mA = i_term_mA;
    g_shared_state.pi_d_term_mA = d_term_mA;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 更新前馈电流到共享状态（供数据收集线程读取） */
void update_feedforward_current(float feedforward_mA) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.feedforward_current_mA = feedforward_mA;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 同时更新前馈电流、目标电流和算法DeltaF到共享状态（确保数据一致性） */
void update_feedforward_and_target(float feedforward_mA, float target_mA, float deltaf_kg, float pressure_kg) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.feedforward_current_mA = feedforward_mA;
    g_shared_state.target_current_a = target_mA / 1000.0f;  // mA -> A
    g_shared_state.algo_deltaf_kg = deltaf_kg;              // 算法实际使用的DeltaF
    g_shared_state.algo_pressure_kg = pressure_kg;          // 算法实际使用的压力值
    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 更新PI累积电流到共享状态（供数据收集线程读取） */
void update_pi_last_current(float last_current_mA) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.pi_last_current_mA = last_current_mA;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

uint32_t get_timestamp_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* 获取微秒时间戳 */
static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* 计算编码器变化量（正确处理回绕） */
static int32_t calculate_encoder_delta(uint32_t current, uint32_t last) {
    int32_t delta = (int32_t)(current - last);
    return delta;
}

int get_sensor_data(SensorDataRaw_t *data) {
    if (data == NULL) return -1;
    
    SensorData_t encoder_data, pressure_data;
    
    /* 获取编码器数据 - sensor_mgr_get_data已经计算了控制周期的M/T法数据 */
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) != ERR_OK) {
        return -1;
    }
    
    /* 获取压力传感器数据 */
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure_data) != ERR_OK) {
        return -1;
    }
    
    /* 填充原始数据 - 使用滤波后的压力值 */
    data->pressure_kg = pressure_data.data.pressure.pressure_filtered_kg;
    /* 应用编码器方向系数 - 确保重物上升时位置增加 */
    data->encoder_position_m = encoder_data.data.encoder.rope_length_mm / 1000.0f * ENCODER_DIRECTION;
    /* 直接使用sensor_mgr_get_data计算的M/T法数据 */
    data->encoder_pulse_delta = encoder_data.data.encoder.pulse_delta * ENCODER_DIRECTION;
    data->encoder_time_delta_us = encoder_data.data.encoder.time_delta_us;
    data->timestamp_ms = get_timestamp_ms();
    data->data_valid = (encoder_data.data_valid && pressure_data.data_valid);
    
    return 0;
}

int set_motor_velocity(float velocity) {
    if (!g_motor_enabled || !g_motor.initialized) return -1;
    
    /* 异步控制：只更新目标值，不直接通信
     * 实际通信由actuator_control_thread处理
     */
    pthread_mutex_lock(&g_actuator_target.mutex);
    g_actuator_target.target_velocity_rpm = velocity;
    g_actuator_target.motor_target_updated = 1;
    pthread_mutex_unlock(&g_actuator_target.mutex);
    
    return 0;
}

int get_motor_actual_velocity(float *velocity) {
    if (!g_motor_enabled || !g_motor.initialized || velocity == NULL) return -1;

    /* 使用缓存读取，避免CANopen阻塞调用
     * 缓存值由电机状态更新线程(10ms周期)通过PDO更新
     * 对于100Hz控制循环，10ms的缓存延迟是可接受的
     */
    int32_t vel;
    if (motor_get_velocity_cached(&g_motor, &vel) == ERR_OK) {
        *velocity = (float)vel;
        return 0;
    }
    return -1;
}

/* 测试模式：允许在电源板未就绪时运行电机控制 */
#define TEST_MODE_ALLOW_MOTOR_WITHOUT_POWER 1

int set_clutch_current(float current_mA) {
    uint16_t current_u16 = (uint16_t)(current_mA + 0.5f);

    if (current_u16 > SAFETY_CLUTCH_CURRENT_MAX_MA) {
        current_u16 = SAFETY_CLUTCH_CURRENT_MAX_MA;
    }

    /* 测试模式：电源板未就绪时跳过电流设置，只运行电机控制 */
    if (TEST_MODE_ALLOW_MOTOR_WITHOUT_POWER && !g_power.initialized) {
        static int warned = 0;
        if (!warned) {
            printf("[TEST] Power board not initialized, skipping clutch current setting\n");
            warned = 1;
        }
        return 0; /* 返回成功，允许电机继续运行 */
    }

    /* 异步控制：只更新目标值，不直接通信
     * 实际通信由actuator_control_thread处理
     */
    pthread_mutex_lock(&g_actuator_target.mutex);
    g_actuator_target.target_clutch_current_ma = current_u16;
    g_actuator_target.clutch_target_updated = 1;
    pthread_mutex_unlock(&g_actuator_target.mutex);
    
    /* 注意：target_current_a现在由update_feedforward_and_target统一更新
     * 以确保前馈电流和目标电流的一致性
     */

    return 0;
}

/******************************************************************************
 * 信号处理
 ******************************************************************************/
void signal_handler(int sig) {
    (void)sig;
    printf("\n[INFO] Signal received, shutting down...\n");
    g_running = 0;
    
    /* 紧急停止：立即停止电机和离合器，确保安全 */
    printf("[EMERGENCY STOP] Stopping motor and clutch immediately...\n");
    
    /* 立即停止电机 - 直接调用底层接口，不通过异步缓冲区 */
    if (g_motor.initialized) {
        motor_set_velocity(&g_motor, 0);  /* 设置速度为0 */
        motor_disable(&g_motor);           /* 失能电机 */
        printf("[EMERGENCY STOP] Motor stopped and disabled\n");
    }
    
    /* 立即停止离合器电流 */
    if (g_power.initialized && g_power.state == POWER_STATE_ON) {
        power_set_current(&g_power, 0);   /* 设置电流为0 */
        power_deinit(&g_power);            /* 关闭电源 */
        printf("[EMERGENCY STOP] Clutch current set to 0\n");
    }
    
    /* 确保保存编码器数据 */
    sensor_mgr_deinit(&g_sensor_mgr);
}

/******************************************************************************
 * 系统预检测函数
 ******************************************************************************/
static int check_sensors_impl(void) {
    SensorData_t encoder, pressure;
    
    printf("[CHECK] Checking sensors...\n");
    fflush(stdout);
    
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder) != ERR_OK) {
        printf("[CHECK FAIL] Encoder data not available\n");
        return -1;
    }
    if (!encoder.data_valid) {
        printf("[CHECK FAIL] Encoder data invalid\n");
        return -1;
    }
    
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure) != ERR_OK) {
        printf("[CHECK FAIL] Pressure sensor data not available\n");
        return -1;
    }
    if (!pressure.data_valid) {
        printf("[CHECK FAIL] Pressure sensor data invalid\n");
        return -1;
    }
    
    printf("[CHECK PASS] Sensors OK (Pressure: %.3f kg filtered, %.3f kg raw, Encoder: %.3f m)\n",
           pressure.data.pressure.pressure_filtered_kg,
           pressure.data.pressure.pressure_kg,
           encoder.data.encoder.rope_length_mm / 1000.0f);
    return 0;
}

static int check_motor_impl(void) {
    if (!g_motor_enabled) {
        printf("[CHECK SKIP] Motor disabled\n");
        return 0;
    }
    
    if (!g_motor.initialized) {
        printf("[CHECK FAIL] Motor not initialized\n");
        return -1;
    }
    
    printf("[CHECK] Checking motor communication... ");
    fflush(stdout);
    
    int retries = 3;
    int motor_ok = 0;
    while (retries-- > 0) {
        if (motor_update_state(&g_motor) == ERR_OK) {
            motor_ok = 1;
            break;
        }
        usleep(100000);
    }
    
    if (!motor_ok) {
        printf("FAILED\n");
        printf("[CHECK FAIL] Cannot communicate with motor after retries\n");
        return -1;
    }
    
    printf("OK\n");
    printf("[CHECK PASS] Motor OK (NodeID=%d)\n", g_motor.node_id);
    return 0;
}

static int check_power_impl(void) {
    uint16_t current, voltage;
    
    if (power_get_status(&g_power, &current, &voltage) != ERR_OK) {
        /* 测试模式：允许在电源板未就绪时继续运行 */
        if (TEST_MODE_ALLOW_MOTOR_WITHOUT_POWER) {
            printf("[CHECK WARN] Power board not available, running in test mode\n");
            return 0;
        }
        printf("[CHECK FAIL] Cannot communicate with power board\n");
        return -1;
    }
    
    printf("[CHECK PASS] Power board OK (%.2f V, %.3f A)\n",
           voltage / 1000.0f, current / 1000.0f);
    return 0;
}

static int check_safety_impl(void) {
    SensorData_t pressure;
    
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure) != ERR_OK) {
        return -1;
    }
    
    /* 安全检查使用原始压力值，确保能检测到真实的异常 */
    if (pressure.data.pressure.pressure_kg < SAFETY_PRESSURE_MIN_KG ||
        pressure.data.pressure.pressure_kg > SAFETY_PRESSURE_MAX_KG) {
        printf("[CHECK FAIL] Pressure out of safety range: %.3f kg (raw: %.3f kg)\n",
               pressure.data.pressure.pressure_filtered_kg, pressure.data.pressure.pressure_kg);
        return -1;
    }
    
    printf("[CHECK PASS] Safety check OK\n");
    return 0;
}

/******************************************************************************
 * 共享内存数据线程 - 40Hz输出
 ******************************************************************************/
static void* data_output_thread(void* arg) {
    (void)arg;
    
    /* 设置线程高优先级，确保40Hz输出稳定 */
    struct sched_param param;
    param.sched_priority = 90;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        printf("[SHM] Warning: Failed to set high priority for data output thread\n");
    }
    
    printf("[SHM] Data output thread started (%dHz)\n", 1000 / SHM_DATA_OUTPUT_PERIOD_MS);
    
    SharedData_t data;
    const long period_ms = SHM_DATA_OUTPUT_PERIOD_MS;
    
    while (g_running) {
        uint32_t start_time = get_timestamp_ms();
        
        /* 清零数据 */
        memset(&data, 0, sizeof(SharedData_t));
        
        /* 工业级优化：从共享状态缓冲区读取数据 */
        pthread_mutex_lock(&g_shared_state.mutex);
        
        /* 传感器数据 - 从共享缓冲区读取 */
        data.pressure_kg = g_shared_state.pressure_kg;
        data.rope_length_m = g_shared_state.rope_length_m;
        data.encoder_value = g_shared_state.encoder_value;
        data.encoder_angle_deg = g_shared_state.encoder_angle_deg;
        
        /* 重量采集模块数据 - 从共享缓冲区读取 (UART/TTL, 100Hz) */
        data.weight_raw_kg = g_shared_state.weight_raw_kg;
        data.weight_filtered_kg = g_shared_state.weight_filtered_kg;
        
        /* 电源数据 - 从共享缓冲区读取 */
        data.current_a = g_shared_state.current_a;
        data.voltage_v = g_shared_state.voltage_v;
        
        /* 电机数据 - 从共享缓冲区读取 */
        data.motor_speed_rpm = g_shared_state.motor_speed_rpm;
        data.motor_position = g_shared_state.motor_position_m;
        data.motor_status = g_shared_state.motor_status;
        
        pthread_mutex_unlock(&g_shared_state.mutex);
        
        /* 算法状态 */
        if (g_algorithm_enabled) {
            AlgoStatus_t status;
            gravity_unload_get_status(&g_gravity_ctrl, &status);
            data.algorithm_state = (int32_t)status.state;
            data.algorithm_error = (int32_t)status.error;
            data.clutch_current_ma = 0;
            data.motor_cmd_rpm = 0;
        } else {
            data.algorithm_state = 0;
            data.algorithm_error = 0;
            data.clutch_current_ma = 0;
            data.motor_cmd_rpm = 0;
        }
        
        data.emergency_stop = false;
        
        /* 写入共享内存 */
        if (g_shm_initialized) {
            shm_write_data(&g_shm_mgr, &data);
        }
        
        /* 同时打印到控制台（用于调试） */
        static int console_counter = 0;
        if (++console_counter >= 20) { /* 1Hz控制台输出 */
            console_counter = 0;
            /* 计算线速度 */
            float linear_vel = 0.0f;
            if (g_motor.initialized) {
                linear_vel = motor_calculate_linear_velocity(&g_motor, data.motor_speed_rpm);
            }
            printf("[DATA] P=%.3fkg Pos=%.3fm I=%.3fA V=%.2fV Motor=%.2frpm Vline=%.3fm/s Algo=%s\n",
                   data.pressure_kg,
                   data.rope_length_m,
                   data.current_a,
                   data.voltage_v,
                   data.motor_speed_rpm,
                   linear_vel,
                   g_algorithm_enabled ? "RUN" : "STOP");
            fflush(stdout);
        }
        
        /* 计算剩余延迟时间 */
        uint32_t elapsed_ms = get_timestamp_ms() - start_time;
        if (elapsed_ms < period_ms) {
            /* 使用usleep进行相对延迟（可被信号中断） */
            usleep((period_ms - elapsed_ms) * 1000);
        }
    }
    
    printf("[SHM] Data output thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 命令处理线程 - 实时响应上位机命令
 ******************************************************************************/
static void* command_handler_thread(void* arg) {
    (void)arg;
    
    printf("[SHM] Command handler thread started\n");
    
    SharedCommand_t cmd;
    
    while (g_running) {
        if (g_shm_initialized) {
            /* 读取命令 */
            if (shm_read_command(&g_shm_mgr, &cmd) == 0) {
                /* 检查是否有新命令 */
                if (cmd.command_id != g_last_cmd_id && cmd.cmd_type != 0) {
                    g_last_cmd_id = cmd.command_id;
                    
                    printf("[CMD] Received command id=%u type=%d value=%.2f\n",
                           cmd.command_id, cmd.cmd_type, cmd.cmd_value);
                    
                    /* 处理命令 */
                    switch (cmd.cmd_type) {
                        case 1: /* 速度命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                /* 确保电机处于循环同步速度模式(CSV) */
                                motor_set_mode(&g_motor, MOTOR_MODE_CSV);
                                /* 发送速度命令（单位：rpm），支持浮点数 */
                                float speed = cmd.cmd_value;
                                ErrorCode_t ret = motor_set_velocity(&g_motor, speed);
                                printf("[CMD] Set motor velocity: %.2f rpm (%s)\n", speed, 
                                       ret == ERR_OK ? "OK" : "FAILED");
                            } else {
                                printf("[CMD] Motor velocity command rejected (enabled=%d, initialized=%d, algo=%d)\n",
                                       g_motor_enabled, g_motor.initialized, g_algorithm_enabled);
                            }
                            break;
                            
                        case 2: /* 位置命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                /* 确保电机处于位置模式 */
                                motor_set_mode(&g_motor, MOTOR_MODE_PP);
                                motor_set_position(&g_motor, (int32_t)cmd.cmd_value);
                                printf("[CMD] Set motor position: %.0f\n", cmd.cmd_value);
                            }
                            break;
                            
                        case 3: /* 停止命令 */
                            if (g_motor_enabled && g_motor.initialized) {
                                motor_set_velocity(&g_motor, 0);
                                printf("[CMD] Motor stop\n");
                            }
                            break;
                            
                        case 4: /* 使能命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                /* 设置为循环同步速度模式(CSV)后再使能 */
                                motor_set_mode(&g_motor, MOTOR_MODE_CSV);
                                ErrorCode_t ret = motor_enable(&g_motor);
                                uint16_t kp, ki, kff;

                                // 先读取当前值
                                //motor_get_velocity_pid(&g_motor, &kp, &ki, &kff);
                                //printf("Current PID: Kp=%d, Ki=%d, Kff=%d\n", kp, ki, kff);

                                // 设置新值（示例：降低P增益）
                                //motor_set_velocity_pid(&g_motor, 500, 800, 1000);
                                //printf("[CMD] Motor enable (mode: CSV) - %s\n", 
                                //       ret == ERR_OK ? "OK" : "FAILED");
                            }
                            break;
                            
                        case 5: /* 失能命令 */
                            if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                                motor_disable(&g_motor);
                                printf("[CMD] Motor disable\n");
                            }
                            break;
                            
                        case 6: /* 设置电流命令 (A) */
                            {
                                uint16_t current_ma = (uint16_t)(cmd.cmd_value * 1000.0f);
                                ErrorCode_t ret = power_set_current(&g_power, current_ma);
                                if (ret == ERR_OK) {
                                    printf("[CMD] Power current set to %.2f A (%d mA)\n", 
                                           cmd.cmd_value, current_ma);
                                } else {
                                    printf("[CMD] Failed to set power current\n");
                                }
                            }
                            break;
                            
                        case 7: /* 设置电压命令 (V) */
                            {
                                printf("[CMD] Power voltage set to %.2f V (voltage control not implemented)\n", 
                                       cmd.cmd_value);
                                // 电压控制需要根据电源板硬件能力实现
                            }
                            break;
                            
                        default:
                            printf("[CMD] Unknown command type: %d\n", cmd.cmd_type);
                            break;
                    }
                    
                    /* 清空命令 */
                    shm_clear_command(&g_shm_mgr);
                }
                
                /* 处理算法控制命令 */
                if (cmd.algorithm_start && !g_algorithm_enabled) {
                    printf("[CMD] Starting algorithm...\n");
                    /* 算法启动由主循环处理 */
                }
                
                if (cmd.algorithm_stop && g_algorithm_enabled) {
                    printf("[CMD] Stopping algorithm...\n");
                    /* 算法停止由主循环处理 */
                }
                
                /* 处理数据记录命令 */
                if (cmd.data_log_start && !g_logging_enabled) {
                    printf("[CMD] Starting data logging...\n");
                    g_logging_enabled = 1;
                    /* 重置手动F0计算，以便新采集重新计算 */
                    g_manual_f0_sample_count = 0;
                    g_manual_f0_sum = 0.0f;
                    g_manual_f0_kg = 0.0f;
                    start_logging();
                }

                if (cmd.data_log_stop && g_logging_enabled) {
                    printf("[CMD] Stopping data logging...\n");
                    g_logging_enabled = 0;
                    stop_logging();
                }
            }
        }
        
        /* 10ms轮询 */
        usleep(10000);
    }
    
    printf("[SHM] Command handler thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 监控线程 - 1Hz打印状态
 ******************************************************************************/
static void* monitor_thread(void* arg) {
    (void)arg;
    
    printf("[MONITOR] Monitor thread started (1Hz)\n");
    
    while (g_running) {
        /* 打印重力卸载控制器状态 */
        if (g_algorithm_enabled) {
            gravity_unload_print_status(&g_gravity_ctrl);
        }
        
        /* 使用多个usleep调用，提高退出响应速度 */
        for (int i = 0; i < 10 && g_running; i++) {
            usleep(100000); /* 100ms */
        }
    }
    
    printf("[MONITOR] Monitor thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 数据收集线程 - 100Hz更新共享缓冲区
 ******************************************************************************/
/* 日志记录相关全局变量 */
static FILE *g_log_file = NULL;
static uint32_t g_log_count = 0;
static char g_log_filename[128];

/* 开启日志记录 */
void start_logging(void) {
    if (g_log_file != NULL) return;

    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    snprintf(g_log_filename, sizeof(g_log_filename),
             "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/logdata/gravity_data_%04d%02d%02d_%02d%02d%02d.csv",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);

    mkdir("/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/logdata", 0755);

    g_log_file = fopen(g_log_filename, "w");
    if (g_log_file != NULL) {
        fprintf(g_log_file, "%-20s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-14s,%-14s,%-16s,%-14s,%-20s,%-20s,%-16s,%-18s,%-22s,%-14s,%-14s\n",
                "Time", "Current(A)", "TargetCurrent(A)", "Voltage(V)", "PressureRaw(kg)", "PressureFiltered(kg)", "F0(kg)", "DeltaF",
                "PI_P(mA)", "PI_I(mA)", "PI_D(mA)", "PI_LastCurrent(mA)", "Feedforward(mA)", "AlgoDeltaF(kg)", "AlgoPressure(kg)",
                "RopeLength(m)", "EncoderValue", "EncoderAngle(deg)", "MotorSpeed(rpm)",
                "MotorLinearVel(m/s)", "MotorTheoryVel(m/s)", "MotorPosition(m)",
                "RopeVelocityRaw(m/s)", "RopeVelocityFiltered(m/s)", "WeightRaw(kg)", "WeightFiltered(kg)");
        printf("[LOG] Started logging to: %s\n", g_log_filename);
        g_log_count = 0;

        /* 开始记录时重置last_current_mA为0，确保从0开始累积 */
        pthread_mutex_lock(&g_gravity_ctrl.mutex);
        g_gravity_ctrl.last_current_mA = 0.0f;
        pthread_mutex_unlock(&g_gravity_ctrl.mutex);
        printf("[LOG] Reset last_current_mA to 0 for clean recording start\n");
    } else {
        printf("[ERROR] Failed to create log file: %s\n", g_log_filename);
    }
}

/* 停止日志记录 */
void stop_logging(void) {
    if (g_log_file == NULL) return;
    
    fclose(g_log_file);
    g_log_file = NULL;
    printf("[LOG] Stopped logging, total records: %u\n", g_log_count);
}

static void* data_collection_thread(void* arg) {
    (void)arg;
    
    /* 设置线程高优先级（低于传感器线程88），确保100Hz采集稳定 */
    struct sched_param param;
    param.sched_priority = 87;  /* 高于普通线程，低于传感器线程(88) */
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        printf("[DATA] Warning: Failed to set high priority for data collection thread\n");
    }
    
    printf("[DATA] Data collection thread started (%dHz) - Industrial Grade Strict Timing\n", 1000 / SHM_DATA_COLLECTION_PERIOD_MS);
    
    const long period_ms = SHM_DATA_COLLECTION_PERIOD_MS;
    int log_counter = 0;  /* 用于实现50Hz记录（每2个100Hz周期记录一次） */
    
    /* 工业级严格周期控制 - 使用绝对时间戳 */
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    uint64_t next_time_us = next_time.tv_sec * 1000000ULL + next_time.tv_nsec / 1000;
    
    while (g_running) {
        uint32_t cycle_start_time = get_timestamp_ms();
        
        /* ========== 第一步：读取所有传感器数据（原子性） ========== */
        SensorData_t encoder_data, pressure_data;
        float pressure_kg = 0.0f, rope_length_m = 0.0f;
        uint32_t encoder_value = 0;
        float encoder_angle = 0.0f;
        
        /* 同时读取编码器和压力传感器，确保时间一致性 */
        int encoder_ok = (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) == ERR_OK);
        int pressure_ok = (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure_data) == ERR_OK);
        
        if (encoder_ok) {
            encoder_value = encoder_data.data.encoder.multi_turn_value;
            encoder_angle = encoder_data.data.encoder.angle_deg;
            rope_length_m = encoder_data.data.encoder.rope_length_mm / 1000.0f;
        }
        
        float pressure_raw_kg = 0.0f;  /* 原始压力值（未滤波） */
        if (pressure_ok) {
            pressure_raw_kg = pressure_data.data.pressure.pressure_kg;           /* 原始值 */
            pressure_kg = pressure_data.data.pressure.pressure_filtered_kg;      /* 滤波后值 */
        }
        
        /* ========== 第二步：更新数据到共享缓冲区 ========== */
        if (encoder_ok) {
            update_sensor_to_buffer(&encoder_data, NULL);
        }
        if (pressure_ok) {
            update_sensor_to_buffer(NULL, &pressure_data);
        }
        update_power_to_buffer(&g_power);
        if (g_motor.initialized) {
            update_motor_to_buffer();
        }
        
        /* ========== 第三步：读取重量采集数据（100Hz同步） ========== */
        /* 优化：只读取一次滤波后的重量数据，减少通信时间 */
        float weight_filtered_kg = 0.0f;
        if (g_weight.initialized) {
            weight_get_weight(&g_weight, &weight_filtered_kg);
        }
        
        /* 更新重量数据到共享缓冲区 */
        pthread_mutex_lock(&g_shared_state.mutex);
        g_shared_state.weight_raw_kg = weight_filtered_kg;  /* 临时使用滤波值作为原始值 */
        g_shared_state.weight_filtered_kg = weight_filtered_kg;
        pthread_mutex_unlock(&g_shared_state.mutex);
        
        /* ========== 第四步：原子性读取所有数据（关键！） ========== */
        float current_a = 0.0f, voltage_v = 0.0f;
        float motor_speed_rpm = 0.0f, motor_pos_m = 0.0f, motor_linear_vel = 0.0f;
        float motor_theory_vel = 0.0f;
        float rope_vel_raw = 0.0f, rope_vel_filtered = 0.0f;
        float pressure_f0_kg = 0.0f, pressure_deltaf = 0.0f;
        float pi_p_term_mA = 0.0f, pi_i_term_mA = 0.0f, pi_d_term_mA = 0.0f;
        float pi_last_current_mA = 0.0f;
        float target_current_a = 0.0f;
        float feedforward_current_mA = 0.0f;
        float algo_deltaf_kg = 0.0f;
        float algo_pressure_kg = 0.0f;
        
        /* 使用单个锁，原子性读取所有数据，确保一致性 */
        pthread_mutex_lock(&g_shared_state.mutex);
        current_a = g_shared_state.current_a;
        voltage_v = g_shared_state.voltage_v;
        motor_speed_rpm = g_shared_state.motor_speed_rpm;
        motor_pos_m = g_shared_state.motor_position_m;
        motor_linear_vel = g_shared_state.motor_linear_velocity_m_s;
        motor_theory_vel = g_shared_state.motor_theory_linear_velocity_m_s;
        rope_vel_raw = g_shared_state.rope_velocity_raw_m_s;
        rope_vel_filtered = g_shared_state.rope_velocity_filtered_m_s;
        pressure_f0_kg = g_shared_state.pressure_f0_kg;
        pi_p_term_mA = g_shared_state.pi_p_term_mA;
        pi_i_term_mA = g_shared_state.pi_i_term_mA;
        pi_d_term_mA = g_shared_state.pi_d_term_mA;
        pi_last_current_mA = g_shared_state.pi_last_current_mA;
        target_current_a = g_shared_state.target_current_a;
        feedforward_current_mA = g_shared_state.feedforward_current_mA;
        algo_deltaf_kg = g_shared_state.algo_deltaf_kg;
        algo_pressure_kg = g_shared_state.algo_pressure_kg;
        pthread_mutex_unlock(&g_shared_state.mutex);

        /* 使用算法提供的DeltaF作为数据源，确保与AlgoDeltaF一致 */
        /* 如果算法未运行（pressure_f0_kg为0），使用手动模式计算DeltaF */
        if (pressure_f0_kg != 0.0f) {
            /* 算法运行中，使用算法提供的DeltaF */
            pressure_deltaf = algo_deltaf_kg;
        } else {
            /* 手动模式：采集前50个点的平均压力作为F0，然后计算DeltaF */
            if (g_manual_f0_sample_count < MANUAL_F0_SAMPLE_COUNT) {
                g_manual_f0_sum += pressure_kg;
                g_manual_f0_sample_count++;
                g_manual_f0_kg = g_manual_f0_sum / g_manual_f0_sample_count;
            }
            pressure_deltaf = pressure_kg - g_manual_f0_kg;
        }

        /* 100Hz数据记录（集成到采集线程，避免锁竞争） */
        if (g_log_file != NULL && g_logging_enabled) {
            struct timeval tv;
            struct tm *t;
            gettimeofday(&tv, NULL);
            t = localtime(&tv.tv_sec);

            char time_str[32];
            snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d.%06ld",
                     t->tm_hour, t->tm_min, t->tm_sec, (long)tv.tv_usec);

            /* 在手动模式下，记录手动计算的F0 */
            float log_f0_kg = (pressure_f0_kg != 0.0f) ? pressure_f0_kg : g_manual_f0_kg;
            /* 从共享状态读取重量数据 */
            float log_weight_raw_kg = 0.0f, log_weight_filtered_kg = 0.0f;
            pthread_mutex_lock(&g_shared_state.mutex);
            log_weight_raw_kg = g_shared_state.weight_raw_kg;
            log_weight_filtered_kg = g_shared_state.weight_filtered_kg;
            pthread_mutex_unlock(&g_shared_state.mutex);
            
            fprintf(g_log_file, "%-20s,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-14.5f,%-14u,%-16.3f,%-14.3f,%-20.3f,%-20.3f,%-16.3f,%-18.5f,%-22.5f,%-14.3f,%-14.3f\n",
                    time_str,
                    current_a, target_current_a, voltage_v, pressure_raw_kg, pressure_kg, log_f0_kg, pressure_deltaf,
                    pi_p_term_mA, pi_i_term_mA, pi_d_term_mA, pi_last_current_mA, feedforward_current_mA, algo_deltaf_kg, algo_pressure_kg, rope_length_m,
                    encoder_value, encoder_angle, motor_speed_rpm,
                    motor_linear_vel, motor_theory_vel, motor_pos_m,
                    rope_vel_raw, rope_vel_filtered, log_weight_raw_kg, log_weight_filtered_kg);
            
            g_log_count++;
            
            /* 每100条记录刷新一次 */
            if (g_log_count % 100 == 0) {
                fflush(g_log_file);
            }
        }
        log_counter++;
        
        /* 工业级严格周期控制 - 使用绝对时间戳 */
        next_time_us += period_ms * 1000;  /* 下一个周期的时间点 */
        
        uint64_t current_time_us = get_time_us();
        int64_t sleep_us = (int64_t)(next_time_us - current_time_us);
        
        if (sleep_us > 0) {
            /* 正常情况：等待到下一个周期时间点 */
            struct timespec sleep_ts;
            sleep_ts.tv_sec = sleep_us / 1000000;
            sleep_ts.tv_nsec = (sleep_us % 1000000) * 1000;
            nanosleep(&sleep_ts, NULL);
        } else if (sleep_us < -5000) {
            /* 严重超时（超过5ms）：打印警告并重新同步 */
            printf("[DATA] WARNING: Cycle deadline missed by %ld us, resynchronizing...\n", 
                   (long)(-sleep_us));
            /* 重新同步到下一个周期 */
            next_time_us = current_time_us + period_ms * 1000;
        }
        /* 如果sleep_us在[-5000, 0]之间，说明轻微超时，继续执行不等待 */
    }
    
    /* 线程退出时关闭日志文件 */
    if (g_log_file != NULL) {
        fclose(g_log_file);
        g_log_file = NULL;
    }
    
    printf("[DATA] Data collection thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 主函数
 ******************************************************************************/
int main(int argc, char *argv[]) {
    /* 关键修复：切换到SDK目录运行，确保SDK能找到所有依赖文件 */
    const char* sdk_path = "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/lib";
    if (chdir(sdk_path) != 0) {
        printf("WARNING: Failed to change directory to SDK path: %s\n", sdk_path);
    } else {
        printf("Running from SDK directory: %s\n", sdk_path);
    }
    
    /* 设置LD_LIBRARY_PATH */
    setenv("LD_LIBRARY_PATH", sdk_path, 1);
    
    /* 解析命令行参数 */
    if (argc > 1 && (strcmp(argv[1], "--no-motor") == 0 || strcmp(argv[1], "-n") == 0)) {
        g_motor_enabled = 0;
        printf("Motor control disabled\n");
    } else {
        printf("Motor control enabled (use -n or --no-motor to disable)\n");
    }
    fflush(stdout);
    
    /* 注册信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* 初始化日志 */
    logger_init(&g_logger, NULL, LOG_LEVEL_DEBUG, 1);
    
    printf("========================================\n");
    printf("CANOpNode_Sys v4.0.0 (Gravity Unload)\n");
    printf("NiMotion SDK Integration - Industrial Grade\n");
    printf("Shared Memory Communication Enabled\n");
    printf("========================================\n\n");
    fflush(stdout);
    
    /* 初始化共享状态缓冲区（必须在传感器管理器之前） */
    shared_state_init();

    /* 初始化异步执行器控制 */
    actuator_target_init();
    
    /* ========== 阶段1: 初始化硬件 ========== */
    printf("[INIT] Phase 1: Initializing hardware...\n");
    
    /* 1. 初始化统一传感器管理器 */
    printf("  -> Sensor manager... ");
    fflush(stdout);
    if (sensor_mgr_init(&g_sensor_mgr, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE) != ERR_OK) {
        printf("WARNING (sensor disabled)\n");
        printf("     ! Sensors will not be available\n");
    } else {
        printf("OK\n");
    }
    
    sensor_mgr_set_encoder_rope_params(&g_sensor_mgr, 100.0f, 4096);
    
    /* 2. 初始化电源板 */
    printf("  -> Power driver... ");
    fflush(stdout);
    if (power_init(&g_power, POWER_UART_DEVICE, POWER_UART_BAUDRATE) != ERR_OK) {
        printf("WARNING (power disabled)\n");
        printf("     ! Power monitoring will not be available\n");
    } else {
        printf("OK\n");
    }
    
    /* 3. 初始化重量采集模块（新增 - UART/TTL, 100Hz） */
    printf("  -> Weight driver... ");
    fflush(stdout);
    if (weight_init(&g_weight, WEIGHT_UART_DEVICE, WEIGHT_UART_BAUDRATE) != ERR_OK) {
        printf("WARNING (weight disabled)\n");
        printf("     ! Weight monitoring will not be available\n");
    } else {
        printf("OK\n");
    }
    
    /* 4. 初始化电机 */
    if (g_motor_enabled) {
        printf("  -> Motor driver... ");
        fflush(stdout);
        if (motor_init(&g_motor, MOTOR_NODE_ID, MOTOR_CAN_INTERFACE) != ERR_OK) {
            printf("WARNING (non-critical)\n");
            g_motor_enabled = 0;
        } else {
            printf("OK\n");
            printf("  -> Motor ready (will enable when needed)\n");
        }
    }
    
    /* 4. 启动传感器管理线程 */
    printf("  -> Starting sensor manager thread... ");
    fflush(stdout);
    if (sensor_mgr_start(&g_sensor_mgr) != ERR_OK) {
        printf("WARNING (sensor thread disabled)\n");
    } else {
        printf("OK\n");
        
        /* 等待传感器线程完成首次读取 */
        printf("  -> Waiting for sensors to stabilize... ");
        fflush(stdout);
        sleep(2);
        
        /* 等待编码器数据有效 */
        SensorData_t encoder_data;
        int wait_count = 0;
        while (wait_count < 50) {  /* 最多等待5秒 */
            if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) == ERR_OK 
                && encoder_data.data_valid) {
                break;
            }
            usleep(100000);
            wait_count++;
        }
        
        if (!encoder_data.data_valid) {
            printf("WARNING (sensor data invalid)\n");
        } else {
            printf("OK\n");
            
            /* 5. 执行校准（必须在传感器线程启动并数据有效后） */
            printf("  -> Calibrating sensors... ");
            fflush(stdout);
            sensor_mgr_encoder_zero_calibration(&g_sensor_mgr);
            usleep(500000);
            sensor_mgr_pressure_tare(&g_sensor_mgr);
            printf("OK\n");
        }
    }
    
    printf("\n");
    
    /* ========== 阶段2: 初始化共享内存 ========== */
    printf("[INIT] Phase 2: Initializing shared memory...\n");
    printf("  -> Creating shared memory segment... ");
    fflush(stdout);
    
    if (shm_init(&g_shm_mgr, true) != 0) {
        printf("FAILED\n");
        printf("[WARNING] Shared memory init failed, continuing without it\n");
    } else {
        g_shm_initialized = 1;
        printf("OK (name=%s, size=%d)\n", SHM_NAME, SHM_SIZE);
    }
    
    /* ========== 阶段3: 系统预检测 ========== */
    printf("\n[INIT] Phase 3: System health check...\n");
    
    int check_pass = 1;
    
    if (check_sensors_impl() != 0) check_pass = 0;
    if (check_motor_impl() != 0) check_pass = 0;
    if (check_power_impl() != 0) check_pass = 0;
    if (check_safety_impl() != 0) check_pass = 0;
    
    if (!check_pass) {
        printf("\n[ERROR] System check FAILED! Please check hardware connections.\n");
        sensor_mgr_stop(&g_sensor_mgr);
        power_deinit(&g_power);
        weight_deinit(&g_weight);  /* 新增重量采集驱动反初始化 */
        sensor_mgr_deinit(&g_sensor_mgr);
        if (g_shm_initialized) shm_close(&g_shm_mgr);
        return 1;
    }
    
    printf("\n[INIT] All system checks PASSED!\n\n");
    
    /* ========== 阶段4: 启动数据通信线程 ========== */
    printf("[INIT] Phase 4: Starting communication threads...\n");
    
    printf("  -> Starting data collection thread (%dHz)... ", 1000 / SHM_DATA_COLLECTION_PERIOD_MS);
    fflush(stdout);
    pthread_create(&g_collection_thread_tid, NULL, data_collection_thread, NULL);
    printf("OK\n");
    
    printf("  -> Starting motor state update thread... ");
    fflush(stdout);
    pthread_create(&g_motor_state_thread_tid, NULL, motor_state_update_thread, NULL);
    printf("OK\n");
    
    printf("  -> Starting power state update thread... ");
    fflush(stdout);
    pthread_create(&g_power_state_thread_tid, NULL, power_state_update_thread, NULL);
    printf("OK\n");

    printf("  -> Starting asynchronous actuator control thread (100Hz)... ");
    fflush(stdout);
    pthread_create(&g_actuator_thread_tid, NULL, actuator_control_thread, NULL);
    printf("OK\n");

    printf("  -> Starting data output thread (%dHz)... ", 1000 / SHM_DATA_OUTPUT_PERIOD_MS);
    fflush(stdout);
    pthread_create(&g_data_thread_tid, NULL, data_output_thread, NULL);
    printf("OK\n");
    
    printf("  -> Starting command handler thread... ");
    fflush(stdout);
    pthread_create(&g_command_thread_tid, NULL, command_handler_thread, NULL);
    printf("OK\n");
    
    /* ========== 阶段5: 用户确认 ========== */
    printf("\n[INIT] Phase 5: System ready for remote monitoring\n");
    printf("  - Shared Memory: %s\n", g_shm_initialized ? "ACTIVE" : "OFFLINE");
    printf("  - Data Output: %dHz\n", 1000 / SHM_DATA_OUTPUT_PERIOD_MS);
    printf("  - Data Collection: %dHz\n", 1000 / SHM_DATA_COLLECTION_PERIOD_MS);
    printf("  - Data Logging: 50Hz (on demand)\n");
    printf("  - Motor Control: Available (manual mode)\n");
    printf("  - Algorithm: Standby\n\n");
    
    printf("Waiting for commands from GravShow...\n");
    printf("Or press Enter to start algorithm mode locally\n");
    printf("Press Ctrl+C to exit\n\n");
    
    /* 非阻塞检查用户输入 */
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; /* 100ms */
    
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    
    int user_input = 0;
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            user_input = 1;
        }
    }
    
    /* ========== 阶段6: 算法模式（可选） ========== */
    if (user_input) {
        printf("\n[INFO] Starting algorithm mode...\n\n");
        g_algorithm_mode = 1;
        
        /* 先使能电机 */
        if (g_motor_enabled) {
            printf("  -> Enabling motor... ");
            fflush(stdout);
            if (motor_enable(&g_motor) != ERR_OK) {
                printf("WARNING (non-critical)\n");
                g_motor_enabled = 0;
            } else {
                printf("OK\n");
            }
        }
        
        /* 初始化算法 */
        if (gravity_unload_init(&g_gravity_ctrl) != 0) {
            printf("[ERROR] Failed to initialize algorithm\n");
            g_running = 0;
        } else {
            /* 启动算法 */
            if (gravity_unload_start(&g_gravity_ctrl) != 0) {
                printf("[ERROR] Failed to start algorithm\n");
                g_running = 0;
            } else {
                g_algorithm_enabled = 1;
                printf("[INIT] Algorithm started successfully\n");
                
                /* 创建监控线程 */
                pthread_create(&g_monitor_thread_tid, NULL, monitor_thread, NULL);
            }
        }
    } else {
        printf("[INFO] Running in manual mode (remote control enabled)\n\n");
    }
    
    printf("========================================\n");
    printf("System RUNNING\n");
    if (g_algorithm_enabled) {
        printf("Mode: ALGORITHM (Gravity Unload)\n");
    } else {
        printf("Mode: MANUAL (Remote Control)\n");
    }
    printf("Press Ctrl+C to stop\n");
    printf("========================================\n\n");
    fflush(stdout);
    
    /* ========== 主循环 ========== */
    while (g_running) {
        /* 算法模式：检查算法状态 */
        if (g_algorithm_enabled) {
            AlgoStatus_t status;
            gravity_unload_get_status(&g_gravity_ctrl, &status);
            
            if (status.state == ALGO_STATE_EMERGENCY_STOP) {
                printf("\n[EMERGENCY] Algorithm stopped due to safety violation!\n");
                break;
            }
            
            if (status.error != ALGO_ERR_NONE && status.error != ALGO_ERR_INVALID_PARAM) {
                printf("\n[WARNING] Algorithm error: %d\n", status.error);
            }
        }
        
        /* 检查共享内存命令中的算法控制 */
        if (g_shm_initialized) {
            SharedCommand_t cmd;
            if (shm_read_command(&g_shm_mgr, &cmd) == 0) {
                if (cmd.algorithm_start && !g_algorithm_enabled) {
                    printf("\n[CMD] Starting algorithm from remote...\n");
                    /* 这里可以添加算法启动逻辑 */
                }
                if (cmd.algorithm_stop && g_algorithm_enabled) {
                    printf("\n[CMD] Stopping algorithm from remote...\n");
                    g_running = 0;
                }
            }
        }
        
        sleep(1);
    }
    
    /* ========== 清理 ========== */
    printf("\n[SHUTDOWN] Stopping system...\n");
    
    /* 停止算法 */
    if (g_algorithm_enabled) {
        printf("  -> Stopping algorithm... ");
        fflush(stdout);
        gravity_unload_stop(&g_gravity_ctrl);
        gravity_unload_deinit(&g_gravity_ctrl);
        printf("OK\n");
        
        /* 等待监控线程结束 */
        pthread_join(g_monitor_thread_tid, NULL);
    }
    
    /* 等待通信线程结束 */
    printf("  -> Waiting for communication threads... ");
    fflush(stdout);
    pthread_join(g_collection_thread_tid, NULL);
    pthread_join(g_motor_state_thread_tid, NULL);
    pthread_join(g_power_state_thread_tid, NULL);
    pthread_join(g_data_thread_tid, NULL);
    pthread_join(g_command_thread_tid, NULL);
    printf("OK\n");
    
    /* 停止传感器管理器 */
    printf("  -> Stopping sensor manager... ");
    fflush(stdout);
    sensor_mgr_stop(&g_sensor_mgr);
    printf("OK\n");
    
    /* 打印最终状态 */
    printf("\n");
    sensor_mgr_print_status(&g_sensor_mgr);
    
    /* 反初始化设备 */
    power_deinit(&g_power);
    weight_deinit(&g_weight);  /* 新增重量采集驱动反初始化 */
    sensor_mgr_deinit(&g_sensor_mgr);
    
    /* 关闭共享内存 */
    if (g_shm_initialized) {
        printf("  -> Closing shared memory... ");
        fflush(stdout);
        shm_close(&g_shm_mgr);
        printf("OK\n");
    }
    
    printf("\n[System shutdown completed]\n");
    logger_deinit(NULL);
    
    return 0;
}
