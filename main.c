/******************************************************************************
 * @file    main.c
 * @brief   CANOpNode_Sys 主程序 - 重力卸载控制系统 v4.0
 * @author  System Architect
 * @date    2026-06-12
 * @version 4.0.0
 * 
 * @description
 * 重力卸载控制系统主程序
 * - 工业级100Hz实时控制（合并线程，直接同步调用电源批量读写）
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
#include <math.h>
#include <errno.h>

#include "config/system_config.h"
#include "utils/logger.h"
#include "include/shared_memory.h"
#include "drivers/sensor_manager.h"
#include "drivers/power_driver.h"
#include "drivers/motor_driver.h"
#include "drivers/weight_driver.h"
#include "algorithms/gravity_unload.h"
#include "algorithms/system_check.h"
#include "../NimServoSDK-MM-bin-linux-x64/inc/nimservosdk.h"

/******************************************************************************
 * 全局变量
 ******************************************************************************/
static volatile int g_running = 1;
static volatile int g_algorithm_enabled = 0;
static volatile int g_shm_initialized = 0;
static volatile uint32_t g_last_cmd_id = 0;

static SensorManager_t g_sensor_mgr;
PowerDriver_t g_power;
static MotorDriver_t g_motor;
static WeightDriver_t g_weight;
static GravityUnloadController_t g_gravity_ctrl;
static ShmManager_t g_shm_mgr;

static int g_motor_enabled = 1;
static int g_algorithm_mode = 0;
static volatile int g_logging_enabled = 0;

/* 手动模式下的F0计算 - 使用滑动窗口平均 */
#define MANUAL_F0_SAMPLE_COUNT  50
static float g_manual_f0_kg = 0.0f;
static int g_manual_f0_sample_count = 0;

/* 正弦测试全局变量 */
static volatile int g_sine_test_enabled = 0;
static float g_sine_amplitude = 0.3f;   // 默认振幅 0.3 Nm
static float g_sine_frequency = 0.5f;   // 默认频率 0.5 Hz
static float g_sine_offset = 0.0f;      // 默认偏置 0 Nm
static double g_sine_start_time = 0.0;  // 开始时间
static uint64_t g_sine_cycle_count = 0; // 周期计数
static float g_sine_last_target = 0.0f; // 上一个目标值（用于响应时间测量）
static float g_sine_response_time_ms = 0.0f; // 测得的响应时间
static float g_manual_f0_sum = 0.0f;

/* 线程ID */
static pthread_t g_data_thread_tid;
static pthread_t g_command_thread_tid;
static pthread_t g_monitor_thread_tid;
static pthread_t g_collection_thread_tid;
static pthread_t g_motor_state_thread_tid;

/* 全局变量用于PDO线程与算法线程同步 */
static volatile uint32_t g_algo_pdo_cycle_count = 0;
static pthread_mutex_t g_pdo_sync_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_pdo_send_thread_tid;

/* 全局变量用于电机状态更新线程同步 */
static volatile uint32_t g_algo_motor_cycle_count = 0;
static pthread_mutex_t g_motor_sync_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 全局变量用于数据收集线程同步 */
static volatile uint32_t g_algo_data_cycle_count = 0;
static volatile int g_algo_data_cycle_updated = 0;  /* 标志：表示新周期数据已更新 */
static pthread_mutex_t g_data_sync_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_data_sync_cond = PTHREAD_COND_INITIALIZER;
/* 注意：actuator_control_thread 和 power_state_update_thread 已被删除 */

/* 函数前置声明 */
void start_logging(void);
void stop_logging(void);
void update_pi_terms(float p_term_mA, float p_term_filtered_mA, float p_term_raw_mA, float i_term_mA, float d_term_mA);

/******************************************************************************
 * 共享状态缓冲区 - 用于线程间数据共享
 ******************************************************************************/
typedef struct {
    pthread_mutex_t mutex;
    /* 传感器数据 */
    float pressure_kg;
    float rope_length_m;
    uint32_t encoder_value;
    float encoder_angle_deg;
    /* 重量采集模块数据 */
    float weight_raw_kg;
    float weight_filtered_kg;
    /* 电源数据 - 由算法线程直接更新 */
    float current_a;
    float voltage_v;
    float target_current_a;
    /* 电机数据 */
    float motor_speed_rpm;
    float motor_position_m;
    int32_t motor_status;
    float motor_linear_velocity_m_s;
    float motor_theory_linear_velocity_m_s;
    float motor_torque_percent;     /* 电机目标力矩 (Nm) */
    float actual_torque_nm;         /* 电机实际力矩 (Nm) */
    int32_t control_mode;           /* 控制模式: 0=速度模式, 1=力矩模式 */
    /* 绳子速度数据 */
    float rope_velocity_raw_m_s;
    float rope_velocity_filtered_m_s;
    /* 压力传感器数据 */
    float pressure_f0_kg;
    float pressure_deltaf;
    /* PID控制数据 */
    float pi_p_term_mA;             /* P项最终值（滤波+动态调整后）(Nm) */
    float pi_p_term_filtered_mA;    /* P项滤波后值（未进行动态调整）(Nm) */
    float pi_p_term_raw_mA;         /* P项原始值（未滤波、未调整）(Nm) */
    float pi_i_term_mA;
    float pi_d_term_mA;
    /* 前馈控制数据 */
    float feedforward_current_mA;
    float feedforward_torque_Nm;    /* 扭矩前馈 */
    /* 算法内部数据 */
    float algo_deltaf_kg;
    float algo_pressure_kg;
    /* PI累积电流 */
    float pi_last_current_mA;
} SharedStateBuffer_t;

static SharedStateBuffer_t g_shared_state;

/* 电机转速滑动平均滤波器 */
#define MOTOR_SPEED_FILTER_WINDOW_SIZE  5
static float s_motor_speed_buffer[MOTOR_SPEED_FILTER_WINDOW_SIZE] = {0};
static int s_motor_speed_index = 0;
static int s_motor_speed_count = 0;

static float filter_motor_speed(float raw_speed) {
    s_motor_speed_buffer[s_motor_speed_index] = raw_speed;
    s_motor_speed_index = (s_motor_speed_index + 1) % MOTOR_SPEED_FILTER_WINDOW_SIZE;
    if (s_motor_speed_count < MOTOR_SPEED_FILTER_WINDOW_SIZE)
        s_motor_speed_count++;
    float sum = 0.0f;
    for (int i = 0; i < s_motor_speed_count; i++)
        sum += s_motor_speed_buffer[i];
    return sum / s_motor_speed_count;
}

static void shared_state_init(void) {
    pthread_mutex_init(&g_shared_state.mutex, NULL);
    memset((void*)&g_shared_state + sizeof(pthread_mutex_t), 0,
           sizeof(SharedStateBuffer_t) - sizeof(pthread_mutex_t));
    memset(s_motor_speed_buffer, 0, sizeof(s_motor_speed_buffer));
    s_motor_speed_index = 0;
    s_motor_speed_count = 0;
}

static void update_sensor_to_buffer(SensorData_t *encoder, SensorData_t *pressure) {
    pthread_mutex_lock(&g_shared_state.mutex);
    if (encoder && encoder->data_valid) {
        g_shared_state.encoder_value = encoder->data.encoder.multi_turn_value;
        g_shared_state.encoder_angle_deg = encoder->data.encoder.angle_deg;
        g_shared_state.rope_length_m = encoder->data.encoder.rope_length_mm * 0.001f;
    }
    if (pressure && pressure->data_valid) {
        g_shared_state.pressure_kg = pressure->data.pressure.pressure_filtered_kg;
    }
    pthread_mutex_unlock(&g_shared_state.mutex);
}

static void update_motor_to_buffer(void) {
    pthread_mutex_lock(&g_shared_state.mutex);
    int32_t motor_speed_raw;
    if (motor_get_velocity_cached(&g_motor, &motor_speed_raw) == ERR_OK) {
        float motor_speed_filtered = filter_motor_speed((float)motor_speed_raw);
        g_shared_state.motor_speed_rpm = motor_speed_filtered;
    }
    g_shared_state.motor_position_m = (float)motor_get_position_m(&g_motor);
    g_shared_state.motor_status = g_motor.state;
    float pulley_diameter_m = 0.2f;
    float motor_speed_rpm = g_shared_state.motor_speed_rpm;
    float pulley_speed_rpm = motor_speed_rpm / 3.0f;
    g_shared_state.motor_linear_velocity_m_s = -3.14159f * pulley_diameter_m * pulley_speed_rpm / 60.0f;

    // 注意：目标扭矩由算法通过 update_motor_target_torque 更新
    // 实际扭矩通过 motor_get_torque_cached 在数据收集时读取
    // 不要在此处覆盖 motor_torque_percent，避免目标/实际扭矩混淆

    // 更新控制模式
    g_shared_state.control_mode = (gravity_unload_get_control_mode() == CONTROL_MODE_TORQUE) ? 1 : 0;

    pthread_mutex_unlock(&g_shared_state.mutex);
}

/* 电机状态更新线程 - 独立运行 (100Hz) */
static void* motor_state_update_thread(void* arg) {
    (void)arg;
    struct sched_param param;
    param.sched_priority = 80;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    printf("[MOTOR] Motor state update thread started (100Hz, independent)\n");

    /* 使用绝对时间戳进行精确周期控制 */
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    const int period_us = 10000;  /* 10ms = 100Hz */

    while (g_running) {
        if (g_motor.initialized) {
            motor_update_state(&g_motor);
            /* 更新到共享状态 */
            update_motor_to_buffer();
        }

        /* 精确周期控制 */
        next_time.tv_nsec += period_us * 1000;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec += 1;
            next_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    return NULL;
}

/* 算法线程调用此函数通知PDO线程 */
void notify_pdo_cycle(uint32_t cycle_count) {
    pthread_mutex_lock(&g_pdo_sync_mutex);
    g_algo_pdo_cycle_count = cycle_count;
    pthread_mutex_unlock(&g_pdo_sync_mutex);
}

/* 算法线程调用此函数通知电机状态更新线程 */
void notify_motor_update_cycle(uint32_t cycle_count) {
    pthread_mutex_lock(&g_motor_sync_mutex);
    g_algo_motor_cycle_count = cycle_count;
    pthread_mutex_unlock(&g_motor_sync_mutex);
}

/* 算法线程调用此函数通知数据收集线程 */
void notify_data_collection_cycle(uint64_t timestamp_us, uint32_t cycle_count) {
    (void)timestamp_us;
    pthread_mutex_lock(&g_data_sync_mutex);
    g_algo_data_cycle_count = cycle_count;
    g_algo_data_cycle_updated = 1;  /* 设置更新标志 */
    pthread_cond_signal(&g_data_sync_cond);  /* 通知数据收集线程 */
    pthread_mutex_unlock(&g_data_sync_mutex);
}

/* PDO发送线程 - 与算法线程同步 (100Hz) */
static void* pdo_send_thread(void* arg) {
    (void)arg;
    struct sched_param param;
    param.sched_priority = 95;  /* 最高优先级，确保实时性 */
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    printf("[PDO] PDO send thread started (100Hz, synchronized with algorithm)\n");

    static int motor_enabled_printed = 0;
    static int last_sent_torque = 0;
    static int consecutive_errors = 0;
    static int64_t total_error_count = 0;
    uint32_t last_pdo_cycle = 0;

    while (g_running) {
        /* 获取当前算法周期号 */
        uint32_t current_algo_cycle = 0;
        pthread_mutex_lock(&g_pdo_sync_mutex);
        current_algo_cycle = g_algo_pdo_cycle_count;
        pthread_mutex_unlock(&g_pdo_sync_mutex);

        /* 如果本周期已经发送过，等待下一个周期 */
        if (current_algo_cycle == last_pdo_cycle) {
            usleep(500);  /* 500us */
            continue;
        }
        last_pdo_cycle = current_algo_cycle;

        /* 必须检查电机是否已使能，否则发送扭矩指令无效 */
        if (g_motor.initialized && g_motor_enabled && g_motor.enabled) {
            if (!motor_enabled_printed) {
                printf("[PDO] Motor enabled, starting torque transmission\n");
                motor_enabled_printed = 1;
            }
            /* 从缓存读取目标扭矩并发送 */
            int target_torque = 0;
            pthread_mutex_lock(&g_motor.data_mutex);
            target_torque = g_motor.target_torque;
            pthread_mutex_unlock(&g_motor.data_mutex);

            /* 只在扭矩变化超过阈值时发送，减少CAN总线负载
             * 最小变化阈值：3 (0.003 rated torque) = 约0.015 Nm
             */
            const int MIN_TORQUE_CHANGE_THRESHOLD = 3;
            int torque_diff = abs(target_torque - last_sent_torque);

            if (torque_diff >= MIN_TORQUE_CHANGE_THRESHOLD || consecutive_errors > 0) {
                /* 发送目标扭矩到电机 - 使用1ms超时快速模式 */
                int ret = Nim_set_targetTorque(g_motor.sdk_master, g_motor.node_id, target_torque, 1);
                if (ret != 0) {
                    consecutive_errors++;
                    total_error_count++;
                    /* 错误抑制：只打印前3次和每1000次错误 */
                    if (consecutive_errors <= 3 || consecutive_errors % 1000 == 0) {
                        printf("[PDO] ERROR: Nim_set_targetTorque failed: %d, torque=%d, consecutive=%d\n",
                               ret, target_torque, consecutive_errors);
                    }
                } else {
                    if (torque_diff >= MIN_TORQUE_CHANGE_THRESHOLD) {
                        printf("[PDO] Torque sent: %d (0.001 rated), change=%d\n", target_torque, torque_diff);
                        last_sent_torque = target_torque;
                    }
                    consecutive_errors = 0;
                }
            }
        } else {
            if (motor_enabled_printed) {
                printf("[PDO] Motor disabled, stopping torque transmission\n");
                motor_enabled_printed = 0;
                last_sent_torque = 0;
                consecutive_errors = 0;
            }
        }
    }
    return NULL;
}



/******************************************************************************
 * 外部接口实现（供算法模块调用）
 ******************************************************************************/
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

void update_pi_terms(float p_term_mA, float p_term_filtered_mA, float p_term_raw_mA, float i_term_mA, float d_term_mA) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.pi_p_term_mA = p_term_mA;              /* 最终值：滤波+动态调整 */
    g_shared_state.pi_p_term_filtered_mA = p_term_filtered_mA;  /* 滤波后值：未动态调整 */
    g_shared_state.pi_p_term_raw_mA = p_term_raw_mA;      /* 原始值：未滤波、未调整 */
    g_shared_state.pi_i_term_mA = i_term_mA;
    g_shared_state.pi_d_term_mA = d_term_mA;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_feedforward_current(float feedforward_mA) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.feedforward_current_mA = feedforward_mA;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_feedforward_and_target(float feedforward_mA, float target_mA, float deltaf_kg, float pressure_kg) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.feedforward_current_mA = feedforward_mA;
    g_shared_state.target_current_a = target_mA / 1000.0f;
    g_shared_state.algo_deltaf_kg = deltaf_kg;
    g_shared_state.algo_pressure_kg = pressure_kg;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_feedforward_torque(float feedforward_torque_Nm) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.feedforward_torque_Nm = feedforward_torque_Nm;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_pi_last_current(float last_current_mA) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.pi_last_current_mA = last_current_mA;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_power_feedback(float current_a, float voltage_v, float target_a) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.current_a = current_a;
    g_shared_state.voltage_v = voltage_v;
    g_shared_state.target_current_a = target_a;
    pthread_mutex_unlock(&g_shared_state.mutex);
}

void update_motor_target_torque(float torque_nm) {
    pthread_mutex_lock(&g_shared_state.mutex);
    g_shared_state.motor_torque_percent = torque_nm;  /* 电机力矩 (Nm) */
    pthread_mutex_unlock(&g_shared_state.mutex);
}

uint32_t get_timestamp_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

int get_sensor_data(SensorDataRaw_t *data) {
    if (data == NULL) return -1;
    SensorData_t encoder_data, pressure_data;
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) != ERR_OK) return -1;
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure_data) != ERR_OK) return -1;
    data->pressure_kg = pressure_data.data.pressure.pressure_filtered_kg;
    data->encoder_position_m = encoder_data.data.encoder.rope_length_mm / 1000.0f * ENCODER_DIRECTION;
    data->encoder_pulse_delta = encoder_data.data.encoder.pulse_delta * ENCODER_DIRECTION;
    data->encoder_time_delta_us = encoder_data.data.encoder.time_delta_us;
    data->timestamp_ms = get_timestamp_ms();
    data->data_valid = (encoder_data.data_valid && pressure_data.data_valid);
    return 0;
}

/* 直接同步设置电机速度（不再经过异步缓冲区） */
int set_motor_velocity(float velocity) {
    if (!g_motor_enabled || !g_motor.initialized) return -1;
    return (motor_set_velocity(&g_motor, velocity) == ERR_OK) ? 0 : -1;
}

/* 直接同步设置电机力矩（新增）- 使用缓存版本，避免CAN通信阻塞 */
int set_motor_torque(int torque) {
    if (!g_motor_enabled || !g_motor.initialized) return -1;
    return (motor_set_torque_cached(&g_motor, torque) == ERR_OK) ? 0 : -1;
}

int get_motor_actual_velocity(float *velocity) {
    if (!g_motor_enabled || !g_motor.initialized || velocity == NULL) return -1;
    int32_t vel;
    if (motor_get_velocity_cached(&g_motor, &vel) == ERR_OK) {
        *velocity = (float)vel;
        return 0;
    }
    return -1;
}

/* 注意：set_clutch_current 不再由算法线程调用，保留空实现以避免链接错误 */
int set_clutch_current(float current_mA) {
    (void)current_mA;
    return 0;
}

/******************************************************************************
 * 信号处理
 ******************************************************************************/
void signal_handler(int sig) {
    (void)sig;
    printf("\n[INFO] Signal received, shutting down...\n");
    g_running = 0;
    printf("[EMERGENCY STOP] Stopping motor and clutch immediately...\n");
    if (g_motor.initialized) {
        motor_set_velocity(&g_motor, 0);
        motor_disable(&g_motor);
        printf("[EMERGENCY STOP] Motor stopped and disabled\n");
    }
    if (g_power.initialized && g_power.state == POWER_STATE_ON) {
        power_set_current(&g_power, 0);
        power_deinit(&g_power);
        printf("[EMERGENCY STOP] Clutch current set to 0\n");
    }
    sensor_mgr_deinit(&g_sensor_mgr);
}

/******************************************************************************
 * 系统预检测函数
 ******************************************************************************/
static int check_sensors_impl(void) {
    SensorData_t encoder, pressure;
    printf("[CHECK] Checking sensors...\n");
    fflush(stdout);
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder) != ERR_OK || !encoder.data_valid) {
        printf("[CHECK FAIL] Encoder not available\n");
        return -1;
    }
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure) != ERR_OK || !pressure.data_valid) {
        printf("[CHECK FAIL] Pressure sensor not available\n");
        return -1;
    }
    printf("[CHECK PASS] Sensors OK\n");
    return 0;
}

static int check_motor_impl(void) {
    if (!g_motor_enabled) return 0;
    if (!g_motor.initialized) return -1;
    int retries = 3;
    int motor_ok = 0;
    while (retries-- > 0) {
        if (motor_update_state(&g_motor) == ERR_OK) { motor_ok = 1; break; }
        usleep(100000);
    }
    if (!motor_ok) return -1;
    printf("[CHECK PASS] Motor OK\n");
    return 0;
}

static int check_power_impl(void) {
    uint16_t current, voltage;
    if (power_get_status(&g_power, &current, &voltage) != ERR_OK) {
        printf("[CHECK WARN] Power board not available\n");
        return 0;
    }
    printf("[CHECK PASS] Power board OK\n");
    return 0;
}

static int check_safety_impl(void) {
    SensorData_t pressure;
    if (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure) != ERR_OK) return -1;
    if (pressure.data.pressure.pressure_kg < SAFETY_PRESSURE_MIN_KG ||
        pressure.data.pressure.pressure_kg > SAFETY_PRESSURE_MAX_KG) {
        printf("[CHECK FAIL] Pressure out of safety range: %.3f kg\n", pressure.data.pressure.pressure_kg);
        return -1;
    }
    printf("[CHECK PASS] Safety check OK\n");
    return 0;
}

/******************************************************************************
 * 共享内存数据线程 - 20Hz输出
 ******************************************************************************/
static void* data_output_thread(void* arg) {
    (void)arg;
    struct sched_param param;
    param.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    printf("[SHM] Data output thread started (%dHz)\n", 1000 / SHM_DATA_OUTPUT_PERIOD_MS);
    SharedData_t data;
    const long period_ms = SHM_DATA_OUTPUT_PERIOD_MS;
    while (g_running) {
        uint32_t start_time = get_timestamp_ms();
        memset(&data, 0, sizeof(SharedData_t));
        pthread_mutex_lock(&g_shared_state.mutex);
        data.pressure_kg = g_shared_state.pressure_kg;
        data.rope_length_m = g_shared_state.rope_length_m;
        data.encoder_value = g_shared_state.encoder_value;
        data.encoder_angle_deg = g_shared_state.encoder_angle_deg;
        data.weight_raw_kg = g_shared_state.weight_raw_kg;
        data.weight_filtered_kg = g_shared_state.weight_filtered_kg;
        data.current_a = g_shared_state.current_a;
        data.voltage_v = g_shared_state.voltage_v;
        data.motor_speed_rpm = g_shared_state.motor_speed_rpm;
        data.motor_position = g_shared_state.motor_position_m;
        data.motor_status = g_shared_state.motor_status;
        // 力矩控制数据（新增）
        data.control_mode = g_shared_state.control_mode;
        data.motor_torque_percent = g_shared_state.motor_torque_percent;
        // 读取实际扭矩并更新到共享内存和数据包
        int actual_torque_cmd = 0;
        motor_get_torque_cached(&g_motor, &actual_torque_cmd);
        float actual_torque_nm = (actual_torque_cmd / 1000.0f) * 1.27f;  // 转换为Nm
        g_shared_state.actual_torque_nm = actual_torque_nm;
        data.actual_torque_nm = actual_torque_nm;
        pthread_mutex_unlock(&g_shared_state.mutex);
        if (g_algorithm_enabled) {
            AlgoStatus_t status;
            gravity_unload_get_status(&g_gravity_ctrl, &status);
            data.algorithm_state = (int32_t)status.state;
            data.algorithm_error = (int32_t)status.error;
        } else {
            data.algorithm_state = 0;
            data.algorithm_error = 0;
        }
        data.emergency_stop = false;
        if (g_shm_initialized) shm_write_data(&g_shm_mgr, &data);
        uint32_t elapsed_ms = get_timestamp_ms() - start_time;
        if (elapsed_ms < period_ms) usleep((period_ms - elapsed_ms) * 1000);
    }
    printf("[SHM] Data output thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 命令处理线程
 ******************************************************************************/
static void* command_handler_thread(void* arg) {
    (void)arg;
    printf("[SHM] Command handler thread started\n");
    SharedCommand_t cmd;
    while (g_running) {
        if (g_shm_initialized && shm_read_command(&g_shm_mgr, &cmd) == 0 && cmd.command_id != g_last_cmd_id) {
            g_last_cmd_id = cmd.command_id;
            
            // 处理力矩控制命令（新增）
            // 只有当control_mode_changed标志被设置时才处理控制模式切换
            if (cmd.control_mode_changed) {
                if (cmd.control_mode == 1) {
                    // 切换到力矩模式
                    printf("[CMD] Switching to TORQUE mode\n");
                    gravity_unload_set_control_mode(CONTROL_MODE_TORQUE);
                } else if (cmd.control_mode == 0) {
                    // 切换到速度模式
                    printf("[CMD] Switching to VELOCITY mode\n");
                    gravity_unload_set_control_mode(CONTROL_MODE_VELOCITY);
                }
            }
            
            // 设置目标力矩并直接输出（不依赖算法线程）
            if (cmd.target_torque_nm != 0.0f || cmd.control_mode == 1) {
                printf("[CMD] Set target torque: %.3f Nm\n", cmd.target_torque_nm);
                gravity_unload_set_target_torque(cmd.target_torque_nm);
                
                // 更新共享内存中的目标扭矩值（用于上位机显示和数据记录）
                update_motor_target_torque(cmd.target_torque_nm);
                
                // 如果当前是力矩模式且电机已使能，直接输出力矩
                if (gravity_unload_get_control_mode() == CONTROL_MODE_TORQUE && 
                    g_motor_enabled && g_motor.initialized && g_motor.enabled) {
                    // 将 Nm 转换为电机指令 (0.001倍额定)
                    // 额定力矩为 1.27Nm，则 0.4Nm = 315 (0.315倍额定)
                    int torque_cmd = (int)((cmd.target_torque_nm / 1.27f) * 1000.0f);
                    if (torque_cmd > 1000) torque_cmd = 1000;
                    if (torque_cmd < -1000) torque_cmd = -1000;
                    printf("[CMD] Output torque command: %d (0.001 rated, %.3f Nm)\n", torque_cmd, cmd.target_torque_nm);
                    motor_set_torque(&g_motor, torque_cmd);
                }
            }
            
            // 使能力矩模式
            if (cmd.enable_torque_mode) {
                printf("[CMD] Enabling torque mode (CST)\n");
                if (g_motor_enabled && g_motor.initialized) {
                    motor_enable_torque_mode(&g_motor);
                }
            }
            
            // 处理正弦测试命令
            if (cmd.sine_test_start && !g_sine_test_enabled) {
                printf("[CMD] Starting sine test: A=%.3f Nm, f=%.2f Hz, offset=%.3f Nm\n",
                       cmd.sine_amplitude, cmd.sine_frequency, cmd.sine_offset);
                g_sine_amplitude = cmd.sine_amplitude;
                g_sine_frequency = cmd.sine_frequency;
                g_sine_offset = cmd.sine_offset;
                g_sine_start_time = 0.0;
                g_sine_cycle_count = 0;
                g_sine_test_enabled = 1;
                
                // 自动切换到力矩模式
                gravity_unload_set_control_mode(CONTROL_MODE_TORQUE);
                if (g_motor_enabled && g_motor.initialized && !g_motor.enabled) {
                    motor_enable_torque_mode(&g_motor);
                }
            }
            
            if (cmd.sine_test_stop && g_sine_test_enabled) {
                printf("[CMD] Stopping sine test, cycles=%lu\n", g_sine_cycle_count);
                g_sine_test_enabled = 0;
                // 发送零力矩
                gravity_unload_set_target_torque(0.0f);
                int torque_cmd = 0;
                motor_set_torque(&g_motor, torque_cmd);
            }
            
            // 处理传统命令类型
            if (cmd.cmd_type != 0) {
                printf("[CMD] Received command id=%u type=%d value=%.2f\n", cmd.command_id, cmd.cmd_type, cmd.cmd_value);
                switch (cmd.cmd_type) {
                    case 1:
                        if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                            motor_set_mode(&g_motor, MOTOR_MODE_CSV);
                            motor_set_velocity(&g_motor, cmd.cmd_value);
                        }
                        break;
                    case 2:
                        if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                            motor_set_mode(&g_motor, MOTOR_MODE_PP);
                            motor_set_position(&g_motor, (int32_t)cmd.cmd_value);
                        }
                        break;
                    case 3:
                        if (g_motor_enabled && g_motor.initialized) motor_set_velocity(&g_motor, 0);
                        break;
                    case 4:
                        if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) {
                            // 根据当前控制模式选择使能模式
                            ControlMode_t current_mode = gravity_unload_get_control_mode();
                            if (current_mode == CONTROL_MODE_TORQUE) {
                                printf("[CMD] Enabling motor in TORQUE mode (CST)\n");
                                motor_enable_torque_mode(&g_motor);
                            } else {
                                printf("[CMD] Enabling motor in VELOCITY mode (CSV)\n");
                                motor_set_mode(&g_motor, MOTOR_MODE_CSV);
                                motor_enable(&g_motor);
                            }
                        }
                        break;
                    case 5:
                        if (g_motor_enabled && g_motor.initialized && !g_algorithm_enabled) motor_disable(&g_motor);
                        break;
                    case 6:
                        power_set_current(&g_power, (uint16_t)(cmd.cmd_value * 1000.0f));
                        break;
                    default: break;
                }
            }
            shm_clear_command(&g_shm_mgr);
        }
        if (cmd.algorithm_start && !g_algorithm_enabled) printf("[CMD] Starting algorithm...\n");
        if (cmd.algorithm_stop && g_algorithm_enabled) printf("[CMD] Stopping algorithm...\n");
        if (cmd.data_log_start && !g_logging_enabled) {
            g_logging_enabled = 1;
            g_manual_f0_sample_count = 0;
            g_manual_f0_sum = 0.0f;
            g_manual_f0_kg = 0.0f;
            start_logging();
        }
        if (cmd.data_log_stop && g_logging_enabled) {
            g_logging_enabled = 0;
            stop_logging();
        }
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
        if (g_algorithm_enabled) gravity_unload_print_status(&g_gravity_ctrl);
        for (int i = 0; i < 10 && g_running; i++) usleep(100000);
    }
    printf("[MONITOR] Monitor thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 数据收集线程 - 100Hz更新共享缓冲区
 ******************************************************************************/
static FILE *g_log_file = NULL;
static uint32_t g_log_count = 0;
static char g_log_filename[128];

void start_logging(void) {
    if (g_log_file) return;
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    snprintf(g_log_filename, sizeof(g_log_filename),
             "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys_torque_6_15/logdata/gravity_data_%04d%02d%02d_%02d%02d%02d.csv",
             t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    mkdir("/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys_torque_6_15/logdata", 0755);
    g_log_file = fopen(g_log_filename, "w");
    if (g_log_file) {
        fprintf(g_log_file, "%-20s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-15s,%-12s,%-12s,%-12s,%-12s,%-12s,%-12s,%-14s,%-14s,%-16s,%-14s,%-20s,%-20s,%-16s,%-18s,%-22s,%-14s,%-14s,%-14s,%-14s,%-14s,%-14s\n",
                "Time","Current(A)","TargetCurrent(A)","Voltage(V)","PressureRaw(kg)","PressureFiltered(kg)","F0(kg)","DeltaF",
                "PI_P(Nm)","PI_P_Filtered(Nm)","PI_P_Raw(Nm)","PI_I(Nm)","PI_D(Nm)","PI_LastTorque(Nm)","Feedforward(mA)","FeedforwardTorque(Nm)","AlgoDeltaF(kg)","AlgoPressure(kg)",
                "RopeLength(m)","EncoderValue","EncoderAngle(deg)","MotorSpeed(rpm)",
                "MotorLinearVel(m/s)","MotorTheoryVel(m/s)","MotorPosition(m)",
                "RopeVelocityRaw(m/s)","RopeVelocityFiltered(m/s)","WeightRaw(kg)","WeightFiltered(kg)","TargetTorque(Nm)","ActualTorque(Nm)");
        printf("[LOG] Started logging to: %s\n", g_log_filename);
        g_log_count = 0;
    } else {
        printf("[ERROR] Failed to create log file: %s\n", g_log_filename);
    }
}

void stop_logging(void) {
    if (g_log_file) {
        fclose(g_log_file);
        g_log_file = NULL;
        printf("[LOG] Stopped logging, total records: %u\n", g_log_count);
    }
}



static void* data_collection_thread(void* arg) {
    (void)arg;
    struct sched_param param;
    param.sched_priority = 87;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    printf("[DATA] Data collection thread started - 100Hz independent mode\n");

    /* 使用绝对时间戳进行精确周期控制 */
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    const int period_us = 10000;  /* 10ms = 100Hz */

    uint32_t last_recorded_cycle = 0;
    int first_run = 1;

    while (g_running) {
        /* 首次运行时初始化时间基准 */
        if (first_run) {
            clock_gettime(CLOCK_MONOTONIC, &next_time);
            first_run = 0;
        }

        /* 在算法模式下，使用条件变量等待通知 */
        /* 在手动模式下，使用周期控制 */
        if (g_algorithm_mode) {
            pthread_mutex_lock(&g_data_sync_mutex);
            int wait_result = 0;
            while (g_algo_data_cycle_count == last_recorded_cycle && g_running) {
                /* 等待条件变量，最多等待15ms */
                struct timespec timeout;
                clock_gettime(CLOCK_REALTIME, &timeout);
                timeout.tv_nsec += 15 * 1000 * 1000;  /* 15ms */
                if (timeout.tv_nsec >= 1000000000) {
                    timeout.tv_sec += 1;
                    timeout.tv_nsec -= 1000000000;
                }
                wait_result = pthread_cond_timedwait(&g_data_sync_cond, &g_data_sync_mutex, &timeout);
                /* 如果超时且没有更新标志，继续等待 */
                if (wait_result == ETIMEDOUT && !g_algo_data_cycle_updated) {
                    continue;
                }
                /* 如果收到信号，重置更新标志 */
                if (g_algo_data_cycle_updated) {
                    g_algo_data_cycle_updated = 0;
                    break;
                }
            }
            pthread_mutex_unlock(&g_data_sync_mutex);
        }

        uint32_t current_algo_cycle = g_algo_data_cycle_count;

        /* 如果本周期已经记录过，跳过（仅在算法模式下） */
        if (g_algorithm_mode && current_algo_cycle == last_recorded_cycle) {
            continue;
        }
        
        SensorData_t encoder_data, pressure_data;
        float pressure_kg = 0.0f, rope_length_m = 0.0f;
        uint32_t encoder_value = 0;
        float encoder_angle = 0.0f;
        int encoder_ok = (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_ENCODER, &encoder_data) == ERR_OK);
        int pressure_ok = (sensor_mgr_get_data(&g_sensor_mgr, SENSOR_TYPE_PRESSURE, &pressure_data) == ERR_OK);
        if (encoder_ok) {
            encoder_value = encoder_data.data.encoder.multi_turn_value;
            encoder_angle = encoder_data.data.encoder.angle_deg;
            rope_length_m = encoder_data.data.encoder.rope_length_mm / 1000.0f;
        }
        float pressure_raw_kg = 0.0f;
        if (pressure_ok) {
            pressure_raw_kg = pressure_data.data.pressure.pressure_kg;
            pressure_kg = pressure_data.data.pressure.pressure_filtered_kg;
        }
        if (encoder_ok) update_sensor_to_buffer(&encoder_data, NULL);
        if (pressure_ok) update_sensor_to_buffer(NULL, &pressure_data);
        if (g_motor.initialized) update_motor_to_buffer();
        /* 重量数据 - 从缓存读取 */
        static float weight_cached_kg = 0.0f;
        if (g_weight.initialized) {
            float temp_weight = 0.0f;
            weight_get_data(&g_weight, &temp_weight, 1);
            if (temp_weight > 0.0f) weight_cached_kg = temp_weight;
        }
        pthread_mutex_lock(&g_shared_state.mutex);
        g_shared_state.weight_raw_kg = weight_cached_kg;
        g_shared_state.weight_filtered_kg = weight_cached_kg;
        pthread_mutex_unlock(&g_shared_state.mutex);
        /* 电源状态更新 - 定期读取实际电流电压到缓存 */
        static int power_update_counter = 0;
        if (g_power.initialized && (++power_update_counter >= 10)) {  /* 每100ms更新一次 */
            power_update_counter = 0;
            uint16_t actual_current_ma = 0, actual_voltage_mv = 0;
            power_get_status(&g_power, &actual_current_ma, &actual_voltage_mv);
            /* 更新到共享状态 */
            pthread_mutex_lock(&g_shared_state.mutex);
            g_shared_state.current_a = actual_current_ma / 1000.0f;
            g_shared_state.voltage_v = actual_voltage_mv / 1000.0f;
            pthread_mutex_unlock(&g_shared_state.mutex);
        }
        /* 原子读取所有数据 */
        float current_a = 0.0f, voltage_v = 0.0f, motor_speed_rpm = 0.0f, motor_pos_m = 0.0f, motor_linear_vel = 0.0f;
        float motor_theory_vel = 0.0f, rope_vel_raw = 0.0f, rope_vel_filtered = 0.0f;
        float pressure_f0_kg = 0.0f, pressure_deltaf = 0.0f;
        float pi_p_term_mA = 0.0f, pi_p_term_filtered_mA = 0.0f, pi_p_term_raw_mA = 0.0f, pi_i_term_mA = 0.0f, pi_d_term_mA = 0.0f, pi_last_current_mA = 0.0f;
        float target_current_a = 0.0f, feedforward_current_mA = 0.0f, feedforward_torque_Nm = 0.0f, algo_deltaf_kg = 0.0f, algo_pressure_kg = 0.0f;
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
        pi_p_term_filtered_mA = g_shared_state.pi_p_term_filtered_mA;
        pi_p_term_raw_mA = g_shared_state.pi_p_term_raw_mA;
        pi_i_term_mA = g_shared_state.pi_i_term_mA;
        pi_d_term_mA = g_shared_state.pi_d_term_mA;
        pi_last_current_mA = g_shared_state.pi_last_current_mA;
        target_current_a = g_shared_state.target_current_a;
        feedforward_current_mA = g_shared_state.feedforward_current_mA;
        feedforward_torque_Nm = g_shared_state.feedforward_torque_Nm;
        algo_deltaf_kg = g_shared_state.algo_deltaf_kg;
        algo_pressure_kg = g_shared_state.algo_pressure_kg;
        pthread_mutex_unlock(&g_shared_state.mutex);
        /* 正弦测试信号生成 - 100Hz */
        float sine_target_torque = 0.0f;
        if (g_sine_test_enabled) {
            g_sine_start_time += 0.01;  // 100Hz, 每周期10ms
            double omega = 2.0 * M_PI * g_sine_frequency;
            sine_target_torque = g_sine_offset + g_sine_amplitude * sin(omega * g_sine_start_time);
            
            // 更新目标力矩并输出到电机
            gravity_unload_set_target_torque(sine_target_torque);
            if (gravity_unload_get_control_mode() == CONTROL_MODE_TORQUE && 
                g_motor_enabled && g_motor.initialized && g_motor.enabled) {
                int torque_cmd = (int)((sine_target_torque / 1.27f) * 1000.0f);
                if (torque_cmd > 1000) torque_cmd = 1000;
                if (torque_cmd < -1000) torque_cmd = -1000;
                motor_set_torque(&g_motor, torque_cmd);
            }
            
            // 周期计数
            static double last_phase = 0.0;
            double current_phase = fmod(omega * g_sine_start_time, 2.0 * M_PI);
            if (current_phase < last_phase) {
                g_sine_cycle_count++;
            }
            last_phase = current_phase;
        }
        
        /* 手动模式 F0 计算 */
        if (pressure_f0_kg == 0.0f) {
            if (g_manual_f0_sample_count < MANUAL_F0_SAMPLE_COUNT) {
                g_manual_f0_sum += pressure_kg;
                g_manual_f0_sample_count++;
                g_manual_f0_kg = g_manual_f0_sum / g_manual_f0_sample_count;
            }
            pressure_deltaf = pressure_kg - g_manual_f0_kg;
        } else {
            pressure_deltaf = algo_deltaf_kg;
        }
        /* 日志记录 */
        if (g_log_file != NULL && g_logging_enabled) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            struct tm *t = localtime(&tv.tv_sec);
            char time_str[32];
            snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d.%06ld", t->tm_hour, t->tm_min, t->tm_sec, (long)tv.tv_usec);
            float log_f0_kg = (pressure_f0_kg != 0.0f) ? pressure_f0_kg : g_manual_f0_kg;
            float log_weight_raw = 0.0f, log_weight_filtered = 0.0f;
            pthread_mutex_lock(&g_shared_state.mutex);
            log_weight_raw = g_shared_state.weight_raw_kg;
            log_weight_filtered = g_shared_state.weight_filtered_kg;
            pthread_mutex_unlock(&g_shared_state.mutex);
            // 获取目标扭矩和实际扭矩
            float target_torque_nm;
            if (g_sine_test_enabled) {
                target_torque_nm = sine_target_torque;  // 使用正弦目标值
            } else {
                // 从共享内存读取算法计算的电机目标扭矩（加锁保护）
                // motor_torque_percent 现在直接存储Nm值
                pthread_mutex_lock(&g_shared_state.mutex);
                target_torque_nm = g_shared_state.motor_torque_percent;  // 直接是Nm单位
                pthread_mutex_unlock(&g_shared_state.mutex);
            }
            int actual_torque_cmd = 0;
            motor_get_torque_cached(&g_motor, &actual_torque_cmd);
            float actual_torque_nm = (actual_torque_cmd / 1000.0f) * 1.27f;  // 转换为Nm
            fprintf(g_log_file, "%-20s,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-15.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-12.3f,%-14.5f,%-14u,%-16.3f,%-14.3f,%-20.3f,%-20.3f,%-16.3f,%-18.5f,%-22.5f,%-14.3f,%-14.3f,%-14.3f,%-14.3f,%-14.3f\n",
                    time_str, current_a, target_current_a, voltage_v, pressure_raw_kg, pressure_kg, log_f0_kg, pressure_deltaf,
                    pi_p_term_mA, pi_p_term_filtered_mA, pi_p_term_raw_mA, pi_i_term_mA, pi_d_term_mA, pi_last_current_mA, feedforward_current_mA, feedforward_torque_Nm, algo_deltaf_kg, algo_pressure_kg,
                    rope_length_m, encoder_value, encoder_angle, motor_speed_rpm,
                    motor_linear_vel, motor_theory_vel, motor_pos_m,
                    rope_vel_raw, rope_vel_filtered, log_weight_raw, log_weight_filtered, target_torque_nm, actual_torque_nm);
            g_log_count++;
            if (g_log_count % 100 == 0) fflush(g_log_file);
            
            /* 记录本次采集的周期号 */
            last_recorded_cycle = current_algo_cycle;
        }

        /* 精确周期控制 - 100Hz */
        next_time.tv_nsec += period_us * 1000;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec += 1;
            next_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    if (g_log_file) { fclose(g_log_file); g_log_file = NULL; }
    printf("[DATA] Data collection thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 主函数
 ******************************************************************************/
int main(int argc, char *argv[]) {
    const char* sdk_path = "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/lib";
    if (chdir(sdk_path) != 0) printf("WARNING: Failed to change directory to SDK path: %s\n", sdk_path);
    setenv("LD_LIBRARY_PATH", sdk_path, 1);
    if (argc > 1 && (strcmp(argv[1], "--no-motor") == 0 || strcmp(argv[1], "-n") == 0)) {
        g_motor_enabled = 0;
        printf("Motor control disabled\n");
    } else {
        printf("Motor control enabled\n");
    }
    fflush(stdout);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    logger_init(&g_logger, NULL, LOG_LEVEL_DEBUG, 1);
    printf("========================================\n");
    printf("CANOpNode_Sys v4.0.0 (Gravity Unload)\n");
    printf("NiMotion SDK Integration - Industrial Grade\n");
    printf("Shared Memory Communication Enabled\n");
    printf("========================================\n\n");
    fflush(stdout);

    shared_state_init();

    /* 初始化硬件 */
    printf("[INIT] Phase 1: Initializing hardware...\n");
    if (sensor_mgr_init(&g_sensor_mgr, ENCODER_UART_DEVICE, ENCODER_UART_BAUDRATE) != ERR_OK) {
        printf("  -> Sensor manager: WARNING (sensor disabled)\n");
    } else {
        printf("  -> Sensor manager: OK\n");
    }
    sensor_mgr_set_encoder_rope_params(&g_sensor_mgr, 100.0f, 4096);
    if (power_init(&g_power, POWER_UART_DEVICE, POWER_UART_BAUDRATE) != ERR_OK) {
        printf("  -> Power driver: WARNING (power disabled)\n");
    } else {
        printf("  -> Power driver: OK\n");
        /* 设置电源初始电流为880mA */
        power_set_current(&g_power, 880);
        printf("  -> Power initial current: 880mA\n");
    }
    if (weight_init(&g_weight, WEIGHT_UART_DEVICE, WEIGHT_UART_BAUDRATE) != ERR_OK) {
        printf("  -> Weight driver: WARNING (weight disabled)\n");
    } else {
        printf("  -> Weight driver: OK\n");
        if (weight_start_collection(&g_weight) == ERR_OK) printf("  -> Weight collection thread: OK\n");
    }
    if (g_motor_enabled) {
        if (motor_init(&g_motor, MOTOR_NODE_ID, MOTOR_CAN_INTERFACE) != ERR_OK) {
            printf("  -> Motor driver: WARNING\n");
            g_motor_enabled = 0;
        } else {
            printf("  -> Motor driver: OK\n");
        }
    }
    if (sensor_mgr_start(&g_sensor_mgr) != ERR_OK) {
        printf("  -> Sensor manager thread: WARNING\n");
    } else {
        printf("  -> Sensor manager thread: OK\n");
        sleep(2);
        sensor_mgr_encoder_zero_calibration(&g_sensor_mgr);
        usleep(500000);
        sensor_mgr_pressure_tare(&g_sensor_mgr);
    }
    printf("\n");

    /* 共享内存初始化 */
    printf("[INIT] Phase 2: Initializing shared memory...\n");
    if (shm_init(&g_shm_mgr, true) != 0) {
        printf("  -> Shared memory: FAILED\n");
    } else {
        g_shm_initialized = 1;
        printf("  -> Shared memory: OK\n");
    }

    /* 系统预检测 */
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
        weight_stop_collection(&g_weight);
        weight_deinit(&g_weight);
        sensor_mgr_deinit(&g_sensor_mgr);
        if (g_shm_initialized) shm_close(&g_shm_mgr);
        return 1;
    }
    printf("\n[INIT] All system checks PASSED!\n\n");

    /* 启动数据通信线程 */
    printf("[INIT] Phase 4: Starting communication threads...\n");
    printf("  -> Starting data collection thread (%dHz)... ", 1000 / SHM_DATA_COLLECTION_PERIOD_MS);
    fflush(stdout);
    pthread_create(&g_collection_thread_tid, NULL, data_collection_thread, NULL);
    printf("OK\n");
    printf("  -> Starting motor state update thread... ");
    fflush(stdout);
    pthread_create(&g_motor_state_thread_tid, NULL, motor_state_update_thread, NULL);
    printf("OK\n");
    printf("  -> Starting PDO send thread (200Hz)... ");
    fflush(stdout);
    pthread_create(&g_pdo_send_thread_tid, NULL, pdo_send_thread, NULL);
    printf("OK\n");
    /* 注：actuator_control_thread 和 power_state_update_thread 已删除 */
    printf("  -> Starting data output thread (%dHz)... ", 1000 / SHM_DATA_OUTPUT_PERIOD_MS);
    fflush(stdout);
    pthread_create(&g_data_thread_tid, NULL, data_output_thread, NULL);
    printf("OK\n");
    printf("  -> Starting command handler thread... ");
    fflush(stdout);
    pthread_create(&g_command_thread_tid, NULL, command_handler_thread, NULL);
    printf("OK\n");

    /* 用户输入判断 */
    printf("\n[INIT] Phase 5: System ready for remote monitoring\n");
    printf("Waiting for commands from GravShow...\n");
    printf("Or press Enter to start algorithm mode locally\n");
    printf("Press Ctrl+C to exit\n\n");
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    int user_input = 0;
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) user_input = 1;
    }

    if (user_input) {
        printf("\n[INFO] Starting algorithm mode...\n\n");
        g_algorithm_mode = 1;
        if (g_motor_enabled) {
            /* 先设置电机为CST模式（扭矩控制模式），再使能 */
            printf("  -> Setting motor to CST mode... ");
            fflush(stdout);
            if (motor_set_mode(&g_motor, MOTOR_MODE_CST) != ERR_OK) {
                printf("WARNING\n");
            } else {
                printf("OK\n");
            }

            printf("  -> Enabling motor... ");
            fflush(stdout);
            if (motor_enable(&g_motor) != ERR_OK) {
                printf("WARNING\n");
                g_motor_enabled = 0;
            } else {
                printf("OK (Mode: %s)\n",
                       g_motor.mode == MOTOR_MODE_CST ? "CST" : "OTHER");
            }

            /* 确保目标扭矩清零，避免初始扭矩冲击 */
            printf("  -> Resetting target torque to 0... ");
            fflush(stdout);
            pthread_mutex_lock(&g_motor.data_mutex);
            g_motor.target_torque = 0;
            pthread_mutex_unlock(&g_motor.data_mutex);
            /* 立即发送零扭矩到电机 */
            Nim_set_targetTorque(g_motor.sdk_master, g_motor.node_id, 0, 2);
            printf("OK\n");
        }
        if (gravity_unload_init(&g_gravity_ctrl) != 0) {
            printf("[ERROR] Failed to initialize algorithm\n");
            g_running = 0;
        } else {
            /* 启动算法前确保电机已使能 */
            if (g_motor_enabled && g_motor.initialized && g_motor.enabled) {
                printf("[INIT] Motor ready in CST mode, starting algorithm...\n");
            } else if (g_motor_enabled && g_motor.initialized && !g_motor.enabled) {
                printf("[INIT] WARNING: Motor not enabled, algorithm may not work correctly\n");
            }
            
            if (gravity_unload_start(&g_gravity_ctrl) != 0) {
                printf("[ERROR] Failed to start algorithm\n");
                g_running = 0;
            } else {
                g_algorithm_enabled = 1;
                pthread_create(&g_monitor_thread_tid, NULL, monitor_thread, NULL);
            }
        }
    } else {
        printf("[INFO] Running in manual mode (remote control enabled)\n\n");
    }

    printf("========================================\n");
    printf("System RUNNING\n");
    printf("Mode: %s\n", g_algorithm_enabled ? "ALGORITHM" : "MANUAL");
    printf("Press Ctrl+C to stop\n");
    printf("========================================\n\n");
    fflush(stdout);

    while (g_running) {
        if (g_algorithm_enabled) {
            AlgoStatus_t status;
            gravity_unload_get_status(&g_gravity_ctrl, &status);
            if (status.state == ALGO_STATE_EMERGENCY_STOP) break;
        }
        sleep(1);
    }

    /* 清理 */
    printf("\n[SHUTDOWN] Stopping system...\n");
    if (g_algorithm_enabled) {
        gravity_unload_stop(&g_gravity_ctrl);
        gravity_unload_deinit(&g_gravity_ctrl);
        pthread_join(g_monitor_thread_tid, NULL);
    }
    pthread_join(g_collection_thread_tid, NULL);
    pthread_join(g_motor_state_thread_tid, NULL);
    pthread_join(g_pdo_send_thread_tid, NULL);
    pthread_join(g_data_thread_tid, NULL);
    pthread_join(g_command_thread_tid, NULL);
    sensor_mgr_stop(&g_sensor_mgr);
    power_deinit(&g_power);
    weight_stop_collection(&g_weight);
    weight_deinit(&g_weight);
    sensor_mgr_deinit(&g_sensor_mgr);
    if (g_shm_initialized) shm_close(&g_shm_mgr);
    printf("\n[System shutdown completed]\n");
    logger_deinit(NULL);
    return 0;
}