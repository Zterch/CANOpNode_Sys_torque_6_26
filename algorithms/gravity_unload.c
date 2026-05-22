/******************************************************************************
 * @file    gravity_unload.c
 * @brief   重力卸载控制算法实现
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#include "gravity_unload.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

/* 外部依赖声明 - 需要在主程序中提供 */
extern int get_sensor_data(SensorDataRaw_t *data);
extern int set_motor_velocity(float velocity);
extern int set_clutch_current(float current_mA);
extern int get_motor_actual_velocity(float *velocity);
extern uint32_t get_timestamp_ms(void);
extern void update_rope_velocity(float raw_velocity, float filtered_velocity);
extern void update_motor_theory_velocity(float theory_velocity);
extern void update_pressure_f0_deltaf(float f0_kg, float deltaf);

/* 内部辅助函数 - 获取微秒时间戳 */
static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void* gravity_unload_thread(void *arg);
static void process_sensor_data(GravityUnloadController_t *ctrl,
                                const SensorDataRaw_t *raw,
                                SensorDataFiltered_t *filtered);
static void calculate_control_output(GravityUnloadController_t *ctrl,
                                     const SensorDataFiltered_t *filtered,
                                     ControlOutput_t *output);
static float pressure_filter_update(GravityUnloadController_t *ctrl, float raw_pressure);
static void calibrate_pressure_f0(GravityUnloadController_t *ctrl, float pressure_kg);
static float calculate_motor_speed_by_friction(GravityUnloadController_t *ctrl,
                                                float filtered_pressure_kg,
                                                float pulley_velocity_rpm);

/******************************************************************************
 * 初始化与反初始化
 ******************************************************************************/

int gravity_unload_init(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return -1;
    
    memset(ctrl, 0, sizeof(GravityUnloadController_t));
    
    /* 初始化配置参数 */
    ctrl->pulley_r1_m = PULLEY_R1_MOTOR_RADIUS_M;
    ctrl->pulley_r2_m = PULLEY_R2_ENCODER_RADIUS_M;
    ctrl->clutch_current_per_torque = CLUTCH_CURRENT_PER_TORQUE_MA_NM;
    ctrl->motor_speed_compensation = MOTOR_SPEED_COMPENSATION_C;
    
    /* 初始化滤波器 - 速度和压力需要滤波 */
    /* 速度使用滑动窗口平均滤波 + 低通滤波，双重滤波消除编码器分辨率导致的阶梯 */
    ma_filter_init(&ctrl->velocity_filter);
    lpf_init(&ctrl->velocity_lpf, SPEED_LPF_ALPHA);
    /* 初始化压力平均值滤波 */
    memset(ctrl->pressure_buffer, 0, sizeof(ctrl->pressure_buffer));
    ctrl->pressure_buffer_index = 0;
    ctrl->pressure_buffer_count = 0;
    ctrl->pressure_f0_kg = PRESSURE_F0_DEFAULT_KG;
    ctrl->pressure_f0_calibrated = 0;

    diff_init(&ctrl->position_diff, 0.0f);  /* 位置微分不使用LPF，避免双重滤波 */
    
    /* 初始化PID控制器 */
    pid_init(&ctrl->pid, PID_KP, PID_KI, PID_KD, 
             PID_OUTPUT_MIN, PID_OUTPUT_MAX, ALGO_CONTROL_PERIOD_S);
    ctrl->pid.integral_limit = PID_INTEGRAL_LIMIT;
    
    /* 初始化安全监控 */
    safety_monitor_init(&ctrl->safety);
    
    /* 初始化状态 */
    ctrl->status.state = ALGO_STATE_INIT;
    ctrl->status.error = ALGO_ERR_NONE;
    ctrl->status.cycle_count = 0;
    ctrl->status.error_count = 0;
    ctrl->status.running_time_s = 0.0f;
    ctrl->status.emergency_stop = 0;
    
    /* 初始化统计 */
    ctrl->max_pressure_kg = -9999.0f;
    ctrl->min_pressure_kg = 9999.0f;
    ctrl->max_velocity_m_s = 0.0f;
    
    /* 初始化线程控制 */
    ctrl->running = 0;
    ctrl->paused = 0;
    if (pthread_mutex_init(&ctrl->mutex, NULL) != 0) {
        return -1;
    }
    
    ctrl->first_run = 1;
    
    printf("[GRAVITY_UNLOAD] Controller initialized\n");
    printf("  Pulley R1: %.3f m, R2: %.3f m\n", ctrl->pulley_r1_m, ctrl->pulley_r2_m);
    printf("  Clutch current/torque: %.2f mA/Nm\n", ctrl->clutch_current_per_torque);
    printf("  Control period: %d ms\n", ALGO_CONTROL_PERIOD_MS);
    
    return 0;
}

void gravity_unload_deinit(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    gravity_unload_stop(ctrl);
    pthread_mutex_destroy(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Controller deinitialized\n");
}

/******************************************************************************
 * 线程控制
 ******************************************************************************/

int gravity_unload_start(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return -1;
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (ctrl->running) {
        pthread_mutex_unlock(&ctrl->mutex);
        return 0; /* 已经在运行 */
    }
    
    ctrl->running = 1;
    ctrl->paused = 0;
    ctrl->status.state = ALGO_STATE_RUNNING;
    
    if (pthread_create(&ctrl->thread_id, NULL, gravity_unload_thread, ctrl) != 0) {
        ctrl->running = 0;
        pthread_mutex_unlock(&ctrl->mutex);
        return -1;
    }
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Control thread started\n");
    return 0;
}

void gravity_unload_stop(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    
    if (!ctrl->running) {
        pthread_mutex_unlock(&ctrl->mutex);
        return;
    }
    
    ctrl->running = 0;
    ctrl->status.state = ALGO_STATE_SHUTDOWN;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    pthread_join(ctrl->thread_id, NULL);
    
    printf("[GRAVITY_UNLOAD] Control thread stopped\n");
}

void gravity_unload_pause(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->paused = 1;
    ctrl->status.state = ALGO_STATE_PAUSED;
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Control paused\n");
}

void gravity_unload_resume(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    ctrl->paused = 0;
    ctrl->status.state = ALGO_STATE_RUNNING;
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Control resumed\n");
}

void gravity_unload_emergency_stop(GravityUnloadController_t *ctrl, const char *reason) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock(&ctrl->mutex);
    
    ctrl->status.emergency_stop = 1;
    ctrl->status.state = ALGO_STATE_EMERGENCY_STOP;
    
    /* 触发安全监控的紧急停止 */
    safety_trigger_emergency_stop(&ctrl->safety, reason);
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] EMERGENCY STOP: %s\n", reason ? reason : "Unknown");
}

void gravity_unload_reset(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;

    pthread_mutex_lock(&ctrl->mutex);

    /* 重置滤波器 */
    ma_filter_reset(&ctrl->velocity_filter);
    lpf_reset(&ctrl->velocity_lpf);
    /* 重置压力平均值滤波 */
    memset(ctrl->pressure_buffer, 0, sizeof(ctrl->pressure_buffer));
    ctrl->pressure_buffer_index = 0;
    ctrl->pressure_buffer_count = 0;
    ctrl->pressure_f0_kg = PRESSURE_F0_DEFAULT_KG;
    ctrl->pressure_f0_calibrated = 0;
    diff_reset(&ctrl->position_diff);
    
    /* 重置PID */
    pid_reset(&ctrl->pid);
    
    /* 重置安全监控 */
    safety_clear_emergency_stop(&ctrl->safety);
    
    /* 重置状态 */
    ctrl->status.state = ALGO_STATE_INIT;
    ctrl->status.error = ALGO_ERR_NONE;
    ctrl->status.cycle_count = 0;
    ctrl->status.error_count = 0;
    ctrl->status.running_time_s = 0.0f;
    ctrl->status.emergency_stop = 0;
    
    ctrl->first_run = 1;
    ctrl->paused = 0;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    printf("[GRAVITY_UNLOAD] Controller reset\n");
}

/******************************************************************************
 * 压力平均值滤波
 ******************************************************************************/

static float pressure_filter_update(GravityUnloadController_t *ctrl, float raw_pressure) {
    if (ctrl == NULL) return 0.0f;

    /* 将新值加入缓冲区 */
    ctrl->pressure_buffer[ctrl->pressure_buffer_index] = raw_pressure;
    ctrl->pressure_buffer_index = (ctrl->pressure_buffer_index + 1) % PRESSURE_FILTER_WINDOW_SIZE;

    if (ctrl->pressure_buffer_count < PRESSURE_FILTER_WINDOW_SIZE) {
        ctrl->pressure_buffer_count++;
    }

    /* 计算平均值 */
    float sum = 0.0f;
    for (int i = 0; i < ctrl->pressure_buffer_count; i++) {
        sum += ctrl->pressure_buffer[i];
    }

    return sum / ctrl->pressure_buffer_count;
}

/******************************************************************************
 * F0校准 - 静止时压力值校准
 ******************************************************************************/

static void calibrate_pressure_f0(GravityUnloadController_t *ctrl, float pressure_kg) {
    if (ctrl == NULL) return;

    /* 首次运行或需要重新校准时，使用当前压力值作为F0 */
    if (!ctrl->pressure_f0_calibrated) {
        /* 等待缓冲区填满后再校准 */
        if (ctrl->pressure_buffer_count >= PRESSURE_FILTER_WINDOW_SIZE) {
            ctrl->pressure_f0_kg = pressure_kg;
            ctrl->pressure_f0_calibrated = 1;
            printf("[GRAVITY_UNLOAD] F0 calibrated: %.3f kg\n", ctrl->pressure_f0_kg);
        }
    }
}

/******************************************************************************
 * 基于摩擦力的电机速度计算
 ******************************************************************************/

static float calculate_motor_speed_by_friction(GravityUnloadController_t *ctrl,
                                                float filtered_pressure_kg,
                                                float pulley_velocity_rpm) {
    if (ctrl == NULL) return 0.0f;

    /* 如果F0未校准，返回0 */
    if (!ctrl->pressure_f0_calibrated) {
        return 0.0f;
    }

    /* 计算摩擦力变化量 deltaf = (F0 - Fnow) / 2 / cos(30.5°)
     * 其中 cos(30.5°) ≈ 0.861
     */
    float deltaf = (ctrl->pressure_f0_kg - filtered_pressure_kg) / 2.0f / FRICTION_ANGLE_COS;

    /* 根据deltaf确定电机速度
     * deltaf > 0: 摩擦力减小，需要增加电机速度 -> Wmotor = W + C
     * deltaf < 0: 摩擦力增大，需要减小电机速度 -> Wmotor = W - C
     * deltaf = 0: 保持当前速度
     */
    float motor_speed_rpm;
    if (deltaf > 0.0f) {
        motor_speed_rpm = -(pulley_velocity_rpm + FRICTION_SPEED_OFFSET_C);
    } else if (deltaf < 0.0f) {
        motor_speed_rpm = -(pulley_velocity_rpm - FRICTION_SPEED_OFFSET_C); 
    } else {
        motor_speed_rpm = -pulley_velocity_rpm;
    }

    return motor_speed_rpm;
}

/******************************************************************************
 * 传感器数据处理
 ******************************************************************************/

static void process_sensor_data(GravityUnloadController_t *ctrl,
                                const SensorDataRaw_t *raw,
                                SensorDataFiltered_t *filtered) {
    if (ctrl == NULL || raw == NULL || filtered == NULL) return;

    /* 压力平均值滤波 */
    float filtered_pressure = pressure_filter_update(ctrl, raw->pressure_kg);
    filtered->pressure_kg = filtered_pressure;

    /* F0校准 */
    calibrate_pressure_f0(ctrl, filtered_pressure);

    /* 压力微分（变化率）- 如需启用可取消注释
    filtered->pressure_derivative = diff_update(&ctrl->pressure_diff,
                                                 filtered->pressure_kg,
                                                 raw->timestamp_ms);
    */
    filtered->pressure_derivative = 0.0f;

    /* 位置使用原始值 */
    filtered->position_m = raw->encoder_position_m;

    /* 速度计算：使用M/T法测速数据（传感器线程提供）
     *
     * 原理：
     * 传感器线程使用严格的10ms周期采集编码器，并计算：
     * - encoder_pulse_delta: 脉冲变化量
     * - encoder_time_delta_us: 时间变化量（微秒级精度）
     *
     * 速度 = 位置变化量 / 时间变化量
     *
     * 关键改进：
     * 1. 使用传感器线程的微秒级时间戳，确保时间差准确
     * 2. 避免控制线程周期不稳定对速度计算的影响
     * 3. M/T法测速在低速和高速时都有良好精度
     */
    float raw_velocity = 0.0f;

    /* 使用M/T法数据计算速度 */
    if (raw->encoder_time_delta_us > 0) {
        /* 将脉冲变化量转换为位置变化量
         * 编码器参数：
         * - 分辨率：4096 脉冲/圈
         * - 绳轮周长：314.16 mm/圈 (直径100mm)
         * - 1脉冲 = 314.16/4096 mm = 0.0767 mm = 7.67e-5 m
         */
        const float METERS_PER_PULSE = 7.67e-5f;  /* 314.16/4096/1000 */
        float position_delta_m = raw->encoder_pulse_delta * METERS_PER_PULSE;
        float time_delta_s = raw->encoder_time_delta_us / 1000000.0f;  /* 转换为秒 */

        raw_velocity = position_delta_m / time_delta_s;

        /* 调试输出 - 每100次输出一次 */
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            printf("[VEL MT] pulse_delta=%d, time_us=%u, pos_delta=%.6f, vel=%.6f\n",
                   raw->encoder_pulse_delta, raw->encoder_time_delta_us, position_delta_m, raw_velocity);
        }
    }

    filtered->velocity_raw_m_s = raw_velocity;
    /* 双重滤波：滑动平均 + 低通滤波，消除编码器分辨率导致的阶梯效应 */
    float velocity_ma = ma_filter_update(&ctrl->velocity_filter, raw_velocity);
    filtered->velocity_m_s = lpf_update(&ctrl->velocity_lpf, velocity_ma);

    filtered->timestamp_ms = raw->timestamp_ms;
}

/******************************************************************************
 * 控制输出计算
 ******************************************************************************/

static void calculate_control_output(GravityUnloadController_t *ctrl,
                                     const SensorDataFiltered_t *filtered,
                                     ControlOutput_t *output) {
    if (ctrl == NULL || filtered == NULL || output == NULL) return;

    /* 步骤1: 计算离合器目标转矩 */
    /* 力矩 = 压力(kg) * g * R2 */
    /* 简化: 直接按比例计算电流 */
    float force_n = filtered->pressure_kg * 9.81f;  /* 转换为牛顿 */
    float torque_nm = force_n * ctrl->pulley_r2_m;   /* 力矩 = 力 * 半径 */

    /* 计算目标电流 */
    output->clutch_torque_nm = torque_nm;
    output->clutch_current_mA = torque_nm * ctrl->clutch_current_per_torque;

    /* 增加基础电流，确保即使小压力也能有电流输出 */
    output->clutch_current_mA += 50.0f;  /* 基础电流50mA */

    /* 限制电流范围 */
    if (output->clutch_current_mA < 50.0f) {  /* 直接限制为50mA */
        output->clutch_current_mA = 50.0f;
    } else if (output->clutch_current_mA > SAFETY_CLUTCH_CURRENT_MAX_MA) {
        output->clutch_current_mA = SAFETY_CLUTCH_CURRENT_MAX_MA;
    }

    /* 步骤2: 计算电机目标速度 - 基于摩擦力的控制算法 */
    /* 首先计算大滑轮（顶部）的转速（rpm）
     * 线速度(m/s) -> 转速(rpm): W = V * 60 / (2 * π * R)
     * 简化: W = V * 30 / (π * R)
     */
    float pulley_velocity_rpm = (filtered->velocity_m_s / ctrl->pulley_r1_m) * (30.0f / 3.14159f);

    /* 使用基于摩擦力的控制算法计算电机目标速度
     * 根据deltaf的正负，在滑轮转速基础上加减常量C
     */
    float target_motor_speed = calculate_motor_speed_by_friction(ctrl,
                                                                  filtered->pressure_kg,
                                                                  pulley_velocity_rpm);

    /* 限制电机速度范围 */
    if (target_motor_speed > SAFETY_MOTOR_SPEED_MAX) {
        target_motor_speed = SAFETY_MOTOR_SPEED_MAX;
    } else if (target_motor_speed < -SAFETY_MOTOR_SPEED_MAX) {
        target_motor_speed = -SAFETY_MOTOR_SPEED_MAX;
    }

    /* 保存新算法计算的目标速度（用于数据记录） */
    output->motor_velocity_target = target_motor_speed;

    /* 使用PID控制器跟踪目标速度 */
    float pid_output = pid_update(&ctrl->pid, target_motor_speed, output->motor_velocity_actual);

    output->motor_velocity_cmd = pid_output;
    output->timestamp_ms = filtered->timestamp_ms;
}

/******************************************************************************
 * 控制周期
 ******************************************************************************/

AlgoError_t gravity_unload_control_cycle(GravityUnloadController_t *ctrl,
                                          const SensorDataRaw_t *raw_data,
                                          SensorDataFiltered_t *filtered_data,
                                          ControlOutput_t *control_output) {
    if (ctrl == NULL || raw_data == NULL || filtered_data == NULL || control_output == NULL) {
        return ALGO_ERR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&ctrl->mutex);
    
    /* 检查紧急停止 */
    if (ctrl->status.emergency_stop) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ALGO_ERR_SAFETY_VIOLATION;
    }
    
    /* 检查暂停 */
    if (ctrl->paused) {
        pthread_mutex_unlock(&ctrl->mutex);
        return ALGO_ERR_NONE;
    }
    
    /* 更新统计 */
    if (raw_data->pressure_kg > ctrl->max_pressure_kg) {
        ctrl->max_pressure_kg = raw_data->pressure_kg;
    }
    if (raw_data->pressure_kg < ctrl->min_pressure_kg) {
        ctrl->min_pressure_kg = raw_data->pressure_kg;
    }
    
    /* 步骤1: 处理传感器数据 */
    process_sensor_data(ctrl, raw_data, filtered_data);
    
    /* 步骤2: 获取电机实际速度并转换为滑轮转速 */
    /* 电机转速(rpm) -> 滑轮转速(rpm) = 电机转速 / 减速比3 */
    float actual_velocity = 0.0f;
    get_motor_actual_velocity(&actual_velocity);
    control_output->motor_velocity_actual = actual_velocity / 3.0f;  /* 除以减速比3 */
    
    /* 步骤3: 计算控制输出 */
    calculate_control_output(ctrl, filtered_data, control_output);
    
    /* 步骤4: 安全监控检查 */
    safety_monitor_update(&ctrl->safety, filtered_data, control_output, actual_velocity);
    SafetyStatus_t safety_status = safety_check(&ctrl->safety);
    
    if (safety_status == SAFETY_STATUS_EMERGENCY) {
        ctrl->status.state = ALGO_STATE_EMERGENCY_STOP;
        ctrl->status.error = ALGO_ERR_SAFETY_VIOLATION;
        ctrl->status.error_count++;
        pthread_mutex_unlock(&ctrl->mutex);
        return ALGO_ERR_SAFETY_VIOLATION;
    } else if (safety_status == SAFETY_STATUS_ERROR) {
        ctrl->status.error = ctrl->safety.error_code;
        ctrl->status.error_count++;
    }
    
    /* 更新统计 */
    if (fabsf(filtered_data->velocity_m_s) > ctrl->max_velocity_m_s) {
        ctrl->max_velocity_m_s = fabsf(filtered_data->velocity_m_s);
    }
    
    /* 更新时间 */
    if (ctrl->first_run) {
        ctrl->last_timestamp_ms = raw_data->timestamp_ms;
        ctrl->first_run = 0;
    } else {
        uint32_t dt_ms = raw_data->timestamp_ms - ctrl->last_timestamp_ms;
        ctrl->status.running_time_s += dt_ms / 1000.0f;
        ctrl->last_timestamp_ms = raw_data->timestamp_ms;
    }
    
    ctrl->status.cycle_count = ++ctrl->cycle_count;
    
    pthread_mutex_unlock(&ctrl->mutex);
    
    return ALGO_ERR_NONE;
}

/******************************************************************************
 * 线程函数
 ******************************************************************************/

static void* gravity_unload_thread(void *arg) {
    GravityUnloadController_t *ctrl = (GravityUnloadController_t *)arg;
    
    printf("[GRAVITY_UNLOAD] Thread started - Industrial Grade Strict 10ms Cycle\n");
    
    SensorDataRaw_t raw_data;
    SensorDataFiltered_t filtered_data;
    ControlOutput_t control_output;
    
    uint32_t last_print_time = 0;
    
    /* 工业级严格周期控制 - 使用绝对时间戳 */
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    uint64_t next_time_us = next_time.tv_sec * 1000000ULL + next_time.tv_nsec / 1000;
    
    while (1) {
        pthread_mutex_lock(&ctrl->mutex);
        int should_run = ctrl->running;
        pthread_mutex_unlock(&ctrl->mutex);
        
        if (!should_run) {
            break;
        }
        
        /* 获取传感器数据 */
        if (get_sensor_data(&raw_data) != 0) {
            printf("[GRAVITY_UNLOAD] Failed to get sensor data\n");
            /* 即使获取数据失败，也要保持周期 */
            next_time_us += ALGO_CONTROL_PERIOD_MS * 1000;
            uint64_t current_time_us = get_time_us();
            int64_t sleep_us = (int64_t)(next_time_us - current_time_us);
            if (sleep_us > 0) {
                struct timespec sleep_ts;
                sleep_ts.tv_sec = sleep_us / 1000000;
                sleep_ts.tv_nsec = (sleep_us % 1000000) * 1000;
                nanosleep(&sleep_ts, NULL);
            }
            continue;
        }
        
        /* 执行控制周期 */
        AlgoError_t err = gravity_unload_control_cycle(ctrl, &raw_data, &filtered_data, &control_output);
        
        if (err == ALGO_ERR_SAFETY_VIOLATION) {
            printf("[GRAVITY_UNLOAD] Safety violation detected!\n");
            /* 紧急停止：输出清零 */
            set_motor_velocity(0);
            set_clutch_current(0);
            break;
        }
        
        /* 输出到执行器 */
        set_motor_velocity(control_output.motor_velocity_cmd);  /* 负号反转电机方向 */
        set_clutch_current(control_output.clutch_current_mA);
        
        /* 更新绳子速度到共享状态缓冲区 */
        update_rope_velocity(filtered_data.velocity_raw_m_s, filtered_data.velocity_m_s);
        
        /* 计算电机理论线速度并更新到共享状态缓冲区 */
        /* 理论线速度 = 新算法计算的目标电机速度对应的线速度 */
        /* control_output.motor_velocity_target 是基于摩擦力的控制算法计算的目标速度(rpm) */
        /* 注意：需要除以减速比3，将电机转速转换为滑轮转速 */
        float pulley_rpm = control_output.motor_velocity_target;  /* 电机rpm -> 滑轮rpm */
        float theory_linear_vel = -(pulley_rpm * 2.0f * 3.14159f * ctrl->pulley_r1_m) / 60.0f;
        update_motor_theory_velocity(theory_linear_vel);

        /* 计算并更新F0和dF到共享状态缓冲区 */
        float deltaf = 0.0f;
        if (ctrl->pressure_f0_calibrated) {
            deltaf = (ctrl->pressure_f0_kg - filtered_data.pressure_kg) / 2.0f / FRICTION_ANGLE_COS;
        }
        update_pressure_f0_deltaf(ctrl->pressure_f0_kg, deltaf);

        /* 1Hz调试打印并输出到文件供上位机读取 */
        uint32_t current_time = get_timestamp_ms();
        if (current_time - last_print_time >= 1000) {  /* 1Hz输出 */
            last_print_time = current_time;

            /* 计算电机线速度 (电机转一圈对应的绳子移动距离考虑减速比) */
            /* 电机速度(rpm) -> 滑轮速度(rpm) -> 线速度(m/s) */
            //换算到滑轮线速度，不考虑减速比
            float pulley_speed_rpm = control_output.motor_velocity_cmd;
            float motor_linear_vel = (pulley_speed_rpm * 2.0f * 3.14159f * ctrl->pulley_r1_m) / 60.0f;

            /* 计算摩擦力变化量用于显示 */
            float deltaf = 0.0f;
            if (ctrl->pressure_f0_calibrated) {
                deltaf = (ctrl->pressure_f0_kg - filtered_data.pressure_kg) / 2.0f / FRICTION_ANGLE_COS;
            }

            /* 控制台输出 - 仅输出到控制台，避免文件I/O阻塞 */
            printf("[DATA] P=%.3fkg F0=%.3fkg dF=%.3f Pos=%.3fm V_raw=%.3f V_filt=%.3f V_motor_linear=%.3f I_clutch=%.1fmA V_motor_rpm=%.1f\n",
                   filtered_data.pressure_kg,
                   ctrl->pressure_f0_kg,
                   deltaf,
                   filtered_data.position_m,
                   filtered_data.velocity_raw_m_s,
                   filtered_data.velocity_m_s,
                   motor_linear_vel,
                   control_output.clutch_current_mA,
                   control_output.motor_velocity_cmd);
        }
        
        /* 工业级严格周期控制 - 使用绝对时间戳 */
        next_time_us += ALGO_CONTROL_PERIOD_MS * 1000;  /* 下一个周期的时间点 */
        
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
            printf("[GRAVITY_UNLOAD] WARNING: Cycle deadline missed by %ld us, resynchronizing...\n", 
                   (long)(-sleep_us));
            /* 重新同步到下一个周期 */
            next_time_us = current_time_us + ALGO_CONTROL_PERIOD_MS * 1000;
        }
        /* 如果sleep_us在[-5000, 0]之间，说明轻微超时，继续执行不等待 */
    }
    
    printf("[GRAVITY_UNLOAD] Thread stopped\n");
    return NULL;
}

/******************************************************************************
 * 状态查询与调试
 ******************************************************************************/

void gravity_unload_get_status(const GravityUnloadController_t *ctrl, AlgoStatus_t *status) {
    if (ctrl == NULL || status == NULL) return;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    memcpy(status, &ctrl->status, sizeof(AlgoStatus_t));
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
}

void gravity_unload_print_status(const GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    
    pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
    
    printf("\n========== Gravity Unload Status ==========\n");
    printf("State: %d, Error: %d\n", ctrl->status.state, ctrl->status.error);
    printf("Cycle count: %u\n", ctrl->cycle_count);
    printf("Running time: %.1f s\n", ctrl->status.running_time_s);
    printf("Emergency stop: %s\n", ctrl->status.emergency_stop ? "YES" : "NO");
    printf("\nStatistics:\n");
    printf("  Pressure: %.2f to %.2f kg\n", ctrl->min_pressure_kg, ctrl->max_pressure_kg);
    printf("  Max velocity: %.3f m/s\n", ctrl->max_velocity_m_s);
    printf("\nSafety Status: %s\n", safety_status_to_string(ctrl->safety.status));
    if (ctrl->safety.error_msg[0] != '\0') {
        printf("  Message: %s\n", ctrl->safety.error_msg);
    }
    printf("==========================================\n");
    
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
}
