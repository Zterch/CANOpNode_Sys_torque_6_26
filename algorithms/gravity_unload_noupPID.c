/******************************************************************************
 * @file    gravity_unload.c
 * @brief   重力卸载控制算法实现 - 工业级100Hz版本（直接调用电源批量读写）
 * @author  System Architect
 * @date    2026-06-12
 * @version 2.0.1
 ******************************************************************************/

#include "gravity_unload.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include "power_driver.h"   // 电源批量读写

/* 外部依赖声明 - 需要在主程序中提供 */
extern int get_sensor_data(SensorDataRaw_t *data);
extern int set_motor_velocity(float velocity);
extern int get_motor_actual_velocity(float *velocity);
extern uint32_t get_timestamp_ms(void);
extern void update_rope_velocity(float raw_velocity, float filtered_velocity);
extern void update_motor_theory_velocity(float theory_velocity);
extern void update_pressure_f0_deltaf(float f0_kg, float deltaf);
extern void update_pi_terms(float p_term_mA, float i_term_mA, float d_term_mA);
extern void update_feedforward_current(float feedforward_mA);
extern void update_feedforward_and_target(float feedforward_mA, float target_mA, float deltaf_kg, float pressure_kg);
extern void update_pi_last_current(float last_current_mA);
extern void update_power_feedback(float current_a, float voltage_v, float target_a);  // 新增：更新电源反馈数据
extern PowerDriver_t g_power;   // 电源驱动全局变量

/* 摩擦力方向控制全局变量 */
int g_friction_direction_mode = 2;

static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static void* gravity_unload_thread(void *arg);
static void process_sensor_data(GravityUnloadController_t *ctrl,
                                const SensorDataRaw_t *raw,
                                SensorDataFiltered_t *filtered);
static void calculate_control_output(GravityUnloadController_t *ctrl,
                                     const SensorDataFiltered_t *filtered,
                                     ControlOutput_t *output);
static void calibrate_pressure_f0(GravityUnloadController_t *ctrl, float pressure_kg);
static float_t integral_sum = 0.0f;

/* 未使用的函数已注释，避免警告 */
// static float pressure_filter_update(...) { ... }
// static float calculate_motor_speed_by_friction(...) { ... }

/******************************************************************************
 * 初始化与反初始化
 ******************************************************************************/
int gravity_unload_init(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return -1;
    volatile GravityUnloadController_t *vctrl = ctrl;
    memset((void *)vctrl, 0, sizeof(GravityUnloadController_t));
    ctrl->pressure_f0_kg = 0.0f;
    ctrl->pressure_f0_calibrated = 0;
    ctrl->pressure_f0_sum = 0.0f;
    ctrl->pressure_f0_sample_count = 0;
    ctrl->pressure_stabilize_count = 0;
    ctrl->last_current_mA = 0.0f;
    ctrl->pulley_r1_m = PULLEY_R1_MOTOR_RADIUS_M;
    ctrl->pulley_r2_m = PULLEY_R2_ENCODER_RADIUS_M;
    ctrl->clutch_current_per_torque = CLUTCH_CURRENT_PER_TORQUE_MA_NM;
    ctrl->motor_speed_compensation = MOTOR_SPEED_COMPENSATION_C;
    ma_filter_init(&ctrl->velocity_filter);
    lpf_init(&ctrl->velocity_lpf, SPEED_LPF_ALPHA);
    memset(ctrl->pressure_buffer, 0, sizeof(ctrl->pressure_buffer));
    ctrl->pressure_buffer_index = 0;
    ctrl->pressure_buffer_count = 0;
    diff_init(&ctrl->position_diff, 0.0f);
    pid_init(&ctrl->pid, PID_KP, PID_KI, PID_KD,
             PID_OUTPUT_MIN, PID_OUTPUT_MAX, ALGO_CONTROL_PERIOD_S);
    ctrl->pid.integral_limit = PID_INTEGRAL_LIMIT;
    ctrl->clutch_pi_integral = 0.0f;
    ctrl->clutch_pi_derivative = 0.0f;
    ctrl->last_deltaf = 0.0f;
    ctrl->last_last_deltaf = 0.0f;
    ctrl->last_current_mA = 0.0f;
    integral_sum = 0.0f;
    safety_monitor_init(&ctrl->safety);
    ctrl->status.state = ALGO_STATE_INIT;
    ctrl->status.error = ALGO_ERR_NONE;
    ctrl->status.cycle_count = 0;
    ctrl->status.error_count = 0;
    ctrl->status.running_time_s = 0.0f;
    ctrl->status.emergency_stop = 0;
    ctrl->max_pressure_kg = -9999.0f;
    ctrl->min_pressure_kg = 9999.0f;
    ctrl->max_velocity_m_s = 0.0f;
    ctrl->running = 0;
    ctrl->paused = 0;
    if (pthread_mutex_init(&ctrl->mutex, NULL) != 0) return -1;
    ctrl->first_run = 1;
    printf("[GRAVITY_UNLOAD] Controller initialized\n");
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
    if (ctrl->running) { pthread_mutex_unlock(&ctrl->mutex); return 0; }
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
    if (!ctrl->running) { pthread_mutex_unlock(&ctrl->mutex); return; }
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
    safety_trigger_emergency_stop(&ctrl->safety, reason);
    pthread_mutex_unlock(&ctrl->mutex);
    printf("[GRAVITY_UNLOAD] EMERGENCY STOP: %s\n", reason ? reason : "Unknown");
}

void gravity_unload_reset(GravityUnloadController_t *ctrl) {
    if (ctrl == NULL) return;
    pthread_mutex_lock(&ctrl->mutex);
    ma_filter_reset(&ctrl->velocity_filter);
    lpf_reset(&ctrl->velocity_lpf);
    memset(ctrl->pressure_buffer, 0, sizeof(ctrl->pressure_buffer));
    ctrl->pressure_buffer_index = 0;
    ctrl->pressure_buffer_count = 0;
    ctrl->pressure_f0_kg = PRESSURE_F0_DEFAULT_KG;
    ctrl->pressure_f0_calibrated = 0;
    ctrl->pressure_f0_sum = 0.0f;
    ctrl->pressure_f0_sample_count = 0;
    ctrl->pressure_stabilize_count = 0;
    diff_reset(&ctrl->position_diff);
    pid_reset(&ctrl->pid);
    ctrl->clutch_pi_integral = 0.0f;
    ctrl->clutch_pi_derivative = 0.0f;
    ctrl->last_deltaf = 0.0f;
    ctrl->last_last_deltaf = 0.0f;
    ctrl->last_current_mA = 0.0f;
    integral_sum = 0.0f;
    safety_clear_emergency_stop(&ctrl->safety);
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
 * F0校准
 ******************************************************************************/
static void calibrate_pressure_f0(GravityUnloadController_t *ctrl, float pressure_kg) {
    if (ctrl == NULL) return;
    if (!ctrl->pressure_f0_calibrated) {
        ctrl->pressure_f0_kg = 0.0f;
        if (ctrl->pressure_stabilize_count < PRESSURE_F0_STABILIZE_SAMPLES) {
            ctrl->pressure_stabilize_count++;
            if (ctrl->pressure_stabilize_count == PRESSURE_F0_STABILIZE_SAMPLES)
                printf("[GRAVITY_UNLOAD] Pressure stabilization complete, starting F0 calibration...\n");
            return;
        }
        ctrl->pressure_f0_sum += pressure_kg;
        ctrl->pressure_f0_sample_count++;
        if (ctrl->pressure_f0_sample_count >= PRESSURE_F0_CALIBRATION_SAMPLES) {
            ctrl->pressure_f0_kg = ctrl->pressure_f0_sum / ctrl->pressure_f0_sample_count;
            ctrl->pressure_f0_calibrated = 1;
            integral_sum = 0.0f;
            ctrl->last_current_mA = 0.0f;
            printf("[GRAVITY_UNLOAD] F0 calibrated: %.3f kg (avg of %d samples)\n",
                   ctrl->pressure_f0_kg, ctrl->pressure_f0_sample_count);
        }
    }
}

/******************************************************************************
 * 传感器数据处理
 ******************************************************************************/
static void process_sensor_data(GravityUnloadController_t *ctrl,
                                const SensorDataRaw_t *raw,
                                SensorDataFiltered_t *filtered) {
    if (ctrl == NULL || raw == NULL || filtered == NULL) return;
    float filtered_pressure = raw->pressure_kg;
    filtered->pressure_kg = filtered_pressure;
    calibrate_pressure_f0(ctrl, filtered_pressure);
    filtered->pressure_derivative = 0.0f;
    filtered->position_m = raw->encoder_position_m;
    float raw_velocity = 0.0f;
    if (raw->encoder_time_delta_us > 0) {
        const float METERS_PER_PULSE = 7.67e-5f;
        float position_delta_m = raw->encoder_pulse_delta * METERS_PER_PULSE;
        float time_delta_s = raw->encoder_time_delta_us / 1000000.0f;
        raw_velocity = position_delta_m / time_delta_s;
    }
    filtered->velocity_raw_m_s = raw_velocity;
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
    float delta_f_kg = 0.0f;
    if (ctrl->pressure_f0_calibrated) delta_f_kg = filtered->pressure_kg - ctrl->pressure_f0_kg;
    float feedforward_current_mA = 50.0f + delta_f_kg * 273.0f;
    if (feedforward_current_mA < 50.0f) feedforward_current_mA = 50.0f;
    else if (feedforward_current_mA > 900.0f) feedforward_current_mA = 900.0f;
    float deadzone_deltaf = fabs(delta_f_kg) > PRESSURE_DEADZONE_KG ? delta_f_kg : 0.0f;
    float proportional_increment = CLUTCH_PI_KP * deadzone_deltaf;
    if (fabs(delta_f_kg) > PRESSURE_DEADZONE_KG) integral_sum += delta_f_kg * ALGO_CONTROL_PERIOD_S;
    float integral_increment = CLUTCH_PI_KI * integral_sum;
    float derivative_increment = CLUTCH_PI_KD * (deadzone_deltaf - 2.0f * ctrl->last_deltaf + ctrl->last_last_deltaf);
    float total_increment = proportional_increment + integral_increment + derivative_increment;
    float pi_current_mA = ctrl->last_current_mA * 0.0 + total_increment;
    if (deadzone_deltaf == 0.0f && fabs(ctrl->last_current_mA) > 5.0f)
        pi_current_mA = ctrl->last_current_mA * 0.95f;
    ctrl->clutch_pi_integral = integral_increment;
    if (ctrl->clutch_pi_integral > CLUTCH_PI_INTEGRAL_LIMIT) ctrl->clutch_pi_integral = CLUTCH_PI_INTEGRAL_LIMIT;
    else if (ctrl->clutch_pi_integral < -CLUTCH_PI_INTEGRAL_LIMIT) ctrl->clutch_pi_integral = -CLUTCH_PI_INTEGRAL_LIMIT;
    ctrl->clutch_pi_derivative = derivative_increment;
    float pi_p_term_mA = CLUTCH_PI_KP * delta_f_kg;
    float pi_i_term_mA = ctrl->clutch_pi_integral;
    float pi_d_term_mA = ctrl->clutch_pi_derivative;
    float current_mA = pi_current_mA + feedforward_current_mA;
    float current_mA_raw = current_mA;
    int current_limited = 0;
    if (current_mA < 0.0f) { current_mA = 0.0f; current_limited = -1; }
    else if (current_mA > SAFETY_CLUTCH_CURRENT_MAX_MA) { current_mA = SAFETY_CLUTCH_CURRENT_MAX_MA; current_limited = 1; }
    if (ctrl->pressure_f0_calibrated) {
        ctrl->last_last_deltaf = ctrl->last_deltaf;
        ctrl->last_deltaf = delta_f_kg;
        if (current_limited == -1 && pi_current_mA < 0.0f) ctrl->last_current_mA = 0.0f;
        else if (current_limited == 1 && pi_current_mA > SAFETY_CLUTCH_CURRENT_MAX_MA) ctrl->last_current_mA = SAFETY_CLUTCH_CURRENT_MAX_MA;
        else ctrl->last_current_mA = pi_current_mA;
    } else {
        ctrl->last_last_deltaf = 0.0f;
        ctrl->last_deltaf = 0.0f;
        ctrl->last_current_mA = 0.0f;
    }
    output->clutch_current_mA = current_mA;
    update_pi_terms(pi_p_term_mA, pi_i_term_mA, pi_d_term_mA);
    update_pi_last_current(ctrl->last_current_mA);
    update_feedforward_and_target(feedforward_current_mA, current_mA_raw, delta_f_kg, filtered->pressure_kg);
    if (output->clutch_current_mA < 0.0f) output->clutch_current_mA = 0.0f;
    else if (output->clutch_current_mA > SAFETY_CLUTCH_CURRENT_MAX_MA) output->clutch_current_mA = SAFETY_CLUTCH_CURRENT_MAX_MA;
    ctrl->last_deltaf = delta_f_kg;
    float pulley_velocity_rpm = (filtered->velocity_m_s / ctrl->pulley_r1_m) * (30.0f / 3.14159f);
    float target_motor_speed;
    if (fabsf(delta_f_kg) < FRICTION_DEADZONE_KG) {
        target_motor_speed = -pulley_velocity_rpm;
    } else {
        switch (g_friction_direction_mode) {
            case 1:
                target_motor_speed = (delta_f_kg < 0.0f) ? -60.0f : -60.0f;
                break;
            case 2:
                target_motor_speed = (delta_f_kg > 0.0f) ? 60.0f : -60.0f;
                break;
            default:
                if (delta_f_kg > 0.0f) target_motor_speed = 60.0f;
                else if (delta_f_kg < 0.0f) target_motor_speed = -60.0f;
                else target_motor_speed = -60.0f;
                break;
        }
    }
    if (target_motor_speed > SAFETY_MOTOR_SPEED_MAX) target_motor_speed = SAFETY_MOTOR_SPEED_MAX;
    else if (target_motor_speed < -SAFETY_MOTOR_SPEED_MAX) target_motor_speed = -SAFETY_MOTOR_SPEED_MAX;
    output->motor_velocity_target = target_motor_speed;
    /* PID output 暂时不使用，固定电机指令为300 */
    // float pid_output = pid_update(&ctrl->pid, target_motor_speed, output->motor_velocity_actual);
    output->motor_velocity_cmd = 300.0f;
    output->timestamp_ms = filtered->timestamp_ms;
}

/******************************************************************************
 * 控制周期（主循环调用）
 ******************************************************************************/
AlgoError_t gravity_unload_control_cycle(GravityUnloadController_t *ctrl,
                                          const SensorDataRaw_t *raw_data,
                                          SensorDataFiltered_t *filtered_data,
                                          ControlOutput_t *control_output) {
    if (ctrl == NULL || raw_data == NULL || filtered_data == NULL || control_output == NULL)
        return ALGO_ERR_INVALID_PARAM;
    pthread_mutex_lock(&ctrl->mutex);
    if (ctrl->status.emergency_stop) { pthread_mutex_unlock(&ctrl->mutex); return ALGO_ERR_SAFETY_VIOLATION; }
    if (ctrl->paused) { pthread_mutex_unlock(&ctrl->mutex); return ALGO_ERR_NONE; }
    if (raw_data->pressure_kg > ctrl->max_pressure_kg) ctrl->max_pressure_kg = raw_data->pressure_kg;
    if (raw_data->pressure_kg < ctrl->min_pressure_kg) ctrl->min_pressure_kg = raw_data->pressure_kg;
    process_sensor_data(ctrl, raw_data, filtered_data);
    float actual_velocity = 0.0f;
    get_motor_actual_velocity(&actual_velocity);
    control_output->motor_velocity_actual = actual_velocity / 3.0f;
    calculate_control_output(ctrl, filtered_data, control_output);
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
    if (fabsf(filtered_data->velocity_m_s) > ctrl->max_velocity_m_s)
        ctrl->max_velocity_m_s = fabsf(filtered_data->velocity_m_s);
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
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    uint64_t next_time_us = next_time.tv_sec * 1000000ULL + next_time.tv_nsec / 1000;
    while (1) {
        pthread_mutex_lock(&ctrl->mutex);
        int should_run = ctrl->running;
        pthread_mutex_unlock(&ctrl->mutex);
        if (!should_run) break;
        if (get_sensor_data(&raw_data) != 0) {
            next_time_us += ALGO_CONTROL_PERIOD_MS * 1000;
            uint64_t current_time_us = get_time_us();
            int64_t sleep_us = (int64_t)(next_time_us - current_time_us);
            if (sleep_us > 0) {
                struct timespec sleep_ts = { sleep_us / 1000000, (sleep_us % 1000000) * 1000 };
                nanosleep(&sleep_ts, NULL);
            }
            continue;
        }
        AlgoError_t err = gravity_unload_control_cycle(ctrl, &raw_data, &filtered_data, &control_output);
        if (err == ALGO_ERR_SAFETY_VIOLATION) {
            printf("[GRAVITY_UNLOAD] Safety violation detected!\n");
            set_motor_velocity(0);
            power_set_current(&g_power, 0);
            break;
        }
        /* 输出到执行器 */
        set_motor_velocity(control_output.motor_velocity_cmd);
        /* 使用批量读写直接设置电流并读取实际值 */
        uint16_t actual_current_ma = 0, actual_voltage_mv = 0;
        uint16_t target_ma = (uint16_t)control_output.clutch_current_mA;
        ErrorCode_t ret = power_batch_control(&g_power, target_ma, &actual_current_ma, &actual_voltage_mv);
        if (ret == ERR_OK) {
            /* 通过外部函数更新共享状态中的电源数据（在 main.c 中实现） */
            update_power_feedback(actual_current_ma / 1000.0f, actual_voltage_mv / 1000.0f, control_output.clutch_current_mA / 1000.0f);
        } else {
            static int comm_err_cnt = 0;
            if (++comm_err_cnt % 100 == 0)
                printf("[WARN] power_batch_control failed, ret=%d\n", ret);
        }
        update_rope_velocity(filtered_data.velocity_raw_m_s, filtered_data.velocity_m_s);
        float pulley_rpm = control_output.motor_velocity_target;
        float theory_linear_vel = -(pulley_rpm * 2.0f * 3.14159f * ctrl->pulley_r1_m) / 60.0f;
        update_motor_theory_velocity(theory_linear_vel);
        float delta_f_kg = 0.0f;
        if (ctrl->pressure_f0_calibrated) delta_f_kg = filtered_data.pressure_kg - ctrl->pressure_f0_kg;
        update_pressure_f0_deltaf(ctrl->pressure_f0_kg, delta_f_kg);
        uint32_t current_time = get_timestamp_ms();
        if (current_time - last_print_time >= 1000) {
            last_print_time = current_time;
            float pulley_speed_rpm = control_output.motor_velocity_cmd;
            float motor_linear_vel = (pulley_speed_rpm * 2.0f * 3.14159f * ctrl->pulley_r1_m) / 60.0f;
            float delta_f_display = (ctrl->pressure_f0_calibrated) ? filtered_data.pressure_kg - ctrl->pressure_f0_kg : 0.0f;
            float pi_p_term = CLUTCH_PI_KP * delta_f_display;
            float pi_i_term = ctrl->clutch_pi_integral;
            float pi_total = pi_p_term + pi_i_term;
            printf("[DATA] P=%.3fkg F0=%.3fkg dF=%.3f PI_int=%.3f Pos=%.3fm V_raw=%.3f V_filt=%.3f V_motor_linear=%.3f I_clutch=%.1fmA V_motor_rpm=%.1f Mode=%d\n",
                   filtered_data.pressure_kg, ctrl->pressure_f0_kg, delta_f_display,
                   ctrl->clutch_pi_integral, filtered_data.position_m,
                   filtered_data.velocity_raw_m_s, filtered_data.velocity_m_s,
                   motor_linear_vel, control_output.clutch_current_mA, control_output.motor_velocity_cmd, g_friction_direction_mode);
            printf("[CURRENT_DETAIL] base=50.0mA P=%.1fmA I=%.1fmA PI_total=%.1fmA I_total=%.1fmA Mode=%d(0=双向,1=减,2=增)\n",
                   pi_p_term, pi_i_term, pi_total, control_output.clutch_current_mA, g_friction_direction_mode);
        }
        next_time_us += ALGO_CONTROL_PERIOD_MS * 1000;
        uint64_t current_time_us = get_time_us();
        int64_t sleep_us = (int64_t)(next_time_us - current_time_us);
        if (sleep_us > 0) {
            struct timespec sleep_ts = { sleep_us / 1000000, (sleep_us % 1000000) * 1000 };
            nanosleep(&sleep_ts, NULL);
        } else if (sleep_us < -5000) {
            printf("[GRAVITY_UNLOAD] WARNING: Cycle deadline missed by %ld us, resynchronizing...\n", (long)(-sleep_us));
            next_time_us = current_time_us + ALGO_CONTROL_PERIOD_MS * 1000;
        }
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
    if (ctrl->safety.error_msg[0] != '\0') printf("  Message: %s\n", ctrl->safety.error_msg);
    printf("==========================================\n");
    pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
}