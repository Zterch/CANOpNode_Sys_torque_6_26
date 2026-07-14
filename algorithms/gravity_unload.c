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
#include "power_driver.h"  /* 电源驱动头文件 */

/* 外部依赖声明 - 需要在主程序中提供 */
extern int get_sensor_data(SensorDataRaw_t *data);
extern int set_motor_velocity(float velocity);
extern int set_motor_torque(int torque);  /* 电机扭矩设置 (0.001倍额定转矩) */
extern int set_clutch_current(float current_mA);
extern int get_motor_actual_velocity(float *velocity);
extern uint32_t get_timestamp_ms(void);
extern void update_rope_velocity(float raw_velocity, float filtered_velocity);
extern void update_motor_theory_velocity(float theory_velocity);
extern void update_pressure_f0_deltaf(float f0_kg, float deltaf);
extern void update_pi_terms(float p_term_mA, float p_term_filtered_mA, float p_term_raw_mA, float i_term_mA, float d_term_mA);
extern void update_feedforward_current(float feedforward_mA);
extern void update_feedforward_and_target(float feedforward_mA, float target_mA, float deltaf_kg, float pressure_kg);
extern void update_feedforward_torque(float feedforward_torque_Nm);  /* 更新扭矩前馈 */
extern void update_pi_last_current(float last_current_mA);
extern void update_power_feedback(float current_a, float voltage_v, float target_a);  /* 更新电源反馈数据 */
extern void update_motor_target_torque(float torque_cmd);  /* 更新电机目标扭矩 */
extern void update_adrc_state(float kp, float p_gain_multiplier, float u0, float z1, float z2, float output_torque); /* 更新ADRC状态到共享内存 */

/* 电源驱动外部声明 */
extern PowerDriver_t g_power;  /* 电源驱动全局变量 */

/* 摩擦力方向控制全局变量
 * 0: 双向控制（正常模式）
 * 1: 只响应逆时针方向（deltaf > 0，摩擦力减小，重物变轻）
 * 2: 只响应顺时针方向（deltaf < 0，摩擦力增大，重物变重）
 */
int g_friction_direction_mode = 1;  /* 默认只响应逆时针方向 */

/* ADRC + 双阈值动态P融合控制参数
 * 从6_15工程移植：当|DeltaF|增大并超过阈值时，自动放大P增益以提升响应；
 * 当|DeltaF|减小时，按两级阶梯下降，避免超调和振荡。
 */
float g_deltaf_threshold_kg = ADRC_DYNAMIC_P_THRESHOLD_1;   /* 第一级阈值 */
float g_deltaf_threshold_kg_2 = ADRC_DYNAMIC_P_THRESHOLD_2; /* 第二级阈值 */
static float g_last_deltaf_for_pd_boost = 0.0f;             /* 用于检测DeltaF变化趋势 */
static float g_current_pd_gain_multiplier = 1.0f;           /* 当前动态P增益倍数 */

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
    
    /* 使用volatile防止编译器优化，确保所有字段被清零 */
    volatile GravityUnloadController_t *vctrl = ctrl;
    memset((void *)vctrl, 0, sizeof(GravityUnloadController_t));
    
    /* 显式清零关键字段（不依赖memset） */
    ctrl->pressure_f0_kg = 0.0f;
    ctrl->pressure_f0_calibrated = 0;
    ctrl->pressure_f0_sum = 0.0f;
    ctrl->pressure_f0_sample_count = 0;
    ctrl->pressure_stabilize_count = 0;  /* 稳定延迟计数器清零 */
    ctrl->last_output_Nm = 0.0f;
    
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

    diff_init(&ctrl->position_diff, 0.0f);  /* 位置微分不使用LPF，避免双重滤波 */
    
    /* 初始化ADRC控制器
     * b0 = 5.27 (控制增益), wc = 30 (控制器带宽), wo = 120 (观测器带宽, 3~5倍wc)
     * kp = ADRC_KP (独立可调比例增益)
     * 输出限幅: ±1.27Nm (电机额定扭矩)
     * 对于b0做了修改-0.186
     */
    adrc_init(&ctrl->adrc, -0.186f, 60.0f, 80.0f, ADRC_KP,
              -1.27f, 1.27f, ALGO_CONTROL_PERIOD_S);/**/
    
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
    ctrl->pressure_f0_sum = 0.0f;
    ctrl->pressure_f0_sample_count = 0;
    ctrl->pressure_stabilize_count = 0;  /* 稳定延迟计数器清零 */
    diff_reset(&ctrl->position_diff);
    
    /* 重置ADRC */
    adrc_reset(&ctrl->adrc);
    
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
 * F0校准 - 静止时压力值校准（使用一段时间内的平均值）
 ******************************************************************************/

static void calibrate_pressure_f0(GravityUnloadController_t *ctrl, float pressure_kg) {
    if (ctrl == NULL) return;

    /* 首次运行或需要重新校准时，使用一段时间内的平均压力值作为F0 */
    if (!ctrl->pressure_f0_calibrated) {
        /* F0校准期间，强制pressure_f0_kg为0，避免显示未校准的F0值 */
        ctrl->pressure_f0_kg = 0.0f;

        /* 立即开始F0校准，不进行稳定延迟
         * 避免在稳定延迟期间PID不输出扭矩导致重物下坠
         */
        if (ctrl->pressure_stabilize_count == 0) {
            printf("[GRAVITY_UNLOAD] Starting F0 calibration immediately...\n");
        }
        ctrl->pressure_stabilize_count++;

        /* 累加压力值 */
        ctrl->pressure_f0_sum += pressure_kg;
        ctrl->pressure_f0_sample_count++;

        /* 采集够指定样本数后，计算平均值作为F0 */
        if (ctrl->pressure_f0_sample_count >= PRESSURE_F0_CALIBRATION_SAMPLES) {
            ctrl->pressure_f0_kg = ctrl->pressure_f0_sum / ctrl->pressure_f0_sample_count;
            ctrl->pressure_f0_calibrated = 1;
            ctrl->f0_calibration_cycles = 0;  /* 重置F0校准后的周期计数 */
            /* F0校准完成后，重置ADRC控制器，确保从0开始 */
            adrc_reset(&ctrl->adrc);
            printf("[GRAVITY_UNLOAD] F0 calibrated: %.3f kg (avg of %d samples), ADRC reset\n",
                   ctrl->pressure_f0_kg, ctrl->pressure_f0_sample_count);
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

    /* 压力值直接使用传感器管理器提供的滤波后值（陷波滤波器已处理）
     * 不再进行额外的滑动平均滤波，避免双重滤波导致的延迟
     */
    float filtered_pressure = raw->pressure_kg;
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
 * 控制输出计算 - 重力卸载扭矩控制算法
 * 
 * 机械结构：
 * 电机 --(3:1减速比)--> 磁粉离合器 --(同轴)--> R1滑轮 --(绳子)--> R2滑轮 --(绳子)--> 重物
 *                                      ↑
 *                                夹角30.5°，压力传感器在夹角中线
 * 
 * 控制原理：
 * 1. 离合器保持抱死状态（提供固定阻转矩），作为刚性传动环节
 * 2. 当重物重量变化时，压力传感器检测DeltaF
 * 3. 电机主动提供扭矩，通过抱死的离合器传递给绳子
 * 4. 电机扭矩 × 3（减速比）= 输出到绳子的扭矩
 * 5. 通过控制电机扭矩来抵消重量变化，保持压力恒定
 * 
 * 扭矩计算：
 * - 绳张力 = DeltaF * 9.8 / (2 * cos(30.5°))
 * - R1滑轮所需扭矩 = 绳张力 * R1半径
 * - 电机所需扭矩 = R1滑轮所需扭矩 / 3（减速比）
 ******************************************************************************/

static void calculate_control_output(GravityUnloadController_t *ctrl,
                                     const SensorDataFiltered_t *filtered,
                                     ControlOutput_t *output) {
    if (ctrl == NULL || filtered == NULL || output == NULL) return;

    /* 计算DeltaF（压力变化量）
     * DeltaF = Fnow - F0 （当前压力 - 初始稳定压力）
     * DeltaF > 0: 重物变重，需要电机提供正扭矩提升
     * DeltaF < 0: 重物变轻，需要电机提供负扭矩释放
     */
    float delta_f_kg = 0.0f;
    if (ctrl->pressure_f0_calibrated) {
        delta_f_kg = filtered->pressure_kg - ctrl->pressure_f0_kg;
    } else {
        /* F0校准期间，DeltaF=0，ADRC不输出控制量 */
        delta_f_kg = 0.0f;
    }

    /* ========== ADRC + 动态P控制计算 ==========
     * 动态P增益：根据|DeltaF|大小和变化趋势，自动放大/缩小kp
     * 上升时：|DeltaF|>threshold_2 → boost_2；>threshold_1 → boost_1
     * 下降时：按两级阶梯下降，避免超调
     */
    float motor_rated_torque = 1.27f;
    float abs_deltaf = fabsf(delta_f_kg);
    float abs_last_deltaf = fabsf(g_last_deltaf_for_pd_boost);
    int is_increasing = (abs_deltaf > abs_last_deltaf + 0.0001f);
    int is_decreasing = (abs_deltaf < abs_last_deltaf - 0.0001f);
    float pd_gain_multiplier = g_current_pd_gain_multiplier;

    if (is_increasing) {
        if (abs_deltaf > g_deltaf_threshold_kg_2) {
            pd_gain_multiplier = ADRC_DYNAMIC_P_BOOST_2;
        } else if (abs_deltaf > g_deltaf_threshold_kg) {
            pd_gain_multiplier = ADRC_DYNAMIC_P_BOOST_1;
        } else {
            pd_gain_multiplier = 1.0f;
        }
    } else if (is_decreasing) {
        if (abs_deltaf > g_deltaf_threshold_kg_2) {
            pd_gain_multiplier = ADRC_DYNAMIC_P_BOOST_1;
        } else if (abs_deltaf > g_deltaf_threshold_kg) {
            pd_gain_multiplier = 1.0f;
        } else {
            pd_gain_multiplier = 1.0f;
        }
    }
    g_current_pd_gain_multiplier = pd_gain_multiplier;
    g_last_deltaf_for_pd_boost = delta_f_kg;

    /* ADRC自抗扰控制器：setpoint=0, 测量值=delta_f_kg, 动态P增益=p_d_gain_multiplier
     * ESO自动估计总扰动并补偿，动态P提升大误差响应速度
     */
    float motor_final_torque_Nm = adrc_update(&ctrl->adrc, 0.0f, delta_f_kg, pd_gain_multiplier);

    /* F0校准后的保护期：前50个周期（0.5秒）内，限制扭矩输出 */
    if (ctrl->pressure_f0_calibrated && ctrl->f0_calibration_cycles < 50) {
        /* 保护期内，扭矩输出从0逐渐增加到正常值 */
        float ramp_factor = (float)ctrl->f0_calibration_cycles / 50.0f;
        motor_final_torque_Nm *= ramp_factor;
        ctrl->f0_calibration_cycles++;
        if (ctrl->f0_calibration_cycles == 1) {
            printf("[GRAVITY_UNLOAD] F0 calibration protection period started (50 cycles)\n");
        }
        if (ctrl->f0_calibration_cycles == 50) {
            printf("[GRAVITY_UNLOAD] F0 calibration protection period ended\n");
        }
    }

    /* 限制电机扭矩在额定范围内 */
    if (motor_final_torque_Nm > motor_rated_torque) {
        motor_final_torque_Nm = motor_rated_torque;
    } else if (motor_final_torque_Nm < -motor_rated_torque) {
        motor_final_torque_Nm = -motor_rated_torque;
    }
    
    /* 输出变化率限制（斜坡限制）- 防止扭矩突变导致振荡 */
    /* 每周期最大变化量：0.005 Nm/5ms = 1 Nm/s */
    const float max_torque_change_per_cycle = 0.005f;
    float torque_change = motor_final_torque_Nm - ctrl->last_output_Nm;
    if (torque_change > max_torque_change_per_cycle) {
        motor_final_torque_Nm = ctrl->last_output_Nm + max_torque_change_per_cycle;
    } else if (torque_change < -max_torque_change_per_cycle) {
        motor_final_torque_Nm = ctrl->last_output_Nm - max_torque_change_per_cycle;
    }
    
    /* 保存输出用于下一周期变化率限制 */
    ctrl->last_output_Nm = motor_final_torque_Nm;

    /* ========== 步骤4: 设置离合器电流（保持抱死） ========== */
    /* 离合器保持额定电流，提供最大阻转矩，作为刚性传动 */
    float clutch_current_mA = CLUTCH_RATED_CURRENT_MA;  /* 880mA = 0.88A */
    output->clutch_current_mA = clutch_current_mA;
    output->clutch_torque_nm = CLUTCH_RATED_TORQUE_NM;  /* 5Nm */

    /* ========== 步骤5: 设置电机扭矩指令 ========== */
    /* 转换电机扭矩为指令值（0.001倍额定转矩） */
    /* 指令值 = (扭矩 / 额定扭矩) * 1000 */
    int motor_torque_cmd = (int)((motor_final_torque_Nm / motor_rated_torque) * 1000.0f);
    
    /* 限制电机扭矩指令范围 */
    if (motor_torque_cmd > 1000) {
        motor_torque_cmd = 1000;
    } else if (motor_torque_cmd < -1000) {
        motor_torque_cmd = -1000;
    }
    
    output->motor_torque_cmd = motor_torque_cmd;
    output->motor_torque_nm = motor_final_torque_Nm;  /* 保存实际扭矩值用于显示 */
    output->control_mode = CONTROL_MODE_TORQUE;

    /* 更新ADRC状态到共享内存（用于上位机显示和CSV记录）
     * z1: ESO估计的DeltaF
     * z2: ESO估计的总扰动
     * u0: 比例控制律输出
     * output_torque: 最终输出扭矩
     */
    update_adrc_state(ctrl->adrc.kp, pd_gain_multiplier, adrc_get_u0(&ctrl->adrc),
                      adrc_get_z1(&ctrl->adrc), adrc_get_z2(&ctrl->adrc), motor_final_torque_Nm);

    /* 保持旧update_pi_terms接口兼容，用于6_15模式/上位机旧版本显示 */
    update_pi_terms(adrc_get_z1(&ctrl->adrc), 0.0f, 0.0f, adrc_get_z2(&ctrl->adrc), 0.0f);
    /* 更新last_current为电机扭矩值（Nm），用于CSV记录 */
    update_pi_last_current(motor_final_torque_Nm);
    update_feedforward_and_target(clutch_current_mA, clutch_current_mA, delta_f_kg, filtered->pressure_kg);
    update_feedforward_torque(0.0f);  /* 前馈已由ADRC的z2补偿 */

    /* 计算电机目标速度（用于数据记录） */
    float pulley_velocity_rpm = (filtered->velocity_m_s / ctrl->pulley_r1_m) * (30.0f / 3.14159f);
    output->motor_velocity_target = pulley_velocity_rpm * 3.0f;
    output->motor_velocity_cmd = 0;
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

    printf("[GRAVITY_UNLOAD] Thread started - Industrial Grade Strict 5ms Cycle (200Hz)\n");

    SensorDataRaw_t raw_data;
    SensorDataFiltered_t filtered_data;
    ControlOutput_t control_output;
    memset(&control_output, 0, sizeof(control_output));  /* 初始化控制输出，避免随机值 */

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
            set_motor_torque(0);
            set_clutch_current(0);
            break;
        }
        
        /* 输出到执行器 - 协调控制：离合器电流 + 电机扭矩 */
        set_motor_torque(control_output.motor_torque_cmd);  /* 电机扭矩主控制 */
        set_clutch_current(control_output.clutch_current_mA);  /* 离合器额定电流抱死 */
        
        /* 更新电机目标扭矩到共享状态（用于数据采集） */
        /* 直接传递Nm值，让数据收集代码处理转换 */
        update_motor_target_torque(control_output.motor_torque_nm);
        
        /* 通知数据收集线程 - 确保数据采集同步 */
        extern void notify_data_collection_cycle(uint64_t timestamp_us, uint32_t cycle_count);
        notify_data_collection_cycle(get_time_us(), ctrl->cycle_count);
        
        /* 通知PDO线程 - 确保PDO发送与算法周期同步 */
        extern void notify_pdo_cycle(uint32_t cycle_count);
        notify_pdo_cycle(ctrl->cycle_count);
        
        /* 读取电源实际输出电流和电压（使用缓存，避免阻塞），并更新到共享状态 */
        uint16_t actual_current_ma = 0;
        uint16_t actual_voltage_mv = 0;
        if (g_power.initialized) {
            power_get_status_cached(&g_power, &actual_current_ma, &actual_voltage_mv);
            update_power_feedback(actual_current_ma / 1000.0f, actual_voltage_mv / 1000.0f,
                                  control_output.clutch_current_mA / 1000.0f);
        }
        
        /* 更新绳子速度到共享状态缓冲区 */
        update_rope_velocity(filtered_data.velocity_raw_m_s, filtered_data.velocity_m_s);
        
        /* 计算电机理论线速度并更新到共享状态缓冲区 */
        /* 理论线速度 = 新算法计算的目标电机速度对应的线速度 */
        /* control_output.motor_velocity_target 是基于摩擦力的控制算法计算的目标速度(rpm) */
        /* 注意：需要除以减速比3，将电机转速转换为滑轮转速 */
        float pulley_rpm = control_output.motor_velocity_target;  /* 电机rpm -> 滑轮rpm */
        float theory_linear_vel = -(pulley_rpm * 2.0f * 3.14159f * ctrl->pulley_r1_m) / 60.0f;
        update_motor_theory_velocity(theory_linear_vel);

        /* 计算并更新F0和DeltaF到共享状态缓冲区 */
        float delta_f_kg = 0.0f;
        if (ctrl->pressure_f0_calibrated) {
            delta_f_kg = filtered_data.pressure_kg - ctrl->pressure_f0_kg;
        }
        update_pressure_f0_deltaf(ctrl->pressure_f0_kg, delta_f_kg);

        /* 1Hz调试打印并输出到文件供上位机读取 */
        uint32_t current_time = get_timestamp_ms();
        if (current_time - last_print_time >= 1000) {  /* 1Hz输出 */
            last_print_time = current_time;

            /* 计算压力变化量用于显示 */
            float delta_f_display = 0.0f;
            if (ctrl->pressure_f0_calibrated) {
                delta_f_display = filtered_data.pressure_kg - ctrl->pressure_f0_kg;
            }

            /* 计算绳张力和扭矩 */
            float rope_tension_N = delta_f_display * 9.8f / (2.0f * FRICTION_ANGLE_COS);
            float pulley_torque_Nm = rope_tension_N * ctrl->pulley_r1_m;
            /* 使用 motor_torque_nm (Nm单位)，而不是 motor_torque_cmd (0.001倍额定) */
            float motor_torque_Nm = control_output.motor_torque_nm;

            /* 控制台输出 - 重力卸载扭矩控制信息 */
            float adrc_z1 = adrc_get_z1(&ctrl->adrc);
            float adrc_z2 = adrc_get_z2(&ctrl->adrc);
            
            printf("[DATA] P=%.3fkg F0=%.3fkg dF=%.3f Pos=%.3fm V_filt=%.3f I_clutch=%.1fmA T_pulley=%.2fNm T_motor=%.3fNm Mode=%d\n",
                   filtered_data.pressure_kg,
                   ctrl->pressure_f0_kg,
                   delta_f_display,
                   filtered_data.position_m,
                   filtered_data.velocity_m_s,
                   control_output.clutch_current_mA,
                   pulley_torque_Nm,
                   motor_torque_Nm,
                   g_friction_direction_mode);
             printf("[TORQUE_DETAIL] T_pulley_req=%.2fNm T_motor_target=%.3fNm T_motor_cmd=%d I_clutch=%.1fmA z1=%.3f z2=%.3f\n",
                    pulley_torque_Nm, motor_torque_Nm, control_output.motor_torque_cmd,
                    control_output.clutch_current_mA, adrc_z1, adrc_z2);
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

/******************************************************************************
 * 控制模式与目标力矩设置
 ******************************************************************************/

/* 静态变量用于存储控制模式和目标力矩 */
static ControlMode_t s_control_mode = CONTROL_MODE_TORQUE;
static float s_target_torque_nm = 0.0f;

void gravity_unload_set_control_mode(ControlMode_t mode) {
    s_control_mode = mode;
    printf("[GRAVITY_UNLOAD] Control mode set to: %s\n", 
           mode == CONTROL_MODE_TORQUE ? "TORQUE" : "VELOCITY");
}

ControlMode_t gravity_unload_get_control_mode(void) {
    return s_control_mode;
}

void gravity_unload_set_target_torque(float torque_nm) {
    /* 限制力矩范围在电机额定范围内 */
    if (torque_nm > 1.27f) {
        torque_nm = 1.27f;
    } else if (torque_nm < -1.27f) {
        torque_nm = -1.27f;
    }
    s_target_torque_nm = torque_nm;
}

float gravity_unload_get_target_torque(void) {
    return s_target_torque_nm;
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