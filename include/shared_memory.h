#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHM_NAME "/gravshow_shm"
#define SHM_SIZE 4096
#define SHM_MAGIC 0x47524156  // "GRAV"

// 数据包版本
#define DATA_VERSION 2

/**
 * @brief 实时数据结构 - 20Hz更新
 */
typedef struct {
    uint32_t magic;          // 魔数 0x47524156
    uint32_t version;        // 版本号
    uint64_t timestamp_us;   // 微秒级时间戳
    uint32_t sequence;       // 序列号，用于检测丢包
    
    // 传感器数据
    float pressure_kg;       // 压力 (kg)
    float rope_length_m;     // 绳长 (m)
    uint32_t encoder_value;  // 编码器原始值
    float encoder_angle_deg; // 编码器角度
    
    // 新增重量采集模块数据 (UART/TTL, 100Hz)
    float weight_raw_kg;       // 原始重量 (kg)
    float weight_filtered_kg;  // 滤波后重量 (kg)
    
    // 电源数据
    float current_a;         // 电流 (A)
    float voltage_v;         // 电压 (V)
    
    // 电机数据
    float motor_speed_rpm;   // 电机速度
    float motor_position;    // 电机位置
    int32_t motor_status;    // 电机状态
    
    // 算法状态
    int32_t algorithm_state; // 0=停止, 1=运行中
    int32_t algorithm_error; // 错误码
    bool emergency_stop;     // 紧急停止
    
    // 控制输出
    float clutch_current_ma; // 离合器电流
    float motor_cmd_rpm;     // 电机指令速度
    
    // 力矩控制数据（新增）
    float motor_torque_percent; // 电机目标力矩 (Nm)
    float actual_torque_nm;     // 电机实际力矩 (Nm)
    int32_t control_mode;       // 控制模式: 0=速度模式, 1=力矩模式
    
    // ADRC自抗扰中间变量（用于数据记录与上位机显示）
    float adrc_kp;              // ADRC比例增益
    float adrc_p_gain_multiplier; // 动态P增益倍数
    float adrc_u0;              // 比例控制律输出 u0
    float adrc_z1;              // ESO估计的 DeltaF (kg)
    float adrc_z2;              // ESO估计的总扰动 (Nm)
    float adrc_output_torque;   // ADRC最终输出扭矩 (Nm)
    
    // 保留字段（扩展用）
    float reserved[1];
    
} SharedData_t;

/**
 * @brief 控制命令结构 - 即时响应
 */
typedef struct {
    uint32_t magic;          // 魔数
    uint32_t command_id;     // 命令ID，递增
    
    int32_t cmd_type;        // 0=无, 1=速度, 2=位置, 3=停止, 4=使能, 5=失能, 6=设置电流, 7=设置电压
    float cmd_value;         // 命令值
    float cmd_accel;         // 加速度
    
    bool algorithm_start;    // 启动算法标志
    bool algorithm_stop;     // 停止算法标志
    
    bool data_log_start;     // 开始数据记录标志
    bool data_log_stop;      // 停止数据记录标志
    
    // 力矩控制字段（新增）
    int32_t control_mode;       // 0=速度模式, 1=力矩模式
    float target_torque_nm;     // 目标力矩 (Nm)
    bool enable_torque_mode;    // 使能力矩模式标志
    bool control_mode_changed;  // 控制模式改变标志（新增）
    
    // 正弦测试字段（新增）
    bool sine_test_start;       // 开始正弦测试
    bool sine_test_stop;        // 停止正弦测试
    float sine_amplitude;       // 正弦振幅 (Nm)
    float sine_frequency;       // 正弦频率 (Hz)
    float sine_offset;          // 正弦偏置 (Nm)
    
} SharedCommand_t;

/**
 * @brief 共享内存管理器
 */
typedef struct {
    int shm_fd;
    void *shm_ptr;
    SharedData_t *data;
    SharedCommand_t *command;
    bool is_creator;
} ShmManager_t;

// 初始化共享内存
int shm_init(ShmManager_t *mgr, bool create);

// 关闭共享内存
void shm_close(ShmManager_t *mgr);

// 写入数据（下位机调用）
int shm_write_data(ShmManager_t *mgr, const SharedData_t *data);

// 读取数据（上位机调用）
int shm_read_data(ShmManager_t *mgr, SharedData_t *data);

// 写入命令（上位机调用）
int shm_write_command(ShmManager_t *mgr, const SharedCommand_t *cmd);

// 读取命令（下位机调用）
int shm_read_command(ShmManager_t *mgr, SharedCommand_t *cmd);

// 清空命令（下位机调用后执行）
void shm_clear_command(ShmManager_t *mgr);

#ifdef __cplusplus
}
#endif

#endif // SHARED_MEMORY_H
