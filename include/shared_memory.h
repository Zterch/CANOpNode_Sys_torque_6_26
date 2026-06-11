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
    
    // 保留字段（扩展用）
    float reserved[4];
    
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
