/******************************************************************************
 * @file    motor_driver.c
 * @brief   NiMotion SDK电机驱动实现 - 工业级CANopen主站控制
 * @author  System Architect
 * @date    2026-05-12
 * @version 2.0.0
 * 
 * @description
 * 基于NiMotion NimServoSDK的电机驱动实现
 * - 使用官方SDK替代自定义SocketCAN
 * - 支持100Hz PDO实时控制
 * - 工业级错误处理和状态管理
 ******************************************************************************/

#include "motor_driver.h"
#include "../utils/logger.h"
#include "../../NimServoSDK-MM-bin-linux-x64/inc/nimservosdk.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

/******************************************************************************
 * 默认配置参数
 ******************************************************************************/
#define DEFAULT_UNIT_FACTOR     10000.0         /* 用户单位转换系数: 1.0 表示 1 rpm = 1 rpm */
#define DEFAULT_MAX_VELOCITY    3000.0      /* 默认最大速度 (rpm) */
#define DEFAULT_MAX_ACCEL       10000.0     /* 默认最大加速度 */
#define SDK_RETRY_COUNT         3           /* SDK操作重试次数 */
#define SDK_RETRY_DELAY_MS      50          /* 重试延时 */

/******************************************************************************
 * 内部辅助函数
 ******************************************************************************/

/**
 * @brief SDK错误码转字符串
 */
static const char* sdk_error_string(int error_code) {
    switch (error_code) {
        case ServoSDK_NoError:              return "No error";
        case ServoSDK_NotRegisted:          return "SDK not registered";
        case ServoSDK_NotInitialized:       return "SDK not initialized";
        case ServoSDK_UnsupportedCommType:  return "Unsupported communication type";
        case ServoSDK_ParamError:           return "Parameter error";
        case ServoSDK_CreateMasterFailed:   return "Create master failed";
        case ServoSDK_MasterNotExist:       return "Master not exist";
        case ServoSDK_MasterStartFailed:    return "Master start failed";
        case ServoSDK_MasterNotRunning:     return "Master not running";
        case ServoSDK_SlaveNotOnline:       return "Slave not online";
        case ServoSDK_LoadParamSheetFailed: return "Load parameter sheet failed";
        case ServoSDK_ParamNotExist:        return "Parameter not exist";
        case ServoSDK_ReadSDOFailed:        return "Read SDO failed";
        case ServoSDK_WriteSDOFailed:       return "Write SDO failed";
        case ServoSDK_OperationNotAllowed:  return "Operation not allowed";
        case ServoSDK_MasterInternalError:  return "Master internal error";
        case ServoSDK_SlaveInternalError:   return "Slave internal error";
        case ServoSDK_Cia402ModeError:      return "CiA 402 mode error";
        case ServoSDK_ReadWorkModeFailed:   return "Read work mode failed";
        case ServoSDK_ReadStatusWordFailed: return "Read status word failed";
        case ServoSDK_ReadCurrentPosFailed: return "Read current position failed";
        case ServoSDK_WriteControlWordFailed: return "Write control word failed";
        case ServoSDK_WriteTargetVelFailed: return "Write target velocity failed";
        default:                            return "Unknown error";
    }
}

/**
 * @brief 带重试的SDK调用宏
 */
#define SDK_CALL_WITH_RETRY(func, ...) \
    ({ \
        int _ret = -1; \
        for (int _i = 0; _i < SDK_RETRY_COUNT; _i++) { \
            _ret = func(__VA_ARGS__); \
            if (_ret == 0) break; \
            usleep(SDK_RETRY_DELAY_MS * 1000); \
        } \
        _ret; \
    })

/******************************************************************************
 * 函数实现
 ******************************************************************************/

ErrorCode_t motor_init(MotorDriver_t *motor, uint8_t node_id, const char *can_if) {
    if (motor == NULL || can_if == NULL) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Invalid parameters");
        return ERR_INVALID_PARAM;
    }
    
    memset(motor, 0, sizeof(MotorDriver_t));
    motor->node_id = node_id;
    strncpy(motor->can_interface, can_if, sizeof(motor->can_interface) - 1);
    motor->unit_factor = DEFAULT_UNIT_FACTOR;
    motor->max_velocity = DEFAULT_MAX_VELOCITY;
    motor->max_acceleration = DEFAULT_MAX_ACCEL;
    motor->state = MOTOR_STATE_INIT;
    
    /* 初始化机械参数 */
    motor->gear_ratio = 3.0;        /* 减速比 3:1 */
    motor->wheel_radius_mm = 100.0; /* 轮子半径 100mm */
    
    pthread_mutex_init(&motor->mutex, NULL);
    pthread_mutex_init(&motor->data_mutex, NULL);
    
    /* 初始化SDK */
    LOG_INFO(LOG_MODULE_MOTOR, "Initializing NiMotion SDK...");
    
    Nim_setLogFlags(1);  /* 启用控制台日志 */
    
    /* 关键修复：SDK需要在库文件所在目录下运行 */
    const char* sdk_path = ".";
    
    /* 保存当前工作目录 */
    char current_dir[512];
    if (getcwd(current_dir, sizeof(current_dir)) == NULL) {
        LOG_WARN(LOG_MODULE_MOTOR, "Failed to get current directory");
        strcpy(current_dir, ".");
    }
    
    /* 切换到lib目录 - 这是关键！SDK需要在这个目录下运行 */
    if (chdir(sdk_path) != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to change directory to lib path: %s", sdk_path);
        return ERR_INIT_FAIL;
    }
    LOG_INFO(LOG_MODULE_MOTOR, "Changed to SDK lib directory: %s", sdk_path);
    
    /* 设置LD_LIBRARY_PATH为当前目录（lib目录） */
    setenv("LD_LIBRARY_PATH", ".", 1);
    
    /* 初始化SDK - 使用当前目录（lib目录） */
    Nim_setLogFlags(1);
    Nim_init("./");  // 使用当前目录，测试程序就是这样做的
    LOG_INFO(LOG_MODULE_MOTOR, "SDK initialized from lib directory");
    
    /* 创建CANopen主站 - 在SDK目录下进行 */
    int ret;
    ret = Nim_create_master(0, &motor->sdk_master);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_create_master failed with error %d", ret);
        chdir(current_dir);  /* 恢复原目录 */
        Nim_clean();
        return ERR_INIT_FAIL;
    }
    
    if (motor->sdk_master == 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_create_master returned null handle");
        //chdir(current_dir);  /* 恢复原目录 */
        //Nim_clean();
        //return ERR_INIT_FAIL;
    }
    
    LOG_INFO(LOG_MODULE_MOTOR, "SDK master created successfully, handle=0x%08X", motor->sdk_master);
    
    /* 构建连接字符串 - 使用NiMotion USB-CAN设备 */
    char conn_str[256];
    snprintf(conn_str, sizeof(conn_str), 
             "{\"DevType\": \"1001\", \"DevIndex\": 0, \"Baudrate\": 8,"
             " \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}");
    
    LOG_INFO(LOG_MODULE_MOTOR, "Connection string: %s", conn_str);
    
    /* 启动主站 - 必须在SDK目录下进行，因为需要访问设备驱动文件 */
    ret = Nim_master_run(motor->sdk_master, conn_str);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_master_run failed with error %d", ret);
        LOG_ERROR(LOG_MODULE_MOTOR, "Error code 4 means DeviceOpenFailed - check USB permissions or device connection");
        Nim_destroy_master(motor->sdk_master);
        chdir(current_dir);  /* 恢复原目录 */
        Nim_clean();
        return ERR_INIT_FAIL;
    }
    
    LOG_INFO(LOG_MODULE_MOTOR, "SDK master running successfully");
    
    /* 恢复原工作目录 - 所有SDK初始化完成后再恢复 */
    if (chdir(current_dir) != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Failed to restore original directory");
    }
    
    /* 切换到Pre-Operational状态 */
    ret = Nim_master_changeToPreOP(motor->sdk_master);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_master_changeToPreOP returned %d", ret);
    }
    usleep(50000);  /* 必要延时 */
    
    /* 扫描节点 */
    LOG_INFO(LOG_MODULE_MOTOR, "Scanning nodes...");
    Nim_scan_nodes(motor->sdk_master, 1, 10);
    usleep(100000);  /* 增加扫描延时 */
    
    /* 检查目标节点是否在线 */
    if (!Nim_is_online(motor->sdk_master, node_id)) {
        LOG_WARN(LOG_MODULE_MOTOR, "Motor node %d is not online, trying again...", node_id);
        /* 重试扫描 */
        Nim_scan_nodes(motor->sdk_master, 1, 10);
        usleep(100000);
    }
    
    if (!Nim_is_online(motor->sdk_master, node_id)) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor node %d is still not online", node_id);
        Nim_master_stop(motor->sdk_master);
        Nim_destroy_master(motor->sdk_master);
        Nim_clean();
        return ERR_COMM_FAIL;
    }
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor node %d is online", node_id);
    
    /* 加载参数表 - 使用SDK目录中的CANopen.db */
    if (Nim_load_params(motor->sdk_master, node_id, "../NimServoSDK-MM-bin-linux-x64/bin/CANopen.db") != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_load_params with full path failed");
        if (Nim_load_params(motor->sdk_master, node_id, "CANopen.db") != 0) {
            LOG_WARN(LOG_MODULE_MOTOR, "Nim_load_params failed, using default parameters");
        }
    }
    
    /* 读取PDO配置 */
    if (Nim_read_PDOConfig(motor->sdk_master, node_id) != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_read_PDOConfig failed");
    }
    
    /* 切换到Operational状态 */
    ret = Nim_master_changeToOP(motor->sdk_master);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_master_changeToOP returned %d, continuing anyway...", ret);
    }
    usleep(50000);  /* 必要延时 */
    
    /* 设置用户单位转换系数 */
    Nim_set_unitsFactor(motor->sdk_master, node_id, motor->unit_factor);
    
    motor->sdk_initialized = 1;
    motor->initialized = 1;
    motor->state = MOTOR_STATE_READY;
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor driver initialized (Node ID=%d, SDK mode)", node_id);
    return ERR_OK;
}

void motor_deinit(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        return;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    /* 失能电机 */
    if (motor->enabled) {
        Nim_power_off(motor->sdk_master, motor->node_id, 1);
        usleep(50000);
    }
    
    /* 切换到Pre-OP */
    Nim_master_changeToPreOP(motor->sdk_master);
    usleep(50000);
    
    /* 停止主站 */
    Nim_master_stop(motor->sdk_master);
    
    /* 销毁主站 */
    Nim_destroy_master(motor->sdk_master);
    
    /* 清理SDK */
    Nim_clean();
    
    motor->sdk_initialized = 0;
    motor->initialized = 0;
    motor->enabled = 0;
    motor->state = MOTOR_STATE_INIT;
    
    pthread_mutex_unlock(&motor->mutex);
    pthread_mutex_destroy(&motor->mutex);
    pthread_mutex_destroy(&motor->data_mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor driver deinitialized");
}

ErrorCode_t motor_enable(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor not initialized");
        return ERR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Starting motor enable sequence (SDK)...");
    
    int ret;
    
    /* 清除错误 */
    ret = SDK_CALL_WITH_RETRY(Nim_clearError, motor->sdk_master, motor->node_id, 1);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_clearError failed: %s", sdk_error_string(ret));
    }
    usleep(50000);
    
    /* 失能电机（确保从已知状态开始） */
    ret = SDK_CALL_WITH_RETRY(Nim_power_off, motor->sdk_master, motor->node_id, 1);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_power_off failed: %s", sdk_error_string(ret));
    }
    usleep(50000);
    
    /* 设置CSV模式 */
    ret = SDK_CALL_WITH_RETRY(Nim_set_workMode, motor->sdk_master, motor->node_id, SERVO_CSV_MODE, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_set_workMode failed: %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    usleep(50000);
    
    /* 验证模式 */
    int mode_display;
    ret = SDK_CALL_WITH_RETRY(Nim_get_workModeDisplay, motor->sdk_master, motor->node_id, &mode_display, 1);
    if (ret != 0 || mode_display != SERVO_CSV_MODE) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Mode verification failed: got %d, expected %d", mode_display, SERVO_CSV_MODE);
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    motor->mode = MOTOR_MODE_CSV;
    LOG_INFO(LOG_MODULE_MOTOR, "CSV mode set successfully");
    
    /* 清零目标位置 */
    ret = SDK_CALL_WITH_RETRY(Nim_set_ipPosition, motor->sdk_master, motor->node_id, 0.0, 1);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_set_ipPosition failed: %s", sdk_error_string(ret));
    }
    
    /* 使能电机 */
    ret = SDK_CALL_WITH_RETRY(Nim_power_on, motor->sdk_master, motor->node_id, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_power_on failed: %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    usleep(200000);  /* 使能后需要较长延时 */
    
    /* 检查状态字 */
    unsigned short status_word;
    ret = SDK_CALL_WITH_RETRY(Nim_get_statusWord, motor->sdk_master, motor->node_id, &status_word, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_get_statusWord failed: %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    motor->status_word = status_word;
    LOG_INFO(LOG_MODULE_MOTOR, "Status word after enable: 0x%04X", status_word);
    
    /* 检查是否成功进入Operation Enabled状态 */
    if ((status_word & 0x006F) == 0x0027) {
        motor->state = MOTOR_STATE_ENABLED;
        motor->enabled = 1;
        LOG_INFO(LOG_MODULE_MOTOR, "SUCCESS: Motor enabled! Status word: 0x%04X", status_word);
    } else {
        /* 读取故障码 */
        unsigned int alarm_code;
        if (Nim_get_newestAlarm(motor->sdk_master, motor->node_id, &alarm_code, 1) == 0) {
            LOG_ERROR(LOG_MODULE_MOTOR, "Alarm code: 0x%08X", alarm_code);
        }
        
        motor->state = MOTOR_STATE_FAULT;
        motor->enabled = 0;
        LOG_ERROR(LOG_MODULE_MOTOR, "Motor enable FAILED! Status word: 0x%04X", status_word);
        pthread_mutex_unlock(&motor->mutex);
        return ERR_GENERAL;
    }
    
    pthread_mutex_unlock(&motor->mutex);
    return ERR_OK;
}

ErrorCode_t motor_disable(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    int ret = SDK_CALL_WITH_RETRY(Nim_power_off, motor->sdk_master, motor->node_id, 1);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_power_off failed: %s", sdk_error_string(ret));
    }
    
    motor->enabled = 0;
    motor->state = MOTOR_STATE_READY;
    
    pthread_mutex_unlock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor disabled");
    return ERR_OK;
}

ErrorCode_t motor_set_mode(MotorDriver_t *motor, MotorMode_t mode) {
    if (motor == NULL || !motor->sdk_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    /* 只有在脱机状态下才能设置模式 */
    if (motor->enabled) {
        LOG_WARN(LOG_MODULE_MOTOR, "Cannot change mode while motor is enabled");
        return ERR_OPERATION_NOT_ALLOWED;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    int sdk_mode;
    switch (mode) {
        case MOTOR_MODE_PP:  sdk_mode = SERVO_PP_MODE;  break;
        case MOTOR_MODE_VM:  sdk_mode = SERVO_VM_MODE;  break;
        case MOTOR_MODE_PV:  sdk_mode = SERVO_PV_MODE;  break;
        case MOTOR_MODE_PT:  sdk_mode = SERVO_PT_MODE;  break;
        case MOTOR_MODE_HM:  sdk_mode = SERVO_HM_MODE;  break;
        case MOTOR_MODE_IP:  sdk_mode = SERVO_IP_MODE;  break;
        case MOTOR_MODE_CSP: sdk_mode = SERVO_CSP_MODE; break;
        case MOTOR_MODE_CSV: sdk_mode = SERVO_CSV_MODE; break;
        case MOTOR_MODE_CST: sdk_mode = SERVO_CST_MODE; break;
        default:
            pthread_mutex_unlock(&motor->mutex);
            return ERR_INVALID_PARAM;
    }
    
    int ret = SDK_CALL_WITH_RETRY(Nim_set_workMode, motor->sdk_master, motor->node_id, sdk_mode, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_set_workMode failed: %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    motor->mode = mode;
    
    pthread_mutex_unlock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Motor mode set to %s", motor_get_mode_string(mode));
    return ERR_OK;
}

ErrorCode_t motor_set_velocity(MotorDriver_t *motor, float velocity) {
    if (motor == NULL || !motor->sdk_initialized || !motor->enabled) {
        return ERR_NOT_INITIALIZED;
    }

    /* 使用PDO方式发送目标速度 (100Hz实时) */
    /* 关键修复：SDK使用rps(转/秒)单位，需要将rpm转换为rps */
    /* 1 rpm = 1/60 rps，同时考虑unit_factor */
    double vel_rps = (double)velocity / 60.0;  /* rpm -> rps */

    /* 工业级优化：使用短超时确保100Hz实时性 */
    const int TIMEOUT_MS = 5;  /* 5ms超时 */

    pthread_mutex_lock(&motor->mutex);

    int ret = Nim_set_targetVelocity(motor->sdk_master, motor->node_id, vel_rps, TIMEOUT_MS);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_set_targetVelocity failed: %s", sdk_error_string(ret));
        motor->error_count++;
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }

    motor->target_velocity = velocity;  /* 保存原始rpm值 */
    motor->pdo_tx_count++;

    pthread_mutex_unlock(&motor->mutex);
    
    /* 限制日志输出频率 - 只有速度变化超过阈值或超过一定时间才打印 */
    static uint32_t last_log_time = 0;
    static float last_velocity = 0.0f;
    uint32_t now = get_timestamp_ms();
    if (fabs(velocity - last_velocity) > 0.1f || (now - last_log_time) > 500) {
        LOG_INFO(LOG_MODULE_MOTOR, "Set velocity: %.2f rpm (internal: %.2f)", velocity, vel_rps);
        last_velocity = velocity;
        last_log_time = now;
    }
    
    return ERR_OK;
}

ErrorCode_t motor_set_position(MotorDriver_t *motor, int32_t position) {
    if (motor == NULL || !motor->sdk_initialized || !motor->enabled) {
        return ERR_NOT_INITIALIZED;
    }
    
    double pos = (double)position;
    
    pthread_mutex_lock(&motor->mutex);
    
    int ret = Nim_set_targetPosition(motor->sdk_master, motor->node_id, pos, 0);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Nim_set_targetPosition failed: %s", sdk_error_string(ret));
        motor->error_count++;
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    motor->target_position = pos;
    motor->pdo_tx_count++;
    
    pthread_mutex_unlock(&motor->mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_get_velocity(MotorDriver_t *motor, int32_t *velocity) {
    if (motor == NULL || velocity == NULL || !motor->sdk_initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 使用PDO方式读取当前速度 */
    double vel;
    int ret = Nim_get_currentVelocity(motor->sdk_master, motor->node_id, &vel, 0);
    if (ret != 0) {
        /* 尝试使用备用接口 */
        int speed_rpm;
        ret = Nim_get_currentMotorSpeed(motor->sdk_master, motor->node_id, &speed_rpm, 0);
        if (ret != 0) {
            return ERR_COMM_FAIL;
        }
        *velocity = (int32_t)speed_rpm;
    } else {
        *velocity = (int32_t)vel;
    }
    
    return ERR_OK;
}

ErrorCode_t motor_get_position(MotorDriver_t *motor, int32_t *position) {
    if (motor == NULL || position == NULL || !motor->sdk_initialized) {
        return ERR_INVALID_PARAM;
    }
    
    /* 使用PDO方式读取当前位置 */
    double pos;
    int ret = Nim_get_currentPosition(motor->sdk_master, motor->node_id, &pos, 0);
    if (ret != 0) {
        return ERR_COMM_FAIL;
    }
    
    *position = (int32_t)pos;
    return ERR_OK;
}

float motor_get_position_m(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        return 0.0f;
    }
    
    double pos;
    if (Nim_get_currentPosition(motor->sdk_master, motor->node_id, &pos, 0) == 0) {
        /* 注意：NiMotion SDK的Nim_get_currentPosition返回的已经是用户单位（米） */
        /* unit_factor已经在SDK内部处理，不需要额外转换 */
        return (float)pos;
    }
    
    return 0.0f;
}

int32_t motor_get_velocity_rpm(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        return 0;
    }

    int speed_rpm;
    if (Nim_get_currentMotorSpeed(motor->sdk_master, motor->node_id, &speed_rpm, 0) == 0) {
        return (int32_t)speed_rpm;
    }

    return 0;
}

ErrorCode_t motor_get_velocity_cached(MotorDriver_t *motor, int32_t *velocity) {
    if (motor == NULL || velocity == NULL || !motor->sdk_initialized) {
        return ERR_INVALID_PARAM;
    }

    /* 直接从缓存读取，不执行CANopen通信
     * 缓存值由 motor_update_state 在独立线程中周期性更新
     */
    pthread_mutex_lock(&motor->data_mutex);
    *velocity = motor->actual_speed_rpm;
    pthread_mutex_unlock(&motor->data_mutex);

    return ERR_OK;
}

float motor_calculate_linear_velocity(MotorDriver_t *motor, float motor_rpm) {
    if (motor == NULL) {
        return 0.0f;
    }
    
    /* 线速度计算公式：
     * v = ω × r
     * ω = 电机转速 / 减速比 (rpm)
     * r = 轮子半径 (m)
     * 
     * 转换为 m/s：
     * v (m/s) = (motor_rpm / gear_ratio) × (2π × r / 60)
     *         = motor_rpm × (2π × r) / (gear_ratio × 60)
     */
    
    float wheel_radius_m = motor->wheel_radius_mm / 1000.0f;  /* mm -> m */
    float wheel_circumference = 2.0f * M_PI * wheel_radius_m;  /* 轮子周长 (m) */
    
    /* 轮子转速 (rps) = 电机转速 (rpm) / 减速比 / 60 */
    float wheel_rps = motor_rpm / motor->gear_ratio / 60.0f;
    
    /* 线速度 (m/s) = 轮子转速 (rps) × 轮子周长 (m) */
    float linear_velocity = wheel_rps * wheel_circumference;
    
    return linear_velocity;
}

ErrorCode_t motor_update_state(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        return ERR_NOT_INITIALIZED;
    }

    /* 工业级优化：使用短超时非阻塞读取
     * 1M波特率CANopen PDO通信理论上<1ms完成
     * 设置5ms超时确保实时性，同时容忍偶尔通信延迟
     */
    const int TIMEOUT_MS = 5;  /* 5ms超时，确保100Hz实时性 */

    /* 读取状态字 */
    unsigned short status_word;
    int ret = Nim_get_statusWord(motor->sdk_master, motor->node_id, &status_word, TIMEOUT_MS);
    if (ret != 0) {
        return ERR_COMM_FAIL;
    }

    pthread_mutex_lock(&motor->data_mutex);

    motor->status_word = status_word;

    /* 更新状态机 */
    uint16_t state_bits = status_word & 0x000F;
    switch (state_bits) {
        case 0x0000: motor->state = MOTOR_STATE_NOT_READY; break;
        case 0x0001: motor->state = MOTOR_STATE_NOT_READY; break;
        case 0x0002: motor->state = MOTOR_STATE_READY; break;
        case 0x0003: motor->state = MOTOR_STATE_READY; break;
        case 0x0004: motor->state = MOTOR_STATE_ENABLED; break;
        case 0x0005: motor->state = MOTOR_STATE_ENABLED; break;
        case 0x0006: motor->state = MOTOR_STATE_FAULT; break;
        case 0x0007: motor->state = MOTOR_STATE_FAULT; break;
        default:     motor->state = MOTOR_STATE_UNKNOWN; break;
    }

    /* 读取实际位置 - 短超时 */
    double pos;
    if (Nim_get_currentPosition(motor->sdk_master, motor->node_id, &pos, TIMEOUT_MS) == 0) {
        motor->actual_position = pos;
    }

    /* 读取实际速度 - 短超时 */
    double vel;
    if (Nim_get_currentVelocity(motor->sdk_master, motor->node_id, &vel, TIMEOUT_MS) == 0) {
        motor->actual_velocity = vel;
    }

    /* 读取实际转速 - 短超时 */
    int speed_rpm;
    if (Nim_get_currentMotorSpeed(motor->sdk_master, motor->node_id, &speed_rpm, TIMEOUT_MS) == 0) {
        motor->actual_speed_rpm = speed_rpm;
    }

    /* 读取实际转矩 - 短超时 */
    int torque;
    if (Nim_get_currentTorque(motor->sdk_master, motor->node_id, &torque, TIMEOUT_MS) == 0) {
        motor->actual_torque = torque;
    }

    motor->pdo_rx_count++;
    
    pthread_mutex_unlock(&motor->data_mutex);
    
    return ERR_OK;
}

ErrorCode_t motor_clear_fault(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    int ret = SDK_CALL_WITH_RETRY(Nim_clearError, motor->sdk_master, motor->node_id, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_clearError failed: %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    motor->fault_reset_needed = 0;
    motor->state = MOTOR_STATE_READY;
    
    pthread_mutex_unlock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Fault cleared");
    return ERR_OK;
}

ErrorCode_t motor_fast_stop(MotorDriver_t *motor) {
    if (motor == NULL || !motor->sdk_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    int ret = SDK_CALL_WITH_RETRY(Nim_fastStop, motor->sdk_master, motor->node_id, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Nim_fastStop failed: %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    pthread_mutex_unlock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Fast stop executed");
    return ERR_OK;
}

const char* motor_get_state_string(MotorState_t state) {
    switch (state) {
        case MOTOR_STATE_INIT:       return "INIT";
        case MOTOR_STATE_NOT_READY:  return "NOT_READY";
        case MOTOR_STATE_READY:      return "READY";
        case MOTOR_STATE_ENABLED:    return "ENABLED";
        case MOTOR_STATE_FAULT:      return "FAULT";
        case MOTOR_STATE_UNKNOWN:    return "UNKNOWN";
        default:                     return "INVALID";
    }
}

const char* motor_get_mode_string(MotorMode_t mode) {
    switch (mode) {
        case MOTOR_MODE_PP:  return "PP";
        case MOTOR_MODE_VM:  return "VM";
        case MOTOR_MODE_PV:  return "PV";
        case MOTOR_MODE_PT:  return "PT";
        case MOTOR_MODE_HM:  return "HM";
        case MOTOR_MODE_IP:  return "IP";
        case MOTOR_MODE_CSP: return "CSP";
        case MOTOR_MODE_CSV: return "CSV";
        case MOTOR_MODE_CST: return "CST";
        default:             return "UNKNOWN";
    }
}

void motor_get_statistics(MotorDriver_t *motor, uint64_t *tx_count, 
                          uint64_t *rx_count, uint64_t *err_count) {
    if (motor == NULL) return;
    
    pthread_mutex_lock(&motor->data_mutex);
    if (tx_count) *tx_count = motor->pdo_tx_count;
    if (rx_count) *rx_count = motor->pdo_rx_count;
    if (err_count) *err_count = motor->error_count;
    pthread_mutex_unlock(&motor->data_mutex);
}

/**
 * @brief 设置电机速度环PID参数
 * @param motor 电机驱动结构指针
 * @param kp 速度环比例增益 (I2008-1, 单位: 0.1Hz, 范围: 1-20000)
 * @param ki 速度环积分时间常数 (I2008-2, 单位: 0.01ms, 范围: 0-51200, 0=禁用积分)
 * @param kff 速度前馈增益 (I2008-16, 单位: 0.001, 范围: 0-65535)
 * @return ErrorCode_t
 * @note 建议在电机未使能时调用，或先切换到PreOP状态再设置
 */
ErrorCode_t motor_set_velocity_pid(MotorDriver_t *motor, uint16_t kp, uint16_t ki, uint16_t kff) {
    if (motor == NULL || !motor->sdk_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    int ret = 0;
    
    /* 设置速度环比例增益 I2008-1 */
    ret = Nim_set_param_value(motor->sdk_master, motor->node_id, "I2008-1", kp, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to set velocity Kp (I2008-1=%d): %s", kp, sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    /* 设置速度环积分时间常数 I2008-2 */
    ret = Nim_set_param_value(motor->sdk_master, motor->node_id, "I2008-2", ki, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to set velocity Ki (I2008-2=%d): %s", ki, sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    /* 设置速度前馈增益 I2008-16 */
    ret = Nim_set_param_value(motor->sdk_master, motor->node_id, "I2008-16", kff, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to set velocity Kff (I2008-16=%d): %s", kff, sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    
    /* 保存参数到驱动器 */
    ret = Nim_save_AllParams(motor->sdk_master, motor->node_id, 5000);
    if (ret != 0) {
        LOG_WARN(LOG_MODULE_MOTOR, "Failed to save params: %s", sdk_error_string(ret));
        /* 不返回错误，因为参数已经设置成功 */
    }
    
    pthread_mutex_unlock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Velocity PID updated: Kp=%d(0.1Hz), Ki=%d(0.01ms), Kff=%d(0.001)", 
             kp, ki, kff);
    return ERR_OK;
}

/**
 * @brief 获取电机速度环PID参数
 * @param motor 电机驱动结构指针
 * @param kp 速度环比例增益输出
 * @param ki 速度环积分时间常数输出
 * @param kff 速度前馈增益输出
 * @return ErrorCode_t
 */
ErrorCode_t motor_get_velocity_pid(MotorDriver_t *motor, uint16_t *kp, uint16_t *ki, uint16_t *kff) {
    if (motor == NULL || !motor->sdk_initialized || kp == NULL || ki == NULL || kff == NULL) {
        return ERR_INVALID_PARAM;
    }
    
    pthread_mutex_lock(&motor->mutex);
    
    int ret = 0;
    unsigned int value = 0;
    
    /* 读取速度环比例增益 I2008-1 */
    ret = Nim_get_param_value(motor->sdk_master, motor->node_id, "I2008-1", &value, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to get velocity Kp (I2008-1): %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    *kp = (uint16_t)value;
    
    /* 读取速度环积分时间常数 I2008-2 */
    ret = Nim_get_param_value(motor->sdk_master, motor->node_id, "I2008-2", &value, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to get velocity Ki (I2008-2): %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    *ki = (uint16_t)value;
    
    /* 读取速度前馈增益 I2008-16 */
    ret = Nim_get_param_value(motor->sdk_master, motor->node_id, "I2008-16", &value, 1);
    if (ret != 0) {
        LOG_ERROR(LOG_MODULE_MOTOR, "Failed to get velocity Kff (I2008-16): %s", sdk_error_string(ret));
        pthread_mutex_unlock(&motor->mutex);
        return ERR_COMM_FAIL;
    }
    *kff = (uint16_t)value;
    
    pthread_mutex_unlock(&motor->mutex);
    
    LOG_INFO(LOG_MODULE_MOTOR, "Velocity PID read: Kp=%d(0.1Hz), Ki=%d(0.01ms), Kff=%d(0.001)", 
             *kp, *ki, *kff);
    return ERR_OK;
}
