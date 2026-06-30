/******************************************************************************
 * @file    adrc_controller.h
 * @brief   ADRC自抗扰控制器模块 (Active Disturbance Rejection Control)
 * @author  System Architect
 * @date    2026-06-24
 * @version 1.0.0
 *
 * @description
 * 自抗扰控制器包含：
 * 1. ESO (扩张状态观测器) - 实时估计总扰动
 * 2. 比例控制律 + 扰动前馈补偿
 * 3. 补偿后系统简化为积分器，仅需比例控制即可无静差
 *
 * 被控对象模型: ẏ = f(y, d, t) + b₀·u
 *   y  = DeltaF (压力变化量, kg)
 *   u  = 电机扭矩指令 (Nm)
 *   f  = 总扰动 (重力变化、摩擦、建模误差等)
 *   b₀ = 控制增益 (kg/Nm)
 ******************************************************************************/

#ifndef __ADRC_CONTROLLER_H__
#define __ADRC_CONTROLLER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 类型定义
 ******************************************************************************/

typedef struct {
    /* ESO状态 */
    float z1;       /* 估计的DeltaF */
    float z2;       /* 估计的总扰动 */
    
    /* ESO增益 (带宽法: beta1=2*wo, beta2=wo^2) */
    float beta1;
    float beta2;
    
    /* 控制增益 */
    float kp;       /* 比例增益 (= wc) */
    float b0;       /* 控制增益 */
    
    /* 上一时刻控制量 (用于ESO模型) */
    float u_prev;
    
    /* 输出限幅 */
    float u_max;
    float u_min;
    
    /* 采样周期 */
    float dt;
    
    /* 带宽参数 (保存用于调试) */
    float wc;       /* 控制器带宽 (rad/s) */
    float wo;       /* 观测器带宽 (rad/s) */
} ADRC_Controller_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化ADRC控制器
 * @param adrc 控制器实例
 * @param b0   控制增益 (kg/Nm)
 * @param wc   控制器带宽 (rad/s), 建议10~50
 * @param wo   观测器带宽 (rad/s), 建议3~5*wc
 * @param u_min 输出下限
 * @param u_max 输出上限
 * @param dt   采样周期 (s)
 */
void adrc_init(ADRC_Controller_t *adrc,
               float b0, float wc, float wo,
               float u_min, float u_max,
               float dt);

/**
 * @brief 重置ADRC控制器
 * @param adrc 控制器实例
 */
void adrc_reset(ADRC_Controller_t *adrc);

/**
 * @brief 更新ADRC控制器
 * @param adrc  控制器实例
 * @param y_ref 参考输入 (目标值, 通常为0)
 * @param y_meas 测量值 (DeltaF)
 * @return 控制输出 u (扭矩, Nm)
 */
float adrc_update(ADRC_Controller_t *adrc, float y_ref, float y_meas);

/**
 * @brief 获取ESO估计的DeltaF
 * @param adrc 控制器实例
 * @return z1
 */
float adrc_get_z1(const ADRC_Controller_t *adrc);

/**
 * @brief 获取ESO估计的总扰动
 * @param adrc 控制器实例
 * @return z2
 */
float adrc_get_z2(const ADRC_Controller_t *adrc);

#ifdef __cplusplus
}
#endif

#endif /* __ADRC_CONTROLLER_H__ */