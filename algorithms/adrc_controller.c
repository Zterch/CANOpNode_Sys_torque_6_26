/******************************************************************************
 * @file    adrc_controller.c
 * @brief   ADRC自抗扰控制器实现
 * @author  System Architect
 * @date    2026-06-24
 * @version 1.0.0
 ******************************************************************************/

#include "adrc_controller.h"
#include <string.h>
#include <math.h>

void adrc_init(ADRC_Controller_t *adrc,
               float b0, float wc, float wo, float kp,
               float u_min, float u_max,
               float dt) {
    if (adrc == NULL) return;
    
    memset(adrc, 0, sizeof(ADRC_Controller_t));
    
    adrc->b0 = b0;
    adrc->wc = wc;
    adrc->wo = wo;
    adrc->kp = kp;              /* kp 独立可配，不再强制等于 wc */
    adrc->beta1 = 2.0f * wo;
    adrc->beta2 = wo * wo;
    adrc->u_min = u_min;
    adrc->u_max = u_max;
    adrc->dt = dt;
    adrc->u_prev = 0.0f;
}

void adrc_reset(ADRC_Controller_t *adrc) {
    if (adrc == NULL) return;
    
    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;
    adrc->u_prev = 0.0f;
}

float adrc_update(ADRC_Controller_t *adrc, float y_ref, float y_meas, float p_gain_multiplier) {
    if (adrc == NULL) return 0.0f;
    
    float e, u0, u;
    
    /* ---- ESO扩张状态观测器 ---- */
    /* 观测误差 */
    e = adrc->z1 - y_meas;
    
    /* 状态估计更新 (离散化, 欧拉积分) */
    /* z1_dot = z2 - beta1*e + b0*u_prev */
    /* z2_dot = -beta2*e */   
    float z1_pred, z2_pred;

    e = adrc->z1 - y_meas;

    // 预测步
    z1_pred = adrc->z1 + (adrc->z2 - adrc->beta1*e + adrc->b0*adrc->u_prev) * adrc->dt;
    z2_pred = adrc->z2 + (-adrc->beta2 * e) * adrc->dt;

    // 校正步（用预测值重新计算误差）
    float e_corr = z1_pred - y_meas;
    adrc->z1 = adrc->z1 + (z1_pred - adrc->z1 + (-adrc->beta1 * e_corr) * adrc->dt);
    adrc->z2 = adrc->z2 + (z2_pred - adrc->z2 + (-adrc->beta2 * e_corr) * adrc->dt);


    /* 限幅防止观测器发散 */
    if (adrc->z1 > 10.0f)  adrc->z1 = 10.0f;
    if (adrc->z1 < -10.0f) adrc->z1 = -10.0f;
    if (adrc->z2 > 10.0f)  adrc->z2 = 10.0f;
    if (adrc->z2 < -10.0f) adrc->z2 = -10.0f;
    
    // 仅在零点小误差区间投入弱积分，大误差只保留P
    static float integral = 0.0f;
    float err = y_ref - adrc->z1;
    float effective_kp = adrc->kp * fmaxf(p_gain_multiplier, 0.0f);
    if (effective_kp < 0.0f) effective_kp = 0.0f;

    adrc->u0 = effective_kp * err; /* 保存 u0 */

    if(fabsf(err) < 0.05f)
    {
        integral += err * adrc->dt;
        integral = fminf(fmaxf(integral, -0.2f), 0.2f); // 积分限幅，防止饱和震荡
        u0 = adrc->u0 + 0.4f * integral;
    }
    else
    {
        integral = 0.0f; // 误差大时清零积分，避免累积
        u0 = adrc->u0; // 大偏差下维持纯P，保证快速响应
    }

    u  = (u0 - adrc->z2) / adrc->b0;


    /* 输出限幅 */
    if (u > adrc->u_max) u = adrc->u_max;
    if (u < adrc->u_min) u = adrc->u_min;
    
    /* 死区 (小信号不动作) */
    if (fabsf(u) < 0.005f) u = 0.0f;
    
    adrc->u_prev = u;
    return u;
}

float adrc_get_z1(const ADRC_Controller_t *adrc) {
    if (adrc == NULL) return 0.0f;
    return adrc->z1;
}

float adrc_get_z2(const ADRC_Controller_t *adrc) {
    if (adrc == NULL) return 0.0f;
    return adrc->z2;
}

float adrc_get_u0(const ADRC_Controller_t *adrc) {
    if (adrc == NULL) return 0.0f;
    return adrc->u0;
}

