/******************************************************************************
 * @file    signal_filter.c
 * @brief   信号滤波模块实现
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#include "signal_filter.h"
#include <string.h>

/******************************************************************************
 * 移动平均滤波器实现
 ******************************************************************************/

void ma_filter_init(MovingAverageFilter_t *filter) {
    if (filter == NULL) return;
    
    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;
}

float ma_filter_update(MovingAverageFilter_t *filter, float input) {
    if (filter == NULL) return input;
    
    /* 减去最旧的数据 */
    filter->sum -= filter->buffer[filter->index];
    
    /* 添加新数据 */
    filter->buffer[filter->index] = input;
    filter->sum += input;
    
    /* 更新索引 */
    filter->index++;
    if (filter->index >= SPEED_FILTER_WINDOW_SIZE) {
        filter->index = 0;
    }
    
    /* 更新有效数据计数 */
    if (filter->count < SPEED_FILTER_WINDOW_SIZE) {
        filter->count++;
    }
    
    /* 返回平均值 */
    return filter->sum / filter->count;
}

void ma_filter_reset(MovingAverageFilter_t *filter) {
    if (filter == NULL) return;
    ma_filter_init(filter);
}

/******************************************************************************
 * 低通滤波器实现
 ******************************************************************************/

void lpf_init(LowPassFilter_t *lpf, float alpha) {
    if (lpf == NULL) return;
    
    /* 限制alpha范围 */
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    lpf->alpha = alpha;
    lpf->output = 0.0f;
    lpf->initialized = 0;
}

float lpf_update(LowPassFilter_t *lpf, float input) {
    if (lpf == NULL) return input;
    
    if (!lpf->initialized) {
        lpf->output = input;
        lpf->initialized = 1;
        return input;
    }
    
    /* 一阶低通滤波: y[n] = alpha * x[n] + (1-alpha) * y[n-1] */
    lpf->output = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->output;
    
    return lpf->output;
}

void lpf_reset(LowPassFilter_t *lpf) {
    if (lpf == NULL) return;
    lpf->output = 0.0f;
    lpf->initialized = 0;
}

/******************************************************************************
 * 微分器实现
 ******************************************************************************/

void diff_init(Differentiator_t *diff, float lpf_alpha) {
    if (diff == NULL) return;
    
    diff->last_input = 0.0f;
    diff->last_time = 0;
    diff->output = 0.0f;
    diff->use_lpf = (lpf_alpha > 0.0f && lpf_alpha < 1.0f);
    if (diff->use_lpf) {
        lpf_init(&diff->lpf, lpf_alpha);
    }
}

float diff_update(Differentiator_t *diff, float input, uint32_t timestamp_ms) {
    if (diff == NULL) return 0.0f;
    
    /* 首次更新，只记录数据 */
    if (diff->last_time == 0) {
        diff->last_input = input;
        diff->last_time = timestamp_ms;
        return 0.0f;
    }
    
    /* 计算时间差 */
    float dt = (float)(timestamp_ms - diff->last_time) / 1000.0f; /* 转换为秒 */
    
    if (dt <= 0.0f) {
        return diff->output; /* 时间无效，返回上次结果 */
    }
    
    /* 计算微分 */
    float derivative = (input - diff->last_input) / dt;
    
    /* 对微分结果进行低通滤波（如果启用） */
    if (diff->use_lpf) {
        diff->output = lpf_update(&diff->lpf, derivative);
    } else {
        diff->output = derivative;
    }
    
    /* 更新记录 */
    diff->last_input = input;
    diff->last_time = timestamp_ms;
    
    return diff->output;
}

void diff_reset(Differentiator_t *diff) {
    if (diff == NULL) return;
    diff->last_input = 0.0f;
    diff->last_time = 0;
    diff->output = 0.0f;
    lpf_reset(&diff->lpf);
}
