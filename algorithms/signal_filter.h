/******************************************************************************
 * @file    signal_filter.h
 * @brief   信号滤波模块 - 移动平均滤波器和低通滤波器
 * @author  System Architect
 * @date    2026-04-23
 * @version 1.0.0
 ******************************************************************************/

#ifndef __SIGNAL_FILTER_H__
#define __SIGNAL_FILTER_H__

#include <stdint.h>
#include "algorithm_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * 移动平均滤波器结构体
 ******************************************************************************/
typedef struct {
    float buffer[SPEED_FILTER_WINDOW_SIZE];  /* 数据缓冲区 */
    uint8_t index;                           /* 当前索引 */
    uint8_t count;                           /* 有效数据数量 */
    float sum;                               /* 数据和（用于快速计算） */
} MovingAverageFilter_t;

/******************************************************************************
 * 低通滤波器结构体
 ******************************************************************************/
typedef struct {
    float alpha;        /* 滤波系数 0-1，越大越响应快 */
    float output;       /* 当前输出 */
    int initialized;    /* 是否已初始化 */
} LowPassFilter_t;

/******************************************************************************
 * 微分器结构体（带低通滤波）
 ******************************************************************************/
typedef struct {
    float last_input;   /* 上次输入 */
    uint32_t last_time; /* 上次时间 ms */
    float output;       /* 微分输出 */
    LowPassFilter_t lpf; /* 低通滤波器 */
    int use_lpf;        /* 是否使用LPF: 1=使用, 0=不使用 */
} Differentiator_t;

/******************************************************************************
 * 函数声明
 ******************************************************************************/

/**
 * @brief 初始化移动平均滤波器
 * @param filter 滤波器实例
 */
void ma_filter_init(MovingAverageFilter_t *filter);

/**
 * @brief 更新移动平均滤波器
 * @param filter 滤波器实例
 * @param input 输入值
 * @return 滤波后的输出
 */
float ma_filter_update(MovingAverageFilter_t *filter, float input);

/**
 * @brief 重置移动平均滤波器
 * @param filter 滤波器实例
 */
void ma_filter_reset(MovingAverageFilter_t *filter);

/**
 * @brief 初始化低通滤波器
 * @param lpf 低通滤波器实例
 * @param alpha 滤波系数 0-1
 */
void lpf_init(LowPassFilter_t *lpf, float alpha);

/**
 * @brief 更新低通滤波器
 * @param lpf 低通滤波器实例
 * @param input 输入值
 * @return 滤波后的输出
 */
float lpf_update(LowPassFilter_t *lpf, float input);

/**
 * @brief 重置低通滤波器
 * @param lpf 低通滤波器实例
 */
void lpf_reset(LowPassFilter_t *lpf);

/**
 * @brief 初始化微分器
 * @param diff 微分器实例
 * @param lpf_alpha 微分结果低通滤波系数
 */
void diff_init(Differentiator_t *diff, float lpf_alpha);

/**
 * @brief 更新微分器
 * @param diff 微分器实例
 * @param input 输入值
 * @param timestamp_ms 当前时间戳 ms
 * @return 微分结果（变化率）
 */
float diff_update(Differentiator_t *diff, float input, uint32_t timestamp_ms);

/**
 * @brief 重置微分器
 * @param diff 微分器实例
 */
void diff_reset(Differentiator_t *diff);

#ifdef __cplusplus
}
#endif

#endif /* __SIGNAL_FILTER_H__ */
