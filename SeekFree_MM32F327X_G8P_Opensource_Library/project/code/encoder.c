/*********************************************************************************************************************
* 文件名称          encoder
* 功能描述          智能车正交编码器速度读取模块
* 适用平台          MM32F327X_G8P
* 说明              在 PIT 中断中调用 encoder_update() 读取并清零编码器计数值
*                   正交解码模式：A/B两相正交信号，硬件自动4倍频并判断方向
*********************************************************************************************************************/

#include "encoder.h"

//==================================================== 全局变量 ====================================================

int16 encoder_speed_1 = 0;                                                          // 左轮速度（PIT周期内脉冲数，正=前进）
int16 encoder_speed_2 = 0;                                                          // 右轮速度（PIT周期内脉冲数）
int32 encoder_total_1 = 0;                                                          // 左轮累计脉冲
int32 encoder_total_2 = 0;                                                          // 右轮累计脉冲

float encoder_speed_filt_1 = 0.0f;                                                   // 左轮速度低通滤波值（供 PID 使用）
float encoder_speed_filt_2 = 0.0f;                                                   // 右轮速度低通滤波值

#define ENCODER_LOWPASS         (0.4f)                                              // 编码器低通滤波系数（0~1，越小滤波越重）

//==================================================== 编码器初始化 ====================================================

void encoder_init(void)
{
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
    encoder_speed_1 = 0;
    encoder_speed_2 = 0;
    encoder_total_1 = 0;
    encoder_total_2 = 0;
}

//==================================================== 编码器数据更新 ====================================================

void encoder_update(void)
{
    encoder_speed_1 = encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);

    encoder_speed_2 = -encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);

    // 一阶低通滤波：滤除编码器量化噪声（±1脉冲抖动）
    // 滤波系数 ENCODER_LOWPASS=0.4 → 时间常数约 2~3 个 PIT 周期（10~15ms）
    encoder_speed_filt_1 = ENCODER_LOWPASS * (float)encoder_speed_1
                         + (1.0f - ENCODER_LOWPASS) * encoder_speed_filt_1;
    encoder_speed_filt_2 = ENCODER_LOWPASS * (float)encoder_speed_2
                         + (1.0f - ENCODER_LOWPASS) * encoder_speed_filt_2;

    encoder_total_1 += encoder_speed_1;
    encoder_total_2 += encoder_speed_2;
}
