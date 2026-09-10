/*********************************************************************************************************************
* 文件名称          encoder
* 功能描述          智能车正交编码器速度读取模块
* 适用平台          MM32F327X_G8P
* 说明              基于逐飞 zf_driver_encoder 驱动，使用正交解码模式
*                   编码器1：TIM3, B4(A)/B5(B)  4倍频正交解码
*                   编码器2：TIM4, B6(A)/B7(B)  4倍频正交解码
*********************************************************************************************************************/

#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "zf_common_headfile.h"

//==================================================== 编码器引脚配置 ====================================================

#define ENCODER_1                   (TIM3_ENCODER)                                  // 左轮编码器
#define ENCODER_1_A                 (TIM3_ENCODER_CH1_B4)
#define ENCODER_1_B                 (TIM3_ENCODER_CH2_B5)

#define ENCODER_2                   (TIM4_ENCODER)                                  // 右轮编码器
#define ENCODER_2_A                 (TIM4_ENCODER_CH1_B6)
#define ENCODER_2_B                 (TIM4_ENCODER_CH2_B7)

//==================================================== 外部变量 ====================================================

extern int16 encoder_speed_1;                                                       // 左轮速度（PIT周期内脉冲数，正=前进）
extern int16 encoder_speed_2;                                                       // 右轮速度（PIT周期内脉冲数）
extern int32 encoder_total_1;                                                       // 左轮累计脉冲
extern int32 encoder_total_2;                                                       // 右轮累计脉冲
extern float encoder_speed_filt_1;                                                   // 左轮速度低通滤波值（供 PID 使用）
extern float encoder_speed_filt_2;                                                   // 右轮速度低通滤波值

//==================================================== 函数声明 ====================================================

void encoder_init(void);                                                            // 初始化双路正交编码器
void encoder_update(void);                                                          // 读取并清零编码器计数值（在PIT中断中调用）

#endif
