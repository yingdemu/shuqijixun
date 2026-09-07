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

    encoder_total_1 += encoder_speed_1;
    encoder_total_2 += encoder_speed_2;
}
