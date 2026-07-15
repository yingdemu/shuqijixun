/*********************************************************************************************************************
* 文件名称          control
* 功能描述          智能车摄像头扫描巡线 - 舵机与电机控制实现
* 适用平台          MM32F327X_G8P
*
* 舵机控制原理：
*   50Hz PWM（周期 20ms），高电平 0.5ms~2.5ms 对应 0°~180°
*   高电平 1.0ms → 0°，1.5ms → 90°（正中），2.0ms → 180°
*   占空比 = PWM_DUTY_MAX / 20ms * (0.5ms + angle/90° * 1ms)
*
* 电机控制原理：
*   DRV8701E 驱动芯片，DIR 控制方向，PWM 控制转速
*   DIR=HIGH + PWM=占空比 → 前进，DIR=LOW + PWM=占空比 → 后退
*
* 参考例程：
*   E02_04_drv8701e_double_motor_contro_demo（双侧电机）
*   E02_06_servo_control_demo（舵机）
*********************************************************************************************************************/

#include "control.h"

//==================================================== 控制模块初始化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：control_init
// 功能：初始化舵机 PWM 和双侧电机 GPIO/PWM
//-------------------------------------------------------------------------------------------------------------------
void control_init(void)
{
    // ---- 舵机 PWM 初始化 ----
    // 频率 50Hz，初始占空比为 0
    pwm_init(SERVO_PWM, SERVO_FREQ, 0);

    // ---- 左电机 GPIO 和 PWM 初始化 ----
    gpio_init(MOTOR_L_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                     // 左电机方向引脚，默认高电平（前进）
    pwm_init(MOTOR_L_PWM, MOTOR_PWM_FREQ, 0);                                   // 左电机 PWM，频率 17KHz，初始占空比 0

    // ---- 右电机 GPIO 和 PWM 初始化 ----
    gpio_init(MOTOR_R_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                     // 右电机方向引脚，默认高电平（前进）
    pwm_init(MOTOR_R_PWM, MOTOR_PWM_FREQ, 0);                                   // 右电机 PWM，频率 17KHz，初始占空比 0
}

//==================================================== 舵机角度控制 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：servo_set_angle
// 功能：设置舵机转动到指定角度
// 参数：angle —— 目标角度（0.0 ~ 180.0），90° 为正中
//
// 限幅：
//   angle < SERVO_ANGLE_MIN → angle = SERVO_ANGLE_MIN
//   angle > SERVO_ANGLE_MAX → angle = SERVO_ANGLE_MAX
//
// 占空比计算：
//   50Hz 频率下，PWM_DUTY_MAX=10000，周期=20ms
//   0.5ms 高电平(0°) 对应占空比 = 10000/20*0.5 = 250
//   1.5ms 高电平(90°) 对应占空比 = 10000/20*1.5 = 750
//   2.5ms 高电平(180°)对应占空比 = 10000/20*2.5 = 1250
//   通用公式：duty = PWM_DUTY_MAX / (1000/freq) * (0.5 + angle/90)
//-------------------------------------------------------------------------------------------------------------------
void servo_set_angle(float angle)
{
    // ---- 第一步：限幅（保护机械结构，防止打死方向） ----
    if(angle < SERVO_ANGLE_MIN)
        angle = SERVO_ANGLE_MIN;
    else if(angle > SERVO_ANGLE_MAX)
        angle = SERVO_ANGLE_MAX;

    // ---- 第二步：计算占空比并输出 ----
    uint32 duty = SERVO_DUTY(angle);
    pwm_set_duty(SERVO_PWM, duty);
}

//==================================================== 电机转速控制 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：motor_set_duty
// 功能：设置双侧电机转速（支持差速转向）
// 参数：left_duty  —— 左电机占空比（-100 ~ 100）
// 参数：right_duty —— 右电机占空比（-100 ~ 100）
//
// 正值 = 前进（DIR高电平），负值 = 后退（DIR低电平），0 = 停止
//
// 例如：
//   motor_set_duty(50, 50);   → 两轮同速前进
//   motor_set_duty(-50, -50); → 两轮同速后退
//   motor_set_duty(80, 20);   → 左轮快右轮慢，向右转弯
//   motor_set_duty(20, 80);   → 左轮慢右轮快，向左转弯
//   motor_set_duty(0, 0);     → 停止
//-------------------------------------------------------------------------------------------------------------------
void motor_set_duty(int16 left_duty, int16 right_duty)
{
    uint32 left_pwm, right_pwm;

    // ==================== 左电机处理 ====================

    // ---- 限幅 ----
    if(left_duty > MOTOR_DUTY_MAX)
        left_duty = MOTOR_DUTY_MAX;
    else if(left_duty < -MOTOR_DUTY_MAX)
        left_duty = -MOTOR_DUTY_MAX;

    if(left_duty >= 0)                                                          // 正值 → 前进
    {
        gpio_set_level(MOTOR_L_DIR, GPIO_HIGH);
    }
    else                                                                        // 负值 → 后退
    {
        gpio_set_level(MOTOR_L_DIR, GPIO_LOW);
        left_duty = -left_duty;                                                 // 取绝对值用于计算 PWM
    }

    // 占空比计算：PWM_DUTY_MAX * duty / 100
    left_pwm = (uint32)left_duty * PWM_DUTY_MAX / 100;
    pwm_set_duty(MOTOR_L_PWM, left_pwm);

    // ==================== 右电机处理 ====================

    // ---- 限幅 ----
    if(right_duty > MOTOR_DUTY_MAX)
        right_duty = MOTOR_DUTY_MAX;
    else if(right_duty < -MOTOR_DUTY_MAX)
        right_duty = -MOTOR_DUTY_MAX;

    if(right_duty >= 0)                                                         // 正值 → 前进
    {
        gpio_set_level(MOTOR_R_DIR, GPIO_HIGH);
    }
    else                                                                        // 负值 → 后退
    {
        gpio_set_level(MOTOR_R_DIR, GPIO_LOW);
        right_duty = -right_duty;
    }

    right_pwm = (uint32)right_duty * PWM_DUTY_MAX / 100;
    pwm_set_duty(MOTOR_R_PWM, right_pwm);
}
