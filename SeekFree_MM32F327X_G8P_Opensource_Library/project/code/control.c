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

// 阿克曼差速全局变量
float ackermann_gain_big = 1.9f;   //这个速度可以考虑给到2                                                // 差速增益（蓝牙可调）
float ackermann_gain_small = 0.0f;   //这个速度可以考虑给到2                                                // 差速增益（蓝牙可调）
float ackermann_gain = 1.0f;   //这个速度可以考虑给到2                                                // 差速增益（蓝牙可调）


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
// 参数：angle —— 目标角度（内部限幅±12°，偏移64°后输出）
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
    angle=-angle;
    if(angle < SERVO_ANGLE_MIN)
        angle = SERVO_ANGLE_MIN;
    else if(angle > SERVO_ANGLE_MAX)
        angle = SERVO_ANGLE_MAX;
    angle=angle+64;
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
//注意：此函数第一个参数虽然叫right_duty，但实际上是左电机占空比，第二个参数是右电机占空比
//-------------------------------------------------------------------------------------------------------------------
void motor_set_duty(float right_duty, float left_duty)
{
    uint32 left_pwm, right_pwm;

    // ==================== 左电机处理 ====================

    // ---- 限幅 ----
    if(left_duty != 0){
            if(left_duty > (float)MOTOR_DUTY_MAX){
                            left_duty = (float)MOTOR_DUTY_MAX;

            }
            else if(left_duty < (float)MOTOR_DUTY_MIN){
                        left_duty = (float)MOTOR_DUTY_MIN;

            }

    }

    if(left_duty >= 0.0f)                                                       // 正值 → 前进
    {
        gpio_set_level(MOTOR_L_DIR, GPIO_HIGH);
    }
    else                                                                        // 负值 → 后退
    {
        gpio_set_level(MOTOR_L_DIR, GPIO_LOW);
        left_duty = -left_duty;                                                 // 取绝对值
    }

    left_pwm = (uint32)(left_duty * (float)PWM_DUTY_MAX / 100.0f);
    pwm_set_duty(MOTOR_L_PWM, left_pwm);

    // ==================== 右电机处理 ====================

    if(right_duty != 0){
            if(right_duty > (float)MOTOR_DUTY_MAX){
                        right_duty = (float)MOTOR_DUTY_MAX;

            }
            else if(right_duty < (float)MOTOR_DUTY_MIN){
                        right_duty = (float)MOTOR_DUTY_MIN;

            }

    }

    if(right_duty >= 0.0f)
    {
        gpio_set_level(MOTOR_R_DIR, GPIO_HIGH);
    }
    else
    {
        gpio_set_level(MOTOR_R_DIR, GPIO_LOW);
        right_duty = -right_duty;
    }

    right_pwm = (uint32)(right_duty * (float)PWM_DUTY_MAX / 100.0f);
    pwm_set_duty(MOTOR_R_PWM, right_pwm);
}

//==================================================== 阿克曼差速 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：ackermann_differential
// 功能：根据舵机打角，基于阿克曼转向几何计算左右电机差速占空比
// 参数：servo_angle_deg —— 舵机打角（°，正=左转，负=右转）
// 参数：base_duty       —— 基础占空比（0~100）
// 参数：left_duty       —— 输出左电机占空比
// 参数：right_duty      —— 输出右电机占空比
// 返回：void
//
// 原理：
//   转弯半径 R = L / tan(δ)
//   速度差   ΔV = v × W × tan(δ) / L
//   左转(δ<0)：左轮(内侧)=减速，右轮(外侧)=加速
//   右转(δ>0)：右轮(内侧)=减速，左轮(外侧)=加速
//
// 使用示例：
//   float L, R;
//   ackermann_differential(servo_angle, motor_duty, &L, &R);
//   motor_set_duty(L, R);
//-------------------------------------------------------------------------------------------------------------------
void ackermann_differential(float servo_angle_deg, float base_duty, float *left_duty, float *right_duty)
{
    // 死区：打角绝对值小于阈值时不产生差速（避免直线微摆）
    float abs_angle = (servo_angle_deg > 0.0f) ? servo_angle_deg : -servo_angle_deg;
    if(abs_angle < ACKERMANN_DEADZONE_DEG)
    {
        *left_duty  = base_duty;
        *right_duty = base_duty;
        return;
    }

    // 角度转弧度，计算 tan(δ)
    #define DEG2RAD 0.017453293f                                                  // PI/180
    float angle_rad = servo_angle_deg * DEG2RAD;
    float tan_angle = angle_rad;                                                  // 小角度近似 tan(θ) ≈ θ（<12° 误差<2%）
    // 如需精确计算可替换为：tan_angle = tanf(angle_rad);

    // if(abs_angle>8){ackermann_gain = ackermann_gain_big;
    // }else {ackermann_gain = ackermann_gain_small;}

    // 阿克曼差速因子：diff = tan(δ) × W / L × gain
    float diff = tan_angle * ACKERMANN_TRACK / ACKERMANN_WHEELBASE * ackermann_gain;

    if(diff >0.0f){
    *left_duty  = base_duty * (1.0f + diff)*1.0f;
    *right_duty = base_duty * (1.0f - diff)*0.9f;

    }else{
    *left_duty  = base_duty * (1.0f + diff)*0.9f;
    *right_duty = base_duty * (1.0f - diff)*1.0f;
    
    }
    //  *left_duty  = base_duty * (1.0f + diff)*1.0f;
    //  *right_duty = base_duty * (1.0f - diff)*1.0f;

}
