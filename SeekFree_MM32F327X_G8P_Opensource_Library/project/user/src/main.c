/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 MM32F327X-G8P 开源库的一部分
*
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
* 2024-07-14        yingdemu            智能车摄像头扫描巡线 - 按键菜单 + IPS200显示
********************************************************************************************************************/

// ==================== 智能车摄像头扫描巡线程序说明 ====================
//
// 【硬件连接】
//   摄像头：总钻风 MT9V03X 灰度摄像头（接主板摄像头接口）
//   显示屏：2寸 IPS200 模块（SPI或并口，接主板屏幕接口）
//   按键：  主板上的 KEY1~KEY4
//
// 【程序架构】
//   1. 菜单系统（common_menu + common_Mymenu）
//      - 纯数据结构（链表树）+ IPS200 文字显示
//      - KEY_1~KEY_4 四键操作，无需串口
//
//   2. 图像处理管线（image_process）
//      - 快速大津法（OTSU）→ 二值化 → 画黑框 → 八邻域爬线
//      - A/B/C/D 关键点检测 → 十字路口补线 → 赛道中线提取
//      - 中线数据存入 center_line[90] 数组
//
//   3. 巡线主控（line_follow）
//      - 状态机管理：等待图像 → 处理 → 数据就绪
//
// 【按键操作说明】
//   普通菜单模式：
//     KEY_1 → 光标上移
//     KEY_2 → 光标下移
//     KEY_3 → 确认 / 进入子菜单 / 选中编辑参数
//     KEY_4 → 返回上级 / 切换步进值
//   参数编辑模式（选中参数后）：
//     KEY_1 → 增大参数（长按=10倍快速调节）
//     KEY_2 → 减小参数（长按=10倍快速调节）
//     KEY_3 → 保存并退出编辑
//     KEY_4 → 取消编辑，恢复原值
//   图像显示模式：
//     KEY_4 → 退出图像显示
//
// 【菜单结构】
//   Main Menu
//     ├── image      → IPS200 显示摄像头灰度图像
//     ├── servo_pid  → servo_kp / servo_ki / servo_kd
//     └── motor_pid  → motor_kp / motor_ki / motor_kd
//
// ======================================================================





//可以试一下 angle_kd=0.460  ,ackermann_gain=12.200;A
#include "zf_common_headfile.h"
#include "common_menu.h"
#include "common_Mymenu.h"
#include "line_follow.h"
#include "control.h"
#include "encoder.h"
#include "bluetooth.h"
#include "zf_device_imu963ra.h"




#define IPS200_TYPE             (IPS200_TYPE_SPI)                     // 双排排针并口 → IPS200_TYPE_PARALLEL8
                                                                                // 单排排针 SPI → IPS200_TYPE_SPI
#define PIT                     (TIM6_PIT )                                     // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY            (TIM6_IRQn)                                     // 对应周期中断的中断编号
#define SERVO_LOWPASS            (0.5f)                                          // 弯道舵机互补滤波系数
#define STRAIGHT_DETECT_ROW       (3)                                             // 直道检测行号
#define STRAIGHT_BLEND            (0.5f)                                          // 直道中线50%滤波系数
#define SERVO_CLIP_MAX            (10.0f)                                         // 舵机限幅上界
#define SERVO_CLIP_MIN            (-10.0f)                                        // 舵机限幅下界
#define SERVO_RATE_LIMIT          (4.0f)                                          // 舵机速率限制（°/帧）
#define STRAIGHT_FUSION_ALPHA     (0.2f)                                          // 直道 servo_fusion_alpha
#define TURN_FUSION_ALPHA         (0.10f)                                         // 弯道 servo_fusion_alpha
#define STRAIGHT_RECOVERY_TICKS   (40)                                            // 直道恢复计时（40×5ms=0.2s）
#define TURN_TIMER_THRESH1        (80)                                            // 弯道第一阶段
#define TURN_TIMER_THRESH2        (150)                                           // 弯道第二阶段

// ==================== 主函数 ====================


float target_L=0, target_R=0;                                                     // 阿克曼输出的左右轮目标速度（编码器单位）
uint8 turn_timer_cnt = 0;                                                         // 弯道状态1计时：PIT累加，0=空闲
uint8 straight_rec_cnt = 0;                                                       // 直道恢复计时：PIT递减，0=已恢复

int main(void)
{
    // ---- 第1步：初始化系统时钟 120MHz ----
    clock_init(SYSTEM_CLOCK_120M);

    // ---- 第2步：初始化调试串口（仅用于 printf 调试输出，不用于菜单交互） ----
    debug_init();




    // ---- 第3步：初始化 IPS200 显示屏 ----
    ips200_init(IPS200_TYPE);
    ips200_set_dir(IPS200_CROSSWISE);                                           // 横屏模式 320×240（默认竖屏 240×320 会导致x溢出）
    ips200_clear();

    // ---- 第4步：显示启动画面 ----
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_show_string(0, 0 * 16, "Smart Car Init...");
    ips200_show_string(0, 1 * 16, "Platform: MM32F327X");
    ips200_show_string(0, 2 * 16, "Camera:  MT9V03X");
    ips200_show_string(0, 3 * 16, "Display: IPS200");

    // ---- 第5步：初始化 MT9V03X 总钻风摄像头 ----
    ips200_show_string(0, 4 * 16, "Init Camera...");
    while(1)
    {
        if(mt9v03x_init())                                                      // 失败返回非0
        {
            ips200_show_string(0, 5 * 16, "Retry...");
            system_delay_ms(500);
        }
        else
        {
            ips200_show_string(0, 5 * 16, "Camera OK!     ");
            break;
        }
    }

    // ---- 第6步：初始化按键模块（5ms扫描周期） ----
    key_init(5);
    ips200_show_string(0, 6 * 16, "Keys OK!       ");

    // ---- 第7步：初始化 HC-04 蓝牙模块 ----
    bluetooth_init();
    ips200_show_string(0, 7 * 16, "Bluetooth OK!   ");

    // ---- 第8步：初始化巡线模块 ----
    line_follow_init();

    // ---- 第9步：初始化控制模块（舵机 PWM + 双电机 GPIO/PWM） ----
    control_init();
    ips200_show_string(0, 8 * 16, "Control OK!     ");

    // ---- 第8.5步：初始化编码器 ----
    encoder_init();
    ips200_show_string(0, 9 * 16, "Encoder OK!     ");

    // ---- 第9.5步：初始化 IMU963RA 陀螺仪 ----
    while(imu963ra_init())
    {
        ips200_show_string(0, 9 * 16, "IMU963RA Retry...");
        system_delay_ms(500);
    }
    ips200_show_string(0, 9 * 16, "IMU963RA OK!    ");

    // ---- 第9.6步：初始化姿态解算（六轴互补滤波） ----
    atti_init();
    ips200_show_string(0, 10 * 16, "Atti OK!        ");

    // ---- 第10步：初始化菜单系统（创建菜单树 + 绘制初始界面） ----
    menu_init();
    // ips200_clear();                                                             // 首次绘制前清屏
    // menu_show_All();
    menu_need_refresh = 1;                                                        // 首次绘制前刷新菜单
    // ---- 第10步：所有初始化完成后，启动 PIT 周期中断（按键扫描 + 菜单处理） ----
    pit_ms_init(PIT, 5);
    interrupt_set_priority(PIT_PRIORITY, 0);

    // ---- 第11步：启动微秒定时器（用于测量图像处理耗时） ----
    timer_init(TIM_7, TIMER_US);                                                // TIM7 配置为微秒计数器（TIM3已被编码器占用）
    timer_start(TIM_7);                                                         // 启动计数

    // ---- 短暂延时 ----
    system_delay_ms(300);

    // ==================== 主循环 ====================
    while(1)
    {
        // ---- 蓝牙接收（调参命令解析） ----
        bluetooth_receive_process();

        // ---- 清屏处理（由中断中的 menu_key_process 触发） ----
        if(menu_need_clear)
        {
            ips200_clear();
            menu_need_clear = 0;
        }

        // ---- 菜单刷新（由中断中的 menu_key_process 触发） ----
        if(menu_need_refresh && !menu_in_image_mode)
        {
            menu_show_All();
        }

        // ==================== 图像显示模式 ====================
        if(menu_in_image_mode)
        {
            menu_image_display_process();

            float weight_position2 = get_weight_position(center_line);
            float groy_z2 = get_gyro_z();
            float IMU_target2 = image_pid_set(0, IMG_W/2 - weight_position2);
            float servo_angle2 = IMU_pid_set(IMU_target2, groy_z2);
            static float prev_yaw2 = 0.0f;
            float angle_out2 = angle_pid_set(prev_yaw2, atti_yaw);
            prev_yaw2 = atti_yaw;
            float final_servo2 = servo_fusion(angle_out2, servo_angle2);
            servo_set_angle(final_servo2);
            prev_servo_angle = final_servo2;
        }
        else
        {
            // ==================== 巡线处理 ====================
            {
                uint16 t_start = timer_get(TIM_7);                              // 开始计时（µs）
                line_follow_process();
                uint16 t_end = timer_get(TIM_7);                                // 结束计时（µs）
                uint16 elapsed_us = (t_end >= t_start) ? (t_end - t_start) : (65535 - t_start + t_end + 1);

                //当一帧处理完成时，通过蓝牙发送耗时
                // if(line_data_ready)
                // {
                //     float elapsed_ms = elapsed_us / 1000.0f;
                //     serial_printf("frame: %u us (%.2f ms)  OTSU=%u\r\n",
                //                 elapsed_us, elapsed_ms, otsu_threshold);
                // }


            //---- 后续 PID 控制可在此添加 ----
            if(line_data_ready)
            {
                if(car_go_flag){
                // ---- 保护：底部中央10×10矩形全白或全黑 → 停车 ----
                // 矩形：左下角(IMG_H-3, IMG_W/2-5)，往上10行往右10列
                uint8 image_lost = 0;
                {
                    uint16 black_cnt = 0, total = 0;
                    uint8 r0 = IMG_H - 3 - 9;
                    uint8 c0 = IMG_W / 2 - 5;
                    for(uint8 r = r0; r <= IMG_H - 3; r++)
                        for(uint8 c = c0; c < c0 + 10; c++)
                        {
                            if(binary_image[r][c] == BLACK) black_cnt++;
                            total++;
                        }
                    if(black_cnt >= total * 9 / 10)
                        image_lost = 1;
                }

                // 必须在PID计算之前检查：丢线时不跑PID，避免污染D项状态
                if(image_lost)
                {
                    //menu_need_refresh=1;
                    //menu_need_clear=1;
                    //motor_set_duty(0, 0); 
                    //car_go_flag=0;

                }
                else
                {
                    // ---- 直道/弯道判别（在使用中线前检测） ----
                    uint8 is_straight = 0;
                    {
                        uint8 row = STRAIGHT_DETECT_ROW;
                        // 统计 IMG_W/3 ~ IMG_W*2/3 范围内的白点数量
                        uint8 white_cnt = 0;
                        {
                            int16 c;
                            for(c = IMG_W / 3; c <= IMG_W * 2 / 3; c++)
                            {
                                if(binary_image[row][c] == WHITE) white_cnt++;
                            }
                        }
                        if(white_cnt >= 4)
                        {
                            if(left_valid[row] && right_valid[row])
                            {
                                if(left_boundary[row] >= 10 &&
                                   right_boundary[row] <= IMG_W - 10)
                                {
                                    is_straight = 1;
                                }
                            }
                        }
                    }

                    // 直→弯转换校验：上一帧直道但本帧非直道时，需确认边界确实偏移
                    //{
                    //    static uint8 prev_was_straight = 0;
                    //    if(prev_was_straight && !is_straight)
                    //    {
                    //        // 找左边界最后一个有效点
                    //        int16 l_last = -1, r_last = -1;
                    //        int16 i;
                    //        for(i = 3; i <= IMG_H - 2; i++)
                    //        {
                    //            if(l_last < 0 && left_valid[i])  l_last = i;
                    //            if(r_last < 0 && right_valid[i]) r_last = i;
                    //            if(l_last >= 0 && r_last >= 0)  break;
                    //        }
                    //        uint8 confirm_turn = 0;
                    //        if(l_last >= 0 && left_boundary[l_last] > IMG_W / 2)
                    //            confirm_turn = 1;
                    //        if(r_last >= 0 && right_boundary[r_last] < IMG_W / 2)
                    //            confirm_turn = 1;
                    //        if(!confirm_turn) is_straight = 1;
                    //    }
                    //    prev_was_straight = is_straight;
                    //}

                    float weight_position = get_weight_position(center_line);

                    // 直道时中线与图像中心混合滤波，减小不必要的转向修正
                    if(is_straight)
                    {
                        weight_position = STRAIGHT_BLEND * ((float)IMG_W / 2.0f)
                                        + (1.0f - STRAIGHT_BLEND) * weight_position;
                    }

                    float groy_z = get_gyro_z();
                    float IMU_target = image_pid_set(0, IMG_W/2 - weight_position);
                    float servo_angle = IMU_pid_set(IMU_target, groy_z);

                    // 角度 PID：检测偏航角突变，提供稳定补偿
                    static float prev_yaw = 0.0f;
                    float angle_out = angle_pid_set(prev_yaw, atti_yaw);
                    prev_yaw = atti_yaw;

                    // 融合 IMU PID 和角度 PID 输出
                    float final_servo = servo_fusion(angle_out, servo_angle);

                    // 连续限幅（servo_set_angle 内部也会再做一次）
                    if(final_servo > SERVO_CLIP_MAX)  final_servo = 12.0f;
                    if(final_servo < SERVO_CLIP_MIN) final_servo = -12.0f;

                    // 弯道时对舵机打角互补滤波，减少抖动
                    if(!is_straight)
                    {
                        static float servo_filt = 0.0f;
                        static uint8 filt_init = 1;
                        if(filt_init) { servo_filt = final_servo; filt_init = 0; }
                        else { servo_filt = SERVO_LOWPASS * final_servo + (1.0f - SERVO_LOWPASS) * servo_filt; }
                        final_servo = servo_filt;
                    }

                    // 舵机输出速率限制
                    {
                        static float prev_servo_out = 0.0f;
                        float delta = final_servo - prev_servo_out;
                        if(delta > SERVO_RATE_LIMIT)       final_servo = prev_servo_out + SERVO_RATE_LIMIT;
                        else if(delta < -SERVO_RATE_LIMIT) final_servo = prev_servo_out - SERVO_RATE_LIMIT;
                        prev_servo_out = final_servo;
                    }

                    servo_set_angle(final_servo);
                    prev_servo_angle = final_servo;

                    // ---- 速度决策：直道全速，弯道降速 ----
                    float v_target;

                    // 0→1跳变检测
                    static uint8 prev_straight = 0;

                    if(is_straight)
                    {
                        servo_fusion_alpha = STRAIGHT_FUSION_ALPHA;                 // 直道：20%角度+80%IMU
                        turn_timer_cnt = 0;

                        if(!prev_straight){
                            straight_rec_cnt = STRAIGHT_RECOVERY_TICKS;            // ×5ms = 0.2s
                        prev_straight = 1;
                        }

                        if(straight_rec_cnt > 0){
                            v_target = v_max_straight_start;
}
                        else{

                            v_target = v_max_straight;}
                    }
                    else
                    {
                        servo_fusion_alpha = TURN_FUSION_ALPHA;                     // 弯道：10%角度+90%IMU
                        straight_rec_cnt = 0;                                      // 弯道清零
                        prev_straight = 0;

                        // 弯道中丢线侧翻转 → 重置转弯计时
                        {
                            uint8 row = RING_NEAR_ROW;
                            uint8 left_lost  = (left_boundary[row] <= 2);
                            uint8 right_lost = (right_boundary[row] >= IMG_W - 3);
                            // 0=都没丢, 1=丢左边, 2=丢右边
                            uint8 lost_side = left_lost ? 1 : (right_lost ? 2 : 0);
                            static uint8 prev_lost_side = 0;
                            if(prev_lost_side != 0 && lost_side != 0
                               && lost_side != prev_lost_side)
                            {
                                turn_timer_cnt = 0;                                // 丢线侧翻转→重新计时
                            }
                            if(lost_side != 0) prev_lost_side = lost_side;
                        }

                        if(turn_timer_cnt < TURN_TIMER_THRESH1){
                            if(turn_timer_cnt == 0) turn_timer_cnt = 1;
                            v_target = speed_min;
                        }
                        else if(turn_timer_cnt < TURN_TIMER_THRESH2)
                        {
                            v_target = v_max_turn_start;

                        }
                        else
                        {
                            v_target = v_max_turn_cancel;

                        }
                    }

                    // 阿克曼：根据舵角分配左右轮目标（编码器单位）
                    ackermann_gain=0.0 + 0.24 *(abs(final_servo)-3.5f);

                    ackermann_differential(final_servo, v_target, &target_L, &target_R);

                    // 左右轮独立速度闭环
                    float L_duty = speed_pid_set(0, target_L, (float)encoder_speed_1);
                    float R_duty = speed_pid_set(1, target_R, (float)encoder_speed_2);
                    motor_set_duty(L_duty, R_duty);

                    // // 蓝牙发送
                    //serial_printf("%.3f,%.3f\r\n",L_duty, R_duty);
                }
                }
            }


            }

        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 周期中断回调（5ms），按键扫描 + 菜单按键处理
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    key_scanner();
    menu_key_process();
    encoder_update();                                                               // 读取编码器速度
    atti_update();                                                                  // 姿态解算（替代 imu963ra_get_gyro，内部已同时读取加速度计+陀螺仪）
    if(turn_timer_cnt > 0 && turn_timer_cnt < TURN_TIMER_THRESH2) turn_timer_cnt++;  // 弯道状态1计时
    if(straight_rec_cnt > 0) straight_rec_cnt--;                                    // 直道恢复计时（5ms/次）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU963RA 的 Z 轴角速度（单位：°/s）
// 参数说明     void
// 返回参数     float —— Z轴角速度值（°/s），正值=逆时针，负值=顺时针
//-------------------------------------------------------------------------------------------------------------------
