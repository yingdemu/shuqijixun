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
#define PIT_PRIORITY            (TIM6_IRQn)                                     // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

// ==================== 主函数 ====================


float target_L=0, target_R=0;                                                     // 阿克曼输出的左右轮目标速度（编码器单位）
uint8 turn_timer_cnt = 0;                                                         // 弯道状态1计时：PIT累加，0=空闲，1~19=状态1，≥20=到期

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
                    motor_set_duty(0, 0); 
                    //car_go_flag=0;

                }
                else
                {
                    float weight_position = get_weight_position(center_line);
                    float groy_z = get_gyro_z();
                    float IMU_target = image_pid_set(0, IMG_W/2 - weight_position);
                    float servo_angle = IMU_pid_set(IMU_target, groy_z);

                    // 角度 PID：检测偏航角突变，提供稳定补偿
                    static float prev_yaw = 0.0f;
                    float angle_out = angle_pid_set(prev_yaw, atti_yaw);
                    prev_yaw = atti_yaw;

                    // 融合 IMU PID 和角度 PID 输出
                    float final_servo = servo_fusion(angle_out, servo_angle);

                    // 连续限幅 ±12°（servo_set_angle 内部也会再做一次）
                    if(final_servo > 10.0f)  final_servo = 12.0f;
                    if(final_servo < -10.0f) final_servo = -12.0f;

                    // 舵机输出速率限制：最大 4°/帧，防止突变
                    {
                        static float prev_servo_out = 0.0f;
                        float delta = final_servo - prev_servo_out;
                        if(delta > 4.0f)       final_servo = prev_servo_out + 4.0f;
                        else if(delta < -4.0f) final_servo = prev_servo_out - 4.0f;
                        prev_servo_out = final_servo;
                    }

                    servo_set_angle(final_servo);
                    prev_servo_angle = final_servo;

                    // ---- 直道/弯道判别 ----
                    // 条件1: 图像顶部中央3像素全白（前方是赛道）
                    // 条件2: 左右边界来自八邻域有效爬线
                    // 条件3: 左右边界未丢线（不贴边）
                    uint8 is_straight = 0;
                    {
                        uint8 row = 4;
                        uint8 col = IMG_W / 2;
                        if(binary_image[row][col] == WHITE &&
                           binary_image[row][col-1] == WHITE &&
                           binary_image[row][col+1] == WHITE)
                        {
                            if(left_valid[row] && right_valid[row])
                            {
                                if(left_boundary[row] >= 5 &&
                                   right_boundary[row] <= IMG_W - 6)
                                {
                                    is_straight = 1;
                                }
                            }
                        }
                    }

                    // ---- 速度决策：直道全速，弯道100ms降速 ----
                    float v_target;
                    if(is_straight)
                    {
                        turn_timer_cnt = 0;                                       // 直道：清零计时器
                        v_target = v_max_straight;
                    }
                    else
                    {
                        // 弯道状态1：首次检测到弯道或计时器未满20次

                        if(turn_timer_cnt<15){

                            if(turn_timer_cnt == 0) turn_timer_cnt = 1;           // 启动计时（PIT中断会累加）
                            v_target = v_max_turn_start;


                        }else if(turn_timer_cnt < 50)
                        {
                            if(turn_timer_cnt == 0) turn_timer_cnt = 1;           // 启动计时（PIT中断会累加）
                            float servo_dev = (final_servo > 0) ? final_servo : -final_servo;
                            v_target = v_max_turn - (v_max_turn - speed_min) * servo_dev * speed_decision_k / 12.0f;
                            if(v_target < speed_min) v_target = speed_min;
                        }
                        else
                        {
                            // 100ms已到：恢复直道速度，清零准备下一轮
                            //turn_timer_cnt = 0;
                            v_target = v_max_turn_cancel;
                        }
                    }

                    // 阿克曼：根据舵角分配左右轮目标（编码器单位）
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
    if(turn_timer_cnt > 0 && turn_timer_cnt < 50) turn_timer_cnt++;                // 弯道状态1计时（5ms/次，累加到20=100ms）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU963RA 的 Z 轴角速度（单位：°/s）
// 参数说明     void
// 返回参数     float —— Z轴角速度值（°/s），正值=逆时针，负值=顺时针
//-------------------------------------------------------------------------------------------------------------------
