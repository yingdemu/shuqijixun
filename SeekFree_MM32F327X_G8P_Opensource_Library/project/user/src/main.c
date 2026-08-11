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
#define SERVO_LOWPASS            (0.1f)                                          // 弯道舵机互补滤波系数
#define STRAIGHT_BLEND            (0.7f)                                          // 直道中线50%滤波系数
#define SERVO_CLIP_MAX            (11.0f)                                         // 舵机限幅上界
#define SERVO_CLIP_MIN            (-11.0f)                                        // 舵机限幅下界
#define SERVO_RATE_LIMIT          (4.0f)                                          // 舵机速率限制（°/帧）
#define STRAIGHT_FUSION_ALPHA     (0.2f)                                          // 直道 servo_fusion_alpha
#define TURN_FUSION_ALPHA         (0.0f)                                         // 弯道 servo_fusion_alpha
#define STRAIGHT_RECOVERY_TICKS   (60)                                            // 直道恢复计时（120×5ms=0.6s）
#define TURN_TIMER_THRESH1        (80)                                            // 弯道第一阶段
#define TURN_TIMER_THRESH2        (150)                                           // 弯道第二阶段
#define DUTY_LOWPASS              (0.2f)                                          // 电机占空比低通（固定5ms PIT，可用较轻滤波）

#define CURVE_LOCK_TICKS          (100)                                           // 弯道锁定计时（100×5ms=0.5s），0.5s内不能变直道

// ==================== 主函数 ====================


float target_L=0, target_R=0;                                                     // 阿克曼输出的左右轮目标速度（编码器单位）
float g_target_L = 0, g_target_R = 0;                                             // PIT 电机 PID 目标速度（主循环写入，PIT读取）
uint8  g_motor_run = 0;                                                           // PIT 电机 PID 使能标志（1=运行）
uint8 turn_timer_cnt = 0;                                                         // 弯道状态1计时：PIT累加，0=空闲
uint8 straight_rec_cnt = 0;                                                       // 直道恢复计时：PIT递减，0=已恢复
uint8 zebra_stop_flag = 0;                                                        // 斑马线停车标志：1=停车
uint16 zebra_cooldown = 0;                                                         // 斑马线冷却计时：PIT递减
uint8 g_duty_filt_reset = 0;                                                       // 占空比滤波重置（主循环写入，PIT读取清零）
static uint8 g_main_need_reset = 0;                                                 // 主循环状态重置标志（0→1发车时置1，帧末清零）
static uint8 g_prev_car_go = 0;                                                     // 上一帧 car_go_flag 状态（用于检测0→1跳变）

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
    interrupt_set_priority(PIT_PRIORITY, 1);                                      // 与VSYNC同级，不阻塞摄像头

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

            // 直道/弯道判别：中心列扫描 + 滞回滤波（与巡线模式一致）
            uint8 is_straight2;
            {
                static uint8 is_straight_state2 = 1;
                static uint8 curve_frames2 = 0;
                static uint8 straight_frames2 = 0;
                uint8 col = IMG_W / 2;
                uint8 r;
                uint8 black_cnt = 0;
                for(r = RING_FAR_ROW; r <= IMG_H - 3; r++)
                {
                    if(binary_image[r][col] == BLACK)
                        black_cnt++;
                }
                if(black_cnt >= 2)       { curve_frames2++;    straight_frames2 = 0; }
                else if(black_cnt == 0)  { straight_frames2++; curve_frames2 = 0;    }
                if(curve_frames2 >= 2)       is_straight_state2 = 0;
                else if(straight_frames2 >= 15) is_straight_state2 = 1;
                is_straight2 = is_straight_state2;
            }

            static float prev_wp2 = (float)(IMG_W / 2);

            // ---- 十字路口强制打角（crossroad_fix 输出） ----
            if(crossroad_forced_angle != 0)
            {
                servo_set_angle((float)crossroad_forced_angle);
                prev_servo_angle = (float)crossroad_forced_angle;
                continue;
            }

            // ---- 脱困检测1：中心列黑色超1/3 + 单侧列全黑 → 硬打角 ----
            {
                uint16 center_black_cnt = 0;
                uint8 col5_all_black  = 1;
                uint8 colr6_all_black = 1;
                int16 cr;
                int16 total_rows = IMG_H - 3;
                for(cr = 2; cr <= IMG_H - 2; cr++)
                {
                    if(binary_image[cr][IMG_W / 2] == BLACK) center_black_cnt++;
                    if(binary_image[cr][5]         == WHITE) col5_all_black  = 0;
                    if(binary_image[cr][IMG_W - 6] == WHITE) colr6_all_black = 0;
                }
                if(center_black_cnt > total_rows / 3)
                {
                    if(col5_all_black && !colr6_all_black)
                    {
                        servo_set_angle(12.0f);
                        prev_servo_angle = 12.0f;
                        continue;
                    }
                    else if(!col5_all_black && colr6_all_black)
                    {
                        servo_set_angle(-12.0f);
                        prev_servo_angle = -12.0f;
                        continue;
                    }
                }
            }

            // ---- 脱困检测2：底部半图三列4/5黑检测 ----
            {
                int16 r;
                uint16 center_black = 0, col4_black = 0, colr5_black = 0;
                int16 scan_start = IMG_H - 2;
                int16 scan_end   = IMG_H / 2;
                uint16 total = scan_start - scan_end + 1;
                uint16 thresh = total * 4 / 5;
                for(r = scan_end; r <= scan_start; r++)
                {
                    if(binary_image[r][IMG_W / 2] == BLACK) center_black++;
                    if(binary_image[r][4]          == BLACK) col4_black++;
                    if(binary_image[r][IMG_W - 5]  == BLACK) colr5_black++;
                }
                if(center_black >= thresh)
                {
                    if(col4_black >= thresh && colr5_black < thresh)
                    {
                        servo_set_angle(12.0f);
                        prev_servo_angle = 12.0f;
                        continue;
                    }
                    else if(col4_black < thresh && colr5_black >= thresh)
                    {
                        servo_set_angle(-12.0f);
                        prev_servo_angle = -12.0f;
                        continue;
                    }
                }
            }

            // ---- 脱困检测3：中线全部无效 → 按上一帧方向硬打角 ----
            {
                uint8 valid_cnt = 0;
                int16 vi;
                for(vi = 0; vi < IMG_H; vi++)
                    if(center_line_valid[vi] == 1) valid_cnt++;
                if(valid_cnt == 0)
                {
                    if(prev_wp2 >= IMG_W / 2)
                        servo_set_angle(12.0f);
                    else
                        servo_set_angle(-12.0f);
                    continue;
                }
            }

            float weight_position2 = get_weight_position(center_line, is_straight2);

            if(is_straight2)
                weight_position2 = STRAIGHT_BLEND * ((float)IMG_W / 2.0f)
                                 + (1.0f - STRAIGHT_BLEND) * weight_position2;

            float groy_z2 = get_gyro_z();
            float IMU_target2 = image_pid_set(0, IMG_W/2 - weight_position2);
            float servo_angle2 = IMU_pid_set(IMU_target2, groy_z2);
            static float prev_yaw2 = 0.0f;
            float angle_out2 = angle_pid_set(prev_yaw2, atti_yaw);
            prev_yaw2 = atti_yaw;
            float final_servo2 = servo_fusion(angle_out2, servo_angle2);

            if(final_servo2 > SERVO_CLIP_MAX)  final_servo2 = 12.0f;
            if(final_servo2 < SERVO_CLIP_MIN) final_servo2 = -12.0f;

            // 弯道舵机互补滤波（直→弯跳变时重置）
            {
                static float servo_filt2 = 0.0f;
                static uint8 last_was_straight2 = 1;
                if(!is_straight2)
                {
                    if(last_was_straight2) servo_filt2 = final_servo2;
                    else servo_filt2 = SERVO_LOWPASS * final_servo2 + (1.0f - SERVO_LOWPASS) * servo_filt2;
                    final_servo2 = servo_filt2;
                }
                last_was_straight2 = is_straight2;
            }

            {
                static float prev_servo_out2 = 0.0f;
                float delta = final_servo2 - prev_servo_out2;
                if(delta > SERVO_RATE_LIMIT)       final_servo2 = prev_servo_out2 + SERVO_RATE_LIMIT;
                else if(delta < -SERVO_RATE_LIMIT) final_servo2 = prev_servo_out2 - SERVO_RATE_LIMIT;
                prev_servo_out2 = final_servo2;
            }

            // 位置-舵角方向一致性约束：中线偏右禁止左转，偏左禁止右转
            if(weight_position2 > IMG_W / 2 && final_servo2 < -2.0f)
                final_servo2 = -2.0f;
            else if(weight_position2 < IMG_W / 2 && final_servo2 > 2.0f)
                final_servo2 = 2.0f;

            servo_set_angle(final_servo2);
            prev_servo_angle = final_servo2;

            if(is_straight2) servo_fusion_alpha = STRAIGHT_FUSION_ALPHA;
            else             servo_fusion_alpha = TURN_FUSION_ALPHA;

            // 保存本帧有效位置，供下帧中线全无效时判断硬打角方向
            prev_wp2 = weight_position2;
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
                // ---- 发车 0→1 跳变：重置所有 PID 和滤波器状态 ----
                if(!g_prev_car_go) { g_main_need_reset = 1; control_state_reset(); }
                g_prev_car_go = 1;

                // ---- 保护：底部中央10×10矩形全白或全黑 → 停车 ----
                // 矩形：左下角(IMG_H-3, IMG_W/2-5)，往上10行往右10列
                uint8 image_lost = 0;
                {
                    uint16 black_cnt = 0, total = 0;
                    uint8 r = IMG_H - 5;
                    uint8 c0 = 4;
                        for(uint8 c = c0; c < IMG_W - 4; c++)
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
                    // menu_need_refresh = 1;
                    // menu_need_clear   = 1;
                    zebra_stop_flag   = 1;
                    g_motor_run = 0;
                    motor_set_duty(0, 0);                                          // 出界立即刹车
                }
                else
                {
                    // 不提前清零 g_motor_run，避免 PIT 中断在图像处理期间误关电机
                    // g_motor_run 只在上方 image_lost 或下方正常路径中被设置

                    // 丢线侧翻转时置1，跳过舵机滤波和速率限制
                    uint8 reset_servo_filt = 0;

                    // ---- 直道/弯道判别：中心列扫描 + 滞回滤波 ----
                    // 直→弯敏感（≥2黑点+2帧确认），弯→直迟钝（全白+4帧确认）
                    static uint8 is_straight_state = 1;
                    static uint8 curve_frames = 0;
                    static uint8 straight_frames = 0;
                    if(g_main_need_reset) { is_straight_state = 1; curve_frames = 0; straight_frames = 0; }
                    {
                        uint8 col = IMG_W / 2;
                        uint8 r;
                        uint8 black_cnt = 0;
                        for(r = RING_FAR_ROW; r <= IMG_H - 3; r++)
                        {
                            if(binary_image[r][col] == BLACK)
                                black_cnt++;
                        }

                        if(black_cnt >= 2)      { curve_frames++;    straight_frames = 0; }
                        else if(black_cnt == 0) { straight_frames++; curve_frames = 0;    }
                        else                    { /* 1个黑点：保持当前状态，两边都不累计 */ }

                        if(curve_frames >= 2)       is_straight_state = 0;
                        else if(straight_frames >= 15) is_straight_state = 1;
                    }
                    uint8 is_straight = is_straight_state;

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

                    float weight_position = get_weight_position(center_line, is_straight);

                    // 直道时中线与图像中心混合滤波，减小不必要的转向修正
                    if(is_straight)
                    {
                        weight_position = STRAIGHT_BLEND * ((float)IMG_W / 2.0f)
                                        + (1.0f - STRAIGHT_BLEND) * weight_position;
                    }

                    // ---- 十字路口强制打角（crossroad_fix 输出） ----
                    if(crossroad_forced_angle != 0)
                    {
                        servo_set_angle((float)crossroad_forced_angle);
                        prev_servo_angle = (float)crossroad_forced_angle;
                        g_main_need_reset = 0;
                        continue;
                    }

                    // 中心列黑色超1/3 + 单侧列全黑 → 硬打角（十字路口/急弯脱困）
                    {
                        uint16 center_black_cnt = 0;
                        uint8 col5_all_black  = 1;
                        uint8 colr6_all_black = 1;
                        int16 cr;
                        int16 total_rows = IMG_H - 3;
                        for(cr = 2; cr <= IMG_H - 2; cr++)
                        {
                            if(binary_image[cr][IMG_W / 2] == BLACK) center_black_cnt++;
                            if(binary_image[cr][5]         == WHITE) col5_all_black  = 0;
                            if(binary_image[cr][IMG_W - 6] == WHITE) colr6_all_black = 0;
                        }

                        if(center_black_cnt > total_rows / 3)
                        {
                            // 左侧第5列全黑 + 右侧IMG_W-6列不全黑 → 右转
                            if(col5_all_black && !colr6_all_black)
                            {
                                servo_set_angle(12.0f);
                                prev_servo_angle = 12.0f;
                                g_main_need_reset = 0;
                                continue;
                            }
                            // 左侧第5列不全黑 + 右侧IMG_W-6列全黑 → 左转
                            else if(!col5_all_black && colr6_all_black)
                            {
                                servo_set_angle(-12.0f);
                                prev_servo_angle = -12.0f;
                                g_main_need_reset = 0;
                                continue;
                            }
                        }
                    }

                    // 底部1/2区域三列4/5黑检测 → 硬打角脱困
                    // 中心列+左侧列同时黑 → 右边有路，右转12°
                    // 中心列+右侧列同时黑 → 左边有路，左转-12°
                    {
                        int16 r;
                        uint16 center_black = 0, col4_black = 0, colr5_black = 0;
                        int16 scan_start = IMG_H - 2;
                        int16 scan_end   = IMG_H / 2;
                        uint16 total = scan_start - scan_end + 1;
                        uint16 thresh = total * 4 / 5;                               // 4/5 阈值

                        for(r = scan_end; r <= scan_start; r++)
                        {
                            if(binary_image[r][IMG_W / 2] == BLACK) center_black++;
                            if(binary_image[r][4]          == BLACK) col4_black++;
                            if(binary_image[r][IMG_W - 5]  == BLACK) colr5_black++;
                        }

                        if(center_black >= thresh)
                        {
                            // 左侧列黑 + 右侧列不黑 → 右转
                            if(col4_black >= thresh && colr5_black < thresh)
                            {
                                servo_set_angle(12.0f);
                                prev_servo_angle = 12.0f;
                                g_main_need_reset = 0;
                                continue;
                            }
                            // 左侧列不黑 + 右侧列黑 → 左转
                            else if(col4_black < thresh && colr5_black >= thresh)
                            {
                                servo_set_angle(-12.0f);
                                prev_servo_angle = -12.0f;
                                g_main_need_reset = 0;
                                continue;
                            }
                        }
                    }

                    // 中线全部无效 → 按上一帧pos方向硬打角脱困
                    static float prev_weight_position = (float)(IMG_W / 2);
                    {
                        if(g_main_need_reset) prev_weight_position = (float)(IMG_W / 2);
                        uint8 valid_cnt = 0;
                        int16 vi;
                        for(vi = 0; vi < IMG_H; vi++)
                            if(center_line_valid[vi] == 1) valid_cnt++;
                        if(valid_cnt == 0)
                        {
                            if(prev_weight_position >= IMG_W / 2)
                                servo_set_angle(12.0f);
                            else
                                servo_set_angle(-12.0f);
                            g_main_need_reset = 0;
                            continue;                                                   // 跳过本轮 PID 和速度决策
                        }
                    }

                    float groy_z = get_gyro_z();
                    float IMU_target = image_pid_set(0, IMG_W/2 - weight_position);
                    float servo_angle = IMU_pid_set(IMU_target, groy_z);

                    // 角度 PID：检测偏航角突变，提供稳定补偿
                    static float prev_yaw = 0.0f;
                    if(g_main_need_reset) prev_yaw = 0.0f;
                    float angle_out = angle_pid_set(prev_yaw, atti_yaw);
                    prev_yaw = atti_yaw;

                    // 融合 IMU PID 和角度 PID 输出
                    float final_servo = servo_fusion(angle_out, servo_angle);

                    // 连续限幅（servo_set_angle 内部也会再做一次）
                    if(final_servo > SERVO_CLIP_MAX)  final_servo = 12.0f;
                    if(final_servo < SERVO_CLIP_MIN) final_servo = -12.0f;

                    // 弯道时对舵机打角互补滤波，减少抖动
                    // 直→弯跳变时重置滤波，避免前一个弯的残留污染新弯
                    {
                        static float servo_filt = 0.0f;
                        static uint8 last_was_straight = 1;
                        if(g_main_need_reset) { servo_filt = 0.0f; last_was_straight = 1; }
                        if(!is_straight)
                        {
                            if(last_was_straight) servo_filt = final_servo;          // 刚入弯：重置
                            else if(reset_servo_filt) servo_filt = final_servo;     // 丢线翻转：立即重置
                            else servo_filt = SERVO_LOWPASS * final_servo + (1.0f - SERVO_LOWPASS) * servo_filt;
                            final_servo = servo_filt;
                        }
                        last_was_straight = is_straight;
                    }

                    // 舵机输出速率限制（丢线翻转时跳过，快速反向打角）
                    {
                        static float prev_servo_out = 0.0f;
                        if(g_main_need_reset) prev_servo_out = 0.0f;
                        if(!reset_servo_filt)
                        {
                            float delta = final_servo - prev_servo_out;
                            if(delta > SERVO_RATE_LIMIT)       final_servo = prev_servo_out + SERVO_RATE_LIMIT;
                            else if(delta < -SERVO_RATE_LIMIT) final_servo = prev_servo_out - SERVO_RATE_LIMIT;
                        }
                        prev_servo_out = final_servo;
                    }

                    // 位置-舵角方向一致性约束：中线偏右禁止左转，偏左禁止右转
                    if(weight_position > IMG_W / 2 && final_servo < -2.0f)
                        final_servo = -2.0f;
                    else if(weight_position < IMG_W / 2 && final_servo > 2.0f)
                        final_servo = 2.0f;

                    servo_set_angle(final_servo);
                    prev_servo_angle = final_servo;

                    // ---- 速度决策：直道全速，弯道降速 ----
                    float v_target;

                    // 0→1跳变检测
                    static uint8 prev_straight = 0;
                    static float prev_v_target = 0;
                    static float straight_start_speed = 0.0f;
                    if(g_main_need_reset) { prev_straight = 0; prev_v_target = 0; straight_start_speed = v_max_straight_start; }

                    if(is_straight)
                    {
                        servo_fusion_alpha = STRAIGHT_FUSION_ALPHA;                 // 直道：20%角度+80%IMU
                        turn_timer_cnt = 0;

                        if(!prev_straight){
                            straight_rec_cnt = STRAIGHT_RECOVERY_TICKS;            // ×5ms = 0.2s
                            straight_start_speed = (prev_v_target > 20.0f)         // 从出弯实际速度起步（上电首帧用默认值）
                                                        ? prev_v_target
                                                        : v_max_straight_start;
                        prev_straight = 1;
                        }

                        // 直道中远端单侧丢线 → 强制退回第一阶段（可能是假直道，前方有弯）
                        {
                            uint8 far_left_ok  = left_valid[RING_FAR_ROW];
                            uint8 far_right_ok = right_valid[RING_FAR_ROW];
                            if(far_left_ok != far_right_ok)                         // 仅单侧有边界
                                straight_rec_cnt = STRAIGHT_RECOVERY_TICKS;
                        }

                        // 直道阶梯升速：从出弯实际速度线性过渡到 v_max_straight
                        if(straight_rec_cnt > 0)
                        {
                            float t = 1.0f - (float)straight_rec_cnt / (float)STRAIGHT_RECOVERY_TICKS;
                            v_target = straight_start_speed + (v_max_straight - straight_start_speed) * t;
                        }
                        else
                        {
                            v_target = v_max_straight;
                        }
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
                            if(g_main_need_reset) prev_lost_side = 0;
                            if(prev_lost_side != 0 && lost_side != 0
                               && lost_side != prev_lost_side)
                            {
                                turn_timer_cnt = TURN_TIMER_THRESH1;                // 丢线侧翻转→直接进入弯道第二阶段
                                reset_servo_filt = 1;                               // 跳过舵机滤波+速率限制
                            }
                            if(lost_side != 0) prev_lost_side = lost_side;
                        }

                        // 弯道阶梯升速：从 speed_min 线性过渡到 v_max_turn_cancel
                        {
                            if(turn_timer_cnt == 0) turn_timer_cnt = 1;
                            if(turn_timer_cnt < TURN_TIMER_THRESH2)
                            {
                                float t = (float)turn_timer_cnt / (float)TURN_TIMER_THRESH2;
                                v_target = speed_min + (v_max_turn_cancel - speed_min) * t;
                            }
                            else
                            {
                                v_target = v_max_turn_cancel;
                            }
                        }
                    }

                    // 中端警告：如果中心列在中端行处为黑，说明即将出界，强制降速
                    if(binary_image[RING_MID_ROW][IMG_W / 2] == BLACK)
                    {
                        if(v_target > v_warning) v_target = v_warning;
                    }

                    // 速度目标低通滤波
                    {
                        #define VTARGET_LOWPASS 0.5f
                        static float v_filt = 0.0f;
                        static uint8 vf_init = 1;
                        static uint8 prev_was_straight = 1;
                        if(g_main_need_reset) { v_filt = 0.0f; vf_init = 1; prev_was_straight = 1; }

                        // 直→弯跳变：重置滤波，确保快速降速
                        if(prev_was_straight && !is_straight)
                        {
                            vf_init = 1;
                            g_duty_filt_reset = 1;                                    // 通知PIT重置占空比滤波
                        }
                        prev_was_straight = is_straight;

                        // 弯道第一阶段跳过滤波
                        if(!(!is_straight && turn_timer_cnt < TURN_TIMER_THRESH1))
                        {
                            if(vf_init) { v_filt = v_target; vf_init = 0; }
                            else { v_filt = VTARGET_LOWPASS * v_target + (1.0f - VTARGET_LOWPASS) * v_filt; }
                            v_target = v_filt;
                        }
                    }

                    prev_v_target = v_target;                           // 保存本帧最终速度，供下帧出弯过渡使用

                    // 斑马线检测：RING_NEAR_ROW 行 BW 跳变计数，两阶段确认后停车
                    if(zebra_cooldown == 0)
                    {
                        static uint8 zebra_cnt = 0;
                        if(g_main_need_reset) zebra_cnt = 0;
                        uint8 row = RING_NEAR_ROW;
                        uint8 trans = 0;
                        int16 c;
                        for(c = 3; c < IMG_W - 2; c++)
                            if(binary_image[row][c] == BLACK && binary_image[row][c+1] == WHITE)
                                trans++;
                        if(trans > 3)
                        {
                            zebra_cnt++;
                            if(zebra_cnt == 1)
                            {
                                zebra_cooldown = 400;                                  // 第一次检测：启动冷却
                            }
                            else if(zebra_cnt >= 2)
                            {
                                zebra_stop_flag = 1;                                  // 第二次检测：停车
                                zebra_cnt = 0;
                            }
                        }
                    }

                    // 阿克曼差速系数：直道 / 弯道第一阶段 / 弯道后期 三档独立
                    {
                        float raw_gain;
                        float abs_angle = (final_servo > 0.0f) ? final_servo : -final_servo;

                        // 增益状态编码：0=直道, 1=弯道第一阶段, 2=弯道后期
                        uint8 gain_state = is_straight ? 0
                                         : (turn_timer_cnt < TURN_TIMER_THRESH1 ? 1 : 2);

                        float actual_speed = (encoder_speed_filt_1 + encoder_speed_filt_2) / 2.0f;

                        if(gain_state == 0)
                        {
                            // ---- 直道：小差速，以速度为主，减少无谓的左右摆动 ----
                            raw_gain = 0.0f + 0.015f * (actual_speed);
                        }
                        else if(gain_state == 1)
                        {
                            // ---- 弯道第一阶段：大差速，以舵角为主，快速入弯 ----
                            raw_gain = 0.0f + 0.017f * (actual_speed);
                        }
                        else // gain_state == 2
                        {
                            // ---- 弯道后期：与第一阶段相同公式（后续可独立调参） ----
                            raw_gain = 0.0f + 0.0019f * (actual_speed);
                        }

                        // 中端警告时增大差速，增强修正能力防止出界
                        if(binary_image[RING_MID_ROW][IMG_W / 2] == BLACK)
                            raw_gain *= 1.3f;

                        #define ACKERMANN_LOWPASS 0.3f
                        static float filt_gain = 0.0f;
                        static uint8 gain_init = 1;
                        if(g_main_need_reset) { filt_gain = 0.0f; gain_init = 1; }

                        if(gain_init) { filt_gain = raw_gain; gain_init = 0; }
                        else { filt_gain = ACKERMANN_LOWPASS * raw_gain + (1.0f - ACKERMANN_LOWPASS) * filt_gain; }
                        ackermann_gain = filt_gain;
                    }

                    ackermann_differential(final_servo, v_target, &target_L, &target_R);

                    // 左右轮目标速度传入全局变量（PID + 占空比输出由 PIT 5ms 中断执行）
                    g_target_L = zebra_stop_flag ? 0.0f : target_L;
                    g_target_R = zebra_stop_flag ? 0.0f : target_R;
                    g_motor_run = 1;
                    // 保存本帧有效 pos，供下帧中线全无效时判断硬打角方向
                    prev_weight_position = weight_position;
                    g_main_need_reset = 0;                                              // 首帧结束，清除重置标志

                    // // 蓝牙发送
                    //serial_printf("%.3f,%.3f\r\n",L_duty, R_duty);
                }
                }
                else
                {
                    // car_go_flag == 0：用户手动停车，立即关电机
                    g_prev_car_go = 0;
                    g_motor_run = 0;
                    motor_set_duty(0, 0);
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
    if(zebra_cooldown > 0) zebra_cooldown--;                                        // 斑马线冷却计时（5ms/次）

    // ---- 电机速度 PID（固定5ms周期，不受摄像头帧率影响） ----
    {
        static uint8 prev_motor_run = 0;                                                // 用于检测电机0→1启动

        if(g_motor_run)
        {
            float L_duty = speed_pid_set(0, g_target_L, encoder_speed_filt_1);
            float R_duty = speed_pid_set(1, g_target_R, encoder_speed_filt_2);

            // 占空比低通滤波
            {
                static float L_filt = 0.0f, R_filt = 0.0f;
                static uint8 duty_init = 1;
                if(!prev_motor_run) duty_init = 1;                                      // 电机刚启动，重置滤波
                if(g_duty_filt_reset) { duty_init = 1; g_duty_filt_reset = 0; }        // 直→弯跳变：立即重置
                if(duty_init) { L_filt = L_duty; R_filt = R_duty; duty_init = 0; }
                else {
                    L_filt = DUTY_LOWPASS * L_duty + (1.0f - DUTY_LOWPASS) * L_filt;
                    R_filt = DUTY_LOWPASS * R_duty + (1.0f - DUTY_LOWPASS) * R_filt;
                }
                L_duty = L_filt;
                R_duty = R_filt;
            }

            motor_set_duty(L_duty, R_duty);
        }
        else
        {
            motor_set_duty(0, 0);
        }
        prev_motor_run = g_motor_run;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU963RA 的 Z 轴角速度（单位：°/s）
// 参数说明     void
// 返回参数     float —— Z轴角速度值（°/s），正值=逆时针，负值=顺时针
//-------------------------------------------------------------------------------------------------------------------
