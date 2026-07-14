/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 MM32F327X-G8P 开源库的一部分
*
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
* 2024-07-14        yingdemu            智能车摄像头扫描巡线 - 按键菜单 + IPS114显示
********************************************************************************************************************/

// ==================== 智能车摄像头扫描巡线程序说明 ====================
//
// 【硬件连接】
//   摄像头：总钻风 MT9V03X 灰度摄像头（接主板摄像头接口）
//   显示屏：2寸 IPS114 模块（SPI或并口，接主板屏幕接口）
//   按键：  主板上的 KEY1~KEY4
//
// 【程序架构】
//   1. 菜单系统（common_menu + common_Mymenu）
//      - 纯数据结构（链表树）+ IPS114 文字显示
//      - KEY_1~KEY_4 四键操作，无需串口
//
//   2. 图像处理管线（image_process）
//      - 快速大津法（OTSU）→ 二值化 → 画黑框 → 八邻域爬线
//      - A/B/C/D 关键点检测 → 十字路口补线 → 赛道中线提取
//      - 中线数据存入 center_line[60] 数组
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
//     ├── image      → IPS114 显示摄像头灰度图像
//     ├── servo_pid  → servo_kp / servo_ki / servo_kd
//     └── motor_pid  → motor_kp / motor_ki / motor_kd
//
// ======================================================================

#include "zf_common_headfile.h"
#include "common_menu.h"
#include "common_Mymenu.h"
#include "line_follow.h"




#define PIT                     (TIM6_PIT )                                     // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY            (TIM6_IRQn)                                     // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

// ==================== 主函数 ====================

int main(void)
{
    // ---- 第1步：初始化系统时钟 120MHz ----
    clock_init(SYSTEM_CLOCK_120M);

    // ---- 第2步：初始化调试串口（仅用于 printf 调试输出，不用于菜单交互） ----
    debug_init();




    //------配置中断-----------------------------------
    pit_ms_init(PIT, 5);                                                     // 初始化 PIT 为周期中断 1000ms 周期

    interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT 对周期中断的中断优先级为 0


    // ---- 第3步：初始化 IPS114 显示屏（SPI 驱动，无需参数） ----
    ips114_init();
    ips114_clear();

    // ---- 第4步：显示启动画面 ----
    ips114_set_color(RGB565_BLACK, RGB565_WHITE);
    ips114_show_string(0, 0 * 16, "Smart Car Init...");
    ips114_show_string(0, 1 * 16, "Platform: MM32F327X");
    ips114_show_string(0, 2 * 16, "Camera:  MT9V03X");
    ips114_show_string(0, 3 * 16, "Display: IPS114");

    // ---- 第5步：初始化 MT9V03X 总钻风摄像头 ----
    ips114_show_string(0, 4 * 16, "Init Camera...");
    while(1)
    {
        if(mt9v03x_init())                                                      // 失败返回非0
        {
            ips114_show_string(0, 5 * 16, "Retry...");
            system_delay_ms(500);
        }
        else
        {
            ips114_show_string(0, 5 * 16, "Camera OK!     ");
            break;
        }
    }

    // ---- 第6步：初始化按键模块（5ms扫描周期） ----
    key_init(5);
    ips114_show_string(0, 6 * 16, "Keys OK!       ");

    // ---- 第7步：初始化巡线模块 ----
    line_follow_init();

    // ---- 第8步：初始化菜单系统（创建菜单树 + 绘制初始界面） ----
    menu_init();
    menu_show_All();

    // ---- 短暂延时让用户看到启动信息 ----
    system_delay_ms(300);

    // ==================== 主循环 ====================
    while(1)
    {
        // ==================== 按键扫描 ====================
        

        // ==================== 图像显示模式处理 ====================
        if(menu_in_image_mode)
        {
            // ---- IPS114 显示摄像头图像 ----
            menu_image_display_process();
        }
        else
        {
            // ==================== 菜单按键处理 ====================
            // 在非图像模式下，根据按键状态执行菜单操作

            // ---- KEY_1：上 / 增大参数 ----
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                key_clear_state(KEY_1);
                Menu_upFuntion(0);                                              // 短按
            }
            else if(key_get_state(KEY_1) == KEY_LONG_PRESS)
            {
                key_clear_state(KEY_1);
                Menu_upFuntion(1);                                              // 长按（编辑模式下10倍快速调节）
            }

            // ---- KEY_2：下 / 减小参数 ----
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                key_clear_state(KEY_2);
                Menu_downFuntion(0);                                            // 短按
            }
            else if(key_get_state(KEY_2) == KEY_LONG_PRESS)
            {
                key_clear_state(KEY_2);
                Menu_downFuntion(1);                                            // 长按（编辑模式下10倍快速调节）
            }

            // ---- KEY_3：确认 / 进入 / 选中编辑 ----
            if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
            {
                key_clear_state(KEY_3);
                Menu_enterFuntion();
            }

            // ---- KEY_4：返回 / 取消编辑 / 切换步进 ----
            if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
            {
                key_clear_state(KEY_4);
                Menu_quitFuntion();
            }

            // ---- 刷新菜单显示 ----
            if(menu_need_refresh)
            {
                menu_show_All();
            }

            // ==================== 巡线处理 ====================
            // 非图像模式下执行巡线（图像处理管线消费 mt9v03x_finish_flag）
            line_follow_process();

            // ---- 后续 PID 控制可在此添加 ----
            // if(line_data_ready)
            // {
            //     int16 error = calc_deviation(15);
            //     // servo_pid_control(error);
            //     // motor_pid_control(error);
            // }
        }
    }
}




void pit_handler (void)
{
        key_scanner();
}