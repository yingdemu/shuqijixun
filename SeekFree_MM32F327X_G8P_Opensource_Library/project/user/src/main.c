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
//     ├── image      → IPS200 显示摄像头灰度图像
//     ├── servo_pid  → servo_kp / servo_ki / servo_kd
//     └── motor_pid  → motor_kp / motor_ki / motor_kd
//
// ======================================================================

#include "zf_common_headfile.h"
#include "common_menu.h"
#include "common_Mymenu.h"
#include "line_follow.h"




#define IPS200_TYPE             (IPS200_TYPE_SPI)                     // 双排排针并口 → IPS200_TYPE_PARALLEL8
                                                                                // 单排排针 SPI → IPS200_TYPE_SPI
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

    // ---- 第7步：初始化巡线模块 ----
    line_follow_init();

    // ---- 第8步：初始化菜单系统（创建菜单树 + 绘制初始界面） ----
    menu_init();
    menu_show_All();

    // ---- 短暂延时让用户看到启动信息 ----
    system_delay_ms(300);

    //-------启动定时器（用来调试）
        // timer_init(TIM_2, TIMER_US);  
        //             timer_start(TIM_2);                                                         // 启动定时
 
    // ==================== 主循环 ====================
    while(1)
    {
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
        }
        else
        {
            // ==================== 巡线处理 ====================
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 周期中断回调（5ms），按键扫描 + 菜单按键处理
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    key_scanner();
    menu_key_process();
}