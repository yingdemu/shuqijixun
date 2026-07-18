/*********************************************************************************************************************
* 文件名称          common_Mymenu
* 功能描述          智能车摄像头扫描巡线 - 菜单内容定义与按键交互实现
* 适用平台          MM32F327X_G8P
* 说明              移植自 WwuSama 多级菜单 v5.0.0，改为 IPS200 屏幕 + 四键交互
*
* 按键操作：
*   KEY_1 → 光标上移 / 参数增大（编辑模式下长按=10倍快速增加）
*   KEY_2 → 光标下移 / 参数减小（编辑模式下长按=10倍快速减小）
*   KEY_3 → 确认 / 进入子菜单 / 选中编辑 / 保存退出编辑
*   KEY_4 → 返回上级 / 取消编辑 / 切换步进值 / 退出图像模式
*
* IPS200 屏幕布局（320×240，8×16字体）：
*   第0行  (y=0)   ：标题栏（蓝底白字，显示当前路径）
*   第1~13行       ：菜单项列表（-> 标记当前选中，参数值右对齐）
*   第14行 (y=224) ：操作提示栏（绿底黑字，显示按键功能）
*********************************************************************************************************************/

#include "common_Mymenu.h"
#include "image_process.h"
#include <string.h>
#include <stdio.h>

// 外部变量（定义在 image_process.c 中）
extern uint8 threshold_mode;
extern uint8 fixed_threshold;

//==================================================== PID参数变量定义 ====================================================

// ---- 舵机PID控制参数 ----
// 舵机PID用于控制前轮转向角度
// servo_kp: 比例系数 —— 根据位置偏差进行比例调节
// servo_ki: 积分系数 —— 消除稳态误差
// servo_kd: 微分系数 —— 抑制振荡和超调
float servo_kp = 0.29f;                                                          // 舵机 Kp 默认 0.80
float servo_ki = 0.0f;                                                          // 舵机 Ki 默认 0.0
float servo_kd = 0.32f;                                                          // 舵机 Kd 默认 0.32
float servo_lowpass = 0.8f;                                                       // 舵机低通滤波系数（默认 0.8）
// ---- 电机PID控制参数 ----
// 电机PID用于控制后轮驱动速度
float motor_kp = 1.0f;                                                          // 电机 Kp 默认 1.0
float motor_ki = 0.1f;                                                          // 电机 Ki 默认 0.1
float motor_kd = 0.0f;                                                          // 电机 Kd 默认 0.0

//-----发车标志位-----
bool car_go_flag = 0;                                                            // 发车标志位（1=开始巡线，0=停止巡线）
uint8 motor_duty = 13;                                                             //电机占空比
//==================================================== 菜单全局变量 ====================================================

Folder_Menu myMenu;                                                             // 菜单根节点（主菜单）
Folder_Menu *key_menu_p = NULL;                                                 // 当前光标所在的菜单节点
uint8 menu_in_image_mode = 0;                                                   // 图像显示模式标志
uint8 menu_need_refresh = 1;                                                    // 菜单刷新标志
uint8 menu_need_clear = 0;                                                      // 清屏标志（主循环执行 ips200_clear）

//==================================================== 编辑模式变量 ====================================================

static uint8  menu_is_editing = 0;                                              // 编辑模式标志（1=正在编辑参数值）
static float edit_saved_value = 0.0f;                                           // 编辑前保存的原始值（用于取消时恢复）

//==================================================== 步进值配置 ====================================================

// 步进值数组：在参数编辑未选中时，按 KEY_4 切换当前步进值
// 编辑模式下按 KEY_1/KEY_2 以当前步进值增减参数
static float SetupNumber[SETUP_LEN] = {0.001f, 0.01f, 0.1f, 1.0f, 10.0f, 100.0f, 1000.0f};
static uint8 SetupIndex = 2;                                                    // 默认步进值 = 0.1

//==================================================== 菜单树创建 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：my_create_Menus
// 功能：创建完整的菜单树结构（内部函数，由 menu_init 调用）
//
// 菜单结构：
//   myMenu (根节点 "<Menu>")
//     ├── image (文件夹，进入后IPS200显示摄像头图像)
//     ├── servo_pid (文件夹)
//     │   ├── servo_kp (float, 限幅 0.00~100.00)
//     │   ├── servo_ki (float, 限幅 0.00~10.00)
//     │   └── servo_kd (float, 限幅 0.00~100.00)
//     └── motor_pid (文件夹)
//         ├── motor_kp (float, 限幅 0.00~100.00)
//         ├── motor_ki (float, 限幅 0.00~10.00)
//         └── motor_kd (float, 限幅 0.00~100.00)
//-------------------------------------------------------------------------------------------------------------------
static void my_create_Menus(void)
{
    // ==================== 第一层：主菜单项 ====================

    // image 文件夹 —— 进入后 IPS200 显示摄像头灰度图像
    Folder_Menu *image_folder = dynamicCreate_Menu_Folder(&myMenu, "image");

    // servo_pid 文件夹 —— 舵机PID参数子菜单
    Folder_Menu *servo_pid_folder = dynamicCreate_Menu_Folder(&myMenu, "servo_pid");

    // motor_pid 文件夹 —— 电机PID参数子菜单
    Folder_Menu *motor_pid_folder = dynamicCreate_Menu_Folder(&myMenu, "motor_pid");


    //=========================第一层，发车标志位===============================

    dynamicCreate_Menu_NumberBox(&myMenu, "car_go", &car_go_flag, bool_Box);

    //=========================第一层，阈值模式选择===============================

    dynamicCreate_Menu_NumberBox(&myMenu, "thresh_mode", &threshold_mode, bool_Box);
    dynamicCreate_Menu_LimitNumberBox(&myMenu, "fix_thresh", &fixed_threshold, uint8_Box, 0, 255);
    dynamicCreate_Menu_LimitNumberBox(&myMenu, "motor_duty", &motor_duty,uint8_Box , 0, 30);


    // ==================== 第二层：servo_pid 子菜单 ====================

    dynamicCreate_Menu_LimitNumberBox(servo_pid_folder, "servo_kp", &servo_kp, float_Box, -10.0f, 10.0f);
    dynamicCreate_Menu_LimitNumberBox(servo_pid_folder, "servo_ki", &servo_ki, float_Box, -10.0f, 10.0f);
    dynamicCreate_Menu_LimitNumberBox(servo_pid_folder, "servo_kd", &servo_kd, float_Box, -10.0f, 10.0f);
    dynamicCreate_Menu_LimitNumberBox(servo_pid_folder, "servo_lowpass", &servo_lowpass, float_Box, 0.0f, 1.0f);

    // ==================== 第二层：motor_pid 子菜单 ====================

    dynamicCreate_Menu_LimitNumberBox(motor_pid_folder, "motor_kp", &motor_kp, float_Box, 0.0f, 100.0f);
    dynamicCreate_Menu_LimitNumberBox(motor_pid_folder, "motor_ki", &motor_ki, float_Box, 0.0f, 10.0f);
    dynamicCreate_Menu_LimitNumberBox(motor_pid_folder, "motor_kd", &motor_kd, float_Box, 0.0f, 100.0f);


}

//==================================================== 菜单初始化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：menu_init
// 功能：初始化整个菜单系统
//   1. 初始化根节点 myMenu
//   2. 调用 my_create_Menus() 创建所有菜单项
//   3. 设置 key_menu_p 指向第一个子节点
//   4. 调用 All_Folder_Menu_Init() 将兄弟链表转为环形链表
//   5. 标记需要刷新屏幕
//-------------------------------------------------------------------------------------------------------------------
void menu_init(void)
{
    // ---- 1. 初始化根节点 ----
    myMenu.father = NULL;
    myMenu.son_first = NULL;
    myMenu.next_brother = NULL;
    myMenu.last_brother = NULL;
    myMenu.name = "<Menu>";
    myMenu.sons_Count = 0;
    myMenu.No = 0;
    myMenu.kind = Normal_Folder;

    // ---- 2. 创建所有菜单项 ----
    my_create_Menus();

    // ---- 3. 将光标指向第一个子节点 ----
    if(myMenu.son_first != NULL)
        key_menu_p = myMenu.son_first;

    // ---- 4. 将兄弟链表转换为环形链表 ----
    All_Folder_Menu_Init(&myMenu);

    // ---- 5. 标记需要刷新 ----
    menu_need_clear = 1;                                                        // 首次绘制前清屏（后续导航不再全清）
    menu_need_refresh = 1;
    menu_is_editing = 0;
    menu_in_image_mode = 0;
}

//==================================================== IPS200 菜单显示 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Mymenu_show_title
// 功能：在 IPS200 屏幕顶部绘制标题栏（蓝底白字，显示当前路径面包屑）
//
// 路径构建：从根节点到当前节点的路径
//   例如：<Menu> / servo_pid /
// 如果路径太长则只显示最后两级
//-------------------------------------------------------------------------------------------------------------------
static void Mymenu_show_title(void)
{
    char path_buf[32];                                                          // 路径字符串缓冲区
    Folder_Menu *p;
    uint8 depth = 0;
    char *names[8];                                                             // 最多8层深度

    // ---- 从当前节点向上回溯，收集所有父节点名称 ----
    p = key_menu_p;
    while(p != NULL && depth < 8)
    {
        names[depth++] = (char *)p->name;
        p = p->father;
    }

    // ---- 构建路径字符串（从根到当前） ----
    path_buf[0] = '\0';
    for(int8 i = depth - 1; i >= 0; i--)
    {
        if(strlen(path_buf) + strlen(names[i]) + 3 < 34)
        {
            strcat(path_buf, names[i]);
            if(i > 0) strcat(path_buf, " / ");
        }
    }

    // ---- 绘制标题栏背景（蓝色填充，一整行） ----
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);
    ips200_show_string(0, MENU_TITLE_Y, "                                        ");  // 40个空格清空整行（320/8=40字）
    ips200_show_string(2, MENU_TITLE_Y, path_buf);                              // 显示路径文字
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);                                // 恢复默认配色（黑字白底）
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Mymenu_show_Setup
// 功能：在标题行右侧显示当前步进值
//-------------------------------------------------------------------------------------------------------------------
static void Mymenu_show_Setup(void)
{
    char buf[12];
    ips200_set_color(RGB565_WHITE, RGB565_BLUE);                                 // 蓝底白字（与标题栏一致）
    if(SetupNumber[SetupIndex] < 1.0f)
        sprintf(buf, "stp:%.3f", SetupNumber[SetupIndex]);
    else
        sprintf(buf, "stp:%.0f ", SetupNumber[SetupIndex]);
    ips200_show_string(240, MENU_TITLE_Y, buf);                                  // 显示在屏幕右侧（IPS200 320宽）
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Mymenu_show_footer
// 功能：在 IPS200 底部绘制操作提示栏
// 说明：根据当前状态显示不同的按键功能提示
//-------------------------------------------------------------------------------------------------------------------
static void Mymenu_show_footer(void)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);

    if(menu_is_editing)                                                         // 编辑模式（24字/192px < 240）
    {
        ips200_show_string(0, MENU_FOOTER_Y,
            "K1:+ K2:- K3:Save K4:Cancel ");
    }
    else                                                                        // 普通菜单模式
    {
        ips200_show_string(0, MENU_FOOTER_Y,
            "K1:Up K2:Dn K3:OK K4:Back   ");
    }
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Mymenu_show_task
// 功能：在 IPS200 上绘制当前层级的所有菜单项
// 说明：
//   - 遍历当前父节点下的所有子节点（兄弟链表）
//   - "->" 箭头标记当前选中的项
//   - 文件夹显示名称 + [子项数]
//   - 参数项显示名称 + 数值（编辑中被选中时显示 <值>）
//   - 如果菜单项超过屏幕可显示行数，支持滚动
//-------------------------------------------------------------------------------------------------------------------
static void Mymenu_show_task(void)
{
    Folder_Menu *son_p = key_menu_p->father->son_first;                         // 从第一个子节点开始
    uint8 sons_count = key_menu_p->father->sons_Count;                          // 同级节点总数
    uint8 display_start = 0;                                                    // 从第几个开始显示（滚动偏移）

    // ---- 计算滚动偏移（当菜单项超过可显示行数时） ----
    if(sons_count > MENU_MAX_DISPLAY_ITEMS)
    {
        // 让选中项尽量在可见区域中间
        if(key_menu_p->No > MENU_MAX_DISPLAY_ITEMS / 2
           && key_menu_p->No <= sons_count - MENU_MAX_DISPLAY_ITEMS / 2)
        {
            display_start = key_menu_p->No - MENU_MAX_DISPLAY_ITEMS / 2 - 1;
        }
        else if(key_menu_p->No > sons_count - MENU_MAX_DISPLAY_ITEMS / 2)
        {
            display_start = sons_count - MENU_MAX_DISPLAY_ITEMS;
        }
    }

    // ---- 遍历并显示菜单项 ----
    for(uint8 i = 0; i < sons_count && i < MENU_MAX_DISPLAY_ITEMS; i++)
    {
        uint8 item_no = i + display_start + 1;                                  // 当前要显示的节点编号（从1开始）
        uint16 y_pos = MENU_START_Y + i * FONT_H;                                // Y 坐标

        // ---- 找到编号为 item_no 的节点 ----
        Folder_Menu *disp_p = son_p;
        for(uint8 n = 1; n < item_no && disp_p != NULL; n++)
            disp_p = disp_p->next_brother;

        if(disp_p == NULL)
            break;

        // ---- 清空该行 ----
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_show_string(0, y_pos, "                                        ");

        // ---- 光标指示和名称 ----
        if(disp_p == key_menu_p)                                                // 当前选中项
        {
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);                       // 反白显示（白字黑底）
            ips200_show_string(0, y_pos, "->");
        }
        else
        {
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);                       // 正常显示
            ips200_show_string(0, y_pos, "  ");
        }

        // ---- 显示菜单项内容 ----
        char line_buf[36];                                                      // 行缓冲区（IPS200: 40字/行）

        if(disp_p->kind == Normal_Folder)                                       // 文件夹类型
        {
            sprintf(line_buf, "%-16s [%d]            ",
                    disp_p->name, disp_p->sons_Count);
        }
        else                                                                    // 数据项类型
        {
            switch(disp_p->kind)
            {
                case bool_Box:
                {
                    if(*(uint8 *)disp_p->private_data)
                        sprintf(line_buf, "%-16s : Y            ", disp_p->name);
                    else
                        sprintf(line_buf, "%-16s : N            ", disp_p->name);
                    break;
                }
                case float_Box:
                {
                    float val = *(float *)disp_p->private_data;
                    int16 int_part = (int16)val;
                    uint16 dec_part = (uint16)((val - int_part) * 100);
                    if(dec_part < 10)
                    {
                        if(disp_p->number_box_select)                           // 编辑中显示尖括号
                            sprintf(line_buf, "%-14s:<%5d.0%d>      ", disp_p->name, int_part, dec_part);
                        else
                            sprintf(line_buf, "%-14s: %5d.0%d       ", disp_p->name, int_part, dec_part);
                    }
                    else
                    {
                        if(disp_p->number_box_select)
                            sprintf(line_buf, "%-14s:<%5d.%d>      ", disp_p->name, int_part, dec_part);
                        else
                            sprintf(line_buf, "%-14s: %5d.%d       ", disp_p->name, int_part, dec_part);
                    }
                    break;
                }
                case int_Box:
                {
                    if(disp_p->number_box_select)
                        sprintf(line_buf, "%-14s:<%8d>       ", disp_p->name, *(int *)disp_p->private_data);
                    else
                        sprintf(line_buf, "%-14s: %8d        ", disp_p->name, *(int *)disp_p->private_data);
                    break;
                }
                case uint8_Box:
                {
                    if(disp_p->number_box_select)
                        sprintf(line_buf, "%-14s:<%8u>       ", disp_p->name, *(uint8 *)disp_p->private_data);
                    else
                        sprintf(line_buf, "%-14s: %8u        ", disp_p->name, *(uint8 *)disp_p->private_data);
                    break;
                }
                case uint16_Box:
                {
                    if(disp_p->number_box_select)
                        sprintf(line_buf, "%-14s:<%8u>       ", disp_p->name, *(uint16 *)disp_p->private_data);
                    else
                        sprintf(line_buf, "%-14s: %8u        ", disp_p->name, *(uint16 *)disp_p->private_data);
                    break;
                }
                case uint32_Box:
                {
                    if(disp_p->number_box_select)
                        sprintf(line_buf, "%-14s:<%8lu>      ", disp_p->name, *(uint32 *)disp_p->private_data);
                    else
                        sprintf(line_buf, "%-14s: %8lu       ", disp_p->name, *(uint32 *)disp_p->private_data);
                    break;
                }
                default:
                {
                    sprintf(line_buf, "%-14s: ?            ", disp_p->name);
                    break;
                }
            }
        }

        // ---- 在屏幕上显示该行 ----
        ips200_show_string(FONT_W * 2, y_pos, line_buf);

        // ---- 恢复默认配色 ----
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：menu_show_All
// 功能：在 IPS200 上绘制完整的菜单界面
// 说明：清屏后依次绘制标题栏、步进值、菜单项列表、底部操作提示
//-------------------------------------------------------------------------------------------------------------------
void menu_show_All(void)
{
    if(menu_in_image_mode)                                                      // 图像显示模式下不绘制菜单
        return;

    // 不再 ips200_clear() 全屏填充（76800像素SPI传输~20ms太慢）
    // 改为逐行清+写，每行已经用40空格覆盖后再写新内容

    Mymenu_show_title();                                                        // 标题栏
    Mymenu_show_Setup();                                                        // 步进值显示
    Mymenu_show_task();                                                         // 菜单项列表（含逐行清空）
    Mymenu_show_footer();                                                       // 底部操作提示

    // 清除剩余未使用的空行（防止上次显示残留）
    {
        uint8 sons_count = key_menu_p->father->sons_Count;
        if(sons_count < MENU_MAX_DISPLAY_ITEMS)
        {
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);
            for(uint8 i = sons_count; i < MENU_MAX_DISPLAY_ITEMS; i++)
            {
                ips200_show_string(0, MENU_START_Y + i * FONT_H,
                                   "                                        ");
            }
        }
    }

    menu_need_refresh = 0;                                                      // 清除刷新标志
}

//==================================================== 菜单导航函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 光标上移（移向上一个兄弟节点，环形链表）
//-------------------------------------------------------------------------------------------------------------------
static void menu_function_up(void)
{
    key_menu_p = key_menu_p->last_brother;
}

//-------------------------------------------------------------------------------------------------------------------
// 光标下移（移向下一个兄弟节点，环形链表）
//-------------------------------------------------------------------------------------------------------------------
static void menu_function_down(void)
{
    key_menu_p = key_menu_p->next_brother;
}

//-------------------------------------------------------------------------------------------------------------------
// 进入子菜单
//-------------------------------------------------------------------------------------------------------------------
static void menu_function_enter(void)
{
    if(key_menu_p->son_first == NULL) return;
    key_menu_p = key_menu_p->son_first;
}

//-------------------------------------------------------------------------------------------------------------------
// 返回上级菜单
//-------------------------------------------------------------------------------------------------------------------
static void menu_function_quit(void)
{
    if(key_menu_p->father->father == NULL) return;                              // 已经是根的直接子节点
    key_menu_p = key_menu_p->father;
}

//-------------------------------------------------------------------------------------------------------------------
// 切换布尔值
//-------------------------------------------------------------------------------------------------------------------
static void menu_function_toggle(void)
{
    *(uint8 *)(key_menu_p->private_data) = !(*(uint8 *)(key_menu_p->private_data));
}

//-------------------------------------------------------------------------------------------------------------------
// 进入编辑模式（选中数值项）
//-------------------------------------------------------------------------------------------------------------------
static void menu_function_select(void)
{
    key_menu_p->number_box_select = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 退出编辑模式（取消选中数值项）
//-------------------------------------------------------------------------------------------------------------------
static void menu_function_unselect(void)
{
    key_menu_p->number_box_select = 0;
}

//==================================================== 步进值切换 ====================================================

static void MenuNumber_SetupCtrl_Plus(void)
{
    SetupIndex = (SetupIndex + 1) % SETUP_LEN;
}

//==================================================== 数值增减 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 数值增加一个步进值
//-------------------------------------------------------------------------------------------------------------------
static void menu_funtion_NumberPlus(void)
{
    float step = SetupNumber[SetupIndex];

    switch(key_menu_p->kind)
    {
        case float_Box:
        {
            float *val = (float *)key_menu_p->private_data;
            *val += step;
            if(key_menu_p->isLimit && *val > key_menu_p->limit_max)
                *val = key_menu_p->limit_max;
            break;
        }
        case int_Box:
        {
            int *val = (int *)key_menu_p->private_data;
            *val += (int)step;
            if(key_menu_p->isLimit && *val > (int)key_menu_p->limit_max)
                *val = (int)key_menu_p->limit_max;
            break;
        }
        case uint8_Box:
        {
            uint8 *val = (uint8 *)key_menu_p->private_data;
            *val += (uint8)step;
            if(key_menu_p->isLimit && *val > (uint8)key_menu_p->limit_max)
                *val = (uint8)key_menu_p->limit_max;
            break;
        }
        case uint16_Box:
        {
            uint16 *val = (uint16 *)key_menu_p->private_data;
            *val += (uint16)step;
            if(key_menu_p->isLimit && *val > (uint16)key_menu_p->limit_max)
                *val = (uint16)key_menu_p->limit_max;
            break;
        }
        case uint32_Box:
        {
            uint32 *val = (uint32 *)key_menu_p->private_data;
            *val += (uint32)step;
            if(key_menu_p->isLimit && *val > (uint32)key_menu_p->limit_max)
                *val = (uint32)key_menu_p->limit_max;
            break;
        }
        default: break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 数值减小一个步进值
//-------------------------------------------------------------------------------------------------------------------
static void menu_funtion_NumberSub(void)
{
    float step = SetupNumber[SetupIndex];

    switch(key_menu_p->kind)
    {
        case float_Box:
        {
            float *val = (float *)key_menu_p->private_data;
            *val -= step;
            if(key_menu_p->isLimit && *val < key_menu_p->limit_min)
                *val = key_menu_p->limit_min;
            break;
        }
        case int_Box:
        {
            int *val = (int *)key_menu_p->private_data;
            *val -= (int)step;
            if(key_menu_p->isLimit && *val < (int)key_menu_p->limit_min)
                *val = (int)key_menu_p->limit_min;
            break;
        }
        case uint8_Box:
        {
            uint8 *val = (uint8 *)key_menu_p->private_data;
            if(key_menu_p->isLimit && *val < (uint8)key_menu_p->limit_min + (uint8)step)
                *val = (uint8)key_menu_p->limit_min;
            else
                *val -= (uint8)step;
            break;
        }
        case uint16_Box:
        {
            uint16 *val = (uint16 *)key_menu_p->private_data;
            if(key_menu_p->isLimit && *val < (uint16)key_menu_p->limit_min + (uint16)step)
                *val = (uint16)key_menu_p->limit_min;
            else
                *val -= (uint16)step;
            break;
        }
        case uint32_Box:
        {
            uint32 *val = (uint32 *)key_menu_p->private_data;
            if(key_menu_p->isLimit && *val < (uint32)key_menu_p->limit_min + (uint32)step)
                *val = (uint32)key_menu_p->limit_min;
            else
                *val -= (uint32)step;
            break;
        }
        default: break;
    }
}

//==================================================== 四键控制逻辑 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// KEY_1：上 / 增大参数
//
// 编辑模式下的逻辑：
//   如果当前参数项处于编辑选中状态（number_box_select == 1）：
//     短按 KEY_1 → 参数值增加一个步进值
//     长按 KEY_1 → 参数值增加 10 倍步进值（快速调节）
//   如果当前参数项未处于编辑状态 → 光标上移
//
// 非编辑模式 → 光标上移
//-------------------------------------------------------------------------------------------------------------------
void Menu_upFuntion(uint8 is_long_press)
{
    // ---- 编辑模式：增大参数值 ----
    if(menu_is_editing
       && key_menu_p->number_box_select == 1
       && key_menu_p->kind != Normal_Folder
       && key_menu_p->kind != bool_Box)
    {
        if(is_long_press)                                                       // 长按 → 10倍快速调节
        {
            float saved_step = SetupNumber[SetupIndex];
            SetupNumber[SetupIndex] = saved_step * 10.0f;
            menu_funtion_NumberPlus();
            SetupNumber[SetupIndex] = saved_step;
        }
        else                                                                    // 短按 → 正常步进
        {
            menu_funtion_NumberPlus();
        }
        menu_need_refresh = 1;
        return;
    }

    // ---- 普通模式：光标上移 ----
    menu_function_up();
    menu_need_refresh = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// KEY_2：下 / 减小参数
//-------------------------------------------------------------------------------------------------------------------
void Menu_downFuntion(uint8 is_long_press)
{
    // ---- 编辑模式：减小参数值 ----
    if(menu_is_editing
       && key_menu_p->number_box_select == 1
       && key_menu_p->kind != Normal_Folder
       && key_menu_p->kind != bool_Box)
    {
        if(is_long_press)                                                       // 长按 → 10倍快速调节
        {
            float saved_step = SetupNumber[SetupIndex];
            SetupNumber[SetupIndex] = saved_step * 10.0f;
            menu_funtion_NumberSub();
            SetupNumber[SetupIndex] = saved_step;
        }
        else
        {
            menu_funtion_NumberSub();
        }
        menu_need_refresh = 1;
        return;
    }

    // ---- 普通模式：光标下移 ----
    menu_function_down();
    menu_need_refresh = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// KEY_3：确认 / 进入 / 选中编辑 / 保存退出编辑
//
// 逻辑：
//   - 文件夹类型 → 进入子菜单
//   - "image" 文件夹 → 进入图像显示模式（IPS200显示摄像头）
//   - bool_Box → 切换 true/false
//   - 数值类型未编辑 → 进入编辑模式（选中该参数）
//   - 数值类型已在编辑 → 退出编辑模式（保存当前值）
//-------------------------------------------------------------------------------------------------------------------
void Menu_enterFuntion(void)
{
    // ---- 如果正在编辑参数，KEY_3 确认保存并退出编辑 ----
    if(menu_is_editing && key_menu_p->number_box_select == 1)
    {
        menu_is_editing = 0;
        menu_function_unselect();
        menu_need_refresh = 1;
        return;
    }

    switch(key_menu_p->kind)
    {
        case Normal_Folder:
        {
            // ---- 特殊处理：进入 "image" 文件夹 → 图像显示模式 ----
            if(strcmp(key_menu_p->name, "image") == 0)
            {
                menu_need_clear = 1;                                            // 主循环会执行 ips200_clear()
                menu_in_image_mode = 1;
                return;
            }
            // ---- 普通文件夹 → 进入子菜单 ----
            menu_function_enter();
            menu_need_refresh = 1;
            break;
        }

        case bool_Box:
        {
            menu_function_toggle();
            menu_need_refresh = 1;
            break;
        }

        default:                                                                // 数值类型
        {
            // ---- 进入编辑模式 ----
            menu_function_select();
            menu_is_editing = 1;
            menu_need_refresh = 1;
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// KEY_4：返回上级 / 取消编辑 / 切换步进 / 退出图像模式
//
// 逻辑：
//   - 图像显示模式 → 退出图像模式
//   - 编辑模式（参数被选中）→ 取消编辑（恢复原始值）
//   - 文件夹/普通状态 → 返回上级菜单
//   - 如果光标在数值项上但未进入编辑 → 切换步进值
//-------------------------------------------------------------------------------------------------------------------
void Menu_quitFuntion(void)
{
    // ---- 图像显示模式 → 退出 ----
    if(menu_in_image_mode)
    {
        menu_in_image_mode = 0;
        menu_need_clear = 1;                                                    // 主循环执行 ips200_clear()
        menu_need_refresh = 1;

        menu_function_quit();                                                        // 返回上级菜单//===================================自己加的======================================
        return;
    }

    // // ---- 编辑模式（参数被选中） → 取消编辑 ----
    // if(menu_is_editing && key_menu_p->number_box_select == 1)
    // {
    //     menu_is_editing = 0;
    //     menu_function_unselect();
    //     menu_need_refresh = 1;
    //     return;
    // }

    // ---- 文件夹或未编辑状态 → 返回上级 ----
    switch(key_menu_p->kind)
    {
        case Normal_Folder:
        case bool_Box:
        {
            menu_function_quit();
            menu_need_refresh = 1;
            break;
        }

        default:                                                                // 数值类型（未编辑）
        {
            if(key_menu_p->number_box_select==0){
                menu_function_quit();
                menu_need_refresh = 1;
                break;
            }
            // 如果光标在数值项上但未进入编辑 → 切换步进值
            MenuNumber_SetupCtrl_Plus();
            menu_need_refresh = 1;
            break;
        }
    }
}

//==================================================== 图像显示模式 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：menu_image_display_process
// 功能：图像显示模式处理（在主循环中调用）
// 说明：
//   1. 检测 mt9v03x_finish_flag，有新图像帧时在 IPS200 上显示
//   2. 图像原始尺寸 141×90，在 IPS200 上缩放显示为 240×100（非等比缩放）
//   3. 在屏幕底部显示退出提示
//   4. 检测 KEY_4 退出图像模式
//-------------------------------------------------------------------------------------------------------------------
void menu_image_display_process(void)
{
    // ---- 检测是否有新图像帧（按键退出由中断中的 menu_key_process 统一处理） ----
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        // ---- 执行完整图像处理管线（大津法→二值化→画框→爬线→ABCD→补线→中线） ----
        image_process_pipeline();

        // ---- 上半屏：显示处理后的二值化图像 240×100（含补线、黑框） ----
        ips200_show_gray_image(0, 0, (const uint8 *)binary_image,
                               IMG_W, IMG_H,              // 源图 141×90
                               240, 100,                   // 显示 240×100
                               1);                         // 二值化阈值=1

        // ---- 在二值化图像上叠加赛道中线 + 左右边界 ----
        // 坐标从图像坐标系(141×90)映射到显示坐标系(240×100)
        for(int16 r = 1; r < IMG_H; r++)
        {
            uint16 y1 = (r - 1) * 100 / IMG_H;
            uint16 y2 = r * 100 / IMG_H;

            // 中线（红色）
            ips200_draw_line(center_line[r - 1] * 240 / IMG_W, y1,
                             center_line[r] * 240 / IMG_W,     y2, RGB565_RED);

            // 左边界（蓝色）
            ips200_draw_line(left_boundary[r - 1] * 240 / IMG_W, y1,
                             left_boundary[r] * 240 / IMG_W,     y2, RGB565_BLUE);

            // 右边界（绿色）
            ips200_draw_line(right_boundary[r - 1] * 240 / IMG_W, y1,
                             right_boundary[r] * 240 / IMG_W,     y2, RGB565_GREEN);
        }

        // ---- 分隔线 ----
        ips200_draw_line(0, 102, 239, 102, RGB565_RED);

        // ---- 下半屏：显示原始灰度图像 240×100（对比用） ----
        ips200_show_gray_image(0, 106, (const uint8 *)mt9v03x_image,
                               MT9V03X_W, MT9V03X_H,      // 源图 141×90
                               240, 100,                   // 显示 240×100
                               0);                         // 阈值=0=灰度模式

        // ---- 灰度图下方显示加权位置和阈值（定位在 footer 上方） ----
        {
            char info_buf[40];
            float pos = get_weight_position(center_line);
            sprintf(info_buf, "pos:%.1f  OTSU:%3u", pos, otsu_threshold);
            ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
            ips200_show_string(0, MENU_FOOTER_Y - 32, info_buf);
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        }

        // ---- 屏幕底部显示退出提示 ----
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(0, MENU_FOOTER_Y, "K4:Exit Image                ");
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    }
}

//==================================================== 按键处理（中断中调用） ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：menu_key_process
// 功能：统一按键处理（放在中断中与 key_scanner 同频调用）
// 说明：
//   本函数只修改菜单数据结构和状态标志，不执行任何 SPI 显示操作。
//   显示更新（ips200_clear / menu_show_All）由主循环根据标志位执行。
//
//   按键映射：
//     KEY_1 → 光标上移 / 参数增大（长按=10倍快调）
//     KEY_2 → 光标下移 / 参数减小（长按=10倍快调）
//     KEY_3 → 确认 / 进入子菜单 / 选中编辑 / 保存退出编辑
//     KEY_4 → 返回上级 / 取消编辑 / 切换步进 / 退出图像模式
//-------------------------------------------------------------------------------------------------------------------
void menu_key_process(void)
{
    // ---- 图像显示模式：只响应 KEY_4 退出 ----
    if(menu_in_image_mode)
    {
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_4);
            menu_in_image_mode = 0;
            menu_need_clear = 1;                                                // 主循环执行 ips200_clear()
            menu_need_refresh = 1;
        }
        return;                                                                 // 图像模式下不处理其他按键
    }

    // ---- KEY_1：上 / 增大参数 ----
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);
        Menu_upFuntion(0);
    }
    else if(key_get_state(KEY_1) == KEY_LONG_PRESS)
    {
        key_clear_state(KEY_1);
        Menu_upFuntion(1);
    }

    // ---- KEY_2：下 / 减小参数 ----
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        Menu_downFuntion(0);
    }
    else if(key_get_state(KEY_2) == KEY_LONG_PRESS)
    {
        key_clear_state(KEY_2);
        Menu_downFuntion(1);
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
}
