/*********************************************************************************************************************
* 文件名称          bluetooth
* 功能描述          智能车摄像头扫描巡线 - HC-04 蓝牙透传模块实现
* 适用平台          MM32F327X_G8P
*
* HC-04 工作原理：
*   配对成功后，模块进入透传模式。MCU 通过 UART TX 发送的数据
*   会无线传输到手机，手机端（串口助手APP）即可接收。
*   手机发送的数据会从 MCU UART RX 输出。
*
* 手机端：
*   下载"蓝牙串口助手"类 APP，配对 HC-04（密码通常 1234 或 0000）
*   连接后即可收发数据
*********************************************************************************************************************/

#include "bluetooth.h"
#include <stdarg.h>
#include <stdio.h>

//==================================================== 蓝牙初始化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：bluetooth_init
// 功能：初始化 HC-04 蓝牙模块对应的 UART
// 说明：HC-04 上电后约 2 秒进入 AT 模式（可配参数），之后自动进入透传模式
//       本函数只初始化 MCU 端 UART，不配置 HC-04 本身
//       HC-04 出厂默认：9600 波特率，无需额外配置
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_init(void)
{
    // 初始化 UART：串口号、波特率、TX 引脚、RX 引脚
    uart_init(BLUETOOTH_UART, BLUETOOTH_BAUD, BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
}

//==================================================== 蓝牙发送 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：serial_printf
// 功能：通过蓝牙向手机发送格式化字符串（用法完全同 printf）
// 参数：fmt —— 格式化字符串
// 参数：... —— 可变参数列表
// 说明：
//   内部缓冲 256 字节，超出会自动截断
//   使用 uart_write_string 通过 UART 发送
//
// 示例：
//   serial_printf("Hello from smart car!\r\n");
//   serial_printf("servo=%.1f motor_L=%d motor_R=%d\r\n", angle, L, R);
//   serial_printf("OTSU threshold: %u\r\n", otsu_threshold);
//-------------------------------------------------------------------------------------------------------------------
void serial_printf(const char *fmt, ...)
{
    char buf[256];                                                              // 格式化缓冲区
    va_list args;
    va_start(args, fmt);
    vsprintf(buf, fmt, args);                                                   // 格式化字符串
    va_end(args);
    uart_write_string(BLUETOOTH_UART, buf);                                     // 通过 UART 发送
}

//==================================================== 蓝牙接收（调参命令解析） ====================================================

// 外部PID参数（定义在 common_Mymenu.c 中）
extern float servo_kp;
extern float servo_ki;
extern float servo_kd;

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：bluetooth_receive_process
// 功能：读取蓝牙接收到的数据，解析 [slider,参数名,值] 格式的调参命令
// 参数：void
// 返回：void
//
// 命令格式：
//   [slider,servo_kp,0.5]   → 设置 servo_kp = 0.5
//   [slider,servo_ki,0.01]  → 设置 servo_ki = 0.01
//   [slider,servo_kd,0.32]  → 设置 servo_kd = 0.32
//
// 接收采用逐字节轮询，状态机解析
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_receive_process(void)
{
    static char buf[80];
    static uint8 idx = 0;
    uint8 byte;

    // 所有字节无脑累积，收到 ] 时往前搜 [ 然后解析
    while(uart_query_byte(BLUETOOTH_UART, &byte))
    {
        if(idx < sizeof(buf) - 1)
            buf[idx++] = byte;

        // 缓冲区溢出保护
        if(idx >= sizeof(buf) - 2)
        {
            idx = 0;                                                            // 清空重来
            continue;
        }

        if(byte == ']')
        {
            buf[idx] = '\0';

            // 向前搜索最近的 '['
            int16 start = -1;
            int16 i;
            for(i = idx - 2; i >= 0; i--)
            {
                if(buf[i] == '[')
                {
                    start = i;
                    break;
                }
            }

            if(start >= 0)
            {
                char name[16];
                float val;
                if(sscanf(buf + start, "[slider,%15[^,],%f]", name, &val) == 2)
                {
                    if(strcmp(name, "servo_kp") == 0)
                    {
                        servo_kp = val;
                        serial_printf("OK kp=%.3f\r\n", servo_kp);
                    }
                    else if(strcmp(name, "servo_ki") == 0)
                    {
                        servo_ki = val;
                        serial_printf("OK ki=%.3f\r\n", servo_ki);
                    }
                    else if(strcmp(name, "servo_kd") == 0)
                    {
                        servo_kd = val;
                        serial_printf("OK kd=%.3f\r\n", servo_kd);
                    }
                    else
                    {
                        serial_printf("ERR %s\r\n", name);
                    }
                }
            }
            idx = 0;                                                            // 解析完清缓冲
        }
    }
}
