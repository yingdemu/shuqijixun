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
#include <string.h>

// 蓝牙接收 FIFO
static uint8 bt_rx_fifo_buf[128];
static fifo_struct bt_rx_fifo;

//==================================================== 蓝牙初始化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：bluetooth_init
// 功能：初始化 HC-04 蓝牙模块对应的 UART，开启 RX 中断 + FIFO
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_init(void)
{
    uart_init(BLUETOOTH_UART, BLUETOOTH_BAUD, BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
    fifo_init(&bt_rx_fifo, FIFO_DATA_8BIT, bt_rx_fifo_buf, sizeof(bt_rx_fifo_buf)); // 初始化接收FIFO

    // 注册 UART RX 中断回调（ISR 中 wireless_module_uart_handler 之后调用）
    extern callback_function wireless_module_uart_handler;
    wireless_module_uart_handler = bluetooth_uart_isr_callback;
    uart_rx_interrupt(BLUETOOTH_UART, 1);                                       // 确保RX中断开启
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：bluetooth_uart_isr_callback
// 功能：由 UART6 RX 中断调用，将接收字节存入 FIFO
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_uart_isr_callback(void)
{
    uint8 data;
    if(uart_query_byte(BLUETOOTH_UART, &data))
    {
        fifo_write_buffer(&bt_rx_fifo, &data, 1);
    }
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
    vsnprintf(buf, sizeof(buf), fmt, args);                                    // 格式化字符串（带边界检查）
    va_end(args);
    uart_write_string(BLUETOOTH_UART, buf);                                     // 通过 UART 发送
}

//==================================================== 蓝牙接收（调参命令解析） ====================================================

// 外部PID参数（定义在 common_Mymenu.c 中）
extern float image_kp_a;
extern float image_kp_b;
extern float image_kd;
extern float IMU_kp;
extern float IMU_ki;
extern float IMU_kd;
extern float motor_kp_a;
extern float motor_kp_b;
extern float motor_kd;

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

    // 从 ISR 填充的 FIFO 中读取（不丢字节），收到 ] 时往前搜 [ 解析
    {
        uint32 len = 1;
        while(fifo_read_buffer(&bt_rx_fifo, &byte, &len, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS && len == 1)
        {
            len = 1;
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
                // 手动解析 [slider,<name>,<value>]（microlib 的 sscanf 不支持 %f）
                char *p = buf + start;                                          // 指向 '['
                char name[16];
                char val_str[16];
                float val = 0.0f;
                int16 n;

                // 跳过 "[slider,"
                if(strncmp(p, "[slider,", 8) == 0)
                {
                    p += 8;                                                     // 指向参数名首字符
                    // 提取参数名（到 ',' 为止）
                    n = 0;
                    while(*p != ',' && *p != '\0' && n < 15)
                        name[n++] = *p++;
                    name[n] = '\0';

                    if(*p == ',') p++;                                          // 跳过 ','

                    // 提取值字符串（到 ']' 为止）
                    n = 0;
                    while(*p != ']' && *p != '\0' && n < 15)
                        val_str[n++] = *p++;
                    val_str[n] = '\0';

                    // 字符串转 float（手写，microlib 的 strtof 不可用）
                    {
                        float sign = 1.0f, int_part = 0.0f, frac_part = 0.0f, frac_div = 1.0f;
                        char *s = val_str;
                        if(*s == '-') { sign = -1.0f; s++; }
                        else if(*s == '+') { s++; }
                        while(*s >= '0' && *s <= '9')
                            { int_part = int_part * 10.0f + (*s - '0'); s++; }
                        if(*s == '.')
                        {
                            s++;
                            while(*s >= '0' && *s <= '9')
                                { frac_part = frac_part * 10.0f + (*s - '0'); frac_div *= 10.0f; s++; }
                        }
                        val = sign * (int_part + frac_part / frac_div);
                    }
                }

                if(strcmp(name, "image_kp_a") == 0)
                {
                    image_kp_a = val;
                    serial_printf("OK kp_a=%.3f\r\n", image_kp_a);
                }
                else if(strcmp(name, "image_kp_b") == 0)
                {
                    image_kp_b = val;
                    serial_printf("OK kp_b=%.3f\r\n", image_kp_b);
                }
                else if(strcmp(name, "image_kd") == 0)
                {
                    image_kd = val;
                    serial_printf("OK kd=%.3f\r\n", image_kd);
                }
                else if(strcmp(name, "IMU_kp") == 0)
                {
                    IMU_kp = val;
                    serial_printf("OK IMU_kp=%.3f\r\n", IMU_kp);
                }
                else if(strcmp(name, "IMU_ki") == 0)
                {
                    IMU_ki = val;
                    serial_printf("OK IMU_ki=%.3f\r\n", IMU_ki);
                }
                else if(strcmp(name, "IMU_kd") == 0)
                {
                    IMU_kd = val;
                    serial_printf("OK IMU_kd=%.3f\r\n", IMU_kd);
                }
                else if(strcmp(name, "motor_kp_a") == 0)
                {
                    motor_kp_a = val;
                    serial_printf("OK motor_kp_a=%.3f\r\n", motor_kp_a);
                }
                else if(strcmp(name, "motor_kp_b") == 0)
                {
                    motor_kp_b = val;
                    serial_printf("OK motor_kp_b=%.3f\r\n", motor_kp_b);
                }
                else if(strcmp(name, "motor_kd") == 0)
                {
                    motor_kd = val;
                    serial_printf("OK motor_kd=%.3f\r\n", motor_kd);
                }
                else
                {
                    serial_printf("ERR %s\r\n", name);
                }
            }
            idx = 0;                                                            // 解析完清缓冲
            }
        }
    }
}




float get_gyro_z(void)
{
    return imu963ra_gyro_transition(imu963ra_gyro_z);
}