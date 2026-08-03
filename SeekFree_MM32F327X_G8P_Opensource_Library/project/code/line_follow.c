/*********************************************************************************************************************
* 文件名称          line_follow
* 功能描述          智能车摄像头扫描巡线 - 巡线主控模块实现
* 适用平台          MM32F327X_G8P
* 说明              巡线模块的核心工作流程：
*
*                  ┌─────────────────────────────────────────────────┐
*                  │              巡线工作流程图                       │
*                  ├─────────────────────────────────────────────────┤
*                  │  1. 等待摄像头图像采集完成 (mt9v03x_finish_flag)  │
*                  │  2. 执行图像处理管线 (image_process_pipeline)     │
*                  │     ├── 大津法计算阈值                           │
*                  │     ├── 图像二值化                               │
*                  │     ├── 画黑框                                   │
*                  │     ├── 寻找边界起始点                           │
*                  │     ├── 八邻域爬线                               │
*                  │     ├── 寻找ABCD点                               │
*                  │     ├── 十字路口补线                             │
*                  │     └── 提取赛道中线                             │
*                  │  3. 中线数据就绪 (line_data_ready = 1)           │
*                  │  4. 等待下一帧图像                               │
*                  └─────────────────────────────────────────────────┘
*
*                 中线数据存储完成后，后续的舵机PID控制和电机PID控制
*                 可以通过 get_center_line()、calc_deviation() 等接口
*                 获取中线数据用于控制计算。
*********************************************************************************************************************/

#include "line_follow.h"
#include "control.h"

//==================================================== 全局变量定义 ====================================================

line_follow_state_enum line_state = LINE_STATE_IDLE;                            // 巡线当前工作状态
uint8 line_data_ready = 0;                                                      // 中线数据就绪标志（1=可被读取）



static const uint8 weight[IMG_H]={   1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
                        2 , 3 , 3 , 4 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 12 , 13 ,
                        14, 14, 15, 16, 16, 17, 18, 18, 19, 19, 18, 18, 17, 17, 16,
                        16, 15, 15, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9 , 9 , 8 ,
                        8 , 7 , 7 , 6 , 6 , 5 , 5 , 4 , 4 , 3 , 3 , 2 , 2 , 1 , 1 ,
                        5 , 4 , 4 , 4 , 3 , 3 , 3 , 3 , 2 , 2 , 2 , 1 , 1 , 1 , 1 };    //加权数组 15*6

static const uint8 weight2[IMG_H]={   20, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 18, 17, 17, 17,
                        17, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13 ,
                        13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9 ,
                        9 , 9 , 9 , 8 , 8 , 8 , 8 , 7 , 7 , 7 , 7 , 6 , 6 , 6 , 6 ,
                        5 , 5 , 5 , 5 , 4 , 4 , 4 , 4 , 3 , 3 , 3 , 3 , 2 , 2 , 2 ,
                        2 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 };    //加权数组 15*6

//==================================================== 巡线模块初始化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：line_follow_init
// 功能：初始化巡线模块
// 参数：void
// 返回：void
// 说明：
//   设置巡线模块的初始状态为等待图像采集
//   初始化中线数组（全部填充为图像中心值）
//   初始化边界数组（左边界=0，右边界=IMG_W-1）
//-------------------------------------------------------------------------------------------------------------------
void line_follow_init(void)
{
    uint8 i;

    // ---- 初始化中线数组（默认值 = 图像中心 IMG_W/2 = 70） ----
    for(i = 0; i < IMG_H; i++)
    {
        center_line[i] = IMG_W / 2;                                             // 默认为图像中心
        left_boundary[i] = 0;       
        center_line_valid[i] = 0;                                            // 左边界默认为最左边
        right_boundary[i] = IMG_W - 1;                                          // 右边界默认为最右边
    }

    // ---- 设置初始状态 ----
    line_state = LINE_STATE_WAIT_IMAGE;                                         // 进入等待图像状态
    line_data_ready = 0;                                                        // 中线数据尚未就绪

    // ---- 初始化二值化阈值 ----
    otsu_threshold = 180;                                                       // 初始阈值180（中等灰度值）
    last_threshold = 180;

    // ---- 重置圆环状态（确保每次发车从正常模式开始） ----
    ring_state = RING_S_NONE;
}

//==================================================== 巡线主处理函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：line_follow_process
// 功能：巡线主处理函数（在主循环中每帧调用一次）
// 参数：void
// 返回：void
// 说明：
//   采用状态机模式管理巡线流程：
//     LINE_STATE_WAIT_IMAGE  —— 等待摄像头图像采集完成
//     LINE_STATE_PROCESSING   —— 执行图像处理管线
//     LINE_STATE_DONE         —— 处理完成，中线数据已就绪
//
//   状态转换条件：
//     WAIT_IMAGE → PROCESSING：检测到 mt9v03x_finish_flag 置位
//     PROCESSING  → DONE：     image_process_pipeline() 执行完毕
//     DONE        → WAIT_IMAGE：清除标志，等待下一帧图像
//
//   注意事项：
//     1. 图像处理管线在 PROCESSING 状态只执行一次
//     2. 处理完成后 line_data_ready 置1，外部模块可读取中线数据
//     3. 直到下一帧图像到来前，中线数据保持不变
//-------------------------------------------------------------------------------------------------------------------
void line_follow_process(void)
{
    switch(line_state)
    {
        // ==================== 状态1：等待图像采集完成 ====================
        case LINE_STATE_WAIT_IMAGE:
        {
            // 检测摄像头是否完成了一帧图像的采集
            if(mt9v03x_finish_flag)
            {
                line_state = LINE_STATE_PROCESSING;                             // 转入处理状态
                line_data_ready = 0;                                            // 清除数据就绪标志
            }
            break;
        }

        // ==================== 状态2：执行图像处理管线 ====================
        case LINE_STATE_PROCESSING:
        {
            // ---- 执行完整的图像处理管线 ----
            // 此函数会依次执行：
            //   大津法 → 二值化 → 画边框 → 找起始点 →
            //   八邻域爬线 → 找ABCD点 → 补线 → 中线提取
            image_process_pipeline();

            // 丢线边界补偿
            //boundary_lost_compensate();

            // ---- 清除摄像头采集完成标志（准备接收下一帧） ----
            mt9v03x_finish_flag = 0;

            // ---- 设置数据就绪标志 ----
            line_data_ready = 1;                                                // 中线数据可供读取
            line_state = LINE_STATE_DONE;                                       // 转入完成状态
            break;


        }

        // ==================== 状态3：处理完成 ====================
        case LINE_STATE_DONE:
        {
            // 中线数据已经就绪，等待外部模块读取
            // 不在此处做任何操作，由外部模块通过 get_center_line() 等函数读取数据

            // 等待下一帧图像到来
            if(mt9v03x_finish_flag)
            {
                line_state = LINE_STATE_PROCESSING;                             // 有新图像，转入处理状态
                line_data_ready = 0;                                            // 清除数据就绪标志
            }
            break;
        }

        // ==================== 空闲状态（异常情况） ====================
        case LINE_STATE_IDLE:
        default:
        {
            // 在空闲状态下等待图像
            if(mt9v03x_finish_flag)
            {
                line_state = LINE_STATE_PROCESSING;
                line_data_ready = 0;
            }
            break;
        }
    }
}

//==================================================== 中线数据访问接口 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：get_center_line
// 功能：获取指定行的赛道中线X坐标
// 参数：row —— 行号（0 ~ IMG_H-1，0是图像最上方，IMG_H-1=89是图像最下方/最近处）
// 返回：uint8 —— 该行的中线X坐标（0 ~ IMG_W-1 = 0~140）
// 说明：
//   row参数会被限幅到有效范围[0, IMG_H-1]
//   如果中线数据尚未就绪，返回图像中心值 IMG_W/2=70（安全默认值）
//   典型用法：
//     - 获取最近处中线：get_center_line(89)  → 车身前方最近处
//     - 获取前瞻中线：  get_center_line(59)  → 前方30行处（最底行-30）
//     - 获取远处中线：  get_center_line(29)  → 前方60行处（最底行-60）
//-------------------------------------------------------------------------------------------------------------------
uint8 get_center_line(uint8 row)
{
    // ---- 行号限幅 ----
    if(row >= IMG_H)
        row = IMG_H - 1;                                                        // 最大行号 = IMG_H-1

    // ---- 返回中线数据 ----
    // 如果数据尚未就绪，返回图像中心作为安全默认值
    if(line_data_ready)
    {
        return center_line[row];
    }
    else
    {
        return IMG_W / 2;                                                       // 默认返回图像中心（94）
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：get_left_boundary
// 功能：获取指定行的左边界X坐标
// 参数：row —— 行号（0 ~ IMG_H-1）
// 返回：uint8 —— 该行的左边界X坐标
// 说明：如果边界数据尚未就绪，返回0（最左边）
//-------------------------------------------------------------------------------------------------------------------
uint8 get_left_boundary(uint8 row)
{
    // ---- 行号限幅 ----
    if(row >= IMG_H)
        row = IMG_H - 1;

    if(line_data_ready)
    {
        return left_boundary[row];
    }
    else
    {
        return 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：get_right_boundary
// 功能：获取指定行的右边界X坐标
// 参数：row —— 行号（0 ~ IMG_H-1）
// 返回：uint8 —— 该行的右边界X坐标
// 说明：如果边界数据尚未就绪，返回 IMG_W-1=140（最右边）
//-------------------------------------------------------------------------------------------------------------------
uint8 get_right_boundary(uint8 row)
{
    // ---- 行号限幅 ----
    if(row >= IMG_H)
        row = IMG_H - 1;

    if(line_data_ready)
    {
        return right_boundary[row];
    }
    else
    {
        return IMG_W - 1;                                                       // 默认返回最右边
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：get_current_threshold
// 功能：获取当前使用的二值化阈值
// 参数：void
// 返回：uint8 —— 当前二值化阈值（0~255）
// 说明：可用于在屏幕上显示当前阈值，或用于调试分析
//-------------------------------------------------------------------------------------------------------------------
uint8 get_current_threshold(void)
{
    return otsu_threshold;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：calc_deviation
// 功能：计算车身相对于赛道中心的横向偏差
// 参数：look_ahead_rows —— 前瞻行数（从底部向上偏移的行数）
//                         0 表示看最底行（IMG_H-1）
//                         15 表示看第 IMG_H-1-15 行
// 返回：int16 —— 偏差值
//         正值：中线在图像中心右侧 → 车身偏左 → 需要向右修正
//         负值：中线在图像中心左侧 → 车身偏右 → 需要向左修正
//         0：车身在赛道正中心
// 说明：
//   图像中心X坐标 = IMG_W/2 = 70
//   偏差 = 前瞻行的中线X - 70
//   例如：
//     - 中线在99列: 偏差 = +5  → 车身偏左，应向右修正
//     - 中线在89列: 偏差 = -5  → 车身偏右，应向左修正
//     - 中线在70列: 偏差 = 0   → 车身居中，不需修正
//
//   前瞻距离越大（look_ahead_rows越大），看的越远，
//   适合高速时使用，可以提前预判弯道趋势
//   前瞻距离越小，看的越近，适合低速时使用，响应更及时
//-------------------------------------------------------------------------------------------------------------------
int16 calc_deviation(uint8 look_ahead_rows)
{
    uint8 row;
    uint8 mid_x;

    // ---- 计算目标行号（从底部向上偏移） ----
    if(look_ahead_rows >= IMG_H)
        look_ahead_rows = IMG_H - 1;

    row = IMG_H - 1 - look_ahead_rows;                                          // 目标行号

    // ---- 获取该行的中线X坐标 ----
    mid_x = get_center_line(row);

    // ---- 计算偏差（相对于图像中心列的偏移量） ----
    // IMG_W/2 = 70 是图像的水平中心位置
    return (int16)mid_x - (int16)(IMG_W / 2);
}

float get_weight_position(uint8 *center_line, uint8 is_straight)
{
    const uint8 *w = is_straight ? weight : weight2;

    float weighted_sum = 0.0f;
    float weight_total = 0.0f;

    int16 i;
    for(i = 0; i < IMG_H; i++)
    {
        if(center_line_valid[i] == 1)
        {
            weighted_sum += (float)center_line[i] * w[i];
            weight_total += w[i];
        }
    }

    float raw_pos;
    if(weight_total > 0.0f)
        raw_pos = weighted_sum / weight_total;
    else
        raw_pos = 0.0f;

    // 一阶低通滤波：滤除中线位置的帧间抖动
    #define POS_LOWPASS 0.3f
    static float filtered_pos = 0.0f;
    static uint8 first_run = 1;
    if(first_run)
    {
        filtered_pos = raw_pos;
        first_run = 0;
    }
    else
    {
        filtered_pos = POS_LOWPASS * raw_pos + (1.0f - POS_LOWPASS) * filtered_pos;
    }
    return filtered_pos;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：boundary_lost_compensate
// 功能：丢线边界补偿（单侧丢线>9/10时，对侧边界向内偏移30px）
// 说明：
//   统计行10~IMG_H-1
//   左边界=1→丢线，右边界=IMG_W-2→丢线
//   左丢→右边界左移30px，右丢→左边界右移30px，双侧都丢→不偏移
//-------------------------------------------------------------------------------------------------------------------
void boundary_lost_compensate(void)
{
    uint16 total_rows = 0;
    uint16 left_lost = 0, right_lost = 0;
    int16 k;
    for(k = IMG_H/2; k >=2 ; k--)
    {
        total_rows++;
        if(left_boundary[k] <= 3)       left_lost++;
        if(right_boundary[k] >= IMG_W - 4) right_lost++;
    }
    if(total_rows == 0) return;

    uint8 left_lost_flag  = (left_lost * 10  >= total_rows * 9);
    uint8 right_lost_flag = (right_lost * 10 >= total_rows * 9);

    if(left_lost_flag && !right_lost_flag)
    {
        // 额外条件：中心线>IMG_W/2的行数≥5（确保右边界可靠）
        uint8 center_right_count = 0;
        for(k = 2; k < IMG_H/2; k++)
        {
            if(center_line[k] < IMG_W / 2) center_right_count++;
        }
        if(center_right_count >= 5)
        {
            for(k = 2; k < IMG_H; k++)
            {
                if(right_boundary[k] >= 10)
                    right_boundary[k] -= 10;
                else
                    right_boundary[k] = 0;
                center_line[k] = (left_boundary[k] + right_boundary[k]) / 2;
            }
        }
    }
    else if(!left_lost_flag && right_lost_flag)
    {
        // 额外条件：中心线>IMG_W/2的行数≥5（确保左边界可靠）
        uint8 center_right_count = 0;
        for(k = 2; k < IMG_H/2; k++)
        {
            if(center_line[k] > IMG_W / 2) center_right_count++;
        }
        if(center_right_count >= 5)
        {
            for(k = 2; k < IMG_H; k++)
            {
                if(left_boundary[k] <= IMG_W - 11)
                    left_boundary[k] += 10;
                else
                    left_boundary[k] = IMG_W - 1;
                center_line[k] = (left_boundary[k] + right_boundary[k]) / 2;
            }
        }
    }
}

// ---- 图像 PID：中线偏差 → 目标角速度 ----
float image_pid_error=0;
static float image_pid_outd=0;
static float image_pid_outp=0;
static float image_kp=0;
float image_pid_set(float target,float actual)
{
    static uint8 first = 1;
    image_pid_error = target - actual;
    if(first) { image_pid_outp = image_pid_error; first = 0; return 0.0f; }  // 首帧跳过D项防尖峰
    image_pid_outd = (image_pid_error - image_pid_outp)*image_lowpass+image_pid_outd*(1-image_lowpass);
    image_pid_outp = image_pid_error;
    image_kp=image_kp_a + (image_pid_error*image_pid_error)*image_kp_b;
    {
        float abs_img_err = (image_pid_error > 0.0f) ? image_pid_error : -image_pid_error;
        if(abs_img_err < 12.0f) image_kp = image_kp_a;
    }
    return (-(image_kp*image_pid_outp + image_kd*image_pid_outd ));
}

// ---- IMU PID：角速度闭环 → 舵机打角 ----
static float IMU_pid_error=0;
static float IMU_pid_outd=0;
static float IMU_pid_outp=0;
static float IMU_kp=0;
static float finall_out;
float IMU_pid_set(float target,float actual)
{
    static uint8 first = 1;
    IMU_pid_error = target - actual;
    if(first) { IMU_pid_outp = IMU_pid_error; first = 0; return 0.0f; }
    IMU_pid_outd = (IMU_pid_error - IMU_pid_outp)*IMU_lowpass+IMU_pid_outd*(1-IMU_lowpass);
    IMU_pid_outp = IMU_pid_error;
    IMU_kp=IMU_kp_a + (IMU_pid_error*IMU_pid_error)*IMU_kp_b;
    {
        float abs_img_err = (image_pid_error > 0.0f) ? image_pid_error : -image_pid_error;
        if(abs_img_err < 12.0f) IMU_kp = IMU_kp_a;
    }
    finall_out= -(IMU_kp*IMU_pid_outp + IMU_kd*IMU_pid_outd );
    if(finall_out >12){
        finall_out=12;
    }else if(finall_out<-12){
        finall_out=-12;
    }
    return (finall_out);
}

// ---- 电机 PID：中线偏差 → 差速量 ----
static float motor_pid_error=0;
static float motor_pid_outd=0;
static float motor_pid_outp=0;
static float motor_kp=0;
float motor_pid_set(float target,float actual)
{
    static uint8 first = 1;
    motor_pid_error = target - actual;
    if(first) { motor_pid_outp = motor_pid_error; first = 0; return 0.0f; }
    motor_pid_outd = (motor_pid_error - motor_pid_outp)*motor_lowpass+motor_pid_outd*(1-motor_lowpass);
    motor_pid_outp = motor_pid_error;
    motor_kp=motor_kp_a + (motor_pid_error*motor_pid_error)*motor_kp_b;
    return (-(motor_kp*motor_pid_outp + motor_kd*motor_pid_outd ));
}

//==================================================== 姿态解算（六轴互补滤波 AHRS） ====================================================

#include <math.h>

// ---- 互补滤波参数 ----
#define ATTI_KP             0.0001f                                             // 加速度计修正比例增益
#define ATTI_KI             -0.0000324f                                        // 加速度计修正积分增益
#define ATTI_DT             0.005f                                              // 采样周期（s），与 PIT 一致
#define ATTI_ACC_ALPHA      0.3f                                                // 加速度低通滤波系数

// ---- 四元数状态 ----
static float atti_q0 = 1.0f, atti_q1 = 0.0f, atti_q2 = 0.0f, atti_q3 = 0.0f;
static float atti_I_ex = 0.0f, atti_I_ey = 0.0f, atti_I_ez = 0.0f;

// ---- 输出 ----
float atti_yaw = 0.0f;                                                          // 当前偏航角（°），左=负，右=正

// ---- 角度 PID 参数（蓝牙可调） ----
float angle_kp_a = 0.0f;
float angle_kp_b = 0.0f;
float angle_kd  = 0.32f;
float angle_lowpass = 0.8f;

// ---- 融合系数 ----
float servo_fusion_alpha = 0.10f;                                                // 0=纯IMU_PID, 1=纯角度PID

// ---- 上一帧舵机角度（用于小角度kp_b抑制） ----
float prev_servo_angle = 0.0f;

//---- 上次偏航角 ----
static float prev_angle_yaw = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：atti_init
// 功能：姿态解算初始化
//-------------------------------------------------------------------------------------------------------------------
void atti_init(void)
{
    atti_q0 = 1.0f; atti_q1 = 0.0f; atti_q2 = 0.0f; atti_q3 = 0.0f;
    atti_I_ex = 0.0f; atti_I_ey = 0.0f; atti_I_ez = 0.0f;
    atti_yaw = 0.0f;
    prev_angle_yaw = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：fast_inv_sqrt
// 功能：快速平方根倒数（Quake III 算法）
//-------------------------------------------------------------------------------------------------------------------
static float fast_inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    int32 i;
    // memcpy not used due to microlib, use union instead
    union { float f; int32 i; } u;
    u.f = y;
    u.i = 0x5f3759df - (u.i >> 1);
    y = u.f;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：atti_update
// 功能：姿态解算更新（在 PIT 中断中调用，5ms 周期）
// 说明：基于六轴互补滤波 AHRS
//       陀螺仪提供高频角速度 → 四元数积分
//       加速度计提供低频重力方向 → PI 修正陀螺漂移
//       最后从四元数提取偏航角 Yaw
//-------------------------------------------------------------------------------------------------------------------
void atti_update(void)
{
    float ax, ay, az;
    float gx, gy, gz;
    float norm;
    float ex, ey, ez;
    float q0, q1, q2, q3;
    static float acc_fx = 0.0f, acc_fy = 0.0f, acc_fz = 0.0f;
    static uint8 first = 1;

    // ---- 1. 读取加速度计（低通滤波） ----
    imu963ra_get_acc();
    ax = imu963ra_acc_transition((float)imu963ra_acc_x);
    ay = imu963ra_acc_transition((float)imu963ra_acc_y);
    az = imu963ra_acc_transition((float)imu963ra_acc_z);
    if(first) { acc_fx = ax; acc_fy = ay; acc_fz = az; first = 0; }
    else
    {
        acc_fx = ATTI_ACC_ALPHA * ax + (1.0f - ATTI_ACC_ALPHA) * acc_fx;
        acc_fy = ATTI_ACC_ALPHA * ay + (1.0f - ATTI_ACC_ALPHA) * acc_fy;
        acc_fz = ATTI_ACC_ALPHA * az + (1.0f - ATTI_ACC_ALPHA) * acc_fz;
    }

    // ---- 2. 读取陀螺仪，转为 rad/s ----
    imu963ra_get_gyro();
    gx = imu963ra_gyro_transition((float)imu963ra_gyro_x) * 3.1415926f / 180.0f;
    gy = imu963ra_gyro_transition((float)imu963ra_gyro_y) * 3.1415926f / 180.0f;
    gz = imu963ra_gyro_transition((float)imu963ra_gyro_z) * 3.1415926f / 180.0f;

    // ---- 3. 归一化加速度计 ----
    norm = fast_inv_sqrt(acc_fx * acc_fx + acc_fy * acc_fy + acc_fz * acc_fz);
    ax = acc_fx * norm;
    ay = acc_fy * norm;
    az = acc_fz * norm;

    // ---- 4. 估计重力方向（从当前四元数） ----
    q0 = atti_q0; q1 = atti_q1; q2 = atti_q2; q3 = atti_q3;
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // ---- 5. 误差 = 测量加速度 × 估计重力（叉积） ----
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // ---- 6. PI 修正陀螺仪 ----
    float halfT = 0.5f * ATTI_DT;
    atti_I_ex += halfT * ex;
    atti_I_ey += halfT * ey;
    atti_I_ez += halfT * ez;
    gx = gx + ATTI_KP * ex + ATTI_KI * atti_I_ex;
    gy = gy + ATTI_KP * ey + ATTI_KI * atti_I_ey;
    gz = gz + ATTI_KP * ez + ATTI_KI * atti_I_ez;

    // ---- 7. 一阶龙格库塔更新四元数 ----
    q0 = atti_q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = atti_q1 + ( atti_q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = atti_q2 + ( atti_q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = atti_q3 + ( atti_q0 * gz + q1 * gy - q2 * gx) * halfT;

    // ---- 8. 归一化四元数 ----
    norm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    atti_q0 = q0 * norm; atti_q1 = q1 * norm;
    atti_q2 = q2 * norm; atti_q3 = q3 * norm;

    // ---- 9. 提取偏航角 Yaw（°），左=负，右=正 ----
    atti_yaw = atan2f(2.0f * (atti_q1 * atti_q2 + atti_q0 * atti_q3),
                      -2.0f * atti_q2 * atti_q2 - 2.0f * atti_q3 * atti_q3 + 1.0f) * 57.29578f;
}

// ---- 角度 PID：偏航角稳定控制 ----
float angle_pid_error = 0;
float angle_pid_outd = 0;
float angle_pid_outp = 0;
float angle_kp = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：angle_pid_set
// 功能：角度 PID（偏航角稳定）
// 参数：target —— 目标角度（上一次偏航角，°）
// 参数：actual —— 实际角度（当前偏航角，°）
// 返回：float —— 舵机角度（左=负，右=正），限幅 ±12°
//
// 工作逻辑：
//   error = target - actual = prev_yaw - curr_yaw = -∆yaw
//   若车向右转（curr > prev）→ error < 0 → 输出负 → 左转 ← 抑制过度右转
//   若车向左转（curr < prev）→ error > 0 → 输出正 → 右转 ← 抑制过度左转
//-------------------------------------------------------------------------------------------------------------------
float angle_pid_set(float target, float actual)
{
    static uint8 first = 1;
    angle_pid_error = target - actual;
    if(first) { angle_pid_outp = angle_pid_error; first = 0; return 0.0f; }
    angle_pid_outd = (angle_pid_error - angle_pid_outp) * angle_lowpass
                   + angle_pid_outd * (1.0f - angle_lowpass);
    angle_pid_outp = angle_pid_error;
    angle_kp = angle_kp_a + (angle_pid_error * angle_pid_error) * angle_kp_b;
    {
        float abs_img_err = (image_pid_error > 0.0f) ? image_pid_error : -image_pid_error;
        if(abs_img_err < 12.0f) angle_kp = angle_kp_a;
    }
    float out = -(angle_kp * angle_pid_outp + angle_kd * angle_pid_outd);
    if(out > 12.0f)  out = 12.0f;
    if(out < -12.0f) out = -12.0f;
    return out;
}




//-------------------------------------------------------------------------------------------------------------------
// 函数名称：speed_pid_set
// 功能：速度闭环 PID（增量式，带输出饱和抗积分饱和），左右轮独立状态
// 参数：channel —— 0=左轮, 1=右轮
// 参数：target  —— 目标速度（脉冲/5ms）
// 参数：actual  —— 实际速度（脉冲/5ms）
// 返回：float —— 电机占空比（-100~100）
//-------------------------------------------------------------------------------------------------------------------

float speed_pid_set(uint8 channel, float target, float actual)
{
    static uint8 first[2] = {1, 1};
    static float speed_pid_out[2] = {0.0f, 0.0f};
    static float error_prev[2]  = {0.0f, 0.0f};
    static float error_prev2[2] = {0.0f, 0.0f};

    float err = target - actual;

    if(first[channel])
    {
        speed_pid_out[channel] = (float)motor_duty;
        error_prev[channel]  = err;
        error_prev2[channel] = err;
        first[channel] = 0;
        return speed_pid_out[channel];
    }

    float err_p = error_prev[channel];
    float err_pp = error_prev2[channel];

    // 增量式 PID：Δu = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
    float increment = speed_kp * (err - err_p)
                    + speed_ki * err
                    + speed_kd * (err - 2.0f * err_p + err_pp);

    // 增量限幅：encoder≈duty×10，±8匹配正常duty范围0~20
    float inc_max = 8.0f;
    if(increment > inc_max)  increment = inc_max;
    if(increment < -inc_max) increment = -inc_max;

    speed_pid_out[channel] += increment;

    // 输出饱和 + 抗积分饱和（条件积分法）
    // 只有输出已饱和且增量同向时才钳位，堵转时允许输出继续上升
    if(speed_pid_out[channel] > (float)MOTOR_DUTY_MAX)
        speed_pid_out[channel] = (float)MOTOR_DUTY_MAX;
    else if(speed_pid_out[channel] < (float)MOTOR_DUTY_MIN)
        speed_pid_out[channel] = (float)MOTOR_DUTY_MIN;

    // 更新历史误差
    error_prev2[channel] = err_p;
    error_prev[channel]  = err;

    return speed_pid_out[channel];
}


//-------------------------------------------------------------------------------------------------------------------
// 函数名称：servo_fusion
// 功能：融合角度 PID 和 IMU PID 的输出
// 参数：angle_out —— 角度 PID 输出
// 参数：IMU_out   —— IMU PID 输出
// 返回：融合后的舵机角度
// 公式：out = α × angle_out + (1-α) × IMU_out
//-------------------------------------------------------------------------------------------------------------------
float servo_fusion(float angle_out, float IMU_out)
{
    float out = servo_fusion_alpha * angle_out + (1.0f - servo_fusion_alpha) * IMU_out;
    if(out > 12.0f)  out = 12.0f;
    if(out < -12.0f) out = -12.0f;
    return out;
}






