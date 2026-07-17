/*********************************************************************************************************************
* 文件名称          image_process
* 功能描述          智能车摄像头扫描巡线 - 图像处理算法实现
* 适用平台          MM32F327X_G8P
* 摄像头型号        MT9V03X 总钻风灰度摄像头（60行×80列）
* 说明              完整的赛道图像处理管线：
*                   原始灰度图像 → 大津法阈值 → 二值化 → 画边框 →
*                   寻找起始点 → 八邻域爬线 → 找ABCD点 → 补线 → 中线提取
*
* 核心算法说明：
*   1. 快速大津法：每隔2行2列采样（只用1/4像素），计算类间方差最大化阈值
*   2. 八邻域爬线：从底部起始点沿黑白交界向上追踪，提取完整赛道边界
*   3. ABCD点检测：寻找赛道边界的拐角点，用于判断十字路口和弯道
*   4. 中线提取：逐行计算左右边界的中点，得到赛道中心轨迹
*********************************************************************************************************************/

#include "image_process.h"

//==================================================== 全局变量定义 ====================================================

// ---- 二值化阈值 ----
uint8 otsu_threshold = 100;                                                     // 大津法计算得到的二值化阈值，默认100
uint8 last_threshold = 100;                                                     // 上一次有效的阈值，异常时回退使用
float filtered_threshold = 100.0f;                                              // 互补滤波后的平滑阈值
uint8 threshold_mode = 1;                                                       // 阈值模式：1=大津法，0=固定阈值
uint8 fixed_threshold = 220;                                                    // 固定阈值默认值（0~255）

#define THRESHOLD_ALPHA         0.3f                                            // 互补滤波系数（0~1，越小越平滑，越大越灵敏）

// ---- 二值化后的图像 ----
uint8 binary_image[IMG_H][IMG_W];                                               // 二值化图像：0=黑(边界线), 255=白(赛道)

// ---- 八邻域爬线相关 ----
edge_point left_edge[BOUNDARY_SEARCH_MAX];                                      // 左边界点数组
edge_point right_edge[BOUNDARY_SEARCH_MAX];                                     // 右边界点数组
uint8 left_edge_count = 0;                                                      // 左边界点个数
uint8 right_edge_count = 0;                                                     // 右边界点个数
uint8 left_find_flag = 0;                                                       // 左边界起始点找到标志
uint8 right_find_flag = 0;                                                      // 右边界起始点找到标志
int16 left_start_row = 0, left_start_col = 0;                                   // 左边界搜索起始点
int16 right_start_row = 0, right_start_col = 0;                                 // 右边界搜索起始点
uint8 left_lose_rows = 0;                                                       // 左边界丢失行数
uint8 right_lose_rows = 0;                                                      // 右边界丢失行数

// ---- A/B/C/D 关键点 ----
uint8 point_A_row = 0, point_A_col = 0;                                        // A点（左边界底部起点）
uint8 point_B_row = 0, point_B_col = 0;                                        // B点（右边界底部起点）
uint8 point_C_row = 0, point_C_col = 0;                                        // C点（左边界上部拐点）
uint8 point_D_row = 0, point_D_col = 0;                                        // D点（右边界上部拐点）

// ---- 赛道中线 ----
uint8 center_line[IMG_H];                                                       // 中线数组
uint8 center_line_valid[IMG_H];                                                 // 中线有效标记（1=真实边界，0=插值）
uint8 left_boundary[IMG_H];                                                     // 左边界数组（默认0=最左边）
uint8 right_boundary[IMG_H];                                                    // 右边界数组（默认79=最右边）

//==================================================== 快速大津法（OTSU） ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：otsu_threshold_calc
// 功能：快速大津法（OTSU）计算图像二值化阈值
// 参数：*image —— 原始灰度图像数组指针（一维数组，IMG_SIZE = 4800）
// 返回：uint8 —— 最佳二值化阈值（0~255）
//
// 原理说明：
//   大津法（OTSU）是一种自适应的图像二值化阈值计算方法。
//   它通过遍历所有可能的阈值（0~255），计算每个阈值下的类间方差，
//   选择使类间方差最大的那个阈值作为最佳二值化阈值。
//
//   类间方差公式：g = w0 * w1 * (u0 - u1)^2
//     其中：w0 = 前景像素占比（灰度值 < 阈值的像素比例）
//           w1 = 背景像素占比（灰度值 ≥ 阈值的像素比例）
//           u0 = 前景平均灰度
//           u1 = 背景平均灰度
//
//   类间方差越大，说明前景和背景分离得越好。
//
// 优化策略：
//   1. 每隔2行2列采样（只用1/4像素），大幅降低计算量，适合嵌入式实时处理
//   2. 只在最小灰度~最大灰度范围内搜索，进一步缩小遍历范围
//   3. 当类间方差开始下降时提前退出（启发式优化）
//   4. 阈值合理性检查（90~130之间），异常时沿用上次有效阈值
//-------------------------------------------------------------------------------------------------------------------
uint8 otsu_threshold_calc(uint8 *image)
{
    #define GRAY_SCALE 256                                                      // 灰度级 0~255
    #define KUAN       30                                                       // 两峰之间最小间隔（灰度级）

    uint16 i, j;
    uint8 pixel_min = 255, pixel_max = 0;
    uint8 *data = image;

    // ---- 快速对比度检测：先找 min/max（轻量扫描，隔2行2列） ----
    for(i = 0; i < IMG_H; i += 2)
    {
        for(j = 0; j < IMG_W; j += 2)
        {
            uint8 val = data[i * IMG_W + j];
            if(val < pixel_min) pixel_min = val;
            if(val > pixel_max) pixel_max = val;
        }
    }

    // 图像几乎均匀（对比度<60），沿用滤波后的阈值，避免无效处理
    if(pixel_max - pixel_min < 60)
    {
        filtered_threshold = filtered_threshold * 0.95f + last_threshold * 0.05f;
        return (uint8)(filtered_threshold + 0.5f);
    }

    // ---- 1. 全像素统计灰度直方图 ----
    int32 pixel_count[GRAY_SCALE] = {0};
    for(i = 0; i < IMG_H; i++)
    {
        for(j = 0; j < IMG_W; j++)
        {
            pixel_count[data[i * IMG_W + j]]++;
        }
    }

    // ---- 2. 寻找第一高峰（直方图中频数最高的灰度值） ----
    int32 H1 = 0;
    uint8 D1 = 0;

    for(i = 0; i < 255; i++)
    {
        if(pixel_count[i] > H1)
        {
            H1 = pixel_count[i];
            D1 = (uint8)i;
        }
    }

    // ---- 3. 降水位线法寻找第二高峰 ----
    int32 H2 = 0;
    uint8 D2 = 0;
    uint8 OK = 0;

    for(i = H1 - 5; i > 0; i -= 5)
    {
        for(j = 0; j < 256; j++)
        {
            if(pixel_count[j] > i && abs(j - D1) > KUAN)
            {
                H2 = i;
                D2 = (uint8)j;
                OK = 1;
                break;
            }
        }
        if(OK) break;
    }

    // ---- 4. 寻找两峰之间的谷底 ----
    int32 H3 = IMG_SIZE;
    uint8 D3 = 0;

    if(OK)
    {
        if(D1 < D2)
        {
            for(i = D1; i < D2; i++)
            {
                if(pixel_count[i] < H3)
                {
                    H3 = pixel_count[i];
                    D3 = (uint8)i;
                }
            }
        }
        else
        {
            for(i = D2; i < D1; i++)
            {
                if(pixel_count[i] < H3)
                {
                    H3 = pixel_count[i];
                    D3 = (uint8)i;
                }
            }
        }
        last_threshold = D3;
    }
    else
    {
        D3 = last_threshold;
    }

    // ---- 5. 互补滤波平滑：new = α·raw + (1-α)·old ----
    // α 越小越平滑但响应慢，α 越大越灵敏但噪声多
    filtered_threshold = THRESHOLD_ALPHA * (float)D3 + (1.0f - THRESHOLD_ALPHA) * filtered_threshold;
    return (uint8)(filtered_threshold + 0.5f);                                  // 四舍五入
}

//==================================================== 图像二值化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：image_binarize
// 功能：根据阈值将灰度图像转换为二值图像
// 参数：src_image  —— 原始灰度图像（一维数组，IMG_SIZE个元素）
// 参数：dst_image  —— 输出二值化图像（二维数组，IMG_H × IMG_W）
// 参数：threshold  —— 二值化阈值
// 返回：void
// 说明：
//   灰度值 > 阈值 → WHITE(255) 赛道区域（浅色）
//   灰度值 ≤ 阈值 → BLACK(0)   边界线/暗区域（深色）
//   这是因为赛道通常是浅色（白色），赛道边界线是深色（黑色）
//-------------------------------------------------------------------------------------------------------------------
void image_binarize(uint8 *src_image, uint8 dst_image[IMG_H][IMG_W], uint8 threshold)
{
    uint16 i, j;

    for(i = 0; i < IMG_H; i++)                                                  // 遍历所有行
    {
        for(j = 0; j < IMG_W; j++)                                              // 遍历所有列
        {
            uint16 index = i * IMG_W + j;                                       // 计算一维数组索引

            if(src_image[index] > threshold)                                    // 灰度值大于阈值 → 亮像素（赛道）
            {
                dst_image[i][j] = WHITE;                                        // 白点 —— 赛道可行驶区域
            }
            else                                                                // 灰度值小于等于阈值 → 暗像素（边界线）
            {
                dst_image[i][j] = BLACK;                                        // 黑点 —— 赛道边界或外部
            }
        }
    }
}

//==================================================== 图像外围画黑框 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：image_draw_border
// 功能：在图像外围画黑色边框（为八邻域搜索提供安全边界）
// 参数：image —— 二值化图像数组（IMG_H × IMG_W）
// 返回：void
// 说明：
//   将图像最外2行和最外2列设为黑色(BLACK=0)
//   目的：八邻域爬线时需要访问 pixel[i-1][j-1] 等相邻像素，
//         如果图像边界处没有黑框保护，可能会越界访问到错误数据
//   边框宽度为2像素，为八邻域搜索提供足够的安全区域
//-------------------------------------------------------------------------------------------------------------------
void image_draw_border(uint8 image[IMG_H][IMG_W])
{
    uint8 i;

    // ---- 画左右两侧的黑色竖边框（最外2列为黑） ----
    for(i = 0; i < IMG_H; i++)                                                  // 遍历所有行
    {
        image[i][0] = BLACK;                                                    // 最左边第0列 → 黑色
        image[i][1] = BLACK;                                                    // 左边第1列 → 黑色
        image[i][IMG_W - 1] = BLACK;                                            // 最右边列 → 黑色
        image[i][IMG_W - 2] = BLACK;                                            // 右边倒数第2列 → 黑色
    }

    // ---- 画上方和下方的黑色横边框（最上2行和最下2行为黑） ----
    for(i = 0; i < IMG_W; i++)                                                  // 遍历所有列
    {
        image[0][i] = BLACK;                                                    // 最上面第0行 → 黑色
        image[1][i] = BLACK;                                                    // 上面第1行 → 黑色
        image[IMG_H - 1][i] = BLACK;                                            // 最下面行 → 黑色
        image[IMG_H - 2][i] = BLACK;                                            // 下面倒数第2行 → 黑色
    }
}

//==================================================== 寻找边界起始点 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：find_boundary_start
// 功能：在图像底部寻找左右赛道边界的起始点
// 参数：image —— 二值化图像数组（IMG_H × IMG_W）
// 返回：void
// 说明：
//   从图像最底行（IMG_H-1，即最后一行）向上扫描
//   左边界搜索：从中间列（IMG_W/2=40）向左扫描，寻找上升沿（黑→白跳变）
//               image[i][j-1]-image[i][j]==1 表示从黑(1)进入白(0)
//               注意二值化逻辑中 BLACK=0, WHITE=255
//               上升沿判断：(image[j-1]==BLACK) && (image[j]==WHITE)
//   右边界搜索：从中间列向右扫描，寻找上升沿（白→黑跳变）
//               (image[j]==WHITE) && (image[j+1]==BLACK)
//
//   扫描范围：从靠近底部的行开始（IMG_H-3，避开底部黑框），
//            可能需要在多行中搜索直到找到有效的边界起始点
//-------------------------------------------------------------------------------------------------------------------
void find_boundary_start(uint8 image[IMG_H][IMG_W])
{
    int16 i, j;
    int16 search_row;                                                           // 当前搜索的行号

    // ---- 初始化标志 ----
    left_find_flag = 0;
    right_find_flag = 0;
    left_start_row = 0;
    left_start_col = 0;
    right_start_row = 0;
    right_start_col = 0;

    // ---- 从底部向上搜索，找到有效的边界起始行 ----
    // 从 IMG_H-3 开始（避开底部2行黑框），向上最多搜索到 IMG_H的2/3行 行
    for(search_row = IMG_H - 3; search_row > ((IMG_H / 3) * 2); search_row--)
    {
        // ==================== 寻找左边界起始点 ====================
        // 从中间列向左扫描到最左边
        if(!left_find_flag)
        {
            for(j = IMG_W / 2; j > 2; j--)                             // 从中间向左，只扫到1/3宽度
            {
                // 检测上升沿：左边黑、右边白 → 左边黑点即为边界起始点
                if(image[search_row][j - 1] == BLACK && image[search_row][j] == WHITE)
                {
                    left_start_row = search_row;                                // 记录起始点行坐标
                    left_start_col = j - 1;                                     // 黑点列坐标（边界线上）
                    left_find_flag = 1;
                    break;
                }
            }
        }

        // ==================== 寻找右边界起始点 ====================
        // 从中间列向右扫描到最右边
        if(!right_find_flag)
        {
            for(j = IMG_W / 2; j < IMG_W -3; j++)                         // 从中间向右，只扫到2/3宽度
            {
                // 检测上升沿：当前白、右边黑 → 右边黑点即为边界起始点
                if(image[search_row][j] == WHITE && image[search_row][j + 1] == BLACK)
                {
                    right_start_row = search_row;                               // 记录起始点行坐标
                    right_start_col = j + 1;                                    // 黑点列坐标（边界线上）
                    right_find_flag = 1;
                    break;
                }
            }
        }

        // ---- 如果左右边界都找到了，停止搜索 ----
        if(left_find_flag && right_find_flag)
            break;
    }

    // ---- 如果没找到左边界，默认从左边框黑线开始爬 ----
    if(!left_find_flag)
    {
        left_start_row = IMG_H - 3;
        left_start_col = 1;                                                     // 左边框第1列是黑点（image_draw_border画的）
        left_find_flag = 1;
    }

    // ---- 如果没找到右边界，默认从右边框黑线开始爬 ----
    if(!right_find_flag)
    {
        right_start_row = IMG_H - 3;
        right_start_col = IMG_W - 2;                                            // 右边框倒数第2列是黑点
        right_find_flag = 1;
    }
}

//==================================================== 八邻域边界追踪（爬线算法） ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：boundary_trace
// 功能：八邻域爬线算法 —— 从起始点沿黑白边界线追踪，提取完整的赛道边界
// 参数：image —— 二值化图像数组（IMG_H × IMG_W）
// 返回：void
//
// 原理说明：
//   八邻域方向编号（以当前点为中心）：
//     左上(7)  正上(0)  右上(6)
//     正左(4)   当前点   正右(5)
//     左下(3)  正下(1)  右下(2)
//
//   爬线核心判断条件：
//     某个邻域点是黑色（边界线），且其相邻的某个方向是白色（赛道）
//     → 说明该邻域点正好处于黑白交界处，是边界线的一部分
//
//   dire_left/dire_right 记录上一次移动的方向，用于避免"回头"：
//     例如上一次是从"正右"(5)方向找到当前点，那本次搜索就排除"正左"(4)方向
//     （因为正左就是回到上一步的位置，会形成死循环）
//
// 左右边界的方向判断逻辑是镜像的：
//   左边界：黑色在左/上，白色在右/下 → 赛道在黑色右边
//   右边界：黑色在右/上，白色在左/下 → 赛道在黑色左边
//-------------------------------------------------------------------------------------------------------------------
void boundary_trace(uint8 image[IMG_H][IMG_W])
{
    // ---- 1. 初始化边界点计数器 ----
    left_edge_count = 0;
    right_edge_count = 0;

    // ==================== 左边界爬线 ====================
    if(left_find_flag)                                                          // 如果左边界起始点已找到
    {
        // ---- 1.1 记录边界起始点（第0个点） ----
        left_edge[0].row = left_start_row;                                      // 起始点行坐标
        left_edge[0].col = left_start_col;                                      // 起始点列坐标
        left_edge[0].flag = 1;                                                  // 标记为有效的边界点

        int16 curr_row = left_start_row;                                        // 当前追踪点的行坐标
        int16 curr_col = left_start_col;                                        // 当前追踪点的列坐标
        uint8 dire_left = 0;                                                    // 初始化来源方向为"正上"(0)

        // ---- 1.2 开始八邻域迭代爬线 ----
        for(int i = 1; i < BOUNDARY_SEARCH_MAX; i++)
        {
            // ---- 左边界越界检查（col范围 [0, IMG_W*2/3]） ----
            if(curr_row < BOUNDARY_SEARCH_END || curr_row >= IMG_H - 2
               || curr_col < 1 || curr_col >= IMG_W * 2 / 3)
                break;

            // ======== 按优先级依次检查7个邻域方向 ========
            // 注意：正下方(方向1)不检查，因为爬线方向总体上是从下往上

            // --- 方向7：左上 ---
            // 条件：不是从上一步的"右下(2)"来的（避免回头）
            //       image[curr_row-1][curr_col-1]==BLACK  → 左上角像素是黑色（边界线）
            //       image[curr_row-1][curr_col]==WHITE    → 正上方像素是白色（赛道）
            //       黑色在左上、白色在正上 → 黑白交界 → 这就是边界！
            if(dire_left != 2
               && image[curr_row - 1][curr_col - 1] == BLACK
               && image[curr_row - 1][curr_col] == WHITE)
            {
                curr_row = curr_row - 1;                                        // 向上移动一行
                curr_col = curr_col - 1;                                        // 向左移动一列
                left_edge_count++;                                              // 边界点计数+1
                dire_left = 7;                                                  // 记录本次移动方向为"左上"
                left_edge[i].row = curr_row;
                left_edge[i].col = curr_col;
                left_edge[i].flag = 1;
            }

            // --- 方向6：右上 ---
            // 条件：不是从"左下(3)"来的
            //       image[curr_row-1][curr_col+1]==BLACK  → 右上角是黑色
            //       image[curr_row][curr_col+1]==WHITE    → 正右方是白色
            else if(dire_left != 3
                    && image[curr_row - 1][curr_col + 1] == BLACK
                    && image[curr_row][curr_col + 1] == WHITE)
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                left_edge_count++;
                dire_left = 6;
                left_edge[i].row = curr_row;
                left_edge[i].col = curr_col;
                left_edge[i].flag = 1;
            }

            // --- 方向0：正上 ---
            // 无来向过滤（正上方向不会和之前的移动方向形成回头关系）
            //       image[curr_row-1][curr_col]==BLACK     → 正上方是黑色
            //       image[curr_row-1][curr_col+1]==WHITE   → 右上角是白色
            else if(image[curr_row - 1][curr_col] == BLACK
                    && image[curr_row - 1][curr_col + 1] == WHITE)
            {
                curr_row = curr_row - 1;
                left_edge_count++;
                dire_left = 0;
                left_edge[i].row = curr_row;
                left_edge[i].col = curr_col;
                left_edge[i].flag = 1;
            }

            // --- 方向4：正左 ---
            // 条件：不是从"正右(5)"来的
            //       image[curr_row][curr_col-1]==BLACK     → 正左方是黑色
            //       image[curr_row-1][curr_col-1]==WHITE   → 左上方是白色
            else if(dire_left != 5
                    && image[curr_row][curr_col - 1] == BLACK
                    && image[curr_row - 1][curr_col - 1] == WHITE)
            {
                curr_col = curr_col - 1;                                        // 只改变列坐标（同一行内左移）
                left_edge_count++;
                dire_left = 4;
                left_edge[i].row = curr_row;
                left_edge[i].col = curr_col;
                left_edge[i].flag = 1;
            }

            // --- 方向5：正右 ---
            // 条件：不是从"正左(4)"来的
            //       image[curr_row][curr_col+1]==BLACK     → 正右方是黑色
            //       image[curr_row+1][curr_col+1]==WHITE   → 右下方是白色
            else if(dire_left != 4
                    && image[curr_row][curr_col + 1] == BLACK
                    && image[curr_row + 1][curr_col + 1] == WHITE)
            {
                curr_col = curr_col + 1;
                left_edge_count++;
                dire_left = 5;
                left_edge[i].row = curr_row;
                left_edge[i].col = curr_col;
                left_edge[i].flag = 1;
            }

            // --- 方向3：左下 ---
            // 条件：不是从"右上(6)"来的
            //       image[curr_row+1][curr_col-1]==BLACK   → 左下方是黑色
            //       image[curr_row][curr_col-1]==WHITE     → 正左方是白色
            else if(dire_left != 6
                    && image[curr_row + 1][curr_col - 1] == BLACK
                    && image[curr_row][curr_col - 1] == WHITE)
            {
                curr_row = curr_row + 1;                                        // 向下移动一行（追踪回旋的边界）
                curr_col = curr_col - 1;
                left_edge_count++;
                dire_left = 3;
                left_edge[i].row = curr_row;
                left_edge[i].col = curr_col;
                left_edge[i].flag = 1;
            }

            // --- 方向2：右下 ---
            // 条件：不是从"左上(7)"来的
            //       image[curr_row+1][curr_col+1]==BLACK   → 右下方是黑色
            //       image[curr_row+1][curr_col]==WHITE     → 正下方是白色
            else if(dire_left != 7
                    && image[curr_row + 1][curr_col + 1] == BLACK
                    && image[curr_row + 1][curr_col] == WHITE)
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                left_edge_count++;
                dire_left = 2;
                left_edge[i].row = curr_row;
                left_edge[i].col = curr_col;
                left_edge[i].flag = 1;
            }

            // --- 以上7个方向都不满足 → 没有下一个边界点了，停止爬线 ---
            else
                break;
        }

        // 左边界实际有效点数 = 计数+1（加上第0个起始点）
        left_edge_count = left_edge_count + 1;
    }

    // ==================== 右边界爬线 ====================
    // 原理与左边界相同，但方向编号的映射关系不同
    // 因为左右边界线的黑白位置关系是镜像的
    if(right_find_flag)                                                         // 如果右边界起始点已找到
    {
        // ---- 2.1 记录起始点 ----
        right_edge[0].row = right_start_row;
        right_edge[0].col = right_start_col;
        right_edge[0].flag = 1;

        int16 curr_row = right_start_row;
        int16 curr_col = right_start_col;
        uint8 dire_right = 0;                                                   // 初始化来源方向

        // ---- 2.2 开始八邻域迭代爬线 ----
        for(int i = 1; i < BOUNDARY_SEARCH_MAX; i++)
        {
            // ---- 右边界越界检查（col范围 [IMG_W/3, IMG_W-1]） ----
            if(curr_row < BOUNDARY_SEARCH_END || curr_row >= IMG_H - 2
               || curr_col <= IMG_W / 3 || curr_col >= IMG_W - 1)
                break;

            // --- 方向6：右上 ---
            // image[curr_row-1][curr_col+1]==BLACK  → 右上黑
            // image[curr_row-1][curr_col]==WHITE    → 正上白
            if(dire_right != 3
               && image[curr_row - 1][curr_col + 1] == BLACK
               && image[curr_row - 1][curr_col] == WHITE)
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                right_edge_count++;
                dire_right = 6;
                right_edge[i].row = curr_row;
                right_edge[i].col = curr_col;
                right_edge[i].flag = 1;
            }

            // --- 方向7：左上 ---
            // image[curr_row-1][curr_col-1]==BLACK  → 左上黑
            // image[curr_row][curr_col-1]==WHITE    → 正左白
            else if(dire_right != 2
                    && image[curr_row - 1][curr_col - 1] == BLACK
                    && image[curr_row][curr_col - 1] == WHITE)
            {
                curr_row = curr_row - 1;
                curr_col = curr_col - 1;
                right_edge_count++;
                dire_right = 7;
                right_edge[i].row = curr_row;
                right_edge[i].col = curr_col;
                right_edge[i].flag = 1;
            }

            // --- 方向0：正上 ---
            // image[curr_row-1][curr_col]==BLACK     → 正上黑
            // image[curr_row-1][curr_col-1]==WHITE   → 左上白
            else if(image[curr_row - 1][curr_col] == BLACK
                    && image[curr_row - 1][curr_col - 1] == WHITE)
            {
                curr_row = curr_row - 1;
                right_edge_count++;
                dire_right = 0;
                right_edge[i].row = curr_row;
                right_edge[i].col = curr_col;
                right_edge[i].flag = 1;
            }

            // --- 方向5：正右 ---
            // image[curr_row][curr_col+1]==BLACK     → 正右黑
            // image[curr_row-1][curr_col+1]==WHITE   → 右上白
            else if(dire_right != 4
                    && image[curr_row][curr_col + 1] == BLACK
                    && image[curr_row - 1][curr_col + 1] == WHITE)
            {
                curr_col = curr_col + 1;
                right_edge_count++;
                dire_right = 5;
                right_edge[i].row = curr_row;
                right_edge[i].col = curr_col;
                right_edge[i].flag = 1;
            }

            // --- 方向4：正左 ---
            // image[curr_row][curr_col-1]==BLACK     → 正左黑
            // image[curr_row+1][curr_col-1]==WHITE   → 左下白
            else if(dire_right != 5
                    && image[curr_row][curr_col - 1] == BLACK
                    && image[curr_row + 1][curr_col - 1] == WHITE)
            {
                curr_col = curr_col - 1;
                right_edge_count++;
                dire_right = 4;
                right_edge[i].row = curr_row;
                right_edge[i].col = curr_col;
                right_edge[i].flag = 1;
            }

            // --- 方向3：左下 ---
            // image[curr_row+1][curr_col-1]==BLACK   → 左下黑
            // image[curr_row+1][curr_col]==WHITE     → 正下白
            else if(dire_right != 6
                    && image[curr_row + 1][curr_col - 1] == BLACK
                    && image[curr_row + 1][curr_col] == WHITE)
            {
                curr_row = curr_row + 1;
                curr_col = curr_col - 1;
                right_edge_count++;
                dire_right = 3;
                right_edge[i].row = curr_row;
                right_edge[i].col = curr_col;
                right_edge[i].flag = 1;
            }

            // --- 方向2：右下 ---
            // image[curr_row+1][curr_col+1]==BLACK   → 右下黑
            // image[curr_row][curr_col+1]==WHITE     → 正右白
            else if(dire_right != 7
                    && image[curr_row + 1][curr_col + 1] == BLACK
                    && image[curr_row][curr_col + 1] == WHITE)
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                right_edge_count++;
                dire_right = 2;
                right_edge[i].row = curr_row;
                right_edge[i].col = curr_col;
                right_edge[i].flag = 1;
            }
            else
                break;
        }

        // 右边界实际有效点数 = 计数+1（加上第0个起始点）
        right_edge_count = right_edge_count + 1;
    }
}

//==================================================== 寻找 A/B/C/D 关键点 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：find_key_points
// 功能：寻找赛道边界的四个关键拐角点 A、B、C、D
// 参数：image —— 二值化图像数组（IMG_H × IMG_W）
// 返回：void
// 说明：
//   A点 —— 左边界线在图像底部的上升沿位置（赛道左边界的起点）
//   B点 —— 右边界线在图像底部的上升沿位置（赛道右边界的起点）
//   C点 —— 左边界线向上追踪的拐点/终点（边界线方向发生明显变化的位置）
//   D点 —— 右边界线向上追踪的拐点/终点
//
// 搜索策略：
//   1. 在图像最底行从中间列向左右扫描，找到A、B点（黑白交界处）
//   2. 从A点上方开始逐行向上追踪C点：
//      - 从A点附近向左上方向逐行搜索
//      - 找到白色赛道区域后回退一列到黑白交界处
//      - 判断该位置上方是否还有黑色边界线延续
//   3. 从B点上方开始逐行向上追踪D点（与C点对称）
//-------------------------------------------------------------------------------------------------------------------
void find_key_points(uint8 image[IMG_H][IMG_W])
{
    int16 i, j;

    // ---- 1. A/B点直接使用 find_boundary_start 的结果（避免重复扫描） ----
    point_A_row = left_start_row;
    point_A_col = left_start_col;
    point_B_row = right_start_row;
    point_B_col = right_start_col;

    // ---- 2. 初始化 C/D 点 ----
    point_C_row = 0;
    point_C_col = 0;
    point_D_row = 0;
    point_D_col = 0;

    // ---- 3. 向上迭代追踪C点（左边界线上端拐点） ----
    // 从A点左上方开始，逐行向上搜索
    point_C_row = point_A_row - 1;                                              // C点初始Y坐标 = A点上方一行
    point_C_col = point_A_col - 1;                                              // C点初始X坐标 = A点左边一列

    for(i = point_C_row; i > BOUNDARY_SEARCH_END; i--)
    {
        // 在当前行，从左向右逐列扫描，找白点（赛道）
        for(j = point_C_col; j < IMG_W - 2; j++)
        {
            if(image[i][j] == WHITE)                                            // 找到赛道区域
            {
                point_C_col = j - 1;                                            // 退回到黑白交界（黑点=边界线）
                break;
            }
        }

        // 判断上一行边界位置是否为白点（白点=边界线断开=拐点）
        if(i > BOUNDARY_SEARCH_END + 1
           && point_C_col > 2 && point_C_col < IMG_W - 2
           && image[i - 1][point_C_col] == WHITE)                               // 上方断开→拐点
        {
            point_C_row = i;                                                    // 记录C点Y坐标
            break;
        }
    }

    // ---- 4. 向上迭代追踪D点（右边界线上端拐点） ----
    point_D_row = point_B_row - 1;                                              // D点初始Y坐标 = B点上方一行
    point_D_col = point_B_col + 1;                                              // D点初始X坐标 = B点右边一列

    for(i = point_D_row; i > BOUNDARY_SEARCH_END; i--)
    {
        // 在当前行，从右向左逐列扫描，找白点（赛道）
        for(j = point_D_col; j > 2; j--)
        {
            if(image[i][j] == WHITE)                                            // 找到赛道区域
            {
                point_D_col = j + 1;                                            // 前进到黑白交界（黑点=边界线）
                break;
            }
        }

        // 判断上一行边界位置是否为白点（白点=边界线断开=拐点）
        if(i > BOUNDARY_SEARCH_END + 1
           && point_D_col > 2 && point_D_col < IMG_W - 2
           && image[i - 1][point_D_col] == WHITE)                               // 上方断开→拐点
        {
            point_D_row = i;                                                    // 记录D点Y坐标
            break;
        }
    }
}

//==================================================== 十字路口判断与补线 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：crossroad_fix
// 功能：十字路口判断与补线处理
// 参数：image —— 二值化图像数组（IMG_H × IMG_W）
// 返回：void
// 说明：
//   判断条件：
//     1. |CY - DY| < 10 —— C和D的高度差小于10行（左右拐点高度接近，对称）
//     2. CY > 15 且 DY > 15 —— 拐点在前方较远处
//     3. 拐点�方区域黑点很少（赛道边界断开）
//   补线方法：
//     十字路口处赛道边界线会断开（因为路口区域没有边界线）
//     需要人为沿AC方向和BD方向画延长线（黑色边界线）
//     使小车能看到"虚拟的赛道边界"，从而正确通过十字路口
//-------------------------------------------------------------------------------------------------------------------
void crossroad_fix(uint8 image[IMG_H][IMG_W])
{
    int16 cy_diff;                                                              // C点和D点的高度差（取绝对值）

    // ---- 1. 检查C、D点是否存在且有效 ----
    if(point_C_row == 0 || point_D_row == 0)
        return;                                                                 // 如果拐点无效，直接返回

    // ---- 2. 计算C、D点的高度差 ----
    if(point_C_row > point_D_row)
        cy_diff = point_C_row - point_D_row;
    else
        cy_diff = point_D_row - point_C_row;

    // ---- 3. 判断是否为十字路口 ----
    // 条件：C和D高度接近（差<10行）、且拐点在前方较远处（行号>15）
    if(cy_diff < 10 && point_C_row > 15 && point_D_row > 15)
    {
        uint8 check_row;                                                        // 用于统计的参考行
        uint8 black_count = 0;                                                  // 上方区域的黑点计数器

        // 取C、D中较小的Y值（较高的行号），作为十字路口的上边界参考
        if(point_C_row < point_D_row)
            check_row = point_C_row;
        else
            check_row = point_D_row;

        // ---- 4. 在拐点上方区域进行抽样统计 ----
        // Y方向：每隔2行抽样
        // X方向：每隔5列抽样
        // 如果上方区域黑点很少，说明赛道边界确实断开了
        {
            int16 i, j;
            for(i = check_row; i > check_row - 10 && i > 0; i -= 2)
            {
                for(j = 10; j < IMG_W - 10; j += 5)
                {
                    if(image[i][j] == BLACK)
                    {
                        black_count++;
                    }
                }
            }
        }

        // ---- 5. 如果上方黑点少于阈值，确认为十字路口并补线 ----
        if(black_count < 10)
        {
            float k_left, k_right;                                              // 补线斜率
            int16 i;

            // ======== 补左侧延长线（A→C方向） ========
            // 斜率 = ΔX / ΔY（注意：图像坐标系Y轴向下）
            // 防止除以零
            if(point_C_row != point_A_row)
            {
                k_left = (float)(point_C_col - point_A_col) / (float)(point_C_row - point_A_row);

                // 从C点向上延长补线
                for(i = point_C_row; i > point_C_row - 25 && i > 0; i--)
                {
                    int16 offset = (int16)((i - point_C_row) * k_left);         // i<C_row，offset为负→向左延伸
                    int16 draw_col = point_C_col + offset;                      // 左边界补线继续向左上方

                    // 限幅检查
                    if(draw_col > 2 && draw_col < IMG_W - 2)
                    {
                        image[i][draw_col] = BLACK;                             // 把对应位置涂黑（边界线主体）
                    }
                    if(draw_col - 1 > 2 && draw_col - 1 < IMG_W - 2)
                    {
                        image[i][draw_col - 1] = BLACK;                         // 相邻列也涂黑（2像素宽边界线）
                    }
                }
            }

            // ======== 补右侧延长线（B→D方向） ========
            if(point_D_row != point_B_row)
            {
                k_right = (float)(point_D_col - point_B_col) / (float)(point_D_row - point_B_row);

                // 从D点向上延长补线
                for(i = point_D_row; i > point_D_row - 25 && i > 0; i--)
                {
                    int16 offset = (int16)((i - point_D_row) * k_right);        // i<D_row，offset符号与k_right一致
                    int16 draw_col = point_D_col + offset;                      // 右边界补线继续延伸

                    // 限幅检查
                    if(draw_col > 2 && draw_col < IMG_W - 2)
                    {
                        image[i][draw_col] = BLACK;
                    }
                    if(draw_col - 1 > 2 && draw_col - 1 < IMG_W - 2)
                    {
                        image[i][draw_col - 1] = BLACK;
                    }
                }
            }
        }
    }
}

//==================================================== 赛道中线提取 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：extract_centerline
// 功能：根据八邻域爬线得到的左右边界点，逐行提取赛道中线
// 参数：image —— 二值化图像数组（IMG_H × IMG_W）（保留参数，用于底部兼容）
// 返回：void
//
// 核心思路：
//   八邻域爬线已经精确追踪了左右边界线（left_edge[] 和 right_edge[]），
//   每个边界点记录了 (row, col) 坐标。
//   中线提取不再重新扫描二值化图像，而是直接用这些边界点：
//     1. 将边界点按行号映射到 left_boundary[] 和 right_boundary[]
//     2. 同一行有多个边界点时：左边界取最大值（最靠右的黑点），右边界取最小值
//     3. 没有边界点的行：用最近的有效行插值填充
//     4. 中线 = (左边界 + 右边界) / 2
//
//   这种方法比"从上一行中线向左右扫描"更精确，因为：
//     - 八邻域追踪已经过滤了噪声
//     - 不会因某行误判而逐行累积偏差
//     - 边界点的连续性更好
//
//   结果存储在：
//     center_line[]    —— 每行的中线X坐标（0~79）
//     left_boundary[]  —— 每行的左边界X坐标
//     right_boundary[] —— 每行的右边界X坐标
//-------------------------------------------------------------------------------------------------------------------
void extract_centerline(uint8 image[IMG_H][IMG_W])
{
    int16 i, k;

    // ---- 1. 初始化边界数组为无效值（用 0xFF 标记"未填充"） ----
    for(i = 0; i < IMG_H; i++)
    {
        left_boundary[i] = 0xFF;                                                // 0xFF = 无效标记
        right_boundary[i] = 0xFF;
    }

    // ---- 2. 从八邻域左边界点数组填充 left_boundary[] ----
    // 同一行可能有多个边界点（边界线横向走），取 col 最大的（最靠近图像中心）
    for(k = 0; k < left_edge_count && k < BOUNDARY_SEARCH_MAX; k++)
    {
        if(left_edge[k].flag == 1)                                              // 有效的边界点
        {
            int16 row = left_edge[k].row;
            int16 col = left_edge[k].col;

            if(row >= 0 && row < IMG_H && col >= 0 && col < IMG_W)
            {
                if(left_boundary[row] == 0xFF                                   // 该行首次填充
                   || col > left_boundary[row])                                 // 或取更靠右的黑点
                {
                    left_boundary[row] = (uint8)col;
                }
            }
        }
    }

    // ---- 3. 从八邻域右边界点数组填充 right_boundary[] ----
    // 同一行有多个边界点，取 col 最小的（最靠近图像中心）
    for(k = 0; k < right_edge_count && k < BOUNDARY_SEARCH_MAX; k++)
    {
        if(right_edge[k].flag == 1)
        {
            int16 row = right_edge[k].row;
            int16 col = right_edge[k].col;

            if(row >= 0 && row < IMG_H && col >= 0 && col < IMG_W)
            {
                if(right_boundary[row] == 0xFF
                   || col < right_boundary[row])
                {
                    right_boundary[row] = (uint8)col;
                }
            }
        }
    }

    // ---- 3.5 标记哪些行的中线来自真实边界点（在插值填充之前） ----
    for(i = 0; i < IMG_H; i++)
    {
        // 左右边界都有真实数据 → 中线有效
        center_line_valid[i] = (left_boundary[i] != 0xFF && right_boundary[i] != 0xFF) ? 1 : 0;
    }

    // ---- 4. 对于没有边界点的行，用最近的有效行插值填充 ----
    // 4.1 从下往上填充左边界空缺（用下方最近的有效值）
    {
        uint8 last_valid = 0;                                                   // 上一个有效值
        uint8 has_valid = 0;                                                    // 是否已遇到有效值
        for(i = IMG_H - 1; i >= 0; i--)
        {
            if(left_boundary[i] != 0xFF)
            {
                last_valid = left_boundary[i];
                has_valid = 1;
            }
            else if(has_valid)
            {
                left_boundary[i] = last_valid;                                  // 用下方有效值填充
            }
        }
    }

    // 4.2 从上往下填充左边界空缺（用上方最近的有效值）
    {
        uint8 last_valid = 0;
        uint8 has_valid = 0;
        for(i = 0; i < IMG_H; i++)
        {
            if(left_boundary[i] != 0xFF)
            {
                last_valid = left_boundary[i];
                has_valid = 1;
            }
            else if(has_valid)
            {
                left_boundary[i] = last_valid;                                  // 用上方有效值填充
            }
            else
            {
                left_boundary[i] = 0;                                           // 顶部无有效值 → 默认最左边
            }
        }
    }

    // 4.3 从下往上填充右边界空缺
    {
        uint8 last_valid = IMG_W - 1;
        uint8 has_valid = 0;
        for(i = IMG_H - 1; i >= 0; i--)
        {
            if(right_boundary[i] != 0xFF)
            {
                last_valid = right_boundary[i];
                has_valid = 1;
            }
            else if(has_valid)
            {
                right_boundary[i] = last_valid;
            }
        }
    }

    // 4.4 从上往下填充右边界空缺
    {
        uint8 last_valid = IMG_W - 1;
        uint8 has_valid = 0;
        for(i = 0; i < IMG_H; i++)
        {
            if(right_boundary[i] != 0xFF)
            {
                last_valid = right_boundary[i];
                has_valid = 1;
            }
            else if(has_valid)
            {
                right_boundary[i] = last_valid;
            }
            else
            {
                right_boundary[i] = IMG_W - 1;                                  // 顶部无有效值 → 默认最右边
            }
        }
    }

    // ---- 4.5 跳变滤波：相邻两行列坐标差不超过15（从底部向上约束） ----
    for(i = IMG_H - 2; i >= 0; i--)
    {
        int16 diff;
        diff = (int16)left_boundary[i] - (int16)left_boundary[i + 1];
        if(diff > 15)   left_boundary[i] = left_boundary[i + 1] + 15;
        if(diff < -15)  left_boundary[i] = left_boundary[i + 1] - 15;

        diff = (int16)right_boundary[i] - (int16)right_boundary[i + 1];
        if(diff > 15)   right_boundary[i] = right_boundary[i + 1] + 15;
        if(diff < -15)  right_boundary[i] = right_boundary[i + 1] - 15;
    }

    // ---- 5. 逐行计算中线 = (左边界 + 右边界) / 2 ----
    for(i = 0; i < IMG_H; i++)
    {
        // 安全检查：左边界不能大于右边界
//        if(left_boundary[i] < right_boundary[i])
//        {
            center_line[i] = (left_boundary[i] + right_boundary[i]) / 2;
//        }
//        else
//        {
//            // 异常情况：使用相邻行的中线或图像中心
//            if(i < IMG_H - 1)
//                center_line[i] = center_line[i + 1];
//            else
//                center_line[i] = IMG_W / 2;
//        }
    }
}

//==================================================== 清除边界点数据 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：clear_edge_data
// 功能：清除（初始化）所有已找到的左右边界点数据
// 参数：void
// 返回：void
// 说明：在每次开始新一轮边界搜索前调用，避免上次搜索的残留数据干扰
//-------------------------------------------------------------------------------------------------------------------
void clear_edge_data(void)
{
    int16 i;

    // 遍历左边界点数组，将所有点清零
    for(i = 0; i < BOUNDARY_SEARCH_MAX; i++)
    {
        left_edge[i].row = 0;
        left_edge[i].col = 0;
        left_edge[i].flag = 0;
    }

    // 遍历右边界点数组，将所有点清零
    for(i = 0; i < BOUNDARY_SEARCH_MAX; i++)
    {
        right_edge[i].row = 0;
        right_edge[i].col = 0;
        right_edge[i].flag = 0;
    }

    // 重置各项计数器和标志
    left_edge_count = 0;
    right_edge_count = 0;
    left_lose_rows = 0;
    right_lose_rows = 0;
}

//==================================================== 加权位置计算 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：get_weight_position_2
// 功能：只使用真实边界点得到的中线行（center_line_valid=1）计算加权位置
// 返回：float —— 加权后的中线偏移量（相对于图像中心的偏差）
//
// 权重设计：越靠近车身（行号越大）权重越大
//   权重 = row + 1（底部行号大，贡献更大）
//
// 只统计有效行（center_line_valid[i]==1），忽略插值填充的行
//-------------------------------------------------------------------------------------------------------------------
// float get_weight_position_2(void)
// {
//     float weighted_sum = 0.0f;                                                  // 加权累积（中线X * 权重）
//     float weight_total = 0.0f;                                                  // 权重总和
//     int16 i;

//     for(i = 0; i < IMG_H; i++)
//     {
//         if(center_line_valid[i] == 1)                                           // 只使用真实边界点对应的中线
//         {
//             float weight = (float)(i + 1);                                      // 行号越大（越靠近车身）权重越大
//             weighted_sum += (float)center_line[i] * weight;
//             weight_total += weight;
//         }
//     }

//     if(weight_total > 0.0f)
//     {
//         float weighted_center = weighted_sum / weight_total;                    // 加权平均中线位置
//         return weighted_center - (float)(IMG_W / 2);                            // 减去图像中心 → 偏差
//     }
//     return 0.0f;                                                                // 没有有效行 → 返回0
// }

//==================================================== 完整图像处理管线 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：image_process_pipeline
// 功能：执行完整的图像处理管线（一站式调用）
// 参数：void
// 返回：void
// 说明：
//   按照以下顺序依次执行所有处理步骤：
//     1. 大津法计算二值化阈值（基于原始灰度图像）
//     2. 图像二值化（灰度图 → 黑白二值图）
//     3. 图像外围画黑框（为八邻域爬线提供安全边界）
//     4. 寻找边界起始点（在图像底部找到左右边界的起点）
//     5. 八邻域边界追踪（从起点向上追踪完整边界线）
//     6. 寻找A/B/C/D关键点（找到边界的拐角位置）
//     7. 十字路口判断与补线（在十字路口处补画虚拟边界线）
//     8. 提取赛道中线（逐行计算左右边界的中点）
//
//   所有结果存储在全局变量中：
//     otsu_threshold   —— 二值化阈值
//     binary_image[][] —— 二值化图像
//     left_edge[]      —— 左边界点数组
//     right_edge[]     —— 右边界点数组
//     point_A/B/C/D    —— 四个关键点
//     center_line[]    —— 赛道中线数组
//     left_boundary[]  —— 左边界数组
//     right_boundary[] —— 右边界数组
//
//   使用前提：mt9v03x_image[][] 中已有最新一帧的摄像头灰度图像
//           且 mt9v03x_finish_flag 已置位（图像采集完成）
//-------------------------------------------------------------------------------------------------------------------
void image_process_pipeline(void)
{
    // ---- 第1步：清除上次的边界点数据 ----
    clear_edge_data();

    // ---- 第2步：根据模式计算二值化阈值 ----
    if(threshold_mode)                                                          // 模式1：大津法
    {
        otsu_threshold = otsu_threshold_calc((uint8 *)mt9v03x_image);
    }
    else                                                                        // 模式0：固定阈值
    {
        otsu_threshold = fixed_threshold;
    }

    // ---- 第3步：图像二值化 ----
    image_binarize((uint8 *)mt9v03x_image, binary_image, otsu_threshold);

    // ---- 第4步：图像外围画黑框 ----
    // 为八邻域爬线提供安全边界，防止越界访问
    image_draw_border(binary_image);

    // ---- 第5步：寻找左右边界起始点 ----
    // 在图像底部找到赛道左右边界的起点位置
    find_boundary_start(binary_image);

    // ---- 第7步：寻找A/B/C/D关键点 ----
    // 在二值化图像中找到四个关键拐角点
    find_key_points(binary_image);

    // ---- 第8步：十字路口判断与补线 ----
    // 检测十字路口并在必要时补画虚拟边界线
    crossroad_fix(binary_image);
    
    // ---- 第6步：八邻域边界追踪 ----
    // 从起始点向上爬线，提取完整的赛道左右边界
    boundary_trace(binary_image);

    // ---- 第9步：提取赛道中线 ----
    // 根据边界线逐行计算赛道中心线坐标
    extract_centerline(binary_image);
}
