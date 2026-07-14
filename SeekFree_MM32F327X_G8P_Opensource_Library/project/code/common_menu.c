/*********************************************************************************************************************
* 文件名称          common_menu
* 功能描述          智能车摄像头扫描巡线 - 多级菜单数据结构实现（纯数据层，无UI依赖）
* 适用平台          MM32F327X_G8P
* 说明              移植自 WwuSama 多级菜单 v4.0.0
*                   本文件实现菜单树形结构的创建和管理
*                   使用双向循环链表组织菜单节点
*                   提供静态内存池用于动态分配节点
*
* 核心设计：
*   - 每个菜单项是一个 Folder_Menu 节点
*   - 父子关系通过 father/son_first 指针实现（纵向层级）
*   - 同级关系通过 next_brother/last_brother 指针实现（横向遍历）
*   - 环形链表：兄弟链表的尾节点 next_brother 指向首节点
*   - 静态内存池：预分配 MAX_MENU_ITEMS 个节点，避免频繁 malloc
*********************************************************************************************************************/

#include "common_menu.h"

//==================================================== 静态内存池 ====================================================

// 用于动态分配菜单节点的静态内存池
// 使用静态数组而非 malloc，避免堆内存碎片，适合嵌入式系统
static Folder_Menu menu_pool[MAX_MENU_ITEMS];                                   // 内存池数组
static uint16 pool_index = 0;                                                   // 内存池当前分配索引（指向下一个可用位置）

//==================================================== 内部通用创建函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：__create_menu_item
// 功能：内部通用菜单节点创建函数（所有创建函数最终调用此函数）
// 参数：father    —— 父节点指针
// 参数：me        —— 要初始化的节点指针
// 参数：name      —— 节点名称字符串
// 参数：kind      —— 节点类型
// 参数：data      —— 绑定的数据指针（文件夹为 NULL）
// 参数：isLimit   —— 是否启用限幅
// 参数：limit_min —— 最小值限制
// 参数：limit_max —— 最大值限制
// 返回：void
// 说明：
//   1. 检查父节点是否为文件夹（非文件夹不能有子节点）
//   2. 初始化节点的基本属性
//   3. 将节点插入父节点的子链表末尾
//   4. 更新父节点的子节点计数
//-------------------------------------------------------------------------------------------------------------------
static void __create_menu_item(Folder_Menu *father, Folder_Menu *me,
                               const char *name, Folder_Class kind,
                               void *data, uint8 isLimit,
                               float limit_min, float limit_max)
{
    // ---- 安全检查：只有文件夹类型才能拥有子节点 ----
    if(father->kind != Normal_Folder)
        return;

    // ---- 初始化节点的通用属性 ----
    me->name = name;                                                            // 设置节点名称
    me->kind = kind;                                                            // 设置节点类型
    me->private_data = data;                                                    // 绑定数据指针
    me->isLimit = isLimit;                                                      // 是否限幅
    me->limit_min = limit_min;                                                  // 最小值
    me->limit_max = limit_max;                                                  // 最大值
    me->number_box_select = 0;                                                  // 初始未被选中编辑
    me->sons_Count = 0;                                                         // 初始无子节点

    // ---- 初始化链表指针 ----
    me->father = father;                                                        // 指向父节点
    me->next_brother = NULL;                                                    // 暂无下一个兄弟
    me->last_brother = NULL;                                                    // 暂无上一个兄弟
    me->son_first = NULL;                                                       // 暂无子节点

    // ---- 将当前节点追加到父节点的子链表末尾 ----
    if(!father->son_first)                                                      // 父节点还没有任何子节点
    {
        father->son_first = me;                                                 // 当前节点成为第一个子节点
        me->No = 1;                                                             // 编号为 1
    }
    else                                                                        // 父节点已有子节点，追加到链表末尾
    {
        Folder_Menu *p = father->son_first;                                     // 从第一个子节点开始遍历
        while(p->next_brother)                                                  // 找到链表的最后一个节点
            p = p->next_brother;
        p->next_brother = me;                                                   // 链接到链表末尾
        me->last_brother = p;                                                   // 设置前驱指针
        me->No = father->sons_Count + 1;                                        // 编号 = 当前子节点数 + 1
    }
    father->sons_Count++;                                                       // 父节点子节点计数 +1
}

//==================================================== 静态创建函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：create_Menu_Folder
// 功能：向父节点注册一个子文件夹（静态创建，节点由调用者预先分配）
//-------------------------------------------------------------------------------------------------------------------
void create_Menu_Folder(Folder_Menu *father, Folder_Menu *me, const char *name)
{
    __create_menu_item(father, me, name, Normal_Folder, NULL, 0, 0, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：create_Menu_BoolBox
// 功能：向父节点注册一个布尔型复选框（静态创建）
//-------------------------------------------------------------------------------------------------------------------
void create_Menu_BoolBox(Folder_Menu *father, Folder_Menu *me, const char *name, void *check)
{
    __create_menu_item(father, me, name, bool_Box, check, 0, 0, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：create_Menu_NumberBox
// 功能：向父节点注册一个数值型参数（无限制，静态创建）
//-------------------------------------------------------------------------------------------------------------------
void create_Menu_NumberBox(Folder_Menu *father, Folder_Menu *me, const char *name, void *number, Folder_Class kind)
{
    __create_menu_item(father, me, name, kind, number, 0, 0, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：create_Menu_LimitNumberBox
// 功能：向父节点注册一个带限幅的数值型参数（静态创建）
//-------------------------------------------------------------------------------------------------------------------
void create_Menu_LimitNumberBox(Folder_Menu *father, Folder_Menu *me, const char *name, void *number, Folder_Class kind, float limit_Min, float limit_Max)
{
    __create_menu_item(father, me, name, kind, number, 1, limit_Min, limit_Max);
}

//==================================================== 动态创建函数 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：dynamicCreate_Menu_Folder
// 功能：从内存池中动态分配并创建一个子文件夹
// 返回：新节点的指针，若内存池已满则返回 NULL
// 说明：推荐使用动态创建方式，无需手动管理节点内存
//-------------------------------------------------------------------------------------------------------------------
Folder_Menu *dynamicCreate_Menu_Folder(Folder_Menu *father, const char *name)
{
    if(pool_index >= MAX_MENU_ITEMS)                                            // 内存池已满
        return NULL;

    Folder_Menu *me = &menu_pool[pool_index++];                                 // 从内存池中分配一个节点
    create_Menu_Folder(father, me, name);                                       // 初始化为文件夹
    return me;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：dynamicCreate_Menu_NumberBox
// 功能：从内存池中动态分配并创建一个数值型参数
//-------------------------------------------------------------------------------------------------------------------
Folder_Menu *dynamicCreate_Menu_NumberBox(Folder_Menu *father, const char *name, void *number, Folder_Class kind)
{
    if(pool_index >= MAX_MENU_ITEMS)
        return NULL;

    Folder_Menu *me = &menu_pool[pool_index++];
    create_Menu_NumberBox(father, me, name, number, kind);
    return me;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：dynamicCreate_Menu_LimitNumberBox
// 功能：从内存池中动态分配并创建一个带限幅的数值型参数
//-------------------------------------------------------------------------------------------------------------------
Folder_Menu *dynamicCreate_Menu_LimitNumberBox(Folder_Menu *father, const char *name, void *number, Folder_Class kind, float limit_Min, float limit_Max)
{
    if(pool_index >= MAX_MENU_ITEMS)
        return NULL;

    Folder_Menu *me = &menu_pool[pool_index++];
    create_Menu_LimitNumberBox(father, me, name, number, kind, limit_Min, limit_Max);
    return me;
}

//==================================================== 环形链表初始化 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：All_Folder_Menu_Init
// 功能：将所有兄弟链表转换为环形链表（尾节点 next_brother 指向首节点，首节点 last_brother 指向尾节点）
// 参数：Menu —— 菜单根节点指针
// 返回：void
// 说明：
//   递归遍历整个菜单树，将每一层的兄弟链表首尾相连形成环形链表
//   环形链表的好处：在菜单中按"下"键从最后一项可以直接跳到第一项（循环导航）
//   必须在创建完所有菜单节点后调用一次
//-------------------------------------------------------------------------------------------------------------------
void All_Folder_Menu_Init(Folder_Menu *Menu)
{
    // ---- 递归终止条件：如果当前节点没有子节点，直接返回 ----
    if(Menu->son_first == NULL)
        return;

    Folder_Menu *hp = Menu->son_first;                                          // 保存首节点指针
    Folder_Menu *p = Menu->son_first;                                           // 遍历指针

    // ---- 如果只有一个子节点，递归处理后返回 ----
    if(hp->next_brother == NULL)
    {
        All_Folder_Menu_Init(p);                                                // 递归处理子节点的子树
    }

    // ---- 遍历所有兄弟节点，递归处理每个文件夹节点的子树 ----
    while(p->next_brother != NULL)
    {
        if(p->kind == Normal_Folder)
            All_Folder_Menu_Init(p);                                            // 递归处理子文件夹
        p = p->next_brother;                                                    // 移到下一个兄弟
    }

    // ---- 处理最后一个节点的子树 ----
    if(hp->next_brother != NULL)
        All_Folder_Menu_Init(p);                                                // p 现在指向最后一个节点

    // ---- 将兄弟链表首尾相连，形成环形链表 ----
    p->next_brother = hp;                                                       // 尾节点的 next 指向首节点
    hp->last_brother = p;                                                       // 首节点的 last 指向尾节点
}
