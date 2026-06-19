#ifndef __GUI_MENU_H
#define __GUI_MENU_H

#include "main.h"
#include "st7789v.h"
#include "ec11.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file    gui_menu.h
 * @brief   轻量多级菜单系统
 *          - 内置 6×8 ASCII 字库，无需 GuiLite C++ 对象
 *          - 直接调用 st7789v HAL 驱动绘图
 *          - EC11 编码器：CW=下移，CCW=上移，短按=确认，长按=返回
 ******************************************************************************
 */

/* ============================================================
 * 屏幕参数
 * ============================================================ */
#define GUI_SCREEN_W        240
#define GUI_SCREEN_H        280

/* ============================================================
 * 字体参数（6×8 点阵，可 2× 缩放为 12×16）
 * ============================================================ */
#define FONT_W              6
#define FONT_H              8
#define FONT_SCALE          2               /* 放大倍数：1=6×8, 2=12×16 */
#define CHAR_W              (FONT_W  * FONT_SCALE)
#define CHAR_H              (FONT_H  * FONT_SCALE)

/* ============================================================
 * 菜单布局
 * ============================================================ */
#define MENU_TITLE_H        36              /* 标题栏高度 */
#define MENU_ITEM_H         44              /* 每个条目高度 */
#define MENU_ITEM_MAX       5               /* 屏幕最多可见条目数 */
#define MENU_TEXT_X         18              /* 文字起始 X */
#define MENU_TEXT_Y_OFF     ((MENU_ITEM_H - CHAR_H) / 2)  /* 文字垂直居中偏移 */
#define MENU_INDICATOR_W    5               /* 左侧选中指示条宽 */
#define MENU_ARROW_X        (GUI_SCREEN_W - CHAR_W - 8)   /* ">" 箭头 X */

/* ============================================================
 * 颜色（RGB565）
 * ============================================================ */
#define GUI_COL_BG          0x0000U         /* 背景黑 */
#define GUI_COL_TITLE_BG    0x1082U         /* 标题深灰蓝 */
#define GUI_COL_TITLE_FG    0xFFFFU         /* 标题白 */
#define GUI_COL_ITEM_BG     0x0000U         /* 条目背景黑 */
#define GUI_COL_ITEM_FG     0xC618U         /* 条目文字浅灰 */
#define GUI_COL_SEL_BG      0x0329U         /* 选中深蓝 */
#define GUI_COL_SEL_FG      0xFFFFU         /* 选中文字白 */
#define GUI_COL_INDICATOR   0x07E0U         /* 指示条绿 */
#define GUI_COL_ARROW       0x8410U         /* 箭头灰 */
#define GUI_COL_DIVIDER     0x2104U         /* 分隔线深灰 */
#define GUI_COL_SCROLLBAR   0x18C3U         /* 滚动条背景 */

/* ============================================================
 * 长按阈值（ms）
 * ============================================================ */
#define EC11_LONG_MS        800U

/* ============================================================
 * 菜单数据结构
 * ============================================================ */
typedef void (*MenuAction_t)(void);

typedef struct MenuItem_t MenuItem_t;
struct MenuItem_t
{
    const char   *label;        /* 条目文字（ASCII）*/
    MenuItem_t   *children;     /* 子菜单数组，叶节点为 NULL */
    uint8_t       child_count;
    MenuAction_t  action;       /* 叶节点回调：执行一次性动作并立即返回菜单 */
    struct CustomPage_t *page;  /* 叶节点回调：进入自定义页面，接管屏幕直到长按返回 */
};

/* ============================================================
 * 自定义页面（用于需要持续刷新/周期任务、且可能有自己内部交互逻辑/
 * 多级返回的子界面，例如 Simulation mode：内部有"参数列表"和
 * "数值调节"两层状态，长按在不同层含义不同）
 *
 * 生命周期：
 *   on_enter()      —— 进入时调用一次（清屏、画静态布局）
 *   on_tick(events) —— 主循环中持续调用（周期任务 + 动态刷新）
 *                       events 为本次 Poll 到的 EC11 事件掩码，
 *                       包含 CW/CCW/KEY_SHORT/KEY_LONG 全部事件
 *                       （长按不再由框架直接拦截，交给页面自行判断）
 *                       返回值：1=页面请求退出（回到上级菜单），
 *                               0=页面继续保持（即使本次是长按，
 *                               页面可以选择"消化掉"用于内部返回，
 *                               例如从调节态返回到列表态）
 *   on_exit()       —— 页面真正退出（返回1后）时调用一次，可为NULL
 *
 *   框架行为：
 *     调用 on_tick(events)，若返回1，框架才调用 on_exit() 并退出页面；
 *     若返回0，框架什么都不做，继续停留在该页面。
 *     这样页面就能自己实现"长按从调节态返回列表态，
 *     列表态再长按才真正退出页面"这种多级返回逻辑。
 * ============================================================ */
typedef struct CustomPage_t
{
    void   (*on_enter)(void);
    uint8_t (*on_tick)(uint32_t events);
    void   (*on_exit)(void);
} CustomPage_t;

/* ============================================================
 * 菜单系统上下文
 * ============================================================ */
#define MENU_MAX_DEPTH  8U

typedef struct
{
    MenuItem_t *stack[MENU_MAX_DEPTH];  /* 各层菜单指针 */
    uint8_t     count[MENU_MAX_DEPTH];  /* 各层条目数 */
    uint8_t     cursor[MENU_MAX_DEPTH]; /* 各层光标 */
    uint8_t     scroll[MENU_MAX_DEPTH]; /* 各层滚动偏移 */
    uint8_t     depth;                  /* 当前深度 */
    uint8_t     need_redraw;

    /* 自定义页面状态 */
    CustomPage_t *active_page;          /* 非NULL表示当前处于自定义页面模式 */
} MenuCtx_t;

/* ============================================================
 * API
 * ============================================================ */
void     GUI_Menu_Init(MenuItem_t *root, uint8_t count);
void     GUI_Menu_Process(void);    /* while(1) 中调用，内部已包含 EC11_Poll() */
void     GUI_Menu_Redraw(void);

/* 字符串绘制（供外部使用）*/
void     GUI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg);
void     GUI_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);

/* 自定义页面辅助接口（供 CustomPage_t 的 on_enter/on_tick 内部调用）*/
void     GUI_Page_DrawTitle(const char *title);   /* 绘制与菜单一致风格的标题栏 */

#ifdef __cplusplus
}
#endif
#endif /* __GUI_MENU_H */
