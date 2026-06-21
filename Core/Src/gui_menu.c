/**
 ******************************************************************************
 * @file    gui_menu.c
 * @brief   轻量多级菜单（纯 C，内置 6×8 ASCII 字库）
 *
 * 特点：
 *   - 完全纯 C，不调用任何 GuiLite C++ 对象
 *   - 自带 6×8 点阵字库，支持 2× 缩放为 12×16
 *   - 所有绘制通过 st7789v HAL 驱动完成
 *   - EC11 旋转编码器控制（无消抖）
 ******************************************************************************
 */

#include "gui_menu.h"
#include "debug_rtt.h"
#include <string.h>

/* ============================================================
 * 6×8 ASCII 点阵字库（从 0x20 空格开始，共 96 个字符）
 * 每个字符 6 列，每列 8 位（bit7=顶行）
 * ============================================================ */
static const uint8_t s_font6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 0x20 空格 */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* 0x21 ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* 0x22 " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* 0x23 # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* 0x24 $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* 0x25 % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* 0x26 & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* 0x27 ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* 0x28 ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* 0x29 ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* 0x2A * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* 0x2B + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* 0x2C , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* 0x2D - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* 0x2E . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* 0x2F / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0x30 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 0x31 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 0x32 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 0x33 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 0x34 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 0x35 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 0x36 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 0x37 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 0x38 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 0x39 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* 0x3A : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* 0x3B ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* 0x3C < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* 0x3D = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* 0x3E > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* 0x3F ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* 0x40 @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* 0x41 A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* 0x42 B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* 0x43 C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* 0x44 D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* 0x45 E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* 0x46 F */
    {0x3E,0x41,0x41,0x49,0x7A,0x00}, /* 0x47 G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* 0x48 H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* 0x49 I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* 0x4A J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* 0x4B K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* 0x4C L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* 0x4D M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* 0x4E N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* 0x4F O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* 0x50 P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* 0x51 Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* 0x52 R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* 0x53 S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* 0x54 T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* 0x55 U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* 0x56 V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* 0x57 W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* 0x58 X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* 0x59 Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* 0x5A Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* 0x5B [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* 0x5C \ */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* 0x5D ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* 0x5E ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* 0x5F _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* 0x60 ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* 0x61 a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* 0x62 b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* 0x63 c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* 0x64 d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* 0x65 e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* 0x66 f */
    {0x08,0x54,0x54,0x54,0x3C,0x00}, /* 0x67 g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* 0x68 h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* 0x69 i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* 0x6A j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* 0x6B k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* 0x6C l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* 0x6D m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* 0x6E n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* 0x6F o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* 0x70 p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* 0x71 q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* 0x72 r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* 0x73 s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* 0x74 t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* 0x75 u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* 0x76 v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* 0x77 w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* 0x78 x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* 0x79 y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* 0x7A z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* 0x7B { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* 0x7C | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* 0x7D } */
    {0x08,0x04,0x08,0x10,0x08,0x00}, /* 0x7E ~ */
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 0x7F DEL（占位）*/
};

/* ============================================================
 * 内部状态
 * ============================================================ */
static MenuCtx_t s_ctx;

/* ============================================================
 * GUI_DrawChar  - 绘制单个 ASCII 字符（带缩放）
 *
 * 性能优化说明：
 *   旧实现对每个字符的每个放大像素块单独调用 LCD_FillRect，
 *   每次都会触发一次 CASET/RASET/RAMWR（SetWindow），
 *   对于 12×16 的字符相当于 48 次 SetWindow + SPI 传输，
 *   导致刷新极慢。
 *
 *   现改为：先在本地缓冲区中拼好整个字符的像素数据，
 *   再用一次 LCD_SetWindow + 连续 SPI 突发传输完成，
 *   SetWindow 调用次数从 48 次降为 1 次。
 * ============================================================ */
void GUI_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg)
{
    static uint16_t char_buf[CHAR_W * CHAR_H];
    uint8_t idx;
    uint8_t col, row;

    if ((uint8_t)c < 0x20U || (uint8_t)c > 0x7FU)
        c = '?';

    if ((x + CHAR_W) > GUI_SCREEN_W || (y + CHAR_H) > GUI_SCREEN_H)
        return;

    idx = (uint8_t)c - 0x20U;

    for (col = 0; col < FONT_W; col++)
    {
        uint8_t col_data = s_font6x8[idx][col];
        for (row = 0; row < FONT_H; row++)
        {
            uint16_t color = (col_data & (1U << row)) ? fg : bg;

#if FONT_SCALE == 1
            char_buf[row * CHAR_W + col] = color;
#else
            uint8_t sx, sy;
            for (sy = 0; sy < FONT_SCALE; sy++)
            {
                uint16_t base = (uint16_t)((row * FONT_SCALE + sy) * CHAR_W
                                            + col * FONT_SCALE);
                for (sx = 0; sx < FONT_SCALE; sx++)
                    char_buf[base + sx] = color;
            }
#endif
        }
    }

    /* 一次性写入整个字符区域 */
    LCD_DrawImage(x, y, CHAR_W, CHAR_H, char_buf);
}

/* ============================================================
 * GUI_DrawString - 绘制 ASCII 字符串
 * ============================================================ */
void GUI_DrawString(uint16_t x, uint16_t y,
                    const char *str,
                    uint16_t fg, uint16_t bg)
{
    while (*str)
    {
        if (x + CHAR_W > GUI_SCREEN_W) break;
        GUI_DrawChar(x, y, *str, fg, bg);
        x    += (uint16_t)CHAR_W;
        str++;
    }
}

/* ============================================================
 * 内部：绘制标题栏
 * ============================================================ */
static void draw_title(const char *title)
{
    LCD_FillRect(0, 0, GUI_SCREEN_W, MENU_TITLE_H, GUI_COL_TITLE_BG);
    /* 底部绿色分隔线 */
    LCD_FillRect(0, MENU_TITLE_H - 2, GUI_SCREEN_W, 2, GUI_COL_INDICATOR);

    uint16_t ty = (uint16_t)((MENU_TITLE_H - 2 - CHAR_H) / 2);
    GUI_DrawString(10, ty, title, GUI_COL_TITLE_FG, GUI_COL_TITLE_BG);
}

/* ============================================================
 * 公开接口：供自定义页面绘制风格统一的标题栏
 * ============================================================ */
void GUI_Page_DrawTitle(const char *title)
{
    draw_title(title);
}

/* ============================================================
 * 内部：绘制单个条目
 * ============================================================ */
static void draw_item(uint8_t row, const char *label,
                      uint8_t selected, uint8_t has_child)
{
    uint16_t item_y = (uint16_t)(MENU_TITLE_H + (uint16_t)row * MENU_ITEM_H);
    uint16_t bg     = selected ? GUI_COL_SEL_BG  : GUI_COL_ITEM_BG;
    uint16_t fg     = selected ? GUI_COL_SEL_FG  : GUI_COL_ITEM_FG;

    /* 条目背景 */
    LCD_FillRect(0, item_y, GUI_SCREEN_W, MENU_ITEM_H, bg);

    /* 左侧选中指示条 */
    if (selected)
        LCD_FillRect(0, item_y, MENU_INDICATOR_W, MENU_ITEM_H, GUI_COL_INDICATOR);

    /* 文字 */
    uint16_t ty = (uint16_t)(item_y + MENU_TEXT_Y_OFF);
    GUI_DrawString(MENU_TEXT_X, ty, label, fg, bg);

    /* 子菜单箭头 ">" */
    if (has_child)
        GUI_DrawChar((uint16_t)MENU_ARROW_X, ty, '>', GUI_COL_ARROW, bg);

    /* 条目底部分隔线 */
    LCD_FillRect(0, (uint16_t)(item_y + MENU_ITEM_H - 1),
                 GUI_SCREEN_W, 1, GUI_COL_DIVIDER);
}

/* ============================================================
 * 内部：绘制右侧滚动条
 * ============================================================ */
static void draw_scrollbar(uint8_t total, uint8_t scroll_off, uint8_t visible)
{
    if (total <= visible) return;

    uint16_t area_h = (uint16_t)(GUI_SCREEN_H - MENU_TITLE_H);
    uint16_t bar_h  = (uint16_t)((uint32_t)area_h * visible / total);
    uint16_t bar_y  = (uint16_t)(MENU_TITLE_H +
                       (uint32_t)area_h * scroll_off / total);
    uint16_t bar_x  = (uint16_t)(GUI_SCREEN_W - 4);

    LCD_FillRect(bar_x, MENU_TITLE_H, 4, area_h, GUI_COL_SCROLLBAR);
    LCD_FillRect(bar_x, bar_y, 4, bar_h, GUI_COL_INDICATOR);
}

/* ============================================================
 * GUI_Menu_Redraw
 * ============================================================ */
void GUI_Menu_Redraw(void)
{
    uint8_t     d      = s_ctx.depth;
    MenuItem_t *items  = s_ctx.stack[d];
    uint8_t     total  = s_ctx.count[d];
    uint8_t     cursor = s_ctx.cursor[d];
    uint8_t     scroll = s_ctx.scroll[d];

    /* 全屏清黑 */
    LCD_FillScreen(GUI_COL_BG);

    /* 标题 */
    if (d == 0)
        draw_title("Main Menu");
    else
        draw_title(s_ctx.stack[d - 1][s_ctx.cursor[d - 1]].label);

    /* 条目 */
    uint8_t visible = (total < MENU_ITEM_MAX) ? total : MENU_ITEM_MAX;
    for (uint8_t i = 0; i < visible; i++)
    {
        uint8_t idx = scroll + i;
        if (idx >= total) break;
        draw_item(i,
                  items[idx].label,
                  (idx == cursor),
                  (items[idx].children != NULL && items[idx].child_count > 0));
    }

    /* 空白区填充 */
    if (visible < MENU_ITEM_MAX)
    {
        uint16_t ey = (uint16_t)(MENU_TITLE_H + (uint16_t)visible * MENU_ITEM_H);
        uint16_t eh = (uint16_t)(GUI_SCREEN_H - ey);
        if (eh > 0)
            LCD_FillRect(0, ey, GUI_SCREEN_W, eh, GUI_COL_BG);
    }

    /* 滚动条 */
    draw_scrollbar(total, scroll, MENU_ITEM_MAX);

    s_ctx.need_redraw = 0;
}

/* ============================================================
 * 内部：移动光标
 *
 * 性能优化说明：
 *   旧实现每次旋转都设置 need_redraw=1，
 *   主循环触发 GUI_Menu_Redraw() 做一次全屏清屏+全部条目重绘
 *   （LCD_FillScreen 240×280 像素 + 标题 + 最多5个条目 + 滚动条），
 *   数据量很大，刷新耗时明显。
 *
 *   现改为：
 *     - 若光标移动未导致滚动区间变化（scroll 不变），
 *       仅重绘"旧选中项"和"新选中项"这两行（partial redraw），
 *       不触碰标题栏和其他条目，速度提升数倍。
 *     - 若发生滚动（scroll 改变），数据量较大且涉及条目错位，
 *       仍走全屏重绘（need_redraw=1）。
 * ============================================================ */

/* 重绘单个条目（根据当前 cursor/scroll 重新计算选中状态）*/
static void redraw_single_item(uint8_t d, uint8_t item_idx, uint8_t screen_row)
{
    MenuItem_t *items = s_ctx.stack[d];

    draw_item(screen_row,
              items[item_idx].label,
              (item_idx == s_ctx.cursor[d]),
              (items[item_idx].children != NULL && items[item_idx].child_count > 0));
}

static void menu_move_down(void)
{
    uint8_t d     = s_ctx.depth;
    uint8_t total = s_ctx.count[d];
    if (total == 0) return;

    uint8_t old_cursor = s_ctx.cursor[d];
    uint8_t old_scroll = s_ctx.scroll[d];

    if (s_ctx.cursor[d] < total - 1U)
    {
        s_ctx.cursor[d]++;
        if (s_ctx.cursor[d] >= s_ctx.scroll[d] + MENU_ITEM_MAX)
            s_ctx.scroll[d]++;
    }
    else
    {
        s_ctx.cursor[d] = 0;
        s_ctx.scroll[d] = 0;
    }

    if (s_ctx.scroll[d] == old_scroll)
    {
        /* 滚动未变化：仅重绘旧/新选中行 */
        redraw_single_item(d, old_cursor,    (uint8_t)(old_cursor    - old_scroll));
        redraw_single_item(d, s_ctx.cursor[d], (uint8_t)(s_ctx.cursor[d] - s_ctx.scroll[d]));
        s_ctx.need_redraw = 0;
    }
    else
    {
        /* 发生滚动：整页重绘 */
        s_ctx.need_redraw = 1;
    }
}

static void menu_move_up(void)
{
    uint8_t d     = s_ctx.depth;
    uint8_t total = s_ctx.count[d];
    if (total == 0) return;

    uint8_t old_cursor = s_ctx.cursor[d];
    uint8_t old_scroll = s_ctx.scroll[d];

    if (s_ctx.cursor[d] > 0)
    {
        s_ctx.cursor[d]--;
        if (s_ctx.cursor[d] < s_ctx.scroll[d])
            s_ctx.scroll[d]--;
    }
    else
    {
        s_ctx.cursor[d] = total - 1U;
        s_ctx.scroll[d] = (total > MENU_ITEM_MAX) ? (uint8_t)(total - MENU_ITEM_MAX) : 0;
    }

    if (s_ctx.scroll[d] == old_scroll)
    {
        /* 滚动未变化：仅重绘旧/新选中行 */
        redraw_single_item(d, old_cursor,    (uint8_t)(old_cursor    - old_scroll));
        redraw_single_item(d, s_ctx.cursor[d], (uint8_t)(s_ctx.cursor[d] - s_ctx.scroll[d]));
        s_ctx.need_redraw = 0;
    }
    else
    {
        /* 发生滚动：整页重绘 */
        s_ctx.need_redraw = 1;
    }
}

static void menu_enter(void)
{
    uint8_t     d    = s_ctx.depth;
    MenuItem_t *item = &s_ctx.stack[d][s_ctx.cursor[d]];

    if (item->children != NULL && item->child_count > 0)
    {
        if (s_ctx.depth < MENU_MAX_DEPTH - 1U)
        {
            s_ctx.depth++;
            uint8_t nd        = s_ctx.depth;
            s_ctx.stack[nd]   = item->children;
            s_ctx.count[nd]   = item->child_count;
            s_ctx.cursor[nd]  = 0;
            s_ctx.scroll[nd]  = 0;
            s_ctx.need_redraw = 1;
        }
    }
    else if (item->page != NULL)
    {
        /* 进入自定义页面：接管屏幕，直到长按退出
         *
         * 关键修复：on_enter() 内部通常会做整屏SPI绘制（耗时可达
         * 数十至数百毫秒，尤其在低主频下），这段时间内 EC11_Poll()
         * 完全不会被调用。如果此时按键状态存在任何残留/抖动，
         * 计时会在"暗中"持续累积；等 on_enter() 返回、下一次
         * EC11_Poll 被调用时，时间差可能已超过长按阈值，导致
         * 页面刚进入就立刻收到一次 KEY_LONG。
         *
         * 修复方式：on_enter() 前后都调用 EC11_ResetKeyState()，
         * 以当前实际引脚电平为基准重新开始，不带历史残留。 */
        DEBUG_PRINTF("[GUI] Enter custom page: %s\n", item->label);

        EC11_ResetKeyState();

        s_ctx.active_page = item->page;
        if (s_ctx.active_page->on_enter != NULL)
            s_ctx.active_page->on_enter();

        EC11_ResetKeyState();   /* on_enter 耗时期间可能又产生新的残留，再清一次 */

        s_ctx.need_redraw = 0;   /* 页面自己管理重绘，不走菜单的全屏重绘逻辑 */
    }
    else if (item->action != NULL)
    {
        item->action();
        s_ctx.need_redraw = 1;
    }
}

static void menu_back(void)
{
    if (s_ctx.active_page != NULL)
    {
        /* 当前在自定义页面中：长按退出页面，回到菜单
         * on_exit() 在此调用，由页面自行做收尾（例如停止发送、复位状态）*/
        DEBUG_PRINTF("[GUI] Exit custom page\n");
        if (s_ctx.active_page->on_exit != NULL)
            s_ctx.active_page->on_exit();
        s_ctx.active_page = NULL;
        s_ctx.need_redraw  = 1;   /* 回到菜单，恢复全屏重绘 */

        /* 同样的防护：GUI_Menu_Redraw() 即将进行的全屏SPI绘制耗时期间，
         * EC11_Poll 不会被调用，重置按键状态避免回到菜单后立刻误触发 */
        EC11_ResetKeyState();
        return;
    }

    if (s_ctx.depth > 0)
    {
        s_ctx.depth--;
        s_ctx.need_redraw = 1;
    }
}

/* ============================================================
 * GUI_Menu_Init
 * ============================================================ */
void GUI_Menu_Init(MenuItem_t *root, uint8_t count)
{
    DEBUG_INIT();
    DEBUG_PRINTF("[GUI] Menu Init, root_count=%d\n", count);

    memset(&s_ctx, 0, sizeof(s_ctx));
    EC11_Init();

    s_ctx.stack[0]     = root;
    s_ctx.count[0]     = count;
    s_ctx.active_page  = NULL;
    s_ctx.need_redraw  = 1;

    GUI_Menu_Redraw();
}

/* ============================================================
 * GUI_Menu_Process  —  在 while(1) 中调用
 *
 * 两种模式：
 *   1) 菜单模式（active_page == NULL）：原有逻辑，光标移动/进入/返回
 *   2) 页面模式（active_page != NULL）：
 *      所有事件（含长按）都透传给 on_tick(events)，
 *      由页面自行决定如何响应；
 *      on_tick 返回1时，框架才调用 on_exit() 并退出页面，
 *      返回0则继续停留（页面可借此实现内部多级返回，
 *      例如"调节态长按→回到列表态"，"列表态长按→才真正退出"）
 * ============================================================ */
void GUI_Menu_Process(void)
{
    /* 注意：EC11_Poll() 不再在此调用！
     * 现在由定时器中断（例如 TIM6，2ms周期）独立驱动 EC11_Poll()，
     * 与本函数（含屏幕SPI绘制）完全解耦，互不阻塞。
     * 详见 ec11.h 中的"定时器中断驱动"用法说明。
     * 此处只需要读取已经在中断里采集好的事件即可。 */
    uint32_t ev = EC11_GetEvents();

    if (ev != 0U)
        DEBUG_PRINTF("[GUI] events=0x%02lX depth=%d cursor=%d page=%p\n",
                     ev, s_ctx.depth,
                     s_ctx.cursor[s_ctx.depth],
                     (void *)s_ctx.active_page);

    if (s_ctx.active_page != NULL)
    {
        /* —— 页面模式：全部事件透传，页面通过返回值决定是否退出 —— */
        uint8_t want_exit = 0U;

        if (s_ctx.active_page->on_tick != NULL)
            want_exit = s_ctx.active_page->on_tick(ev);

        if (want_exit)
        {
            DEBUG_PRINTF("[GUI] page requested exit\n");
            menu_back();
        }
        return;   /* 页面模式下不走菜单的need_redraw全屏重绘逻辑 */
    }

    /* —— 菜单模式 —— */
    if (ev & EC11_EVT_CW)        menu_move_down();
    if (ev & EC11_EVT_CCW)       menu_move_up();
    if (ev & EC11_EVT_KEY_SHORT) menu_enter();
    if (ev & EC11_EVT_KEY_LONG)  menu_back();

    if (s_ctx.need_redraw)
        GUI_Menu_Redraw();
}

