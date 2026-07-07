/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : ST7789V + EC11 菜单 + 双路 CAN + Simulation mode
 *                   单片机：STM32F105RCT6 / HAL 库
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "gpio.h"
#include "tim.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "st7789v.h"
#include "ec11.h"
#include "gui_menu.h"
#include "bsp_can.h"
#include "debug_rtt.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* TIM7 中断（20ms周期）置位此标志，主循环检测到后执行一次UI刷新；
 * volatile 保证主循环每次都从内存重新读取，不被编译器优化成寄存器缓存值 */
static volatile uint8_t s_ui_tick_flag = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void can1_rx_callback(CAN_Msg_t *msg);
static void can2_rx_callback(CAN_Msg_t *msg);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ===========================================================
 * CAN 接收回调（中断上下文，保持简短）
 * =========================================================== */
static void can1_rx_callback(CAN_Msg_t *msg) { (void)msg; }
static void can2_rx_callback(CAN_Msg_t *msg) { (void)msg; }

/* ===========================================================
 * Simulation mode 页面
 *
 * 功能：
 *   进入后每 50ms 周期向 CAN1 发送一帧 ID=0x712 的报文，
 *   报文内容由 4 个可调参数打包而成：
 *     speed       0~120   -> byte[1]（独占一整字节，数值范围决定无法压缩进几个bit）
 *     brake       0~1     -> byte[0] 的 bit0
 *     left_turn   0~1     -> byte[2] 的 bit0
 *     right_turn  0~1     -> byte[2] 的 bit2
 *
 *   位置定义全部由下方宏集中控制，如需调整映射，只需修改这些宏。
 *
 * 界面层级：
 *   SIM_STATE_LIST   —— 显示 Speed/Brake/Left_turn/Right_turn 四个选项，
 *                        旋转切换选中项，短按进入该项的数值调节
 *   SIM_STATE_ADJUST —— 显示当前选中项的数值，旋转改变数值，
 *                        长按返回 SIM_STATE_LIST（不退出整个页面）
 *   仅在 SIM_STATE_LIST 状态下长按，才真正退出 Simulation mode 回到主菜单
 * =========================================================== */

/* ---- 712 报文打包位置定义（集中管理，方便调整）---- */
#define SIM712_BYTE_SPEED        1   /* speed      -> byte[1]（独占，0~120）*/
#define SIM712_BYTE_BRAKE        0   /* brake      -> byte[0] */
#define SIM712_BYTE_TURN         2   /* left/right -> byte[2] */
#define SIM712_BIT_BRAKE         0   /* brake      -> byte[0] bit0 */
#define SIM712_BIT_LEFT_TURN     0   /* left_turn  -> byte[2] bit0 */
#define SIM712_BIT_RIGHT_TURN    2   /* right_turn -> byte[2] bit2 */

#define SIM_TX_PERIOD_MS    50U
#define SIM_ID_712          0x712U

#define SIM_SPEED_MIN       0
#define SIM_SPEED_MAX       120
#define SIM_BOOL_MIN        0
#define SIM_BOOL_MAX        1

/* ---- 内部状态机 ---- */
typedef enum
{
    SIM_STATE_LIST = 0,
    SIM_STATE_ADJUST,
} SimState_t;

typedef enum
{
    SIM_PARAM_SPEED = 0,
    SIM_PARAM_BRAKE,
    SIM_PARAM_LEFT_TURN,
    SIM_PARAM_RIGHT_TURN,
    SIM_PARAM_COUNT
} SimParamIdx_t;

typedef struct
{
    const char *label;
    int32_t     min;
    int32_t     max;
    int32_t     value;
} SimParam_t;

static SimParam_t s_sim_params[SIM_PARAM_COUNT] = {
    { "Speed",      SIM_SPEED_MIN, SIM_SPEED_MAX, 0 },
    { "Brake",      SIM_BOOL_MIN,  SIM_BOOL_MAX,  0 },
    { "Left_turn",  SIM_BOOL_MIN,  SIM_BOOL_MAX,  0 },
    { "Right_turn", SIM_BOOL_MIN,  SIM_BOOL_MAX,  0 },
};

static SimState_t s_sim_state      = SIM_STATE_LIST;
static uint8_t    s_sim_cursor     = 0U;
static uint32_t   s_sim_last_tx    = 0U;
static uint32_t   s_sim_tx_count   = 0U;

/* ---- 布局参数 ---- */
#define SIM_STATUS_BAR_H    22                          /* CAN总线状态行高度 */
#define SIM_STATUS_BAR_Y    MENU_TITLE_H                /* 状态行紧贴标题栏下方 */
#define SIM_LIST_ITEM_H     36
#define SIM_LIST_TOP_Y      (MENU_TITLE_H + SIM_STATUS_BAR_H)
#define SIM_FOOTER_Y        (GUI_SCREEN_H - CHAR_H - 8)

/* CAN状态颜色：正常=绿，警告=黄，严重错误(Bus-Off/Passive)=红 */
#define COL_STATUS_OK       COLOR_GREEN
#define COL_STATUS_WARN     COLOR_YELLOW
#define COL_STATUS_ERROR    COLOR_RED

/* 根据当前4个参数值打包成712报文数据 */
static void sim_build_712_data(uint8_t *data)
{
    memset(data, 0, 8);

    data[SIM712_BYTE_SPEED] = (uint8_t)s_sim_params[SIM_PARAM_SPEED].value;

    if (s_sim_params[SIM_PARAM_BRAKE].value)
        data[SIM712_BYTE_BRAKE] |= (uint8_t)(1U << SIM712_BIT_BRAKE);
    if (s_sim_params[SIM_PARAM_LEFT_TURN].value)
        data[SIM712_BYTE_TURN] |= (uint8_t)(1U << SIM712_BIT_LEFT_TURN);
    if (s_sim_params[SIM_PARAM_RIGHT_TURN].value)
        data[SIM712_BYTE_TURN] |= (uint8_t)(1U << SIM712_BIT_RIGHT_TURN);
}

/* —— 绘制：列表态下单个条目（局部刷新用）—— */
static void sim_draw_list_item(uint8_t idx)
{
    uint16_t y    = (uint16_t)(SIM_LIST_TOP_Y + idx * SIM_LIST_ITEM_H);
    uint8_t  sel  = (idx == s_sim_cursor);
    uint16_t bg   = sel ? GUI_COL_SEL_BG : GUI_COL_ITEM_BG;
    uint16_t fg   = sel ? GUI_COL_SEL_FG : GUI_COL_ITEM_FG;
    char     line[32];

    LCD_FillRect(0, y, GUI_SCREEN_W, SIM_LIST_ITEM_H, bg);

    if (sel)
        LCD_FillRect(0, y, MENU_INDICATOR_W, SIM_LIST_ITEM_H, GUI_COL_INDICATOR);

    snprintf(line, sizeof(line), "%-10s %ld",
            s_sim_params[idx].label, (long)s_sim_params[idx].value);
    GUI_DrawString(MENU_TEXT_X, (uint16_t)(y + (SIM_LIST_ITEM_H - CHAR_H) / 2),
                  line, fg, bg);

    LCD_FillRect(0, (uint16_t)(y + SIM_LIST_ITEM_H - 1), GUI_SCREEN_W, 1,
                GUI_COL_DIVIDER);
}

/* —— 绘制：CAN总线状态行（标题栏正下方）—— */
static CAN_BusStatus_t s_last_shown_status = {0};
static uint8_t         s_status_first_draw = 1U;
static uint32_t        s_last_tx_ok        = 0U;
static uint8_t         s_bus_ok_flag       = 0U;   /* 1=总线实际正常（以tx成功为准）*/

static void sim_draw_can_status(void)
{
    CAN_BusStatus_t st;
    char line1[20];
    char line2[20];
    uint16_t color;

    BSP_CAN_GetStatus(CAN_CH_1, &st);

    /* 恢复检测：以"发送成功次数是否在增加"为准，而不是靠LEC
     * （LEC在错误后不自动清零，靠它判断会导致恢复后仍显示黄色）*/
    uint8_t tx_ok_increasing = (st.tx_ok_count > s_last_tx_ok);
    if (tx_ok_increasing)
        s_bus_ok_flag = 1U;   /* 一旦确认发送成功，置位"总线正常"标记 */
    if (st.bus_off || (st.error_passive && !tx_ok_increasing))
        s_bus_ok_flag = 0U;   /* Bus-Off或持续发送失败，清除"正常"标记 */

    /* 颜色判断：优先以bus_ok_flag（发送成功）为准 */
    if (st.bus_off)
        color = COL_STATUS_ERROR;
    else if (st.error_passive && !s_bus_ok_flag)
        color = COL_STATUS_ERROR;
    else if (s_bus_ok_flag)
        color = COL_STATUS_OK;        /* ← 恢复后变绿 */
    else if (st.error_warning || st.lec != CAN_LEC_NONE)
        color = COL_STATUS_WARN;
    else
        color = COL_STATUS_OK;

    /* 变化检测：把tx_ok_increasing和bus_ok_flag都纳入判断
     * 避免"状态寄存器没变但颜色应该变"的漏刷情况 */
    uint8_t changed = (uint8_t)(s_status_first_draw ||
                       st.bus_off       != s_last_shown_status.bus_off ||
                       st.error_passive != s_last_shown_status.error_passive ||
                       st.error_warning != s_last_shown_status.error_warning ||
                       st.lec           != s_last_shown_status.lec ||
                       tx_ok_increasing);   /* ← 关键：tx增加时强制重绘 */

    s_last_tx_ok = st.tx_ok_count;

    if (!changed) return;

    s_last_shown_status = st;
    s_status_first_draw = 0U;

    LCD_FillRect(0, SIM_STATUS_BAR_Y, GUI_SCREEN_W, SIM_STATUS_BAR_H, GUI_COL_BG);

    if (st.bus_off)
    {
        snprintf(line1, sizeof(line1), "BUS-OFF");
        snprintf(line2, sizeof(line2), "fail=%lu", (unsigned long)st.tx_fail_count);
    }
    else if (st.error_passive && !s_bus_ok_flag)
    {
        snprintf(line1, sizeof(line1), "PASSIVE");
        snprintf(line2, sizeof(line2), "T=%d R=%d", st.tec, st.rec);
    }
    else if (s_bus_ok_flag)
    {
        snprintf(line1, sizeof(line1), "CAN1 OK");
        snprintf(line2, sizeof(line2), "tx=%lu", (unsigned long)st.tx_ok_count);
    }
    else if (st.lec != CAN_LEC_NONE)
    {
        snprintf(line1, sizeof(line1), "WARN:%s", BSP_CAN_LecToString(st.lec));
        snprintf(line2, sizeof(line2), "R=%d", st.rec);
    }
    else
    {
        snprintf(line1, sizeof(line1), "CAN1 OK");
        snprintf(line2, sizeof(line2), " ");
    }

    uint16_t y = (uint16_t)(SIM_STATUS_BAR_Y + 2U);
    GUI_DrawString(MENU_TEXT_X, y, line1, color, GUI_COL_BG);
    GUI_DrawString((uint16_t)(MENU_TEXT_X + CHAR_W * 9), y, line2,
                  color, GUI_COL_BG);
}

/* —— 绘制：发送状态行（标题栏下方，列表态/调节态都常驻显示）—— */
static void sim_draw_tx_status(const uint8_t *data)
{
    char line[40];
    uint16_t y = (uint16_t)(SIM_LIST_TOP_Y + SIM_PARAM_COUNT * SIM_LIST_ITEM_H + 4);

    LCD_FillRect(0, y, GUI_SCREEN_W, (uint16_t)(CHAR_H * 2 + 8), GUI_COL_BG);

    snprintf(line, sizeof(line), "TX 0x%03X  cnt=%lu", SIM_ID_712, s_sim_tx_count);
    GUI_DrawString(MENU_TEXT_X, y, line, COLOR_CYAN, GUI_COL_BG);

    snprintf(line, sizeof(line), "D: %02X %02X %02X %02X %02X %02X %02X %02X",
            data[0], data[1], data[2], data[3],
            data[4], data[5], data[6], data[7]);
    GUI_DrawString(MENU_TEXT_X, (uint16_t)(y + CHAR_H + 4), line,
                  COLOR_WHITE, GUI_COL_BG);
}

/* —— 绘制：整个列表态页面（全量，仅在切换回列表态时调用一次）—— */
static void sim_redraw_list(void)
{
    uint8_t data[8];
    sim_build_712_data(data);

    LCD_FillScreen(GUI_COL_BG);
    GUI_Page_DrawTitle("Simulation mode");

    /* CAN状态行：强制下一次 sim_draw_can_status 重绘（全屏清屏后旧状态已失效）*/
    s_status_first_draw = 1U;
    sim_draw_can_status();

    for (uint8_t i = 0; i < SIM_PARAM_COUNT; i++)
        sim_draw_list_item(i);

    sim_draw_tx_status(data);

    GUI_DrawString(MENU_TEXT_X, (uint16_t)SIM_FOOTER_Y,
                  "Long: exit", GUI_COL_ITEM_FG, GUI_COL_BG);
}

/* —— 绘制：调节态页面（全量，仅在进入调节态时调用一次）—— */
static void sim_redraw_adjust(void)
{
    char line[32];
    SimParam_t *p = &s_sim_params[s_sim_cursor];

    LCD_FillScreen(GUI_COL_BG);
    GUI_Page_DrawTitle(p->label);

    /* 调节态同样常驻显示CAN状态行，便于调参过程中也能看到总线情况 */
    s_status_first_draw = 1U;
    sim_draw_can_status();

    GUI_DrawString(MENU_TEXT_X, 90, "Rotate to adjust", GUI_COL_ITEM_FG, GUI_COL_BG);

    snprintf(line, sizeof(line), "Range: %ld ~ %ld", (long)p->min, (long)p->max);
    GUI_DrawString(MENU_TEXT_X, 118, line, GUI_COL_ITEM_FG, GUI_COL_BG);

    GUI_DrawString(MENU_TEXT_X, (uint16_t)SIM_FOOTER_Y,
                  "Long: back to list", GUI_COL_ITEM_FG, GUI_COL_BG);
}

/* —— 绘制：调节态下的数值（局部刷新，旋转时高频调用）—— */
#define SIM_ADJUST_VALUE_Y   150
static void sim_draw_adjust_value(void)
{
    char line[24];
    SimParam_t *p = &s_sim_params[s_sim_cursor];

    LCD_FillRect(0, SIM_ADJUST_VALUE_Y, GUI_SCREEN_W, (uint16_t)(CHAR_H * 2 + 20),
                GUI_COL_BG);

    snprintf(line, sizeof(line), "%ld", (long)p->value);

    /* 数值用2倍大字体（直接复用GUI_DrawChar的缩放机制，这里简单地用
     * 默认字号但加大字间距以示突出，避免引入额外字体表） */
    GUI_DrawString(MENU_TEXT_X, SIM_ADJUST_VALUE_Y, line, COLOR_YELLOW, GUI_COL_BG);
}

static void sim_page_on_enter(void)
{
    DEBUG_PRINTF("[SIM] Enter Simulation mode\n");

    s_sim_state    = SIM_STATE_LIST;
    s_sim_cursor   = 0U;
    s_sim_last_tx  = HAL_GetTick();
    s_sim_tx_count = 0U;

    sim_redraw_list();
}

/**
 * @brief  Simulation mode 主tick：处理事件 + 周期发送712
 * @retval 1=请求退出整个页面，0=继续停留
 */
static uint8_t sim_page_on_tick(uint32_t events)
{
    uint32_t now = HAL_GetTick();

    if (events != 0U)
        DEBUG_PRINTF("[SIM] on_tick received events=0x%02lX state=%d key_pressed=%d "
                     "long_fired=%d key_down_tick=%lu now=%lu\n",
                     events, s_sim_state, hec11.key_pressed, hec11.long_fired,
                     hec11.key_down_tick, now);

    /* —— 接收处理：避免页面停留期间 FIFO 溢出 —— */
    CAN_Msg_t rx_msg;
    while (BSP_CAN_Receive(CAN_CH_1, &rx_msg)) { (void)rx_msg; }
    while (BSP_CAN_Receive(CAN_CH_2, &rx_msg)) { (void)rx_msg; }

    /* —— 事件处理：按当前内部状态分别解释 CW/CCW/SHORT/LONG —— */
    if (s_sim_state == SIM_STATE_LIST)
    {
        if (events & EC11_EVT_CW)
        {
            uint8_t old = s_sim_cursor;
            s_sim_cursor = (uint8_t)((s_sim_cursor + 1U) % SIM_PARAM_COUNT);
            sim_draw_list_item(old);
            sim_draw_list_item(s_sim_cursor);
        }
        if (events & EC11_EVT_CCW)
        {
            uint8_t old = s_sim_cursor;
            s_sim_cursor = (uint8_t)((s_sim_cursor == 0U)
                                     ? (SIM_PARAM_COUNT - 1U)
                                     : (s_sim_cursor - 1U));
            sim_draw_list_item(old);
            sim_draw_list_item(s_sim_cursor);
        }
        if (events & EC11_EVT_KEY_SHORT)
        {
            DEBUG_PRINTF("[SIM] Enter adjust: %s\n", s_sim_params[s_sim_cursor].label);
            s_sim_state = SIM_STATE_ADJUST;

            /* sim_redraw_adjust 是全屏SPI绘制，耗时期间EC11_Poll不会
             * 被调用，重置按键状态避免误判长按 */
            EC11_ResetKeyState();

            sim_redraw_adjust();
            sim_draw_adjust_value();
        }
        if (events & EC11_EVT_KEY_LONG)
        {
            /* 列表态长按：真正退出整个 Simulation mode */
            DEBUG_PRINTF("[SIM] Exit request from LIST state\n");
            return 1U;
        }
    }
    else /* SIM_STATE_ADJUST */
    {
        SimParam_t *p = &s_sim_params[s_sim_cursor];
        uint8_t value_changed = 0U;

        if (events & EC11_EVT_CW)
        {
            if (p->value < p->max) { p->value++; value_changed = 1U; }
        }
        if (events & EC11_EVT_CCW)
        {
            if (p->value > p->min) { p->value--; value_changed = 1U; }
        }
        if (value_changed)
        {
            sim_draw_adjust_value();
            DEBUG_PRINTF("[SIM] %s = %ld\n", p->label, (long)p->value);
        }

        /* 短按在调节态下不做任何事（已经在调节了）*/

        if (events & EC11_EVT_KEY_LONG)
        {
            /* 调节态长按：仅返回列表态，不退出整个页面 */
            DEBUG_PRINTF("[SIM] Back to LIST from adjust\n");
            s_sim_state = SIM_STATE_LIST;

            /* sim_redraw_list 是全屏SPI绘制，同样需要防护 */
            EC11_ResetKeyState();

            sim_redraw_list();
        }
    }

    /* —— 周期发送：50ms 一次，与界面状态无关，列表态/调节态都持续发送 —— */
    if ((now - s_sim_last_tx) >= SIM_TX_PERIOD_MS)
    {
        s_sim_last_tx = now;
        s_sim_tx_count++;

        uint8_t data[8];
        sim_build_712_data(data);

        CAN_Msg_t tx = {
            .id         = SIM_ID_712,
            .frame_type = CAN_FRAME_STD,
            .dlc        = 8U,
        };
        memcpy(tx.data, data, 8);

        BSP_CAN_Send(CAN_CH_1, &tx);

        /* CAN状态行在列表态/调节态都常驻更新（实时反映总线健康度，
         * sim_draw_can_status 内部已做"仅状态变化才重绘"的判断，
         * 不会因为高频调用而产生不必要的SPI开销）*/
        sim_draw_can_status();

        /* 发送状态行只在列表态显示更新（调节态界面不含该区域，
         * 避免在调节态时把数值显示区覆盖掉） */
        if (s_sim_state == SIM_STATE_LIST)
            sim_draw_tx_status(data);
    }

    return 0U;   /* 继续停留在 Simulation mode 内 */
}

static void sim_page_on_exit(void)
{
    DEBUG_PRINTF("[SIM] Exit Simulation mode, total tx=%lu\n", s_sim_tx_count);
}

static CustomPage_t sim_page = {
    .on_enter = sim_page_on_enter,
    .on_tick  = sim_page_on_tick,
    .on_exit  = sim_page_on_exit,
};

/* ===========================================================
 * About 页面（一次性 action）
 * =========================================================== */
static void action_about(void)
{
    LCD_FillScreen(GUI_COL_BG);
    GUI_DrawString(10,  80, "Menu System",   COLOR_WHITE,     GUI_COL_BG);
    GUI_DrawString(10, 110, "Ver  1.2.0",    GUI_COL_ITEM_FG, GUI_COL_BG);
    GUI_DrawString(10, 140, "STM32F105RCT6", GUI_COL_ITEM_FG, GUI_COL_BG);
}

/* ===========================================================
 * 菜单树
 * =========================================================== */
static MenuItem_t menu_root[] = {
    /*  label              children  count  action        page       */
    { "Simulation mode",  NULL,      0,     NULL,          &sim_page  },
    { "About",            NULL,      0,     action_about,  NULL       },
};

#define MENU_ROOT_COUNT  (sizeof(menu_root) / sizeof(menu_root[0]))

/* USER CODE END 0 */

/* ===========================================================
 * main
 * =========================================================== */
int main(void)
{
    HAL_Init();

    DEBUG_INIT();
    DEBUG_PRINTF("\n========== System Boot ==========\n");

    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_SPI1_Init();
    MX_TIM6_Init();    /* EC11采样定时器：2ms周期，高优先级 */
    MX_TIM7_Init();    /* UI刷新节拍定时器：20ms周期，低优先级 */

    DEBUG_PRINTF("[MAIN] Peripheral init done\n");

    /* USER CODE BEGIN 2 */

    LCD_Init();
    GUI_Menu_Init(menu_root, (uint8_t)MENU_ROOT_COUNT);

    BSP_CAN_Init(CAN_CH_1, CAN_BAUD_500K, can1_rx_callback);
    BSP_CAN_Init(CAN_CH_2, CAN_BAUD_500K, can2_rx_callback);

    /* —— 启动两个定时器中断 ——
     * TIM6: EC11_Poll() 周期采样，与SPI绘制完全解耦，
     *       无论屏幕刷新多慢，EC11都能精确按2ms采样，不丢事件
     * TIM7: UI刷新节拍标志，主循环根据这个标志决定何时调用
     *       GUI_Menu_Process()，避免主循环空转占满CPU（虽然对功能
     *       无影响，但限制刷新节奏更利于功耗和CAN收发处理的及时性）*/
    DEBUG_PRINTF("[MAIN] Starting TIM6 (EC11 2ms) and TIM7 (UI 20ms)...\n");

    {
        HAL_StatusTypeDef tim6_ret = HAL_TIM_Base_Start_IT(&htim6);
        HAL_StatusTypeDef tim7_ret = HAL_TIM_Base_Start_IT(&htim7);

        DEBUG_PRINTF("[DIAG] HAL_TIM_Base_Start_IT(TIM6)=%d  (TIM7)=%d  "
                    "(0=HAL_OK, 非0表示启动失败)\n", tim6_ret, tim7_ret);

        /* 直接读取定时器寄存器实际配置值，确认CubeMX生成的参数
         * 是否真的是按 PSC=71/ARR=1999(TIM6) PSC=71/ARR=19999(TIM7) 配置的，
         * 而不是凭日志推测 */
        DEBUG_PRINTF("[DIAG] TIM6: PSC=%lu ARR=%lu CR1=0x%04X "
                    "(CEN bit0 应为1表示已使能计数)\n",
                    htim6.Instance->PSC, htim6.Instance->ARR,
                    (unsigned int)htim6.Instance->CR1);
        DEBUG_PRINTF("[DIAG] TIM7: PSC=%lu ARR=%lu CR1=0x%04X\n",
                    htim7.Instance->PSC, htim7.Instance->ARR,
                    (unsigned int)htim7.Instance->CR1);

        /* 确认TIM6/TIM7挂载的APB1时钟实际频率，
         * 用于核对 PSC/ARR 参数是否与实际时钟匹配 */
        DEBUG_PRINTF("[DIAG] HAL_RCC_GetPCLK1Freq()=%lu HAL_RCC_GetHCLKFreq()=%lu\n",
                    HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetHCLKFreq());

        /* 计算理论中断周期，与实测心跳对比 */
        {
            uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
            /* APB1预分频不为1时，定时器时钟 = APB1×2 */
            uint32_t tim_clk = (pclk1 == HAL_RCC_GetHCLKFreq()) ? pclk1 : (pclk1 * 2U);
            uint32_t period_us = (uint32_t)(((uint64_t)(htim6.Instance->PSC + 1U) *
                                             (htim6.Instance->ARR + 1U) * 1000000U) / tim_clk);
            DEBUG_PRINTF("[DIAG] TIM6 计算时钟源=%lu Hz, 理论中断周期=%lu us "
                        "(期望2000us)\n", tim_clk, period_us);
        }
    }

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */

    /* —— 诊断用：每1秒打印一次TIM6心跳增量 ——
     * 正常应显示约500（2ms周期 × 500次 = 1000ms）。
     * 如果显示值远小于500，说明TIM6实际触发频率被拖慢，
     * 是"EC11大部分时间没反应"的直接证据来源。
     * 此打印在主循环里执行（非中断上下文），不会拖慢TIM6本身。
     * 确认问题解决后，可以删除这段诊断代码。 */
    uint32_t hb_last_count = 0U;
    uint32_t hb_last_tick  = HAL_GetTick();

    while (1)
    {
        /* —— UI刷新由 TIM7 节拍驱动 ——
         * s_ui_tick_flag 由 TIM7 中断每20ms置1，
         * 主循环检测到后才执行一次 GUI_Menu_Process()
         * （内部含 EC11 事件读取 + 必要时的 SPI 绘制）。
         * EC11 的实时采样完全独立于这个节拍，由 TIM6 单独驱动，
         * 不受 UI 刷新间隔影响——这就是"两者互不干扰"的核心。 */
        if (s_ui_tick_flag)
        {
            s_ui_tick_flag = 0U;
            GUI_Menu_Process();
        }

        CAN_Msg_t rx_msg;
        while (BSP_CAN_Receive(CAN_CH_1, &rx_msg)) { (void)rx_msg; }
        while (BSP_CAN_Receive(CAN_CH_2, &rx_msg)) { (void)rx_msg; }

        /* TIM6心跳诊断（1秒一次，频率很低，不影响性能）*/
        {
            uint32_t now = HAL_GetTick();
            if ((now - hb_last_tick) >= 1000U)
            {
                uint32_t hb_now   = EC11_GetHeartbeat();
                uint32_t hb_delta = hb_now - hb_last_count;
                DEBUG_PRINTF("[DIAG] TIM6 heartbeat delta=%lu in %lums "
                            "(expect ~500 for 2ms period; low value = ISR starved)\n",
                            hb_delta, now - hb_last_tick);

                /* 额外诊断：TIM6计数器当前值（确认计数器在真实递增，
                 * 而非卡死在0或某个固定值——如果CNT在两次读取间没有
                 * 变化，说明计数器本身没有真正运行，问题在时钟使能
                 * 或预分频配置，而不是中断响应速度问题）*/
                DEBUG_PRINTF("[DIAG] TIM6 CNT=%lu  qsm_state=%d  "
                            "A=%d B=%d KEY=%d (raw pin levels)\n",
                            htim6.Instance->CNT, hec11.qsm_state,
                            HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN),
                            HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN),
                            HAL_GPIO_ReadPin(EC11_KEY_PORT, EC11_KEY_PIN));

                hb_last_count = hb_now;
                hb_last_tick  = now;
            }
        }
    }
    /* USER CODE END WHILE */
}

/* ===========================================================
 * SystemClock_Config
 * =========================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    DEBUG_PRINTF("[CLK] SystemClock_Config begin\n");

    /* —— 外部HSE晶振已确认正常，使用 HSE(8MHz) x PLL(9) = 72MHz —— */
    DEBUG_PRINTF("[CLK] Using external HSE 8MHz + PLLx9 = 72MHz\n");

    RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;   /* 8MHz直接进PLL，不预分频 */
    RCC_OscInitStruct.HSIState        = RCC_HSI_ON;            /* HSI保留作为备用时钟源 */
    RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL9;          /* 8MHz x 9 = 72MHz */

    DEBUG_PRINTF("[CLK] Calling HAL_RCC_OscConfig (HSE+PLL)...\n");
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        DEBUG_PRINTF("[CLK] HAL_RCC_OscConfig FAILED! Check HSE crystal.\n");
        Error_Handler();
    }
    DEBUG_PRINTF("[CLK] HAL_RCC_OscConfig OK, HSE+PLL locked at 72MHz\n");

    /* SYSCLK/HCLK = 72MHz（AHB总线，不分频，CPU/Flash/DMA都跑满速）
     * APB1 硬性上限 36MHz（STM32F1系列规格），必须二分频：72/2=36MHz
     *   -> CAN1/CAN2/SPI2/USART2/I2C 等APB1外设都受此限制
     * APB2 上限 72MHz，不分频：72MHz
     *   -> SPI1/USART1/GPIO/ADC等APB2外设跑满速 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 72MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     /* PCLK1 = 36MHz（APB1上限）*/
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* PCLK2 = 72MHz（APB2跑满速）*/

    /* Flash等待周期：72MHz > 48MHz阈值，必须用 FLASH_LATENCY_2（2个等待周期）
     * 否则CPU读取Flash指令会出错，是72MHz主频下最容易遗漏的一步 */
    DEBUG_PRINTF("[CLK] Calling HAL_RCC_ClockConfig (72MHz, Flash Latency=2)...\n");
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        DEBUG_PRINTF("[CLK] HAL_RCC_ClockConfig FAILED!\n");
        Error_Handler();
    }

    DEBUG_PRINTF("[CLK] Done. SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu\n",
                 HAL_RCC_GetSysClockFreq(), HAL_RCC_GetHCLKFreq(),
                 HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq());
}

/* USER CODE BEGIN 4 */

/* ===========================================================
 * 定时器中断回调
 *
 * TIM6（2ms周期，建议NVIC优先级设高一些，例如0或1）：
 *   只调用 EC11_Poll()，内部仅做GPIO读取+状态机判断，
 *   耗时在微秒级，不会因为屏幕SPI绘制（在主循环里，可能耗时
 *   数十毫秒）而被延误——两者运行在完全独立的执行上下文，
 *   物理上互不阻塞，这正是题目要求的"放在两个定时器中相互不干扰"。
 *
 * TIM7（20ms周期，优先级可以低于TIM6，例如2或3）：
 *   只置一个标志位，真正的UI绘制工作仍在主循环里执行
 *   （SPI传输不适合放在中断上下文里做，HAL_SPI_Transmit
 *   内部某些等待逻辑、以及传输耗时本身，都不应该阻塞其他中断）。
 *   这个定时器的作用只是把"该刷新了"这件事从忙等改成事件驱动，
 *   避免主循环持续空转。
 * ===========================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        EC11_Poll();   /* 仅此一行，快速返回，不做任何阻塞操作 */
    }
    else if (htim->Instance == TIM7)
    {
        s_ui_tick_flag = 1U;   /* 仅置标志，真正绘制工作留给主循环 */
    }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
