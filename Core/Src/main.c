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
    { "Speed",      SIM_SPEED_MIN, SIM_SPEED_MAX, 6 },
    { "Brake",      SIM_BOOL_MIN,  SIM_BOOL_MAX,  0 },
    { "Left_turn",  SIM_BOOL_MIN,  SIM_BOOL_MAX,  0 },
    { "Right_turn", SIM_BOOL_MIN,  SIM_BOOL_MAX,  0 },
};

static SimState_t s_sim_state      = SIM_STATE_LIST;
static uint8_t    s_sim_cursor     = 0U;
static uint32_t   s_sim_last_tx    = 0U;
static uint32_t   s_sim_tx_count   = 0U;

/* ---- 布局参数 ---- */
#define SIM_LIST_ITEM_H     40
#define SIM_LIST_TOP_Y      (MENU_TITLE_H + 4)
#define SIM_FOOTER_Y        (GUI_SCREEN_H - CHAR_H - 10)

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

/* —— 绘制：发送状态行（标题栏下方，列表态/调节态都常驻显示）—— */
static void sim_draw_tx_status(const uint8_t *data)
{
    char line[40];
    uint16_t y = MENU_TITLE_H + 4 + 4 * SIM_LIST_ITEM_H + 6;

    LCD_FillRect(0, y, GUI_SCREEN_W, (uint16_t)(CHAR_H * 2 + 12), GUI_COL_BG);

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

    GUI_DrawString(MENU_TEXT_X, 70, "Rotate to adjust", GUI_COL_ITEM_FG, GUI_COL_BG);

    snprintf(line, sizeof(line), "Range: %ld ~ %ld", (long)p->min, (long)p->max);
    GUI_DrawString(MENU_TEXT_X, 100, line, GUI_COL_ITEM_FG, GUI_COL_BG);

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

    DEBUG_PRINTF("[MAIN] Peripheral init done\n");

    /* USER CODE BEGIN 2 */

    LCD_Init();
    GUI_Menu_Init(menu_root, (uint8_t)MENU_ROOT_COUNT);

    BSP_CAN_Init(CAN_CH_1, CAN_BAUD_500K, can1_rx_callback);
    BSP_CAN_Init(CAN_CH_2, CAN_BAUD_500K, can2_rx_callback);

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        GUI_Menu_Process();

        CAN_Msg_t rx_msg;
        while (BSP_CAN_Receive(CAN_CH_1, &rx_msg)) { (void)rx_msg; }
        while (BSP_CAN_Receive(CAN_CH_2, &rx_msg)) { (void)rx_msg; }
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

    /* —— 实际硬件使用内部HSI，全部时钟跑8MHz，不依赖外部晶振 —— */
    DEBUG_PRINTF("[CLK] Using internal HSI, 8MHz, no PLL, no external crystal\n");

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;   /* 不使用PLL */

    DEBUG_PRINTF("[CLK] Calling HAL_RCC_OscConfig...\n");
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        DEBUG_PRINTF("[CLK] HAL_RCC_OscConfig FAILED!\n");
        Error_Handler();
    }
    DEBUG_PRINTF("[CLK] HAL_RCC_OscConfig OK\n");

    /* HSI(8MHz) 直接作为 SYSCLK，HCLK/PCLK1/PCLK2 全部不分频，均为8MHz
     * Flash等待周期：8MHz远低于24MHz阈值，FLASH_LATENCY_0即可 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 8MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     /* PCLK1 = 8MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* PCLK2 = 8MHz */

    DEBUG_PRINTF("[CLK] Calling HAL_RCC_ClockConfig (HSI direct, 8MHz)...\n");
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        DEBUG_PRINTF("[CLK] HAL_RCC_ClockConfig FAILED!\n");
        Error_Handler();
    }

    DEBUG_PRINTF("[CLK] Done. SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu\n",
                 HAL_RCC_GetSysClockFreq(), HAL_RCC_GetHCLKFreq(),
                 HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq());
}

/* USER CODE BEGIN 4 */
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
