/**
 ******************************************************************************
 * @file    ec11.c
 * @brief   EC11 旋转编码器驱动（轮询 + 正交状态机）
 *
 * 正交状态机说明：
 *
 *   EC11 A/B 两相输出格雷码序列：
 *
 *     状态编号  AB电平   顺时针下一状态  逆时针下一状态
 *       0       00            2               1
 *       1       01            0               3
 *       2       10            3               0
 *       3       11            1               2
 *
 *   每次Poll读取AB两位，查转移表得到下一状态及步进方向。
 *   累计步进达到 +4 输出 CW，达到 -4 输出 CCW（走完完整一格）。
 *   中途抖动只会在状态间来回，步进正负抵消，不会错误累积。
 ******************************************************************************
 */

#include "ec11.h"
#include "debug_rtt.h"

EC11_Handle_t hec11 = {0};

/* ============================================================
 * 正交状态转移表
 * 索引：(当前状态 << 2) | (A<<1 | B)
 * 值低4位：新状态
 * 值高4位：步进（0=无效，1=+1步CW，2=-1步CCW）
 *
 * 使用经典 4步/格 模式，完整一格需累计 ±4 步才输出事件
 * ============================================================ */

/* 状态转移 + 步进编码：高nibble=delta(0=无,1=CW,2=CCW)，低nibble=新状态 */
static const uint8_t QSM_TABLE[16] = {
/*  AB=00   AB=01   AB=10   AB=11  （当前状态在行）*/
    0x00,   0x20,   0x10,   0x00,  /* 状态0（AB=00）*/
    0x10,   0x00,   0x00,   0x20,  /* 状态1（AB=01）*/
    0x20,   0x00,   0x00,   0x10,  /* 状态2（AB=10）*/
    0x00,   0x10,   0x20,   0x00,  /* 状态3（AB=11）*/
};

/* 步进累加器，满±4输出一次事件 */
static int8_t s_step_acc = 0;

/* ============================================================
 * EC11_Init
 * ============================================================ */
void EC11_Init(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t b = (HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN) == GPIO_PIN_SET) ? 1U : 0U;

    /* 根据当前电平推算初始状态，避免上电时误触发 */
    hec11.qsm_state = (uint8_t)((a << 1U) | b);

    hec11.count         = 0;
    hec11.events        = EC11_EVT_NONE;
    hec11.key_last      = 1U;  /* 初始高电平（上拉，未按下）*/
    hec11.key_stable    = 1U;
    hec11.key_pressed   = 0U;
    hec11.long_fired    = 0U;
    hec11.key_down_tick = 0U;
    hec11.key_change_tick = 0U;
    s_step_acc          = 0;
}

/* ============================================================
 * EC11_Poll  —  在 while(1) 中持续调用
 * ============================================================ */
void EC11_Poll(void)
{
    uint32_t now = HAL_GetTick();

    /* ----------------------------------------------------------
     * 旋转解码：正交状态机
     * ---------------------------------------------------------- */
    {
        uint8_t a     = (HAL_GPIO_ReadPin(EC11_A_PORT, EC11_A_PIN) == GPIO_PIN_SET) ? 1U : 0U;
        uint8_t b     = (HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN) == GPIO_PIN_SET) ? 1U : 0U;
        uint8_t ab    = (uint8_t)((a << 1U) | b);
        uint8_t entry = QSM_TABLE[(hec11.qsm_state << 2U) | ab];
        uint8_t delta = (entry >> 4U);

        hec11.qsm_state = entry & 0x0FU;

        if (delta == 1U)        /* CW 步进 */
        {
            s_step_acc++;
            if (s_step_acc >= 4)
            {
                s_step_acc = 0;
                hec11.count++;
                hec11.events |= EC11_EVT_CW;
                DEBUG_PRINTF("[EC11] CW fired, A=%d B=%d qsm=%d\n", a, b, hec11.qsm_state);
            }
        }
        else if (delta == 2U)   /* CCW 步进 */
        {
            s_step_acc--;
            if (s_step_acc <= -4)
            {
                s_step_acc = 0;
                hec11.count--;
                hec11.events |= EC11_EVT_CCW;
                DEBUG_PRINTF("[EC11] CCW fired, A=%d B=%d qsm=%d\n", a, b, hec11.qsm_state);
            }
        }
    }

    /* ----------------------------------------------------------
     * 按键解码：消抖状态机
     * ---------------------------------------------------------- */
    {
        uint8_t key_raw = (HAL_GPIO_ReadPin(EC11_KEY_PORT, EC11_KEY_PIN) == GPIO_PIN_SET)
                          ? 1U : 0U;

        /* 检测电平变化，记录变化时刻 */
        if (key_raw != hec11.key_last)
        {
            hec11.key_last       = key_raw;
            hec11.key_change_tick = now;
        }

        /* 消抖：电平稳定超过阈值才确认 */
        if ((now - hec11.key_change_tick) >= EC11_KEY_DEBOUNCE_MS)
        {
            if (key_raw != hec11.key_stable)
            {
                hec11.key_stable = key_raw;

                if (key_raw == 0U)
                {
                    /* 确认按下 */
                    hec11.key_pressed   = 1U;
                    hec11.key_down_tick = now;
                    hec11.long_fired    = 0U;
                    DEBUG_PRINTF("[EC11] KEY pressed confirmed, tick=%lu\n", now);
                }
                else
                {
                    /* 确认释放 */
                    if (hec11.key_pressed == 1U)
                    {
                        hec11.key_pressed = 0U;
                        DEBUG_PRINTF("[EC11] KEY released confirmed, held=%lu ms\n",
                                     now - hec11.key_down_tick);
                        if (hec11.long_fired == 0U)
                            hec11.events |= EC11_EVT_KEY_SHORT;
                    }
                }
            }
        }

        /* 长按检测 */
        if (hec11.key_pressed == 1U && hec11.long_fired == 0U)
        {
            if ((now - hec11.key_down_tick) >= EC11_LONG_PRESS_MS)
            {
                hec11.long_fired  = 1U;
                hec11.events     |= EC11_EVT_KEY_LONG;
                DEBUG_PRINTF("[EC11] LONG fired, key_pressed=%d key_stable=%d raw=%d\n",
                             hec11.key_pressed, hec11.key_stable, key_raw);
            }
        }
    }
}

/* ============================================================
 * EC11_GetEvents  —  读取并清除所有事件
 * ============================================================ */
uint32_t EC11_GetEvents(void)
{
    uint32_t ev   = hec11.events;
    hec11.events  = EC11_EVT_NONE;
    return ev;
}

int32_t EC11_GetCount(void)   { return hec11.count; }
void    EC11_ResetCount(void) { hec11.count = 0; s_step_acc = 0; }

/* ============================================================
 * EC11_ResetKeyState
 *
 * 重新读取当前实际KEY引脚电平作为基准，清空所有按键相关的
 * 中间状态，避免历史残留（尤其是耗时SPI绘制期间累积的计时）
 * 导致状态切换后第一次Poll产生误判。
 * ============================================================ */
void EC11_ResetKeyState(void)
{
    uint32_t now     = HAL_GetTick();
    uint8_t  key_raw = (HAL_GPIO_ReadPin(EC11_KEY_PORT, EC11_KEY_PIN) == GPIO_PIN_SET)
                       ? 1U : 0U;

    hec11.key_last        = key_raw;
    hec11.key_stable       = key_raw;
    hec11.key_change_tick  = now;
    hec11.key_pressed      = 0U;   /* 强制视为未按下，等下一次真实"变化"再重新判断，
                                     * 避免把切换前残留的按下状态带入新状态 */
    hec11.key_down_tick    = now;
    hec11.long_fired        = 0U;
    hec11.events            = EC11_EVT_NONE;

    DEBUG_PRINTF("[EC11] ResetKeyState: raw=%d now=%lu\n", key_raw, now);
}
