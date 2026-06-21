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

/* 心跳计数：每次EC11_Poll调用都自增一次，用于诊断TIM6实际触发频率。
 * 例如在主循环里每1000ms读取一次差值，理论上应该约等于500（2ms周期）。
 * 如果实测值远小于500，说明TIM6中断被其他更高优先级中断/某种
 * 阻塞操作严重拖慢了实际触发频率。 */
static volatile uint32_t s_poll_heartbeat = 0U;

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
                /* 注意：此处刻意不打印 DEBUG_PRINTF！
                 * EC11_Poll 现在运行在 TIM6 中断（2ms周期）中，
                 * RTT printf 涉及字符串格式化+缓冲区写入，在中断
                 * 上下文里频繁调用会显著拖慢本次中断的执行时间，
                 * 导致下一次 TIM6 中断被推迟，表现为"旋转大部分
                 * 时间没反应"——这正是本次问题的根因。
                 * 如需调试，使用下方的 s_poll_heartbeat 计数器，
                 * 在主循环里低频读取打印，不占用中断时间。 */
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
                /* 同上，不在中断里打印 */
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
                    /* 不在中断里打印，理由同上 */
                }
                else
                {
                    /* 确认释放 */
                    if (hec11.key_pressed == 1U)
                    {
                        hec11.key_pressed = 0U;
                        if (hec11.long_fired == 0U)
                            hec11.events |= EC11_EVT_KEY_SHORT;
                        /* 不在中断里打印，理由同上 */
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
                /* 不在中断里打印，理由同上 */
            }
        }

        /* 心跳计数：每次Poll都自增，不涉及任何耗时操作（仅一条加法指令）。
         * 用于在主循环里低频验证TIM6的实际触发频率是否符合预期(2ms一次)，
         * 调试方法见 EC11_GetHeartbeat() 的注释。 */
        s_poll_heartbeat++;
    }
}

/* ============================================================
 * EC11_GetEvents  —  读取并清除所有事件
 *
 * 临界区保护说明：
 *   EC11_Poll() 现在运行在定时器中断中，而本函数运行在主循环里，
 *   存在"中断写、主循环读"的共享变量竞态：
 *     主循环读取 hec11.events 之后、清零之前，
 *     如果定时器中断恰好在这个窗口触发并写入新事件，
 *     紧接着主循环执行清零，会把这个新事件无声丢弃。
 *   用 __disable_irq/__enable_irq 包裹"读取+清零"，
 *   确保这两步操作不会被中断打断，是原子的。
 *   临界区极短（仅两条赋值语句），不会影响中断响应实时性。
 * ============================================================ */
uint32_t EC11_GetEvents(void)
{
    uint32_t ev;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ev            = hec11.events;
    hec11.events  = EC11_EVT_NONE;
    if (!primask) __enable_irq();

    return ev;
}

int32_t EC11_GetCount(void)
{
    int32_t cnt;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    cnt = hec11.count;
    if (!primask) __enable_irq();

    return cnt;
}

void EC11_ResetCount(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    hec11.count = 0;
    s_step_acc  = 0;
    if (!primask) __enable_irq();
}

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
    uint32_t primask = __get_PRIMASK();

    /* EC11_Poll() 运行在定时器中断中，本函数运行在主循环里，
     * 一次性写入多个字段，必须作为原子操作，否则定时器中断
     * 可能在写到一半时打入，读到不一致的中间状态 */
    __disable_irq();
    hec11.key_last        = key_raw;
    hec11.key_stable       = key_raw;
    hec11.key_change_tick  = now;
    hec11.key_pressed      = 0U;   /* 强制视为未按下，等下一次真实"变化"再重新判断 */
    hec11.key_down_tick    = now;
    hec11.long_fired        = 0U;
    hec11.events            = EC11_EVT_NONE;
    if (!primask) __enable_irq();

    DEBUG_PRINTF("[EC11] ResetKeyState: raw=%d now=%lu\n", key_raw, now);
}

/* ============================================================
 * EC11_GetHeartbeat
 *
 * 调试用：返回 EC11_Poll() 被调用的累计次数。
 * 用法（在主循环里，每隔约1秒读取一次差值）：
 *
 *   static uint32_t last_hb = 0, last_tick = 0;
 *   uint32_t now = HAL_GetTick();
 *   if (now - last_tick >= 1000U) {
 *       uint32_t hb = EC11_GetHeartbeat();
 *       DEBUG_PRINTF("[EC11] heartbeat delta=%lu (expect ~500 for 2ms period)\n",
 *                    hb - last_hb);
 *       last_hb   = hb;
 *       last_tick = now;
 *   }
 *
 * 若实测值远小于500（例如只有几十），说明TIM6中断的实际触发频率
 * 被严重拖慢，需要检查 NVIC 优先级配置，或者是否有其他中断/
 * 代码长时间关闭了全局中断（__disable_irq 持续时间过长）。
 * ============================================================ */
uint32_t EC11_GetHeartbeat(void)
{
    return s_poll_heartbeat;
}
