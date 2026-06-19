#ifndef __EC11_H
#define __EC11_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file    ec11.h
 * @brief   EC11 旋转编码器驱动（HAL库，轮询+正交状态机，无需EXTI中断）
 *
 * 硬件连接：
 *   A 相 -> PC1  (GPIO 普通输入，上拉)
 *   B 相 -> PC2  (GPIO 普通输入，上拉)
 *   KEY  -> PC0  (GPIO 普通输入，上拉)
 *
 * CubeMX 配置（全部改为普通输入，不需要任何EXTI）：
 *   PC0: GPIO_Input, Pull-up, 标签 EC11_KEY
 *   PC1: GPIO_Input, Pull-up, 标签 EC11_A
 *   PC2: GPIO_Input, Pull-up, 标签 EC11_B
 *   NVIC: 不需要使能任何 EXTI
 *
 * 使用方法：
 *   在 while(1) 中以固定间隔（建议 1~5ms）调用 EC11_Poll()，
 *   然后通过 EC11_GetEvents() 取事件即可。
 *   不需要在 HAL_GPIO_EXTI_Callback 中做任何处理。
 *
 * 抗抖动原理：
 *   采用 4步正交状态机（Ben Buxton 算法）。
 *   A/B两相的合法正交序列为：
 *     CW:  00→01→11→10→00  (或反向)
 *   每次Poll读取(A,B)电平更新状态，只有完整走完
 *   一个合法4步序列才输出一次CW/CCW事件。
 *   机械抖动导致的中间态来回摆动不会积累成有效输出，
 *   无需额外时间窗口去抖。
 ******************************************************************************
 */

/* ============================================================
 * 引脚定义
 * ============================================================ */
#define EC11_A_PORT         GPIOC
#define EC11_A_PIN          GPIO_PIN_1

#define EC11_B_PORT         GPIOC
#define EC11_B_PIN          GPIO_PIN_2

#define EC11_KEY_PORT       GPIOC
#define EC11_KEY_PIN        GPIO_PIN_0

/* ============================================================
 * 按键长按阈值（ms）
 * ============================================================ */
#define EC11_LONG_PRESS_MS  800U

/* ============================================================
 * 按键消抖时间（ms）
 * ============================================================ */
#define EC11_KEY_DEBOUNCE_MS 20U

/* ============================================================
 * 事件标志
 * ============================================================ */
typedef enum
{
    EC11_EVT_NONE        = 0x00U,
    EC11_EVT_CW          = 0x01U,   /* 顺时针 */
    EC11_EVT_CCW         = 0x02U,   /* 逆时针 */
    EC11_EVT_KEY_SHORT   = 0x04U,   /* 短按 */
    EC11_EVT_KEY_LONG    = 0x08U,   /* 长按 */
} EC11_Event_t;

/* ============================================================
 * 句柄
 * ============================================================ */
typedef struct
{
    /* 正交状态机 */
    uint8_t          qsm_state;     /* 当前状态（0~3）*/

    /* 累计计数 */
    volatile int32_t count;

    /* 事件标志 */
    volatile uint32_t events;

    /* 按键状态 */
    uint8_t  key_last;              /* 上次读到的KEY电平 */
    uint8_t  key_stable;            /* 消抖后稳定电平 */
    uint32_t key_change_tick;       /* 电平变化时刻 */
    uint8_t  key_pressed;           /* 当前是否处于按下状态 */
    uint32_t key_down_tick;         /* 按下时刻 */
    uint8_t  long_fired;            /* 长按已触发标记 */
} EC11_Handle_t;

extern EC11_Handle_t hec11;

/* ============================================================
 * API
 * ============================================================ */
void     EC11_Init(void);

/**
 * @brief  轮询函数，放在 while(1) 中持续调用
 *         建议调用间隔：1~5ms（可借助 SysTick 或 TIM 周期调用）
 *         若调用间隔不固定，只要远小于旋转一格时间（通常>20ms）即可
 */
void     EC11_Poll(void);

uint32_t EC11_GetEvents(void);   /* 读取并清除所有事件 */
int32_t  EC11_GetCount(void);
void     EC11_ResetCount(void);

/**
 * @brief  重置按键相关的全部内部状态（key_pressed/long_fired/
 *         key_last/key_stable/各tick/events）。
 *
 *         用于状态切换点（例如进入/退出自定义页面前后），防止
 *         耗时的SPI全屏绘制期间 EC11_Poll 未被调用，导致按键的
 *         按下计时在"暗中"持续累积，绘制结束后第一次Poll就
 *         误判为长按，或残留的中间状态导致下一次按下/释放判断错误。
 *
 *         调用后，下一次 EC11_Poll() 会以当前实际引脚电平为基准
 *         重新开始判断，不带任何历史残留。
 */
void     EC11_ResetKeyState(void);

#ifdef __cplusplus
}
#endif
#endif /* __EC11_H */
