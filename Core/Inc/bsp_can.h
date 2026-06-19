#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   STM32F105RCT6 双路 CAN 驱动（HAL 库）
 *
 * 硬件连接：
 *   CAN1  TX -> PA12    CAN1  RX -> PA11
 *   CAN2  TX -> PB13    CAN2  RX -> PB12
 *
 * CubeMX 配置要点：
 *   CAN1 / CAN2:
 *     Mode              : Normal
 *     Prescaler         : 根据波特率计算（见下方说明）
 *     Time Quanta BS1   : 建议 9  Tq
 *     Time Quanta BS2   : 建议 4  Tq
 *     ReSynchronization : 1  Tq
 *     → 72MHz / Prescaler / (1+BS1+BS2) = 波特率
 *       500kbps: Prescaler=8  → 72M/8/9+4+1=500k  ✓（实际：72/8/(1+9+4)=643k，需调整）
 *       推荐：BS1=6, BS2=1, SJW=1, Prescaler=9 → 72M/9/8=1M
 *             BS1=9, BS2=4, SJW=1, Prescaler=6 → 72M/6/14≈857k
 *             BS1=9, BS2=4, SJW=1, Prescaler=9 → 72M/9/14=571k
 *       500kbps最佳：BS1=9, BS2=4, SJW=1, Prescaler=8 → 72/8/14=643k(误差)
 *                    BS1=7, BS2=2, SJW=1, Prescaler=9 → 72/9/10=800k(误差)
 *       推荐直接用：BS1=9, BS2=4, SJW=1, Prescaler=9 → 约572kbps
 *                  或 BS1=13, BS2=2, SJW=1, Prescaler=6 → 500kbps 精确
 *
 *   NVIC:
 *     CAN1 RX0 interrupt (CAN1_RX0_IRQn) → Enable，Priority 2
 *     CAN2 RX0 interrupt (CAN2_RX0_IRQn) → Enable，Priority 2
 *
 *   注意：STM32F105 使用互联型，CAN2 依赖 CAN1 时钟，
 *         即使只用 CAN2，也必须使能 CAN1 时钟。
 ******************************************************************************
 */

/* ============================================================
 * 波特率预设（72MHz APB1，常用配置）
 * Baudrate = Fpclk1 / (Prescaler * (1 + BS1 + BS2))
 * ============================================================ */
typedef enum
{
    CAN_BAUD_1M    = 0U,   /* Prescaler=4,  BS1=13, BS2=2, SJW=1 → 1Mbps   */
    CAN_BAUD_500K  = 1U,   /* Prescaler=8,  BS1=13, BS2=2, SJW=1 → 500kbps */
    CAN_BAUD_250K  = 2U,   /* Prescaler=16, BS1=13, BS2=2, SJW=1 → 250kbps */
    CAN_BAUD_125K  = 3U,   /* Prescaler=32, BS1=13, BS2=2, SJW=1 → 125kbps */
} CAN_BaudRate_t;

/* ============================================================
 * 帧类型
 * ============================================================ */
typedef enum
{
    CAN_FRAME_STD = 0U,    /* 标准帧（11位ID）*/
    CAN_FRAME_EXT = 1U,    /* 扩展帧（29位ID）*/
} CAN_FrameType_t;

/* ============================================================
 * CAN 报文结构体
 * ============================================================ */
typedef struct
{
    uint32_t        id;             /* 帧ID（标准帧11位 / 扩展帧29位）*/
    CAN_FrameType_t frame_type;     /* 帧类型 */
    uint8_t         dlc;            /* 数据长度 0~8 */
    uint8_t         data[8];        /* 数据 */
} CAN_Msg_t;

/* ============================================================
 * 接收回调函数类型
 * ============================================================ */
typedef void (*CAN_RxCallback_t)(CAN_Msg_t *msg);

/* ============================================================
 * CAN 通道
 * ============================================================ */
typedef enum
{
    CAN_CH_1 = 0U,
    CAN_CH_2 = 1U,
} CAN_Channel_t;

/* ============================================================
 * 接收 FIFO 缓冲区大小
 * ============================================================ */
#define CAN_RX_FIFO_SIZE    32U

/* ============================================================
 * 软件 FIFO（线程安全的环形队列）
 * ============================================================ */
typedef struct
{
    CAN_Msg_t   buf[CAN_RX_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    volatile uint32_t overflow;     /* 溢出计数 */
} CAN_RxFifo_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief  初始化 CAN 通道
 * @param  ch       CAN_CH_1 或 CAN_CH_2
 * @param  baud     波特率枚举
 * @param  callback 接收中断回调（NULL 则只入 FIFO）
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef BSP_CAN_Init(CAN_Channel_t ch,
                               CAN_BaudRate_t baud,
                               CAN_RxCallback_t callback);

/**
 * @brief  发送一帧报文（阻塞等待邮箱空闲，超时 10ms）
 * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
 */
HAL_StatusTypeDef BSP_CAN_Send(CAN_Channel_t ch, CAN_Msg_t *msg);

/**
 * @brief  从软件 FIFO 读取一帧（非阻塞，无数据返回 0）
 * @retval 1=成功取到报文  0=FIFO 为空
 */
uint8_t BSP_CAN_Receive(CAN_Channel_t ch, CAN_Msg_t *msg);

/**
 * @brief  获取 FIFO 中待处理报文数量
 */
uint16_t BSP_CAN_GetRxCount(CAN_Channel_t ch);

/**
 * @brief  清空接收 FIFO
 */
void BSP_CAN_FlushRx(CAN_Channel_t ch);

/**
 * @brief  设置过滤器（接收指定 ID，默认接收全部）
 * @param  ch           通道
 * @param  filter_id    过滤 ID（标准帧 11 位 / 扩展帧 29 位）
 * @param  filter_mask  掩码（0=不关心该位，1=必须匹配）
 * @param  frame_type   CAN_FRAME_STD 或 CAN_FRAME_EXT
 * @note   调用前需先 BSP_CAN_Init
 */
HAL_StatusTypeDef BSP_CAN_SetFilter(CAN_Channel_t ch,
                                    uint32_t filter_id,
                                    uint32_t filter_mask,
                                    CAN_FrameType_t frame_type);

/**
 * @brief  内部中断处理，在 HAL CAN RX 回调中调用
 *         用户无需直接调用
 */
void BSP_CAN_RxFifoCallback(CAN_HandleTypeDef *hcan);

/* ============================================================
 * 直接访问 FIFO（调试用）
 * ============================================================ */
extern CAN_RxFifo_t g_can1_rx_fifo;
extern CAN_RxFifo_t g_can2_rx_fifo;

#ifdef __cplusplus
}
#endif
#endif /* __BSP_CAN_H */
