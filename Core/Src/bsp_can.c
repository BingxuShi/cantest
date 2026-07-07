/**
 ******************************************************************************
 * @file    bsp_can.c
 * @brief   STM32F105RCT6 双路 CAN 驱动实现（HAL 库）
 *
 * 架构说明：
 *   ┌─────────────────────────────────────────────┐
 *   │  HAL_CAN_RxFifo0MsgPendingCallback          │  ← HAL 中断回调
 *   │         ↓                                   │
 *   │  BSP_CAN_RxFifoCallback()                   │  ← 读取报文入软件FIFO
 *   │         ↓                   ↓               │
 *   │  CAN_RxFifo_t（环形队列）  用户回调函数      │
 *   │         ↓                                   │
 *   │  BSP_CAN_Receive()（主循环轮询）             │  ← 非阻塞取帧
 *   └─────────────────────────────────────────────┘
 *
 * 注意事项：
 *   1. STM32F105 互联型：CAN2 使用 CAN1 的过滤器组 14~27，
 *      即使只用 CAN2 也必须在 CubeMX 中使能 CAN1。
 *   2. 过滤器默认配置为"接收所有帧"，可调用 BSP_CAN_SetFilter 精确过滤。
 *   3. 发送函数等待空闲邮箱超时为 10ms，超时返回 HAL_TIMEOUT。
 ******************************************************************************
 */

#include "bsp_can.h"
#include "debug_rtt.h"
#include <string.h>

/* ============================================================
 * 内部句柄引用（CubeMX 生成于 can.c）
 * ============================================================ */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* ============================================================
 * 软件接收 FIFO
 * ============================================================ */
CAN_RxFifo_t g_can1_rx_fifo = {0};
CAN_RxFifo_t g_can2_rx_fifo = {0};

/* ============================================================
 * 用户回调函数指针
 * ============================================================ */
static CAN_RxCallback_t s_can1_callback = NULL;
static CAN_RxCallback_t s_can2_callback = NULL;

/* ============================================================
 * 发送成功/失败累计计数（供 BSP_CAN_GetStatus 上报给界面）
 * ============================================================ */
static uint32_t s_tx_ok_count[2]   = {0, 0};   /* [0]=CAN1, [1]=CAN2 */
static uint32_t s_tx_fail_count[2] = {0, 0};

/* ============================================================
 * 波特率精确参数查找表
 *
 * 修复说明：
 *   旧方案固定16TQ动态算Prescaler，36MHz下：
 *   Prescaler=4 → 36M/4/16=562.5Kbps（误差12.5%！）导致ACK Error
 *
 *   改为预计算精确参数表，误差0%。
 *   HAL枚举值：CAN_BS1_xTQ=(x-1), CAN_BS2_xTQ=(x-1)
 *
 *   36MHz APB1验证（72MHz主频/APB1二分频）：
 *     500K: PSC=4  BS1=14TQ BS2=3TQ → 36M/4/18=500000bps 采样点83% ✓
 *     250K: PSC=8  BS1=14TQ BS2=3TQ → 36M/8/18=250000bps 采样点83% ✓
 *     125K: PSC=16 BS1=14TQ BS2=3TQ → 36M/16/18=125000bps采样点83% ✓
 *     1M:   PSC=2  BS1=14TQ BS2=3TQ → 36M/2/18=1000000bps采样点83% ✓
 *   8MHz APB1验证（HSI直通不经PLL）：
 *     500K: PSC=1  BS1=13TQ BS2=3TQ → 8M/1/17≈470588（用16TQ更好）
 *     500K: PSC=1  BS1=12TQ BS2=3TQ → 8M/1/16=500000bps 采样点81% ✓
 * ============================================================ */
typedef struct
{
    uint32_t pclk1_hz;
    uint32_t baud_hz;
    uint32_t prescaler;
    uint32_t bs1;       /* HAL枚举值 = 实际TQ数 - 1 */
    uint32_t bs2;       /* HAL枚举值 = 实际TQ数 - 1 */
} CAN_TimingEntry_t;

typedef struct
{
    uint32_t prescaler;
    uint32_t bs1;
    uint32_t bs2;
    uint32_t sjw;
} CAN_TimingCfg_t;

static const CAN_TimingEntry_t s_timing_lut[] =
{
    /* PCLK1       波特率     PSC   BS1(=TQ-1)  BS2(=TQ-1)  */
    /* 36MHz APB1（72MHz主频，APB1二分频）*/
    { 36000000U, 1000000U,  2U,  13U,  2U }, /* 36M/2/18=1M,    采样点83.3% */
    { 36000000U,  500000U,  4U,  13U,  2U }, /* 36M/4/18=500K,  采样点83.3% */
    { 36000000U,  250000U,  8U,  13U,  2U }, /* 36M/8/18=250K,  采样点83.3% */
    { 36000000U,  125000U, 16U,  13U,  2U }, /* 36M/16/18=125K, 采样点83.3% */
    /* 8MHz APB1（HSI直通或低速配置）*/
    {  8000000U, 1000000U,  1U,   4U,  1U }, /* 8M/1/8=1M,      采样点75%   */
    {  8000000U,  500000U,  1U,  12U,  2U }, /* 8M/1/16=500K,   采样点81.3% */
    {  8000000U,  250000U,  2U,  12U,  2U }, /* 8M/2/16=250K,   采样点81.3% */
    {  8000000U,  125000U,  4U,  12U,  2U }, /* 8M/4/16=125K,   采样点81.3% */
};

static const uint32_t s_baud_value[] = {
    1000000U,   /* CAN_BAUD_1M   */
    500000U,    /* CAN_BAUD_500K */
    250000U,    /* CAN_BAUD_250K */
    125000U,    /* CAN_BAUD_125K */
};

static HAL_StatusTypeDef can_calc_timing(CAN_BaudRate_t baud, CAN_TimingCfg_t *out)
{
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
    uint32_t target = s_baud_value[(uint8_t)baud];
    uint32_t i;

    for (i = 0U; i < sizeof(s_timing_lut) / sizeof(s_timing_lut[0]); i++)
    {
        if (s_timing_lut[i].pclk1_hz == pclk1 &&
            s_timing_lut[i].baud_hz  == target)
        {
            out->prescaler = s_timing_lut[i].prescaler;
            out->bs1       = s_timing_lut[i].bs1;
            out->bs2       = s_timing_lut[i].bs2;
            out->sjw       = CAN_SJW_1TQ;

            /* 验证打印（仅显示Prescaler和实际波特率，BS1/BS2用宏值不便反算）*/
            DEBUG_PRINTF("[CAN] PCLK1=%lu target=%lu presc=%lu "
                         "BS1_macro=0x%lX BS2_macro=0x%lX -> matched LUT entry[%lu]\n",
                         pclk1, target, out->prescaler,
                         out->bs1, out->bs2, i);
            return HAL_OK;
        }
    }

    DEBUG_PRINTF("[CAN] ERROR: 未找到 PCLK1=%lu baud=%lu 的精确参数，"
                 "请在 s_timing_lut 中添加对应行\n", pclk1, target);
    return HAL_ERROR;
}


static void fifo_push(CAN_RxFifo_t *fifo, CAN_Msg_t *msg)
{
    if (fifo->count >= CAN_RX_FIFO_SIZE)
    {
        fifo->overflow++;
        return;
    }
    memcpy(&fifo->buf[fifo->tail], msg, sizeof(CAN_Msg_t));
    fifo->tail = (uint16_t)((fifo->tail + 1U) % CAN_RX_FIFO_SIZE);
    fifo->count++;
}

/* ============================================================
 * 内部：FIFO 读出（主循环调用）
 * ============================================================ */
static uint8_t fifo_pop(CAN_RxFifo_t *fifo, CAN_Msg_t *msg)
{
    uint32_t primask;

    if (fifo->count == 0U) return 0U;

    /* 临界区保护 */
    primask = __get_PRIMASK();
    __disable_irq();

    memcpy(msg, &fifo->buf[fifo->head], sizeof(CAN_Msg_t));
    fifo->head = (uint16_t)((fifo->head + 1U) % CAN_RX_FIFO_SIZE);
    fifo->count--;

    if (!primask) __enable_irq();

    return 1U;
}

/* ============================================================
 * 内部：配置过滤器为"接收全部"（初始化时调用）
 * CAN1 使用过滤器组 0（bank 0~13），CAN2 使用过滤器组 14（bank 14~27）
 * ============================================================ */
static HAL_StatusTypeDef can_filter_accept_all(CAN_Channel_t ch)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterIdHigh         = 0x0000U;
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = 0x0000U;
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation     = CAN_FILTER_ENABLE;

    if (ch == CAN_CH_1)
    {
        filter.FilterBank           = 0U;   /* CAN1 使用 bank 0 */
        filter.SlaveStartFilterBank = 14U;  /* CAN2 从 bank14 开始 */
        return HAL_CAN_ConfigFilter(&hcan1, &filter);
    }
    else
    {
        filter.FilterBank           = 14U;  /* CAN2 使用 bank 14 */
        filter.SlaveStartFilterBank = 14U;
        return HAL_CAN_ConfigFilter(&hcan2, &filter);
    }
}

/* ============================================================
 * BSP_CAN_Init
 * ============================================================ */
HAL_StatusTypeDef BSP_CAN_Init(CAN_Channel_t ch,
                               CAN_BaudRate_t baud,
                               CAN_RxCallback_t callback)
{
    HAL_StatusTypeDef ret;
    CAN_HandleTypeDef *phcan = (ch == CAN_CH_1) ? &hcan1 : &hcan2;
    CAN_TimingCfg_t timing;

    DEBUG_PRINTF("[CAN%d] Init begin, SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu\n",
                 (ch == CAN_CH_1) ? 1 : 2,
                 HAL_RCC_GetSysClockFreq(),
                 HAL_RCC_GetHCLKFreq(),
                 HAL_RCC_GetPCLK1Freq(),
                 HAL_RCC_GetPCLK2Freq());

    /* 动态计算时序（关键修复：不再使用固定写死的 Prescaler）*/
    ret = can_calc_timing(baud, &timing);
    if (ret != HAL_OK)
    {
        DEBUG_PRINTF("[CAN%d] Timing calc FAILED\n", (ch == CAN_CH_1) ? 1 : 2);
        return ret;
    }

    /* 停止并反初始化CAN，然后强制状态回到RESET
     *
     * 关键修复：HAL_CAN_Init内部在执行真正的初始化（写BTR寄存器）
     * 之前会检查 phcan->State，只有在 HAL_CAN_STATE_RESET 时才真正
     * 执行。HAL_CAN_DeInit虽然会清理外设，但在某些版本的HAL里，
     * DeInit后state可能不是RESET（或者MX_CAN1_Init已经把state设为
     * READY，导致我们的Init被跳过），必须手动把state强制设回RESET。
     */
    HAL_CAN_Stop(phcan);
    HAL_CAN_DeInit(phcan);
    phcan->State = HAL_CAN_STATE_RESET;   /* ← 强制复位状态，让Init真正执行 */

    phcan->Instance                  = (ch == CAN_CH_1) ? CAN1 : CAN2;
    phcan->Init.Prescaler            = timing.prescaler;
    phcan->Init.Mode                 = CAN_MODE_NORMAL;
    phcan->Init.SyncJumpWidth        = timing.sjw;
    phcan->Init.TimeSeg1             = timing.bs1;
    phcan->Init.TimeSeg2             = timing.bs2;
    phcan->Init.TimeTriggeredMode    = DISABLE;
    phcan->Init.AutoBusOff           = ENABLE;
    phcan->Init.AutoWakeUp           = DISABLE;
    phcan->Init.AutoRetransmission   = ENABLE;
    phcan->Init.ReceiveFifoLocked    = DISABLE;
    phcan->Init.TransmitFifoPriority = DISABLE;

    ret = HAL_CAN_Init(phcan);
    if (ret != HAL_OK)
    {
        DEBUG_PRINTF("[CAN%d] HAL_CAN_Init FAILED, ErrorCode=0x%08lX\n",
                     (ch == CAN_CH_1) ? 1 : 2, phcan->ErrorCode);
        return ret;
    }

    /* -------------------------------------------------------
     * 强制直接写BTR寄存器
     *
     * 背景：HAL_CAN_Init 在某些状态下会跳过真正的初始化流程，
     * 导致BTR保持MX_CAN1_Init()设置的默认值（BRP=15，750Kbps），
     * 而不是我们需要的参数。
     *
     * 修复：不依赖HAL状态机，直接操控寄存器：
     *   1. 进入初始化模式（INRQ=1，只有此模式下BTR可写）
     *   2. 写入正确的BTR值
     *   3. 退出初始化模式回到正常模式（INRQ=0）
     *
     * BTR寄存器布局：
     *   bit[9:0]   BRP = Prescaler-1
     *   bit[19:16] TS1 = BS1_TQ-1
     *   bit[22:20] TS2 = BS2_TQ-1
     *   bit[25:24] SJW = SJW_TQ-1
     * ------------------------------------------------------- */
    {
        uint32_t btr_val;
        uint32_t brp = timing.prescaler - 1U;
        uint32_t ts1 = timing.bs1;   /* HAL枚举值 = TQ-1，直接就是寄存器值 */
        uint32_t ts2 = timing.bs2;
        uint32_t sjw = 0U;           /* SJW=1TQ，寄存器值=0 */

        btr_val = (brp & 0x3FFU)
                | ((ts1 & 0xFU)  << 16U)
                | ((ts2 & 0x7U)  << 20U)
                | ((sjw & 0x3U)  << 24U);

        /* 请求进入初始化模式 */
        phcan->Instance->MCR |= CAN_MCR_INRQ;
        uint32_t timeout = HAL_GetTick();
        while (!(phcan->Instance->MSR & CAN_MSR_INAK))
        {
            if ((HAL_GetTick() - timeout) > 10U)
            {
                DEBUG_PRINTF("[CAN%d] INRQ timeout!\n", (ch == CAN_CH_1) ? 1 : 2);
                return HAL_TIMEOUT;
            }
        }

        /* 写BTR */
        phcan->Instance->BTR = btr_val;

        /* 退出初始化模式 */
        phcan->Instance->MCR &= ~CAN_MCR_INRQ;
        timeout = HAL_GetTick();
        while (phcan->Instance->MSR & CAN_MSR_INAK)
        {
            if ((HAL_GetTick() - timeout) > 10U)
            {
                DEBUG_PRINTF("[CAN%d] exit INRQ timeout!\n", (ch == CAN_CH_1) ? 1 : 2);
                return HAL_TIMEOUT;
            }
        }

        /* 验证写入结果 */
        uint32_t btr_read = phcan->Instance->BTR;
        uint32_t actual_brp = (btr_read & 0x3FFU) + 1U;
        uint32_t actual_ts1 = ((btr_read >> 16U) & 0xFU) + 1U;
        uint32_t actual_ts2 = ((btr_read >> 20U) & 0x7U) + 1U;
        uint32_t actual_baud = HAL_RCC_GetPCLK1Freq() /
                               (actual_brp * (1U + actual_ts1 + actual_ts2));
        DEBUG_PRINTF("[CAN%d] BTR=0x%08lX BRP=%lu TS1=%lu TS2=%lu "
                     "-> 实际波特率=%lu bps (期望500000)\n",
                     (ch == CAN_CH_1) ? 1 : 2,
                     btr_read, actual_brp, actual_ts1, actual_ts2, actual_baud);
    }

    /* 配置接收全部过滤器 */
    ret = can_filter_accept_all(ch);
    if (ret != HAL_OK)
    {
        DEBUG_PRINTF("[CAN%d] ConfigFilter FAILED\n", (ch == CAN_CH_1) ? 1 : 2);
        return ret;
    }

    /* 注册回调 */
    if (ch == CAN_CH_1)
    {
        s_can1_callback = callback;
        memset(&g_can1_rx_fifo, 0, sizeof(g_can1_rx_fifo));
    }
    else
    {
        s_can2_callback = callback;
        memset(&g_can2_rx_fifo, 0, sizeof(g_can2_rx_fifo));
    }

    /* 使能 FIFO0 接收中断 + 真正的错误中断
     * 注意：不激活 CAN_IT_LAST_ERROR_CODE，该中断在每一帧收发后
     * 只要LEC字段有变化就会触发，并非只在出错时触发，高频激活
     * 会导致 HAL_CAN_ErrorCallback 被每帧调用一次，干扰正常收发。*/
    ret = HAL_CAN_ActivateNotification(phcan, CAN_IT_RX_FIFO0_MSG_PENDING
                                              | CAN_IT_ERROR_WARNING
                                              | CAN_IT_ERROR_PASSIVE
                                              | CAN_IT_BUSOFF
                                              | CAN_IT_ERROR);
    if (ret != HAL_OK)
    {
        DEBUG_PRINTF("[CAN%d] ActivateNotification FAILED\n", (ch == CAN_CH_1) ? 1 : 2);
        return ret;
    }

    /* 启动 CAN */
    ret = HAL_CAN_Start(phcan);
    DEBUG_PRINTF("[CAN%d] Start %s, state=%d\n",
                 (ch == CAN_CH_1) ? 1 : 2,
                 (ret == HAL_OK) ? "OK" : "FAILED",
                 HAL_CAN_GetState(phcan));
    return ret;
}

/* ============================================================
 * BSP_CAN_Send（非阻塞版本）
 *
 * 重要修复：旧版本在邮箱满时会 while循环阻塞等待最多10ms，
 * 如果总线持续异常（仲裁丢失/Bus-Off/无应答），邮箱会一直不空，
 * 每次调用都白白卡住10ms——在 Simulation mode 这种50ms周期发送
 * 场景下，这10ms阻塞会直接拖慢整个主循环，表现为EC11旋转/按键
 * 响应明显延迟（因为 GUI_Menu_Process 和 CAN发送在同一个while循环
 * 里顺序执行，CAN卡住，菜单处理就跟着卡住）。
 *
 * 新版本：只检查一次邮箱状态，没有空闲邮箱立即返回 HAL_BUSY，
 * 不做任何阻塞等待。调用方（例如 sim_page_on_tick）发送失败时
 * 直接跳过这一帧即可，不影响下一次50ms周期重试，也不阻塞界面。
 * ============================================================ */
HAL_StatusTypeDef BSP_CAN_Send(CAN_Channel_t ch, CAN_Msg_t *msg)
{
    CAN_HandleTypeDef *phcan = (ch == CAN_CH_1) ? &hcan1 : &hcan2;
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;
    HAL_StatusTypeDef ret;

    /* —— 节流打印：错误日志不限频会刷爆RTT缓冲区，反而影响调试 —— */
    static uint32_t s_last_err_log_tick[2] = {0, 0};
    uint8_t ch_idx = (ch == CAN_CH_1) ? 0U : 1U;
    uint8_t ch_num = (ch == CAN_CH_1) ? 1U : 2U;

    if (msg == NULL || msg->dlc > 8U)
    {
        DEBUG_PRINTF("[CAN%d] Send: invalid param\n", ch_num);
        return HAL_ERROR;
    }

    /* —— 非阻塞检查：邮箱是否有空位，没有立即返回，不等待 —— */
    if (HAL_CAN_GetTxMailboxesFreeLevel(phcan) == 0U)
    {
        uint32_t now = HAL_GetTick();
        s_tx_fail_count[ch_idx]++;
        if ((now - s_last_err_log_tick[ch_idx]) >= 1000U)   /* 每通道最多1秒打印一次 */
        {
            s_last_err_log_tick[ch_idx] = now;
            DEBUG_PRINTF("[CAN%d] Send BUSY (no free mailbox), ESR=0x%08lX TSR=0x%08lX "
                        "(此日志限频1次/秒，实际发送尝试更频繁)\n",
                        ch_num, phcan->Instance->ESR, phcan->Instance->TSR);
        }
        return HAL_BUSY;
    }

    /* 填写帧头 */
    if (msg->frame_type == CAN_FRAME_STD)
    {
        tx_header.IDE   = CAN_ID_STD;
        tx_header.StdId = msg->id & 0x7FFU;
    }
    else
    {
        tx_header.IDE   = CAN_ID_EXT;
        tx_header.ExtId = msg->id & 0x1FFFFFFFU;
    }
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = msg->dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    ret = HAL_CAN_AddTxMessage(phcan, &tx_header, msg->data, &tx_mailbox);

    if (ret == HAL_OK)
    {
        s_tx_ok_count[ch_idx]++;
    }
    else
    {
        s_tx_fail_count[ch_idx]++;
        uint32_t now = HAL_GetTick();
        if ((now - s_last_err_log_tick[ch_idx]) >= 1000U)
        {
            s_last_err_log_tick[ch_idx] = now;
            DEBUG_PRINTF("[CAN%d] TX id=0x%lX FAIL, ESR=0x%08lX\n",
                        ch_num, msg->id, phcan->Instance->ESR);
        }
    }

    return ret;
}

/* ============================================================
 * BSP_CAN_GetStatus —— 解析 ESR 寄存器，输出结构化总线状态
 *
 * ESR 寄存器位定义（参考 STM32F1 参考手册 bxCAN 章节）：
 *   bit0    EWGF — Error Warning Flag
 *   bit1    EPVF — Error Passive Flag
 *   bit2    BOFF — Bus-Off Flag
 *   bit4:6  LEC  — Last Error Code
 *   bit16:23 TEC — Transmit Error Counter
 *   bit24:31 REC — Receive Error Counter
 * ============================================================ */
void BSP_CAN_GetStatus(CAN_Channel_t ch, CAN_BusStatus_t *status)
{
    CAN_HandleTypeDef *phcan = (ch == CAN_CH_1) ? &hcan1 : &hcan2;
    uint8_t ch_idx = (ch == CAN_CH_1) ? 0U : 1U;
    uint32_t esr;

    if (status == NULL) return;

    esr = phcan->Instance->ESR;

    status->error_warning = (uint8_t)(esr & 0x1U);
    status->error_passive = (uint8_t)((esr >> 1U) & 0x1U);
    status->bus_off        = (uint8_t)((esr >> 2U) & 0x1U);
    status->lec            = (CAN_LecError_t)((esr >> 4U) & 0x7U);
    status->tec            = (uint8_t)((esr >> 16U) & 0xFFU);
    status->rec            = (uint8_t)((esr >> 24U) & 0xFFU);
    status->tx_ok_count   = s_tx_ok_count[ch_idx];
    status->tx_fail_count = s_tx_fail_count[ch_idx];
}

/* ============================================================
 * BSP_CAN_LecToString
 * ============================================================ */
const char *BSP_CAN_LecToString(CAN_LecError_t lec)
{
    switch (lec)
    {
        case CAN_LEC_NONE:    return "OK";
        case CAN_LEC_STUFF:   return "StuffErr";
        case CAN_LEC_FORM:    return "FormErr";
        case CAN_LEC_ACK:     return "AckErr";
        case CAN_LEC_BIT_REC: return "BitRecErr";
        case CAN_LEC_BIT_DOM: return "BitDomErr";
        case CAN_LEC_CRC:     return "CrcErr";
        default:              return "Unknown";
    }
}

/* ============================================================
 * BSP_CAN_Receive（主循环非阻塞取帧）
 * ============================================================ */
uint8_t BSP_CAN_Receive(CAN_Channel_t ch, CAN_Msg_t *msg)
{
    CAN_RxFifo_t *fifo = (ch == CAN_CH_1) ? &g_can1_rx_fifo : &g_can2_rx_fifo;
    return fifo_pop(fifo, msg);
}

/* ============================================================
 * BSP_CAN_GetRxCount
 * ============================================================ */
uint16_t BSP_CAN_GetRxCount(CAN_Channel_t ch)
{
    CAN_RxFifo_t *fifo = (ch == CAN_CH_1) ? &g_can1_rx_fifo : &g_can2_rx_fifo;
    return fifo->count;
}

/* ============================================================
 * BSP_CAN_FlushRx
 * ============================================================ */
void BSP_CAN_FlushRx(CAN_Channel_t ch)
{
    CAN_RxFifo_t *fifo = (ch == CAN_CH_1) ? &g_can1_rx_fifo : &g_can2_rx_fifo;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(fifo, 0, sizeof(CAN_RxFifo_t));
    if (!primask) __enable_irq();
}

/* ============================================================
 * BSP_CAN_SetFilter（精确 ID 过滤）
 * ============================================================ */
HAL_StatusTypeDef BSP_CAN_SetFilter(CAN_Channel_t ch,
                                    uint32_t filter_id,
                                    uint32_t filter_mask,
                                    CAN_FrameType_t frame_type)
{
    CAN_FilterTypeDef filter = {0};
    CAN_HandleTypeDef *phcan = (ch == CAN_CH_1) ? &hcan1 : &hcan2;

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (frame_type == CAN_FRAME_STD)
    {
        /* 标准帧：ID 存放在 [31:21]，IDE 位=0，RTR 位=0 */
        filter.FilterIdHigh     = (uint16_t)((filter_id  & 0x7FFU) << 5U);
        filter.FilterIdLow      = 0x0000U;
        filter.FilterMaskIdHigh = (uint16_t)((filter_mask & 0x7FFU) << 5U);
        filter.FilterMaskIdLow  = 0x0000U;
    }
    else
    {
        /* 扩展帧：ID 存放在 [31:3]，IDE 位=1 */
        filter.FilterIdHigh     = (uint16_t)((filter_id  & 0x1FFFFFFFU) >> 13U);
        filter.FilterIdLow      = (uint16_t)(((filter_id  & 0x1FFFU) << 3U) | 0x04U);
        filter.FilterMaskIdHigh = (uint16_t)((filter_mask & 0x1FFFFFFFU) >> 13U);
        filter.FilterMaskIdLow  = (uint16_t)(((filter_mask & 0x1FFFU) << 3U) | 0x04U);
    }

    filter.FilterBank = (ch == CAN_CH_1) ? 0U : 14U;

    return HAL_CAN_ConfigFilter(phcan, &filter);
}

/* ============================================================
 * BSP_CAN_RxFifoCallback（中断上下文，由 HAL 回调调用）
 * ============================================================ */
void BSP_CAN_RxFifoCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    CAN_Msg_t msg = {0};
    CAN_RxFifo_t *fifo;
    CAN_RxCallback_t cb;
    uint8_t ch_num = (hcan->Instance == CAN1) ? 1U : 2U;

    /* 读取 HAL FIFO0 中所有待处理帧 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data) != HAL_OK)
        {
            DEBUG_PRINTF("[CAN%d] GetRxMessage FAILED\n", ch_num);
            break;
        }

        /* 解析帧头 */
        if (rx_header.IDE == CAN_ID_STD)
        {
            msg.id         = rx_header.StdId;
            msg.frame_type = CAN_FRAME_STD;
        }
        else
        {
            msg.id         = rx_header.ExtId;
            msg.frame_type = CAN_FRAME_EXT;
        }
        msg.dlc = (uint8_t)rx_header.DLC;

        /* 注意：此处刻意不打印每帧报文的 DEBUG_PRINTF！
         * 本回调运行在 CAN1/CAN2_RX0 中断上下文，如果总线流量较大
         * （例如 Simulation mode 50ms周期发送，且总线上有应答/自环），
         * 频繁的格式化打印会显著拖慢本中断执行时间，与 TIM6（2ms周期
         * 驱动EC11采样）的中断产生相互干扰，是"EC11大部分时间没反应"
         * 的潜在根因之一。如需查看收到的报文，请在主循环里通过
         * BSP_CAN_Receive() 取出后再打印（不占用中断时间）。 */

        /* 区分 CAN1 / CAN2 */
        if (hcan->Instance == CAN1)
        {
            fifo = &g_can1_rx_fifo;
            cb   = s_can1_callback;
        }
        else
        {
            fifo = &g_can2_rx_fifo;
            cb   = s_can2_callback;
        }

        /* 压入软件 FIFO */
        fifo_push(fifo, &msg);
        if (fifo->overflow > 0U)
            DEBUG_PRINTF("[CAN%d] FIFO overflow count=%lu\n", ch_num, fifo->overflow);

        /* 触发用户回调 */
        if (cb != NULL)
            cb(&msg);
    }
}

/* ============================================================
 * HAL CAN 接收中断回调（FIFO0，HAL 框架调用）
 * 此函数为 weak 函数覆盖，直接在本文件实现
 * ============================================================ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_CAN_RxFifoCallback(hcan);
}

/* ============================================================
 * HAL CAN 错误回调（调试用：BusOff / 应答错误等会在此打印）
 *
 * 限流说明：此回调运行在 CAN1/CAN2_SCE 中断上下文。如果总线存在
 * 持续性错误（例如未接终端电阻导致间歇性 ACK/Stuff Error），该
 * 中断可能被高频反复触发，每次最多打印9条 DEBUG_PRINTF，会显著
 * 拖慢中断执行时间，进而与 TIM6（2ms周期驱动EC11采样）产生干扰。
 * 加入限流：同一通道最多每500ms打印一次完整错误详情。
 * ============================================================ */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    static uint32_t s_last_err_print_tick[2] = {0, 0};
    uint8_t  ch_num = (hcan->Instance == CAN1) ? 1U : 2U;
    uint8_t  ch_idx = (hcan->Instance == CAN1) ? 0U : 1U;
    uint32_t err    = HAL_CAN_GetError(hcan);
    uint32_t now    = HAL_GetTick();

    /* 限流：同一通道最多每500ms打印一次 */
    if ((now - s_last_err_print_tick[ch_idx]) < 500U)
        return;

    /* 只在真正有非零错误时才打印，过滤掉 HAL_CAN_ERROR_NONE */
    if (err == HAL_CAN_ERROR_NONE)
        return;

    s_last_err_print_tick[ch_idx] = now;

    DEBUG_PRINTF("[CAN%d] ErrorCallback ErrorCode=0x%08lX ESR=0x%08lX\n",
                 ch_num, err, hcan->Instance->ESR);

    if (err & HAL_CAN_ERROR_BOF)
        DEBUG_PRINTF("[CAN%d] -> Bus-Off!\n", ch_num);
    if (err & HAL_CAN_ERROR_EWG)
        DEBUG_PRINTF("[CAN%d] -> Error Warning\n", ch_num);
    if (err & HAL_CAN_ERROR_EPV)
        DEBUG_PRINTF("[CAN%d] -> Error Passive\n", ch_num);
    if (err & HAL_CAN_ERROR_STF)
        DEBUG_PRINTF("[CAN%d] -> Stuff Error\n", ch_num);
    if (err & HAL_CAN_ERROR_FOR)
        DEBUG_PRINTF("[CAN%d] -> Form Error\n", ch_num);
    if (err & HAL_CAN_ERROR_ACK)
        DEBUG_PRINTF("[CAN%d] -> ACK Error (总线上无节点应答)\n", ch_num);
    if (err & HAL_CAN_ERROR_BR)
        DEBUG_PRINTF("[CAN%d] -> Bit Recessive Error\n", ch_num);
    if (err & HAL_CAN_ERROR_BD)
        DEBUG_PRINTF("[CAN%d] -> Bit Dominant Error\n", ch_num);
    if (err & HAL_CAN_ERROR_CRC)
        DEBUG_PRINTF("[CAN%d] -> CRC Error\n", ch_num);
}
