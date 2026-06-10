/**
 ******************************************************************************
 * @file    st7789v.c
 * @brief   ST7789V OLED 屏幕驱动（基于 STM32 HAL 库）
 *
 * 硬件连接：
 *   SCK  -> PA5  (SPI1_SCK)
 *   MOSI -> PA7  (SPI1_MOSI)
 *   RES  -> PC4  (GPIO 输出，硬件复位)
 *   DC   -> PC5  (GPIO 输出，命令/数据选择)
 *   CS   -> PB0  (GPIO 输出，片选，低电平有效)
 *   BLK  -> PB1  (GPIO 输出，背光，高电平亮)
 *
 * CubeMX SPI1 配置：
 *   Mode              : Transmit Only Master
 *   Hardware NSS      : Disable (软件控制 CS)
 *   Data Size         : 8 Bits
 *   First Bit         : MSB First
 *   Clock Polarity    : High  (CPOL = 1)
 *   Clock Phase       : 2 Edge (CPHA = 1)  → SPI Mode 3
 *   Prescaler         : /4 → 18 MHz（72MHz 主频时）
 *   DMA (可选)        : SPI1_TX, DMA1 Channel3, 方向 Mem→Periph
 ******************************************************************************
 */

#include "st7789v.h"
#include <stdlib.h>     /* abs() */

/*============================================================
 * 内部：批量发送缓冲区（FillColor 使用 DMA 时需要）
 * 大小可根据 RAM 酌情调整
 *============================================================*/
#define FILL_BUF_SIZE   512U    /* 单位：像素（每像素2字节，共1024字节）*/
static uint8_t s_fill_buf[FILL_BUF_SIZE * 2U];

/*============================================================
 * 底层：写命令字节
 *============================================================*/
void LCD_WriteCmd(uint8_t cmd)
{
    LCD_DC_L();                         /* DC=0 → 命令 */
    LCD_CS_L();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, &cmd, 1U, LCD_SPI_TIMEOUT);
    LCD_CS_H();
}

/*============================================================
 * 底层：写单字节数据
 *============================================================*/
void LCD_WriteData8(uint8_t data)
{
    LCD_DC_H();                         /* DC=1 → 数据 */
    LCD_CS_L();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, &data, 1U, LCD_SPI_TIMEOUT);
    LCD_CS_H();
}

/*============================================================
 * 底层：写16位数据（高字节先）
 *============================================================*/
void LCD_WriteData16(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8U);
    buf[1] = (uint8_t)(data & 0xFFU);

    LCD_DC_H();
    LCD_CS_L();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, buf, 2U, LCD_SPI_TIMEOUT);
    LCD_CS_H();
}

/*============================================================
 * 底层：写多字节数据缓冲区
 *============================================================*/
void LCD_WriteBuf(const uint8_t *buf, uint32_t len)
{
    LCD_DC_H();
    LCD_CS_L();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, (uint8_t *)buf, (uint16_t)len, LCD_SPI_TIMEOUT);
    LCD_CS_H();
}

/*============================================================
 * 硬件复位
 *============================================================*/
static void LCD_HardReset(void)
{
    LCD_RES_H();
    HAL_Delay(10U);
    LCD_RES_L();
    HAL_Delay(20U);
    LCD_RES_H();
    HAL_Delay(120U);
}

/*============================================================
 * ST7789V 寄存器初始化序列
 *============================================================*/
static void LCD_RegInit(void)
{
    /* 软件复位 */
    LCD_WriteCmd(ST7789_SWRESET);
    HAL_Delay(150U);

    /* 退出睡眠 */
    LCD_WriteCmd(ST7789_SLPOUT);
    HAL_Delay(120U);

    /* 像素格式：RGB565，16bit */
    LCD_WriteCmd(ST7789_COLMOD);
    LCD_WriteData8(0x55U);
    HAL_Delay(10U);

    /* 显示方向：默认0° */
    LCD_WriteCmd(ST7789_MADCTL);
    LCD_WriteData8(0x00U);

    /* 列地址范围：0 ~ 239 */
    LCD_WriteCmd(ST7789_CASET);
    LCD_WriteData16(0x0000U + LCD_X_OFFSET);
    LCD_WriteData16(LCD_WIDTH - 1U + LCD_X_OFFSET);

    /* 行地址范围：0 ~ 279 */
    LCD_WriteCmd(ST7789_RASET);
    LCD_WriteData16(0x0000U + LCD_Y_OFFSET);
    LCD_WriteData16(LCD_HEIGHT - 1U + LCD_Y_OFFSET);

    /* Porch 控制 */
    LCD_WriteCmd(ST7789_PORCTRL);
    LCD_WriteData8(0x0CU);
    LCD_WriteData8(0x0CU);
    LCD_WriteData8(0x00U);
    LCD_WriteData8(0x33U);
    LCD_WriteData8(0x33U);

    /* 门控制 */
    LCD_WriteCmd(ST7789_GCTRL);
    LCD_WriteData8(0x35U);

    /* VCOM 设置 */
    LCD_WriteCmd(ST7789_VCOMS);
    LCD_WriteData8(0x19U);

    /* LCM 控制 */
    LCD_WriteCmd(ST7789_LCMCTRL);
    LCD_WriteData8(0x2CU);

    /* VDV/VRH 命令使能 */
    LCD_WriteCmd(ST7789_VDVVRHEN);
    LCD_WriteData8(0x01U);

    /* VRH 设置 */
    LCD_WriteCmd(ST7789_VRHS);
    LCD_WriteData8(0x12U);

    /* VDV 设置 */
    LCD_WriteCmd(ST7789_VDVSET);
    LCD_WriteData8(0x20U);

    /* 帧率：60Hz */
    LCD_WriteCmd(ST7789_FRCTR2);
    LCD_WriteData8(0x0FU);

    /* 电源控制1 */
    LCD_WriteCmd(ST7789_PWCTRL1);
    LCD_WriteData8(0xA4U);
    LCD_WriteData8(0xA1U);

    /* 正向伽马 */
    LCD_WriteCmd(ST7789_PVGAMCTRL);
    LCD_WriteData8(0xD0U); LCD_WriteData8(0x04U);
    LCD_WriteData8(0x0DU); LCD_WriteData8(0x11U);
    LCD_WriteData8(0x13U); LCD_WriteData8(0x2BU);
    LCD_WriteData8(0x3FU); LCD_WriteData8(0x54U);
    LCD_WriteData8(0x4CU); LCD_WriteData8(0x18U);
    LCD_WriteData8(0x0DU); LCD_WriteData8(0x0BU);
    LCD_WriteData8(0x1FU); LCD_WriteData8(0x23U);

    /* 负向伽马 */
    LCD_WriteCmd(ST7789_NVGAMCTRL);
    LCD_WriteData8(0xD0U); LCD_WriteData8(0x04U);
    LCD_WriteData8(0x0CU); LCD_WriteData8(0x11U);
    LCD_WriteData8(0x13U); LCD_WriteData8(0x2CU);
    LCD_WriteData8(0x3FU); LCD_WriteData8(0x44U);
    LCD_WriteData8(0x51U); LCD_WriteData8(0x2FU);
    LCD_WriteData8(0x1FU); LCD_WriteData8(0x1FU);
    LCD_WriteData8(0x20U); LCD_WriteData8(0x23U);

    /* 显示反转开启（大多数 240×280 屏幕需要；如颜色反则改 INVOFF）*/
    LCD_WriteCmd(ST7789_INVON);

    /* 开启显示 */
    LCD_WriteCmd(ST7789_DISPON);
    HAL_Delay(20U);
}

/*============================================================
 * LCD 初始化（对外主接口）
 * 注意：需在 HAL_Init()、SystemClock_Config()、MX_SPI1_Init()
 *       以及 GPIO 初始化完成之后调用。
 *============================================================*/
void LCD_Init(void)
{
    LCD_CS_H();
    LCD_BLK_L();
    LCD_RES_H();

    LCD_HardReset();
    LCD_RegInit();
    LCD_BacklightOn();
    LCD_FillScreen(COLOR_BLACK);    /* 清屏为黑色 */
}

/*============================================================
 * 背光控制
 *============================================================*/
void LCD_BacklightOn(void)
{
    LCD_BLK_H();
}

void LCD_BacklightOff(void)
{
    LCD_BLK_L();
}

/*============================================================
 * 显示开关
 *============================================================*/
void LCD_DisplayOn(void)
{
    LCD_WriteCmd(ST7789_DISPON);
}

void LCD_DisplayOff(void)
{
    LCD_WriteCmd(ST7789_DISPOFF);
}

/*============================================================
 * 睡眠控制
 *============================================================*/
void LCD_SleepIn(void)
{
    LCD_WriteCmd(ST7789_SLPIN);
    HAL_Delay(5U);
}

void LCD_SleepOut(void)
{
    LCD_WriteCmd(ST7789_SLPOUT);
    HAL_Delay(120U);
}

/*============================================================
 * 设置显示方向
 *============================================================*/
void LCD_SetRotation(uint8_t rotation)
{
    LCD_WriteCmd(ST7789_MADCTL);
    /* bit3=1 选择 BGR 颜色顺序（大多数 ST7789V 屏幕需要）*/
    LCD_WriteData8(rotation | 0x08U);
}

/*============================================================
 * 设置显示窗口
 *============================================================*/
void LCD_SetWindow(uint16_t x_start, uint16_t y_start,
                   uint16_t x_end,   uint16_t y_end)
{
    uint8_t buf[4];

    /* 列地址 */
    LCD_WriteCmd(ST7789_CASET);
    buf[0] = (uint8_t)((x_start + LCD_X_OFFSET) >> 8U);
    buf[1] = (uint8_t)((x_start + LCD_X_OFFSET) & 0xFFU);
    buf[2] = (uint8_t)((x_end   + LCD_X_OFFSET) >> 8U);
    buf[3] = (uint8_t)((x_end   + LCD_X_OFFSET) & 0xFFU);
    LCD_WriteBuf(buf, 4U);

    /* 行地址 */
    LCD_WriteCmd(ST7789_RASET);
    buf[0] = (uint8_t)((y_start + LCD_Y_OFFSET) >> 8U);
    buf[1] = (uint8_t)((y_start + LCD_Y_OFFSET) & 0xFFU);
    buf[2] = (uint8_t)((y_end   + LCD_Y_OFFSET) >> 8U);
    buf[3] = (uint8_t)((y_end   + LCD_Y_OFFSET) & 0xFFU);
    LCD_WriteBuf(buf, 4U);

    /* 开始写显存 */
    LCD_WriteCmd(ST7789_RAMWR);
}

/*============================================================
 * 连续填充同一颜色 count 个像素
 * 使用静态缓冲区分批次发送，提高效率
 *============================================================*/
void LCD_FillColor(uint16_t color, uint32_t count)
{
    uint32_t i;
    uint32_t batch;
    uint8_t  hi = (uint8_t)(color >> 8U);
    uint8_t  lo = (uint8_t)(color & 0xFFU);

    /* 预填充缓冲区 */
    for (i = 0U; i < FILL_BUF_SIZE; i++)
    {
        s_fill_buf[i * 2U]      = hi;
        s_fill_buf[i * 2U + 1U] = lo;
    }

    LCD_DC_H();
    LCD_CS_L();

    while (count > 0U)
    {
        batch = (count > FILL_BUF_SIZE) ? FILL_BUF_SIZE : count;
        HAL_SPI_Transmit(&LCD_SPI_HANDLE, s_fill_buf,
                         (uint16_t)(batch * 2U), LCD_SPI_TIMEOUT);
        count -= batch;
    }

    LCD_CS_H();
}

/*============================================================
 * 画单点
 *============================================================*/
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    LCD_SetWindow(x, y, x, y);
    LCD_FillColor(color, 1U);
}

/*============================================================
 * 填充矩形区域
 *============================================================*/
void LCD_FillRect(uint16_t x, uint16_t y,
                  uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if ((uint32_t)x + w > LCD_WIDTH)  w = (uint16_t)(LCD_WIDTH  - x);
    if ((uint32_t)y + h > LCD_HEIGHT) h = (uint16_t)(LCD_HEIGHT - y);

    LCD_SetWindow(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
    LCD_FillColor(color, (uint32_t)w * h);
}

/*============================================================
 * 全屏填充
 *============================================================*/
void LCD_FillScreen(uint16_t color)
{
    LCD_FillRect(0U, 0U, LCD_WIDTH, LCD_HEIGHT, color);
}

/*============================================================
 * 画直线（Bresenham 算法）
 *============================================================*/
void LCD_DrawLine(uint16_t x0, uint16_t y0,
                  uint16_t x1, uint16_t y1, uint16_t color)
{
    int16_t dx  =  (int16_t)abs((int16_t)x1 - (int16_t)x0);
    int16_t dy  = -(int16_t)abs((int16_t)y1 - (int16_t)y0);
    int16_t sx  = ((int16_t)x0 < (int16_t)x1) ?  1 : -1;
    int16_t sy  = ((int16_t)y0 < (int16_t)y1) ?  1 : -1;
    int16_t err = dx + dy;
    int16_t e2;

    for (;;)
    {
        LCD_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 = (uint16_t)((int16_t)x0 + sx); }
        if (e2 <= dx) { err += dx; y0 = (uint16_t)((int16_t)y0 + sy); }
    }
}

/*============================================================
 * 画空心矩形
 *============================================================*/
void LCD_DrawRect(uint16_t x, uint16_t y,
                  uint16_t w, uint16_t h, uint16_t color)
{
    LCD_DrawLine(x,           y,           x + w - 1U, y,           color);
    LCD_DrawLine(x,           y + h - 1U,  x + w - 1U, y + h - 1U,  color);
    LCD_DrawLine(x,           y,           x,           y + h - 1U,  color);
    LCD_DrawLine(x + w - 1U,  y,           x + w - 1U,  y + h - 1U,  color);
}

/*============================================================
 * 画空心圆（中点圆算法）
 *============================================================*/
void LCD_DrawCircle(uint16_t cx, uint16_t cy,
                    uint16_t r,  uint16_t color)
{
    int16_t x = 0;
    int16_t y = (int16_t)r;
    int16_t d = 1 - (int16_t)r;

    while (x <= y)
    {
        LCD_DrawPixel((uint16_t)((int16_t)cx + x), (uint16_t)((int16_t)cy + y), color);
        LCD_DrawPixel((uint16_t)((int16_t)cx - x), (uint16_t)((int16_t)cy + y), color);
        LCD_DrawPixel((uint16_t)((int16_t)cx + x), (uint16_t)((int16_t)cy - y), color);
        LCD_DrawPixel((uint16_t)((int16_t)cx - x), (uint16_t)((int16_t)cy - y), color);
        LCD_DrawPixel((uint16_t)((int16_t)cx + y), (uint16_t)((int16_t)cy + x), color);
        LCD_DrawPixel((uint16_t)((int16_t)cx - y), (uint16_t)((int16_t)cy + x), color);
        LCD_DrawPixel((uint16_t)((int16_t)cx + y), (uint16_t)((int16_t)cy - x), color);
        LCD_DrawPixel((uint16_t)((int16_t)cx - y), (uint16_t)((int16_t)cy - x), color);

        if (d < 0)
            d += 2 * x + 3;
        else
        {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}

/*============================================================
 * 画实心圆（水平扫描线填充）
 *============================================================*/
void LCD_FillCircle(uint16_t cx, uint16_t cy,
                    uint16_t r,  uint16_t color)
{
    int16_t x, y;
    int16_t ir = (int16_t)r;

    for (y = -ir; y <= ir; y++)
    {
        /* 当前行的水平半径 */
        int16_t xr = 0;
        int32_t val = (int32_t)ir * ir - (int32_t)y * y;
        if (val > 0)
        {
            /* 整数平方根（简单逼近）*/
            xr = (int16_t)ir;
            while ((int32_t)xr * xr > val) xr--;
        }

        int16_t draw_y = (int16_t)cy + y;
        if (draw_y < 0 || draw_y >= (int16_t)LCD_HEIGHT) continue;

        int16_t x_left  = (int16_t)cx - xr;
        int16_t x_right = (int16_t)cx + xr;
        if (x_left  < 0)               x_left  = 0;
        if (x_right >= (int16_t)LCD_WIDTH) x_right = (int16_t)LCD_WIDTH - 1;

        if (x_right >= x_left)
        {
            LCD_SetWindow((uint16_t)x_left,  (uint16_t)draw_y,
                          (uint16_t)x_right, (uint16_t)draw_y);
            LCD_FillColor(color, (uint32_t)(x_right - x_left + 1));
        }
        (void)x;
    }
}

/*============================================================
 * 显示 RGB565 图片（阻塞模式）
 *============================================================*/
void LCD_DrawImage(uint16_t x, uint16_t y,
                   uint16_t w, uint16_t h,
                   const uint16_t *image)
{
    uint32_t total = (uint32_t)w * h;
    uint32_t i;

    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || image == NULL) return;

    LCD_SetWindow(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));

    LCD_DC_H();
    LCD_CS_L();

    /* 逐像素大端发送，或按512像素批次转换再发 */
    for (i = 0U; i < total; )
    {
        uint32_t batch = (total - i > FILL_BUF_SIZE) ? FILL_BUF_SIZE : (total - i);
        uint32_t j;
        for (j = 0U; j < batch; j++)
        {
            s_fill_buf[j * 2U]      = (uint8_t)(image[i + j] >> 8U);
            s_fill_buf[j * 2U + 1U] = (uint8_t)(image[i + j] & 0xFFU);
        }
        HAL_SPI_Transmit(&LCD_SPI_HANDLE, s_fill_buf,
                         (uint16_t)(batch * 2U), LCD_SPI_TIMEOUT);
        i += batch;
    }

    LCD_CS_H();
}

/*============================================================
 * 显示 RGB565 图片（DMA 非阻塞模式）
 * 前提：
 *   1. CubeMX 中为 SPI1_TX 配置 DMA（DMA1 Channel3）
 *   2. 图片数据 image 的生命周期须覆盖整个 DMA 传输过程
 *   3. 数据量受 HAL DMA 单次 65535 字节限制，大图需分段
 * 回调：
 *   HAL_SPI_TxCpltCallback 中调用 LCD_CS_H() 结束传输
 *============================================================*/
void LCD_DrawImage_DMA(uint16_t x, uint16_t y,
                       uint16_t w, uint16_t h,
                       const uint16_t *image)
{
    uint32_t total_bytes = (uint32_t)w * h * 2U;

    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || image == NULL) return;

    LCD_SetWindow(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));

    LCD_DC_H();
    LCD_CS_L();

    /*
     * 注意：HAL_SPI_Transmit_DMA 第3个参数为 uint16_t，
     * 最大 65535（字节），大图像需分段调用。
     * 此处假设总数据量不超过 65535 字节（即 ≤32767 像素）。
     */
    if (total_bytes <= 65535U)
    {
        HAL_SPI_Transmit_DMA(&LCD_SPI_HANDLE,
                             (uint8_t *)image,
                             (uint16_t)total_bytes);
        /* CS 在 HAL_SPI_TxCpltCallback 中释放 */
    }
    else
    {
        /* 大图像回退到阻塞模式 */
        LCD_CS_H();
        LCD_DrawImage(x, y, w, h, image);
    }
}

/**
 * @brief SPI 发送完成回调（DMA 模式使用）
 *        在用户的 stm32f1xx_it.c 或 main.c 中实现此函数，
 *        或将以下代码添加到项目中。
 *
 * void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
 * {
 *     if (hspi->Instance == SPI1)
 *     {
 *         LCD_CS_H();   // 释放片选
 *     }
 * }
 */
