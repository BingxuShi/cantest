#ifndef __ST7789V_H
#define __ST7789V_H

#include "main.h"          /* CubeMX生成，包含 stm32f1xx_hal.h 及引脚定义 */
#include "spi.h"           /* CubeMX生成的 hspi1 句柄 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file    st7789v.h
 * @brief   ST7789V OLED 屏幕驱动头文件（基于 STM32 HAL 库）
 *
 * 硬件连接：
 *   SCK  -> PA5  (SPI1_SCK,  复用推挽)
 *   MOSI -> PA7  (SPI1_MOSI, 复用推挽)
 *   RES  -> PC4  (GPIO 输出)
 *   DC   -> PC5  (GPIO 输出)
 *   CS   -> PB0  (GPIO 输出)
 *   BLK  -> PB1  (GPIO 输出)
 *
 * CubeMX 配置要点：
 *   - SPI1: Mode=Transmit Only Master, NSS=Software
 *           Data Size=8Bit, CPOL=High, CPHA=2Edge (Mode3)
 *           Prescaler 按需设置（建议 4 分频，即 18MHz）
 *   - PC4/PC5/PB0/PB1: GPIO_Output, Push-Pull, No pull
 ******************************************************************************
 */

/*============================================================
 * GPIO 引脚宏（与 CubeMX 生成的标签保持一致）
 * 若 CubeMX 中已定义用户标签，可直接使用标签；
 * 否则手动填写 Port / Pin。
 *============================================================*/

/* RES -> PC4 */
#define LCD_RES_GPIO_PORT       GPIOC
#define LCD_RES_GPIO_PIN        GPIO_PIN_4

/* DC  -> PC5 */
#define LCD_DC_GPIO_PORT        GPIOC
#define LCD_DC_GPIO_PIN         GPIO_PIN_5

/* CS  -> PB0 */
#define LCD_CS_GPIO_PORT        GPIOB
#define LCD_CS_GPIO_PIN         GPIO_PIN_0

/* BLK -> PB1 */
#define LCD_BLK_GPIO_PORT       GPIOB
#define LCD_BLK_GPIO_PIN        GPIO_PIN_1

/*============================================================
 * IO 操作宏（HAL 版本）
 *============================================================*/
#define LCD_RES_H()   HAL_GPIO_WritePin(LCD_RES_GPIO_PORT, LCD_RES_GPIO_PIN, GPIO_PIN_SET)
#define LCD_RES_L()   HAL_GPIO_WritePin(LCD_RES_GPIO_PORT, LCD_RES_GPIO_PIN, GPIO_PIN_RESET)

#define LCD_DC_H()    HAL_GPIO_WritePin(LCD_DC_GPIO_PORT,  LCD_DC_GPIO_PIN,  GPIO_PIN_SET)
#define LCD_DC_L()    HAL_GPIO_WritePin(LCD_DC_GPIO_PORT,  LCD_DC_GPIO_PIN,  GPIO_PIN_RESET)

#define LCD_CS_H()    HAL_GPIO_WritePin(LCD_CS_GPIO_PORT,  LCD_CS_GPIO_PIN,  GPIO_PIN_SET)
#define LCD_CS_L()    HAL_GPIO_WritePin(LCD_CS_GPIO_PORT,  LCD_CS_GPIO_PIN,  GPIO_PIN_RESET)

#define LCD_BLK_H()   HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_GPIO_PIN, GPIO_PIN_SET)
#define LCD_BLK_L()   HAL_GPIO_WritePin(LCD_BLK_GPIO_PORT, LCD_BLK_GPIO_PIN, GPIO_PIN_RESET)

/*============================================================
 * SPI 句柄（CubeMX 生成，在 spi.c 中定义）
 *============================================================*/
extern SPI_HandleTypeDef hspi1;
#define LCD_SPI_HANDLE      hspi1

/*============================================================
 * 屏幕分辨率 & 偏移
 *============================================================*/
#define LCD_WIDTH           240U
#define LCD_HEIGHT          280U

/* 部分屏幕存在行偏移，如无偏移置0；常见值：20 */
#define LCD_X_OFFSET        0U
#define LCD_Y_OFFSET        0U

/*============================================================
 * 常用颜色（RGB565）
 *============================================================*/
#define COLOR_WHITE         0xFFFFU
#define COLOR_BLACK         0x0000U
#define COLOR_RED           0xF800U
#define COLOR_GREEN         0x07E0U
#define COLOR_BLUE          0x001FU
#define COLOR_YELLOW        0xFFE0U
#define COLOR_CYAN          0x07FFU
#define COLOR_MAGENTA       0xF81FU
#define COLOR_ORANGE        0xFD20U
#define COLOR_GRAY          0x8410U

/** RGB888 转 RGB565 宏 */
#define RGB888_TO_RGB565(r, g, b) \
    ((uint16_t)(((r) & 0xF8U) << 8) | (((g) & 0xFCU) << 3) | (((b) & 0xF8U) >> 3))

/*============================================================
 * ST7789V 寄存器命令
 *============================================================*/
#define ST7789_NOP          0x00U
#define ST7789_SWRESET      0x01U
#define ST7789_SLPIN        0x10U
#define ST7789_SLPOUT       0x11U
#define ST7789_INVOFF       0x20U
#define ST7789_INVON        0x21U
#define ST7789_DISPOFF      0x28U
#define ST7789_DISPON       0x29U
#define ST7789_CASET        0x2AU
#define ST7789_RASET        0x2BU
#define ST7789_RAMWR        0x2CU
#define ST7789_MADCTL       0x36U
#define ST7789_COLMOD       0x3AU
#define ST7789_PORCTRL      0xB2U
#define ST7789_GCTRL        0xB7U
#define ST7789_VCOMS        0xBBU
#define ST7789_LCMCTRL      0xC0U
#define ST7789_VDVVRHEN     0xC2U
#define ST7789_VRHS         0xC3U
#define ST7789_VDVSET       0xC4U
#define ST7789_FRCTR2       0xC6U
#define ST7789_PWCTRL1      0xD0U
#define ST7789_PVGAMCTRL    0xE0U
#define ST7789_NVGAMCTRL    0xE1U

/*============================================================
 * 显示方向（MADCTL 参数）
 *============================================================*/
#define LCD_ROTATE_0        0x00U   /* 正常方向 */
#define LCD_ROTATE_90       0x60U   /* 顺时针 90° */
#define LCD_ROTATE_180      0xC0U   /* 顺时针 180° */
#define LCD_ROTATE_270      0xA0U   /* 顺时针 270° */

/*============================================================
 * SPI 超时（ms）
 *============================================================*/
#define LCD_SPI_TIMEOUT     100U

/*============================================================
 * 函数声明
 *============================================================*/

/* 初始化与控制 */
void     LCD_Init(void);
void     LCD_DisplayOn(void);
void     LCD_DisplayOff(void);
void     LCD_BacklightOn(void);
void     LCD_BacklightOff(void);
void     LCD_SetRotation(uint8_t rotation);
void     LCD_SleepIn(void);
void     LCD_SleepOut(void);

/* 底层写操作 */
void     LCD_WriteCmd(uint8_t cmd);
void     LCD_WriteData8(uint8_t data);
void     LCD_WriteData16(uint16_t data);
void     LCD_WriteBuf(const uint8_t *buf, uint32_t len);

/* 窗口与像素 */
void     LCD_SetWindow(uint16_t x_start, uint16_t y_start,
                       uint16_t x_end,   uint16_t y_end);
void     LCD_FillColor(uint16_t color, uint32_t count);
void     LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/* 矩形与全屏 */
void     LCD_FillRect(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h, uint16_t color);
void     LCD_FillScreen(uint16_t color);

/* 图形 */
void     LCD_DrawLine(uint16_t x0, uint16_t y0,
                      uint16_t x1, uint16_t y1, uint16_t color);
void     LCD_DrawRect(uint16_t x, uint16_t y,
                      uint16_t w, uint16_t h, uint16_t color);
void     LCD_DrawCircle(uint16_t cx, uint16_t cy,
                        uint16_t r,  uint16_t color);
void     LCD_FillCircle(uint16_t cx, uint16_t cy,
                        uint16_t r,  uint16_t color);

/* 图像 */
void     LCD_DrawImage(uint16_t x, uint16_t y,
                       uint16_t w, uint16_t h,
                       const uint16_t *image);

/* DMA 加速图像显示（需在 CubeMX 中开启 SPI1_TX DMA）*/
void     LCD_DrawImage_DMA(uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h,
                           const uint16_t *image);

#ifdef __cplusplus
}
#endif

#endif /* __ST7789V_H */
