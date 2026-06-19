#ifndef __DEBUG_RTT_H
#define __DEBUG_RTT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 ******************************************************************************
 * @file    debug_rtt.h
 * @brief   SEGGER J-Link RTT 调试输出封装
 *
 * 使用前准备：
 *   1. 从 J-Link 安装目录复制以下文件到工程：
 *        SEGGER_RTT.c
 *        SEGGER_RTT.h
 *        SEGGER_RTT_Conf.h
 *      路径通常在：
 *        C:\Program Files (x86)\SEGGER\JLink_VXXX\Samples\RTT\SEGGER_RTT_V*.*.*\RTT
 *
 *   2. 将 SEGGER_RTT.c 加入 Keil 工程编译列表
 *   3. 将 SEGGER_RTT.h / SEGGER_RTT_Conf.h 所在目录加入
 *      Include Paths（Options for Target → C/C++ → Include Paths）
 *
 *   4. 调试时：
 *        - J-Link 连接好后，运行程序（无需断点）
 *        - 打开 J-Link RTT Viewer（J-Link 安装目录下）
 *        - 或在 Keil 中：View → Serial Windows → Debug (printf) Viewer
 *          （需在 Debug 设置中勾选 "Use Trace" 或确认 RTT Viewer 单独连接）
 *
 * 启用/关闭调试输出：
 *   全局开关在本文件下方 EC11_DEBUG_EN 宏，
 *   设为 0 可一键关闭所有调试打印，不影响发布版本性能。
 ******************************************************************************
 */

/* ============================================================
 * 调试开关（0=关闭所有 DEBUG_PRINTF，1=开启）
 * ============================================================ */
#define EC11_DEBUG_EN   1

#if EC11_DEBUG_EN
    #include "SEGGER_RTT.h"
    #define DEBUG_PRINTF(...)   SEGGER_RTT_printf(0, __VA_ARGS__)
    #define DEBUG_INIT()        SEGGER_RTT_Init()
#else
    #define DEBUG_PRINTF(...)   do {} while (0)
    #define DEBUG_INIT()        do {} while (0)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __DEBUG_RTT_H */
