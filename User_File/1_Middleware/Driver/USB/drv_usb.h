/**
 * @file drv_usb.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的USB通信初始化与配置流程
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/**
 * usbd_cdc_if.c 通过 drv_usb_callback.h 调用接收回调。
 * 双缓冲切换和下一包接收由 drv_usb.cpp 中的 USB_ReceiveCallback 处理。
 */

#ifndef DRV_USB_H
#define DRV_USB_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "drv_usb_callback.h"
#include "usbd_cdc_if.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define USB_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*USB_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief USB通信处理结构体
 */
struct Struct_USB_Manage_Object
{
    USB_Callback Callback_Function;

    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[USB_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;
    uint8_t *Rx_Buffer_Ready;

    // 接收时间戳
    uint64_t Rx_Time_Stamp;
};

/* Exported variables --------------------------------------------------------*/

extern bool init_finished;

extern struct Struct_USB_Manage_Object USB0_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void USB_Init(USB_Callback Callback_Function);

uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
