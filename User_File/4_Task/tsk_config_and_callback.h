/**
 * @file tsk_config_and_callback.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

// 主任务读取值：角度 °，角速度 rad/s；有效性见 BMI088_Device.status.ready。
extern volatile float yawAngle;
extern volatile float yawTotalAngle;
extern volatile float pitchAngle;
extern volatile float rollAngle;
extern volatile float yawSpeed;
extern volatile float pitchSpeed;
extern volatile float rollSpeed;

/* Exported function declarations --------------------------------------------*/

void Task_Init();

void Task_Loop();

#ifdef __cplusplus
};
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
