/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088 七参数读取、VOFA JustFloat 输出与看门狗维护
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#include "tsk_config_and_callback.h"

#include "2_Device/IMU/BMI088/dvc_bmi088.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/Plotter/Vofa/dvc_vofa.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "tim.h"

bool init_finished = false;
static uint32_t last_imu_retry_ms = 0;
static Class_Vofa_USB Vofa;

// 调试观察变量：角度 °，角速度 rad/s；仅在读取成功时更新。
volatile float yawAngle = 0.0f;
volatile float yawTotalAngle = 0.0f;
volatile float pitchAngle = 0.0f;
volatile float rollAngle = 0.0f;
volatile float yawSpeed = 0.0f;
volatile float pitchSpeed = 0.0f;
volatile float rollSpeed = 0.0f;

void Task_Init()
{
    // 只测试 IMU，保持受控电源关闭；先启动喂狗，再静止校准。
    BSP_Power.Init(false, false, false);
    USB_Init(nullptr);
    Vofa.Init();
    Vofa.Set_Data(7, &BMI088_Device.attitude.yawAngle, &BMI088_Device.attitude.yawTotalAngle,
                  &BMI088_Device.attitude.pitchAngle, &BMI088_Device.attitude.rollAngle,
                  &BMI088_Device.attitude.yawSpeed, &BMI088_Device.attitude.pitchSpeed, &BMI088_Device.attitude.rollSpeed);
    HAL_TIM_Base_Start_IT(&htim7);
    init_finished = true;
    BMI088_Init();
    last_imu_retry_ms = HAL_GetTick();
}

void Task_Loop()
{
    static uint32_t last_imu_ms = 0;
    static uint32_t last_usb_ms = 0;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_imu_ms) < 1U)
    {
        return;
    }
    last_imu_ms = now_ms;

    if (BMI088_Update())
    {
        yawAngle = BMI088_Device.attitude.yawAngle;
        yawTotalAngle = BMI088_Device.attitude.yawTotalAngle;
        pitchAngle = BMI088_Device.attitude.pitchAngle;
        rollAngle = BMI088_Device.attitude.rollAngle;
        yawSpeed = BMI088_Device.attitude.yawSpeed;
        pitchSpeed = BMI088_Device.attitude.pitchSpeed;
        rollSpeed = BMI088_Device.attitude.rollSpeed;
        // 每 10ms 发送 7 个 float + JustFloat 帧尾 00 00 80 7F。
        if ((uint32_t)(now_ms - last_usb_ms) >= 10U)
        {
            last_usb_ms = now_ms;
            Vofa.TIM_1ms_Write_PeriodElapsedCallback();
        }
    }

    // 校准失败后留出静止时间再重试。
    if (!BMI088_Device.status.initialized && (uint32_t)(now_ms - last_imu_retry_ms) >= 3000U)
    {
        BMI088_Init();
        last_imu_retry_ms = HAL_GetTick();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (init_finished && htim->Instance == TIM7)
    {
        TIM_1ms_IWDG_PeriodElapsedCallback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
