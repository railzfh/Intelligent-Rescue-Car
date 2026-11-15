/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 双M2006速度控制、电池监测与任务回调
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Battery/dvc_battery.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BSP/WS2812/bsp_ws2812.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局初始化完成标志位
bool init_finished = false;

// 右轮接CAN1、ID为1，左轮接CAN2、ID为2。
static const Enum_Motor_DJI_ID M2006_ID[2] = {Motor_DJI_ID_0x201, Motor_DJI_ID_0x202};
// 镜像安装的两侧车轮通常需要反向；若整车后退，将两个符号同时取反。
static const float M2006_Direction[2] = {1.0f, -1.0f};
static const float M2006_Forward_Rpm = 0.0f; // 原装36:1减速箱输出轴转速
static const float M2006_Acceleration_Rpm_Per_S = 50.0f; // 恢复100rpm目标时约2s完成起步
// 现有C610驱动的速度PID输出单位为输出轴Nm；0.9 Nm约对应5 A电流指令。
static const float M2006_Torque_Limit = 0.9f;
static const uint32_t M2006_Feedback_Timeout_Ms = 100;
static Class_Motor_DJI_C610 M2006[2];
static volatile uint32_t M2006_Last_Feedback_Ms[2] = {};
static volatile bool M2006_Feedback_Received[2] = {};

static Class_Battery Battery;

/* Private function declarations ---------------------------------------------*/

static void Battery_Process_Samples(uint16_t offset)
{
    // DMA写另一半缓冲区时，平均已完成的64次采样，抑制电压与灯色抖动。
    const volatile uint16_t *samples = ADC1_Manage_Object.ADC_Data;
    uint32_t sum = 0;
    for (uint16_t i = offset; i < offset + ADC_BUFFER_SIZE / 2; i++)
    {
        const uint16_t sample = samples[i];
        if (sample == 65535U)
        {
            // ADC已饱和，无法可信地判断电池电压。
            Battery.Stop();
        }
        sum += sample;
    }
    const float average = (float)sum / (ADC_BUFFER_SIZE / 2);
    Battery.Update(BSP_Power.Convert_ADC_To_Voltage(average), HAL_GetTick());
}

static void M2006_RxCallback(uint8_t motor_index, const FDCAN_RxHeaderTypeDef &header)
{
    if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
        header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8 ||
        header.Identifier != 0x200U + M2006_ID[motor_index])
    {
        return;
    }

    // 驱动直接读取各自CAN管理对象中的Rx_Buffer。
    M2006[motor_index].CAN_RxCpltCallback();
    M2006_Last_Feedback_Ms[motor_index] = HAL_GetTick();
    M2006_Feedback_Received[motor_index] = true;
}

static void M2006_CAN1_RxCallback(FDCAN_RxHeaderTypeDef &header, uint8_t *)
{
    M2006_RxCallback(0, header);
}

static void M2006_CAN2_RxCallback(FDCAN_RxHeaderTypeDef &header, uint8_t *)
{
    M2006_RxCallback(1, header);
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{

}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    TIM_1ms_IWDG_PeriodElapsedCallback();

    Battery.Check_Timeout(HAL_GetTick());
    const bool battery_ready = Battery.Can_Run();
    BSP_Power.Set_DC24_0(battery_ready);
    BSP_Power.Set_DC24_1(battery_ready);
    if (!battery_ready)
    {
        BSP_Power.Set_DC5(false);
    }

    // 电池正常且两台均有新鲜反馈才运行；任一故障，两台均停输出。
    bool motors_ready = battery_ready && M2006_Forward_Rpm > 0.0f;
    for (uint8_t i = 0; i < 2; i++)
    {
        const bool received = M2006_Feedback_Received[i];
        const uint32_t last_feedback_ms = M2006_Last_Feedback_Ms[i];
        if (!received || (uint32_t)(HAL_GetTick() - last_feedback_ms) >= M2006_Feedback_Timeout_Ms)
        {
            motors_ready = false;
        }
    }

    static float forward_rpm = 0.0f;
    if (motors_ready)
    {
        forward_rpm += M2006_Acceleration_Rpm_Per_S * 0.001f;
        if (forward_rpm > M2006_Forward_Rpm)
        {
            forward_rpm = M2006_Forward_Rpm;
        }
    }
    else
    {
        forward_rpm = 0.0f;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        if (motors_ready)
        {
            M2006[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            M2006[i].Set_Target_Omega(M2006_Direction[i] * forward_rpm * BASIC_MATH_RPM_TO_RADPS);
        }
        else
        {
            // 用零扭矩模式停输出，避免速度环在反馈丢失时继续施加电流。
            M2006[i].Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
            M2006[i].Set_Target_Omega(0.0f);
            M2006[i].Set_Target_Torque(0.0f);
            M2006[i].PID_Omega.Set_Integral_Error(0.0f);
        }
        M2006[i].TIM_Calculate_PeriodElapsedCallback();
    }

    // C610的1~4号电流命令为0x200；两条总线分别发送，未使用的槽位保持零。
    CAN_Transmit_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
    CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{

}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{

}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    M2006[0].Init(&hfdcan1, M2006_ID[0], Motor_DJI_Control_Method_OMEGA);
    M2006[1].Init(&hfdcan2, M2006_ID[1], Motor_DJI_Control_Method_OMEGA);
    for (uint8_t i = 0; i < 2; i++)
    {
        // 初始PI参数需上板调整；1ms计算周期，积分与总输出均限幅。
        M2006[i].PID_Omega.Init(0.15f, 0.30f, 0.0f, 0.0f, M2006_Torque_Limit, M2006_Torque_Limit, 0.001f);
    }
    CAN_Init(&hfdcan1, M2006_CAN1_RxCallback);
    CAN_Init(&hfdcan2, M2006_CAN2_RxCallback);
    // 电池采样有效前关闭电调供电；主控继续运行以显示低压/故障红灯。
    BSP_Power.Init(false, false);
    SPI_Init(&hspi6, nullptr);
    BSP_WS2812.Init(255, 0, 0);
    if (ADC_Init(&hadc1, ADC_BUFFER_SIZE) != HAL_OK)
    {
        Battery.Stop();
    }

    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
    static uint32_t last_led_ms = 0;
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_led_ms) < 10U)
    {
        return;
    }
    last_led_ms = now_ms;

    const uint8_t green = Battery.Get_LED_Green();
    BSP_WS2812.Set_RGB(255U - green, green, 0);
    // SPI6为阻塞发送，放在前台执行，不占用1ms电机控制中断。
    BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Battery_Process_Samples(0);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Battery_Process_Samples(ADC_BUFFER_SIZE / 2);
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Battery.Stop();
    }
}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
