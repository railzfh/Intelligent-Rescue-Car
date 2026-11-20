#ifndef DVC_BMI088_H
#define DVC_BMI088_H

#include <stdbool.h>
#include <stdint.h>

// 角度 °，角速度 rad/s；字段顺序对应 VOFA 通道 0～6。
typedef struct {
    float yawAngle;
    float yawTotalAngle;
    float pitchAngle;
    float rollAngle;
    float yawSpeed;
    float pitchSpeed;
    float rollSpeed;
} BMI088_Attitude_t;

typedef struct {
    float accel_mps2[3];
    float gyro_radps[3];
    float temperature_c;
} BMI088_Sample_t;

typedef struct {
    float gyro_bias_radps[3];
    float accel_scale;
    float gravity_mps2;
    float temperature_c;
} BMI088_Calibration_t;

typedef enum {
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
    BMI088_SPI_ERROR = 0x20,
    BMI088_CALIBRATION_ERROR = 0x21,
    BMI088_FILTER_ERROR = 0x22,
    BMI088_UPDATE_ERROR = 0x23,
    BMI088_NO_SENSOR = 0xFF
} BMI088_Error_t;

typedef struct {
    bool initialized;
    bool ready;
    BMI088_Error_t error;
} BMI088_Status_t;

typedef struct {
    BMI088_Attitude_t attitude;
    BMI088_Sample_t sample;
    BMI088_Calibration_t calibration;
    BMI088_Status_t status;
} BMI088_Device_t;

// ±6g 与 ±2000°/s 量程对应的单位换算。
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_GYRO_2000_SEN 0.0010652644360316953f
#define BMI088_GYRO_CALIBRATION_SPREAD_MAX_RAD_S 0.15f

#ifdef __cplusplus
extern "C" {
#endif

// 板载单个 BMI088，由前台更新；业务代码读取此对象。
extern BMI088_Device_t BMI088_Device;

// 静止初始化；成功后再次调用不会重复校准。
bool BMI088_Init(void);
// 约每 1ms 更新；成功才发布数据，失败保留上次值并清除 ready。
bool BMI088_Update(void);

#ifdef __cplusplus
}
#endif
#endif
