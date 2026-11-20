// 主机测试：HAL/DWT 寄存器模拟，采集、计时和 EKF 使用实际模块代码。
#include "dvc_bmi088.h"
#include "alg_bmi088.h"
#include "spi.h"
#include <math.h>
#include <string.h>

#define CHECK(value) do { if (!(value)) { return __LINE__; } } while (0)

// 模拟芯片寄存器地址，不依赖驱动内部的寄存器定义。
enum {
    BMI088_ACC_CHIP_ID = 0x00, BMI088_ACC_CHIP_ID_VALUE = 0x1e,
    BMI088_GYRO_CHIP_ID = 0x00, BMI088_GYRO_CHIP_ID_VALUE = 0x0f,
    BMI088_ACCEL_XOUT_L = 0x12, BMI088_GYRO_RANGE = 0x0f
};

SPI_HandleTypeDef hspi2;
GPIO_TypeDef mock_gpio;
Mock_DWT_Type mock_dwt;
Mock_CoreDebug_Type mock_core_debug;
uint32_t SystemCoreClock = 480000000U;
static uint32_t tick, transfers;
static uint32_t fail_after_transfer;
static uint8_t accel[256], gyro[256], selected, address, position, reading;
static bool missing, spi_timeout, bad_register, moving;
static int16_t gx = 8, gy = -5, gz = 10;
static int16_t noise_counts;
static float ax, ay, az = 9.81f, sample_dt = 0.001f;

static void Put16(uint8_t *buffer, unsigned index, int16_t value)
{
    buffer[index] = (uint8_t)value;
    buffer[index + 1] = (uint8_t)((uint16_t)value >> 8);
}

static void Samples(void)
{
    Put16(accel, BMI088_ACCEL_XOUT_L, (int16_t)(ax / BMI088_ACCEL_6G_SEN));
    Put16(accel, BMI088_ACCEL_XOUT_L + 2, (int16_t)(ay / BMI088_ACCEL_6G_SEN));
    Put16(accel, BMI088_ACCEL_XOUT_L + 4, (int16_t)(az / BMI088_ACCEL_6G_SEN));
    const int16_t noise = (tick & 1U) ? noise_counts : -noise_counts;
    Put16(gyro, 2, gx + noise + (moving ? ((tick & 1U) ? 100 : -100) : 0));
    Put16(gyro, 4, gy + noise);
    Put16(gyro, 6, gz + noise);
}

static void Reset_Mock(void)
{
    memset(accel, 0, sizeof(accel));
    memset(gyro, 0, sizeof(gyro));
    accel[BMI088_ACC_CHIP_ID] = BMI088_ACC_CHIP_ID_VALUE;
    gyro[BMI088_GYRO_CHIP_ID] = BMI088_GYRO_CHIP_ID_VALUE;
    tick = transfers = 0;
    missing = spi_timeout = bad_register = moving = false;
    noise_counts = 0;
    fail_after_transfer = 0;
    selected = 0;
}

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; mock_dwt.CYCCNT += ms * (SystemCoreClock / 1000U); }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port;
    selected = state == GPIO_PIN_RESET ? (pin == BMI088_ACCEL__SPI_CS_Pin ? 1 : 2) : 0;
    position = 0;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx,
                                        uint16_t length, uint32_t timeout)
{
    (void)spi;
    ++transfers;
    if (length != 1 || selected == 0) { return HAL_ERROR; }
    if (spi_timeout || (fail_after_transfer && transfers >= fail_after_transfer))
    {
        tick += timeout;
        return HAL_TIMEOUT;
    }
    *rx = 0;
    uint8_t *registers = selected == 1 ? accel : gyro;
    if (position == 0)
    {
        Samples();
        address = *tx & 0x7fU;
        reading = *tx & 0x80U;
    }
    else if (!reading)
    {
        registers[address++] = *tx;
    }
    else if (!(selected == 1 && position == 1))
    {
        *rx = missing ? 0 : registers[address];
        if (selected == 2 && address == BMI088_GYRO_RANGE && bad_register) { *rx = 0xff; }
        ++address;
    }
    ++position;
    return HAL_OK;
}

float arm_sin_f32(float angle) { return sinf(angle); }
float arm_cos_f32(float angle) { return cosf(angle); }

static bool Run(BMI088_Attitude_t *imu)
{
    ++tick;
    mock_dwt.CYCCNT += (uint32_t)(sample_dt * SystemCoreClock);
    if (!BMI088_Update()) { return false; }
    *imu = BMI088_Device.attitude;
    return true;
}

int main(void)
{
    BMI088_Attitude_t imu = {0};
    CHECK(!BMI088_Update() && !BMI088_Device.status.ready);

    Reset_Mock(); missing = true;
    CHECK(!BMI088_Init());
    CHECK(transfers < 100 && tick < 500 && !BMI088_Device.status.ready);

    Reset_Mock(); bad_register = true;
    CHECK(!BMI088_Init());
    CHECK(BMI088_Device.status.error == BMI088_GYRO_RANGE_ERROR);
    CHECK(transfers < 200 && tick < 500);

    Reset_Mock(); spi_timeout = true;
    CHECK(!BMI088_Init());
    CHECK(BMI088_Device.status.error == BMI088_SPI_ERROR && transfers == 1);

    Reset_Mock(); moving = true;
    CHECK(!BMI088_Init());
    CHECK(BMI088_Device.status.error == BMI088_CALIBRATION_ERROR && tick < 3500);

    // 匀速旋转与异常重力仍须拒绝，不能仅凭峰峰值小就通过。
    Reset_Mock(); gx = 80;
    CHECK(!BMI088_Init() && BMI088_Device.status.error == BMI088_CALIBRATION_ERROR);
    gx = 8;
    Reset_Mock(); az = 11.0f;
    CHECK(!BMI088_Init() && BMI088_Device.status.error == BMI088_CALIBRATION_ERROR);
    az = 9.81f;

    // 板上实测静止峰峰值约 0.06~0.08 rad/s，模拟 0.068 rad/s 噪声。
    Reset_Mock();
    noise_counts = 32;
    CHECK(BMI088_Init());
    noise_counts = 0;
    CHECK(mock_dwt.LAR == 0xc5acce55U && (mock_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk));
    CHECK((mock_core_debug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && BMI088_Device.status.error == BMI088_NO_ERROR);
    CHECK(tick >= 1000 && tick < 3500);
    CHECK(BMI088_Device.status.initialized && !BMI088_Device.status.ready);
    CHECK(fabsf(BMI088_Device.calibration.gyro_bias_radps[0] - 8 * BMI088_GYRO_2000_SEN) < 0.00001f);
    const uint32_t initialized_transfers = transfers;
    CHECK(BMI088_Init() && transfers == initialized_transfers);
    for (unsigned i = 0; i < 100; ++i) { CHECK(Run(&imu)); }
    CHECK(fabsf(imu.pitchAngle) < 0.5f && fabsf(imu.rollAngle) < 0.5f);
    CHECK(fabsf(imu.yawSpeed) < 0.001f && fabsf(imu.pitchSpeed) < 0.001f && fabsf(imu.rollSpeed) < 0.001f);

    // 正向跨一圈，再反向跨两圈；累计 yaw 应连续，角速度保持 rad/s。
    gz = 10 + (int16_t)(1.0f / BMI088_GYRO_2000_SEN);
    for (unsigned i = 0; i < 7000; ++i) { CHECK(Run(&imu)); }
    CHECK(imu.yawTotalAngle > 390.0f && imu.yawTotalAngle < 410.0f);
    CHECK(imu.yawAngle >= -180.0f && imu.yawAngle <= 180.0f);
    CHECK(fabsf(imu.yawSpeed - 1.0f) < 0.01f);
    gz = 10 - (int16_t)(1.0f / BMI088_GYRO_2000_SEN);
    for (unsigned i = 0; i < 14000; ++i) { CHECK(Run(&imu)); }
    CHECK(imu.yawTotalAngle < -390.0f && imu.yawTotalAngle > -410.0f);

    BMI088_Attitude_t previous = imu;
    BMI088_Sample_t previous_sample = BMI088_Device.sample;
    missing = true;
    CHECK(!Run(&imu) && !BMI088_Device.status.ready);
    CHECK(memcmp(&previous, &imu, sizeof(imu)) == 0);
    CHECK(memcmp(&previous_sample, &BMI088_Device.sample, sizeof(previous_sample)) == 0);
    missing = false; spi_timeout = true;
    uint32_t before = transfers;
    CHECK(!Run(&imu) && BMI088_Device.status.error == BMI088_SPI_ERROR);
    CHECK(transfers == before + 1 && memcmp(&previous, &imu, sizeof(imu)) == 0);
    spi_timeout = false;
    CHECK(Run(&imu) && BMI088_Device.status.ready);

    // 采集到一半断线时，对外采样和姿态仍必须是上一整帧。
    previous = BMI088_Device.attitude;
    previous_sample = BMI088_Device.sample;
    fail_after_transfer = transfers + 12U;
    CHECK(!Run(&imu) && BMI088_Device.status.error == BMI088_SPI_ERROR);
    CHECK(memcmp(&previous, &BMI088_Device.attitude, sizeof(previous)) == 0);
    CHECK(memcmp(&previous_sample, &BMI088_Device.sample, sizeof(previous_sample)) == 0);
    fail_after_transfer = 0;
    CHECK(Run(&imu));

    sample_dt = 0.2f;
    previous = imu;
    CHECK(!Run(&imu) && memcmp(&previous, &imu, sizeof(imu)) == 0);
    CHECK(BMI088_Device.status.error == BMI088_UPDATE_ERROR);
    sample_dt = 0.001f;

    // 同步模拟陀螺旋转和重力方向变化，验证 pitch/roll 与板体 Y/X 轴的映射。
    gz = 10;
    gy = -5 + (int16_t)(0.34906585f / BMI088_GYRO_2000_SEN);
    for (unsigned i = 1; i <= 1000; ++i)
    {
        float angle = 0.34906585f * i / 1000.0f;
        ax = -9.81f * sinf(angle); az = 9.81f * cosf(angle);
        CHECK(Run(&imu));
    }
    gy = -5;
    for (unsigned i = 0; i < 100; ++i) { CHECK(Run(&imu)); }
    CHECK(fabsf(imu.pitchAngle - 20.0f) < 2.0f && fabsf(imu.rollAngle) < 2.0f);
    gy = -5 - (int16_t)(0.34906585f / BMI088_GYRO_2000_SEN);
    for (unsigned i = 1; i <= 1000; ++i)
    {
        float angle = 0.34906585f * (1000 - i) / 1000.0f;
        ax = -9.81f * sinf(angle); az = 9.81f * cosf(angle);
        CHECK(Run(&imu));
    }
    gy = -5;
    gx = 8 + (int16_t)(0.52359878f / BMI088_GYRO_2000_SEN);
    for (unsigned i = 1; i <= 1000; ++i)
    {
        float angle = 0.52359878f * i / 1000.0f;
        ay = 9.81f * sinf(angle); az = 9.81f * cosf(angle);
        CHECK(Run(&imu));
    }
    gx = 8;
    for (unsigned i = 0; i < 100; ++i) { CHECK(Run(&imu)); }
    CHECK(fabsf(imu.rollAngle - 30.0f) < 2.0f && fabsf(imu.pitchAngle) < 2.0f);
    CHECK(!BMI088_Attitude_Update(NULL, 0.001f, &imu));
    return 0;
}
