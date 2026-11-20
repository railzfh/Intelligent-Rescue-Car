#include "dvc_bmi088.h"
#include "alg_bmi088.h"
#include "spi.h"
#include <math.h>
#include <stddef.h>

// 当前 ±6g、±2000°/s 配置使用的寄存器。
#define BMI088_ACC_CHIP_ID 0x00 // the register is  " Who am I "
#define BMI088_ACC_CHIP_ID_VALUE 0x1E
#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_TEMP_M 0x22
#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_CONF_MUST_Set 0x80
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
#define BMI088_INT_MAP_DATA 0x58
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_ON 0x04
#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6
#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_ON 0x80
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18
#define BMI088_GYRO_DRDY_IO_INT3 0x01

static void BMI088_Port_SelectAccel(bool selected);
static void BMI088_Port_SelectGyro(bool selected);
static bool spi_failed;

static void BMI088_Port_Init(void)
{
    spi_failed = false;
    BMI088_Port_SelectAccel(false);
    BMI088_Port_SelectGyro(false);
}

static void BMI088_Port_ResetError(void) { spi_failed = false; }
static bool BMI088_Port_HasError(void) { return spi_failed; }

static void BMI088_Port_SelectAccel(bool selected)
{
    HAL_GPIO_WritePin(BMI088_ACCEL__SPI_CS_GPIO_Port, BMI088_ACCEL__SPI_CS_Pin,
                      selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void BMI088_Port_SelectGyro(bool selected)
{
    HAL_GPIO_WritePin(BMI088_GYRO__SPI_CS_GPIO_Port, BMI088_GYRO__SPI_CS_Pin,
                      selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}


static uint8_t BMI088_Port_TransferByte(uint8_t tx)
{
    uint8_t rx = 0;
    if (spi_failed) { return 0; }
    if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 2) != HAL_OK) { spi_failed = true; }
    return rx;
}

/**
 * DWT 计时适配，基于 Wang Hongxi 的 bsp_dwt.c (V1.1.0, 2022/3/8)。
 */

static uint32_t cpu_hz;

static void BMI088_Time_Init(uint32_t cpu_mhz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55UL;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cpu_hz = cpu_mhz * 1000000U;
}

static uint32_t BMI088_Time_Now(void) { return DWT->CYCCNT; }

static float BMI088_Time_GetDelta(uint32_t *last_cycles)
{
    const uint32_t now = BMI088_Time_Now();
    const float dt = (uint32_t)(now - *last_cycles) / (float)cpu_hz;
    *last_cycles = now;
    return dt;
}

/**
 * BMI088 采集与校准，基于来源模块 BMI088driver.c (V1.2.0, 2022/3/8)。
 */

typedef struct {
    uint8_t address;
    uint8_t value;
    BMI088_Error_t error;
} BMI088_RegisterConfig_t;

static const BMI088_RegisterConfig_t accel_config[] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};

static const BMI088_RegisterConfig_t gyro_config[] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

static void Select(bool accel, bool active)
{
    if (accel) { BMI088_Port_SelectAccel(active); }
    else { BMI088_Port_SelectGyro(active); }
}

static void WriteRegister(bool accel, uint8_t address, uint8_t value)
{
    Select(accel, true);
    BMI088_Port_TransferByte(address);
    BMI088_Port_TransferByte(value);
    Select(accel, false);
}

static void ReadRegisters(bool accel, uint8_t address, uint8_t *data, unsigned size)
{
    Select(accel, true);
    BMI088_Port_TransferByte(address | 0x80U);
    // BMI088 加速度计 SPI 读取有一个 dummy 字节，陀螺仪没有。
    if (accel) { BMI088_Port_TransferByte(0x55); }
    for (unsigned i = 0; i < size; ++i) { data[i] = BMI088_Port_TransferByte(0x55); }
    Select(accel, false);
}

static uint8_t ReadRegister(bool accel, uint8_t address)
{
    uint8_t value;
    ReadRegisters(accel, address, &value, 1);
    return value;
}

static BMI088_Error_t ConfigureSensor(bool accel)
{
    const uint8_t chip_id = accel ? BMI088_ACC_CHIP_ID : BMI088_GYRO_CHIP_ID;
    ReadRegister(accel, chip_id);
    HAL_Delay(1);
    ReadRegister(accel, chip_id);
    HAL_Delay(1);
    WriteRegister(accel, accel ? BMI088_ACC_SOFTRESET : BMI088_GYRO_SOFTRESET,
                  accel ? BMI088_ACC_SOFTRESET_VALUE : BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(80);
    ReadRegister(accel, chip_id);
    HAL_Delay(1);
    const uint8_t id = ReadRegister(accel, chip_id);
    HAL_Delay(1);
    if (BMI088_Port_HasError()) { return BMI088_SPI_ERROR; }
    if (id != (accel ? BMI088_ACC_CHIP_ID_VALUE : BMI088_GYRO_CHIP_ID_VALUE)) { return BMI088_NO_SENSOR; }

    const BMI088_RegisterConfig_t *config = accel ? accel_config : gyro_config;
    const unsigned count = accel ? sizeof(accel_config) / sizeof(accel_config[0]) :
                                   sizeof(gyro_config) / sizeof(gyro_config[0]);
    for (unsigned i = 0; i < count; ++i)
    {
        WriteRegister(accel, config[i].address, config[i].value);
        HAL_Delay(1);
        const uint8_t value = ReadRegister(accel, config[i].address);
        HAL_Delay(1);
        if (BMI088_Port_HasError()) { return BMI088_SPI_ERROR; }
        if (value != config[i].value) { return config[i].error; }
    }
    return BMI088_NO_ERROR;
}

static int16_t ReadInt16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static BMI088_Error_t BMI088_Driver_Read(BMI088_Sample_t *sample, const BMI088_Calibration_t *calibration)
{
    if (!sample || !calibration) { return BMI088_UPDATE_ERROR; }
    BMI088_Port_ResetError();
    const uint8_t accel_id = ReadRegister(true, BMI088_ACC_CHIP_ID);
    if (BMI088_Port_HasError()) { return BMI088_SPI_ERROR; }
    if (accel_id != BMI088_ACC_CHIP_ID_VALUE) { return BMI088_NO_SENSOR; }

    uint8_t accel[6], gyro[8], temperature[2];
    ReadRegisters(true, BMI088_ACCEL_XOUT_L, accel, sizeof(accel));
    ReadRegisters(false, BMI088_GYRO_CHIP_ID, gyro, sizeof(gyro));
    ReadRegisters(true, BMI088_TEMP_M, temperature, sizeof(temperature));
    if (BMI088_Port_HasError()) { return BMI088_SPI_ERROR; }
    if (gyro[0] != BMI088_GYRO_CHIP_ID_VALUE) { return BMI088_NO_SENSOR; }

    BMI088_Sample_t next;
    float norm_squared = 0.0f;
    for (unsigned i = 0; i < 3; ++i)
    {
        next.accel_mps2[i] = ReadInt16(&accel[2 * i]) * BMI088_ACCEL_6G_SEN * calibration->accel_scale;
        next.gyro_radps[i] = ReadInt16(&gyro[2 + 2 * i]) * BMI088_GYRO_2000_SEN - calibration->gyro_bias_radps[i];
        norm_squared += next.accel_mps2[i] * next.accel_mps2[i];
    }
    if (!(norm_squared > 0.000001f)) { return BMI088_UPDATE_ERROR; }
    int16_t raw_temperature = (int16_t)((temperature[0] << 3) | (temperature[1] >> 5));
    if (raw_temperature > 1023) { raw_temperature -= 2048; }
    next.temperature_c = raw_temperature * 0.125f + 23.0f;
    *sample = next;
    return BMI088_NO_ERROR;
}

static BMI088_Error_t Calibrate(BMI088_Calibration_t *calibration)
{
    const uint32_t start = HAL_GetTick();
    float gyro_sum[3] = {0}, gyro_min[3] = {0}, gyro_max[3] = {0};
    float norm_sum = 0.0f, norm_min = 0.0f, norm_max = 0.0f;
    BMI088_Sample_t sample;
    for (unsigned i = 0; i < 1000U; ++i)
    {
        if ((uint32_t)(HAL_GetTick() - start) >= 3000U) { return BMI088_CALIBRATION_ERROR; }
        const BMI088_Error_t error = BMI088_Driver_Read(&sample, calibration);
        if (error != BMI088_NO_ERROR) { return error; }
        const float *accel = sample.accel_mps2;
        const float norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
        if (i == 0U)
        {
            norm_min = norm_max = norm;
            for (unsigned j = 0; j < 3; ++j) { gyro_min[j] = gyro_max[j] = sample.gyro_radps[j]; }
        }
        if (norm < norm_min) { norm_min = norm; }
        if (norm > norm_max) { norm_max = norm; }
        norm_sum += norm;
        for (unsigned j = 0; j < 3; ++j)
        {
            const float gyro = sample.gyro_radps[j];
            if (gyro < gyro_min[j]) { gyro_min[j] = gyro; }
            if (gyro > gyro_max[j]) { gyro_max[j] = gyro; }
            gyro_sum[j] += gyro;
        }
        HAL_Delay(1);
    }
    const float norm_mean = norm_sum / 1000.0f;
    if (fabsf(norm_mean - 9.81f) > 0.5f || norm_max - norm_min > 0.5f) { return BMI088_CALIBRATION_ERROR; }
    BMI088_Calibration_t next = *calibration;
    for (unsigned i = 0; i < 3; ++i)
    {
        const float offset = gyro_sum[i] / 1000.0f;
        if (fabsf(offset) > 0.05f || gyro_max[i] - gyro_min[i] > BMI088_GYRO_CALIBRATION_SPREAD_MAX_RAD_S)
        {
            return BMI088_CALIBRATION_ERROR;
        }
        next.gyro_bias_radps[i] = offset;
    }
    next.accel_scale = 9.81f / norm_mean;
    next.gravity_mps2 = norm_mean;
    next.temperature_c = sample.temperature_c;
    *calibration = next;
    return BMI088_NO_ERROR;
}

static BMI088_Error_t BMI088_Driver_Init(BMI088_Calibration_t *calibration)
{
    if (!calibration) { return BMI088_UPDATE_ERROR; }
    BMI088_Port_ResetError();
    BMI088_Error_t error = ConfigureSensor(true);
    if (error != BMI088_NO_ERROR) { return error; }
    error = ConfigureSensor(false);
    if (error != BMI088_NO_ERROR) { return error; }
    BMI088_Calibration_t next = {.accel_scale = 1.0f, .gravity_mps2 = 9.81f};
    error = Calibrate(&next);
    if (error == BMI088_NO_ERROR) { *calibration = next; }
    return error;
}

BMI088_Device_t BMI088_Device = {.status.error = BMI088_NO_SENSOR};
static uint32_t last_update_cycles;

bool BMI088_Init(void)
{
    if (BMI088_Device.status.initialized) { return true; }
    BMI088_Device.status.ready = false;
    BMI088_Port_Init();
    BMI088_Time_Init(SystemCoreClock / 1000000U);
    BMI088_Device.status.error = BMI088_Driver_Init(&BMI088_Device.calibration);
    if (BMI088_Device.status.error != BMI088_NO_ERROR) { return false; }
    if (!BMI088_Attitude_Init())
    {
        BMI088_Device.status.error = BMI088_FILTER_ERROR;
        return false;
    }
    BMI088_Device.attitude = (BMI088_Attitude_t){0};
    BMI088_Device.sample = (BMI088_Sample_t){0};
    last_update_cycles = BMI088_Time_Now();
    BMI088_Device.status.initialized = true;
    return true;
}

bool BMI088_Update(void)
{
    BMI088_Device.status.ready = false;
    if (!BMI088_Device.status.initialized) { return false; }
    const float dt = BMI088_Time_GetDelta(&last_update_cycles);
    if (!(dt > 0.0f && dt <= 0.1f))
    {
        BMI088_Device.status.error = BMI088_UPDATE_ERROR;
        return false;
    }
    BMI088_Sample_t sample;
    BMI088_Device.status.error = BMI088_Driver_Read(&sample, &BMI088_Device.calibration);
    if (BMI088_Device.status.error != BMI088_NO_ERROR) { return false; }
    BMI088_Attitude_t attitude;
    if (!BMI088_Attitude_Update(&sample, dt, &attitude))
    {
        BMI088_Device.status.error = BMI088_UPDATE_ERROR;
        return false;
    }
    BMI088_Device.sample = sample;
    BMI088_Device.attitude = attitude;
    BMI088_Device.status.ready = true;
    return true;
}
