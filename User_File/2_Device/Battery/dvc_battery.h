#ifndef DVC_BATTERY_H
#define DVC_BATTERY_H

#include <stdint.h>

// 板载ADC仅测整包电压，单节电压是6串电池的平均值。
class Class_Battery
{
public:
    void Update(float pack_voltage, uint32_t now_ms)
    {
        Pack_Voltage = pack_voltage;
        Last_Sample_Ms = now_ms;
        Sample_Received = true;
        if (!(pack_voltage >= 6.0f * 3.3f))
        {
            Stop();
        }
    }

    void Check_Timeout(uint32_t now_ms)
    {
        if (Sample_Received && (uint32_t)(now_ms - Last_Sample_Ms) >= 100U)
        {
            Stop();
        }
    }

    void Stop()
    {
        // 低压或采样故障后不因电压回弹自动恢复，重新上电/复位才解除。
        Stop_Latched = true;
    }

    bool Can_Run() const
    {
        return Sample_Received && !Stop_Latched;
    }

    float Get_Pack_Voltage() const
    {
        return Pack_Voltage;
    }

    float Get_Cell_Voltage() const
    {
        return Pack_Voltage / 6.0f;
    }

    uint8_t Get_LED_Green() const
    {
        if (!Can_Run())
        {
            return 0;
        }
        // 3.6 V为纯红，4.2 V为纯绿；中间线性混色。
        float green = (Get_Cell_Voltage() - 3.6f) / (4.2f - 3.6f);
        if (green <= 0.0f)
        {
            return 0;
        }
        if (green >= 1.0f)
        {
            return 255;
        }
        return (uint8_t)(green * 255.0f + 0.5f);
    }

private:
    volatile float Pack_Voltage = 0.0f;
    volatile uint32_t Last_Sample_Ms = 0;
    volatile bool Sample_Received = false;
    volatile bool Stop_Latched = false;
};

#endif
