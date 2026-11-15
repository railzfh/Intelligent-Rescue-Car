#include "2_Device/Battery/dvc_battery.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <limits>

int main()
{
    Class_Battery pending;
    pending.Check_Timeout(1000);
    assert(!pending.Can_Run());
    assert(pending.Get_LED_Green() == 0);
    pending.Update(25.2f, 1000);
    assert(pending.Can_Run());
    assert(fabsf(pending.Get_Cell_Voltage() - 4.2f) < 0.0001f);
    assert(pending.Get_LED_Green() == 255);

    // 整个显示范围应连续单调；超出范围时颜色限幅。
    Class_Battery display;
    int previous_green = -1;
    for (int millivolts = 21600; millivolts <= 25200; millivolts++)
    {
        display.Update(millivolts / 1000.0f, 0);
        assert(display.Can_Run());
        const int green = display.Get_LED_Green();
        assert(green >= previous_green);
        previous_green = green;
    }
    display.Update(21.6f, 0);
    assert(display.Get_LED_Green() == 0);
    display.Update(23.4f, 0);
    assert(display.Get_LED_Green() >= 127 && display.Get_LED_Green() <= 128);
    display.Update(26.0f, 0);
    assert(display.Get_LED_Green() == 255);
    display.Update(21.59f, 0);
    assert(display.Can_Run() && display.Get_LED_Green() == 0);

    // “低于3.3V”才停机；回弹不能解除停机。
    Class_Battery cutoff;
    cutoff.Update(19.8f, 0);
    assert(cutoff.Can_Run());
    cutoff.Update(19.799f, 1);
    assert(!cutoff.Can_Run());
    cutoff.Update(25.2f, 2);
    assert(!cutoff.Can_Run() && cutoff.Get_LED_Green() == 0);

    Class_Battery timeout;
    timeout.Update(24.0f, 1000);
    timeout.Check_Timeout(1099);
    assert(timeout.Can_Run());
    timeout.Check_Timeout(1100);
    assert(!timeout.Can_Run());
    timeout.Update(24.0f, 1101);
    assert(!timeout.Can_Run());

    // HAL_GetTick回绕后仍按100ms判断，不误判为立即超时。
    Class_Battery wraparound;
    wraparound.Update(24.0f, UINT32_MAX - 49);
    wraparound.Check_Timeout(49);
    assert(wraparound.Can_Run());
    wraparound.Check_Timeout(50);
    assert(!wraparound.Can_Run());

    Class_Battery hardware_error;
    hardware_error.Stop();
    hardware_error.Update(24.0f, 0);
    assert(!hardware_error.Can_Run());

    const float invalid_voltages[] = {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()};
    for (float invalid : invalid_voltages)
    {
        Class_Battery battery;
        battery.Update(invalid, 0);
        assert(!battery.Can_Run() && battery.Get_LED_Green() == 0);
    }

    puts("PASS: 6S voltage, color sweep, cutoff boundary/latch, ADC timeout/error, tick wraparound");
}
