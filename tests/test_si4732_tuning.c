#include "si4732.h"
#include "hardware/i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

i2c_inst_t test_i2c;
uint64_t test_time_us;
static uint8_t last_command;
static uint16_t commanded_khz;
static int16_t commanded_bfo;

int i2c_write_blocking(i2c_inst_t *bus, uint8_t address, const uint8_t *data,
                       size_t count, bool nostop)
{
    (void)bus; (void)address; (void)nostop;
    if (count) last_command = data[0];
    if (count == 6 && data[0] == 0x40)
        commanded_khz = (uint16_t)((data[2] << 8) | data[3]);
    if (count == 6 && data[0] == 0x12 && data[2] == 1 && data[3] == 0)
        commanded_bfo = (int16_t)((data[4] << 8) | data[5]);
    return (int)count;
}

int i2c_read_blocking(i2c_inst_t *bus, uint8_t address, uint8_t *data,
                      size_t count, bool nostop)
{
    (void)bus; (void)address; (void)nostop;
    memset(data, 0, count);
    if (count) data[0] = 0x81; /* CTS + tune complete */
    if (count == 9 && last_command == 0x10) {
        const uint8_t rev[] = {0x81, 0x20, '1', '0', 0x9d, 0x29, '6', '0', 0x41};
        memcpy(data, rev, sizeof rev);
    }
    if (count == 8 && last_command == 0x42) {
        data[2] = (uint8_t)(commanded_khz >> 8);
        data[3] = (uint8_t)commanded_khz;
    }
    return (int)count;
}

static void expect_plan(uint32_t dial, int32_t calibration, uint32_t khz,
                        int16_t bfo, int32_t configured_if)
{
    assert(si4732_get_hz() == dial);
    assert(si4732_get_cal_hz() == calibration);
    assert(si4732_get_khz() == khz);
    assert(si4732_get_bfo_hz() == bfo);
    assert(si4732_get_configured_if_hz() == configured_if);
    assert(si4732_get_actual_hz() == configured_if - calibration);
    assert(!si4732_bfo_overridden());
}

int main(void)
{
    /* Empty flash: first startup must apply the default to PLL AND BFO. */
    assert(si4732_init());
    expect_plan(19500000, -4300, 19496, 300, 19495700);
    assert(!si4732_is_on());
    assert(si4732_set_mode(SI4732_MODE_USB));
    assert(commanded_khz == 19496);
    assert(commanded_bfo == 300);
    si4732_power_down();

    /* Valid saved zero overrides the build default, including during init. */
    si4732_restore(SI4732_MODE_USB, 19359800, 50, 2200, 0);
    assert(si4732_init());
    expect_plan(19359800, 0, 19360, 200, 19359800);

    /* The two documented calibration workflows must address the same IF. */
    si4732_restore(SI4732_MODE_USB, 19403000, 50, 2200, -4300);
    expect_plan(19403000, -4300, 19399, 300, 19398700);
    const int32_t calibrated_if = si4732_get_configured_if_hz();
    si4732_restore(SI4732_MODE_USB, 19398700, 50, 2200, 0);
    assert(si4732_get_configured_if_hz() == calibrated_if);

    /* Invalid saved calibration falls back, a valid custom offset survives. */
    si4732_restore(SI4732_MODE_USB, 19500000, 50, 2200, 200001);
    expect_plan(19500000, SI4732_DEFAULT_CAL_HZ, 19496, 300, 19495700);
    si4732_restore(SI4732_MODE_USB, 19359800, 50, 2200, 1234);
    expect_plan(19359800, 1234, 19361, -34, 19361034);
    assert(si4732_init());
    expect_plan(19359800, 1234, 19361, -34, 19361034);

    /* Normal tuning reaches the wire with the same sign as the restored plan. */
    si4732_restore(SI4732_MODE_USB, 19359800, 50, 2200, 0);
    assert(si4732_set_mode(SI4732_MODE_USB));
    const uint32_t dial[] = {19359800, 19360200, 19360499, 19360500, 19360501};
    const uint32_t pll[] = {19360, 19360, 19360, 19361, 19361};
    const int16_t bfo[] = {200, -200, -499, 500, 499};
    for (size_t i = 0; i < sizeof dial / sizeof dial[0]; ++i) {
        assert(si4732_tune_hz(dial[i]));
        expect_plan(dial[i], 0, pll[i], bfo[i], (int32_t)dial[i]);
        assert(commanded_khz == pll[i]);
        assert(commanded_bfo == bfo[i]);
    }

    /* A raw BFO changes effective tuning; the next frequency command clears it. */
    assert(si4732_tune_hz(19360200));
    assert(si4732_set_bfo_hz(500));
    assert(si4732_get_hz() == 19360200);
    assert(si4732_get_configured_if_hz() == 19359500);
    assert(si4732_bfo_overridden());
    assert(si4732_tune_hz(19360200));
    expect_plan(19360200, 0, 19360, -200, 19360200);

    /* Save/load reconstruction must keep calibrated sub-kHz tuning unchanged. */
    assert(si4732_set_cal_hz(-4300));
    assert(si4732_tune_hz(19403000));
    si4732_power_down();
    si4732_restore(SI4732_MODE_USB, 19403000, 50, 2200, -4300);
    assert(si4732_init());
    expect_plan(19403000, -4300, 19399, 300, 19398700);

    /* Rejected tunes/calibrations must not silently modify the previous plan. */
    assert(si4732_set_cal_hz(0));
    assert(si4732_tune_hz(2300000));
    assert(!si4732_set_cal_hz(-4300));
    expect_plan(2300000, 0, 2300, 0, 2300000);
    assert(!si4732_tune_hz(2299999));
    expect_plan(2300000, 0, 2300, 0, 2300000);
    assert(si4732_tune_hz(26100000));
    assert(!si4732_set_cal_hz(1));
    assert(!si4732_tune_hz(26100001));
    expect_plan(26100000, 0, 26100, 0, 26100000);

    /* AM has no BFO fine tuning; report its whole-kHz configured IF honestly. */
    si4732_restore(SI4732_MODE_AM, 19359800, 50, 4000, 0);
    expect_plan(19359800, 0, 19360, 200, 19360000);
    assert(si4732_set_bfo_hz(500));
    assert(si4732_get_configured_if_hz() == 19360000);
    assert(!si4732_bfo_overridden());

    puts("Si4732 tuning regression tests passed");
    return 0;
}
