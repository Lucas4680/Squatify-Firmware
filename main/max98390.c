// MAX98390 amplifier setup: see max98390.h.
//
// Register addresses, bit positions and value tables are taken from the
// Analog Devices MAX98390 datasheet register map.

#include "max98390.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_log.h"

#include "speaker_config.h"


// ============================================================
// REGISTERS
// ============================================================

#define REG_SOFTWARE_RESET          0x2000
#define REG_INT_STATE1              0x2005
#define REG_INT_STATE2              0x2006
#define REG_INT_STATE3              0x2007
#define REG_INT_EN1                 0x200B
#define REG_INT_EN2                 0x200C
#define REG_INT_EN3                 0x200D
#define REG_INT_CLR1                0x200E
#define REG_INT_CLR2                0x200F
#define REG_INT_CLR3                0x2010
#define REG_IRQ_CTRL                0x2011
#define REG_CLK_MON                 0x2012
#define REG_DAT_MON                 0x2014
#define REG_PCM_RX_EN_A             0x201B
#define REG_PCM_CH_SRC_1            0x2021
#define REG_PCM_MODE_CFG            0x2024
#define REG_PCM_MASTER_MODE         0x2025
#define REG_PCM_CLK_SETUP           0x2026
#define REG_PCM_SR_SETUP            0x2027
#define REG_AMP_EN                  0x203A
#define REG_MEAS_EN                 0x203F
#define REG_BOOST_CTRL0             0x2041
#define REG_BOOST_CTRL1             0x2043
#define REG_MEAS_ADC_CH0_READ       0x204D
#define REG_MEAS_ADC_CH1_READ       0x204E
#define REG_MEAS_ADC_CH2_READ       0x204F
#define REG_PWR_GATE_CTL            0x2050
#define REG_VBAT_LOWEST             0x2052
#define REG_BROWNOUT_STATUS         0x2054
#define REG_BROWNOUT_EN             0x2055
#define REG_BROWNOUT_LVL_HOLD       0x2058
#define REG_BROWNOUT_LVL1_THRESH    0x2059
#define REG_BROWNOUT_LVL2_THRESH    0x205A
#define REG_BROWNOUT_LVL3_THRESH    0x205B
#define REG_BROWNOUT_LVL4_THRESH    0x205C
#define REG_BROWNOUT_HYST           0x205D
#define REG_BROWNOUT_GAIN_ATK_REL   0x205F
#define REG_BROWNOUT_LVL1_CUR_LIMIT 0x2061      // four registers per level
#define REG_BROWNOUT_LOWEST         0x2071
#define REG_ENV_TRACK_HEADROOM      0x2076
#define REG_BOOST_BYPASS1           0x207C
#define REG_FET_SCALING3            0x2081
#define REG_DSM_EQ_BQ1_B0           0x2129      // 8 biquads x 5 coefficients x 4 bytes
#define REG_DSM_TPROT_THRESH        0x238E
#define REG_DSM_TPROT_ROOM_TEMP     0x2390
#define REG_DSM_TPROT_RECIP_RDC     0x2392
#define REG_DSM_TPROT_RECIP_TCONST  0x2395
#define REG_DSM_TPROT_ATTN          0x2398
#define REG_DSM_TPROT_PILOT         0x2399
#define REG_THERMAL_RDC_RD_HI       0x239C
#define REG_THERMAL_RDC_RD_LO       0x239D
#define REG_THERMAL_COILTEMP_RD_HI  0x239E
#define REG_THERMAL_COILTEMP_RD_LO  0x239F
#define REG_DSMIG_EN                0x23E0
#define REG_DSP_GLOBAL_EN           0x23E1
#define REG_GLOBAL_EN               0x23FF
#define REG_REV_ID                  0x24FF

// Interrupt bits worth reporting.
#define INT1_THERMSHDN_START        0x80
#define INT1_THERMWARN_START        0x20
#define INT1_BDE_L4                 0x08
#define INT1_BDE_LEVEL_CHANGE       0x04

#define INT2_BST_UVLO               0x80
#define INT2_SPK_OVC                0x40
#define INT2_PWRUP_FAIL             0x20
#define INT2_BOOSTCURRLIM           0x04
#define INT2_WATCHDOGFAIL           0x02

#define INT3_DMON_ERR               0x40

#define INT1_REPORT_MASK \
    (INT1_THERMSHDN_START | INT1_THERMWARN_START | INT1_BDE_L4 | INT1_BDE_LEVEL_CHANGE)

#define INT2_REPORT_MASK \
    (INT2_BST_UVLO | INT2_SPK_OVC | INT2_PWRUP_FAIL | INT2_BOOSTCURRLIM | INT2_WATCHDOGFAIL)

#define INT3_REPORT_MASK \
    (INT3_DMON_ERR)

#define TRY(expr)                           \
    do {                                    \
        const esp_err_t try_err_ = (expr);  \
        if (try_err_ != ESP_OK) {           \
            return try_err_;                \
        }                                   \
    } while (0)


static const char *TAG = "MAX98390";

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t dev = NULL;

static int irq_gpio = -1;


// ============================================================
// REGISTER ACCESS
// ============================================================

static esp_err_t wr(uint16_t reg, uint8_t value)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[3];

    data[0] = (uint8_t)(reg >> 8);
    data[1] = (uint8_t)(reg & 0xFF);
    data[2] = value;

    const esp_err_t err = i2c_master_transmit(dev, data, sizeof(data), 100);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: reg=0x%04X value=0x%02X: %s",
                 (unsigned)reg, (unsigned)value, esp_err_to_name(err));
    }

    return err;
}


static esp_err_t rd(uint16_t reg, uint8_t *value)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t addr[2];

    addr[0] = (uint8_t)(reg >> 8);
    addr[1] = (uint8_t)(reg & 0xFF);

    return i2c_master_transmit_receive(dev, addr, sizeof(addr), value, 1, 100);
}


// ============================================================
// VALUE CONVERSIONS (datasheet tables)
// ============================================================

// BST_VOUT: 6.5 V + 0.125 V per step, up to 10 V.
static uint8_t boost_vout_code(int mv)
{
    if (mv < 6500) {
        mv = 6500;
    }

    if (mv > 10000) {
        mv = 10000;
    }

    return (uint8_t)((mv - 6500) / 125);
}


// BST_ILIM and BDE_Ln_ILIM: 0 = 0 A, 1 = 0.5 A, 2 = 1.0 A, then 50 mA steps
// up to 64 = 4.1 A.
static uint8_t boost_ilim_code(int ma)
{
    if (ma < 1000) {
        return 1;
    }

    if (ma > 4100) {
        ma = 4100;
    }

    return (uint8_t)(2 + (ma - 1000) / 50);
}


#if MAX98390_BDE_ENABLE

// Supply thresholds use the VBAT measurement scale: 2.4 V + 12.5 mV per step.
static uint8_t vbat_code(int mv)
{
    if (mv < 2400) {
        mv = 2400;
    }

    int code = ((mv - 2400) * 2) / 25;

    if (code > 255) {
        code = 255;
    }

    return (uint8_t)code;
}


// Hold times: 0 = 0 ms, 1 = 0.1 ms, 2 = 1 ms, then 1 ms steps to 255 = 254 ms.
static uint8_t hold_code(int ms)
{
    if (ms <= 0) {
        return 0;
    }

    if (ms > 254) {
        ms = 254;
    }

    return (uint8_t)(ms + 1);
}


static int min_int(int a, int b)
{
    return (a < b) ? a : b;
}

#endif


// ============================================================
// HARDWARE RESET
// ============================================================

static esp_err_t hardware_reset(int reset_gpio)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << reset_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    TRY(gpio_config(&cfg));

    gpio_set_level(reset_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}


// ============================================================
// I2C
// ============================================================

static esp_err_t i2c_setup(const max98390_config_t *cfg)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = cfg->i2c_sda_gpio,
        .scl_io_num = cfg->i2c_scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    TRY(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->i2c_addr,
        .scl_speed_hz = 100000,
    };

    TRY(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev));

    return ESP_OK;
}


// ============================================================
// BROWNOUT ENGINE
// ============================================================

#if MAX98390_BDE_ENABLE

static esp_err_t configure_brownout(void)
{
    // Gain reduction per level, in 0.25 dB steps (datasheet BDE_Ln_AMP1_GAIN).
    static const uint8_t gain_code[4] = {
        6,      // level 1: -1.5 dB
        12,     // level 2: -3 dB
        24,     // level 3: -6 dB
        48,     // level 4: -12 dB
    };

    // Each level must not allow more current than the level above it.
    const int ilim_ma[4] = {
        min_int(MAX98390_BOOST_ILIM_MA, 1250),
        min_int(MAX98390_BOOST_ILIM_MA, 1000),
        500,
        500,
    };

    TRY(wr(REG_BROWNOUT_LVL_HOLD, hold_code(MAX98390_BDE_HOLD_MS)));

    TRY(wr(REG_BROWNOUT_LVL1_THRESH, vbat_code(MAX98390_BDE_L1_MV)));
    TRY(wr(REG_BROWNOUT_LVL2_THRESH, vbat_code(MAX98390_BDE_L2_MV)));
    TRY(wr(REG_BROWNOUT_LVL3_THRESH, vbat_code(MAX98390_BDE_L3_MV)));
    TRY(wr(REG_BROWNOUT_LVL4_THRESH, vbat_code(MAX98390_BDE_L4_MV)));

    // Hysteresis is in 12.5 mV steps.
    TRY(wr(REG_BROWNOUT_HYST, (uint8_t)((MAX98390_BDE_HYST_MV * 2) / 25)));

    // Gain moves down at 80 us/dB and back up at 82 ms/dB.
    TRY(wr(REG_BROWNOUT_GAIN_ATK_REL, 0x4B));

    for (int level = 0; level < 4; level++) {

        const uint16_t base = (uint16_t)(REG_BROWNOUT_LVL1_CUR_LIMIT + 4 * level);

        TRY(wr(base + 0, boost_ilim_code(ilim_ma[level])));     // current limit
        TRY(wr(base + 1, 0x00));                                // limiter knee: 0 dBFS (unused)
        TRY(wr(base + 2, 0x00));                                // clip level: 0 dBFS (unused)
        TRY(wr(base + 3, gain_code[level]));                    // gain reduction
    }

    // BDE_EN | BDE_AMP_EN: act on the boost current and on the audio gain.
    TRY(wr(REG_BROWNOUT_EN, 0x03));

    ESP_LOGI(TAG, "Brownout engine: %d / %d / %d / %d mV, hysteresis %d mV",
             MAX98390_BDE_L1_MV, MAX98390_BDE_L2_MV,
             MAX98390_BDE_L3_MV, MAX98390_BDE_L4_MV,
             MAX98390_BDE_HYST_MV);

    return ESP_OK;
}

#endif


// ============================================================
// EXPERIMENTAL: ON-CHIP COIL TEMPERATURE SENSING
// ============================================================

#if MAX98390_DSM_THERMAL

// Three-byte DSM values must be written low byte first.
static esp_err_t wr24(uint16_t reg_byte0, uint32_t value)
{
    TRY(wr(reg_byte0 + 0, (uint8_t)(value & 0xFF)));
    TRY(wr(reg_byte0 + 1, (uint8_t)((value >> 8) & 0xFF)));
    TRY(wr(reg_byte0 + 2, (uint8_t)((value >> 16) & 0xFF)));

    return ESP_OK;
}


static esp_err_t wr16(uint16_t reg_byte0, uint16_t value)
{
    TRY(wr(reg_byte0 + 0, (uint8_t)(value & 0xFF)));
    TRY(wr(reg_byte0 + 1, (uint8_t)(value >> 8)));

    return ESP_OK;
}


static esp_err_t configure_dsm_thermal(void)
{
    // The EQ stays disabled, but its coefficients power up undefined, so
    // load the pass-through values the datasheet gives for unused filters.
    for (int bq = 0; bq < 8; bq++) {
        for (int coef = 0; coef < 5; coef++) {

            const uint16_t reg = (uint16_t)(REG_DSM_EQ_BQ1_B0 + 20 * bq + 4 * coef);

            TRY(wr24(reg, (coef == 0) ? 0x100000 : 0x000000));
        }
    }

    // Temperatures: 16-bit fixed point <4,12>, value = degrees C / 100.
    TRY(wr16(REG_DSM_TPROT_THRESH,
             (uint16_t)lroundf(MAX98390_COIL_TEMP_LIMIT_C / 100.0f * 4096.0f)));

    TRY(wr16(REG_DSM_TPROT_ROOM_TEMP,
             (uint16_t)lroundf(MAX98390_ROOM_TEMP_C / 100.0f * 4096.0f)));

    // 1 / (Rdc x 3/11), fixed point <4,20>.
    TRY(wr24(REG_DSM_TPROT_RECIP_RDC,
             (uint32_t)lroundf(1048576.0f / (SPK_DRV_RE_OHM * 3.0f / 11.0f))));

    // 1 / (100 x temperature coefficient of copper), fixed point <4,20>.
    TRY(wr24(REG_DSM_TPROT_RECIP_TCONST,
             (uint32_t)lroundf(1048576.0f / (100.0f * 0.00393f))));

    // Skip the first 4 gain calculations, limit the attenuation to -6 dB.
    TRY(wr(REG_DSM_TPROT_ATTN, 0x08));

    // Pilot tone: 32 Hz at -40 dBFS.
    TRY(wr(REG_DSM_TPROT_PILOT, 0x00));

    // Speaker voltage and current sensing on.
    TRY(wr(REG_MEAS_EN, 0x03));

    // THERMAL_PROT_EN only. Every other DSM feature stays off.
    TRY(wr(REG_DSMIG_EN, 0x01));

    ESP_LOGW(TAG, "EXPERIMENTAL on-chip coil temperature sensing enabled");

    return ESP_OK;
}

#endif


// ============================================================
// INITIALIZATION
// ============================================================

esp_err_t max98390_init(const max98390_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    TRY(hardware_reset(cfg->reset_gpio));
    TRY(i2c_setup(cfg));

    uint8_t rev = 0;

    TRY(rd(REG_REV_ID, &rev));

    ESP_LOGI(TAG, "MAX98390 found at 0x%02X, revision 0x%02X",
             (unsigned)cfg->i2c_addr, (unsigned)rev);

    TRY(wr(REG_SOFTWARE_RESET, 0x01));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Everything off while configuring (datasheet write procedure).
    TRY(wr(REG_GLOBAL_EN, 0x00));
    vTaskDelay(pdMS_TO_TICKS(20));

    TRY(wr(REG_AMP_EN, 0x80));
    TRY(wr(REG_DSP_GLOBAL_EN, 0x00));
    vTaskDelay(pdMS_TO_TICKS(50));

    // I2S slave, 16-bit, 48 kHz, 32 bit clocks per frame, play channel 0.
    TRY(wr(REG_PCM_RX_EN_A, 0x01));
    TRY(wr(REG_PCM_MODE_CFG, 0x40));
    TRY(wr(REG_PCM_MASTER_MODE, 0x00));
    TRY(wr(REG_PCM_CLK_SETUP, 0x22));
    TRY(wr(REG_PCM_SR_SETUP, 0x08));
    TRY(wr(REG_PCM_CH_SRC_1, 0x00));

    // Clock monitor on with automatic restart, data monitor off.
    TRY(wr(REG_CLK_MON, 0x6F));
    TRY(wr(REG_DAT_MON, 0x00));

    // No automatic muting of quiet passages.
    TRY(wr(REG_PWR_GATE_CTL, 0x00));

    // Boost converter. Automatic boost bypass stays at its default: the
    // chip runs straight from 5 V until the signal needs the boost rail.
    TRY(wr(REG_BOOST_CTRL0, boost_vout_code(MAX98390_BOOST_MV)));
    TRY(wr(REG_BOOST_CTRL1, boost_ilim_code(MAX98390_BOOST_ILIM_MA)));

    // Same values the Linux kernel driver uses for these three.
    TRY(wr(REG_BOOST_BYPASS1, 0x46));
    TRY(wr(REG_ENV_TRACK_HEADROOM, 0x0E));
    TRY(wr(REG_FET_SCALING3, 0x03));

#if MAX98390_BDE_ENABLE
    TRY(configure_brownout());
#endif

    // IRQ pin: open drain, active low. Only real faults pull it low.
    TRY(wr(REG_INT_EN1, INT1_REPORT_MASK));
    TRY(wr(REG_INT_EN2, INT2_REPORT_MASK));
    TRY(wr(REG_INT_EN3, INT3_REPORT_MASK));
    TRY(wr(REG_IRQ_CTRL, 0x01));

    irq_gpio = cfg->irq_gpio;

    if (irq_gpio >= 0) {

        const gpio_config_t irq_cfg = {
            .pin_bit_mask = 1ULL << irq_gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        TRY(gpio_config(&irq_cfg));

        ESP_LOGI(TAG, "IRQ on GPIO %d", irq_gpio);
    }
    else {
        ESP_LOGI(TAG, "IRQ pin not configured, fault state is polled over I2C");
    }

#if MAX98390_DSM_THERMAL
    TRY(configure_dsm_thermal());
#endif

    ESP_LOGI(TAG, "Boost %d mV, input current limit %d mA",
             MAX98390_BOOST_MV, MAX98390_BOOST_ILIM_MA);

    return ESP_OK;
}


// ============================================================
// ENABLE
// ============================================================

static esp_err_t global_enable(void)
{
    TRY(wr(REG_GLOBAL_EN, 0x01));
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t value = 0;

    TRY(rd(REG_GLOBAL_EN, &value));

    if (value != 0x01) {
        ESP_LOGE(TAG, "GLOBAL_EN did not latch (read 0x%02X)", (unsigned)value);
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t max98390_enable(void)
{
#if MAX98390_DSM_THERMAL
    TRY(wr(REG_DSP_GLOBAL_EN, 0x01));
#endif

    TRY(wr(REG_AMP_EN, 0x81));
    vTaskDelay(pdMS_TO_TICKS(10));

    TRY(global_enable());

#if MAX98390_BDE_ENABLE
    {
        // The brownout levels assume the chip's VBAT pin sits on the 5 V USB
        // rail. If the idle reading is already at or below level 1, the
        // engine would hold the amplifier turned down forever, so switch it
        // off and say so. BDE_EN may only be cleared while EN = 0.
        uint8_t code = 0;

        vTaskDelay(pdMS_TO_TICKS(100));

        TRY(rd(REG_MEAS_ADC_CH0_READ, &code));

        const int vbat_mv = 2400 + ((int)code * 25) / 2;

        if (vbat_mv < MAX98390_BDE_L1_MV + 100) {

            ESP_LOGW(TAG, "Supply reads %d mV at idle, too close to the %d mV brownout level",
                     vbat_mv, MAX98390_BDE_L1_MV);
            ESP_LOGW(TAG, "Brownout engine DISABLED. Adjust MAX98390_BDE_Ln_MV in max98390.h");

            TRY(wr(REG_GLOBAL_EN, 0x00));
            vTaskDelay(pdMS_TO_TICKS(20));

            TRY(wr(REG_BROWNOUT_EN, 0x00));

            TRY(global_enable());
        }
        else {
            ESP_LOGI(TAG, "Supply reads %d mV at idle, brownout engine active", vbat_mv);
        }
    }
#endif

    ESP_LOGI(TAG, "Amplifier enabled");

    return ESP_OK;
}


// ============================================================
// STATUS
// ============================================================

esp_err_t max98390_read_status(max98390_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t v = 0;

    // Measurement ADC (datasheet conversion formulas).
    TRY(rd(REG_MEAS_ADC_CH0_READ, &v));
    status->vbat_v = 2.4f + 0.0125f * (float)v;

    TRY(rd(REG_VBAT_LOWEST, &v));
    status->vbat_lowest_v = 2.4f + 0.0125f * (float)v;

    TRY(rd(REG_MEAS_ADC_CH1_READ, &v));
    status->pvdd_v = 4.8f + 0.025f * (float)v;

    TRY(rd(REG_MEAS_ADC_CH2_READ, &v));
    status->die_temp_c = 1.28f * (float)v - 29.0f;

    TRY(rd(REG_BROWNOUT_STATUS, &v));
    status->bde_level = v & 0x07;

    TRY(rd(REG_BROWNOUT_LOWEST, &v));
    status->bde_lowest = v & 0x07;

    // Latched fault state. With an IRQ pin, only read it when the pin is low.
    status->int_state[0] = 0;
    status->int_state[1] = 0;
    status->int_state[2] = 0;

    if (irq_gpio < 0 || gpio_get_level(irq_gpio) == 0) {

        TRY(rd(REG_INT_STATE1, &status->int_state[0]));
        TRY(rd(REG_INT_STATE2, &status->int_state[1]));
        TRY(rd(REG_INT_STATE3, &status->int_state[2]));

        // Writing a 1 clears the matching latched bit.
        if (status->int_state[0] != 0) {
            TRY(wr(REG_INT_CLR1, status->int_state[0]));
        }

        if (status->int_state[1] != 0) {
            TRY(wr(REG_INT_CLR2, status->int_state[1]));
        }

        if (status->int_state[2] != 0) {
            TRY(wr(REG_INT_CLR3, status->int_state[2]));
        }
    }

    status->coil_valid = false;
    status->coil_rdc_ohm = 0.0f;
    status->coil_temp_c = 0.0f;

#if MAX98390_DSM_THERMAL
    {
        uint8_t hi = 0;
        uint8_t lo = 0;

        // Rdc: 14-bit fixed point <6,8>, ohms = value x 11/3.
        TRY(rd(REG_THERMAL_RDC_RD_HI, &hi));
        TRY(rd(REG_THERMAL_RDC_RD_LO, &lo));

        const uint16_t rdc_code = (uint16_t)(((uint16_t)(hi & 0x3F) << 8) | lo);

        status->coil_rdc_ohm = (float)rdc_code / 256.0f * 11.0f / 3.0f;

        // Coil temperature: signed 16-bit fixed point <4,12>, value = degrees C / 100.
        TRY(rd(REG_THERMAL_COILTEMP_RD_HI, &hi));
        TRY(rd(REG_THERMAL_COILTEMP_RD_LO, &lo));

        const int16_t temp_code = (int16_t)(((uint16_t)hi << 8) | lo);

        status->coil_temp_c = (float)temp_code / 4096.0f * 100.0f;
        status->coil_valid = true;
    }
#endif

    return ESP_OK;
}


void max98390_log_faults(const max98390_status_t *status)
{
    if (status == NULL) {
        return;
    }

    const uint8_t s1 = status->int_state[0] & INT1_REPORT_MASK;
    const uint8_t s2 = status->int_state[1] & INT2_REPORT_MASK;
    const uint8_t s3 = status->int_state[2] & INT3_REPORT_MASK;

    if (s1 & INT1_THERMSHDN_START) {
        ESP_LOGE(TAG, "FAULT: amplifier thermal shutdown");
    }

    if (s1 & INT1_THERMWARN_START) {
        ESP_LOGW(TAG, "FAULT: amplifier thermal warning");
    }

    if (s1 & INT1_BDE_L4) {
        ESP_LOGW(TAG, "FAULT: supply reached brownout level 4");
    }

    if (s1 & INT1_BDE_LEVEL_CHANGE) {
        ESP_LOGW(TAG, "Brownout level changed (now %u, worst %u)",
                 (unsigned)status->bde_level, (unsigned)status->bde_lowest);
    }

    if (s2 & INT2_BST_UVLO) {
        ESP_LOGW(TAG, "FAULT: boost undervoltage lockout");
    }

    if (s2 & INT2_SPK_OVC) {
        ESP_LOGE(TAG, "FAULT: speaker output over-current");
    }

    if (s2 & INT2_PWRUP_FAIL) {
        ESP_LOGE(TAG, "FAULT: amplifier power-up failed");
    }

    if (s2 & INT2_BOOSTCURRLIM) {
        ESP_LOGI(TAG, "Boost input current limit was reached");
    }

    if (s2 & INT2_WATCHDOGFAIL) {
        ESP_LOGW(TAG, "FAULT: watchdog failure");
    }

    if (s3 & INT3_DMON_ERR) {
        ESP_LOGW(TAG, "FAULT: data monitor error");
    }
}
