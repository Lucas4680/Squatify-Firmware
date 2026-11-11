// MAX98390 amplifier setup for USB-powered use.
//
// Only register fields that the Analog Devices datasheet documents are
// written here. The speaker-specific processing is done on the ESP32 in
// speaker_dsp.c; the chip is used for what only it can do:
//
//   - boost converter with an input current limit
//   - brownout engine that backs off when the USB supply sags
//   - die temperature, over-current and clock fault reporting
//   - supply voltage readback for tuning

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"


// ============================================================
// BOOST CONVERTER
// ============================================================
//
// Boost output voltage, 6500 to 10000 mV in 125 mV steps. 6.5 V is plenty
// for a 4 ohm driver on a USB budget. If this changes, SPK_OUT_CEILING_V in
// speaker_config.h must stay about 0.5 V below it.

#define MAX98390_BOOST_MV               6500

// Peak current the boost converter may draw from the 5 V supply.
// 500, or 1000 to 4100 in 50 mA steps. Lower is kinder to the USB port but
// lets the boost rail droop on loud bass peaks.

#define MAX98390_BOOST_ILIM_MA          1500


// ============================================================
// BROWNOUT ENGINE
// ============================================================
//
// The chip measures the 5 V supply (its VBAT pin). When the supply sags
// below a level, the chip lowers its own gain and boost current until the
// supply recovers. This protects the ESP32 from resetting.
//
//   level    supply below    gain       boost current limit
//   1        4.70 V          -1.5 dB    up to 1.25 A
//   2        4.60 V          -3 dB      up to 1.0 A
//   3        4.50 V          -6 dB      0.5 A
//   4        4.35 V          -12 dB     0.5 A

#define MAX98390_BDE_ENABLE             1

#define MAX98390_BDE_L1_MV              4700
#define MAX98390_BDE_L2_MV              4600
#define MAX98390_BDE_L3_MV              4500
#define MAX98390_BDE_L4_MV              4350
#define MAX98390_BDE_HYST_MV            50
#define MAX98390_BDE_HOLD_MS            200


// ============================================================
// EXPERIMENTAL: ON-CHIP COIL TEMPERATURE SENSING
// ============================================================
//
// Off by default. When set to 1, the chip's DSM path is switched on with
// only its thermal protection enabled, and the status log shows the voice
// coil resistance and temperature the chip measures.
//
// Why it is off: the datasheet documents the thermal registers written
// here, but the chip also uses an internal filter that only Analog's DSM
// Sound Studio tool sets up. If the reported coil resistance is not close
// to the driver's 3.7 ohm, the readings cannot be trusted and this should
// be turned off again. It is not needed for protection on USB power: the
// power budget in speaker_config.h is far below the driver's 10 W rating.

#define MAX98390_DSM_THERMAL            0
#define MAX98390_COIL_TEMP_LIMIT_C      100.0f
#define MAX98390_ROOM_TEMP_C            25.0f


// ============================================================
// API
// ============================================================

typedef struct {
    int i2c_sda_gpio;
    int i2c_scl_gpio;
    int reset_gpio;
    int irq_gpio;           // GPIO wired to the IRQ pin, or -1 if unknown
    uint8_t i2c_addr;
} max98390_config_t;


typedef struct {
    float vbat_v;           // 5 V supply as the chip sees it, right now
    float vbat_lowest_v;    // lowest supply voltage since the last read
    float pvdd_v;           // boost rail
    float die_temp_c;       // amplifier die temperature
    uint8_t bde_level;      // brownout level now (0 = normal)
    uint8_t bde_lowest;     // worst brownout level since the last read
    uint8_t int_state[3];   // latched interrupt state registers
    bool coil_valid;        // true when MAX98390_DSM_THERMAL is on
    float coil_rdc_ohm;
    float coil_temp_c;
} max98390_status_t;


// Reset the chip, bring up I2C and write the full configuration.
// The amplifier stays off until max98390_enable().
esp_err_t max98390_init(const max98390_config_t *cfg);

// Turn the amplifier on. Call after the I2S clocks are running.
esp_err_t max98390_enable(void);

// Read supply, temperature and fault state, and clear latched faults.
esp_err_t max98390_read_status(max98390_status_t *status);

// Log a line for every fault bit set in a status read.
void max98390_log_faults(const max98390_status_t *status);
