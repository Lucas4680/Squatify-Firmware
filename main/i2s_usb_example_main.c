#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

#include "usb_device_uac.h"


// ============================================================
// HARDWARE
// ============================================================

#define MAX98390_I2C_ADDR       0x38

#define I2C_SDA_GPIO            15
#define I2C_SCL_GPIO            35

#define MAX98390_RESET_GPIO     37

#define I2S_DOUT_GPIO           16
#define I2S_BCLK_GPIO           17
#define I2S_WS_GPIO             18

#define I2S_PORT                I2S_NUM_0

#define STATUS_LED_GPIO         48


// ============================================================
// AUDIO
// ============================================================

#define SAMPLE_RATE             48000

#define AUDIO_BITS              16
#define AUDIO_CHANNELS          2

#define AUDIO_FRAME_BYTES       4


// ============================================================
// HARD LOW-FREQUENCY CUTOFF
// ============================================================

#define HIGH_PASS_CUTOFF_HZ     30.0f


// ============================================================
// LOW-FREQUENCY BOOST
// ============================================================

#define LOW_FREQ_CUTOFF_HZ      180.0f
#define LOW_FREQ_BOOST_DB       15.0f
#define LOW_FREQ_SLOPE          1.0f


// ============================================================
// HARD LIMITER
// ============================================================

#define LIMITER_CEILING_DB      5.0f


// ============================================================
// VOLUME CONTROL
// ============================================================
//
// USB volume is treated as 0..100.
//
// 0%:
//
//     -60 dB
//
// 100%:
//
//     +5 dB
//
// The mapping is linear in dB, not amplitude.
//
// This is much more appropriate for a perceived-volume
// control than:
//
//     gain = volume / 100
//
// which makes the lower part of the control feel excessively
// compressed.
//
// The final +5 dB maximum is intentional.
//

#define VOLUME_MIN_DB           -60.0f
#define VOLUME_MAX_DB           5.0f

#define VOLUME_MAX_GAIN         1.77827941f
#define VOLUME_MIN_GAIN         0.001f


// ============================================================
// VOLUME RAMP
// ============================================================
//
// Volume changes are never applied immediately.
//
// 30 ms gives a smooth transition while remaining responsive.
//

#define VOLUME_RAMP_MS          30.0f

#define VOLUME_RAMP_SAMPLES \
    ((uint32_t)((SAMPLE_RATE * VOLUME_RAMP_MS) / 1000.0f))


// ============================================================
// AUDIO QUEUE
// ============================================================

#define AUDIO_BLOCK_SIZE        1024
#define AUDIO_QUEUE_LENGTH      24


// ============================================================
// TASK CONFIGURATION
// ============================================================

#define AUDIO_I2S_TASK_STACK    4096
#define AUDIO_I2S_TASK_PRIORITY 24

#define MONITOR_TASK_STACK      3072
#define MONITOR_TASK_PRIORITY   5

#define STATUS_TASK_STACK       2048
#define STATUS_TASK_PRIORITY    1


// ============================================================
// TAG
// ============================================================

static const char *TAG =
    "MAX98390_USB_AUDIO";


// ============================================================
// GLOBAL HANDLES
// ============================================================

static i2c_master_bus_handle_t i2c_bus = NULL;

static i2c_master_dev_handle_t max98390 = NULL;

static i2s_chan_handle_t i2s_tx = NULL;

static QueueHandle_t audio_queue = NULL;


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

static void audio_i2s_task(void *arg);

static void audio_monitor_task(void *arg);

static void status_led_task(void *arg);


// ============================================================
// AUDIO BLOCK
// ============================================================

typedef struct
{
    size_t len;

    uint8_t data[AUDIO_BLOCK_SIZE];

} audio_block_t;


// ============================================================
// AUDIO STATISTICS
// ============================================================

static volatile uint32_t audio_packets_received = 0;

static volatile uint32_t audio_bytes_received = 0;

static volatile uint32_t audio_packets_dropped = 0;

static volatile uint32_t audio_i2s_errors = 0;

static volatile uint32_t audio_short_writes = 0;


// ============================================================
// HIGH-PASS FILTER
// ============================================================

typedef struct
{
    float b0;
    float b1;
    float b2;

    float a1;
    float a2;

    float x1;
    float x2;

    float y1;
    float y2;

} high_pass_filter_t;


static high_pass_filter_t high_pass_left_1;
static high_pass_filter_t high_pass_left_2;

static high_pass_filter_t high_pass_right_1;
static high_pass_filter_t high_pass_right_2;


// ============================================================
// LOW-SHELF FILTER
// ============================================================

typedef struct
{
    float b0;
    float b1;
    float b2;

    float a1;
    float a2;

    float x1;
    float x2;

    float y1;
    float y2;

} low_shelf_filter_t;


static low_shelf_filter_t low_shelf_left;
static low_shelf_filter_t low_shelf_right;


// ============================================================
// HARD LIMITER
// ============================================================

typedef struct
{
    float ceiling_linear;

    uint32_t samples_limited;

    float max_input;

} hard_limiter_t;


static hard_limiter_t hard_limiter;


// ============================================================
// VOLUME STATE
// ============================================================
//
// IMPORTANT:
//
// The USB callback only changes target_gain.
//
// The audio callback is responsible for moving current_gain
// toward target_gain.
//
// This prevents the USB control path from introducing an
// instantaneous discontinuity into the audio waveform.
//

static volatile float volume_target_gain =
    1.0f;

static volatile float volume_target_db =
    0.0f;

static volatile uint32_t volume_percent =
    100;

static volatile uint32_t volume_muted =
    0;


// Current gain is ONLY modified by the audio callback.

static float volume_current_gain =
    1.0f;


// ============================================================
// VOLUME INITIALIZATION
// ============================================================

static void volume_init(void)
{
    volume_target_gain = 1.0f;

    volume_target_db = 0.0f;

    volume_percent = 100;

    volume_muted = 0;

    volume_current_gain = 1.0f;

    ESP_LOGI(
        TAG,
        "Volume control initialized"
    );

    ESP_LOGI(
        TAG,
        "  Range       : %.1f dB to +%.1f dB",
        VOLUME_MIN_DB,
        VOLUME_MAX_DB
    );

    ESP_LOGI(
        TAG,
        "  Max gain    : %.6f",
        VOLUME_MAX_GAIN
    );

    ESP_LOGI(
        TAG,
        "  Ramp        : %.1f ms",
        VOLUME_RAMP_MS
    );

    ESP_LOGI(
        TAG,
        "  Ramp samples: %" PRIu32,
        (uint32_t)VOLUME_RAMP_SAMPLES
    );
}


// ============================================================
// USB VOLUME -> GAIN
// ============================================================
//
// Converts the USB UAC volume value into a gain.
//
// The important point is that this is a dB mapping:
//
//     volume -> dB -> linear gain
//
// rather than:
//
//     volume -> linear gain
//
// This gives much more useful control at normal listening
// levels.
//

static float volume_to_gain(uint32_t volume)
{
    if (volume > 100) {

        volume = 100;
    }


    if (volume == 0) {

        return VOLUME_MIN_GAIN;
    }


    float normalized =
        (float)volume / 100.0f;


    float db =
        VOLUME_MIN_DB
        +
        normalized *
        (
            VOLUME_MAX_DB -
            VOLUME_MIN_DB
        );


    float gain =
        powf(
            10.0f,
            db / 20.0f
        );


    if (!isfinite(gain)) {

        gain = VOLUME_MIN_GAIN;
    }


    if (gain < VOLUME_MIN_GAIN) {

        gain = VOLUME_MIN_GAIN;
    }


    if (gain > VOLUME_MAX_GAIN) {

        gain = VOLUME_MAX_GAIN;
    }


    return gain;
}


// ============================================================
// VOLUME RAMP PROCESS
// ============================================================
//
// Moves current gain toward target gain.
//
// This is intentionally performed PER SAMPLE.
//
// Therefore a volume change cannot produce:
//
//     old_gain * sample
//
// immediately followed by:
//
//     new_gain * sample
//
// which is what causes the audible click.
//
// Instead:
//
//     old_gain
//          |
//          v
//      intermediate
//          |
//          v
//      intermediate
//          |
//          v
//      new_gain
//
// over approximately 30 ms.
//

static inline float volume_process_gain(void)
{
    const float target =
        volume_target_gain;


    float current =
        volume_current_gain;


    if (current == target) {

        return current;
    }


    float difference =
        target - current;


    float step =
        difference /
        (float)VOLUME_RAMP_SAMPLES;


    // --------------------------------------------------------
    // Avoid an extremely small step being rounded away.
    // --------------------------------------------------------

    if (step == 0.0f) {

        current = target;
    }
    else {

        current += step;


        // ----------------------------------------------------
        // Prevent overshoot.
        // ----------------------------------------------------

        if (
            (difference > 0.0f) &&
            (current > target)
        ) {

            current = target;
        }


        if (
            (difference < 0.0f) &&
            (current < target)
        ) {

            current = target;
        }
    }


    volume_current_gain =
        current;


    return current;
}


// ============================================================
// HIGH-PASS INITIALIZATION
// ============================================================

static void high_pass_init(
    high_pass_filter_t *filter,
    float sample_rate,
    float cutoff_hz
)
{
    memset(
        filter,
        0,
        sizeof(*filter)
    );


    const float omega =
        2.0f *
        (float)M_PI *
        cutoff_hz /
        sample_rate;


    const float sn =
        sinf(omega);


    const float cs =
        cosf(omega);


    const float Q =
        0.70710678118f;


    const float alpha =
        sn /
        (2.0f * Q);


    const float b0 =
        (1.0f + cs) /
        2.0f;


    const float b1 =
        -(1.0f + cs);


    const float b2 =
        (1.0f + cs) /
        2.0f;


    const float a0 =
        1.0f + alpha;


    const float a1 =
        -2.0f * cs;


    const float a2 =
        1.0f - alpha;


    filter->b0 =
        b0 / a0;

    filter->b1 =
        b1 / a0;

    filter->b2 =
        b2 / a0;

    filter->a1 =
        a1 / a0;

    filter->a2 =
        a2 / a0;

    filter->x1 = 0.0f;
    filter->x2 = 0.0f;

    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}


// ============================================================
// HIGH-PASS PROCESS
// ============================================================

static inline float high_pass_process(
    high_pass_filter_t *filter,
    float input
)
{
    const float output =
        filter->b0 * input
        + filter->b1 * filter->x1
        + filter->b2 * filter->x2
        - filter->a1 * filter->y1
        - filter->a2 * filter->y2;


    filter->x2 =
        filter->x1;

    filter->x1 =
        input;


    filter->y2 =
        filter->y1;

    filter->y1 =
        output;


    return output;
}


// ============================================================
// LOW-SHELF INITIALIZATION
// ============================================================

static void low_shelf_init(
    low_shelf_filter_t *filter,
    float sample_rate,
    float cutoff_hz,
    float gain_db,
    float slope
)
{
    memset(
        filter,
        0,
        sizeof(*filter)
    );


    if (fabsf(gain_db) < 0.0001f) {

        filter->b0 = 1.0f;

        return;
    }


    const float A =
        powf(
            10.0f,
            gain_db / 40.0f
        );


    const float omega =
        2.0f *
        (float)M_PI *
        cutoff_hz /
        sample_rate;


    const float sn =
        sinf(omega);


    const float cs =
        cosf(omega);


    const float alpha =
        sn / 2.0f *
        sqrtf(
            (A + 1.0f / A) *
            (
                1.0f / slope -
                1.0f
            )
            + 2.0f
        );


    const float two_sqrt_A_alpha =
        2.0f *
        sqrtf(A) *
        alpha;


    const float b0 =
        A *
        (
            (A + 1.0f)
            - (A - 1.0f) * cs
            + two_sqrt_A_alpha
        );


    const float b1 =
        2.0f *
        A *
        (
            (A - 1.0f)
            - (A + 1.0f) * cs
        );


    const float b2 =
        A *
        (
            (A + 1.0f)
            - (A - 1.0f) * cs
            - two_sqrt_A_alpha
        );


    const float a0 =
        (A + 1.0f)
        + (A - 1.0f) * cs
        + two_sqrt_A_alpha;


    const float a1 =
        -2.0f *
        (
            (A - 1.0f)
            + (A + 1.0f) * cs
        );


    const float a2 =
        (A + 1.0f)
        + (A - 1.0f) * cs
        - two_sqrt_A_alpha;


    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;

    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    filter->x1 = 0.0f;
    filter->x2 = 0.0f;

    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}


// ============================================================
// LOW-SHELF PROCESS
// ============================================================

static inline float low_shelf_process(
    low_shelf_filter_t *filter,
    float input
)
{
    const float output =
        filter->b0 * input
        + filter->b1 * filter->x1
        + filter->b2 * filter->x2
        - filter->a1 * filter->y1
        - filter->a2 * filter->y2;


    filter->x2 =
        filter->x1;

    filter->x1 =
        input;


    filter->y2 =
        filter->y1;

    filter->y1 =
        output;


    return output;
}


// ============================================================
// HARD LIMITER INITIALIZATION
// ============================================================

static void hard_limiter_init(void)
{
    hard_limiter.ceiling_linear =
        powf(
            10.0f,
            LIMITER_CEILING_DB / 20.0f
        );


    hard_limiter.samples_limited =
        0;


    hard_limiter.max_input =
        0.0f;


    if (
        !isfinite(
            hard_limiter.ceiling_linear
        )
    ) {

        hard_limiter.ceiling_linear =
            1.0f;
    }


    if (
        hard_limiter.ceiling_linear <= 0.0f
    ) {

        hard_limiter.ceiling_linear =
            1.0f;
    }


    if (
        hard_limiter.ceiling_linear > 1.0f
    ) {

        hard_limiter.ceiling_linear =
            1.0f;
    }


    ESP_LOGI(
        TAG,
        "HARD LIMITER INITIALIZED"
    );


    ESP_LOGI(
        TAG,
        "Ceiling       : %.2f dBFS",
        LIMITER_CEILING_DB
    );


    ESP_LOGI(
        TAG,
        "Linear ceiling: %.8f",
        hard_limiter.ceiling_linear
    );
}


// ============================================================
// HARD LIMITER PROCESS
// ============================================================

static inline float hard_limiter_process(
    float input
)
{
    const float abs_input =
        fabsf(input);


    if (
        abs_input >
        hard_limiter.max_input
    ) {

        hard_limiter.max_input =
            abs_input;
    }


    if (
        input >
        hard_limiter.ceiling_linear
    ) {

        hard_limiter.samples_limited++;

        return hard_limiter.ceiling_linear;
    }


    if (
        input <
        -hard_limiter.ceiling_linear
    ) {

        hard_limiter.samples_limited++;

        return -hard_limiter.ceiling_linear;
    }


    return input;
}


// ============================================================
// AUDIO FILTER INITIALIZATION
// ============================================================

static void audio_filter_init(void)
{
    high_pass_init(
        &high_pass_left_1,
        SAMPLE_RATE,
        HIGH_PASS_CUTOFF_HZ
    );


    high_pass_init(
        &high_pass_left_2,
        SAMPLE_RATE,
        HIGH_PASS_CUTOFF_HZ
    );


    high_pass_init(
        &high_pass_right_1,
        SAMPLE_RATE,
        HIGH_PASS_CUTOFF_HZ
    );


    high_pass_init(
        &high_pass_right_2,
        SAMPLE_RATE,
        HIGH_PASS_CUTOFF_HZ
    );


    low_shelf_init(
        &low_shelf_left,
        SAMPLE_RATE,
        LOW_FREQ_CUTOFF_HZ,
        LOW_FREQ_BOOST_DB,
        LOW_FREQ_SLOPE
    );


    low_shelf_init(
        &low_shelf_right,
        SAMPLE_RATE,
        LOW_FREQ_CUTOFF_HZ,
        LOW_FREQ_BOOST_DB,
        LOW_FREQ_SLOPE
    );


    hard_limiter_init();


    ESP_LOGI(
        TAG,
        "Audio DSP configuration:"
    );


    ESP_LOGI(
        TAG,
        "  High-pass    = %.1f Hz",
        HIGH_PASS_CUTOFF_HZ
    );


    ESP_LOGI(
        TAG,
        "  HPF order    = 4th order"
    );


    ESP_LOGI(
        TAG,
        "  HPF slope    = 24 dB/octave"
    );


    ESP_LOGI(
        TAG,
        "  Bass shelf   = %.1f Hz",
        LOW_FREQ_CUTOFF_HZ
    );


    ESP_LOGI(
        TAG,
        "  Bass boost   = %.1f dB",
        LOW_FREQ_BOOST_DB
    );


    ESP_LOGI(
        TAG,
        "  Limiter      = %.1f dBFS",
        LIMITER_CEILING_DB
    );
}


// ============================================================
// FLOAT -> INT16
// ============================================================

static inline int16_t float_to_int16(
    float value
)
{
    if (value > 0.999969f) {

        value = 0.999969f;
    }


    if (value < -1.0f) {

        value = -1.0f;
    }


    return (int16_t)(
        value * 32767.0f
    );
}


// ============================================================
// MAX98390 WRITE
// ============================================================

static esp_err_t max_write(
    uint16_t reg,
    uint8_t value
)
{
    if (max98390 == NULL) {

        return ESP_ERR_INVALID_STATE;
    }


    uint8_t data[3];


    data[0] =
        (uint8_t)(reg >> 8);

    data[1] =
        (uint8_t)(reg & 0xFF);

    data[2] =
        value;


    esp_err_t err =
        i2c_master_transmit(
            max98390,
            data,
            sizeof(data),
            100
        );


    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "I2C WRITE FAILED: "
            "reg=0x%04X value=0x%02X: %s",
            reg,
            value,
            esp_err_to_name(err)
        );
    }


    return err;
}


// ============================================================
// MAX98390 READ
// ============================================================

static esp_err_t max_read(
    uint16_t reg,
    uint8_t *value
)
{
    if (max98390 == NULL) {

        return ESP_ERR_INVALID_STATE;
    }


    if (value == NULL) {

        return ESP_ERR_INVALID_ARG;
    }


    uint8_t addr[2];


    addr[0] =
        (uint8_t)(reg >> 8);

    addr[1] =
        (uint8_t)(reg & 0xFF);


    return i2c_master_transmit_receive(
        max98390,
        addr,
        sizeof(addr),
        value,
        1,
        100
    );
}


// ============================================================
// MAX98390 HARDWARE RESET
// ============================================================

static void max98390_hardware_reset(void)
{
    gpio_config_t cfg = {

        .pin_bit_mask =
            1ULL << MAX98390_RESET_GPIO,

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE,
    };


    ESP_ERROR_CHECK(
        gpio_config(&cfg)
    );


    gpio_set_level(
        MAX98390_RESET_GPIO,
        0
    );


    vTaskDelay(
        pdMS_TO_TICKS(20)
    );


    gpio_set_level(
        MAX98390_RESET_GPIO,
        1
    );


    vTaskDelay(
        pdMS_TO_TICKS(100)
    );


    ESP_LOGI(
        TAG,
        "MAX98390 reset complete"
    );
}


// ============================================================
// I2C INITIALIZATION
// ============================================================

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {

        .i2c_port =
            I2C_NUM_0,

        .sda_io_num =
            I2C_SDA_GPIO,

        .scl_io_num =
            I2C_SCL_GPIO,

        .clk_source =
            I2C_CLK_SRC_DEFAULT,

        .glitch_ignore_cnt =
            7,

        .flags.enable_internal_pullup =
            false,
    };


    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_cfg,
            &i2c_bus
        )
    );


    i2c_device_config_t dev_cfg = {

        .dev_addr_length =
            I2C_ADDR_BIT_LEN_7,

        .device_address =
            MAX98390_I2C_ADDR,

        .scl_speed_hz =
            100000,
    };


    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            i2c_bus,
            &dev_cfg,
            &max98390
        )
    );


    ESP_LOGI(
        TAG,
        "MAX98390 I2C device added at 0x%02X",
        MAX98390_I2C_ADDR
    );
}


// ============================================================
// CHECK MAX98390
// ============================================================

static void check_max98390(void)
{
    uint8_t value = 0;


    ESP_ERROR_CHECK(
        max_read(
            0x2000,
            &value
        )
    );


    ESP_LOGI(
        TAG,
        "MAX98390 responding: "
        "register 0x2000 = 0x%02X",
        value
    );
}


// ============================================================
// MAX98390 CONFIGURATION
// ============================================================

static void max98390_configure(void)
{
    ESP_LOGI(
        TAG,
        "CONFIGURING MAX98390"
    );


    ESP_ERROR_CHECK(
        max_write(
            0x23FF,
            0x00
        )
    );


    vTaskDelay(
        pdMS_TO_TICKS(20)
    );


    ESP_ERROR_CHECK(
        max_write(
            0x203A,
            0x80
        )
    );


    vTaskDelay(
        pdMS_TO_TICKS(10)
    );


    ESP_ERROR_CHECK(
        max_write(
            0x23E1,
            0x00
        )
    );


    vTaskDelay(
        pdMS_TO_TICKS(10)
    );


    ESP_ERROR_CHECK(
        max_write(
            0x201B,
            0x01
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2024,
            0x40
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2025,
            0x00
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2026,
            0x22
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2027,
            0x08
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2021,
            0x00
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2041,
            0x03
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2012,
            0x6F
        )
    );


    ESP_ERROR_CHECK(
        max_write(
            0x2014,
            0x00
        )
    );


    ESP_LOGI(
        TAG,
        "MAX98390 configuration complete"
    );
}


// ============================================================
// MAX98390 ENABLE
// ============================================================

static void max98390_enable(void)
{
    ESP_ERROR_CHECK(
        max_write(
            0x203A,
            0x81
        )
    );


    vTaskDelay(
        pdMS_TO_TICKS(10)
    );


    ESP_ERROR_CHECK(
        max_write(
            0x23FF,
            0x01
        )
    );


    vTaskDelay(
        pdMS_TO_TICKS(20)
    );


    uint8_t value = 0;


    ESP_ERROR_CHECK(
        max_read(
            0x23FF,
            &value
        )
    );


    ESP_LOGI(
        TAG,
        "MAX98390 GLOBAL_EN = 0x%02X",
        value
    );


    if (value != 0x01) {

        ESP_LOGE(
            TAG,
            "GLOBAL_EN DID NOT LATCH"
        );

        abort();
    }
}


// ============================================================
// I2S INITIALIZATION
// ============================================================

static void i2s_init(void)
{
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_PORT,
            I2S_ROLE_MASTER
        );


    chan_cfg.dma_desc_num =
        16;


    chan_cfg.dma_frame_num =
        256;


    chan_cfg.auto_clear =
        false;


    ESP_ERROR_CHECK(
        i2s_new_channel(
            &chan_cfg,
            &i2s_tx,
            NULL
        )
    );


    i2s_std_config_t std_cfg = {

        .clk_cfg =
            I2S_STD_CLK_DEFAULT_CONFIG(
                SAMPLE_RATE
            ),

        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_16BIT,
                I2S_SLOT_MODE_STEREO
            ),

        .gpio_cfg = {

            .mclk =
                I2S_GPIO_UNUSED,

            .bclk =
                I2S_BCLK_GPIO,

            .ws =
                I2S_WS_GPIO,

            .dout =
                I2S_DOUT_GPIO,

            .din =
                I2S_GPIO_UNUSED,

            .invert_flags = {

                .mclk_inv =
                    false,

                .bclk_inv =
                    false,

                .ws_inv =
                    false,
            },
        },
    };


    std_cfg.clk_cfg.mclk_multiple =
        I2S_MCLK_MULTIPLE_256;


    ESP_ERROR_CHECK(
        i2s_channel_init_std_mode(
            i2s_tx,
            &std_cfg
        )
    );


    ESP_ERROR_CHECK(
        i2s_channel_enable(
            i2s_tx
        )
    );


    ESP_LOGI(
        TAG,
        "I2S initialized: %d Hz / %d bit / stereo",
        SAMPLE_RATE,
        AUDIO_BITS
    );
}


// ============================================================
// AUDIO BUFFER INITIALIZATION
// ============================================================

static void audio_buffer_init(void)
{
    audio_queue =
        xQueueCreate(
            AUDIO_QUEUE_LENGTH,
            sizeof(audio_block_t)
        );


    if (audio_queue == NULL) {

        ESP_LOGE(
            TAG,
            "FAILED TO CREATE AUDIO QUEUE"
        );

        abort();
    }


    BaseType_t result =
        xTaskCreatePinnedToCore(
            audio_i2s_task,
            "audio_i2s",
            AUDIO_I2S_TASK_STACK,
            NULL,
            AUDIO_I2S_TASK_PRIORITY,
            NULL,
            1
        );


    if (result != pdPASS) {

        ESP_LOGE(
            TAG,
            "FAILED TO CREATE I2S TASK"
        );

        abort();
    }


    result =
        xTaskCreatePinnedToCore(
            audio_monitor_task,
            "audio_monitor",
            MONITOR_TASK_STACK,
            NULL,
            MONITOR_TASK_PRIORITY,
            NULL,
            1
        );


    if (result != pdPASS) {

        ESP_LOGE(
            TAG,
            "FAILED TO CREATE MONITOR TASK"
        );

        abort();
    }
}


// ============================================================
// I2S AUDIO TASK
// ============================================================

static void audio_i2s_task(void *arg)
{
    (void)arg;


    audio_block_t block;


    while (1) {

        if (
            xQueueReceive(
                audio_queue,
                &block,
                portMAX_DELAY
            ) != pdTRUE
        ) {

            continue;
        }


        if (block.len == 0) {

            continue;
        }


        size_t offset = 0;


        while (offset < block.len) {

            size_t bytes_written = 0;


            esp_err_t err =
                i2s_channel_write(
                    i2s_tx,
                    block.data + offset,
                    block.len - offset,
                    &bytes_written,
                    pdMS_TO_TICKS(100)
                );


            if (err != ESP_OK) {

                audio_i2s_errors++;


                ESP_LOGE(
                    TAG,
                    "I2S write failed: %s",
                    esp_err_to_name(err)
                );


                break;
            }


            if (bytes_written == 0) {

                audio_i2s_errors++;

                break;
            }


            offset += bytes_written;
        }


        if (offset != block.len) {

            audio_short_writes++;
        }
    }
}


// ============================================================
// USB AUDIO OUTPUT CALLBACK
// ============================================================
//
// DSP:
//
// USB stereo
//      |
//      v
// INT16 -> FLOAT
//      |
//      v
// 45 Hz HPF
//      |
//      v
// +14 dB bass shelf
//      |
//      v
// Stereo -> Mono
//      |
//      v
// Hard limiter
//      |
//      v
// SMOOTH VOLUME RAMP
//      |
//      v
// FLOAT -> INT16
//      |
//      v
// Mono -> L/R
//      |
//      v
// I2S
//
// The volume stage is AFTER the existing limiter so the bass
// processing remains unchanged.
//

static esp_err_t usb_audio_output_cb(
    uint8_t *buf,
    size_t len,
    void *arg
)
{
    (void)arg;


    if (audio_queue == NULL) {

        return ESP_ERR_INVALID_STATE;
    }


    if (buf == NULL || len == 0) {

        return ESP_OK;
    }


    if (
        (len % AUDIO_FRAME_BYTES) != 0
    ) {

        len -=
            len % AUDIO_FRAME_BYTES;
    }


    if (len == 0) {

        return ESP_OK;
    }


    if (len > AUDIO_BLOCK_SIZE) {

        audio_packets_dropped++;

        return ESP_OK;
    }


    for (
        size_t offset = 0;
        offset < len;
        offset += AUDIO_FRAME_BYTES
    ) {

        int16_t left;

        int16_t right;


        memcpy(
            &left,
            &buf[offset],
            sizeof(left)
        );


        memcpy(
            &right,
            &buf[offset + 2],
            sizeof(right)
        );


        float left_f =
            (float)left /
            32768.0f;


        float right_f =
            (float)right /
            32768.0f;


        // ====================================================
        // 45 Hz HIGH-PASS
        // ====================================================

        left_f =
            high_pass_process(
                &high_pass_left_1,
                left_f
            );


        left_f =
            high_pass_process(
                &high_pass_left_2,
                left_f
            );


        right_f =
            high_pass_process(
                &high_pass_right_1,
                right_f
            );


        right_f =
            high_pass_process(
                &high_pass_right_2,
                right_f
            );


        // ====================================================
        // LOW SHELF
        // ====================================================

        left_f =
            low_shelf_process(
                &low_shelf_left,
                left_f
            );


        right_f =
            low_shelf_process(
                &low_shelf_right,
                right_f
            );


        // ====================================================
        // STEREO -> MONO
        // ====================================================

        float mono_f =
            (left_f + right_f) *
            0.5f;


        // ====================================================
        // HARD LIMITER
        // ====================================================

        mono_f =
            hard_limiter_process(
                mono_f
            );


        // ====================================================
        // SMOOTH VOLUME
        // ====================================================
        //
        // This is the important anti-click stage.
        //
        // current_gain changes only a tiny amount per sample.
        //

        const float gain =
            volume_process_gain();


        mono_f *= gain;


        // ====================================================
        // FINAL FLOAT -> INT16 SAFETY CLIP
        // ====================================================

        int16_t mono =
            float_to_int16(
                mono_f
            );


        // ====================================================
        // MONO -> L/R
        // ====================================================

        memcpy(
            &buf[offset],
            &mono,
            sizeof(mono)
        );


        memcpy(
            &buf[offset + 2],
            &mono,
            sizeof(mono)
        );
    }


    audio_block_t block;


    block.len =
        len;


    memcpy(
        block.data,
        buf,
        len
    );


    if (
        xQueueSend(
            audio_queue,
            &block,
            0
        ) != pdTRUE
    ) {

        audio_packets_dropped++;

        return ESP_OK;
    }


    audio_packets_received++;

    audio_bytes_received +=
        len;


    return ESP_OK;
}


// ============================================================
// USB MICROPHONE INPUT
// ============================================================

static esp_err_t usb_audio_input_cb(
    uint8_t *buf,
    size_t len,
    size_t *bytes_read,
    void *arg
)
{
    (void)arg;


    if (buf == NULL) {

        return ESP_ERR_INVALID_ARG;
    }


    memset(
        buf,
        0,
        len
    );


    if (bytes_read != NULL) {

        *bytes_read =
            len;
    }


    return ESP_OK;
}


// ============================================================
// USB MUTE CALLBACK
// ============================================================
//
// IMPORTANT:
//
// Do NOT directly zero the audio buffer here.
//
// Doing that can create the exact same discontinuity that
// causes volume clicks.
//
// Instead, change the target gain and let the normal sample
// ramp handle the transition.
//

static void usb_audio_mute_cb(
    uint32_t mute,
    void *arg
)
{
    (void)arg;


    volume_muted =
        mute ? 1 : 0;


    if (mute) {

        volume_target_gain =
            VOLUME_MIN_GAIN;
    }
    else {

        volume_target_gain =
            volume_to_gain(
                volume_percent
            );
    }


    ESP_LOGI(
        TAG,
        "USB mute = %" PRIu32,
        mute
    );
}


// ============================================================
// USB VOLUME CALLBACK
// ============================================================
//
// UAC gives the volume as a percentage-style value.
//
// We convert:
//
//     USB volume
//
// to:
//
//     dB
//
// then:
//
//     dB -> linear gain
//
// The gain is only assigned to volume_target_gain.
//
// The audio callback performs the actual transition.
//

static void usb_audio_volume_cb(
    uint32_t volume,
    void *arg
)
{
    (void)arg;


    if (volume > 100) {

        volume = 100;
    }


    volume_percent =
        volume;


    float db =
        VOLUME_MIN_DB
        +
        (
            (float)volume /
            100.0f
        )
        *
        (
            VOLUME_MAX_DB -
            VOLUME_MIN_DB
        );


    float gain =
        volume_to_gain(
            volume
        );


    volume_target_db =
        db;


    if (volume_muted) {

        volume_target_gain =
            VOLUME_MIN_GAIN;
    }
    else {

        volume_target_gain =
            gain;
    }


    ESP_LOGI(
        TAG,
        "USB volume = %" PRIu32
        " -> %.2f dB -> gain %.6f",
        volume,
        db,
        gain
    );
}


// ============================================================
// USB AUDIO INITIALIZATION
// ============================================================

static void usb_audio_init(void)
{
    ESP_LOGI(
        TAG,
        "CONFIGURING USB AUDIO"
    );


    uac_device_config_t config = {

        .output_cb =
            usb_audio_output_cb,

        .input_cb =
            usb_audio_input_cb,

        .set_mute_cb =
            usb_audio_mute_cb,

        .set_volume_cb =
            usb_audio_volume_cb,

        .cb_ctx = NULL,
    };


    ESP_ERROR_CHECK(
        uac_device_init(
            &config
        )
    );


    ESP_LOGI(
        TAG,
        "USB Audio initialized"
    );
}


// ============================================================
// AUDIO MONITOR TASK
// ============================================================

static void audio_monitor_task(void *arg)
{
    (void)arg;


    uint32_t last_packets = 0;

    uint32_t last_bytes = 0;

    uint32_t last_dropped = 0;

    uint32_t last_errors = 0;

    uint32_t last_short = 0;


    while (1) {

        UBaseType_t queued =
            uxQueueMessagesWaiting(
                audio_queue
            );


        uint32_t packets =
            audio_packets_received;

        uint32_t bytes =
            audio_bytes_received;

        uint32_t dropped =
            audio_packets_dropped;

        uint32_t errors =
            audio_i2s_errors;

        uint32_t short_writes =
            audio_short_writes;


        ESP_LOGI(
            TAG,
            "AUDIO: queue=%u/%u "
            "packets=%" PRIu32
            " (+%" PRIu32 ") "
            "bytes=%" PRIu32
            " (+%" PRIu32 ") "
            "dropped=%" PRIu32
            " (+%" PRIu32 ") "
            "i2s_err=%" PRIu32
            " (+%" PRIu32 ") "
            "short=%" PRIu32
            " (+%" PRIu32 ")",
            (unsigned)queued,
            (unsigned)AUDIO_QUEUE_LENGTH,

            packets,
            packets - last_packets,

            bytes,
            bytes - last_bytes,

            dropped,
            dropped - last_dropped,

            errors,
            errors - last_errors,

            short_writes,
            short_writes - last_short
        );


        float max_input =
            hard_limiter.max_input;


        float max_input_db =
            -INFINITY;


        if (max_input > 0.0000001f) {

            max_input_db =
                20.0f *
                log10f(max_input);
        }


        ESP_LOGI(
            TAG,
            "LIMITER: ceiling=%.2f dBFS "
            "max_in=%.6f "
            "max_in=%.2f dBFS "
            "limited=%" PRIu32,
            LIMITER_CEILING_DB,

            max_input,

            max_input_db,

            hard_limiter.samples_limited
        );


        float current_db =
            -INFINITY;


        if (
            volume_current_gain >
            0.0000001f
        ) {

            current_db =
                20.0f *
                log10f(
                    volume_current_gain
                );
        }


        ESP_LOGI(
            TAG,
            "VOLUME: usb=%" PRIu32
            " target=%.2f dB "
            " current=%.2f dB "
            " muted=%" PRIu32,
            volume_percent,
            volume_target_db,
            current_db,
            volume_muted
        );


        last_packets =
            packets;

        last_bytes =
            bytes;

        last_dropped =
            dropped;

        last_errors =
            errors;

        last_short =
            short_writes;


        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


// ============================================================
// STATUS LED
// ============================================================

static void status_led_init(void)
{
    gpio_config_t cfg = {

        .pin_bit_mask =
            1ULL << STATUS_LED_GPIO,

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE,
    };


    ESP_ERROR_CHECK(
        gpio_config(&cfg)
    );


    gpio_set_level(
        STATUS_LED_GPIO,
        0
    );
}


// ============================================================
// STATUS LED TASK
// ============================================================

static void status_led_task(void *arg)
{
    (void)arg;


    while (1) {

        gpio_set_level(
            STATUS_LED_GPIO,
            1
        );


        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );


        gpio_set_level(
            STATUS_LED_GPIO,
            0
        );


        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


// ============================================================
// APP MAIN
// ============================================================

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "======================================"
    );


    ESP_LOGI(
        TAG,
        "ESP32-S3 / MAX98390 USB AUDIO"
    );


    ESP_LOGI(
        TAG,
        "======================================"
    );


    // --------------------------------------------------------
    // STATUS LED
    // --------------------------------------------------------

    status_led_init();


    xTaskCreate(
        status_led_task,
        "status_led",
        STATUS_TASK_STACK,
        NULL,
        STATUS_TASK_PRIORITY,
        NULL
    );


    // --------------------------------------------------------
    // MAX98390
    // --------------------------------------------------------

    max98390_hardware_reset();

    i2c_init();

    check_max98390();


    ESP_ERROR_CHECK(
        max_write(
            0x2000,
            0x01
        )
    );


    vTaskDelay(
        pdMS_TO_TICKS(100)
    );


    max98390_configure();


    // --------------------------------------------------------
    // I2S
    // --------------------------------------------------------

    i2s_init();


    // --------------------------------------------------------
    // DSP
    // --------------------------------------------------------

    audio_filter_init();


    // --------------------------------------------------------
    // VOLUME
    // --------------------------------------------------------

    volume_init();


    // --------------------------------------------------------
    // AUDIO QUEUE / TASKS
    // --------------------------------------------------------

    audio_buffer_init();


    // --------------------------------------------------------
    // ENABLE AMP
    // --------------------------------------------------------

    max98390_enable();


    // --------------------------------------------------------
    // USB UAC
    // --------------------------------------------------------

    usb_audio_init();


    // --------------------------------------------------------
    // READY
    // --------------------------------------------------------

    ESP_LOGI(
        TAG,
        "======================================"
    );

    ESP_LOGI(
        TAG,
        "USB AUDIO READY"
    );

    ESP_LOGI(
        TAG,
        "======================================"
    );

    ESP_LOGI(
        TAG,
        "High-pass       : %.1f Hz",
        HIGH_PASS_CUTOFF_HZ
    );

    ESP_LOGI(
        TAG,
        "Bass shelf      : %.1f Hz",
        LOW_FREQ_CUTOFF_HZ
    );

    ESP_LOGI(
        TAG,
        "Bass boost      : +%.1f dB",
        LOW_FREQ_BOOST_DB
    );

    ESP_LOGI(
        TAG,
        "Limiter         : %.1f dBFS",
        LIMITER_CEILING_DB
    );

    ESP_LOGI(
        TAG,
        "Volume range    : %.1f to +%.1f dB",
        VOLUME_MIN_DB,
        VOLUME_MAX_DB
    );

    ESP_LOGI(
        TAG,
        "Volume max gain : %.6f",
        VOLUME_MAX_GAIN
    );

    ESP_LOGI(
        TAG,
        "Volume ramp     : %.1f ms",
        VOLUME_RAMP_MS
    );

    ESP_LOGI(
        TAG,
        "======================================"
    );


    while (1) {

        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}
