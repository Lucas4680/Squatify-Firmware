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
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

#include "usb_device_uac.h"

#include "max98390.h"
#include "speaker_config.h"
#include "speaker_dsp.h"


// ============================================================
// HARDWARE
// ============================================================

#define MAX98390_I2C_ADDR       0x38

#define I2C_SDA_GPIO            15
#define I2C_SCL_GPIO            35

#define MAX98390_RESET_GPIO     37

// GPIO wired to the MAX98390 IRQ pin (open drain, active low). The fault
// registers are only read over I2C while this line is low. Set to -1 to
// poll them every second instead.
#define MAX98390_IRQ_GPIO       36

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

_Static_assert((int)SPK_SAMPLE_RATE_HZ == SAMPLE_RATE,
               "speaker_config.h sample rate must match the I2S sample rate");


// ============================================================
// VOLUME CONTROL
// ============================================================
//
// USB volume is treated as 0..100 and mapped linearly in dB:
//
//     0%   -> -60 dB
//     100% -> +5 dB
//
// The volume is applied BEFORE the speaker processing. At low volume the
// full bass EQ is heard; near the top of the range the limiters in
// speaker_dsp.c hold the bass and the peaks at what the speaker and the
// USB supply can take, so turning up further mostly adds density.

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

#define MONITOR_TASK_STACK      4096
#define MONITOR_TASK_PRIORITY   5

#define STATUS_TASK_STACK       2048
#define STATUS_TASK_PRIORITY    1


// ============================================================
// TAG
// ============================================================

static const char *TAG = "MAX98390_USB_AUDIO";


// ============================================================
// GLOBAL HANDLES
// ============================================================

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

typedef struct {
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
// VOLUME STATE
// ============================================================
//
// The USB callbacks only change the target gain. The audio callback moves
// the current gain toward it one sample at a time, so the USB control path
// can never put a step into the waveform.

static volatile float volume_target_gain = 1.0f;
static volatile float volume_target_db = 0.0f;
static volatile uint32_t volume_percent = 100;
static volatile uint32_t volume_muted = 0;

// Only modified by the audio callback.
static float volume_current_gain = 1.0f;


static void volume_init(void)
{
    volume_target_gain = 1.0f;
    volume_target_db = 0.0f;
    volume_percent = 100;
    volume_muted = 0;
    volume_current_gain = 1.0f;

    ESP_LOGI(TAG, "Volume control initialized");
    ESP_LOGI(TAG, "  Range       : %.1f dB to +%.1f dB", VOLUME_MIN_DB, VOLUME_MAX_DB);
    ESP_LOGI(TAG, "  Max gain    : %.6f", VOLUME_MAX_GAIN);
    ESP_LOGI(TAG, "  Ramp        : %.1f ms", VOLUME_RAMP_MS);
    ESP_LOGI(TAG, "  Ramp samples: %" PRIu32, (uint32_t)VOLUME_RAMP_SAMPLES);
}


// USB volume (0..100) -> dB -> linear gain.
static float volume_to_gain(uint32_t volume)
{
    if (volume > 100) {
        volume = 100;
    }

    if (volume == 0) {
        return VOLUME_MIN_GAIN;
    }

    const float normalized = (float)volume / 100.0f;

    const float db = VOLUME_MIN_DB + normalized * (VOLUME_MAX_DB - VOLUME_MIN_DB);

    float gain = powf(10.0f, db / 20.0f);

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


// Moves the current gain toward the target gain, one small step per sample.
static inline float volume_process_gain(void)
{
    const float target = volume_target_gain;

    float current = volume_current_gain;

    if (current == target) {
        return current;
    }

    const float difference = target - current;

    const float step = difference / (float)VOLUME_RAMP_SAMPLES;

    if (step == 0.0f) {
        // An extremely small step would be rounded away.
        current = target;
    }
    else {
        current += step;

        // Prevent overshoot.
        if ((difference > 0.0f) && (current > target)) {
            current = target;
        }

        if ((difference < 0.0f) && (current < target)) {
            current = target;
        }
    }

    volume_current_gain = current;

    return current;
}


// ============================================================
// FLOAT -> INT16
// ============================================================

static inline int16_t float_to_int16(float value)
{
    if (value > 0.999969f) {
        value = 0.999969f;
    }

    if (value < -1.0f) {
        value = -1.0f;
    }

    return (int16_t)(value * 32767.0f);
}


// ============================================================
// I2S INITIALIZATION
// ============================================================

static void i2s_init(void)
{
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);

    chan_cfg.dma_desc_num = 16;
    chan_cfg.dma_frame_num = 256;
    chan_cfg.auto_clear = false;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx, NULL));

    i2s_std_config_t std_cfg = {

        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx, &std_cfg));

    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));

    ESP_LOGI(TAG, "I2S initialized: %d Hz / %d bit / stereo", SAMPLE_RATE, AUDIO_BITS);
}


// ============================================================
// AUDIO BUFFER INITIALIZATION
// ============================================================

static void audio_buffer_init(void)
{
    audio_queue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(audio_block_t));

    if (audio_queue == NULL) {
        ESP_LOGE(TAG, "FAILED TO CREATE AUDIO QUEUE");
        abort();
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        audio_i2s_task,
        "audio_i2s",
        AUDIO_I2S_TASK_STACK,
        NULL,
        AUDIO_I2S_TASK_PRIORITY,
        NULL,
        1);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "FAILED TO CREATE I2S TASK");
        abort();
    }

    result = xTaskCreatePinnedToCore(
        audio_monitor_task,
        "audio_monitor",
        MONITOR_TASK_STACK,
        NULL,
        MONITOR_TASK_PRIORITY,
        NULL,
        1);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "FAILED TO CREATE MONITOR TASK");
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

        if (xQueueReceive(audio_queue, &block, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (block.len == 0) {
            continue;
        }

        size_t offset = 0;

        while (offset < block.len) {

            size_t bytes_written = 0;

            esp_err_t err = i2s_channel_write(
                i2s_tx,
                block.data + offset,
                block.len - offset,
                &bytes_written,
                pdMS_TO_TICKS(100));

            if (err != ESP_OK) {
                audio_i2s_errors++;
                ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(err));
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
//     USB stereo
//         |
//     INT16 -> FLOAT
//         |
//     Stereo -> Mono
//         |
//     Smooth volume ramp
//         |
//     Speaker processing (speaker_dsp.c):
//         protection high-pass, EQ, bass/excursion limiter,
//         peak limiter, power budget
//         |
//     FLOAT -> INT16
//         |
//     Mono -> L/R
//         |
//     I2S

static esp_err_t usb_audio_output_cb(uint8_t *buf, size_t len, void *arg)
{
    (void)arg;

    if (audio_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (buf == NULL || len == 0) {
        return ESP_OK;
    }

    if ((len % AUDIO_FRAME_BYTES) != 0) {
        len -= len % AUDIO_FRAME_BYTES;
    }

    if (len == 0) {
        return ESP_OK;
    }

    if (len > AUDIO_BLOCK_SIZE) {
        audio_packets_dropped++;
        return ESP_OK;
    }

    for (size_t offset = 0; offset < len; offset += AUDIO_FRAME_BYTES) {

        int16_t left;
        int16_t right;

        memcpy(&left, &buf[offset], sizeof(left));
        memcpy(&right, &buf[offset + 2], sizeof(right));

        // Stereo -> mono.
        float mono_f = ((float)left + (float)right) * (0.5f / 32768.0f);

        // Volume first, so the protection only works when it is loud.
        mono_f *= volume_process_gain();

        // EQ and speaker protection.
        mono_f = speaker_dsp_process(mono_f);

        const int16_t mono = float_to_int16(mono_f);

        // Mono -> L/R.
        memcpy(&buf[offset], &mono, sizeof(mono));
        memcpy(&buf[offset + 2], &mono, sizeof(mono));
    }

    audio_block_t block;

    block.len = len;

    memcpy(block.data, buf, len);

    if (xQueueSend(audio_queue, &block, 0) != pdTRUE) {
        audio_packets_dropped++;
        return ESP_OK;
    }

    audio_packets_received++;
    audio_bytes_received += len;

    return ESP_OK;
}


// ============================================================
// USB MICROPHONE INPUT
// ============================================================

static esp_err_t usb_audio_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    (void)arg;

    if (buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(buf, 0, len);

    if (bytes_read != NULL) {
        *bytes_read = len;
    }

    return ESP_OK;
}


// ============================================================
// USB MUTE CALLBACK
// ============================================================
//
// The buffer is never zeroed directly: that would click. Only the target
// gain changes and the normal sample ramp handles the transition.

static void usb_audio_mute_cb(uint32_t mute, void *arg)
{
    (void)arg;

    volume_muted = mute ? 1 : 0;

    if (mute) {
        volume_target_gain = VOLUME_MIN_GAIN;
    }
    else {
        volume_target_gain = volume_to_gain(volume_percent);
    }

    ESP_LOGI(TAG, "USB mute = %" PRIu32, mute);
}


// ============================================================
// USB VOLUME CALLBACK
// ============================================================

static void usb_audio_volume_cb(uint32_t volume, void *arg)
{
    (void)arg;

    if (volume > 100) {
        volume = 100;
    }

    volume_percent = volume;

    const float db =
        VOLUME_MIN_DB + ((float)volume / 100.0f) * (VOLUME_MAX_DB - VOLUME_MIN_DB);

    const float gain = volume_to_gain(volume);

    volume_target_db = db;

    if (volume_muted) {
        volume_target_gain = VOLUME_MIN_GAIN;
    }
    else {
        volume_target_gain = gain;
    }

    ESP_LOGI(TAG, "USB volume = %" PRIu32 " -> %.2f dB -> gain %.6f", volume, db, gain);
}


// ============================================================
// USB AUDIO INITIALIZATION
// ============================================================

static void usb_audio_init(void)
{
    ESP_LOGI(TAG, "CONFIGURING USB AUDIO");

    uac_device_config_t config = {
        .output_cb = usb_audio_output_cb,
        .input_cb = usb_audio_input_cb,
        .set_mute_cb = usb_audio_mute_cb,
        .set_volume_cb = usb_audio_volume_cb,
        .cb_ctx = NULL,
    };

    ESP_ERROR_CHECK(uac_device_init(&config));

    ESP_LOGI(TAG, "USB Audio initialized");
}


// ============================================================
// AUDIO MONITOR TASK
// ============================================================

static float gain_to_db(float gain)
{
    if (gain < 0.000001f) {
        gain = 0.000001f;
    }

    return 20.0f * log10f(gain);
}


static void audio_monitor_task(void *arg)
{
    (void)arg;

    uint32_t last_packets = 0;
    uint32_t last_bytes = 0;
    uint32_t last_dropped = 0;
    uint32_t last_errors = 0;
    uint32_t last_short = 0;

    while (1) {

        const UBaseType_t queued = uxQueueMessagesWaiting(audio_queue);

        const uint32_t packets = audio_packets_received;
        const uint32_t bytes = audio_bytes_received;
        const uint32_t dropped = audio_packets_dropped;
        const uint32_t errors = audio_i2s_errors;
        const uint32_t short_writes = audio_short_writes;

        ESP_LOGI(TAG,
                 "AUDIO: queue=%u/%u "
                 "packets=%" PRIu32 " (+%" PRIu32 ") "
                 "bytes=%" PRIu32 " (+%" PRIu32 ") "
                 "dropped=%" PRIu32 " (+%" PRIu32 ") "
                 "i2s_err=%" PRIu32 " (+%" PRIu32 ") "
                 "short=%" PRIu32 " (+%" PRIu32 ")",
                 (unsigned)queued, (unsigned)AUDIO_QUEUE_LENGTH,
                 packets, packets - last_packets,
                 bytes, bytes - last_bytes,
                 dropped, dropped - last_dropped,
                 errors, errors - last_errors,
                 short_writes, short_writes - last_short);

        // What the speaker processing did during the last second.
        // The gains are 0 dB when a stage is idle and negative when it is
        // holding something back.
        speaker_dsp_stats_t dsp;

        speaker_dsp_read_stats(&dsp);

        ESP_LOGI(TAG,
                 "DSP: out=%.2f V  cone=%.2f/%.1f mm  radiator=%.2f/%.1f mm  "
                 "power=%.2f/%.1f W  bass=%.1f dB  peak=%.1f dB  budget=%.1f dB  "
                 "clamped=%" PRIu32,
                 dsp.out_peak_v,
                 dsp.xd_peak_mm, SPK_DRV_XMAX_MM,
                 dsp.xp_peak_mm, SPK_PR_XLIMIT_MM,
                 dsp.power_avg_w, SPK_POWER_BUDGET_W,
                 gain_to_db(dsp.bass_gain_min),
                 gain_to_db(dsp.peak_gain_min),
                 gain_to_db(dsp.power_gain),
                 dsp.clamped);

        // What the amplifier measured: the supply line is the one to watch
        // on USB power.
        max98390_status_t amp;

        if (max98390_read_status(&amp) == ESP_OK) {

            ESP_LOGI(TAG,
                     "AMP: supply=%.2f V (low %.2f V)  boost=%.2f V  die=%.0f C  "
                     "brownout=%u (worst %u)",
                     amp.vbat_v, amp.vbat_lowest_v,
                     amp.pvdd_v, amp.die_temp_c,
                     (unsigned)amp.bde_level, (unsigned)amp.bde_lowest);

            if (amp.coil_valid) {
                ESP_LOGI(TAG, "AMP: coil=%.2f ohm  coil temp=%.0f C",
                         amp.coil_rdc_ohm, amp.coil_temp_c);
            }

            max98390_log_faults(&amp);
        }
        else {
            ESP_LOGW(TAG, "AMP: status read failed");
        }

        float current_db = -INFINITY;

        if (volume_current_gain > 0.0000001f) {
            current_db = 20.0f * log10f(volume_current_gain);
        }

        ESP_LOGI(TAG,
                 "VOLUME: usb=%" PRIu32 " target=%.2f dB  current=%.2f dB  muted=%" PRIu32,
                 volume_percent, volume_target_db, current_db, volume_muted);

        last_packets = packets;
        last_bytes = bytes;
        last_dropped = dropped;
        last_errors = errors;
        last_short = short_writes;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// ============================================================
// STATUS LED
// ============================================================

static void status_led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << STATUS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&cfg));

    gpio_set_level(STATUS_LED_GPIO, 0);
}


static void status_led_task(void *arg)
{
    (void)arg;

    while (1) {
        gpio_set_level(STATUS_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_set_level(STATUS_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// ============================================================
// APP MAIN
// ============================================================

void app_main(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "ESP32-S3 / MAX98390 USB AUDIO");
    ESP_LOGI(TAG, "======================================");

    // Status LED.
    status_led_init();

    xTaskCreate(
        status_led_task,
        "status_led",
        STATUS_TASK_STACK,
        NULL,
        STATUS_TASK_PRIORITY,
        NULL);

    // Amplifier: reset, I2C, full configuration. Stays off for now.
    const max98390_config_t amp_cfg = {
        .i2c_sda_gpio = I2C_SDA_GPIO,
        .i2c_scl_gpio = I2C_SCL_GPIO,
        .reset_gpio = MAX98390_RESET_GPIO,
        .irq_gpio = MAX98390_IRQ_GPIO,
        .i2c_addr = MAX98390_I2C_ADDR,
    };

    ESP_ERROR_CHECK(max98390_init(&amp_cfg));

    // I2S clocks must run before the amplifier is enabled.
    i2s_init();

    // Speaker model, EQ and limiters.
    speaker_dsp_init();

    volume_init();

    // Audio queue and tasks.
    audio_buffer_init();

    // Amplifier on.
    ESP_ERROR_CHECK(max98390_enable());

    // USB audio.
    usb_audio_init();

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "USB AUDIO READY");
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "High-pass       : %.1f Hz", SPK_HPF_HZ);
    ESP_LOGI(TAG, "Bass split      : %.1f Hz", SPK_SPLIT_HZ);
    ESP_LOGI(TAG, "Cone limit      : %.2f mm", SPK_DRV_XMAX_MM);
    ESP_LOGI(TAG, "Radiator limit  : %.2f mm", SPK_PR_XLIMIT_MM);
    ESP_LOGI(TAG, "Output ceiling  : %.2f V", SPK_OUT_CEILING_V);
    ESP_LOGI(TAG, "Power budget    : %.2f W", SPK_POWER_BUDGET_W);
    ESP_LOGI(TAG, "DSP latency     : %" PRIu32 " samples", speaker_dsp_latency_samples());
    ESP_LOGI(TAG, "Volume range    : %.1f to +%.1f dB", VOLUME_MIN_DB, VOLUME_MAX_DB);
    ESP_LOGI(TAG, "Volume ramp     : %.1f ms", VOLUME_RAMP_MS);
    ESP_LOGI(TAG, "======================================");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
