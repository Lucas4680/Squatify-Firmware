// Log output over the USB serial port: see usb_serial_log.h.
//
// This file is compiled as part of the TinyUSB component (see
// main/CMakeLists.txt) and only when CONFIG_USB_DEVICE_UAC_AS_PART is on.

#include "usb_serial_log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "tusb.h"


// Longest piece of log text handled in one go. Longer text is cut off on
// the USB port only; UART0 always gets the full text.
#define LINE_BUF_SIZE       384

// How much start-up log is kept for replay. It has to fit in the serial
// transmit buffer together with the two banner lines.
#define BOOT_LOG_SIZE       3072

TU_VERIFY_STATIC(BOOT_LOG_SIZE + 512 <= CFG_TUD_CDC_TX_BUFSIZE,
                 "boot log must fit in the CDC transmit buffer");

// How long a task may wait for another task that is writing to the port.
#define WRITE_LOCK_WAIT_MS  20


static vprintf_like_t prev_vprintf = NULL;

static SemaphoreHandle_t write_lock = NULL;

static volatile bool usb_started = false;

static char boot_log[BOOT_LOG_SIZE];
static size_t boot_len = 0;
static portMUX_TYPE boot_mux = portMUX_INITIALIZER_UNLOCKED;

static bool terminal_connected = false;


// ============================================================
// WRITE TO THE SERIAL PORT
// ============================================================

// Queue text for the PC, turning "\n" into "\r\n" so plain terminals show
// it correctly. Never blocks: when the PC is not reading, TinyUSB keeps
// only the newest text.
static void cdc_write_text(const char *text, size_t len)
{
    size_t start = 0;

    for (size_t i = 0; i < len; i++) {

        if (text[i] != '\n') {
            continue;
        }

        if (i > start) {
            tud_cdc_write(text + start, (uint32_t)(i - start));
        }

        tud_cdc_write("\r\n", 2);

        start = i + 1;
    }

    if (len > start) {
        tud_cdc_write(text + start, (uint32_t)(len - start));
    }

    tud_cdc_write_flush();
}


static bool write_lock_take(void)
{
    if (write_lock == NULL) {
        return false;
    }

    return xSemaphoreTake(write_lock, pdMS_TO_TICKS(WRITE_LOCK_WAIT_MS)) == pdTRUE;
}


static void write_lock_give(void)
{
    xSemaphoreGive(write_lock);
}


// ============================================================
// LOG HOOK
// ============================================================

// Installed with esp_log_set_vprintf(), so every ESP_LOGx call ends up here.
// It may be called from several tasks at once.
static int log_vprintf(const char *fmt, va_list args)
    __attribute__((format(printf, 1, 0)));

static int log_vprintf(const char *fmt, va_list args)
{
    va_list copy;

    va_copy(copy, args);

    // The normal console (UART0) first, exactly as before.
    const int ret = (prev_vprintf != NULL) ? prev_vprintf(fmt, args) : vprintf(fmt, args);

    // The rest takes locks, which is not allowed inside an interrupt.
    if (xPortInIsrContext()) {
        va_end(copy);
        return ret;
    }

    char line[LINE_BUF_SIZE];

    int len = vsnprintf(line, sizeof(line), fmt, copy);

    va_end(copy);

    if (len <= 0) {
        return ret;
    }

    if (len >= (int)sizeof(line)) {
        len = (int)sizeof(line) - 1;
    }

    // Keep the first part of the log for replay.
    portENTER_CRITICAL(&boot_mux);

    if (boot_len < BOOT_LOG_SIZE) {

        size_t n = (size_t)len;

        if (n > BOOT_LOG_SIZE - boot_len) {
            n = BOOT_LOG_SIZE - boot_len;
        }

        memcpy(boot_log + boot_len, line, n);
        boot_len += n;
    }

    portEXIT_CRITICAL(&boot_mux);

    // Live output, once the USB stack is up and the PC has configured us.
    if (usb_started && tud_ready() && write_lock_take()) {
        cdc_write_text(line, (size_t)len);
        write_lock_give();
    }

    return ret;
}


// ============================================================
// TINYUSB CALLBACKS
// ============================================================

// Called by TinyUSB when the PC opens or closes the port (DTR line).
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)rts;

    if (dtr && !terminal_connected && write_lock_take()) {

        static const char banner_start[] = "\n--- start-up log ---\n";
        static const char banner_end[] = "\n--- live log ---\n";

        // Whatever is still queued was written while nobody was listening.
        tud_cdc_write_clear();

        cdc_write_text(banner_start, sizeof(banner_start) - 1);

        portENTER_CRITICAL(&boot_mux);
        const size_t n = boot_len;
        portEXIT_CRITICAL(&boot_mux);

        // boot_log is only ever appended to, so the first n bytes are stable.
        cdc_write_text(boot_log, n);

        cdc_write_text(banner_end, sizeof(banner_end) - 1);

        write_lock_give();
    }

    terminal_connected = dtr;
}


// Called by TinyUSB when the PC sends data. Nothing is done with input.
void tud_cdc_rx_cb(uint8_t itf)
{
    tud_cdc_n_read_flush(itf);
}


// ============================================================
// API
// ============================================================

void usb_serial_log_install(void)
{
    if (write_lock != NULL) {
        return;
    }

    write_lock = xSemaphoreCreateMutex();

    prev_vprintf = esp_log_set_vprintf(log_vprintf);
}


void usb_serial_log_usb_started(void)
{
    usb_started = true;
}
