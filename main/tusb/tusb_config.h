// TinyUSB configuration for the combined device: USB audio plus a serial
// port for the log.
//
// This replaces the tusb_config.h that the usb_device_uac component uses
// on its own, and is only used when CONFIG_USB_DEVICE_UAC_AS_PART is
// enabled. The audio settings still come from the component's headers, so
// channel count, sample rate and bit depth keep following menuconfig.

#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"

// From the usb_device_uac component (its tusb_uac directory).
#include "uac_config.h"
#include "uac_descriptors.h"
#include "tusb_config_uac.h"


// ============================================================
// PORT
// ============================================================

#ifdef CONFIG_TINYUSB_RHPORT_HS
#if CONFIG_IDF_TARGET_ESP32P4
#define CFG_TUSB_RHPORT1_MODE   (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#else
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
#endif
#else
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#endif


// ============================================================
// COMMON
// ============================================================

// Set by the TinyUSB component's build flags.
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS             OPT_OS_FREERTOS
#endif

#ifndef ESP_PLATFORM
#define ESP_PLATFORM            1
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG          0
#endif

// ESP-IDF needs the "freertos/" prefix on FreeRTOS includes.
#if TU_CHECK_MCU(OPT_MCU_ESP32S2, OPT_MCU_ESP32S3, OPT_MCU_ESP32P4)
#define CFG_TUSB_OS_INC_PATH    freertos/
#endif

#define CFG_TUD_ENABLED         1

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN      __attribute__ ((aligned(4)))
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE  64
#endif


// ============================================================
// SERIAL PORT (CDC)
// ============================================================
//
// One serial port, used to send the log to the PC. The transmit buffer is
// large enough to hold the start-up messages that are replayed when a
// terminal connects (see usb_serial_log.c).

#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  64
#define CFG_TUD_CDC_TX_BUFSIZE  4096
#define CFG_TUD_CDC_EP_BUFSIZE  64

#ifdef __cplusplus
}
#endif

#endif // TUSB_CONFIG_H_
