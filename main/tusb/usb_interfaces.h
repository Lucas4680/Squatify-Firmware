// USB interface and endpoint numbers for the combined device:
// USB speaker (and microphone, if enabled) plus a serial port for the log.
//
// Only used when CONFIG_USB_DEVICE_UAC_AS_PART is enabled.

#pragma once

#include "sdkconfig.h"


// ============================================================
// INTERFACES
// ============================================================
//
// The audio interfaces must come first and stay in this order: the audio
// descriptor macro from the usb_device_uac component numbers them as
// control, control + 1, control + 2.

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
#if CONFIG_UAC_SPEAKER_CHANNEL_NUM > 0
    ITF_NUM_AUDIO_STREAMING_SPK,
#endif
#if CONFIG_UAC_MIC_CHANNEL_NUM > 0
    ITF_NUM_AUDIO_STREAMING_MIC,
#endif
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};


// ============================================================
// ENDPOINTS
// ============================================================
//
// The ESP32-S3 USB controller has four IN endpoints besides endpoint 0,
// and an IN endpoint must use number 1 to 4. This layout uses all four:
// speaker feedback, microphone, serial notification and serial data.

#define EPNUM_AUDIO_OUT     0x01    // speaker data, host to device
#define EPNUM_AUDIO_FB      0x81    // speaker rate feedback
#define EPNUM_AUDIO_IN      0x82    // microphone data
#define EPNUM_CDC_NOTIF     0x83    // serial port notifications
#define EPNUM_CDC_OUT       0x04    // serial data, host to device
#define EPNUM_CDC_IN        0x84    // serial data, device to host
