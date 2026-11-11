// USB descriptors for the combined device: USB audio plus a serial port
// for the log.
//
// The audio part is the usb_device_uac component's own descriptor macro, so
// the speaker and microphone appear exactly as they did before. A serial
// port (CDC ACM) is added after it.
//
// This file is compiled as part of the TinyUSB component (see
// main/CMakeLists.txt) and only when CONFIG_USB_DEVICE_UAC_AS_PART is on.

#include <string.h>

#include "tusb.h"

#include "uac_descriptors.h"
#include "usb_interfaces.h"


// ============================================================
// IDENTITY
// ============================================================
//
// The product ID differs from the audio-only firmware (0x8000) on purpose.
// Windows remembers the driver setup for each ID, and a device that changes
// its set of interfaces under the same ID can fail to load its drivers.

#define USB_VID             0x303A      // Espressif
#define USB_PID             0x4011      // audio + serial port
#define USB_BCD_DEVICE      0x0100

#define USB_MANUFACTURER    "Espressif"
#define USB_PRODUCT         "ESP UAC Device"
#define USB_SERIAL_NUMBER   "12345678"


// ============================================================
// DEVICE DESCRIPTOR
// ============================================================

static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // A device made of several functions grouped by interface association
    // descriptors must report this class / subclass / protocol.
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = USB_BCD_DEVICE,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};


uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}


// ============================================================
// STRINGS
// ============================================================
//
// The audio descriptor macro uses three consecutive string indexes starting
// at STRID_AUDIO: control, first streaming interface, second streaming
// interface.

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_AUDIO,
#if SPEAK_CHANNEL_NUM
    STRID_SPEAKER,
#endif
#if MIC_CHANNEL_NUM
    STRID_MICROPHONE,
#endif
    STRID_CDC,
    STRID_COUNT
};

static char const *const string_desc_arr[STRID_COUNT] = {
    [STRID_LANGID]       = (const char[]) { 0x09, 0x04 },   // English (0x0409)
    [STRID_MANUFACTURER] = USB_MANUFACTURER,
    [STRID_PRODUCT]      = USB_PRODUCT,
    [STRID_SERIAL]       = USB_SERIAL_NUMBER,
    [STRID_AUDIO]        = "usb uac",
#if SPEAK_CHANNEL_NUM
    [STRID_SPEAKER]      = "speaker",
#endif
#if MIC_CHANNEL_NUM
    [STRID_MICROPHONE]   = "microphone",
#endif
    [STRID_CDC]          = "Speaker log port",
};

static uint16_t desc_str[32];


uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    uint8_t chr_count;

    if (index == STRID_LANGID) {
        memcpy(&desc_str[1], string_desc_arr[STRID_LANGID], 2);
        chr_count = 1;
    }
    else {
        if (index >= STRID_COUNT) {
            return NULL;
        }

        const char *str = string_desc_arr[index];

        chr_count = (uint8_t)strlen(str);

        if (chr_count > 31) {
            chr_count = 31;
        }

        for (uint8_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = (uint16_t)str[i];
        }
    }

    // First byte is the length including this header, second is the type.
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return desc_str;
}


// ============================================================
// CONFIGURATION DESCRIPTOR
// ============================================================

#define CONFIG_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + TUD_AUDIO_DEVICE_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_configuration[] = {

    // Config number, interface count, string index, total length,
    // attributes, power in mA.
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // Audio: first interface, string index, speaker OUT endpoint,
    // microphone IN endpoint, speaker feedback endpoint.
    TUD_AUDIO_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, STRID_AUDIO,
                         EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN, EPNUM_AUDIO_FB),

    // Serial port: first interface, string index, notification endpoint and
    // its size, data OUT endpoint, data IN endpoint, data endpoint size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC,
                       EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
                 "configuration descriptor length mismatch");


uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;

    return desc_configuration;
}
