// Sends the ESP log to the PC over the USB serial port that sits next to
// the USB speaker on the same cable.
//
// The ESP32-S3 has one USB connection. Once the firmware turns it into a
// USB speaker, the chip's built-in serial/JTAG port is gone, so the log
// would otherwise only be visible on the UART0 pin. With this, the device
// shows up as a speaker plus a COM port, and any serial monitor (for
// example "ESP-IDF: Monitor Device" in VS Code) shows the log.
//
// The normal console output on UART0 keeps working as before.
//
// Only available when CONFIG_USB_DEVICE_UAC_AS_PART is enabled.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Start capturing log output. Call first thing in app_main, before anything
// worth reading is logged. Nothing is sent over USB yet: the first few
// kilobytes are kept in memory and replayed when a terminal connects, so
// the start-up messages are not lost.
void usb_serial_log_install(void);

// Tell the logger that the USB stack is running. Call after
// uac_device_init(). From here on, log output is also sent to the USB
// serial port.
void usb_serial_log_usb_started(void);

#ifdef __cplusplus
}
#endif
