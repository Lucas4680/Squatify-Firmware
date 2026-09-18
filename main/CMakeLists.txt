idf_component_register(
    SRCS "i2s_usb_example_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_i2s
        esp_driver_i2c
        esp_driver_gpio
        usb_device_uac
)