idf_component_register(
    SRCS
        "i2s_usb_example_main.c"
        "max98390.c"
        "speaker_dsp.c"
    INCLUDE_DIRS
        "."
        "tusb"
    REQUIRES
        esp_driver_i2s
        esp_driver_i2c
        esp_driver_gpio
        esp_timer
        usb_device_uac
    LDFRAGMENTS
        "linker.lf"
)

# The speaker processing runs floating-point maths on every sample and has
# to finish each millisecond of audio before the next arrives. Build this
# component with -O2 whatever optimization level the project is set to (the
# ESP-IDF default, -Og, makes it several times slower).
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)

# USB speaker + serial log port on the same cable.
#
# With CONFIG_USB_DEVICE_UAC_AS_PART enabled, the usb_device_uac component
# stops providing its own TinyUSB configuration and USB descriptors, and the
# ones in main/tusb are used instead. They add a serial port next to the
# audio interfaces. This follows Espressif's usb_extend_screen example.
#
# With the option disabled, nothing here applies and the device is a plain
# USB speaker with no serial port, as before.
if(CONFIG_USB_DEVICE_UAC_AS_PART)

    # Components fetched by the component manager carry an "espressif__" prefix.
    idf_build_get_property(build_components BUILD_COMPONENTS)

    if(tinyusb IN_LIST build_components)
        set(tinyusb_name tinyusb)
    else()
        set(tinyusb_name espressif__tinyusb)
    endif()

    if(usb_device_uac IN_LIST build_components)
        set(uac_name usb_device_uac)
    else()
        set(uac_name espressif__usb_device_uac)
    endif()

    idf_component_get_property(tusb_lib ${tinyusb_name} COMPONENT_LIB)
    idf_component_get_property(uac_dir ${uac_name} COMPONENT_DIR)

    # TinyUSB must see our tusb_config.h, and the audio headers it includes.
    target_include_directories(${tusb_lib} PUBLIC
        "${COMPONENT_DIR}/tusb"
        "${uac_dir}/tusb_uac")

    # The descriptors and the log port are built as part of TinyUSB.
    target_sources(${tusb_lib} PRIVATE
        "${COMPONENT_DIR}/tusb/usb_descriptors.c"
        "${COMPONENT_DIR}/tusb/usb_serial_log.c")

    # app_main calls usb_serial_log_*(), which now live in the TinyUSB library.
    target_link_libraries(${COMPONENT_LIB} PRIVATE ${tusb_lib})

endif()
