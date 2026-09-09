cmake_minimum_required(VERSION 3.13)

set(CONFIG_CHERRYUSB 1)
set(CONFIG_CHERRYUSB_DEVICE 1)
set(CONFIG_USB_DEVICE 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(pcan_hpm5300)

# make usb_config.h visible to the CherryUSB middleware compilation units
sdk_inc(src)
sdk_app_inc(src)

sdk_app_src(src/main.c)
sdk_app_src(src/pcan_usb.c)
sdk_app_src(src/pcan_timestamp.c)
sdk_app_src(src/pcan_led.c)

generate_ide_projects()
