cmake_minimum_required(VERSION 3.13)

set(CONFIG_CHERRYUSB 1)
set(CONFIG_CHERRYUSB_DEVICE 1)
set(CONFIG_USB_DEVICE 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(pcan_hpm5300)

# PCAN-USB is a full-speed device; force the EHCI-device PFSC bit so the host
# enumerates at 12 Mbps and the FS bulk maxpacket sizes (16/64) stay legal.
sdk_compile_definitions(-DCONFIG_USB_DEVICE_FS=1)

# make usb_config.h visible to the CherryUSB middleware compilation units
sdk_inc(src)
sdk_app_inc(src)
# platform-independent PCAN-USB protocol layer headers (pcan_packet.h, punker.h)
sdk_app_inc(src/protocol)

sdk_app_src(src/main.c)
sdk_app_src(src/pcan_usb.c)
sdk_app_src(src/pcan_timestamp.c)
sdk_app_src(src/pcan_led.c)
sdk_app_src(src/pcan_can.c)
sdk_app_src(src/protocol/pcan_protocol.c)

generate_ide_projects()
