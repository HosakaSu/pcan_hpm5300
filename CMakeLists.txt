cmake_minimum_required(VERSION 3.13)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(pcan_hpm5300)

sdk_app_inc(src)
sdk_app_src(src/main.c)

generate_ide_projects()
