NAME := AG_WAN


$(NAME)_MBINS_TYPE := app
$(NAME)_VERSION := 1.0.0
$(NAME)_SUMMARY := AG_WAN


$(NAME)_COMPONENTS += 1km_common linkkit_sdk_c 1km_algorithm ota

$(NAME)_SOURCES += app_main.c

GLOBAL_DEFINES += ENABLE_AOS_OTA

GLOBAL_INCLUDES += ./

