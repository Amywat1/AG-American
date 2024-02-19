/*
 project name: AG
 create time: 2022-11-29 18:02:59
 desined by lunanting
 From 1KM Co,Ltd
*/


#include <stdio.h>
#include <aos/kernel.h>
#include "boardio.h"
#include "aos/kv.h"
#include "fsl_lpuart.h"
#include "MIMXRT1052.h"
#include "board.h"
#include "bsp.h"
#include "mqtt.h"
#include "remote_cmd.h"
#include "../../../1km/common/boardio.h"

#define APP_name "AG"
#define APP_version "AG-0.0.7"

static char product_key[]   ="k01omUSTr04";         // product_key 和 product_secret 匹配对应产品
static char product_secret[]="dlStncmKY3ja7AXY";

/**************************ota version*********************/
const char *ota_hal_version(unsigned char dev_type, char* dn)
{
    if(dev_type > 0){
        return "v1.0.0-20180101-1000";//SYSINFO_APP_VERSION;
    }
    else{
        return APP_version;
    }
}

/******************app debug***************/
extern int xp_service_debug(char* type, char* fun, char* param);
extern int xp_module_debug(char* type, char* fun, char* param);
extern int osal_debug(char* type, char* fun, char* param);

int app_debug(char *type,char *fun,char *param){
    if(xp_mqtt_debug(type,fun,param)){}             //先mqtt，在xp_module_debug会分割下划线导致mqtt配置有下划线的参数时有问题
	else if(xp_service_debug(type,fun,param)){}
    else if(osal_debug(type,fun,param)){}
	else if(xp_module_debug(type,fun,param)){}
	else if(strcmp(type,"hardware") == 0){
		println("Hardware: %s %s.", APP_name, APP_version);
	}
	else{
		return 0;
	}

	return 1;
}

/**********************user code*************************/
int application_start(int argc, char *argv[])
{
    /* ==================== 该段为了配置蓝牙名字和波特率 ====================*/
    LPUART_SetBaudRate(LPUART1, 9600, BOARD_DebugConsoleSrcFreq());
    //用\r\n或者只发送一两次配置不成功，原因未知
    printf("AT+NAMEYGL\n");
    printf("AT+NAMEYGL\n");
    printf("AT+NAMEYGL\n");
    printf("AT+NAMEYGL\n");
    printf("AT+NAMEYGL\n");
    aos_msleep(10);
    printf("AT+BAUD8\n");
    printf("AT+BAUD8\n");
    printf("AT+BAUD8\n");
    printf("AT+BAUD8\n");
    printf("AT+BAUD8\n");
    printf("AT+BAUD8\n");
    LPUART_SetBaudRate(LPUART1, 115200, BOARD_DebugConsoleSrcFreq());
    /* ==================== 该段为了配置蓝牙名字和波特率 ====================*/
    xp_cli_init();
    xp_wellcome_worlds(APP_name, APP_version);
    xp_rtc_init();
    xp_boardIO_init(1);
    extern void set_app_version(char *version);
    set_app_version(APP_version);
    extern int xp_osal_init(void);
    xp_osal_init();
    extern int xp_error_state_init(void);
    // xp_error_state_init();
    extern int xp_service_init(void);
    xp_service_init();
    extern int xp_net_init(void);           //最后初始化net，回调函数赋值
    xp_net_init();
    while(1)
    {
        aos_msleep(1000);
    }
}
