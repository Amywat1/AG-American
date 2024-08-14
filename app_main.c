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
#include "app/ag_service.h"

#define APP_name "AG"
#define APP_version "AG_WAN-5.2.0"                     // 首位0~4开头为国内版本，5~8为海外版本（有线版本均带WAN，如：AG_WAN-5.0.1）

// static char product_key[]   ="g8wlob1ucMb";         // product_key 和 product_secret 匹配对应产品
// static char product_secret[]="PuPs5ZNw3rTOrFtc";

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
    //上电检测符合相关逻辑则进入检测程序，否则进入应用程序
    //进入测试程序逻辑：初始DI01，DI02高电平、DI03，DI04输出都是低电平，播报语音后3S内DI01，DI02、DI03，DI04均为高电平
    if(1 == xp_boardIO_get(1) && 1 == xp_boardIO_get(2) && 0 == xp_boardIO_get(3) && 0 == xp_boardIO_get(4)){
        xp_voice_init();
        aos_msleep(500);
        char *file = aos_malloc(50);
        sprintf(file,"fs:/sdcard/tips.mp3");
        xp_voice_clr_source();
        xp_voice_start(file,80);
        aos_free(file);
        uint8_t cnt = 0;
        while (cnt++ < 10)
        {
            if(1 == xp_boardIO_get(1) && 1 == xp_boardIO_get(2) && 1 == xp_boardIO_get(3) && 1 == xp_boardIO_get(4)){
                xp_board_check();
                aos_reboot();
            }
            aos_msleep(300);
        }
    }
    set_app_version(APP_version);
    extern int xp_osal_init(void);
    xp_osal_init();
    extern int xp_error_state_init(void);
    xp_error_state_init();
    extern int xp_service_init(void);
    xp_service_init();
    // aos_kv_set("product_key",    product_key,    strlen(product_key), 1);
    // aos_kv_set("product_secret", product_secret, strlen(product_secret), 1);
    extern int xp_net_init(void);           //最后初始化net，回调函数赋值
    xp_net_init();
    while(1)
    {
        aos_msleep(1000);
    }
}
