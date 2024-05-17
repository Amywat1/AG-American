/*
 project name: check_main
 create time: 2022-02-15 16:58:14
 desined by lunanting
 From 1KM Co,Ltd
*/


#include <stdio.h>
#include <aos/kernel.h>
#include "boardio.h"
#include "aos/kv.h"
#include "bsp.h"
#include "mqtt.h"
#include "remote_cmd.h"
#include "voice.h"
#include "stdlib.h"
#include "app/log_level.h"
#include "fsl_lpuart.h"

// #define APP_name "check_main"
// #define APP_version "check_main-0.0.1"


/********************************************** ��������� ***************************************/
#define CHECK_times     5       //���ܲ��Դ���
#define CHECK_adc_v     3.3     //adc ���ĵ�ѹ
#define CHECK_adc_v2    5.0     //adc ���ĵ�ѹ




/************************************************************************************************/

static char product_key[]   ="a19lUJsZzJJ";         // ���豸ר����װ����
static char product_secret[]="vf3MEpB9urLeK8cl";
static char device_name[]   ="check-02";
static char device_secret[] ="dded29906a85569902e28a2848920d07";

static MQTT_class mqtt={0};

void xp_port_check(void);
int ec20_rssi_get(void);

/**************************ota version*********************/
// const char *ota_hal_version(unsigned char dev_type, char* dn)
// {
//     if(dev_type > 0){
//         return "v1.0.0-20180101-1000";//SYSINFO_APP_VERSION;
//     }
//     else{
//         return APP_version;
//     }
// }


static char record_device_name[50] = {0};
static char record_device_secret[50] = {0};
static char value[50] = {0};
static int  len = 50;
/**********************user code*************************/
int xp_board_check(void)
{
    bool isExitDevName = false;
    bool isExitDevSecret = false;

    //��ȡ��ǰ�豸����Կֵ��������ָ���������������
    memset(value, 0, strlen(value));
    if(0 == aos_kv_get("device_name", value, &len)){
        isExitDevName = true;
        println("Dev exist device_name: %s", value);
        strcpy(record_device_name, value);
    }
    len = 50;           //��ȡkvʱ�����������Ҫ���ڻ�ȡ�ĳ���
    memset(value, 0, strlen(value));
    if(0 == aos_kv_get("device_secret", value, &len)){
        isExitDevSecret = true;
        println("Dev exist device_secret: %s", value);
        strcpy(record_device_secret, value);
    }
    aos_kv_set("product_key",    product_key,    strlen(product_key), 1);
    aos_kv_set("product_secret", product_secret, strlen(product_secret), 1);
    aos_kv_set("device_name",    device_name,    strlen(device_name), 1);
    aos_kv_set("device_secret",  device_secret,  strlen(device_secret), 1);
    xp_mqtt_init(&mqtt);
    aos_msleep(1000);
    xp_port_check();
    //��Ʒ��Կ��app_main�����������
    if(isExitDevName)   aos_kv_set("device_name",    record_device_name,    strlen(record_device_name), 1);
    if(isExitDevSecret) aos_kv_set("device_secret",  record_device_secret,  strlen(record_device_secret), 1);
    while(1){
        // LOG_UPLOAD("4G rssi: %d.",ec20_rssi_get());
        aos_msleep(60000);
        return 0;
    }
    // aos_reboot();
}

/*
rs485 port check
return:
    0 =success
    -1=fail
*/
int xp_rs485_check(void){

    // Init_LPUART(2, 115200, 0);
    // Init_LPUART(4, 115200, 0);

    // LPUART_recv_enable(2, true);
    // LPUART_recv_enable(4, true);

    // char *msg = "hello";
    // push_to_tx_ringbuffer(4, msg, strlen(msg));
    // flush_tx_ringbuffer(4);
    // aos_msleep(50);

    // uint8_t *data;
    // size_t len = 0;

    // pop_from_rx_ringbuffer(2, &data, &len);
    // if (len){
    //     println("uart 1 recv %s", data);
    //     push_to_tx_ringbuffer(2);
    //     aos_msleep(50);
    // }else{
    //     println("uart 1 not recv");
    //     return -3;
    // }

    // pop_from_rx_ringbuffer(4, &data, &len);
    // if (len){
    //     println("uart 2 recv %s", data);
    //     push_to_tx_ringbuffer(4);
    //     aos_msleep(50);
    // }else{
    //     println("uart 2 not recv");
    //     return -4;
    // }


    int sta=0;
    uart_dev_t uart1={0},uart2={0};
    char *str="hello turbo";
    char buf[100]={0};
    int rec_len=0;

    uart1.port=1;
    uart1.config.baud_rate=115200;
    uart1.config.data_width=DATA_WIDTH_8BIT;
    uart1.config.parity=NO_PARITY;
    uart1.config.stop_bits=STOP_BITS_1;
    uart1.config.flow_control=FLOW_CONTROL_DISABLED;
    uart1.config.mode=MODE_TX_RX;

    memcpy(&uart2,&uart1,sizeof(uart_dev_t));
    uart2.port=2;

    hal_uart_finalize(&uart1);
    hal_uart_finalize(&uart2);
    if(hal_uart_init(&uart1)!=0){
        return -1;
    }
    if(hal_uart_init(&uart2)!=0){
        return -2;
    }
    xp_uart_rec_clear(&uart1);
    xp_uart_rec_clear(&uart2);

    for(int i=0;i<CHECK_times;i++){
        memset(buf,0,strlen(buf));
        hal_uart_send(&uart1,str,strlen(str),100);
        hal_uart_recv_II(&uart2,buf,strlen(str),&rec_len,100);
        println("%d RS485 2 rec= %s.",i,buf);
        if(strncmp(buf,str,strlen(str))!=0){
            return -3;
        }
        memset(buf,0,strlen(buf));
        hal_uart_send(&uart2,str,strlen(str),100);
        hal_uart_recv_II(&uart1,buf,strlen(str),&rec_len,100);
        println("%d RS485 1 rec= %s.",i,buf);
        if(strncmp(buf,str,strlen(str))!=0){
            return -4;
        }
        aos_msleep(10);
    }
    return 0;
}



/*
can port check
return:
    0 =success
    -1=fail
*/
int xp_can_check(void){
    CAN_dev can1,can2;
    can_frame_t can_frame={0};
    char *str1="hello",*str2="world";
    int count=10;

    if(xp_can_device_regist(&can1,101,1)!=0){
        return -1;
    }
    if(xp_can_device_regist(&can2,100,2)!=0){
        return -2;
    }

    for(int i=0;i<CHECK_times;i++){
        xp_can_send(can1.bus,0,str1,strlen(str1));
        while(count--){
            memset(&can_frame,0,sizeof(can_frame_t));
            xp_can_recive(can2.bus,100,&can_frame);
            if(can_frame.header.id==100)break;
        }
        println("%d can2 recive: id= %d, len= %d, data= %s.",i,can_frame.header.id,can_frame.header.dlc,can_frame.data);
        if(strcmp(can_frame.data,str1)!=0){
            return -3;
        }

        xp_can_send(can2.bus,0,str2,strlen(str2));
        count=10;
        while(count--){
            memset(&can_frame,0,sizeof(can_frame_t));
            xp_can_recive(can1.bus,100,&can_frame);
            if(can_frame.header.id==101)break;
        }
        println("%d can1 recive: id= %d, len= %d, data= %s.",i,can_frame.header.id,can_frame.header.dlc,can_frame.data);
        if(strcmp(can_frame.data,str2)!=0){
            return -3;
        }
        aos_msleep(10);
    }

    return 0;
}


/*
analog port check
return:
    0 =success
    -1=fail
*/
int xp_analog_check(void){
    float adc_v[5]={0};
    int count=1000;

    xp_adc_init(4);
    aos_msleep(500);
    while(--count){
        adc_v[0]=xp_adc_data_get(adc_channel_0)->data;
        adc_v[1]=xp_adc_data_get(adc_channel_1)->data;
        adc_v[2]=xp_adc_data_get(adc_channel_2)->data;
        adc_v[3]=xp_adc_data_get(adc_channel_3)->data;
        if(adc_v[0]>0.001&&adc_v[1]>0.001&&adc_v[2]>0.001&&adc_v[3]>0.001){
            println("adc adpate: %0.2fv %0.2fv %0.2fv %0.2fv",adc_v[0],adc_v[1],adc_v[2],adc_v[3]);
            break;
        }
        aos_msleep(10);
    }
    if(count==0){
        println("adc adpate: %0.2fv %0.2fv %0.2fv %0.2fv",adc_v[0],adc_v[1],adc_v[2],adc_v[3]);
        println("adc adpate fail.");
        return -1;
    }
    for(int i=0;i<CHECK_times;i++){
        adc_v[0]=xp_adc_data_get(adc_channel_0)->data;
        adc_v[1]=xp_adc_data_get(adc_channel_1)->data;
        adc_v[2]=xp_adc_data_get(adc_channel_2)->data;
        adc_v[3]=xp_adc_data_get(adc_channel_3)->data;
        adc_v[4]=adc_v[0]+adc_v[1]+adc_v[2]+adc_v[3];
        adc_v[4]=adc_v[4]/4;
        println(" %d adc analog: %0.2fv, %0.2fv, %0.2fv, %0.2fv    average= %0.2fv",i,adc_v[0],adc_v[1],adc_v[2],adc_v[3],adc_v[4]);
        // if(adc_v[4]<(CHECK_adc_v-0.1)||adc_v[4]>(CHECK_adc_v+0.1)){
        //     return -1;
        // }
        if(abs((CHECK_adc_v2-adc_v[0])*10)>2||abs((CHECK_adc_v2-adc_v[1])*10)>2||abs((CHECK_adc_v-adc_v[2])*10)>2||abs((CHECK_adc_v-adc_v[3])*10)>2){
            return -1;
        }
        aos_msleep(10);
    }
    return 0;
}

/*
board io check
return:
    0 =success
    -1=fail
*/
int xp_boardio_check(void){

    for(int i=0;i<CHECK_times;i++){
        //open
        xp_boardIO_set(1,0);
        aos_msleep(20);
        if(xp_boardIO_get(1)!=0){
            return -1;
        }
        xp_boardIO_set(1,1);
        aos_msleep(20);
        if(xp_boardIO_get(1)!=1){
            return -1;
        }

        xp_boardIO_set(2,0);
        aos_msleep(20);
        if(xp_boardIO_get(2)!=0){
            return -2;
        }
        xp_boardIO_set(2,1);
        aos_msleep(20);
        if(xp_boardIO_get(2)!=1){
            return -2;
        }
        xp_boardIO_set(3,0);
        aos_msleep(20);
        if(xp_boardIO_get(3)!=0){
            return -3;
        }
        xp_boardIO_set(3,1);
        aos_msleep(20);
        if(xp_boardIO_get(3)!=1){
            return -3;
        }
        xp_boardIO_set(4,0);
        aos_msleep(20);
        if(xp_boardIO_get(4)!=0){
            return -4;
        }
        xp_boardIO_set(4,1);
        aos_msleep(20);
        if(xp_boardIO_get(4)!=1){
            return -4;
        }
        aos_msleep(50);
        //close
        println("%d board io 1~4 open and close is right",i);
        aos_msleep(200);
    }
    return 0;
}


/*
4g module network check
return:
    0 =success
    -1=fail
*/
int xp_4gNetwork_check(void){

    u64 time_last;

    time_last=aos_now_ms();
    while((aos_now_ms()-time_last)<30000){
        if(mqtt.online){
            aos_msleep(3000);
            return 0;
        }
        aos_msleep(100);
    }
    return -1;
}



/*
sd card check
param:
    type: 0=play fail music,1=play success music
return:
    0 =success
    -1=fail
*/
int xp_sdcard_check(int type){
    char file[100];

    if(type==0){
        sprintf(file,"fs:/sdcard/ok.mp3");
    }
    else{
        sprintf(file,"fs:/sdcard/x.mp3");
    }
    xp_voice_clr_source();
    if(xp_voice_start(file,80)!=0){
        return -1;
    }
    return 0;
}



/*
port check
*/
void xp_port_check(void){
    int sta=0,ret=0;
    println("��ʼ���˿ڹ���....\n\n");

    println("���ڼ�� RS485 1,2 �˿�:");
    sta=xp_rs485_check();
    ret|=sta;
    if(sta!=0){
        println("rs485 port not normal ! ret= %d.",sta);
        if(sta==-3){
            println("RS485 �˿�2���ղ����˿�1���͵�����!\n\n");
        }
        else if(sta==-4){
            println("RS485 �˿�1���ղ����˿�2���͵�����!\n\n");
        }
        else if(sta==-1){
            println("RS485 �˿�1���ڳ�ʼ��������!\n\n");
        }
        else if(sta==-2){
            println("RS485 �˿�2���ڳ�ʼ��������!\n\n");
        }

        // goto FAIL;
    }
    else{
        println("RS485 �˿� %d �λ�������, ���ȫ������ !!\n\n",CHECK_times);
    }

    println("���ڼ�� CAN 1,2 �˿�:");
    sta=xp_can_check();
    ret|=sta;
    if(sta!=0){
        println("can port not normal ! ret= %d.",sta);
        if(sta==-1){
            println("CAN �˿�1��ʼ�������� !\n\n");
        }
        else if(sta==-2){
            println("CAN �˿�2��ʼ�������� !\n\n");
        }
        else if(sta==-3){
            println("CAN �˿�2���ն˿�1���ݲ����� !\n\n");
        }
        else if(sta==-4){
            println("CAN �˿�1���ն˿�2���ݲ����� !\n\n");
        }
        // goto FAIL;
    }
    else{
        println("CAN �˿� %d �λ�������, ���ȫ������ !!\n\n",CHECK_times);
    }

    println("���ڼ�� ADC 4 ��ͨ��, ���Ե�ѹΪ %0.2fv, %0.2fv",CHECK_adc_v,CHECK_adc_v2);
    sta=xp_analog_check();
    ret|=sta;
    if(sta!=0){
        println("adc port not normal !, ret= %d.",sta);
        println("ADC �˿ڼ�ⲻ���� !!!!!\n\n");
    }
    else{
        println("ADC �����˿� %d ��, ���ȫ������ !\n\n",CHECK_times);
    }

    println("���ڼ��board io 1~4 �˿�:");
    sta=xp_boardio_check();
    ret|=sta;
    if(sta!=0){
        println("Board io port not normal !, ret= %d.",sta);
        println("board io %d ������������������ !\n\n",0-sta);
    }
    else{
        println("Board io 1~4�˿����������� %d ��, ���ȫ������ !\n\n",CHECK_times);
    }

    println("���ڼ�� 4Gģ����������:");
    sta=xp_4gNetwork_check();
    ret|=sta;
    if(sta!=0){
        println("4G module can't not connect nework !");
        println("4G ģ�鲻������,����������,���߼���������Ƿ����� !!!!\n\n");
    }
    else{
        println("4G ��������,ģ�鹤������ !!!!\n\n");
    }


    println("���ڼ�� sdcard �ļ�ϵͳ����Ƶ���Ŷ˿�:");
    sta=xp_sdcard_check(ret);
    ret|=sta;
    aos_msleep(100);
    if(sta!=0){
        println("sdcar file system not normal !");
        println("sdcard �ļ�ϵͳ����������sd����û�в��ŵ���Ƶ�ļ�,���� ~\n\n");
    }
    else{
        println("sdcard �ļ�ϵͳ����, ��ע���Ƿ��в������� ~\n\n");
    }


    if(ret!=0){
        goto FAIL;
    }
    aos_msleep(1000);
    println("\n\n\n\n ������,���м�⹦��ȫ������ !! OK OK OK\n\n\n\n");
    return;
FAIL:
    println("\n\n\n\n ������,����豸�в������˿� ! ��ϸ������~  X X X X X \n\n\n\n");
    return;
}


