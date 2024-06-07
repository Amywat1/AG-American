/**
 * @file 	 ag_net.c
 * @brief       
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 * 
 * @copyright Copyright (c) 2022  YGL
 * 
 */

#include "../../../../include/aos/kernel.h"
#include "../../../../1km/common/mqtt.h"
#include "ag_net.h"
#include "ag_err_state.h"
#include "../../../../utility/cjson/include/cJSON.h"
#include "log_level.h"
#include "ag_service.h"

#define IOT_ADD_PROPERTY_NODE(identifier, data, data_type, diff_range, cmd_id)      Iot_add_property_node(identifier, &data, data_type, sizeof(data), diff_range, cmd_id)
#define IOT_ADD_PROPERTY_NODE_STR(identifier, data, data_type, diff_range, cmd_id)  Iot_add_property_node(identifier, data, data_type, sizeof(data), diff_range, cmd_id)

static MQTT_class   mqtt            = {0};
static slist_t      list_property   = {0};
uint32_t autoReportIntervalMs = DEFAULT_AUTO_REPORT_INTERVAL_MS;

static Type_ModelSts_Def *localModelSts = NULL;
static Type_ModelCmd_Def *localModelCmd = NULL;

uint8_t repeatLogCnt = 0;
static char logBuf[1024] = {0};
static char recordLogStr[1024] = {0};
static char logSendBuf[1024] = {0};
static bool isSync = false;                 //是否在同步状态

void xp_iot_auto_report_thread(void *arg);
void xp_mqtt_heart_thread(void *arg);
void xp_iot_data_register_thread(void *arg);
void xp_log_upload_thread(void *arg);
int32_t Iot_auto_property_batch_report_polling(void);

/**
 * @brief       错误上报回调
 * @param[in]	buf                 
 * @return      int                 
 */
int xp_err_upload_callback(char *buf)
{
    uint8_t retry_cnt = 1;

    while(retry_cnt--){
        if(xp_mqtt_post_property(&mqtt, buf) >= 0){
            return 0;
        }
        LOG_WARN("xp error post message fail , retry remian cnt %d!, ret %d", retry_cnt);
        aos_msleep(200);
	}
    return -1;
}

/**
 * @brief       远程指令执行完成回调
 * @param[in]	identifier          点位标识符
 * @param[in]	value               点位值
 * @return      int                 
 */
int xp_cmd_excuted_complete_callback(char *identifier, int value)
{
    char buf[MQTT_PORPERTE_IDENTIFIER_MAX_LEN + 10] = {0};

    if(identifier){
        sprintf(buf,"{\"%s\":%d}", identifier, value);
        if(xp_mqtt_post_property(&mqtt, buf) >= 0) {
            LOG_INFO("Set cmd: %s to %d", identifier, value);
        }
        return 0;
    }
    else{
        return -1;
    }
}

/**
 * @brief       日志上传远程平台回调
 * @param[in]	logStr              日志信息
 * @return      int                 
 */
int xp_mqtt_send_log_callback(char *logStr)
{
    (0 == strcmp(logStr, recordLogStr)) ? (repeatLogCnt++) : (repeatLogCnt = 0);    //限制重复打印的次数，减少流量
    if(repeatLogCnt > 5){
        return -1;
    }
    //日志字符超过一定数量，立刻上传当前日志
    if((strlen(logBuf) + strlen(logStr) > sizeof(logBuf) - 20)){
        sprintf(logSendBuf,"{\"sts_log\":\"%s\"}", logBuf);
        memset(logBuf, 0, sizeof(logBuf));
        if(1 == mqtt.connect)   xp_mqtt_post_property(&mqtt, logSendBuf);
    }
    strcat(logBuf, logStr);
    strcat(logBuf, "\r\n");
    strcpy(recordLogStr, logStr);
    return 0;
}

/**
 * @brief       日志信息上报，检查当前是否有存储的日志，有则上报
 * @param[in]	arg                 
 */
void xp_log_upload_thread(void *arg)
{
    while(1){
        if(strlen(logBuf) > 0){
            sprintf(logSendBuf,"{\"sts_log\":\"%s\"}", logBuf);
            memset(logBuf, 0, sizeof(logBuf));
            if(1 == mqtt.connect)   xp_mqtt_post_property(&mqtt, logSendBuf);
        }
        aos_msleep(30000);
    }
}

/*                                                         =======================                                                         */
/* ========================================================    物联网信息初始化    ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       云平台物联网初始化
 */
int xp_net_init(void)
{
    int ret = -1;

    xp_mqtt_region_set(region_american);
    ret = xp_mqtt_init(&mqtt);
    if(ret != 0){
        LOG_UPLOAD("xp_mqtt_init failed!");
    }
    localModelSts = get_modelSts_Obj();
    localModelCmd = get_modelCmd_Obj();

    xp_error_upload_callback_regist(xp_err_upload_callback);
    xp_cmd_excuted_complete_callback_regist(xp_cmd_excuted_complete_callback);
    xp_mqtt_send_log_callback_regist(xp_mqtt_send_log_callback);

    aos_task_new("iot_data_register", xp_iot_data_register_thread, NULL, 8192);
    aos_task_new("iot_auto_report", xp_iot_auto_report_thread, NULL, 8192);
    aos_task_new("mqtt_heart", xp_mqtt_heart_thread, NULL, 2048);
    aos_task_new("log_upLoad", xp_log_upload_thread, NULL, 2048);

    return 0;
}

/**
 * @brief       添加需要自动上报的属性到列表
 * @param[in]	identifier          属性标识
 * @param[in]	data                数据指针
 * @param[in]	data_type           数据类型
 * @param[in]	sizeof_data         数据类型大小
 * @param[in]	diff_range          变化范围(此参数指定当前属性数据在变化超过此范围后自动上报)
 * @param[in]	cmd_id              属性设置回调中入队列的命令ID
 * @return      bool                true：成功；false：失败
 */
bool Iot_add_property_node(char * const identifier, void * const data, Type_data_type_Enum data_type, uint8_t sizeof_data, int32_t diff_range, int cmd_id)
{
    if (identifier && data && strlen(identifier) < MQTT_PORPERTE_IDENTIFIER_MAX_LEN - 1 && sizeof_data < MQTT_PORPERTE_STRING_MAX_LEN){
        Type_PropertyNode_Def *pNode = aos_malloc(sizeof(Type_PropertyNode_Def));
        if (pNode){
            memcpy(pNode->identifier, identifier, strlen(identifier) + 1);
            pNode->data = data;
            pNode->data_type = data_type;
            pNode->sizeof_data = sizeof_data;
            pNode->diff_range = diff_range;
            pNode->cmd_id = cmd_id;
            pNode->last_data = aos_malloc(pNode->sizeof_data);
            if (NULL == pNode->last_data){
                aos_free(pNode);
                return false;
            }
            memcpy(pNode->last_data, pNode->data, pNode->sizeof_data);
            slist_add_tail(&pNode->next, &list_property);
            return true;
        }
    }
    LOG_FATAL("Registered %s failed, strlen %d sizeof_data %d", identifier, strlen(identifier), sizeof_data);
    return false;
}

/**
 * @brief       通过标识从列表删除已添加的属性
 * @param[in]	identifier          属性标识
 * @return      bool                true：成功；false：失败
 */
bool Iot_del_property_Node(char * const identifier)
{
    if (identifier && strlen(identifier) < MQTT_PORPERTE_IDENTIFIER_MAX_LEN){
        Type_PropertyNode_Def *pNode = NULL;
        slist_t *cur = NULL;
        //遍历链表找出符合匹配规则的节点,然后删除
        slist_for_each_entry_safe(&list_property, cur, pNode, Type_PropertyNode_Def, next) {
            if (0 == strcmp(pNode->identifier, identifier))
                break;
        }
        if (NULL != pNode){
            slist_del(&pNode->next, &list_property);
            if (pNode->diff_range >= 0){
                aos_free(pNode->last_data);
            }
            aos_free(pNode);
        }
    }
    return false;
}

//点位注册线程
void xp_iot_data_register_thread(void *arg)
{
    if(localModelSts == NULL || localModelCmd == NULL){
        LOG_WARN("No model object, exit register");
        aos_task_exit(0);
    }

/* ********************************************注册所有点位******************************************** */
//=============================================只读类点位================================================
    bool err = true;
    //通讯测试点位
    err &= IOT_ADD_PROPERTY_NODE("sts_communication_test",		localModelSts->communicateTest,		TYPE_Bool,	0,	0);
    //1#订单
    err &= IOT_ADD_PROPERTY_NODE("sts_order_number_1",	        localModelSts->order1.orderNumber,  TYPE_Int,   0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_work_state_1",	        localModelSts->order1.workState,    TYPE_Int,	0,	0);
    //2#订单
    err &= IOT_ADD_PROPERTY_NODE("sts_order_number_2",	        localModelSts->order2.orderNumber,  TYPE_Int,   0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_work_state_2",	        localModelSts->order2.workState,    TYPE_Int,	0,	0);
    //洗车判断
    err &= IOT_ADD_PROPERTY_NODE("sts_offline_order",	        localModelSts->washInfo.offlineOrderNum,    TYPE_Int,0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_complete_order",	        localModelSts->washInfo.completeOrderNum,   TYPE_Int,0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_car_head",	            localModelSts->washInfo.carAhead,       TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_stop_ok",	                localModelSts->washInfo.stopOk,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_park_tips",	            localModelSts->washInfo.parkTips,       TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_gate_open_1",	            localModelSts->washInfo.gateOpenDone1,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_gate_close_1",	        localModelSts->washInfo.gateCloseDone1, TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_gate_open_2",	            localModelSts->washInfo.gateOpenDone2,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_gate_close_2",	        localModelSts->washInfo.gateCloseDone2, TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_car_too_long",	        localModelSts->washInfo.carTooLong,     TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_car_too_high",	        localModelSts->washInfo.carTooHigh,     TYPE_Bool,	0,	0);
    // err &= IOT_ADD_PROPERTY_NODE("sts_car_start_wash",	        localModelSts->washInfo.carStartWash,   TYPE_Bool,	0,	0);
    // err &= IOT_ADD_PROPERTY_NODE("sts_car_finish_wash",	        localModelSts->washInfo.carFinishWash,  TYPE_Bool,	0,	0);
    //洗车机状态
    err &= IOT_ADD_PROPERTY_NODE("sts_stopping",	            localModelSts->devSta.stopping,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_standby",	                localModelSts->devSta.standby,          TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_homing",	                localModelSts->devSta.normalHoming,     TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_custom_stopping",	        localModelSts->devSta.customStop,       TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_warn_homing",	            localModelSts->devSta.warnHoming,       TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_dev_warning",	            localModelSts->devSta.warnning,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_warn_code",	        localModelSts->devSta.warnCode,         TYPE_String,0,	0);
    //监控数据
    err &= IOT_ADD_PROPERTY_NODE("sts_car_entry",	            localModelSts->minitor.carEntry,            TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_sewage_high_time",	    localModelSts->minitor.sewageHighTime,      TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_ready_pulse",	            localModelSts->minitor.readyAreaPulse,      TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_work_pulse",	            localModelSts->minitor.workAreaPulse,       TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_complete_pulse",	        localModelSts->minitor.completeAreaPulse,   TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_front_pos",	        localModelSts->minitor.frontLeftPutterPos,  TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_front_pos",	        localModelSts->minitor.frontRightPutterPos, TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_back_pos",	        localModelSts->minitor.backLeftPutterPos,   TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_back_pos",	        localModelSts->minitor.backRightPutterPos,  TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_top_brush_current",	    localModelSts->minitor.topBrushCurrent,     TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_Lfront_brush_current",	localModelSts->minitor.frontLeftBrushCurrent,   TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_Rfront_brush_current",	localModelSts->minitor.frontRightBrushCurrent,  TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_Lback_brush_current",	    localModelSts->minitor.backLeftBrushCurrent,    TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_Rback_brush_current",	    localModelSts->minitor.backRightBrushCurrent,   TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_ota_flag",	            localModelSts->minitor.isOta,		        TYPE_Bool,	0,	0);
    //检测数据
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_firmware_version",	localModelSts->minitor.version,         TYPE_String,0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_device_model",	    localModelSts->minitor.deviceModel,     TYPE_String,0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_wash_start_counts",	    localModelSts->minitor.startCnt,        TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_wash_complete_counts",	localModelSts->minitor.completeCnt,     TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_wash_failed_counts",	    localModelSts->minitor.failedCnt,       TYPE_Int,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_wash_today_counts",	    localModelSts->minitor.toadyTotalCnt,   TYPE_Int,	0,	0);
    //传感器检测
    err &= IOT_ADD_PROPERTY_NODE("sts_emergency",	            localModelSts->sensor.emergency,        TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_reset",	                localModelSts->sensor.reset,            TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_phase_sequence",	        localModelSts->sensor.phaseSequence,    TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_water_press_low",	        localModelSts->sensor.waterPressLow,    TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_front_collision",	localModelSts->sensor.fLeftCollision,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_front_collision",	localModelSts->sensor.fRightCollision,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_back_collision",	    localModelSts->sensor.bLeftCollision,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_back_collision",	localModelSts->sensor.bRightCollision,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_avoid_entry",	    localModelSts->sensor.avoidIntrude,     TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_left_skew",	    localModelSts->sensor.leftSkew,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_right_skew",	    localModelSts->sensor.rightSkew,        TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_stop",	            localModelSts->sensor.stop,             TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_entry",	        localModelSts->sensor.entrance,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_all_in",	        localModelSts->sensor.allIn,            TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_height",	        localModelSts->sensor.high,             TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_rear_end",	        localModelSts->sensor.rearEndProtect,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_exit",	            localModelSts->sensor.exit,             TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_complete",	        localModelSts->sensor.complete,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_pulse_conveyor_1",	    localModelSts->sensor.conveyorPulse1,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_pulse_conveyor_2",	    localModelSts->sensor.conveyorPulse2,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_pulse_conveyor_3",	    localModelSts->sensor.conveyorPulse3,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_skirt_open",	        localModelSts->sensor.leftSkirtOpen,    TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_skirt_open",	    localModelSts->sensor.rightSkirtOpen,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_lifter_up",	            localModelSts->sensor.lifterUp,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_lifter_down",	            localModelSts->sensor.lifterDown,       TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_top_left_loose",	        localModelSts->sensor.lifterLeftLoose,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_top_right_loose",	        localModelSts->sensor.lifterRightLoose, TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_front_crooked",	    localModelSts->sensor.fLeftBrushCrooked,TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_front_zero",	        localModelSts->sensor.fLeftBrushOpen,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_front_incline",	    localModelSts->sensor.fLeftBrushDown,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_front_crooked",	    localModelSts->sensor.fRightBrushCrooked,TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_front_zero",	    localModelSts->sensor.fRightBrushOpen,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_front_incline",	    localModelSts->sensor.fRightBrushDown,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_back_crooked",	    localModelSts->sensor.bLeftBrushCrooked,TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_back_zero",	        localModelSts->sensor.bLeftBrushOpen,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_left_back_incline",	    localModelSts->sensor.bLeftBrushDown,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_back_crooked",	    localModelSts->sensor.bRightBrushCrooked,TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_back_zero",	        localModelSts->sensor.bRightBrushOpen,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_right_back_incline",	    localModelSts->sensor.bRightBrushDown,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_pulse_left_front",	    localModelSts->sensor.fLeftPutterPulse, TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_pulse_right_front",	    localModelSts->sensor.fRightPutterPulse,TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_pulse_left_back",	        localModelSts->sensor.bLeftPutterPulse, TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_pulse_right_back",	    localModelSts->sensor.bRightPutterPulse,TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_signal_ground",	        localModelSts->sensor.ground,           TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_gate_broken",	            localModelSts->sensor.gate1Err,         TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_sewage_level_high",	    localModelSts->sensor.sewageHigh,       TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_shampoo_less",	        localModelSts->sensor.shampooLess,      TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_waxwater_less",	        localModelSts->sensor.waxwaterLess,     TYPE_Bool,	0,	0);
    //LED状态
    err &= IOT_ADD_PROPERTY_NODE("sts_led_go_head",	            localModelSts->led.goHead,      TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_go_back",	            localModelSts->led.goBack,      TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_left_skew",	        localModelSts->led.leftSkew,    TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_right_skew",	        localModelSts->led.rightSkew,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_stop",	            localModelSts->led.stop,        TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_too_long",	        localModelSts->led.tooLong,     TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_wait",	            localModelSts->led.wait,        TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_too_high",	        localModelSts->led.superHigh,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_exception",	        localModelSts->led.expection,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_reset",	            localModelSts->led.reset,       TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_pause_service",	    localModelSts->led.pauseService,TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_ready_start",	        localModelSts->led.readyStart,  TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_move_on",	            localModelSts->led.moveOn,      TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_led_scan",	            localModelSts->led.scaneCode,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_exit_led_green",	        localModelSts->led.exitGreen,   TYPE_Bool,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE("sts_exit_led_red",	        localModelSts->led.exitRed,     TYPE_Bool,	0,	0);
    //IO点位（字符串类型）
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_DI_IO",	            localModelSts->sDI_IO[0],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_I0_IO",	            localModelSts->sDI_IO[1],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_I1_IO",	            localModelSts->sDI_IO[2],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_I2_IO",	            localModelSts->sDI_IO[3],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_I3_IO",	            localModelSts->sDI_IO[4],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_I4_IO",	            localModelSts->sDI_IO[5],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_DO_IO",	            localModelSts->sDO_IO[0],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_Q0_IO",	            localModelSts->sDO_IO[1],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_Q1_IO",	            localModelSts->sDO_IO[2],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_Q2_IO",	            localModelSts->sDO_IO[3],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_Q3_IO",	            localModelSts->sDO_IO[4],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_Q4_IO",	            localModelSts->sDO_IO[5],   TYPE_String,	0,	0);

    err &= IOT_ADD_PROPERTY_NODE_STR("sts_DI_01_04",	        localModelSts->bsDI_IO[0],   TYPE_String,	0,	0);
    err &= IOT_ADD_PROPERTY_NODE_STR("sts_DO_01_04",	        localModelSts->bsDO_IO[0],   TYPE_String,	0,	0);

    char str[15];
    for (uint8_t i = 0; i < BOARD_NUMS - 1; i++)
    {
        for (uint8_t j = 0; j < 6; j++)
        {
            sprintf(str, "sts_I%d_%02d_%02d", i, j*4+1, j*4+4);
            err &= IOT_ADD_PROPERTY_NODE_STR(str, localModelSts->bsDI_IO[i*6 + j + 1],   TYPE_String,	0,	0);
            sprintf(str, "sts_Q%d_%02d_%02d", i, j*4+1, j*4+4);
            err &= IOT_ADD_PROPERTY_NODE_STR(str, localModelSts->bsDO_IO[i*6 + j + 1],   TYPE_String,	0,	0);
        }
    }

    // //IO点位（布尔类型）
    // char str[10];
    // uint8_t i;
    // for (i = 0; i < 4; i++)         //主板点位注册（4个输入输出）
    // {
    //     sprintf(str, "sts_DI_%02d", i + 1);
    //     err &= IOT_ADD_PROPERTY_NODE(str,   localModelSts->bDI_IO[0][i],   TYPE_Bool,	0,	0);
    //     sprintf(str, "sts_DO_%02d", i + 1);
    //     err &= IOT_ADD_PROPERTY_NODE(str,   localModelSts->bDO_IO[0][i],   TYPE_Bool,	0,	0);
    // }
    // for (i = 0; i < BOARD_NUMS - 1; i++)
    // {
    //     for (uint8_t j = 0; j < 24; j++)    //子板点位注册（24个输入输出）
    //     {
    //         sprintf(str, "sts_I%d_%02d", i, j + 1);
    //         err &= IOT_ADD_PROPERTY_NODE(str,   localModelSts->bDI_IO[i + 1][j],   TYPE_Bool,	0,	0);
    //         sprintf(str, "sts_O%d_%02d", i, j + 1);
    //         err &= IOT_ADD_PROPERTY_NODE(str,   localModelSts->bDO_IO[i + 1][j],   TYPE_Bool,	0,	0);
    //     }
    // }
    
    //重启标志
    err &= IOT_ADD_PROPERTY_NODE("sts_reboot_flag",		localModelSts->rebootFlag,		TYPE_Bool,	0,	0);

//=============================================读写类点位================================================
    err &= IOT_ADD_PROPERTY_NODE("cmd_new_order",	            localModelCmd->newOrder,        TYPE_Int,   -1,	CMD_NEW_ORDER);
    err &= IOT_ADD_PROPERTY_NODE("cmd_scan_code",	            localModelCmd->scanCode,        TYPE_Bool,	-1,	CMD_SCAN_CODE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_wash_flag_1",	            localModelCmd->washFlag1,       TYPE_Bool,	-1,	CMD_WASH_FALG_1);
    err &= IOT_ADD_PROPERTY_NODE("cmd_wash_flag_2",	            localModelCmd->washFlag2,       TYPE_Bool,	-1,	CMD_WASH_FALG_2);
    //通讯测试点位
    err &= IOT_ADD_PROPERTY_NODE("cmd_communication_test",		localModelCmd->communicateTest, TYPE_Bool,	-1,	CMD_COMMUNICATION_TEST);
    //远程控制
    err &= IOT_ADD_PROPERTY_NODE("cmd_home",	                localModelCmd->backHome,        TYPE_Bool,	-1,	CMD_HOME);
    err &= IOT_ADD_PROPERTY_NODE("cmd_safe_home",	            localModelCmd->safeBackHome,    TYPE_Bool,	-1,	CMD_SAFE_HOME);
    err &= IOT_ADD_PROPERTY_NODE("cmd_start_wash",	            localModelCmd->startWash,       TYPE_Bool,	-1,	CMD_START_WASH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_wash_mode",				localModelCmd->washMode,		TYPE_Int,	-1,	CMD_WASH_MODE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_stop",	                localModelCmd->stop,            TYPE_Bool,	-1,	CMD_STOP);
    err &= IOT_ADD_PROPERTY_NODE("cmd_custom_stop",	            localModelCmd->customStopWash,  TYPE_Bool,	-1,	CMD_CUSTOM_STOP_WASH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_electrical_reset",	    localModelCmd->electricalReset, TYPE_Bool,	-1,	CMD_ELECTRICAL_RESET);
    err &= IOT_ADD_PROPERTY_NODE("cmd_cancel_order",	        localModelCmd->cancelOrder,     TYPE_Bool,	-1,	CMD_CANCEL_ORDER);
    err &= IOT_ADD_PROPERTY_NODE("cmd_dev_close",	            localModelCmd->devClose,        TYPE_Bool,	-1,	CMD_DEV_CLOSE);
    //手动控制
    err &= IOT_ADD_PROPERTY_NODE("cmd_sync",	                localModelCmd->sync,                TYPE_Bool,	-1,	SYNC);
    err &= IOT_ADD_PROPERTY_NODE("cmd_gate_1_open",	            localModelCmd->gate1Open,           TYPE_Bool,	-1,	CMD_GATE_1_OPEN);
    err &= IOT_ADD_PROPERTY_NODE("cmd_gate_1_close",	        localModelCmd->gate1Close,          TYPE_Bool,	-1,	CMD_GATE_1_CLOSE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_gate_2_open",	            localModelCmd->gate2Open,           TYPE_Bool,	-1,	CMD_GATE_2_OPEN);
    err &= IOT_ADD_PROPERTY_NODE("cmd_gate_2_close",	        localModelCmd->gate2Close,          TYPE_Bool,	-1,	CMD_GATE_2_CLOSE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_gate_3_open",	            localModelCmd->gate3Open,           TYPE_Bool,	-1,	CMD_GATE_3_OPEN);
    err &= IOT_ADD_PROPERTY_NODE("cmd_gate_3_close",	        localModelCmd->gate3Close,          TYPE_Bool,	-1,	CMD_GATE_3_CLOSE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_conveyor_work_1",	        localModelCmd->conveyorMove1,       TYPE_Bool,	-1,	CMD_CONVEYOR_1);
    err &= IOT_ADD_PROPERTY_NODE("cmd_conveyor_work_2",	        localModelCmd->conveyorMove2,       TYPE_Bool,	-1,	CMD_CONVEYOR_2);
    err &= IOT_ADD_PROPERTY_NODE("cmd_conveyor_work_3",	        localModelCmd->conveyorMove3,       TYPE_Bool,	-1,	CMD_CONVEYOR_3);
    err &= IOT_ADD_PROPERTY_NODE("cmd_conveyor_work_all",	    localModelCmd->conveyorMoveAll,     TYPE_Bool,	-1,	CMD_CONVEYOR_ALL);
    err &= IOT_ADD_PROPERTY_NODE("cmd_water_high_press",	    localModelCmd->highPressWater,      TYPE_Bool,	-1,	CMD_HIGH_PRESS_WATER);
    err &= IOT_ADD_PROPERTY_NODE("cmd_water_shampoo",	        localModelCmd->shampoo,             TYPE_Bool,	-1,	CMD_WATER_SHAMPOO);
    err &= IOT_ADD_PROPERTY_NODE("cmd_water_waxwater",	        localModelCmd->waxWater,            TYPE_Bool,	-1,	CMD_WATER_WAXWATER);
    err &= IOT_ADD_PROPERTY_NODE("cmd_water_top",	            localModelCmd->topWater,            TYPE_Bool,	-1,	CMD_WATER_TOP);
    err &= IOT_ADD_PROPERTY_NODE("cmd_water_front_brush",	    localModelCmd->frontSideWater,      TYPE_Bool,	-1,	CMD_WATER_FRONT_SIDE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_water_back_brush",	    localModelCmd->backSideWater,       TYPE_Bool,	-1,	CMD_WATER_BACK_SIDE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_skirt_brush_out",	        localModelCmd->skirtBrushOut,       TYPE_Bool,	-1,	CMD_SKIRT_BRUSH_MOVE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_skirt_rotation",	        localModelCmd->skirtBrushRotate,    TYPE_Bool,	-1,	CMD_SKIRT_BRUSH_ROTATIN);
    err &= IOT_ADD_PROPERTY_NODE("cmd_top_brush_rotation",	    localModelCmd->topBrushRotate,      TYPE_Int,	-1,	CMD_TOP_BRUSH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_lifter_move",	            localModelCmd->lifterMove,          TYPE_Int,	-1,	CMD_LIFTER);
    err &= IOT_ADD_PROPERTY_NODE("cmd_left_front_rotation",	    localModelCmd->fLeftBrushRotate,    TYPE_Int,	-1,	CMD_FRONT_LEFT_BRUSH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_left_front_move",	        localModelCmd->fLeftPutterMove,     TYPE_Int,	-1,	CMD_FRONT_LEFT_MOVE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_right_front_rotation",	localModelCmd->fRightBrushRotate,   TYPE_Int,	-1,	CMD_FRONT_RIGHT_BRUSH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_right_front_move",	    localModelCmd->fRightPutterMove,    TYPE_Int,	-1,	CMD_FRONT_RIGHT_MOVE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_left_back_rotation",	    localModelCmd->bLeftBrushRotate,    TYPE_Int,	-1,	CMD_BACK_LEFT_BRUSH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_left_back_move",	        localModelCmd->bLeftPutterMove,     TYPE_Int,	-1,	CMD_BACK_LEFT_MOVE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_right_back_rotation",	    localModelCmd->bRightBrushRotate,   TYPE_Int,	-1,	CMD_BACK_RIGHT_BRUSH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_right_back_move",	        localModelCmd->bRightPutterMove,    TYPE_Int,	-1,	CMD_BACK_RIGHT_MOVE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_dryer_1",	                localModelCmd->dryer1,              TYPE_Bool,	-1,	CMD_DRYER_1);
    err &= IOT_ADD_PROPERTY_NODE("cmd_dryer_2_5",	            localModelCmd->dryer25,             TYPE_Bool,	-1,	CMD_DRYER_25);
    err &= IOT_ADD_PROPERTY_NODE("cmd_dryer_3_4",	            localModelCmd->dryer34,             TYPE_Bool,	-1,	CMD_DRYER_34);
    err &= IOT_ADD_PROPERTY_NODE("cmd_dryer_6",	                localModelCmd->dryer6,              TYPE_Bool,	-1,	CMD_DRYER_6);
    err &= IOT_ADD_PROPERTY_NODE("cmd_floodlight",	            localModelCmd->floodlight,          TYPE_Bool,	-1,	CMD_FLOODLIGHT);
    err &= IOT_ADD_PROPERTY_NODE("cmd_ambient_light",	        localModelCmd->ambientLight,        TYPE_Bool,	-1,	CMD_AMBIENT_LIGHT);
    err &= IOT_ADD_PROPERTY_NODE("cmd_sewage_pump",	            localModelCmd->sewagePump,          TYPE_Bool,	-1,	CMD_SEWAGE_PUMP);
    err &= IOT_ADD_PROPERTY_NODE("cmd_high_pump",	            localModelCmd->highPump,            TYPE_Bool,	-1,	CMD_HIGH_PUMP);
    err &= IOT_ADD_PROPERTY_NODE("cmd_winter_drainage",	        localModelCmd->winterDrainage,      TYPE_Bool,	-1,	CMD_WINTER_DRAINAGE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_conveyor_valve_1",	    localModelCmd->conveyorValve1,      TYPE_Bool,	-1,	CMD_CONVEYOR_1_VALVE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_conveyor_valve_2",	    localModelCmd->conveyorValve2,      TYPE_Bool,	-1,	CMD_CONVEYOR_2_VALVE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_conveyor_valve_3",	    localModelCmd->conveyorValve3,      TYPE_Bool,	-1,	CMD_CONVEYOR_3_VALVE);
    //功能按钮
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_lifter_loose",	    localModelCmd->func.detectLifterLoose,  TYPE_Bool,	-1,	CMD_DETECT_LIFTER_LOOSE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_water_press",	    localModelCmd->func.detectWaterPress,   TYPE_Bool,	-1,	CMD_DETECT_WATER_PRESS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_shampoo_less",	    localModelCmd->func.detectShampoo,      TYPE_Bool,	-1,	CMD_DETECT_SHAMPOO);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_waxwater_less",	    localModelCmd->func.detectWaxwater,     TYPE_Bool,	-1,	CMD_DETECT_WAXWATER);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_sewage_level",	    localModelCmd->func.detectSewage,       TYPE_Bool,	-1,	CMD_DETECT_SEWAGE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_sewage_pump",	    localModelCmd->func.enableSewagePump,   TYPE_Bool,	-1,	CMD_ENABLE_SEWAGE_PUMP);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_top_swing",	        localModelCmd->func.enableTopSwing,     TYPE_Bool,	-1,	CMD_ENABLE_TOP_SWING);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_left_swing",	        localModelCmd->func.enableLeftSwing,    TYPE_Bool,	-1,	CMD_ENABLE_LEFT_SWING);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_right_swing",	    localModelCmd->func.enableRightSwing,   TYPE_Bool,	-1,	CMD_ENABLE_RIGHT_SWING);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_skirt_brush",	    localModelCmd->func.enableSkirtBrush,   TYPE_Bool,	-1,	CMD_ENABLE_SKIRT_BRUSH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_all_in_signal",	    localModelCmd->func.detectAllinSignal,  TYPE_Bool,	-1,	CMD_DETECT_ALLIN_SIGNAL);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_skirt_signal",	    localModelCmd->func.detectSkirtSignal,  TYPE_Bool,	-1,	CMD_DETECT_SKIRT_SIGNAL);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_dryer_16",	        localModelCmd->func.enableDryer16,      TYPE_Bool,	-1,	CMD_ENABLE_DRYER16);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_dryer_25",	        localModelCmd->func.enableDryer25,      TYPE_Bool,	-1,	CMD_ENABLE_DRYER25);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_dryer_34",	        localModelCmd->func.enableDryer34,      TYPE_Bool,	-1,	CMD_ENABLE_DRYER34);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_gate_1",	            localModelCmd->func.enableGate1,        TYPE_Bool,	-1,	CMD_ENABLE_GATE1);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_dryer_mode",	        localModelCmd->func.enableDryerDateMode,TYPE_Bool,	-1,	CMD_ENABLE_DRYER_MODE);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_auto_reset",	        localModelCmd->func.enableAutoReset,    TYPE_Bool,	-1,	CMD_ENABLE_AUTO_RESET);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_high_press_wash",	localModelCmd->func.enableHighPressWash,TYPE_Bool,	-1,	CMD_ENABLE_HIGH_PRESS_WASH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_super_high",	        localModelCmd->func.detectSuperHigh,    TYPE_Bool,	-1,	CMD_DETECT_SUPER_HIGH);
    err &= IOT_ADD_PROPERTY_NODE("cmd_station_stop_func",	    localModelCmd->func.enableStation,      TYPE_Bool,	-1,	CMD_ENABLE_STATION);
    err &= IOT_ADD_PROPERTY_NODE("cmd_func_manual_mode",	    localModelCmd->func.enableManualMode,   TYPE_Bool,	-1,	CMD_ENABLE_MANUAL_MODE);
    //防撞功能设置
    err &= IOT_ADD_PROPERTY_NODE("cmd_col_func_left_front",	    localModelCmd->func.detectFLeftCollision,   TYPE_Bool,	-1,	CMD_DETECT_FLEFT_COLLISION);
    err &= IOT_ADD_PROPERTY_NODE("cmd_col_func_right_front",	localModelCmd->func.detectFRightCollision,  TYPE_Bool,	-1,	CMD_DETECT_FRIGHT_COLLISION);
    err &= IOT_ADD_PROPERTY_NODE("cmd_col_func_left_back",	    localModelCmd->func.detectBLeftCollision,   TYPE_Bool,	-1,	CMD_DETECT_BLEFT_COLLISION);
    err &= IOT_ADD_PROPERTY_NODE("cmd_col_func_right_back",	    localModelCmd->func.detectBRightCollision,  TYPE_Bool,	-1,	CMD_DETECT_BRIGHT_COLLISION);
    //调整数据
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_water_end",	    localModelCmd->adjust.highPressEndPos,          TYPE_Int,	-1,	CMD_HIGH_PRESS_END_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_skirt_start",	    localModelCmd->adjust.skirtBrushStartPos,       TYPE_Int,	-1,	CMD_SKIRT_BRUSH_START_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_skirt_end",	    localModelCmd->adjust.skirtBrushEndPos,         TYPE_Int,	-1,	CMD_SKIRT_BRUSH_END_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_shampoo_start",	localModelCmd->adjust.shampooStartPos,          TYPE_Int,	-1,	CMD_SHAMPOO_START_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_shampoo_end",	    localModelCmd->adjust.shampooEndPos,            TYPE_Int,	-1,	CMD_SHAMPOO_END_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_top_start",	    localModelCmd->adjust.topBrushStartPos,         TYPE_Int,	-1,	CMD_TOP_BRUSH_START_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_top_end",	        localModelCmd->adjust.topBrushEndPos,           TYPE_Int,	-1,	CMD_TOP_BRUSH_END_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_front_tail",	    localModelCmd->adjust.frontBrushTailEndPos,     TYPE_Int,	-1,	CMD_FRONT_BRUSH_END_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_front_start",	    localModelCmd->adjust.frontBrushStartPos,       TYPE_Int,	-1,	CMD_FRONT_BRUSH_START_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_back_start",	    localModelCmd->adjust.backBrushStartPos,        TYPE_Int,	-1,	CMD_BACK_BRUSH_START_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_back_end",	        localModelCmd->adjust.backBrushEndPos,          TYPE_Int,	-1,	CMD_BACK_BRUSH_END_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_waxwater_start",	localModelCmd->adjust.waxwaterStartPos,         TYPE_Int,	-1,	CMD_WAXWATER_START_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_waxwater_end",	    localModelCmd->adjust.waxwaterEndPos,           TYPE_Int,	-1,	CMD_WAXWATER_END_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_dryer_start",	    localModelCmd->adjust.dryerStartPos,            TYPE_Int,	-1,	CMD_DRYER_START_POS);
    err &= IOT_ADD_PROPERTY_NODE("cmd_adjust_pos_dryer_end",	    localModelCmd->adjust.dryerEndPos,              TYPE_Int,	-1,	CMD_DRYER_END_POS);
    //其它
	err &= IOT_ADD_PROPERTY_NODE("cmd_log",				            localModelCmd->log,			TYPE_Bool,	-1,	CMD_LOG);
    err &= IOT_ADD_PROPERTY_NODE_STR("cmd_debug",				    localModelCmd->debug,		TYPE_String,-1,	CMD_DEBUG);

    aos_task_exit(0);
}

/*                                                         =======================                                                         */
/* ========================================================      状态点位上报      ======================================================== */
/*                                                         =======================                                                         */

/* 自动属性上报线程 */
void xp_iot_auto_report_thread(void *arg)
{
    bool isMqttStaConnected = false;

    while (1) {
        if(mqtt.connect == 1){
            if(!isMqttStaConnected){
                localModelCmd->sync = true;
                isSync = true;
            }
            isMqttStaConnected = true;
            Iot_auto_property_batch_report_polling();
            localModelSts->rebootFlag = 0;
        }
        else{
            isMqttStaConnected = false;
        }
        err_need_flag_handle()->isMqttConnected = isMqttStaConnected;
        aos_msleep(autoReportIntervalMs < 100 ? (100) : (autoReportIntervalMs));
    }
}

/**
 * @brief       对比两个数值差值
 * @param[in]	a                   
 * @param[in]	b                   
 * @return      int32_t             
 */
int32_t get_diff_int(int32_t a, int32_t b){
    if (isSync){
        return INT32_MAX;
    }
    return a > b ? (a - b) : (b - a);
}

/**
 * @brief       轮询列表中值发生变化的属性，生成字符串上报
 * @return      int32_t             
 */
int32_t Iot_auto_property_batch_report_polling(void)
{
    int32_t ret = 0;
    bool isNeedReport = false;
    cJSON *pRoot;
    /* 此buffer用以截取指定长度字符串，防止用户TYPE_String类型的data区字符串超长 */
    char strBuf[MQTT_PORPERTE_STRING_MAX_LEN + 1] = {0};    
    pRoot = cJSON_CreateObject();
    
    if (pRoot){

        Type_PropertyNode_Def *pNode = NULL;
        slist_t *cur = NULL;

        slist_for_each_entry_safe(&list_property, cur, pNode, Type_PropertyNode_Def, next) {
            if (pNode->diff_range < 0 && !isSync){  //diff_range参数小于0则不对此参数进行自动上报处理,由用户手动上报
                continue;   
            }else if(pNode->data){

                bool isChange = false;

                switch (pNode->data_type)
                {
                case TYPE_Int:
                    if (get_diff_int(*(int *)pNode->data, *(int *)pNode->last_data) > pNode->diff_range){
                        cJSON_AddNumberToObject(pRoot, pNode->identifier, (double)*(int *)pNode->data);
                        isChange = true;
                    }
                    break;
                
                case TYPE_Float:
                    if (get_diff_int((int32_t)*(float *)pNode->data, (int32_t)*(float *)pNode->last_data) > pNode->diff_range){
                        cJSON_AddNumberToObject(pRoot, pNode->identifier, (double)*(float *)pNode->data);
                        isChange = true;
                    }
                    break;
                
                case TYPE_String:
                    if (0 != strncmp(pNode->data, pNode->last_data, pNode->sizeof_data) || isSync){
                        snprintf(strBuf, pNode->sizeof_data, "%s", (char *)pNode->data);
                        cJSON_AddStringToObject(pRoot, pNode->identifier, pNode->data);
                        isChange = true;
                    }
                    break;
                case TYPE_Bool:
                    if (get_diff_int(*(bool *)pNode->data, *(bool *)pNode->last_data) > 0){
                        cJSON_AddNumberToObject(pRoot, pNode->identifier, (*(bool *)pNode->data == 0) ? (0):(1));
                        isChange = true;
                    }
                break;
                default:
                    break;
                }
                if (isChange){
                    //pRet 非空则pObject分配到内存且有数据变化并成功新增value字段
                    /******************************************************************************/
                    /* 更新历史值 */
                    memcpy(pNode->last_data, pNode->data, pNode->sizeof_data);
                    isNeedReport = true;
                }else{
                    //pRet 为空则可能pObject未分配到内存或增加value字段出现问题或无数据变化
                    //cJSON_free(pObject);
                }
            }else{
                LOG_INFO("Property_id: %s data point is null", pNode->identifier);
                ret = -3;
            }
        }
    }

    if (isNeedReport){
        char *s = cJSON_Print(pRoot);
        //char *s = cJSON_PrintUnformatted(pRoot);
        if (s){
            // printf("\r\n creatJson : %s\r\n", s);
            //ret = Aliyun_send_property_post(s, true);
            if (strlen(s) > 1024){
                char *p = NULL;
                char *ptr = s;
                char *last_p = NULL;
                last_p = p = strchr(ptr, ',');
                while(NULL != p){
                    if (p - ptr > 1024){
                        *(last_p + 0) = '}';	// ',' -> '}'
                        *(last_p + 1) = '\0';	// '\n' -> '\0'
                        // *(last_p + 2) = '\0';	// '\t' -> '\0'
                        // *(last_p + 3) = '\0';	// '"' -> '\0'
                        xp_mqtt_post_property(&mqtt, ptr);
                        // printf("%s\n", ptr);
                        // *(last_p + 0) = ',';	// '}' -> ','
                        // *(last_p + 1) = '\n';	// '\0' -> '\n'
                        *(last_p + 2) = '{';	// '\0' -> '{'
                        *(last_p + 3) = '"';	// '\0' -> '"'
                        ptr = last_p + 2;
                    }else{
                        last_p = p;
                    }
                    p = strchr(p + 1, ',');
                }
                xp_mqtt_post_property(&mqtt, ptr);
                // printf("%s\n", ptr);
            }else{
                xp_mqtt_post_property(&mqtt, s);
                // printf("%s", s);
            }
            cJSON_free((void *) s);
        }else{
            ret = -4;
        }

        if(isSync == true){
            xp_cmd_excuted_complete_callback("cmd_sync", 0);
            localModelCmd->sync = false;
            isSync = false;
        }
    }
    cJSON_Delete(pRoot);
    return ret;
}

/*                                                         =======================                                                         */
/* ========================================================      命令点位执行      ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       接收到云端对设备的属性设置命令
 * @param[in]	msg                 
 */
void xp_mqtt_recive_property_user_api(const char *msg)
{
    /* 这里默认云平台每次仅会发送一个属性设置命令，因此这里不进行json遍历（通过云平台操作只能每次设置一个属性，但APP可以多个） */

    cJSON* pRoot = cJSON_Parse(msg);
    cJSON* item  = cJSON_GetArrayItem(pRoot, 0);

    if (!cJSON_IsNull(item)) {
        Type_PropertyNode_Def *pNode = NULL;
        slist_t *cur = NULL;
        /* 遍历属性注册列表，找到对应属性的数据指针 */
        slist_for_each_entry_safe(&list_property, cur, pNode, Type_PropertyNode_Def, next) {
            if (0 == strncmp(pNode->identifier, item->string, MQTT_PORPERTE_IDENTIFIER_MAX_LEN)){
                break;
            }
        }

        if (pNode){
            switch (pNode->data_type)
            {
            case TYPE_Int:
                if (cJSON_Number == item->type){
                    memcpy(pNode->data, &item->valueint, sizeof(int));
                    LOG_INFO("Set %s -> %d", item->string, *(int *)pNode->data);
                }
                break;
            
            case TYPE_Float:
                if (cJSON_Number == item->type){
                    memcpy(pNode->data, &item->valuedouble, sizeof(double));
                    LOG_INFO("Set %s -> %f", item->string, *(double *)pNode->data);
                }
                break;
            
            case TYPE_String:
                if (cJSON_String == item->type){
                    strcpy(pNode->data, item->valuestring);
                    LOG_INFO("Set %s -> %s", item->string, item->valuestring);
                }
                break;
            case TYPE_Bool:
                if (0 == item->valueint){
                    *(bool *)(pNode->data) = false;
                }else{
                    *(bool *)(pNode->data) = true;
                }
                LOG_INFO("Set %s -> %d", item->string, *(bool *)pNode->data);
                break;
            default:
                break;
            }

            switch (pNode->cmd_id)
            {
            case CMD_REBOOT:
                aos_reboot();
                break;
            case SYNC:
                if(*(bool *)pNode->data == true){
                    localModelCmd->sync = true;
                    isSync = true;
                }
                break;
            default:
                set_remote_cmd_queue(pNode);
                break;
            }
        }else{
            LOG_UPLOAD("property %s not registered", item->string);
        }
    }else{
        LOG_UPLOAD("property_set.params is NULL");
    }
    cJSON_Delete(pRoot);
}

/*                                                         =======================                                                         */
/* ========================================================        心跳上传        ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       心跳上传
 * @param[in]	arg                 
 */
void xp_mqtt_heart_thread(void *arg)
{
    while (1)
    {
        if(mqtt.connect != 1){
            aos_msleep(2000);
		    LOG_INFO("wait for net connect......");
        }
        else{
            if(xp_mqtt_post_property(&mqtt, "{\"status\":\"keep live\"}") < 0){
                LOG_WARN("post heart fault, check internet");
            }
            aos_msleep(15000);
        }
    }
}



