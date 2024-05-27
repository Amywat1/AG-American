/**
 * @file 	 ag_service.c
 * @brief
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 *
 * @copyright Copyright (c) 2022  YGL
 *
 */

#include "../../../../1km/common/boardio.h"
#include "../../../../1km/bsp/bsp.h"
#include "../../../../1km/bsp/io_config.h"
#include "../../../../1km/common/remote_cmd.h"
#include "aos/kv.h"
#include "log_level.h"
#include "ag_service.h"
#include "ag_module.h"
#include "ag_err_state.h"

#define RECORD_WASH_CNT
#define GATE_OPEN               (-1)
#define GATE_STOP               (0)
#define GATE_CLOSE              (1)

#define TEST_FALG_1_ORDER_NUM   (111)
#define TEST_FALG_2_ORDER_NUM   (222)
#define TEST_LOCAL_ORDER_NUM    (888)

#define MAX_QUEUE_ORDER_NUMBER  (5)             //支持的最大订单排队数量

bool recordIoInputSta[BOARD_NUMS][24];
bool recordIoOuputSta[BOARD_NUMS][24];

uint8_t clearOfflineOrdeBusyCnt = 0;
bool    isAutoOrder = false;

typedef struct {
    Type_ModelSts_Def       localSts;           //本地点位状态
    Type_ModelCmd_Def       localCmd;           //本地接收到的点位指令
    bool                    isDevMove[APP_CRL_NUM];
    aos_queue_t             cmdQueue;
    Type_PropertyNode_Def   cmd;                //指令
    char                    appVersion[15];
} Type_AppInfo_Def;
static Type_AppInfo_Def     appModule = {0};    //app点位信息

typedef struct {
    bool                    isOnlineOrder;      //是否是线上启动的订单
    bool                    isParkingOk;        //是否停好车
    bool                    isOrderStarted;     //订单是否已经启动
    uint32_t                numberId;           //订单编号
} Type_OrderInfo_Def;

typedef struct {
    bool                    isFirstPowerOn;
    Type_ServiceState_Enum 	state;			    //运行状态
    Type_ServiceState_Enum 	lastState;
    Type_WashMode_Enum      washMode;	        //洗车模式
    // Type_WashProc_Enum      washExePorc;        //当前执行的洗车流程
    // Type_ProcStep_Enum      washExeStep;        //当前执行的洗车步骤
    bool                    isFirstSwitch;
    bool                    isWarningErr;       //是否发生了警告错误
    bool                    isGetStartCmd;      //是否收到了洗车指令
    Type_OrderInfo_Def      orderQueue[MAX_QUEUE_ORDER_NUMBER + 1];    //存储订单编号（最后一个数组成员用于订单消耗清除，永远为0，不允许赋值）
    uint64_t                pauseTimeStamp;     //暂停的时间戳
    uint64_t                servicePauseT;      //暂停时间计时
} Type_WashStaInfo_Def;
static Type_WashStaInfo_Def wash = {0};

typedef struct {
    bool                    isCarInReadyArea;   //是否有车在预备区域
    bool                    isCarInWorkArea;    //是否有车在工作区域
    bool                    isCarInCompleteArea;//是否有车在完成区域
    bool                    isAllowToWash;      //车是否已停到位允许洗车
    bool                    isReadyToWash;      //是否扫码启动完成准备洗车
    bool                    isCarFinishWash;    //是否有车清洗结束
    Type_ParkState_Enum     parkState;
    uint8_t                 voiceCnt;           //语音播报次数
    uint64_t                lastCarEnterT;
    uint64_t                parkStaStartT;
    uint64_t                parkOkTimeStamp;    //停好车时的时间戳
} Type_CarMoveAreaSta_Def;
static Type_CarMoveAreaSta_Def carInfo = {0};

static bool isGetBackHomeCmd = false;       //是否收到归位指令
static uint64_t customerOpenGate2TimeStamp = 0;     //顾客开2号道闸的时间戳
static bool isCarWantForwardExit = false;

//检测使能与否的KV记录（枚举赋值不允许改变！）
typedef enum{
	KV_DETECT_LIFTER_LOOSE  = 0,
    KV_DETECT_WATER_PRESS   = 1,
    KV_DETECT_SHAMPOO       = 2,
    KV_DETECT_WAXWATER      = 3,
    KV_DETECT_SEWAGE        = 4,
    KV_DETECT_ALL_IN_SIGNAL = 5,
    KV_DETECT_SKIRT_SIGNAL  = 6,
    KV_DETECT_SUPER_HIGH    = 7,
    KV_DETECT_ALL_COLLISION = 8,
    KV_DETECT_FL_COLLISION  = 9,
    KV_DETECT_FR_COLLISION  = 10,
    KV_DETECT_BL_COLLISION  = 11,
    KV_DETECT_BR_COLLISION  = 12,
    KV_DETECT_SHAMPOO_GREEN = 13,
    KV_DETECT_CLEANER       = 14,
    KV_DETECT_DRIER         = 15,
} Type_KvDetectType_Enum;

//功能使能与否的KV记录（枚举赋值不允许改变！）
typedef enum{
	KV_ENABLE_STATION       = 0,
	KV_ENABLE_LOG_UPLOAD    = 1,
	KV_ENABLE_SEWAGE_PUMP   = 2,
    KV_ENABLE_TOP_SWING     = 3,
    KV_ENABLE_LEFT_SWING    = 4,
    KV_ENABLE_RIGHT_SWING   = 5,
    KV_ENABLE_SKIRT_WASH    = 6,
    KV_ENABLE_DRYER_16      = 7,
    KV_ENABLE_DRYER_25      = 8,
    KV_ENABLE_DRYER_34      = 9,
    KV_ENABLE_GATE_1        = 10,
    KV_ENABLE_AUTO_RESET    = 11,
    KV_ENABLE_DRYER_DATE_MODE = 12,
    KV_ENABLE_HIGH_PRESS_WASH = 13,
    KV_ENABLE_MANUAL_MODE   = 16,
} Type_KvEnableType_Enum;

/*                                                         ========================                                                         */
/* ========================================================    远程控制指令映射表    ======================================================== */
/*                                                         ========================                                                         */

typedef struct {
	REMOTE_CMD_ENUM         cmd;
	Type_AppCrlObject_Enum  crlObj;
    void                    *data;
    char                    *indentifer;
} Type_AppCmdMap_Def;

static Type_AppCmdMap_Def AppCmdMap_Table[] = {
   {CMD_GATE_1_OPEN,            APP_CRL_GATE_1_OPEN,            &appModule.localCmd.gate1Open,          "cmd_gate_1_open"},
   {CMD_GATE_1_CLOSE,           APP_CRL_GATE_1_CLOSE,           &appModule.localCmd.gate1Close,         "cmd_gate_1_close"},
   {CMD_GATE_2_OPEN,            APP_CRL_GATE_2_OPEN,            &appModule.localCmd.gate2Open,          "cmd_gate_2_open"},
   {CMD_GATE_2_CLOSE,           APP_CRL_GATE_2_CLOSE,           &appModule.localCmd.gate2Close,         "cmd_gate_2_close"},
   {CMD_GATE_3_OPEN,            APP_CRL_GATE_3_OPEN,            &appModule.localCmd.gate3Open,          "cmd_gate_3_open"},
   {CMD_GATE_3_CLOSE,           APP_CRL_GATE_3_CLOSE,           &appModule.localCmd.gate3Close,         "cmd_gate_3_close"},
   {CMD_CONVEYOR_1,             APP_CRL_CONVEYOR_1,             &appModule.localCmd.conveyorMove1,      "cmd_conveyor_work_1"},
   {CMD_CONVEYOR_2,             APP_CRL_CONVEYOR_2,             &appModule.localCmd.conveyorMove2,      "cmd_conveyor_work_2"},
   {CMD_CONVEYOR_3,             APP_CRL_CONVEYOR_3,             &appModule.localCmd.conveyorMove3,      "cmd_conveyor_work_3"},
   {CMD_CONVEYOR_ALL,           APP_CRL_CONVEYOR_ALL,           &appModule.localCmd.conveyorMoveAll,    "cmd_conveyor_work_all"},
   {CMD_HIGH_PRESS_WATER,       APP_CRL_HIGH_PRESS_WATER,       &appModule.localCmd.highPressWater,     "cmd_water_high_press"},
   {CMD_WATER_SHAMPOO,          APP_CRL_WATER_SHAMPOO,          &appModule.localCmd.shampoo,            "cmd_water_shampoo"},
   {CMD_WATER_WAXWATER,         APP_CRL_WATER_WAXWATER,         &appModule.localCmd.waxWater,           "cmd_water_waxwater"},
   {CMD_WATER_TOP,              APP_CRL_WATER_TOP,              &appModule.localCmd.topWater,           "cmd_water_top"},
   {CMD_WATER_FRONT_SIDE,       APP_CRL_WATER_FRONT_SIDE,       &appModule.localCmd.frontSideWater,     "cmd_water_front_brush"},
   {CMD_WATER_BACK_SIDE,        APP_CRL_WATER_BACK_SIDE,        &appModule.localCmd.backSideWater,      "cmd_water_back_brush"},
   {CMD_SKIRT_BRUSH_MOVE,       APP_CRL_SKIRT_BRUSH_MOVE,       &appModule.localCmd.skirtBrushOut,      "cmd_skirt_brush_out"},
   {CMD_SKIRT_BRUSH_ROTATIN,    APP_CRL_SKIRT_BRUSH_ROTATIN,    &appModule.localCmd.skirtBrushRotate,   "cmd_skirt_rotation"},
   {CMD_TOP_BRUSH,              APP_CRL_TOP_BRUSH,              &appModule.localCmd.topBrushRotate,     "cmd_top_brush_rotation"},
   {CMD_LIFTER,                 APP_CRL_LIFTER,                 &appModule.localCmd.lifterMove,         "cmd_lifter_move"},
   {CMD_FRONT_LEFT_BRUSH,       APP_CRL_FRONT_LEFT_BRUSH,       &appModule.localCmd.fLeftBrushRotate,   "cmd_left_front_rotation"},
   {CMD_FRONT_LEFT_MOVE,        APP_CRL_FRONT_LEFT_MOVE,        &appModule.localCmd.fLeftPutterMove,    "cmd_left_front_move"},
   {CMD_FRONT_RIGHT_BRUSH,      APP_CRL_FRONT_RIGHT_BRUSH,      &appModule.localCmd.fRightBrushRotate,  "cmd_right_front_rotation"},
   {CMD_FRONT_RIGHT_MOVE,       APP_CRL_FRONT_RIGHT_MOVE,       &appModule.localCmd.fRightPutterMove,   "cmd_right_front_move"},
   {CMD_BACK_LEFT_BRUSH,        APP_CRL_BACK_LEFT_BRUSH,        &appModule.localCmd.bLeftBrushRotate,   "cmd_left_back_rotation"},
   {CMD_BACK_LEFT_MOVE,         APP_CRL_BACK_LEFT_MOVE,         &appModule.localCmd.bLeftPutterMove,    "cmd_left_back_move"},
   {CMD_BACK_RIGHT_BRUSH,       APP_CRL_BACK_RIGHT_BRUSH,       &appModule.localCmd.bRightBrushRotate,  "cmd_right_back_rotation"},
   {CMD_BACK_RIGHT_MOVE,        APP_CRL_BACK_RIGHT_MOVE,        &appModule.localCmd.bRightPutterMove,   "cmd_right_back_move"},
   {CMD_DRYER_1,                APP_CRL_DRYER_1,                &appModule.localCmd.dryer1,             "cmd_dryer_1"},
   {CMD_DRYER_25,               APP_CRL_DRYER_25,               &appModule.localCmd.dryer25,            "cmd_dryer_2_5"},
   {CMD_DRYER_34,               APP_CRL_DRYER_34,               &appModule.localCmd.dryer34,            "cmd_dryer_3_4"},
   {CMD_DRYER_6,                APP_CRL_DRYER_6,                &appModule.localCmd.dryer6,             "cmd_dryer_6"},
   {CMD_SEWAGE_PUMP,            APP_CRL_SEWAGE_PUMP,            &appModule.localCmd.sewagePump,         "cmd_sewage_pump"},
   {CMD_HIGH_PUMP,              APP_CRL_HIGH_PUMP,              &appModule.localCmd.highPump,           "cmd_high_pump"},
   {CMD_WINTER_DRAINAGE,        APP_CRL_WINTER_DRAINAGE,        &appModule.localCmd.winterDrainage,     "cmd_winter_drainage"},
   {CMD_CONVEYOR_1_VALVE,       APP_CRL_CONVEYOR_1_VALVE,       &appModule.localCmd.conveyorValve1,     "cmd_conveyor_valve_1"},
   {CMD_CONVEYOR_2_VALVE,       APP_CRL_CONVEYOR_2_VALVE,       &appModule.localCmd.conveyorValve2,     "cmd_conveyor_valve_2"},
   {CMD_CONVEYOR_3_VALVE,       APP_CRL_CONVEYOR_3_VALVE,       &appModule.localCmd.conveyorValve3,     "cmd_conveyor_valve_3"},
};

static int xp_service_set_state(Type_ServiceState_Enum state);
static void xp_service_thread(void* arg);
static void model_status_update_thread(void *arg);
static void model_cmd_executed_thread(void *arg);
static void order_counter_thread(void *arg);
static void ready_area_detection_thread(void *arg);
static void xp_err_deal_callback(uint16_t code, Type_ErrDealMode_Enum mode);
static void offline_payment_callback(uint8_t washMode);
static int (*xp_cmd_excuted_complete)(char *arg, int value);
static int xp_read_kv_value(void);

/*                                                         =======================                                                         */
/* ========================================================     设备服务初始化     ======================================================== */
/*                                                         =======================                                                         */


int xp_service_init(void)
{
    xp_logSwitch_set(true);                 //开启SD卡日志记录
    while(!xp_ag_osal_get()->init){         //需先等子板的脉冲计数相关引脚赋值完成，再进行子板初始化，不然无法注册成功
        aos_msleep(500);
        LOG_INFO("Wait osal init...");
    };
    if(xp_io_manage_init(BOARD_NUMS - 1) != 0){
        LOG_WARN("IO board init failed !");
    }
    xp_wdg_init(20, 2);
    xp_remoteCmd_init(DEVICE_AG);
    xp_service_module_init();
    xp_read_kv_value();
    xp_cmd_excuted_complete = NULL;

    aos_task_new("service",                 xp_service_thread, NULL, 8192);
    aos_task_new("model_status_update",     model_status_update_thread, NULL, 4096);
    aos_task_new("model_cmd_executed",      model_cmd_executed_thread, NULL, 4096);
    aos_task_new("parking_sta_detection",   ready_area_detection_thread, NULL, 2048);
    aos_task_new("order_counter",           order_counter_thread, NULL, 1024);

    xp_error_deal_callback_regist(xp_err_deal_callback);
    offline_payment_callback_regist(offline_payment_callback);

    wash.isFirstPowerOn = true;
    wash.state = STA_INIT;
    wash.washMode = NORMAL_WASH;
    appModule.localCmd.washMode = wash.washMode;

    return 0;
}

/*                                                         =======================                                                         */
/* ========================================================        KV值读取        ======================================================== */
/*                                                         =======================                                                         */

//服务存储KV值
typedef struct{
    int                     detectData;
    int                     enableData;
    int                     startSkirtBrush;
    int                     startShampoo;
    int                     startTopBrush;
    int                     startFrontBrush;
    int                     startBackBrush;
    int                     startWaxwater;
    int                     startDrying;
    int                     endHighPump;
    int                     endSkirtBrush;
    int                     endShampoo;
    int                     endTopBrush;
    int                     endFrontBrush;
    int                     endBackBrush;
    int                     endWaxwater;
    int                     endDrying;
    RTC_time                time;
    Type_OrdersInfo_Def     order;
} Type_KV_Service_Def;

static Type_KV_Service_Def  kvService = {0};

typedef struct {
    REMOTE_CMD_ENUM         cmd;
    char                    *key;
    void                    *pData;
    void                    *matchAppModule;
    int                     len;
    uint8_t                 matchKvBit;         //非按位存储的KV默认0xFF
} Type_KvDataMap_Def;

static Type_KvDataMap_Def KvDataMap_Table[] = {
    {CMD_DETECT_LIFTER_LOOSE,       "detect",       &kvService.detectData,          &appModule.localCmd.func.detectLifterLoose,     sizeof(kvService.detectData),       KV_DETECT_LIFTER_LOOSE},
    {CMD_DETECT_WATER_PRESS,        "detect",       &kvService.detectData,          &appModule.localCmd.func.detectWaterPress,      sizeof(kvService.detectData),       KV_DETECT_WATER_PRESS},
    {CMD_DETECT_SHAMPOO,            "detect",       &kvService.detectData,          &appModule.localCmd.func.detectShampoo,         sizeof(kvService.detectData),       KV_DETECT_SHAMPOO},
    {CMD_DETECT_WAXWATER,           "detect",       &kvService.detectData,          &appModule.localCmd.func.detectWaxwater,        sizeof(kvService.detectData),       KV_DETECT_WAXWATER},
    {CMD_DETECT_SEWAGE,             "detect",       &kvService.detectData,          &appModule.localCmd.func.detectSewage,          sizeof(kvService.detectData),       KV_DETECT_SEWAGE},
    {CMD_DETECT_ALLIN_SIGNAL,       "detect",       &kvService.detectData,          &appModule.localCmd.func.detectAllinSignal,     sizeof(kvService.detectData),       KV_DETECT_ALL_IN_SIGNAL},
    {CMD_DETECT_SKIRT_SIGNAL,       "detect",       &kvService.detectData,          &appModule.localCmd.func.detectSkirtSignal,     sizeof(kvService.detectData),       KV_DETECT_SKIRT_SIGNAL},
    {CMD_DETECT_SUPER_HIGH,         "detect",       &kvService.detectData,          &appModule.localCmd.func.detectSuperHigh,       sizeof(kvService.detectData),       KV_DETECT_SUPER_HIGH},
    {CMD_DETECT_ALL_COLLISION,      "detect",       &kvService.detectData,          &appModule.localCmd.func.detectAllCollision,    sizeof(kvService.detectData),       KV_DETECT_ALL_COLLISION},
    {CMD_DETECT_FLEFT_COLLISION,    "detect",       &kvService.detectData,          &appModule.localCmd.func.detectFLeftCollision,  sizeof(kvService.detectData),       KV_DETECT_FL_COLLISION},
    {CMD_DETECT_FRIGHT_COLLISION,   "detect",       &kvService.detectData,          &appModule.localCmd.func.detectFRightCollision, sizeof(kvService.detectData),       KV_DETECT_FR_COLLISION},
    {CMD_DETECT_BLEFT_COLLISION,    "detect",       &kvService.detectData,          &appModule.localCmd.func.detectBLeftCollision,  sizeof(kvService.detectData),       KV_DETECT_BL_COLLISION},
    {CMD_DETECT_BRIGHT_COLLISION,   "detect",       &kvService.detectData,          &appModule.localCmd.func.detectBRightCollision, sizeof(kvService.detectData),       KV_DETECT_BR_COLLISION},
    {CMD_DETECT_CLEANER,            "detect",       &kvService.detectData,          &appModule.localCmd.func.detectCleaner,         sizeof(kvService.detectData),       KV_DETECT_CLEANER},
    {CMD_DETECT_DRIER,              "detect",       &kvService.detectData,          &appModule.localCmd.func.detectDrier,           sizeof(kvService.detectData),       KV_DETECT_DRIER},
    {CMD_ENABLE_STATION,            "enable",       &kvService.enableData,          &appModule.localCmd.func.enableStation,         sizeof(kvService.enableData),       KV_ENABLE_STATION},
    {CMD_LOG,                       "enable",       &kvService.enableData,          &appModule.localCmd.log,                        sizeof(kvService.enableData),       KV_ENABLE_LOG_UPLOAD},
    {CMD_ENABLE_SEWAGE_PUMP,        "enable",       &kvService.enableData,          &appModule.localCmd.func.enableSewagePump,      sizeof(kvService.enableData),       KV_ENABLE_SEWAGE_PUMP},
    {CMD_ENABLE_TOP_SWING,          "enable",       &kvService.enableData,          &appModule.localCmd.func.enableTopSwing,        sizeof(kvService.enableData),       KV_ENABLE_TOP_SWING},
    {CMD_ENABLE_LEFT_SWING,         "enable",       &kvService.enableData,          &appModule.localCmd.func.enableLeftSwing,       sizeof(kvService.enableData),       KV_ENABLE_LEFT_SWING},
    {CMD_ENABLE_RIGHT_SWING,        "enable",       &kvService.enableData,          &appModule.localCmd.func.enableRightSwing,      sizeof(kvService.enableData),       KV_ENABLE_RIGHT_SWING},
    {CMD_ENABLE_SKIRT_BRUSH,        "enable",       &kvService.enableData,          &appModule.localCmd.func.enableSkirtBrush,      sizeof(kvService.enableData),       KV_ENABLE_SKIRT_WASH},
    {CMD_ENABLE_DRYER16,            "enable",       &kvService.enableData,          &appModule.localCmd.func.enableDryer16,         sizeof(kvService.enableData),       KV_ENABLE_DRYER_16},
    {CMD_ENABLE_DRYER25,            "enable",       &kvService.enableData,          &appModule.localCmd.func.enableDryer25,         sizeof(kvService.enableData),       KV_ENABLE_DRYER_25},
    {CMD_ENABLE_DRYER34,            "enable",       &kvService.enableData,          &appModule.localCmd.func.enableDryer34,         sizeof(kvService.enableData),       KV_ENABLE_DRYER_34},
    {CMD_ENABLE_GATE1,              "enable",       &kvService.enableData,          &appModule.localCmd.func.enableGate1,           sizeof(kvService.enableData),       KV_ENABLE_GATE_1},
    {CMD_ENABLE_DRYER_MODE,         "enable",       &kvService.enableData,          &appModule.localCmd.func.enableDryerDateMode,   sizeof(kvService.enableData),       KV_ENABLE_DRYER_DATE_MODE},
    {CMD_ENABLE_AUTO_RESET,         "enable",       &kvService.enableData,          &appModule.localCmd.func.enableAutoReset,       sizeof(kvService.enableData),       KV_ENABLE_AUTO_RESET},
    {CMD_ENABLE_HIGH_PRESS_WASH,    "enable",       &kvService.enableData,          &appModule.localCmd.func.enableHighPressWash,   sizeof(kvService.enableData),       KV_ENABLE_HIGH_PRESS_WASH},
    {CMD_ENABLE_MANUAL_MODE,        "enable",       &kvService.enableData,          &appModule.localCmd.func.enableManualMode,      sizeof(kvService.enableData),       KV_ENABLE_MANUAL_MODE},
    {CMD_HIGH_PRESS_END_POS,        "epHighPump",   &kvService.endHighPump,         &appModule.localCmd.adjust.highPressEndPos,     sizeof(kvService.endHighPump),      0xFF},
    {CMD_SKIRT_BRUSH_START_POS,     "spSkirtBrush", &kvService.startSkirtBrush,     &appModule.localCmd.adjust.skirtBrushStartPos,  sizeof(kvService.startSkirtBrush),  0xFF},
    {CMD_SKIRT_BRUSH_END_POS,       "epSkirtBrush", &kvService.endSkirtBrush,       &appModule.localCmd.adjust.skirtBrushEndPos,    sizeof(kvService.endSkirtBrush),    0xFF},
    {CMD_SHAMPOO_START_POS,         "spShampoo",    &kvService.startShampoo,        &appModule.localCmd.adjust.shampooStartPos,     sizeof(kvService.startShampoo),     0xFF},
    {CMD_SHAMPOO_END_POS,           "epShampoo",    &kvService.endShampoo,          &appModule.localCmd.adjust.shampooEndPos,       sizeof(kvService.endShampoo),       0xFF},
    {CMD_TOP_BRUSH_START_POS,       "spTopBrush",   &kvService.startTopBrush,       &appModule.localCmd.adjust.topBrushStartPos,    sizeof(kvService.startTopBrush),    0xFF},
    {CMD_TOP_BRUSH_END_POS,         "epTopBrush",   &kvService.endTopBrush,         &appModule.localCmd.adjust.topBrushEndPos,      sizeof(kvService.endTopBrush),      0xFF},
    {CMD_FRONT_BRUSH_START_POS,     "spFBrush",     &kvService.startFrontBrush,     &appModule.localCmd.adjust.frontBrushStartPos,  sizeof(kvService.startFrontBrush),  0xFF},
    {CMD_FRONT_BRUSH_END_POS,       "epFBrush",     &kvService.endFrontBrush,       &appModule.localCmd.adjust.frontBrushTailEndPos,sizeof(kvService.endFrontBrush),    0xFF},
    {CMD_BACK_BRUSH_START_POS,      "spBBrush",     &kvService.startBackBrush,      &appModule.localCmd.adjust.backBrushStartPos,   sizeof(kvService.startBackBrush),   0xFF},
    {CMD_BACK_BRUSH_END_POS,        "epBBrush",     &kvService.endBackBrush,        &appModule.localCmd.adjust.backBrushEndPos,     sizeof(kvService.endBackBrush),     0xFF},
    {CMD_WAXWATER_START_POS,        "spWaxwater",   &kvService.startWaxwater,       &appModule.localCmd.adjust.waxwaterStartPos,    sizeof(kvService.startWaxwater),    0xFF},
    {CMD_WAXWATER_END_POS,          "epWaxwater",   &kvService.endWaxwater,         &appModule.localCmd.adjust.waxwaterEndPos,      sizeof(kvService.endWaxwater),      0xFF},
    {CMD_DRYER_START_POS,           "spDryer",      &kvService.startDrying,         &appModule.localCmd.adjust.dryerStartPos,       sizeof(kvService.startDrying),      0xFF},
    {CMD_DRYER_END_POS,             "epDryer",      &kvService.endDrying,           &appModule.localCmd.adjust.dryerEndPos,         sizeof(kvService.endDrying),        0xFF},
    {CMD_NULL,                      "time",         &kvService.time,                NULL,   sizeof(kvService.time),                 0xFF},
    {CMD_NULL,                      "completeCnt",  &kvService.order.completeCnt,   NULL,   sizeof(kvService.order.completeCnt),    0xFF},
    {CMD_NULL,                      "offlineCnt",   &kvService.order.offlineStartNum,NULL,  sizeof(kvService.order.offlineStartNum),0xFF},
    {CMD_NULL,                      "todayCnt",     &kvService.order.today,         NULL,   sizeof(kvService.order.today),          0xFF},
    {CMD_NULL,                      "totalCnt",     &kvService.order.total,         NULL,   sizeof(kvService.order.total),          0xFF},
};

static int set_kv_default_value(void)
{
    kvService.detectData            = 0x7FFFFFFF;
    kvService.enableData            = 0x7FFEFFFF;       //低16位默认非手动模式
    kvService.startSkirtBrush       = 50;
    kvService.startShampoo          = 90;
    kvService.startTopBrush         = 150;
    kvService.startFrontBrush       = 360;
    kvService.startBackBrush        = 550;
    kvService.startWaxwater         = 650;
    kvService.startDrying           = 900;
    kvService.endHighPump           = 30;
    kvService.endSkirtBrush         = 110;
    kvService.endShampoo            = 140;
    kvService.endTopBrush           = 330;
    kvService.endFrontBrush         = 460;
    kvService.endBackBrush          = 700;
    kvService.endWaxwater           = 720;
    kvService.endDrying             = 1180;
    //日期不用恢复默认值，线程里获取时间更新
    kvService.order.completeCnt     = 0;
    kvService.order.offlineStartNum = 0;
    memset(&kvService.order.today,  0, sizeof(kvService.order.today));
    memset(kvService.order.month,   0, sizeof(kvService.order.month));
    memset(kvService.order.daily,   0, sizeof(kvService.order.daily));
    memset(&kvService.order.total,  0, sizeof(kvService.order.total));
}

static int xp_read_kv_value(void)
{
    //初始赋值默认值，kv值读取成功则使用kv存储值
    set_kv_default_value();
    for (uint8_t i = 0; i < sizeof(KvDataMap_Table) / sizeof(KvDataMap_Table[0]); i++)
    {
        //整形数据存储多个含义数据的只读取一次
        if(0 == KvDataMap_Table[i].matchKvBit || 0xFF == KvDataMap_Table[i].matchKvBit){
            xp_read_creat_kv_params(KvDataMap_Table[i].key, KvDataMap_Table[i].pData, &KvDataMap_Table[i].len);
            LOG_DEBUG("Read key %s, data 0x%X, len %d", KvDataMap_Table[i].key, *(int*)KvDataMap_Table[i].pData, KvDataMap_Table[i].len);
        }

        //命令点位不会随状态改变自动更新，在这赋值，等待联网后同步所有点位时会上报
        if(KvDataMap_Table[i].matchAppModule){
            if(KvDataMap_Table[i].matchKvBit != 0xFF){
                bool value = 1 & (*(int*)KvDataMap_Table[i].pData >> KvDataMap_Table[i].matchKvBit);
                memcpy(KvDataMap_Table[i].matchAppModule, &value, 1);
            }
            else
            {
                memcpy(KvDataMap_Table[i].matchAppModule, KvDataMap_Table[i].pData, KvDataMap_Table[i].len);
            }
        }
        aos_msleep(1);
    }

    //单量kv值太多了，放在这里循环读
    // char *buf = aos_malloc(20);
    // for (uint8_t i = 0; i < 12; i++)
    // {
    //     sprintf(buf, "monthCnt_%d", i);
    //     int len = sizeof(Type_OrdersNum_Def);
    //     xp_read_creat_kv_params(buf, &kvService.order.month[i], &len);
    //     aos_msleep(1);
    //     for (uint8_t j = 0; j < 31; j++)
    //     {
    //         sprintf(buf, "dailyCnt_%d_%d", i, j);
    //         len = sizeof(Type_OrdersNum_Def);
    //         xp_read_creat_kv_params(buf, &kvService.order.daily[i][j], &len);
    //         aos_msleep(1);
    //     }
    // }
    // aos_free(buf);
    
    err_need_flag_handle()->isAllCollisionEnable        = appModule.localCmd.func.detectAllCollision;
    err_need_flag_handle()->isFrontLeftCollisionEnable  = appModule.localCmd.func.detectFLeftCollision;
    err_need_flag_handle()->isFrontRightCollisionEnable = appModule.localCmd.func.detectFRightCollision;
    err_need_flag_handle()->isBackLeftCollisionEnable   = appModule.localCmd.func.detectBLeftCollision;
    err_need_flag_handle()->isBackRightCollisionEnable  = appModule.localCmd.func.detectBRightCollision;
    err_need_flag_handle()->isDetectLifterLooseEnable   = appModule.localCmd.func.detectLifterLoose;
    err_need_flag_handle()->isDetectWaterPressEnable    = appModule.localCmd.func.detectWaterPress;
    err_need_flag_handle()->isDetectSuperHighEnable     = appModule.localCmd.func.detectSuperHigh;
    err_need_flag_handle()->isDetectShampooEnable       = appModule.localCmd.func.detectShampoo;
    err_need_flag_handle()->isDetectCleanerLess         = appModule.localCmd.func.detectCleaner;
    err_need_flag_handle()->isDetectDyierLess           = appModule.localCmd.func.detectDrier;
    err_need_flag_handle()->isDetectWaxwaterEnable      = appModule.localCmd.func.detectWaxwater;
    err_need_flag_handle()->isDetectSewageEnable        = appModule.localCmd.func.detectSewage;
    set_log_upload_flag(appModule.localCmd.log);

    get_washProcPos_Obj()->startSkirtBrush  = appModule.localCmd.adjust.skirtBrushStartPos;
    get_washProcPos_Obj()->startShampoo     = appModule.localCmd.adjust.shampooStartPos;
    get_washProcPos_Obj()->startTopBrush    = appModule.localCmd.adjust.topBrushStartPos;
    get_washProcPos_Obj()->startFrontBrush  = appModule.localCmd.adjust.frontBrushStartPos;
    get_washProcPos_Obj()->startBackBrush   = appModule.localCmd.adjust.backBrushStartPos;
    get_washProcPos_Obj()->startWaxwater    = appModule.localCmd.adjust.waxwaterStartPos;
    get_washProcPos_Obj()->startDrying      = appModule.localCmd.adjust.dryerStartPos;
    get_washProcPos_Obj()->endHighPump      = appModule.localCmd.adjust.highPressEndPos;
    get_washProcPos_Obj()->endSkirtBrush    = appModule.localCmd.adjust.skirtBrushEndPos;
    get_washProcPos_Obj()->endShampoo       = appModule.localCmd.adjust.shampooEndPos;
    get_washProcPos_Obj()->endTopBrush      = appModule.localCmd.adjust.topBrushEndPos;
    get_washProcPos_Obj()->endFrontBrush    = appModule.localCmd.adjust.frontBrushTailEndPos;
    get_washProcPos_Obj()->endBackBrush     = appModule.localCmd.adjust.backBrushEndPos;
    get_washProcPos_Obj()->endWaxwater      = appModule.localCmd.adjust.waxwaterEndPos;
    get_washProcPos_Obj()->endDrying        = appModule.localCmd.adjust.dryerEndPos;
}

/*                                                         =======================                                                         */
/* ========================================================        单量统计        ======================================================== */
/*                                                         =======================                                                         */

void order_counter_thread(void *arg)
{
    RTC_time *time = NULL;
    while (1)
    {
        time = xp_rtc_get();
        // LOG_DEBUG("Now year %d, month %d, date %d", time->year, time->month, time->date);
        if((time != NULL) && (time->month > 0 && time->month < 13 && time->date > 0 && time->date < 32)){
            if(kvService.time.year != time->year){          //年变化，清除所有记录数据（除了各模式的总单量）
                kvService.time.year = time->year;
                memset(&kvService.order.today, 0, sizeof(Type_OrdersNum_Def));
                memset(kvService.order.month, 0, sizeof(kvService.order.month));
                memset(kvService.order.daily, 0, sizeof(kvService.order.daily));
                kvService.order.isNewDateSave = true;
                kvService.order.isNewOrderSave = true;
            }
            else if(kvService.time.month != time->month){   //月变化，清除当月的记录
                kvService.time.month = time->month;
                memset(&kvService.order.today, 0, sizeof(Type_OrdersNum_Def));
                memset(&kvService.order.month[time->month - 1], 0, sizeof(Type_OrdersNum_Def));
                memset(kvService.order.daily[time->month - 1], 0, sizeof(kvService.order.daily[time->month - 1]));
                kvService.order.isNewDateSave = true;
                kvService.order.isNewOrderSave = true;
            }
            else if(kvService.time.date != time->date){     //日变化，清除当日数据
                kvService.time.date = time->date;
                memset(&kvService.order.today, 0, sizeof(Type_OrdersNum_Def));
                memset(&kvService.order.daily[time->month - 1][time->date - 1], 0, sizeof(kvService.order.daily[time->month - 1][time->date - 1]));
                kvService.order.isNewDateSave = true;
                kvService.order.isNewOrderSave = true;
            }

            if(kvService.order.isNewOrder){
                kvService.order.isNewOrder = false;
                kvService.order.isNewOrderSave = true;
                kvService.order.today.totals++;
                kvService.order.month[time->month - 1].totals++;
                kvService.order.daily[time->month - 1][time->date - 1].totals++;
                kvService.order.total.totals++;
                switch (wash.washMode)
                {
                case QUICK_WASH:
                    kvService.order.today.washQuick++;
                    kvService.order.month[time->month - 1].washQuick++;
                    kvService.order.daily[time->month - 1][time->date - 1].washQuick++;
                    kvService.order.total.washQuick++;
                    break;
                case NORMAL_WASH:
                    kvService.order.today.washNormal++;
                    kvService.order.month[time->month - 1].washNormal++;
                    kvService.order.daily[time->month - 1][time->date - 1].washNormal++;
                    kvService.order.total.washNormal++;
                    break;
                case FINE_WASH:
                    kvService.order.today.washFine++;
                    kvService.order.month[time->month - 1].washFine++;
                    kvService.order.daily[time->month - 1][time->date - 1].washFine++;
                    kvService.order.total.washFine++;
                    break;
                default:
                    break;
                }
            }
        }
        else{
            LOG_UPLOAD("Month %d, date %d illegal", time->month, time->date);
        }

        //KV值存储避开音频播放（两个一起运行偶尔会触发hardFault，原因未知，可能跟中断有关）
        if((kvService.order.isNewOrderSave || kvService.order.isNewCompleteOrderSave 
        || kvService.order.isNewDateSave || kvService.order.isNewOfflineOrderSave)
        && get_diff_ms(get_voice_start_time_stamp()) > 25000 
        && (STA_IDLE == wash.state || STA_STOP == wash.state || STA_EXCEPTION == wash.state)){
            //洗车或者待机过程中计KV，尽量保证启动次数和完成次数数值准确（洗车过程中先不写，避免洗车的时候挂了）
            if(0 == module_lock_voice_mutex(500)){
                if(kvService.order.isNewOrderSave){
                    kvService.order.isNewOrderSave = false;
                    xp_save_kv_params("todayCnt", &kvService.order.today, sizeof(kvService.order.today));
                    aos_msleep(10);                          //测试不加延时连续存储kv时程序运行会崩溃，原因未知
                    
                    //单个KV尽量不要太大，不然会频繁触发gc（kv的垃圾回收机制），达不到减少擦除次数的效果
                    // char *buf = aos_malloc(20);
                    // sprintf(buf, "monthCnt_%d", time->month - 1);
                    // xp_save_kv_params(buf, &kvService.order.month[time->month - 1], sizeof(Type_OrdersNum_Def));
                    // aos_msleep(10);
                    // sprintf(buf, "dailyCnt_%d_%d", time->month - 1, time->date - 1);
                    // xp_save_kv_params(buf, &kvService.order.daily[time->month - 1][time->date - 1], sizeof(Type_OrdersNum_Def));
                    // aos_msleep(10);
                    // aos_free(buf);
                    
                    xp_save_kv_params("totalCnt", &kvService.order.total, sizeof(kvService.order.total));
                    aos_msleep(10);
                }
                if(kvService.order.isNewCompleteOrderSave){
                    kvService.order.isNewCompleteOrderSave = false;
                    xp_save_kv_params("completeCnt", &kvService.order.completeCnt, sizeof(kvService.order.completeCnt));
                    aos_msleep(10);
                }
                if(kvService.order.isNewDateSave){
                    kvService.order.isNewDateSave = false;
                    xp_save_kv_params("time", &kvService.time, sizeof(kvService.time));
                    aos_msleep(10);
                }
                if(kvService.order.isNewOfflineOrderSave){
                    kvService.order.isNewOfflineOrderSave = false;
                    xp_save_kv_params("offlineCnt", &kvService.order.offlineStartNum, sizeof(kvService.order.offlineStartNum));
                    aos_msleep(10);
                }
                aos_msleep(500);                        //存储完延时一段时间再解锁音频播放（可能会触发kv里的gc线程，等待gc线程执行结束）
                module_unlock_voice_mutex();
            }
        }
        aos_msleep(500);
    }
}

Type_OrdersInfo_Def *get_order_info_handle(void)
{
    return &kvService.order;
}

/*                                                         =======================                                                         */
/* ========================================================    所有点位状态更新     ======================================================== */
/*                                                         =======================                                                         */

static void model_status_update_thread(void *arg)
{
    uint64_t highSewageTimeStamp = 0;
    uint8_t logCnt = 0;
    memset(recordIoInputSta, 1, sizeof(recordIoInputSta));
    memset(recordIoOuputSta, 1, sizeof(recordIoOuputSta));
    appModule.localSts.rebootFlag = 1;                  //上电默认值为1，上报后清零，作为重启识别标志
    
    while (1){
        aos_msleep(500);
        logCnt++;
/* *************************************************************  订单状态  ************************************************************** */
        appModule.localSts.order1.workState = (appModule.localSts.order1.orderNumber != 0) ? get_work_state(1) : 0;
        appModule.localSts.order2.workState = (appModule.localSts.order2.orderNumber != 0) ? get_work_state(2) : 0;
/* *************************************************************  洗车判断  ************************************************************** */
        appModule.localSts.washInfo.carAhead        = get_is_allow_next_car_wash_flag() ? false : true;
        appModule.localSts.washInfo.stopOk          = carInfo.isAllowToWash ? true : false;
        // appModule.localSts.washInfo.parkTips        = ;
        appModule.localSts.washInfo.gateOpenDone1   = (is_signal_filter_trigger(SIGNAL_GATE_1_OPEN)) ? true : false;
        appModule.localSts.washInfo.gateCloseDone1  = (is_signal_filter_trigger(SIGNAL_GATE_1_CLOSE)) ? true : false;
        appModule.localSts.washInfo.gateOpenDone2   = (is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) \
                                                    && is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)) ? true : false;
        appModule.localSts.washInfo.gateCloseDone2  = (!is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) \
                                                    && !is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)) ? true : false;
        appModule.localSts.washInfo.carTooLong      = (PARK_TOO_LONG == carInfo.parkState) ? false : true;  //超长上报状态0为超长
        appModule.localSts.washInfo.carTooHigh      = is_signal_filter_trigger(SIGNAL_SUPER_HIGH) ? false : true;    //超高上报状态0为超高
/* *************************************************************  洗车机状态  ************************************************************** */
        appModule.localSts.devSta.stopping      = 0;
        appModule.localSts.devSta.standby       = 0;
        appModule.localSts.devSta.normalHoming  = 0;
        appModule.localSts.devSta.customStop    = 0;
        appModule.localSts.devSta.warnHoming    = 0;
        appModule.localSts.devSta.warnning      = 0;
        if(STA_STOP == wash.state || STA_EXCEPTION == wash.state)   appModule.localSts.devSta.stopping = true;
        else if(STA_IDLE == wash.state)                             appModule.localSts.devSta.standby = true;
        else if(STA_RECOVER == wash.state)                          appModule.localSts.devSta.normalHoming = true;
        else if(STA_SUSPEND == wash.state){
            if(true == appModule.localCmd.customStopWash){
                appModule.localSts.devSta.customStop = true;
            }
            else{
                appModule.localSts.devSta.warnHoming = true;
            }
            appModule.localSts.devSta.warnning = true;
        }
        // strcpy(appModule.localSts.devSta.warnCode, "0");
/* *************************************************************  监控数据  ************************************************************** */
        appModule.localSts.minitor.carEntry             = carInfo.isCarInReadyArea;
        if(true == get_error_state(8104)){
            appModule.localSts.minitor.sewageHighTime   = get_diff_ms(highSewageTimeStamp) / 1000;
        }
        else{
            highSewageTimeStamp = aos_now_ms();
        }
        appModule.localSts.minitor.readyAreaPulse       = xp_osal_get_dev_pos(CONVEYOR_1_MATCH_ID);
        appModule.localSts.minitor.workAreaPulse        = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
        appModule.localSts.minitor.completeAreaPulse    = xp_osal_get_dev_pos(CONVEYOR_3_MATCH_ID);
        appModule.localSts.minitor.frontLeftPutterPos   = xp_osal_get_dev_pos(FRONT_LEFT_MOVE_MATCH_ID);
        appModule.localSts.minitor.frontRightPutterPos  = xp_osal_get_dev_pos(FRONT_RIGHT_MOVE_MATCH_ID);
        appModule.localSts.minitor.backLeftPutterPos    = xp_osal_get_dev_pos(BACK_LEFT_MOVE_MATCH_ID);
        appModule.localSts.minitor.backRightPutterPos   = xp_osal_get_dev_pos(BACK_RIGHT_MOVE_MATCH_ID);
        appModule.localSts.minitor.topBrushCurrent          = get_brush_current(BRUSH_TOP);
        appModule.localSts.minitor.frontLeftBrushCurrent    = get_brush_current(BRUSH_FRONT_LEFT);
        appModule.localSts.minitor.frontRightBrushCurrent   = get_brush_current(BRUSH_FRONT_RIGHT);
        appModule.localSts.minitor.backLeftBrushCurrent     = get_brush_current(BRUSH_BACK_LEFT);
        appModule.localSts.minitor.backRightBrushCurrent    = get_brush_current(BRUSH_BACK_RIGHT);
/* ************************************************************** 检测数据 *************************************************************** */
        strcpy(appModule.localSts.minitor.version, appModule.appVersion);
        strcpy(appModule.localSts.minitor.deviceModel, "AG-America");
        appModule.localSts.minitor.startCnt     = kvService.order.total.totals;
        appModule.localSts.minitor.completeCnt  = kvService.order.completeCnt;
        appModule.localSts.minitor.failedCnt    = kvService.order.total.totals - kvService.order.completeCnt;
        appModule.localSts.minitor.toadyTotalCnt= kvService.order.today.totals;
/* ************************************************************** 传感器检测 *************************************************************** */
        appModule.localSts.sensor.emergency             = !osal_is_io_trigger(BOARD0_INPUT_POWER_ON);
        appModule.localSts.sensor.reset                 = is_signal_filter_trigger(SIGNAL_BUTTON_RESET);
        // appModule.localSts.sensor.phaseSequence         = osal_is_io_trigger(BOARD0_INPUT_PHASE_ORDER_PROTECT);
        appModule.localSts.sensor.waterPressLow         = osal_is_io_trigger(BOARD1_INPUT_WATER_PRESS_LOW_1) && osal_is_io_trigger(BOARD1_INPUT_WATER_PRESS_LOW_2);
        appModule.localSts.sensor.fLeftCollision        = is_signal_filter_trigger(SIGNAL_FL_COLLISION);
        appModule.localSts.sensor.fRightCollision       = is_signal_filter_trigger(SIGNAL_FR_COLLISION);
        appModule.localSts.sensor.bLeftCollision        = is_signal_filter_trigger(SIGNAL_BL_COLLISION);
        appModule.localSts.sensor.bRightCollision       = is_signal_filter_trigger(SIGNAL_BR_COLLISION);
        appModule.localSts.sensor.avoidIntrude          = !is_signal_filter_trigger(SIGNAL_AVOID_INTRUDE);
        appModule.localSts.sensor.leftSkew              = !is_signal_filter_trigger(SIGNAL_LEFT_SKEW);
        appModule.localSts.sensor.rightSkew             = !is_signal_filter_trigger(SIGNAL_RIGHT_SKEW);
        appModule.localSts.sensor.stop                  = !is_signal_filter_trigger(SIGNAL_STOP);
        appModule.localSts.sensor.entrance              = !is_signal_filter_trigger(SIGNAL_ENTRANCE);
        appModule.localSts.sensor.allIn                 = !is_signal_filter_trigger(SIGNAL_ALL_IN);
        appModule.localSts.sensor.high                  = !is_signal_filter_trigger(SIGNAL_SUPER_HIGH);
        appModule.localSts.sensor.rearEndProtect        = !is_signal_filter_trigger(SIGNAL_REAR_END_PROTECT);
        appModule.localSts.sensor.exit                  = !is_signal_filter_trigger(SIGNAL_EXIT);
        appModule.localSts.sensor.complete              = !is_signal_filter_trigger(SIGNAL_FINISH);
        appModule.localSts.sensor.conveyorPulse1        = osal_is_io_trigger(BOARD0_INPUT_CONVEYOR_1_PULSE);
        appModule.localSts.sensor.conveyorPulse2        = osal_is_io_trigger(BOARD0_INPUT_CONVEYOR_2_PULSE);
        appModule.localSts.sensor.conveyorPulse3        = osal_is_io_trigger(BOARD0_INPUT_CONVEYOR_3_PULSE);
        appModule.localSts.sensor.leftSkirtOpen         = is_signal_filter_trigger(SIGNAL_LEFT_SKIRT_ZERO);
        appModule.localSts.sensor.rightSkirtOpen        = is_signal_filter_trigger(SIGNAL_RIGHT_SKIRT_ZERO);
        appModule.localSts.sensor.lifterUp              = is_signal_filter_trigger(SIGNAL_LIFTER_UP);
        appModule.localSts.sensor.lifterDown            = is_signal_filter_trigger(SIGNAL_LIFTER_DOWN);
        appModule.localSts.sensor.lifterLeftLoose       = is_signal_filter_trigger(SIGNAL_LIFTER_LEFT_DETECH);
        appModule.localSts.sensor.lifterRightLoose      = is_signal_filter_trigger(SIGNAL_LIFTER_RIGHT_DETECH);
        appModule.localSts.sensor.fLeftBrushCrooked     = !is_signal_filter_trigger(SIGNAL_FL_BRUSH_CROOKED);
        appModule.localSts.sensor.fLeftBrushOpen        = is_signal_filter_trigger(SIGNAL_FL_MOVE_ZERO);
        appModule.localSts.sensor.fLeftBrushDown        = !is_signal_filter_trigger(SIGNAL_FL_BRUSH_DOWN);
        appModule.localSts.sensor.fRightBrushCrooked    = !is_signal_filter_trigger(SIGNAL_FR_BRUSH_CROOKED);
        appModule.localSts.sensor.fRightBrushOpen       = is_signal_filter_trigger(SIGNAL_FR_MOVE_ZERO);
        appModule.localSts.sensor.fRightBrushDown       = !is_signal_filter_trigger(SIGNAL_FR_BRUSH_DOWN);
        appModule.localSts.sensor.bLeftBrushCrooked     = !is_signal_filter_trigger(SIGNAL_BL_BRUSH_CROOKED);
        appModule.localSts.sensor.bLeftBrushOpen        = is_signal_filter_trigger(SIGNAL_BL_MOVE_ZERO);
        appModule.localSts.sensor.bLeftBrushDown        = !is_signal_filter_trigger(SIGNAL_BL_BRUSH_DOWN);
        appModule.localSts.sensor.bRightBrushCrooked    = !is_signal_filter_trigger(SIGNAL_BR_BRUSH_CROOKED);
        appModule.localSts.sensor.bRightBrushOpen       = is_signal_filter_trigger(SIGNAL_BR_MOVE_ZERO);
        appModule.localSts.sensor.bRightBrushDown       = !is_signal_filter_trigger(SIGNAL_BR_BRUSH_DOWN);
        appModule.localSts.sensor.fLeftPutterPulse      = osal_is_io_trigger(BOARD2_INPUT_FRONT_LEFT_MOVE_PULSE);
        appModule.localSts.sensor.fRightPutterPulse     = osal_is_io_trigger(BOARD2_INPUT_FRONT_RIGHT_MOVE_PULSE);
        appModule.localSts.sensor.bLeftPutterPulse      = osal_is_io_trigger(BOARD2_INPUT_BACK_LEFT_MOVE_PULSE);
        appModule.localSts.sensor.bRightPutterPulse     = osal_is_io_trigger(BOARD2_INPUT_BACK_RIGHT_MOVE_PULSE);
        appModule.localSts.sensor.ground                = is_signal_filter_trigger(SIGNAL_GROUND);
        // appModule.localSts.sensor.gate1Err              = osal_is_io_trigger(BOARD4_INPUT_GATE_1_OVERLOAD);
        // appModule.localSts.sensor.sewageHigh            = osal_is_io_trigger(BOARD1_INPUT_SEWAGE_HIGH);
        appModule.localSts.sensor.shampooLess           = osal_is_io_trigger(BOARD1_INPUT_SHAMPOO_LESS_BULE);
        appModule.localSts.sensor.waxwaterLess          = osal_is_io_trigger(BOARD1_INPUT_WAXWATER_LESS);
/* ************************************************************** LED状态 *************************************************************** */
        memset(&appModule.localSts.led, 0, sizeof(appModule.localSts.led));
        switch (carInfo.parkState)
        {
        case PARK_TOO_BACK:     appModule.localSts.led.goHead = 1;  break;
        case PARK_TOO_FRONT:    appModule.localSts.led.goBack = 1;  break;
        case PARK_TOO_LEFT:     appModule.localSts.led.leftSkew = 1;  break;
        case PARK_TOO_RIGHT:    appModule.localSts.led.rightSkew = 1;  break;
        case PARK_OK:           appModule.localSts.led.stop = 1;  break;
        default:
            break;
        }
        appModule.localSts.led.tooLong  = (PARK_OK == carInfo.parkState && is_signal_filter_trigger(SIGNAL_ALL_IN)) ? true : false;
        // appModule.localSts.led.wait     = ;
        appModule.localSts.led.superHigh= get_error_state(8107);
        if(STA_EXCEPTION == wash.state){
            appModule.localSts.led.expection = 1;
        }
        else if(STA_RECOVER == wash.state){
            appModule.localSts.led.reset = 1;
        }
        else if(STA_STOP == wash.state){
            appModule.localSts.led.pauseService = 1;
        }
        
        // appModule.localSts.led.readyStart   = ;
        // appModule.localSts.led.moveOn       = ;
        appModule.localSts.led.scaneCode    = (PARK_OK == carInfo.parkState && false == appModule.localCmd.scanCode);
        appModule.localSts.led.exitGreen    = carInfo.isCarFinishWash ? true : false;
        appModule.localSts.led.exitRed      = carInfo.isCarFinishWash ? false : true;
/* **************************************************************** 测试组 ************************************************************* */
        appModule.localSts.communicateTest  = appModule.localCmd.communicateTest;
/* ************************************************************** IO点位更新 ************************************************************* */
        if(STA_EXCEPTION == wash.state || STA_STOP == wash.state){
            for (uint8_t board = 0; board < BOARD_NUMS; board++)    //主板和子板轮询
            {
                for (uint8_t i = 0; i < 2; i++)                     //输入和输出轮询
                {
                    char uploadBuf[20]   = {0};
                    char ioChangeBuff[50]= {0};
                    char ioBitState[10]  = {0};
                    uint8_t ioNum = (0 == board) ? 4 : 24;          //主板可用IO 4个，子板可用IO 24个
                    for (uint8_t pin = 0; pin < ioNum; pin++)       //输入或输出所有IO轮询
                    {
                        char tempBuf[10] = {0};
                        bool ioState[24] = {0};
                        switch (i)
                        {
                        case 0:
                            ioState[pin] = (0 == board) ? get_board_input_io(pin) : xp_io_read_input_pin(board, pin);   //读取输入状态
                            if(recordIoInputSta[board][pin] != ioState[pin]){
                                sprintf(tempBuf, "%d:%d-%d;", pin+1, recordIoInputSta[board][pin], ioState[pin]);
                                if(0 == board){
                                    sprintf(uploadBuf, "DI_%02d: %d--%d", pin+1, recordIoInputSta[board][pin], ioState[pin]);
                                }
                                else{
                                    sprintf(uploadBuf, "I%d_%02d: %d--%d", board-1, pin+1, recordIoInputSta[board][pin], ioState[pin]);
                                }
                                recordIoInputSta[board][pin] = ioState[pin];
                            }
                            break;
                        case 1:
                            ioState[pin] = (0 == board) ? get_board_output_io(pin) : xp_io_read_output_pin(board, pin); //读取输出状态
                            if(recordIoOuputSta[board][pin] != ioState[pin]){
                                sprintf(tempBuf, "%d:%d-%d;", pin+1, recordIoOuputSta[board][pin], ioState[pin]);
                                if(0 == board){
                                    sprintf(uploadBuf, "DO_%02d: %d--%d", pin+1, recordIoOuputSta[board][pin], ioState[pin]);
                                }
                                else{
                                    sprintf(uploadBuf, "Q%d_%02d: %d--%d", board-1, pin+1, recordIoOuputSta[board][pin], ioState[pin]);
                                }
                                recordIoOuputSta[board][pin] = ioState[pin];
                            }
                            break;
                        default:
                            break;
                        }
                        strcat(ioChangeBuff, (strlen(ioChangeBuff) > 25) ? "." : tempBuf);      //改变的IO过多，不再打印
                        if(strlen(ioChangeBuff) > 50){
                            while(1){
                                LOG_WARN("ioChangeBuff err!!!!!!!");
                                aos_msleep(100);
                            }
                        }

                        char ioBitTemp[5] ={0};
                        sprintf(ioBitTemp, "%d ", ioState[pin]);
                        strcat(ioBitState, ioBitTemp);
                        if(0 == (pin+1) % 4){
                            if(0 == i){
                                if((0 == board)) strcpy(appModule.localSts.bsDI_IO[board], ioBitState);
                                else             strcpy(appModule.localSts.bsDI_IO[(board - 1)*6 + ((pin+1) / 4)], ioBitState);
                            }
                            else if(1 == i){
                                if((0 == board)) strcpy(appModule.localSts.bsDO_IO[board], ioBitState);
                                else             strcpy(appModule.localSts.bsDO_IO[(board - 1)*6 + (pin+1) / 4], ioBitState);
                            }
                            memset(ioBitState, 0, sizeof(ioBitState));
                        }

                        //布尔量的IO点位赋值
                        // switch (i)
                        // {
                        // case 0: appModule.localSts.bDI_IO[board][pin] = ioState[pin];   break;  //读取输入状态
                        // case 1: appModule.localSts.bDO_IO[board][pin] = ioState[pin];   break;  //读取输出状态
                        // default: break;
                        // }
                    }

                    char *pIoInSta = NULL;
                    char *pIoOutSta = NULL;
                    pIoInSta  = appModule.localSts.sDI_IO[board];
                    pIoOutSta = appModule.localSts.sDO_IO[board];
                    if(pIoInSta != NULL && pIoOutSta != NULL){
                        //IO最新的一个状态改变赋值
                        if(strlen(uploadBuf) > 0){
                            strcpy((0 == i) ? pIoInSta : pIoOutSta, uploadBuf);
                        }
                        //改变的IO及其改变前后状态打印
                        if(strlen(ioChangeBuff) > 0){
                            if(0 == board){
                                LOG_DEBUG("Main Board %s IO change is %s", (0 == i) ? "IN" : "OUT", ioChangeBuff);
                            }
                            else{
                                LOG_DEBUG("IO Board %d %s IO change is %s", board-1, (0 == i) ? "IN" : "OUT", ioChangeBuff);
                            }
                        }
                    }
                    else{
                        LOG_UPLOAD("io state ponit is NULL");
                    }
                }
            }
        }

/* ********************************************************** 远程移动指令完成监测 ****************************************************** */
        for (uint8_t i = 0; i < APP_CRL_NUM; i++)
        {
            if(!appModule.isDevMove[i]) continue;
            
            bool isDevIdle = false;
            switch (i)
            {
            case APP_CRL_GATE_1_OPEN:
            case APP_CRL_GATE_1_CLOSE:
                isDevIdle = is_dev_move_sta_idle(GATE_1_MACH_ID);
                break;
            case APP_CRL_GATE_3_OPEN:
            case APP_CRL_GATE_3_CLOSE:
                isDevIdle = is_dev_move_sta_idle(GATE_3_MACH_ID);
                break;
            case APP_CRL_CONVEYOR_1:
                isDevIdle = is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID);
                break;
            case APP_CRL_CONVEYOR_2:
                isDevIdle = is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID);
                break;
            case APP_CRL_CONVEYOR_3:
                isDevIdle = is_dev_move_sta_idle(CONVEYOR_3_MATCH_ID);
                break;
            case APP_CRL_LIFTER:
                isDevIdle = is_dev_move_sta_idle(LIFTER_MATCH_ID);
                break;
            case APP_CRL_FRONT_LEFT_MOVE:
                isDevIdle = is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID);
                break;
            case APP_CRL_FRONT_RIGHT_MOVE:
                isDevIdle = is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID);
                break;
            case APP_CRL_BACK_LEFT_MOVE:
                isDevIdle = is_dev_move_sta_idle(BACK_LEFT_MOVE_MATCH_ID);
                break;
            case APP_CRL_BACK_RIGHT_MOVE:
                isDevIdle = is_dev_move_sta_idle(BACK_RIGHT_MOVE_MATCH_ID);
                break;
            default:
                break;
            }
            if(isDevIdle){
                appModule.isDevMove[i] = false;
                for (uint8_t j = 0; j < sizeof(AppCmdMap_Table) / sizeof(AppCmdMap_Table[0]); j++)
                {
                    if(AppCmdMap_Table[j].crlObj == i){
                        //这里需要手动区分指令值是bool还是int
                        if(APP_CRL_GATE_1_OPEN == i || APP_CRL_GATE_1_CLOSE == i
                        || APP_CRL_GATE_2_OPEN == i || APP_CRL_GATE_2_CLOSE == i
                        || APP_CRL_GATE_3_OPEN == i || APP_CRL_GATE_3_CLOSE == i
                        || APP_CRL_CONVEYOR_1 == i || APP_CRL_CONVEYOR_2 == i || APP_CRL_CONVEYOR_3 == i){
                            *(bool *)AppCmdMap_Table[j].data = 0;
                        }
                        else{
                            *(int *)AppCmdMap_Table[j].data = 0;
                        }
                        if(xp_cmd_excuted_complete)    xp_cmd_excuted_complete(AppCmdMap_Table[j].indentifer, 0);
                        break;
                    }
                }
            }
        }
    }
}

/*                                                         =======================                                                         */
/* ========================================================      控制指令执行      ======================================================== */
/*                                                         =======================                                                         */

void model_cmd_executed_thread(void *arg)
{
    uint32_t len;
    Type_PropertyNode_Def cmd = {0};
    void *pMem = aos_malloc(sizeof(Type_PropertyNode_Def)*5);
    aos_queue_new(&appModule.cmdQueue, pMem, sizeof(Type_PropertyNode_Def)*5, sizeof(Type_PropertyNode_Def));

    while(1){
        aos_queue_recv(&appModule.cmdQueue, AOS_WAIT_FOREVER, &cmd, &len);
        int dateValue = 0;
        if(TYPE_Bool == cmd.data_type){
            dateValue = (int)*(bool *)(cmd.data);
        }
        else if(TYPE_Int == cmd.data_type){
            dateValue = (int)*(int *)(cmd.data);
        }
        //非断电状态且处于停止状态下才允许调试操作
        bool isStopState = ((STA_EXCEPTION == wash.state || STA_STOP == wash.state) && false == get_emc_power_off_sta()) ? true : false;
        switch (cmd.cmd_id)
        {
        case CMD_WASH_FALG_1:
            if(true == appModule.localCmd.washFlag1){
                if(STA_IDLE == wash.state || STA_RUN == wash.state){
                    add_new_order(TEST_FALG_1_ORDER_NUM, false);
                }
                else{
                    if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_1", 0);
                }
            }
            break;
        case CMD_WASH_FALG_2:
            if(true == appModule.localCmd.washFlag2){
                if(STA_IDLE == wash.state || STA_RUN == wash.state){
                    add_new_order(TEST_FALG_2_ORDER_NUM, false);
                }
                else{
                    if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_2", 0);
                }
            }
            break;
        case CMD_HOME:
        case CMD_SAFE_HOME:
            if(true == appModule.localCmd.backHome){
                if(isStopState && true == appModule.localCmd.func.enableStation && true == get_allow_back_home_flag()) {
                    isGetBackHomeCmd = true;
                }
                else{
                    appModule.localCmd.backHome = false;
                    if(xp_cmd_excuted_complete) xp_cmd_excuted_complete(cmd.identifier, 0);
                    if(false == appModule.localCmd.func.enableStation){
                        LOG_UPLOAD("now dev is stop state, can not backhome");
                    }
                    else if(false == get_allow_back_home_flag()){
                        LOG_UPLOAD("now dev have error, can not backhome");
                    }
                }
            }
            break;
        case CMD_NEW_ORDER:
            if(appModule.localCmd.newOrder != 0){
                add_new_order(appModule.localCmd.newOrder, true);
            }
            break;
        case CMD_START_WASH:
            if(true == appModule.localCmd.startWash){
                wash.isGetStartCmd = true;
                appModule.localCmd.startWash = false;
                if(xp_cmd_excuted_complete) xp_cmd_excuted_complete(cmd.identifier, 0);
            }
            break;
        case CMD_WASH_MODE:
            if(NULL_WASH == appModule.localCmd.washMode
            || NORMAL_WASH == appModule.localCmd.washMode
            || QUICK_WASH == appModule.localCmd.washMode
            || FINE_WASH == appModule.localCmd.washMode){
                wash.washMode = (NULL_WASH == appModule.localCmd.washMode) ? FINE_WASH : appModule.localCmd.washMode;
            }
            break;
        case CMD_STOP:
            if(true == appModule.localCmd.stop){
                xp_service_set_state(STA_STOP);
                appModule.localCmd.stop = false;
                if(xp_cmd_excuted_complete) xp_cmd_excuted_complete(cmd.identifier, 0);
            }
            break;
        case CMD_CUSTOM_STOP_WASH:
            if(true == appModule.localCmd.customStopWash){
                xp_service_set_state(STA_SUSPEND);
                // appModule.localCmd.customStopWash = false;        //等报警归位切换到异常状态再清零
                set_error_state(273, true);
            }
            break;
        case CMD_ELECTRICAL_RESET:
            err_need_flag_handle()->isSetElectricalReset = appModule.localCmd.electricalReset;
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_electrical_reset", 0);
            break;
        case CMD_CANCEL_ORDER:
            //取消当前最新未启动订单
            for (uint8_t i = 0; i < MAX_QUEUE_ORDER_NUMBER; i++)
            {
                memcpy(&wash.orderQueue[i], &wash.orderQueue[i + 1], sizeof(Type_OrderInfo_Def));
            }
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_cancel_order", 0);
            break;
        case CMD_FLOODLIGHT:            app_crl_dev_set(APP_CRL_FLOODLIGHT, appModule.localCmd.floodlight); break;
        case CMD_AMBIENT_LIGHT:         app_crl_dev_set(APP_CRL_AMBIENT_LIGHT, appModule.localCmd.ambientLight); break;
        case CMD_COMMUNICATION_TEST:    appModule.localSts.communicateTest = appModule.localCmd.communicateTest; break;
        case CMD_DEBUG:                 xp_remoteCmd_deal(DEVICE_AG, "debug", appModule.localCmd.debug); break;

        //KV相关指令
        case CMD_ENABLE_STATION:
        case CMD_LOG:
        case CMD_DETECT_LIFTER_LOOSE:
        case CMD_DETECT_WATER_PRESS:
        case CMD_DETECT_SHAMPOO:
        case CMD_DETECT_WAXWATER:
        case CMD_DETECT_SEWAGE:
        case CMD_ENABLE_SEWAGE_PUMP:
        case CMD_ENABLE_TOP_SWING:
        case CMD_ENABLE_LEFT_SWING:
        case CMD_ENABLE_RIGHT_SWING:
        case CMD_ENABLE_SKIRT_BRUSH:
        case CMD_DETECT_ALLIN_SIGNAL:
        case CMD_DETECT_SKIRT_SIGNAL:
        case CMD_ENABLE_DRYER16:
        case CMD_ENABLE_DRYER25:
        case CMD_ENABLE_DRYER34:
        case CMD_ENABLE_GATE1:
        case CMD_ENABLE_DRYER_MODE:
        case CMD_ENABLE_AUTO_RESET:
        case CMD_ENABLE_HIGH_PRESS_WASH:
        case CMD_DETECT_SUPER_HIGH:
        case CMD_DETECT_ALL_COLLISION:
        case CMD_DETECT_FLEFT_COLLISION:
        case CMD_DETECT_FRIGHT_COLLISION:
        case CMD_DETECT_BLEFT_COLLISION:
        case CMD_DETECT_BRIGHT_COLLISION:
        case CMD_HIGH_PRESS_END_POS:
        case CMD_SKIRT_BRUSH_START_POS:
        case CMD_SKIRT_BRUSH_END_POS:
        case CMD_SHAMPOO_START_POS:
        case CMD_SHAMPOO_END_POS:
        case CMD_TOP_BRUSH_START_POS:
        case CMD_TOP_BRUSH_END_POS:
        case CMD_FRONT_BRUSH_START_POS:
        case CMD_BACK_BRUSH_START_POS:
        case CMD_BACK_BRUSH_END_POS:
        case CMD_FRONT_BRUSH_END_POS:
        case CMD_WAXWATER_START_POS:
        case CMD_WAXWATER_END_POS:
        case CMD_DRYER_START_POS:
        case CMD_DRYER_END_POS:
            for (uint8_t i = 0; i < sizeof(KvDataMap_Table) / sizeof(KvDataMap_Table[0]); i++)
            {
                if(cmd.cmd_id == KvDataMap_Table[i].cmd){
                    if(KvDataMap_Table[i].matchAppModule){
                        if(KvDataMap_Table[i].matchKvBit != 0xFF){
                            if(dateValue != 0){
                                *(int*)KvDataMap_Table[i].pData |= (1 << KvDataMap_Table[i].matchKvBit);
                            }
                            else{
                                *(int*)KvDataMap_Table[i].pData &= ~(1 << KvDataMap_Table[i].matchKvBit);
                            }
                        }
                        else{
                            memcpy(KvDataMap_Table[i].pData, KvDataMap_Table[i].matchAppModule, KvDataMap_Table[i].len);
                        }
                    }
                    if(0 == xp_save_kv_params(KvDataMap_Table[i].key, KvDataMap_Table[i].pData, KvDataMap_Table[i].len)){
                        LOG_INFO("KV save %s to %d success", KvDataMap_Table[i].key, *(int*)KvDataMap_Table[i].pData);      //这里打印不兼容除int类型外的值
                    }
                    
                    if(CMD_ENABLE_STATION == cmd.cmd_id && 0 == dateValue)    xp_service_set_state(STA_STOP);
                    else if(CMD_DETECT_LIFTER_LOOSE == cmd.cmd_id)      err_need_flag_handle()->isDetectLifterLooseEnable = appModule.localCmd.func.detectLifterLoose;
                    else if(CMD_DETECT_WATER_PRESS == cmd.cmd_id)       err_need_flag_handle()->isDetectWaterPressEnable = appModule.localCmd.func.detectWaterPress;
                    else if(CMD_DETECT_SHAMPOO == cmd.cmd_id)           err_need_flag_handle()->isDetectShampooEnable = appModule.localCmd.func.detectShampoo;
                    else if(CMD_DETECT_WAXWATER == cmd.cmd_id)          err_need_flag_handle()->isDetectWaxwaterEnable = appModule.localCmd.func.detectWaxwater;
                    else if(CMD_DETECT_SEWAGE == cmd.cmd_id)            err_need_flag_handle()->isDetectSewageEnable = appModule.localCmd.func.detectSewage;
                    else if(CMD_DETECT_CLEANER == cmd.cmd_id)           err_need_flag_handle()->isDetectCleanerLess = appModule.localCmd.func.detectCleaner;
                    else if(CMD_DETECT_DRIER == cmd.cmd_id)             err_need_flag_handle()->isDetectDyierLess = appModule.localCmd.func.detectDrier;
                    else if(CMD_DETECT_FLEFT_COLLISION == cmd.cmd_id)   err_need_flag_handle()->isFrontLeftCollisionEnable = appModule.localCmd.func.detectFLeftCollision;
                    else if(CMD_DETECT_FRIGHT_COLLISION == cmd.cmd_id)  err_need_flag_handle()->isFrontRightCollisionEnable = appModule.localCmd.func.detectFRightCollision;
                    else if(CMD_DETECT_BLEFT_COLLISION == cmd.cmd_id)   err_need_flag_handle()->isBackLeftCollisionEnable = appModule.localCmd.func.detectBLeftCollision;
                    else if(CMD_DETECT_BRIGHT_COLLISION == cmd.cmd_id)  err_need_flag_handle()->isBackRightCollisionEnable = appModule.localCmd.func.detectBRightCollision;
                    else if(CMD_HIGH_PRESS_END_POS == cmd.cmd_id)       get_washProcPos_Obj()->endHighPump      = appModule.localCmd.adjust.highPressEndPos;
                    else if(CMD_SKIRT_BRUSH_START_POS == cmd.cmd_id)    get_washProcPos_Obj()->startSkirtBrush  = appModule.localCmd.adjust.skirtBrushStartPos;
                    else if(CMD_SKIRT_BRUSH_END_POS == cmd.cmd_id)      get_washProcPos_Obj()->endSkirtBrush    = appModule.localCmd.adjust.skirtBrushEndPos;
                    else if(CMD_SHAMPOO_START_POS == cmd.cmd_id)        get_washProcPos_Obj()->startShampoo     = appModule.localCmd.adjust.shampooStartPos;
                    else if(CMD_SHAMPOO_END_POS == cmd.cmd_id)          get_washProcPos_Obj()->endShampoo       = appModule.localCmd.adjust.shampooEndPos;
                    else if(CMD_TOP_BRUSH_START_POS == cmd.cmd_id)      get_washProcPos_Obj()->startTopBrush    = appModule.localCmd.adjust.topBrushStartPos;
                    else if(CMD_TOP_BRUSH_END_POS == cmd.cmd_id)        get_washProcPos_Obj()->endTopBrush      = appModule.localCmd.adjust.topBrushEndPos;
                    else if(CMD_FRONT_BRUSH_START_POS == cmd.cmd_id)    get_washProcPos_Obj()->startFrontBrush  = appModule.localCmd.adjust.frontBrushStartPos;
                    else if(CMD_BACK_BRUSH_START_POS == cmd.cmd_id)     get_washProcPos_Obj()->startBackBrush   = appModule.localCmd.adjust.backBrushStartPos;
                    else if(CMD_BACK_BRUSH_END_POS == cmd.cmd_id)       get_washProcPos_Obj()->endBackBrush     = appModule.localCmd.adjust.backBrushEndPos;
                    else if(CMD_FRONT_BRUSH_END_POS == cmd.cmd_id)      get_washProcPos_Obj()->endFrontBrush    = appModule.localCmd.adjust.frontBrushTailEndPos;
                    else if(CMD_WAXWATER_START_POS == cmd.cmd_id)       get_washProcPos_Obj()->startWaxwater    = appModule.localCmd.adjust.waxwaterStartPos;
                    else if(CMD_WAXWATER_END_POS == cmd.cmd_id)         get_washProcPos_Obj()->endWaxwater      = appModule.localCmd.adjust.waxwaterEndPos;
                    else if(CMD_DRYER_START_POS == cmd.cmd_id)          get_washProcPos_Obj()->startDrying      = appModule.localCmd.adjust.dryerStartPos;
                    else if(CMD_DRYER_END_POS == cmd.cmd_id)            get_washProcPos_Obj()->endDrying        = appModule.localCmd.adjust.dryerEndPos;
                    break;
                }
            }
            break;

        default:
            for (uint8_t i = 0; i < sizeof(AppCmdMap_Table) / sizeof(AppCmdMap_Table[0]); i++)
            {
                if(cmd.cmd_id == AppCmdMap_Table[i].cmd){
                    if((CMD_GATE_2_OPEN == cmd.cmd_id || CMD_GATE_2_CLOSE == cmd.cmd_id)
                    && STA_IDLE == wash.state){
                        isStopState = true;             //待机状态道闸2也允许动作，保障用户在预备想离开时可以向前离开
                    }
                    if(isStopState){
                        app_crl_dev_set(AppCmdMap_Table[i].crlObj, dateValue);
                        appModule.isDevMove[AppCmdMap_Table[i].crlObj] = true;
                    }
                    else{
                        if(xp_cmd_excuted_complete) xp_cmd_excuted_complete(cmd.identifier, 0);
                    }
                    break;
                }
            }
            break;
        }
        //同一机构控制动作时，把相反的动作点位清零
        if(CMD_GATE_1_OPEN == cmd.cmd_id && dateValue != 0){
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_gate_1_close", 0);
        }
        else if(CMD_GATE_1_CLOSE == cmd.cmd_id && dateValue != 0){
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_gate_1_open", 0);
        }
        else if(CMD_GATE_2_OPEN == cmd.cmd_id && dateValue != 0){       //2号闸由于没有结束检测，这里直接清除远程状态位
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_gate_2_open", 0);
            customerOpenGate2TimeStamp = aos_now_ms();
            isCarWantForwardExit = true;
        }
        else if(CMD_GATE_2_CLOSE == cmd.cmd_id && dateValue != 0){
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_gate_2_close", 0);
            isCarWantForwardExit = false;
        }
        else if(CMD_GATE_3_OPEN == cmd.cmd_id && dateValue != 0){
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_gate_3_close", 0);
        }
        else if(CMD_GATE_3_CLOSE == cmd.cmd_id && dateValue != 0){
            if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_gate_3_open", 0);
        }
        cmd.cmd_id = CMD_NULL;
    }
}


/*                                                         =======================                                                         */
/* ========================================================      停车状态检测      ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       预备区车辆监测线程
 * @param[in]	arg                 
 */
void ready_area_detection_thread(void *arg)
{
    uint64_t carInReadyAreaTimeStamp = 0;
    uint64_t carOutReadyAreaTimeStamp = 0;
    bool isCarEntryWorkArea = false;
    bool isPlayForwardVoice = false;
    uint8_t longCarTryCnt = 0;
    bool isCarTooLongTriggerEntrance = false;

    while(1){
        if(wash.state != STA_IDLE && wash.state != STA_RUN && !wash.isWarningErr){    //非待机和运行状态下，不检测预备区车辆信息，洗车过程中有收到警告错误暂停接待后续车辆
            aos_msleep(500);
            carInfo.parkState = PARK_EMPTY;
            carInfo.isAllowToWash = false;
            continue;
        }
        if(true == get_new_car_ready_wash_falg()){  //已经有车准备进入工作区清洗则暂停预备区停车检测
            isCarEntryWorkArea = true;
            carInfo.isAllowToWash = false;          //车辆刚进入时不允许其它车启动
            aos_msleep(500);
            continue;
        }
        else if(isCarEntryWorkArea){                //有车完全进入工作区后，关闭2#道闸
            isCarEntryWorkArea = false;
            isPlayForwardVoice = false;             //车辆从预备区到工作区离开停车光电不需要报前进语音
            carInReadyAreaTimeStamp = aos_now_ms(); //提供道闸1开的判定时间戳（过一段时间再认为车辆不在预备区）
            gate_change_state(CRL_SECTION_2, GATE_CLOSE);
        }

        //待机和工作状态，若没有车正在进入工作区，则一直尝试关闭道闸2#
        if(get_diff_ms(customerOpenGate2TimeStamp) > 30000 && !is_signal_filter_trigger(SIGNAL_ENTRANCE)
        && (is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) || is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN))){
            gate_change_state(CRL_SECTION_2, GATE_CLOSE);
            isCarWantForwardExit = false;
        }

        //出口和完成光电同时触发，认为车已经向前离开，关闭2号闸
        if(isCarWantForwardExit && is_signal_filter_trigger(SIGNAL_EXIT) && is_signal_filter_trigger(SIGNAL_FINISH)){
            gate_change_state(CRL_SECTION_2, GATE_CLOSE);
            isCarWantForwardExit = false;
        }

        //1号道闸的关闭需要有一个触发信号，不能用时间判定，避免开闸后长时间未进入预备区道闸闸门关闭
        //触发左右停偏光电或者停车光电认为车辆已经进入预备区，安全前提下关闭1号道闸
        if(is_signal_filter_trigger(SIGNAL_LEFT_SKEW) || is_signal_filter_trigger(SIGNAL_RIGHT_SKEW) || is_signal_filter_trigger(SIGNAL_STOP)){
            carInfo.isCarInReadyArea = true;
            carOutReadyAreaTimeStamp = aos_now_ms();
            if(!is_signal_filter_trigger(SIGNAL_ALL_IN) && !is_signal_filter_trigger(SIGNAL_GATE_1_PROTECT)){
                if(is_dev_move_sta_idle(GATE_1_MACH_ID) && !is_signal_filter_trigger(SIGNAL_GATE_1_CLOSE)){
                    gate_change_state(CRL_SECTION_1, GATE_CLOSE);
                }
            }
        }
        //长时间没有触发预备区的光电则认为没有车在预备区，有订单的话需要重新打开道闸
        else if(!is_signal_filter_trigger(SIGNAL_ALL_IN) && get_diff_ms(carOutReadyAreaTimeStamp) > 5000){
            carInfo.isCarInReadyArea = false;
            longCarTryCnt = 0;
        }

        if(isCarWantForwardExit){                                   //客户不想洗操作2号道闸向前离开时显示请前进
            //语音停止只一遍，避免一直通讯
            if(carInfo.parkState != PARK_TOO_BACK) voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_SILENCE);
            xp_ag_osal_get()->screen.display(AG_CAR_FORWARD);
            carInfo.parkState = PARK_TOO_BACK;
            longCarTryCnt = 0;
        }
        else if(is_signal_filter_trigger(SIGNAL_LEFT_SKEW)){        //左偏光电触发，说明停车偏左
            if(PARK_TOO_LEFT != carInfo.parkState){
                carInfo.voiceCnt = 0;
                xp_ag_osal_get()->screen.display(AG_CAR_LEFT_SKEW);
            }
            if(0 == carInfo.voiceCnt 
            || (get_diff_ms(carInfo.parkStaStartT) > 10000 && carInfo.voiceCnt < 2)){     //限制语音播报时间和次数
                carInfo.voiceCnt++;
                voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_CAR_LEFT);
                carInfo.parkStaStartT = aos_now_ms();
            }
            carInfo.parkState = PARK_TOO_LEFT;
        }
        else if(is_signal_filter_trigger(SIGNAL_RIGHT_SKEW)){    //右偏光电触发，说明停车偏右
            if(PARK_TOO_RIGHT != carInfo.parkState){
                carInfo.voiceCnt = 0;
                xp_ag_osal_get()->screen.display(AG_CAR_RIGHT_SKEW);
            }
            if(0 == carInfo.voiceCnt 
            || (get_diff_ms(carInfo.parkStaStartT) > 10000 && carInfo.voiceCnt < 2)){     //限制语音播报时间和次数
                carInfo.voiceCnt++;
                voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_CAR_RIGHT);
                carInfo.parkStaStartT = aos_now_ms();
            }
            carInfo.parkState = PARK_TOO_RIGHT;
        }
        else if(is_signal_filter_trigger(SIGNAL_STOP)){
            carInReadyAreaTimeStamp = aos_now_ms();
            isPlayForwardVoice = true;
            //判断当前停车位置是否合适
            if(longCarTryCnt >= 3 || is_signal_filter_trigger(SIGNAL_ALL_IN)){
                if(is_signal_filter_trigger(SIGNAL_ENTRANCE)){
                    isCarTooLongTriggerEntrance = true;
                    if(longCarTryCnt < 3) carInfo.parkState = PARK_EMPTY; //清除状态，使后面显示正确
                    longCarTryCnt = 3;      //全进、停车、入口光电全部触发直接显示超长
                }
                else if(isCarTooLongTriggerEntrance){
                    longCarTryCnt++;
                    isCarTooLongTriggerEntrance = false;
                }
                if(PARK_TOO_LONG != carInfo.parkState){
                    carInfo.voiceCnt = 0;
                    //停车光电和全进光电同时挡住时，车超长，尝试让车往前开一点靠近入口光电的位置
                    if(longCarTryCnt < 3){
                        xp_ag_osal_get()->screen.display(AG_CAR_FORWARD);
                    }
                    else{
                        xp_ag_osal_get()->screen.display(AG_CAR_TOO_LONG);
                    }
                }
                if(0 == carInfo.voiceCnt 
                || (get_diff_ms(carInfo.parkStaStartT) > 10000 && carInfo.voiceCnt < 1)){     //限制语音播报时间和次数
                    carInfo.voiceCnt++;
                    if(longCarTryCnt < 3){
                        voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_CAR_FORWARD);
                    }
                    else{
                        voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_CAR_TOO_LONG);
                    }
                    carInfo.parkStaStartT = aos_now_ms();
                }
                carInfo.parkState = PARK_TOO_LONG;
            }
            else if(is_signal_filter_trigger(SIGNAL_ENTRANCE)){      //停车光电和入口光电都触发，说明停车太靠前
                isCarTooLongTriggerEntrance = true;
                if(PARK_TOO_FRONT != carInfo.parkState){
                    carInfo.voiceCnt = 0;
                    xp_ag_osal_get()->screen.display(AG_CAR_BACKOFF);
                }
                if(0 == carInfo.voiceCnt 
                || (get_diff_ms(carInfo.parkStaStartT) > 10000 && carInfo.voiceCnt < 2)){     //限制语音播报时间和次数
                    carInfo.voiceCnt++;
                    voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_SLOW_BACKOFF);
                    carInfo.parkStaStartT = aos_now_ms();
                }
                carInfo.parkState = PARK_TOO_FRONT;
            }
            else{                                                                   //仅停车光电触发，说明停好车
                if(PARK_OK != carInfo.parkState){
                    carInfo.voiceCnt = 0;
                    xp_ag_osal_get()->screen.display((get_is_allow_next_car_wash_flag() && wash.orderQueue[0].numberId != 0) ? AG_CAR_READY_TRANSFER : AG_CAR_STOP);
                    carInfo.parkOkTimeStamp = aos_now_ms();
                }
                // else if(get_is_allow_next_car_wash_flag() && get_diff_ms(carInfo.parkOkTimeStamp) > 18000){
                //     xp_ag_osal_get()->screen.display((0 == appModule.localCmd.scanCode) ? AG_SCAN_CODE : AG_CAR_STOP);
                // }
                // if(get_is_allow_next_car_wash_flag()){
                    if(0 == carInfo.voiceCnt
                    || (get_diff_ms(carInfo.parkStaStartT) > 15000 && carInfo.voiceCnt < 1)){
                        carInfo.voiceCnt++;
                        voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_CAR_STOP);
                        carInfo.parkStaStartT = aos_now_ms();
                    }
                // }
                carInfo.parkState = PARK_OK;
            }
        }
        else{
            //停车光电未触发，提示前进
            if(PARK_TOO_BACK != carInfo.parkState){
                carInfo.voiceCnt = 0;
                xp_ag_osal_get()->screen.display(AG_CAR_FORWARD);
            }
            if(isPlayForwardVoice){
                if(0 == carInfo.voiceCnt || get_diff_ms(carInfo.parkStaStartT) > 10000){
                    carInfo.voiceCnt++;
                    voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_CAR_FORWARD);
                    carInfo.parkStaStartT = aos_now_ms();
                }
                if(carInfo.voiceCnt > 1){
                    isPlayForwardVoice = false;
                }
            }
            carInfo.parkState = PARK_TOO_BACK;
        }

        //有订单且无车辆在预备区准备则打开1号道闸
        

        //只要触发全进光电就打开1号道闸，使在预备区的无订单车辆可以向后退出
        if(is_signal_filter_trigger(SIGNAL_ALL_IN)){
            if(is_dev_move_sta_idle(GATE_1_MACH_ID) && !is_signal_filter_trigger(SIGNAL_GATE_1_OPEN)){
                gate_change_state(CRL_SECTION_1, GATE_OPEN);
            }
        }
        else if(!carInfo.isCarInReadyArea){     //没车在预备区的话根据有无订单切换道闸开关
            if(wash.orderQueue[0].numberId != 0){
                if(is_dev_move_sta_idle(GATE_1_MACH_ID) && !is_signal_filter_trigger(SIGNAL_GATE_1_OPEN)){
                    gate_change_state(CRL_SECTION_1, GATE_OPEN);
                }
            }
            else{
                if(is_dev_move_sta_idle(GATE_1_MACH_ID) && !is_signal_filter_trigger(SIGNAL_GATE_1_CLOSE)){
                    gate_change_state(CRL_SECTION_1, GATE_CLOSE);
                }
            }
        }

        //允许洗车判定
        carInfo.isAllowToWash = (PARK_OK == carInfo.parkState) ? true : false;
        aos_msleep(100);
    }
}

/*                                                         =======================                                                         */
/* ========================================================     设备状态机控制     ======================================================== */
/*                                                         =======================                                                         */

//状态字符查询表
static const char StateStrList[][10] = { "INIT", "IDLE", "RUN", "SUSPEND", "PAUSE", "EXCEPTION", "RECOVER", "COMPELTE", "STOP" };
static const char* xp_get_state_str(Type_ServiceState_Enum state)
{
    return StateStrList[state];
}

/**
 * @brief       切换设备服务状态
 * @param[in]	state               目标状态
 * @return      int                
 */
static int xp_service_set_state(Type_ServiceState_Enum state)
{
    Type_ServiceState_Enum currState = wash.state;

    if (wash.state != state) {
        //暂停运营->恢复
        if (STA_STOP == wash.state 
        && (STA_RECOVER == state)) {
            wash.state = state;
        }
        //运行->暂停/完成/异常挂起========空闲（AG特殊，无完成状态）
        else if (STA_RUN == wash.state 
        && (STA_PAUSE == state || STA_COMPELTE == state || STA_SUSPEND == state || STA_IDLE == state)) {
            wash.state = state;
        }
        //异常恢复->暂停/空闲/异常挂起
        else if (STA_RECOVER == wash.state 
        && (STA_PAUSE == state || STA_IDLE == state || STA_SUSPEND == state)) {
            wash.state = state;
        }
        //异常挂起->运行
        else if (STA_SUSPEND == wash.state 
        && STA_RUN == state) {
            wash.state = state;
        }
        //异常停止->异常恢复
        else if (STA_EXCEPTION == wash.state 
        && (STA_RECOVER == state)) {
            wash.state = state;
        }
        //空闲->运行/异常挂起
        else if (STA_IDLE == wash.state 
        && (STA_RUN == state || STA_SUSPEND == state)) {
            wash.state = state;
        }
        //暂停->运行/异常挂起
        else if (STA_PAUSE == wash.state 
        && (STA_RUN == state || STA_SUSPEND == state)) {
            wash.state = state;
        }
        //完成->空闲
        else if (STA_COMPELTE == wash.state 
        && (STA_IDLE == state || STA_SUSPEND == state)) {
            wash.state = state;
        }
        else if(STA_STOP == state || STA_EXCEPTION == state){   //任何状态都能转到暂停运营和异常状态
            wash.state = state;
        }
        else {
            LOG_UPLOAD("Switch not allowed, %s -> %s\r\n", xp_get_state_str(currState), xp_get_state_str(state));
            return -1;				    //不允许的状态切换
        }

        wash.lastState = currState;
        wash.isFirstSwitch = true;
        LOG_UPLOAD("State switch OK, %s -> %s\r\n", xp_get_state_str(currState), xp_get_state_str(state));

        //后侧刷只在停止或异常状态下移动限位超过60
        if(STA_STOP == wash.state || STA_EXCEPTION == wash.state){
            osal_set_dev_limit_mode(BACK_LEFT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, 110);
            osal_set_dev_limit_mode(BACK_RIGHT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, 110);
            //异常停止后清除所有订单
            memset(wash.orderQueue, 0, sizeof(wash.orderQueue));
        }
        else{
            osal_set_dev_limit_mode(BACK_LEFT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, 60);
            osal_set_dev_limit_mode(BACK_RIGHT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, 60);
        }
        err_need_flag_handle()->isDevIdleSta                = (STA_IDLE == wash.state) ? true : false;
        err_need_flag_handle()->isDevRunSta                 = (STA_RUN == wash.state) ? true : false;
        //待机空闲或完成状态下不检测防撞
        err_need_flag_handle()->isAllCollisionEnable        = (STA_IDLE == wash.state || STA_COMPELTE == wash.state) ? false : appModule.localCmd.func.detectAllCollision;
        //非运行状态下不检测超高
        err_need_flag_handle()->isDetectSuperHighEnable     = (STA_RUN == wash.state) ? appModule.localCmd.func.detectSuperHigh : false;
        //只在工作状态监测水压
        err_need_flag_handle()->isDetectWaterPressEnable    = (STA_RUN == wash.state) ? appModule.localCmd.func.detectWaterPress : false;
    }
    else {
        if(STA_STOP == state || STA_EXCEPTION == state){
            stop_all_dev_run();
        }
        printf("Already in %s state\r\n", xp_get_state_str(currState));
    }
    return 0;
}

/**
 * @brief       增加新订单
 * @param[in]	orderNum            
 * @param[in]	isOnline            
 * @return      int                 
 */
int add_new_order(int orderNum, bool isOnline)
{
    uint8_t i = 0;
    if(orderNum != 0){
        for (i = 0; i < MAX_QUEUE_ORDER_NUMBER; i++)
        {
            if(0 == wash.orderQueue[i].numberId){
                wash.orderQueue[i].isOnlineOrder = isOnline;
                wash.orderQueue[i].isParkingOk = false;
                wash.orderQueue[i].isOrderStarted = false;
                wash.orderQueue[i].numberId = orderNum;
                LOG_UPLOAD("Add order ID <%d>, currently %d orders have been accumulated", orderNum, i + 1);
                break;
            }
        }
        if(i >= MAX_QUEUE_ORDER_NUMBER) LOG_UPLOAD("Order too much, add id <%d> failed, MAX %d", orderNum, MAX_QUEUE_ORDER_NUMBER);
        else return 0;
    }
    else{
        LOG_UPLOAD("Illegal order number, add failed");
    }
    return -1;
}

/**
 * @brief       准备洗车前的准备动作
 * @param[in]	state               
 * @return      int                 
 */
static int xp_service_start_wash_ready(Type_ServiceState_Enum state)
{
    //连续洗测试
    if(isAutoOrder){
        if(++kvService.order.offlineStartNum > 9999999)   kvService.order.offlineStartNum = 0;
        kvService.order.isNewOfflineOrderSave = true;
        appModule.localSts.washInfo.offlineOrderNum = wash.washMode*110000000 + kvService.order.offlineStartNum;      //上传订单号（包含洗车模式信息）
        add_new_order(appModule.localSts.washInfo.offlineOrderNum, false);
    }

    LOG_INFO("Device start wash, ID <%d>", wash.orderQueue[0].numberId);
    osal_dev_io_state_change(BOARD5_OUTPUT_MACHINE_IDEL, IO_ENABLE);
    clearOfflineOrdeBusyCnt = 0;
    err_need_flag_handle()->isDetectBrushCroooked = true;
    gate_change_state(CRL_SECTION_2, GATE_OPEN);
    set_new_car_ready_wash_falg(true);
    set_is_allow_next_car_wash_flag(false);
    xp_ag_osal_get()->screen.display(AG_CAR_WASH_STARTING);
    voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_WASH_START);
    if(STA_IDLE == state) wash_crl_variable_init();       //待机状态启动，前面肯定没车，重置所有变量
    carInfo.isCarInReadyArea = false;
    kvService.order.isNewOrder = true;
    if(0 == appModule.localSts.order1.orderNumber){
        appModule.localSts.order1.orderNumber = wash.orderQueue[0].numberId;
        if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_1", 1);
        set_new_order_car_id(1);
    }
    else if(0 == appModule.localSts.order2.orderNumber){
        appModule.localSts.order2.orderNumber = wash.orderQueue[0].numberId;
        if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_2", 1);
        set_new_order_car_id(2);
    }
    else{
        set_new_order_car_id(0);                //编号0的Id不参与洗车流程
        LOG_UPLOAD("Over max order number");
        return -1;
    }
    if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_new_order", 0);
    
    //启动后消耗当前订单号
    for (uint8_t i = 0; i < MAX_QUEUE_ORDER_NUMBER; i++)
    {
        memcpy(&wash.orderQueue[i], &wash.orderQueue[i + 1], sizeof(Type_OrderInfo_Def));
    }
    return 0;
}

static void offline_payment_callback(uint8_t washMode)
{
    switch (washMode)
    {
    case 1: wash.washMode = NORMAL_WASH; break;
    case 2: wash.washMode = NORMAL_WASH; break;
    case 3: wash.washMode = NORMAL_WASH; break;
    default:
        LOG_UPLOAD("Not support this mode");
        return;
        break;
    }
    osal_dev_io_state_change(BOARD5_OUTPUT_MACHINE_IDEL, IO_ENABLE);
    //最高位（第9位）表示洗车模式，次高位（第8位）表示按键或者收费机（0为收费机，1为按键）
    //低7位表示线下订单启动数量，每单+1，加到9999999后循环
    if(++kvService.order.offlineStartNum > 9999999)   kvService.order.offlineStartNum = 0;
    kvService.order.isNewOfflineOrderSave = true;
    appModule.localSts.washInfo.offlineOrderNum = wash.washMode*100000000 + kvService.order.offlineStartNum;      //上传订单号（包含洗车模式信息）
    add_new_order(appModule.localSts.washInfo.offlineOrderNum, false);
}

#define BUTTOM_NUM      6               //物理按键个数

typedef enum {
	BUTTON_START_WASH = 0,				//启动洗车按键
	BUTTON_STOP_WASH,					//停止洗车按键
	BUTTON_POWER_RESET,					//电源复位按键
	BUTTON_PAUSE_RESUME,				//暂停继续按键
	BUTTON_ENTRY_START,				    //入口处启动按键
    BUTTON_NUM,
} Type_ButtonType_Enum;

// uint8_t buttomTrigCnt[BUTTON_NUM] = {0};
bool recordIsButtomTrig[BUTTON_NUM] = {0};
bool isButtomTrig[BUTTON_NUM] = {0};

static void check_button(void)
{
    //物理按键检测
    for (uint8_t i = 0; i < BUTTON_NUM; i++)
    {
        if(BUTTON_START_WASH == i)          isButtomTrig[i] = is_signal_filter_trigger(SIGNAL_BUTTON_START) ? true : false;
        else if(BUTTON_STOP_WASH == i)      isButtomTrig[i] = is_signal_filter_trigger(SIGNAL_BUTTON_STOP) ? true : false;
        else if(BUTTON_POWER_RESET == i)    isButtomTrig[i] = is_signal_filter_trigger(SIGNAL_BUTTON_RESET) ? true : false;
        else if(BUTTON_PAUSE_RESUME == i)   isButtomTrig[i] = is_signal_filter_trigger(SIGNAL_BUTTON_PAUSE) ? true : false;
        else if(BUTTON_ENTRY_START == i)    isButtomTrig[i] = is_signal_filter_trigger(SIGNAL_BUTTON_ENTRY_START) ? true : false;
        if(recordIsButtomTrig[i] != isButtomTrig[i]){
            if(!recordIsButtomTrig[i]){
                if(BUTTON_START_WASH == i || BUTTON_ENTRY_START == i){
                    // appModule.localCmd.newOrder = TEST_LOCAL_ORDER_NUM;
                    //最高位（第9位）表示洗车模式，次高位（第8位）表示按键或者收费机（0为收费机，1为按键）
                    //低7位表示线下订单启动数量，每单+1，加到9999999后循环
                    if(++kvService.order.offlineStartNum > 9999999)   kvService.order.offlineStartNum = 0;
                    kvService.order.isNewOfflineOrderSave = true;
                    appModule.localSts.washInfo.offlineOrderNum = wash.washMode*100000000 + 10000000 + kvService.order.offlineStartNum;      //上传订单号（包含洗车模式信息）
                    add_new_order(appModule.localSts.washInfo.offlineOrderNum, false);
                }
                else if(BUTTON_STOP_WASH == i){
                    xp_service_set_state(STA_STOP);
                }
                else if(BUTTON_POWER_RESET == i){
                    if(false == get_emc_power_off_sta()){
                        isGetBackHomeCmd = true;
                    }
                }
                else if(BUTTON_PAUSE_RESUME == i){
                }
            }
            recordIsButtomTrig[i] = isButtomTrig[i];
        }
    }
}

/**
 * @brief       设备服务状态机控制线程
 * @param[in]	arg                 
 */
void xp_service_thread(void* arg)
{
    uint8_t printCnt = 0;
    uint64_t timeStamp = 0;
    uint64_t carBeReadyTimeStamp = 0;
    uint64_t voiceTimeStamp = 0;
    uint8_t voiceCnt = 0;
    uint8_t signalTriggerCnt = 0;
    uint64_t devIdelStaTimeStamp = 0;
    bool isAutoResetDev = false;
    bool isCarBeReady = false;
    bool startWashFlag = false;
    int ret;

    while (1)
    {
        if (0 == printCnt++ % 50) {
            LOG_INFO("======now service state is %s========", xp_get_state_str(wash.state));
        }
        //确认按键（仅手动模式下有效）
        if(appModule.localCmd.func.enableManualMode) check_button();
        //识别当前是否可以启动新订单车辆
        if(get_is_allow_next_car_wash_flag()){
            if(carInfo.isAllowToWash && wash.orderQueue[0].numberId != 0){  //车辆停车位置准确后等待一段时间后启动
                if(wash.orderQueue[0].isOnlineOrder){   //线上订单等待用户点击启动后开始洗车
                    if(wash.isGetStartCmd){
                        startWashFlag = true;
                    }
                }
                else{
                    if(!isCarBeReady){
                        isCarBeReady = true;
                        carBeReadyTimeStamp = aos_now_ms();
                    }
                    else if(get_diff_ms(carBeReadyTimeStamp) > 3000){
                        startWashFlag = true;
                    }
                }
            }
            else{
                isCarBeReady = false;
                startWashFlag = false;
                wash.isGetStartCmd = false;
            }
        }
        //有车结束出口显示绿灯，离开完成光电后红灯
        if(STA_IDLE == wash.state || STA_RUN == wash.state){
            if(!is_signal_filter_trigger(SIGNAL_FINISH)){
                carInfo.isCarFinishWash = false;
            }
            if(isCarWantForwardExit){           //客户不想洗操作2号道闸向前离开时显示绿灯
                osal_dev_io_state_change(BOARD1_OUTPUT_RED_LIGHT_EXIT, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_GREEN_LIGHT_EXIT, IO_ENABLE);
            }
            else if(carInfo.isCarFinishWash){
                osal_dev_io_state_change(BOARD1_OUTPUT_RED_LIGHT_EXIT, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_GREEN_LIGHT_EXIT, IO_ENABLE);
                if(voiceCnt < 5 && get_diff_ms(voiceTimeStamp) > 10000){
                    voiceCnt++;
                    voiceTimeStamp = aos_now_ms();
                    voice_play_set(AG_VOICE_POS_EXIT, AG_VOICE_COMPLETED);
                }
            }
            else{
                voiceCnt = 0;
                osal_dev_io_state_change(BOARD1_OUTPUT_RED_LIGHT_EXIT, IO_ENABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_GREEN_LIGHT_EXIT, IO_DISABLE);
            }
        }
        //停止或异常状态道闸2如果打开则显示绿灯允许通过
        else if(STA_STOP == wash.state || STA_EXCEPTION == wash.state){
            if(is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) && is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)){
                osal_dev_io_state_change(BOARD1_OUTPUT_RED_LIGHT_EXIT, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_GREEN_LIGHT_EXIT, IO_ENABLE);
            }
            else{
                osal_dev_io_state_change(BOARD1_OUTPUT_RED_LIGHT_EXIT, IO_ENABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_GREEN_LIGHT_EXIT, IO_DISABLE);
            }
        }

        switch (wash.state) {
        case STA_INIT:
            xp_service_set_state(STA_STOP);     //初始化完毕转暂停服务状态，等待归位后再开始运营
            break;

        case STA_IDLE:                          //切换到空闲状态
            if(true == wash.isFirstSwitch){
                wash.isFirstSwitch = false;
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_GREEN,   IO_ENABLE);
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_RED,     IO_DISABLE);
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_YELLOW,  IO_DISABLE);
                set_is_allow_next_car_wash_flag(true);
                // set_error_state(8126, false);
                clearOfflineOrdeBusyCnt = 0;
                osal_dev_io_state_change(BOARD5_OUTPUT_MACHINE_IDEL, IO_ENABLE);
                devIdelStaTimeStamp = aos_now_ms();
            }

            if(++clearOfflineOrdeBusyCnt == 100){
                clearOfflineOrdeBusyCnt = 100;
                osal_dev_io_state_change(BOARD5_OUTPUT_MACHINE_IDEL, IO_DISABLE);
            }

            if (startWashFlag) {
                startWashFlag = false;
                wash.isGetStartCmd = false;
                if(0 == xp_service_start_wash_ready(wash.state)){
                    xp_service_set_state(STA_RUN);
                }
                else{
                    xp_service_set_state(STA_EXCEPTION);
                }
            }

            //待机状态检测防追尾光电若灭10S以上，则认为异常，转入异常状态
            if(is_signal_filter_trigger(SIGNAL_REAR_END_PROTECT)){
                if(signalTriggerCnt++ > 100){
                    xp_service_set_state(STA_EXCEPTION);
                    set_error_state(8115, true);
                }
            }
            else{
                signalTriggerCnt = 0;
            }
            break;

        case STA_RUN:
            if(true == wash.isFirstSwitch){
                wash.isFirstSwitch = false;
            }

            if (startWashFlag) {
                startWashFlag = false;
                wash.isGetStartCmd = false;
                if(0 == xp_service_start_wash_ready(wash.state)){
                    // xp_service_set_state(STA_RUN);
                }
                else{
                    xp_service_set_state(STA_EXCEPTION);
                }
            }

            if(++clearOfflineOrdeBusyCnt == 100){
                clearOfflineOrdeBusyCnt = 100;
                osal_dev_io_state_change(BOARD5_OUTPUT_MACHINE_IDEL, IO_DISABLE);
            }

            //仅在输送带传输过程中检测超高
            err_need_flag_handle()->isDetectSuperHighEnable = is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID) ? false : appModule.localCmd.func.detectSuperHigh;
            //转待机时停留2S，等待输送带控制线程把输送带2#停止
            if(0 == appModule.localSts.order1.orderNumber && 0 == appModule.localSts.order2.orderNumber
            && get_diff_ms(timeStamp) > 2000){
                xp_service_set_state(STA_IDLE);
            }

            uint8_t completeCarId = 0;
            ret = step_dev_wash(&completeCarId);
            if (RET_COMPLETE == ret) {
                timeStamp = aos_now_ms();
                carInfo.isCarFinishWash = true;
                if(1 == completeCarId){
                    appModule.localSts.washInfo.completeOrderNum = appModule.localSts.order1.orderNumber;
                    appModule.localSts.order1.orderNumber = 0;
                    if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_1", 0);
                }
                else if(2 == completeCarId){
                    appModule.localSts.washInfo.completeOrderNum = appModule.localSts.order2.orderNumber;
                    appModule.localSts.order2.orderNumber = 0;
                    if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_2", 0);
                }
#ifdef RECORD_WASH_CNT
                kvService.order.completeCnt++;
                kvService.order.isNewCompleteOrderSave = true;
#endif
            }
            else if (ret < 0) {
                if(ERR_TIMEOUT == ret) {
                    set_error_state(8111, true);
                }
                xp_service_set_state(STA_SUSPEND);
                LOG_UPLOAD("Wash execute error, ret %d", ret);
            }
            else {
                //继续循环执行
            }
            break;

        case STA_PAUSE:
            if(true == wash.isFirstSwitch){
                wash.isFirstSwitch = false;
                set_driver_executed_flag(false);                //暂停后，需要重置机构驱动执行标志
                wash.pauseTimeStamp = aos_now_ms();
            }
            
            if (0) {                              //暂停结束后恢复运行状态
                xp_service_set_state(STA_RUN);
            }
            wash.servicePauseT = get_diff_ms(wash.pauseTimeStamp);        //更新暂停时间
            set_step_pause_time(wash.servicePauseT);
            break;

        case STA_RECOVER:
            if(true == wash.isFirstSwitch){
                wash.isFirstSwitch = false;
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_GREEN,   IO_DISABLE);
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_RED,     IO_DISABLE);
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_YELLOW,  IO_ENABLE);
                voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_HOMING);
                set_driver_executed_flag(false);            //置非执行状态，为归位驱动做准备
            }
            ret = step_dev_back_home();
            if(NOR_CONTINUE != ret){
                appModule.localCmd.backHome = false;
                if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_home", 0);
                if(RET_COMPLETE == ret){
                    set_error_state(8101, false);
                    LOG_INFO("Back home success");
                    voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_SILENCE);
                    xp_service_set_state(STA_IDLE);

                    if(isAutoOrder){
                        //连续洗测试
                        if(++kvService.order.offlineStartNum > 9999999)   kvService.order.offlineStartNum = 0;
                        kvService.order.isNewOfflineOrderSave = true;
                        appModule.localSts.washInfo.offlineOrderNum = wash.washMode*110000000 + kvService.order.offlineStartNum;      //上传订单号（包含洗车模式信息）
                        add_new_order(appModule.localSts.washInfo.offlineOrderNum, false);
                    }
                }
                else{
                    set_error_state(8101, true);
                    xp_service_set_state(STA_EXCEPTION);
                    LOG_UPLOAD("Back home err, ret %d", ret);
                }
            }
            break;
        case STA_STOP:
        case STA_EXCEPTION:
            if(true == wash.isFirstSwitch){
                timeStamp = aos_now_ms();
                wash.isFirstSwitch = false;
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_GREEN,   IO_DISABLE);
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_RED,     IO_ENABLE);
                osal_dev_io_state_change(BOARD5_OUTPUT_SIGNAL_LAMP_YELLOW,  IO_DISABLE);
                appModule.localSts.order1.orderNumber = 0;
                appModule.localSts.order2.orderNumber = 0;
                if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_1", 0);
                if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_wash_flag_2", 0);
                set_new_car_ready_wash_falg(false);
                stop_all_dev_run();
                if(false == get_emc_power_off_sta()){
                    if(!is_signal_filter_trigger(SIGNAL_GATE_1_OPEN))    gate_change_state(CRL_SECTION_1, GATE_OPEN);    //首次切换到停止或异常状态，打开两个道闸
                    gate_change_state(CRL_SECTION_2, GATE_OPEN);
                }
                
                if(STA_RECOVER == wash.lastState){              //上次为恢复状态，恢复失败则清除远程归位点位
                    appModule.localCmd.backHome = false;
                    if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_home", 0);
                }
                if(STA_STOP == wash.state){
                    xp_ag_osal_get()->screen.display(AG_PAUSE_SERVICE);
                    if(wash.isFirstPowerOn){
                        wash.isFirstPowerOn = false;
                    }
                    else{
                        voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_SILENCE);
                    }
                }
                else if(STA_EXCEPTION == wash.state){
                    xp_ag_osal_get()->screen.display(AG_SERVICE_EXCEPTION);
                    voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_WASH_EXCEPTION);
                    //如果恢复待机后立马报警则不自动归位，避免无法消除的报警导致一直重复在自动归位
                    isAutoResetDev = (wash.lastState != STA_RECOVER && get_diff_ms(devIdelStaTimeStamp) > 5000) ? true : false;
                }
            }

            //急停状态下打开所有道闸
            if(true == get_emc_power_off_sta()){
                // 刚上电的时候处于急停状态，道闸1即使在开位也没有开位信号，道闸会动一下
                if(!is_signal_filter_trigger(SIGNAL_GATE_1_OPEN) && is_dev_move_sta_idle(GATE_1_MACH_ID))   gate_change_state(CRL_SECTION_1, GATE_OPEN);
                if(!is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) || !is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)){
                    gate_change_state(CRL_SECTION_2, GATE_OPEN);
                }
            }
            else{
                if(isAutoResetDev && STA_EXCEPTION == wash.state && get_diff_ms(timeStamp) > 3000
                && true == appModule.localCmd.func.enableStation && true == get_allow_back_home_flag()
                && !is_signal_filter_trigger(SIGNAL_FL_COLLISION) && !is_signal_filter_trigger(SIGNAL_FR_COLLISION)
                && !is_signal_filter_trigger(SIGNAL_BL_COLLISION) && !is_signal_filter_trigger(SIGNAL_BR_COLLISION)
                && !is_signal_filter_trigger(SIGNAL_ENTRANCE) && !is_signal_filter_trigger(SIGNAL_AVOID_INTRUDE) 
                && !is_signal_filter_trigger(SIGNAL_REAR_END_PROTECT) && !is_signal_filter_trigger(SIGNAL_EXIT) 
                && !is_signal_filter_trigger(SIGNAL_FINISH)){
                    xp_service_set_state(STA_RECOVER);
                    isAutoResetDev = false;
                }
            }
            
            if(true == isGetBackHomeCmd){
                isGetBackHomeCmd = false;
                xp_service_set_state(STA_RECOVER);
            }
            if(true == appModule.localCmd.customStopWash){       //如果是用户主动停止，在报警恢复后清除远程点位标志
                appModule.localCmd.customStopWash = false;
                if(xp_cmd_excuted_complete) xp_cmd_excuted_complete("cmd_custom_stop_wash", 0);
            }
            break;

        case STA_SUSPEND:
            //异常挂起等待恢复，恢复不了则转入异常停止状态
            if(true == wash.isFirstSwitch){
                wash.isFirstSwitch = false;
                stop_all_dev_run();
                xp_ag_osal_get()->screen.display(AG_SERVICE_EXCEPTION);
                voice_play_set(AG_VOICE_POS_ENTRY, AG_VOICE_WASH_EXCEPTION);
                set_driver_executed_flag(false);                //置非执行状态，为归位驱动做准备
            }

            //急停上电稳定后再动作
            if(false == get_emc_power_off_sta()){
                ret = step_dev_warning_home();
                if(NOR_CONTINUE != ret){
                    if(RET_COMPLETE == ret){
                        LOG_INFO("Warning home success");
                    }
                    else{
                        LOG_UPLOAD("Warning home error, ret %d", ret);
                    }
                    xp_service_set_state(STA_EXCEPTION);
                }
            }
            break;

        case STA_COMPELTE:
            if(true == wash.isFirstSwitch){
                wash.isFirstSwitch = false;
                // xp_ag_osal_get()->screen.display(AG_COMPLETED);
                voice_play_set(AG_VOICE_POS_EXIT, AG_VOICE_COMPLETED);
                timeStamp = aos_now_ms();
            }
#ifdef CONTINUE_TEST
            if(get_diff_ms(timeStamp) > 5000){
                xp_service_set_state(STA_IDLE);
                //测试
                wash.isGetStartCmd = true;
#else
            if(get_diff_ms(timeStamp) > 10000){
                xp_service_set_state(STA_IDLE);
#endif
            }
            break;

        default:
            wash.state = STA_INIT;
            break;
        }
        aos_msleep(100);
    }
}

/*                                                         =======================                                                         */
/* ========================================================        其它接口        ======================================================== */
/*                                                         =======================                                                         */

void warnning_deal_thread(void *arg)
{
    while (wash.state != STA_RUN)
    {
        aos_msleep(1000);
    }
    //洗车运行状态结束后判断是否有报警需要停机，有则停机，否则继续接待后续车辆
    if(true == get_attention_err_status()){
        if(STA_IDLE == wash.state || STA_COMPELTE == wash.state) xp_service_set_state(STA_SUSPEND);
    }
    wash.isWarningErr = false;
    aos_task_delete("warnning_deal");
}

/**
 * @brief       异常处理回调
 */
static void xp_err_deal_callback(uint16_t code, Type_ErrDealMode_Enum mode)
{
    LOG_UPLOAD("Device err %d, deal mode %d", code, mode);

    //遇到致命错误时立即处理报警
    if(E_ERROR == mode || E_WARNING == mode){
        if(wash.state != STA_SUSPEND){          //异常挂起状态有新报警过来不做处理
            if(wash.state != STA_STOP){
                xp_service_set_state(STA_RUN == wash.state ? STA_SUSPEND : STA_EXCEPTION);
            }
            else{
                xp_service_set_state(STA_STOP);
            }
        }
    }
    //遇到警告类错误，等待洗车结束后报警（工作区车辆继续送出，预备区不再允许启动）
    else if(E_WARNING == mode){
        // if(STA_RUN == wash.state && ){
        //     if(!wash.isWarningErr){
        //         wash.isWarningErr = true;
        //         xp_ag_osal_get()->screen.display(AG_CAR_WAIT);
        //         aos_task_new("warnning_deal", warnning_deal_thread, NULL, 1024);
        //     }
        //     else{
        //         //已经有警告错误不再处理
        //     }
        // }
        // else{
        //     if(wash.state != STA_SUSPEND){      //异常挂起状态有新报警过来不做处理
        //         (wash.state != STA_STOP) ? xp_service_set_state(STA_EXCEPTION) : xp_service_set_state(STA_STOP);
        //     }
        // }
    }
    else{
        //其它报警这里不做处理
    }
}

/**
 * @brief       远程命令执行完毕回调
 * @param[in]	callback            
 */
void xp_cmd_excuted_complete_callback_regist(int (*callback)(char *arg, int value))
{
    xp_cmd_excuted_complete = callback;
}

/**
 * @brief       获取状态模型对象
 * @return      Type_ModelSts_Def*  
 */
Type_ModelSts_Def *get_modelSts_Obj(void)
{
    return &appModule.localSts;
}

/**
 * @brief       获取命令模型对象
 * @return      Type_ModelCmd_Def*  
 */
Type_ModelCmd_Def *get_modelCmd_Obj(void)
{
    return &appModule.localCmd;
}

/**
 * @brief       接收远程控制指令信号量
 * @param[in]	cmd                 
 */
void set_remote_cmd_queue(Type_PropertyNode_Def *cmd)
{
    aos_queue_send(&appModule.cmdQueue, cmd, sizeof(Type_PropertyNode_Def));
}

void set_app_version(char const *version)
{
    strcpy(appModule.appVersion, version);
}

/*                                                         =======================                                                         */
/* ========================================================       debug接口       ======================================================== */
/*                                                         =======================                                                         */

int xp_service_debug(char* type, char* fun, char* param)
{
    int ret = 0;

    //本调试接口仅支持 "ag" 起始的命令
    if (NULL == strstr(type, "ag_")) {
        return 0;
    }
    else {
        type += strlen("ag_");
    }

    if (strcasecmp(type, "service") == 0) {
        if (strcasecmp(fun, "start") == 0) {
            add_new_order(TEST_LOCAL_ORDER_NUM, false);
        }
        else if (strcasecmp(fun, "back_home") == 0) {
            if(STA_STOP == wash.state || STA_EXCEPTION == wash.state){
                isGetBackHomeCmd = true;
            }
        }
        else if (strcasecmp(fun, "pause") == 0) {
            xp_service_set_state(STA_PAUSE);
        }
        else if (strcasecmp(fun, "continue") == 0) {
            xp_service_set_state(STA_RUN);
        }
        else if (strcasecmp(fun, "stop") == 0) {
            xp_service_set_state(STA_STOP);
        }
        else if (strcasecmp(fun, "clear_wash_record") == 0){
			kvService.order.completeCnt   = 0;
			kvService.order.offlineStartNum = 0;
			memset(&kvService.order.today, 0, sizeof(kvService.order.today));
            memset(kvService.order.month, 0, sizeof(kvService.order.month));
            memset(kvService.order.daily, 0, sizeof(kvService.order.daily));
            memset(&kvService.order.total, 0, sizeof(kvService.order.total));

            kvService.order.isNewCompleteOrderSave = true;
            kvService.order.isNewOfflineOrderSave = true;
            kvService.order.isNewOrderSave = true;
		}
        else if (strcasecmp(fun, "free") == 0) {
            isAutoOrder = atoi(param);
        }
    }
    //清除所有kv值恢复为默认值，除了日期
    else if (strcasecmp(type, "clear_all_kv") == 0){
        set_kv_default_value();
        for (uint8_t i = 0; i < sizeof(KvDataMap_Table) / sizeof(KvDataMap_Table[0]); i++)
        {
            if (KvDataMap_Table[i].cmd)
            {
                xp_save_kv_params(KvDataMap_Table[i].key, KvDataMap_Table[i].pData, KvDataMap_Table[i].len);
            }
        }
        kvService.order.isNewCompleteOrderSave = true;
        kvService.order.isNewOrderSave = true;
    }
    else if (strcasecmp(type, "order_daily") == 0){
        uint8_t month = atoi(fun);
        uint8_t date = atoi(param);
        if(month > 0 && month < 13 && date > 0 && date < 32){
            // LOG_UPLOAD("Date %02d.%02d washQuick order %d", month, date, kvService.order.daily[month - 1][date - 1].washQuick);
            // LOG_UPLOAD("Date %02d.%02d washNormal order %d",month, date, kvService.order.daily[month - 1][date - 1].washNormal);
            // LOG_UPLOAD("Date %02d.%02d washFine order %d",  month, date, kvService.order.daily[month - 1][date - 1].washFine);
            LOG_UPLOAD("Date %02d.%02d totals order %d",    month, date, kvService.order.daily[month - 1][date - 1].totals);
        }
        else{
            LOG_UPLOAD("Month %d, date %d illegal", month, date);
        }
    }
    else if (strcasecmp(type, "order_month") == 0){
        uint8_t month = atoi(fun);
        if(month > 0 && month < 13){
            // LOG_UPLOAD("Month %02d washQuick order %d", month, kvService.order.month[month - 1].washQuick);
            // LOG_UPLOAD("Month %02d washNormal order %d",month, kvService.order.month[month - 1].washNormal);
            // LOG_UPLOAD("Month %02d washFine order %d",  month, kvService.order.month[month - 1].washFine);
            LOG_UPLOAD("Month %02d totals order %d",    month, kvService.order.month[month - 1].totals);
        }
        else{
            LOG_UPLOAD("Month %d illegal", month);
        }
    }
    else if (strcasecmp(type, "order_today") == 0){
        // LOG_UPLOAD("Today washQuick order %d",  kvService.order.today.washQuick);
        // LOG_UPLOAD("Today washNormal order %d", kvService.order.today.washNormal);
        // LOG_UPLOAD("Today washFine order %d",   kvService.order.today.washFine);
        LOG_UPLOAD("Today totals order %d",     kvService.order.today.totals);
    }
    else if (strcasecmp(type, "time") == 0){
        LOG_UPLOAD("RTC time: %s. systime: %s.", xp_time_get_string(), xp_msTostring(aos_now_ms()));
    }
    else {
        return 0;
    }
    return 1;
}

