/**
 * @file 	 ag_service.h
 * @brief       
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 * 
 * @copyright Copyright (c) 2022  YGL
 * 
 */

#include "stdbool.h"
#include "../../../../include/aos/list.h"
#include "../../../../1km/bsp/base_type.h"
#include "ag_osal.h"

#ifndef __AG_SERVICE_H
#define __AG_SERVICE_H

#define MQTT_PORPERTE_IDENTIFIER_MAX_LEN    50

//洗车设备状态机
typedef enum {
    STA_INIT = 0,                           //初始状态，机器初始化
    STA_IDLE,                               //空闲状态，正常待机中
    STA_RUN,                                //运行状态，洗车过程中
    STA_SUSPEND,                            //挂起状态，机器有异常，挂起等待恢复
    STA_PAUSE,                              //暂停状态，非异常，暂停等待状态
    STA_EXCEPTION,                          //异常状态，异常停止中
    STA_RECOVER,                            //恢复状态，回到待机位准备洗车
    STA_COMPELTE,                           //完成状态，洗车服务正常结束
    STA_STOP,                               //停止状态，机器暂停运营中
} Type_ServiceState_Enum;

/* 洗车模式枚举 */
typedef enum{
    NULL_WASH = 0,
	NORMAL_WASH,	    //普通洗
	QUICK_WASH,			//极速洗
	FINE_WASH,			//打蜡精洗
} Type_WashMode_Enum;

//状态点位——订单
typedef struct{
    int                 orderNumber;
    int                 workState;
} Type_ModelSts_Order_Def;

//状态点位——洗车判断
typedef struct{
    int                 offlineOrderNum;
    int                 completeOrderNum;
    bool                carAhead;
    bool                stopOk;
    int                 parkTips;
    bool                gateOpenDone1;
    bool                gateCloseDone1;
    bool                gateOpenDone2;
    bool                gateCloseDone2;
    bool                carTooLong;
    bool                carTooHigh;
} Type_ModelSts_WashInfo_Def;

//状态点位——洗车机状态
typedef struct{
    bool                stopping;
    bool                standby;
    bool                normalHoming;
    bool                customStop;
    bool                warnHoming;
    bool                warnning;
    char                warnCode[40];
} Type_ModelSts_DevSta_Def;

//状态点位——监控数据（检测数据）
typedef struct{
    bool                carEntry;
    // bool                vfdReadingCurrent;
    // bool                devConnecting;
    // bool                vfdReadyCurrent;
    // bool                brushInit;
    int                 sewageHighTime;
    int                 readyAreaPulse;
    int                 workAreaPulse;
    int                 completeAreaPulse;
    int                 frontLeftPutterPos;
    int                 frontRightPutterPos;
    int                 backLeftPutterPos;
    int                 backRightPutterPos;
    int                 topBrushCurrent;
    int                 frontLeftBrushCurrent;
    int                 frontRightBrushCurrent;
    int                 backLeftBrushCurrent;
    int                 backRightBrushCurrent;
    char                version[10];
    char                deviceModel[15];
    int                 startCnt;
    int                 completeCnt;
    int                 failedCnt;
    int                 toadyTotalCnt;
    // int                 errCode;
} Type_ModelSts_monitor_Def;

//状态点位——传感器检测
typedef struct{
    // bool                bLeftSkirtOpen;
    // bool                bRightSkirtOpen;
    bool                emergency;
    bool                reset;
    bool                phaseSequence;
    bool                waterPressLow;
    bool                fLeftCollision;
    bool                fRightCollision;
    bool                bLeftCollision;
    bool                bRightCollision;
    bool                avoidIntrude;
    bool                leftSkew;
    bool                rightSkew;
    bool                stop;
    bool                entrance;
    bool                allIn;
    bool                high;
    bool                rearEndProtect;
    bool                exit;
    bool                complete;
    bool                conveyorPulse1;
    bool                conveyorPulse2;
    bool                conveyorPulse3;
    bool                leftSkirtOpen;
    bool                rightSkirtOpen;
    bool                lifterUp;
    bool                lifterDown;
    bool                lifterLeftLoose;
    bool                lifterRightLoose;
    bool                fLeftBrushCrooked;
    bool                fLeftBrushOpen;
    bool                fLeftBrushDown;
    bool                fRightBrushCrooked;
    bool                fRightBrushOpen;
    bool                fRightBrushDown;
    bool                bLeftBrushCrooked;
    bool                bLeftBrushOpen;
    bool                bLeftBrushDown;
    bool                bRightBrushCrooked;
    bool                bRightBrushOpen;
    bool                bRightBrushDown;
    bool                fLeftPutterPulse;
    bool                fRightPutterPulse;
    bool                bLeftPutterPulse;
    bool                bRightPutterPulse;
    // bool                leftGate2Closed;
    // bool                leftGate2Open;
    // bool                rightGate2Closed;
    // bool                rightGate2Open;
    // bool                leftPressureSwitch;
    // bool                rightPressureSwitch;
    // bool                gate1Closed;
    // bool                gate1Open;
    bool                ground;
    bool                gate1Err;
    bool                sewageHigh;
    bool                shampooLess;
    bool                waxwaterLess;
} Type_ModelSts_sensor_Def;

//状态点位——LED灯状态
typedef struct{
    bool                goHead;
    bool                goBack;
    bool                leftSkew;
    bool                rightSkew;
    bool                stop;
    bool                tooLong;
    bool                wait;
    bool                superHigh;
    bool                expection;
    bool                reset;
    bool                pauseService;
    bool                readyStart;
    bool                moveOn;
    bool                scaneCode;
    bool                exitGreen;
    bool                exitRed;
} Type_ModelSts_LED_Def;

typedef struct{
    Type_ModelSts_Order_Def         order1;
    Type_ModelSts_Order_Def         order2;
    Type_ModelSts_WashInfo_Def      washInfo;
    Type_ModelSts_DevSta_Def        devSta;
    Type_ModelSts_monitor_Def       minitor;
    Type_ModelSts_sensor_Def        sensor;
    Type_ModelSts_LED_Def           led;
    bool                            communicateTest;
    bool                            rebootFlag;
    char                            sDI_IO[BOARD_NUMS][30];          //数组数不能大于最大标识符
    char                            sDO_IO[BOARD_NUMS][30];
    char                            bsDI_IO[BOARD_NUMS + (BOARD_NUMS - 1)*6][10];   //一个点位展示4个IO状态，主板4个点位，子板24个点位
    char                            bsDO_IO[BOARD_NUMS + (BOARD_NUMS - 1)*6][10];
    // bool                            bDI_IO[BOARD_NUMS][24];
    // bool                            bDO_IO[BOARD_NUMS][24];
} Type_ModelSts_Def;

//远程功能按钮控制指令
typedef struct{
    bool enableStation;
    bool detectLifterLoose;
    bool detectWaterPress;
    bool detectShampoo;
    bool detectWaxwater;
    bool detectSewage;
    bool enableSewagePump;
    bool enableTopSwing;
    bool enableLeftSwing;
    bool enableRightSwing;
    bool enableSkirtBrush;
    bool detectAllinSignal;
    bool detectSkirtSignal;
    bool enableDryer16;
    bool enableDryer25;
    bool enableDryer34;
    bool enableGate1;
    bool enableDryerDateMode;
    bool enableAutoReset;
    bool enableHighPressWash;
    bool detectSuperHigh;
    bool detectCleaner;
    bool detectDrier;
    bool enableManualMode;

    bool detectAllCollision;
    bool detectFLeftCollision;
    bool detectFRightCollision;
    bool detectBLeftCollision;
    bool detectBRightCollision;
} Type_ModelFunctionCmd_Def;

//远程功能按钮控制指令
typedef struct{
    int highPressEndPos;
    int skirtBrushStartPos;
    int skirtBrushEndPos;
    int shampooStartPos;
    int shampooEndPos;
    int topBrushStartPos;
    int topBrushEndPos;
    int frontBrushStartPos;
    int frontBrushTailEndPos;
    int backBrushStartPos;
    int backBrushEndPos;
    int waxwaterStartPos;
    int waxwaterEndPos;
    int dryerStartPos;
    int dryerEndPos;
    // int completeAreaHeadStopPos;
    // int frontBrushHeadStartCurrent;
    // int topCurrentAddMin;
    // int topCurrentAddMid;
    // int topCurrentAddMax;
    // int readyAreaMoveMin;
    // int dryerDayFreq;
    // int dryerNightFreq;
} Type_ModelAdjustCmd_Def;

//命令点位
typedef struct{
    int  newOrder;
    bool scanCode;
    bool washFlag1;
    bool washFlag2;
    //远程控制
    bool backHome;
    bool safeBackHome;
    bool startWash;
    bool stop;
    bool customStopWash;
    bool electricalReset;
    bool cancelOrder;
    bool devClose;
    //手动控制
    bool backSkirtBrushOut;
    bool backSkirtBrushRotate;
    bool backSkirtBrushWater;
    bool sync;
    bool gate1Open;
    bool gate1Close;
    bool gate2Open;
    bool gate2Close;
    bool gate3Open;
    bool gate3Close;
    bool conveyorMove1;
    bool conveyorMove2;
    bool conveyorMove3;
    bool conveyorMoveAll;
    bool highPressWater;
    bool shampoo;
    bool waxWater;
    bool topWater;
    bool frontSideWater;
    bool backSideWater;
    bool skirtBrushOut;
    bool skirtBrushRotate;
    int  topBrushRotate;
    int  lifterMove;
    int  fLeftBrushRotate;
    int  fLeftPutterMove;
    int  fRightBrushRotate;
    int  fRightPutterMove;
    int  bLeftBrushRotate;
    int  bLeftPutterMove;
    int  bRightBrushRotate;
    int  bRightPutterMove;
    bool dryer1;
    bool dryer25;
    bool dryer34;
    bool dryer6;
    // int  vfdReset;
    bool floodlight;
    bool ambientLight;
    bool sewagePump;
    bool highPump;
    bool winterDrainage;
    bool conveyorValve1;
    bool conveyorValve2;
    bool conveyorValve3;

    //功能按钮
    Type_ModelFunctionCmd_Def   func;
    //调整数据
    Type_ModelAdjustCmd_Def     adjust;
    //通讯测试
    bool communicateTest;
    //模式选择
    int washMode;
    //其它
    bool log;
    char debug[35];
} Type_ModelCmd_Def;

//云平台下发的控制命令
typedef enum{
    CMD_NULL = 0,
    CMD_REBOOT,
    SYNC,
    CMD_NEW_ORDER,
    CMD_SCAN_CODE,
    CMD_WASH_FALG_1,
    CMD_WASH_FALG_2,
    //远程控制
    CMD_HOME,
    CMD_SAFE_HOME,
    CMD_START_WASH,
    CMD_STOP,
    CMD_CUSTOM_STOP_WASH,
    CMD_ELECTRICAL_RESET,
    CMD_CANCEL_ORDER,
    CMD_DEV_CLOSE,
    //手动控制
    CMD_GATE_1_OPEN,
    CMD_GATE_1_CLOSE,
    CMD_GATE_2_OPEN,
    CMD_GATE_2_CLOSE,
    CMD_GATE_3_OPEN,
    CMD_GATE_3_CLOSE,
    CMD_CONVEYOR_1,
    CMD_CONVEYOR_2,
    CMD_CONVEYOR_3,
    CMD_CONVEYOR_ALL,
    CMD_HIGH_PRESS_WATER,
    CMD_WATER_SHAMPOO,
    CMD_WATER_WAXWATER,
    CMD_WATER_TOP,
    CMD_WATER_FRONT_SIDE,
    CMD_WATER_BACK_SIDE,
    CMD_SKIRT_BRUSH_MOVE,
    CMD_SKIRT_BRUSH_ROTATIN,
    CMD_TOP_BRUSH,
    CMD_LIFTER,
    CMD_FRONT_LEFT_BRUSH,
    CMD_FRONT_LEFT_MOVE,
    CMD_FRONT_RIGHT_BRUSH,
    CMD_FRONT_RIGHT_MOVE,
    CMD_BACK_LEFT_BRUSH,
    CMD_BACK_LEFT_MOVE,
    CMD_BACK_RIGHT_BRUSH,
    CMD_BACK_RIGHT_MOVE,
    CMD_DRYER_1,
    CMD_DRYER_25,
    CMD_DRYER_34,
    CMD_DRYER_6,
    CMD_FLOODLIGHT,
    CMD_AMBIENT_LIGHT,
    CMD_SEWAGE_PUMP,
    CMD_HIGH_PUMP,
    CMD_WINTER_DRAINAGE,
    CMD_CONVEYOR_1_VALVE,
    CMD_CONVEYOR_2_VALVE,
    CMD_CONVEYOR_3_VALVE,
    //功能按钮
    CMD_ENABLE_STATION,
    CMD_DETECT_LIFTER_LOOSE,
    CMD_DETECT_WATER_PRESS,
    CMD_DETECT_SHAMPOO,
    CMD_DETECT_WAXWATER,
    CMD_DETECT_SEWAGE,
    CMD_ENABLE_SEWAGE_PUMP,
    CMD_ENABLE_TOP_SWING,
    CMD_ENABLE_LEFT_SWING,
    CMD_ENABLE_RIGHT_SWING,
    CMD_ENABLE_SKIRT_BRUSH,
    CMD_DETECT_ALLIN_SIGNAL,
    CMD_DETECT_SKIRT_SIGNAL,
    CMD_ENABLE_DRYER16,
    CMD_ENABLE_DRYER25,
    CMD_ENABLE_DRYER34,
    CMD_ENABLE_GATE1,
    CMD_ENABLE_DRYER_MODE,
    CMD_ENABLE_AUTO_RESET,
    CMD_ENABLE_HIGH_PRESS_WASH,
    CMD_DETECT_SUPER_HIGH,
    CMD_DETECT_CLEANER,
    CMD_DETECT_DRIER,
    CMD_ENABLE_MANUAL_MODE,
    //防撞功能设置
    CMD_DETECT_ALL_COLLISION,
    CMD_DETECT_FLEFT_COLLISION,
    CMD_DETECT_FRIGHT_COLLISION,
    CMD_DETECT_BLEFT_COLLISION,
    CMD_DETECT_BRIGHT_COLLISION,
    //调整数据
    CMD_HIGH_PRESS_END_POS,
    CMD_SKIRT_BRUSH_START_POS,
    CMD_SKIRT_BRUSH_END_POS,
    CMD_SHAMPOO_START_POS,
    CMD_SHAMPOO_END_POS,
    CMD_TOP_BRUSH_START_POS,
    CMD_TOP_BRUSH_END_POS,
    CMD_FRONT_BRUSH_START_POS,
    CMD_BACK_BRUSH_START_POS,
    CMD_BACK_BRUSH_END_POS,
    CMD_FRONT_BRUSH_END_POS,
    CMD_WAXWATER_START_POS,
    CMD_WAXWATER_END_POS,
    CMD_DRYER_START_POS,
    CMD_DRYER_END_POS,
    //其它
    CMD_COMMUNICATION_TEST,
    CMD_WASH_MODE,
    CMD_LOG,
    CMD_DEBUG,
    CMD_NUM                   //命令总数
} REMOTE_CMD_ENUM;

typedef enum{
    TYPE_Int = 0,
    TYPE_Float,
    TYPE_String,
    TYPE_Bool,
} Type_data_type_Enum;

// 用于属性变化自动上报
typedef struct 
{
    slist_t                 next;                           //链表next
    char   identifier[MQTT_PORPERTE_IDENTIFIER_MAX_LEN];    //属性标识
    void                    *data;                          //数据指针
    void                    *last_data;                     //记录数据历史值,用于检测变化
    Type_data_type_Enum     data_type;                      //数据类型
    uint8_t                 sizeof_data;                    //数据类型大小
    int32_t                 diff_range;                     //变化范围(此参数指定当前属性数据在变化超过此范围后自动上报)
    REMOTE_CMD_ENUM         cmd_id;                         //属性设置回调中入队列的命令ID
} Type_PropertyNode_Def;

typedef struct {
	uint32_t washQuick;				// 快速洗单量
	uint32_t washNormal;			// 标准洗单量
	uint32_t washFine;				// 精致洗单量
	uint32_t totals;			    // 总单量
} Type_OrdersNum_Def;

typedef struct {
    bool                isNewOrder;
    bool                isNewOrderSave;
    bool                isNewCompleteOrderSave;
    bool                isNewOfflineOrderSave;
    bool                isNewDateSave;
    uint32_t            completeCnt;
    uint32_t            offlineStartNum;
    Type_OrdersNum_Def  today;
    Type_OrdersNum_Def  month[12];
    Type_OrdersNum_Def  daily[12][31];
    Type_OrdersNum_Def  total;
} Type_OrdersInfo_Def;

extern void xp_cmd_excuted_complete_callback_regist(int (*callback)(char *arg, int value));
extern Type_ModelSts_Def *get_modelSts_Obj(void);
extern Type_ModelCmd_Def *get_modelCmd_Obj(void);
extern void set_remote_cmd_queue(Type_PropertyNode_Def *cmd);
extern void set_app_version(char const *version);

#endif
