/**
 * @file 	 ag_module.c
 * @brief       
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 * 
 * @copyright Copyright (c) 2022  YGL
 * 
 */
#include "../../../1km/bsp/bsp.h"
#include "aos/kv.h"
#include "ag_module.h"
#include "ag_err_state.h"

#define BRUSH_FOLLOW_THREAD_FREQ                (30)            //毛刷跟随线程周期
#define BRUSH_CURRENT_UPDATE_THREAD_FREQ        (10)            //毛刷电流更新线程周期

#define TOP_BRUSH_WARNING_CUR                   (180)           //顶刷报警电流
#define SIDE_BRUSH_WARNING_CUR                  (180)           //侧刷报警电流
#define DEV_FORCE_MOVE_TIME                     (1000)          //设备单次强制移动时间（ms）
#define SUPPORT_WASH_NUM_MAX                    (3)             //支持的最大洗车数量
#define ENCODER_FLIP_VALUE                      (0x00FFFFFF)    //子板码盘计数最大值（超过该值翻转清零重新计数，与子板最大计数值对应）
#define CAR_POS_RECORD_INFO_ACCURACY            (5)             //车身位置信息记录精度（输送带位置对应的车身高度）
#define CAR_POS_RECORD_INFO_MAX_NUM             ((CAR_MAX_LENGTH + SIGNAL_ENTRANCE_TO_TOP_BRUSH_OFFSET)/CAR_POS_RECORD_INFO_ACCURACY)    //车身位置信息的记录缓存个数
#define TOP_BRUSH_RECORD_CUR_AREA               (5)             //顶刷记录电流的升降脉冲区域（兼容顶刷在一定高度会打到框架的结构）
#define BRUSH_CALI_READ_CNT                     (5)             //毛刷电流校准采样次数
#define DECISION_BACKWARD_CMD_CNT               (700/BRUSH_FOLLOW_THREAD_FREQ)  //决定执行后退指令的滤波次数
#define DECISION_FORWARD_CMD_CNT                (800/BRUSH_FOLLOW_THREAD_FREQ)  //决定执行前进指令的滤波次数
#define DECISION_STILL_CMD_CNT                  (1)             //决定执行保持指令的滤波次数
#define SIDE_BRUSH_OPEN_POS                     (0)             //侧刷的开位位置，侧刷跟随过程中开位限制在零位移出的一定距离，避免毛刷打到壳体影响电流值
#define PRESS_PROTECT_COMFIRM_CNT               (2)             //压力达到触压保护值的确认次数，用于限制移动速度
#define PRESS_PROTECT_REEASE_CNT                (5)             //触压保护释放确认次数
#define VELOCITY_LIMIT_LEVEL_STOP               (CMD_STILL)
#define VELOCITY_NO_LIMIT                       (CMD_FORWARD)   //无速度限制，按最快速度
#define PICKUP_SLOPE_THRESHOLD_VALUE            (20)            //皮卡斜率判定阈值（高度）
#define SLPOE_JUDGE_FREQ_VALUE                  (35)            //斜率判定周期值（行走距离）

#define SIGNAL_ENTRANCE_TO_TOP_BRUSH_OFFSET     (150)           //入口光电到顶刷的偏移值
#define HEAD_OFFSET_FRONT_BRUSH_OVER_HEAD       (440)           //前侧刷越过车头的偏移值
#define PUTTER_GO_MIDDLE_OFFSET                 (90)            //侧刷一起移动到中间的偏移值
#define PUTTER_GO_SELF_SIDE_OFFSET              (30)            //侧刷一起移动到左侧的偏移值
#define PUTTER_GO_ANOTHER_SIDE_OFFSET           (150)           //侧刷一起移动到右侧的偏移值
#define PUTTER_FOLLOW_MAX_OFFSET                (60)            //侧刷跟随时最大的进给偏移值
#define CAT_TOP_TO_NO_BW_FOLLOW_OFFSET          (25)            //判定顶刷跟随不能向上的偏移值（车顶往下偏移该距离认为已经越过车顶，顶刷只允许向下）
#define CAR_WINDOWS_TO_TOP_OFFSET               (25)            //车窗区域距离顶部的偏移值

//车身信息默认值
#define CAR_DIFF_HUB_MIN_LENGTH                 (100)           //两个轮毂间的最短距离
#define CAR_MAX_LENGTH                          (1000)          //最长车长脉冲数
#define CAR_MIN_LENGTH                          (400)           //最短车长脉冲数
#define INIT_CAR_TOP_POS                        (140)           //车顶位置升降的初始高度

//开关控制参数
#define SWITCH_DELAY_TIME_MS                    (300)           //开关切换的延时等待时间
// #define CRL_COM_SW_NUM                          (sizeof(Com_Switch_Table) / sizeof(Com_Switch_Table[0]))    //参与控制的公有开关个数
#define CRL_PRI_SW_NUM                          (sizeof(Pri_Switch_Table) / sizeof(Pri_Switch_Table[0]))    //参与控制的私有开关个数

//控制方向指令枚举
typedef enum {
	CMD_BACKWARD_S_2 = -3,				//反向速度2
	CMD_BACKWARD_S_1 = -2,				//反向速度1
	CMD_BACKWARD 	 = -1,				//后退或反转
	CMD_STILL 		 = 0,				//停止移动或旋转
	CMD_FORWARD 	 = 1,				//进给或正转
	CMD_FORWARD_S_1  = 2,				//正向速度1
	CMD_FORWARD_S_2  = 3,				//正向速度2
} Type_DriverCmd_Enum;

//公有开关枚举（有正反转的电机设备）
typedef enum {
	// COM_SWITCH_VFD_LEFT = 0,
	// COM_SWITCH_VFD_RIGHT,
    COM_SWITCH_NUMBER,                  //公有开关数量
} Type_ComSwitchType_Enum;

//公有开关切换状态枚举
typedef enum {
	// COM_SWITCH_TO_SIDE_BRUSH = 0,
	// COM_SWITCH_TO_HUB_BRUSH,
	// COM_SWITCH_TO_SWING,
    COM_SWITCH_SHAR_NUM,                //公有开关共享的设备数量
    COM_SWITCH_TO_EMPTY,
} Type_ComSwitchSta_Enum;

//私有开关驱动设备类型枚举（有正反转的电机设备）
typedef enum {
    PRI_SWITCH_CONVEYOR_1 = 0,
    PRI_SWITCH_CONVEYOR_2,
    PRI_SWITCH_CONVEYOR_3,
    PRI_SWITCH_FRONT_LEFT_BRUSH,
    PRI_SWITCH_FRONT_RIGHT_BRUSH,
    PRI_SWITCH_BACK_LEFT_BRUSH,
    PRI_SWITCH_BACK_RIGHT_BRUSH,
    PRI_SWITCH_FRONT_LEFT_PUTTER,
    PRI_SWITCH_FRONT_RIGHT_PUTTER,
    PRI_SWITCH_BACK_LEFT_PUTTER,
    PRI_SWITCH_BACK_RIGHT_PUTTER,
    PRI_SWITCH_LIFTER,
    PRI_SWITCH_TOP_BRUSH,
    PRI_SWITCH_GATE_1,
    PRI_SWITCH_GATE_3,
    PRI_SWITCH_NUM,
} Type_PriSwitchIndex_Enum;

//执行动作类型枚举（只有毛刷旋转为 ACTION_ROTATION 类型）
typedef enum {
	ACTION_MOVE = 0,
    ACTION_FORCE_MOVE,
    ACTION_MOVE_POSE,
    ACTION_MOVE_TIME,
	ACTION_ROTATION,
} Type_ExeActionType_Enum;

//侧刷位置枚举
typedef enum{
	SIDE_BRUSH_POS_OPEN = 0,		//侧刷在开位
    SIDE_BRUSH_POS_MIDDLE,			//侧刷在中间位置
    SIDE_BRUSH_POS_LEFT,		    //侧刷在左位置
    SIDE_BRUSH_POS_RIGHT,			//侧刷在右位置
} Type_SideBrushPos_Enum;

//机构控制指令信息定义
typedef struct {
	Type_DriverCmd_Enum         cmd;           //控制指令
    Type_DriverCmd_Enum         lastCmd;       //上一次的控制指令
	bool                        isCmd;         //是否有新指令
    int16_t                     pos;           //移动的绝对位置
    int16_t                     time;          //移动的时间
} Type_DriverCmdInfo_Def;

//开关注册信息定义
typedef struct {
    Type_OutputIo_Enum          ioIndexSw;      //开关切换状态所用的继电器对应的IO
    Type_InputIo_Enum           ioIndexSwDone;  //开关切换完成反馈IO
    uint8_t                     osalMatchId;    //对应的osal中注册索引号
    Type_ExeActionType_Enum     actionType;     //动作类型
} Type_SwRegisterInfo_Def;

//开关切换控制信息定义
typedef struct {
    uint64_t 					switchStartT;	//开关切换的起始时间

    bool 						isSwitch;		//开关是否处于切换状态
    Type_ComSwitchSta_Enum      comSwitchSta;	//公有开关当前状态
    
    Type_SwRegisterInfo_Def     *info;          //私有开关信息地址
} Type_SwCrlInfo_Def;

typedef enum {
	PROC_NULL = 0,                      
	PROC_START_SKIRT_BRUSH_ROTATION,    //开始裙边刷旋转
	PROC_START_SKIRT_BRUSH_OUT,         //开始裙边刷伸出
	PROC_START_SHAMPOO,                 //开始香波
	PROC_START_TOP_BRUSH,               //开始顶部刷
	PROC_START_FRONT_BRUSH,             //开始前侧刷
	PROC_START_BACK_BRUSH,              //开始后侧刷
	PROC_START_WAXWATER,                //开始蜡水
	PROC_START_DYRING,                  //开始吹风
    PROC_FINISH_HIGH_PUMP,              //结束高压水
    PROC_FINISH_SKIRT_BRUSH,            //结束裙边刷
    PROC_FINISH_SHAMPOO,                //结束香波
	PROC_FINISH_TOP_BRUSH,              //结束顶部刷
	PROC_FINISH_FRONT_BRUSH,            //结束前侧刷
	PROC_FINISH_BACK_BRUSH,             //结束后侧刷
	PROC_FINISH_WAXWAT,                 //结束蜡水
	PROC_FINISH_DYRING,                 //结束吹风
} Type_ProcType_Enum;

Type_CarProcPosInfo_Def   washProcPos = {0};

typedef struct{
    uint32_t            headPos;            //车头位置
    uint32_t            headOffsetPos;      //车头在工作区移动的距离
    uint32_t            tailPos;            //车尾位置
    uint32_t            tailOffsetPos;      //车尾在工作区移动的距离
    uint32_t            topLifter;          //车顶的升降高度
    uint32_t            headWindowsOffsetHead;     //前车窗相对于车头位置
    uint32_t            tailWindowsOffsetHead;     //后车窗相对于车头位置
    uint16_t            length;             //车长
    uint32_t            headMoveValue;      //清零前车头移动的距离（每辆车进入工作区会清零，需要保存工作区内前面车辆的位置信息，这里的距离是扫码启动后移动的距离）
    uint32_t            tailMoveValue;      //清零前车尾移动的距离

    Type_ProcType_Enum  headProc;           //车头位置的洗车进程
    Type_ProcType_Enum  lastHeadProc;       //车头位置上次的洗车进程
    bool                isHeadProcChanged;  //车头洗车进程是否改变
    Type_ProcType_Enum  tailProc;           //车尾位置的洗车进程
    Type_ProcType_Enum  lastTailProc;       //车尾位置上次的洗车进程
    bool                isTailProcChanged;  //车头洗车进程是否改变

    bool                isWashCarHeadFinish;
    bool                isWashCarTailFinish;
    bool                isFrontBrushWashBody;
    bool                isFrontBrushInHeadArea;
    bool                isBackBrushInHeadArea;
    bool                isAllChangeBrushRotation;
    bool                isFrontChangeBrushRotation;
    bool                isCarMoveCompleteArea;
    bool                isBackBrushFinish;
} Type_CarWashInfo_Def;
static Type_CarWashInfo_Def carWash[SUPPORT_WASH_NUM_MAX] = {0};
static bool     isAllowNextCarInWorkArea = false;   //是否允许下一辆车进入清洗
static bool     isNewCarReadyWash = false;          //是否有下一辆车准备进入工作区清洗（扫码启动）
static bool     isNewCarWash = false;               //是否有新一辆车进来洗
static uint8_t  entryCarIndex = 0;                  //进入工作区的车辆分配编号
static uint8_t  headWashCarId = 0;                  //前面那辆车的编号
static uint8_t  washCarNum = 0;                     //正在清洗的车辆数量

//细分步骤执行状态定义
typedef struct {
    bool        isModuleDriverExecuted;
    bool        isComplete;
    uint8_t     subStep;
    uint64_t    retryTimeStamp;
    uint64_t    timeStamp;
    uint64_t    pauseTime;
} Type_ModuleExeSta_Def;
static Type_ModuleExeSta_Def stepSta = {0};

typedef struct {
    bool        isCmdMoveGapTime;               //是否是驱动指令下发的缺口时间（从设置控制指令到实际操作IO动作的时间）
    uint64_t    cmdStartTimeStamp;              //指令起始时间
} Type_cmdExeSta_Def;
static Type_cmdExeSta_Def cmdExeSta[DRIVER_NUM] = {0};

static int32_t recordAreaPos[CAR_POS_RECORD_INFO_MAX_NUM] = {0};        //用于记录输送带在各个位置时的升降高度值
static int32_t recordLifterPos[CAR_POS_RECORD_INFO_MAX_NUM] = {0};

//毛刷各状态值
static Type_BrushInfo_Def brush[BRUSH_NUM] = {0};
static int16_t  topBrushPosCurrent[TOP_BRUSH_RECORD_CUR_AREA];

// static Type_ComSwitchSta_Enum   comSwSta[COM_SWITCH_NUMBER] = {0};  //公有开关的切换状态
// static Type_DriverCmdInfo_Def   comSwCmd[COM_SWITCH_NUMBER] = {0};  //公有开关的驱动指令
static Type_DriverCmdInfo_Def   priSwCmd[PRI_SWITCH_NUM]    = {0};  //私有开关驱动指令

static bool isDriverExecuted = false;                   //驱动执行标志
static bool isSideBrushCantMoveToPose = false;          //侧刷无法到达指定位置标志
static bool isClearConveyorEncFinish = false;           //输送带脉冲值清零结束标志

//私有开关信息
/* osalMatchId ： 驱动机构的索引号 */
/* actionType  ： 动作类型 */
static Type_SwRegisterInfo_Def Pri_Switch_Table[PRI_SWITCH_NUM] = {
    [0] = {
        .osalMatchId = CONVEYOR_1_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [1] = {
        .osalMatchId = CONVEYOR_2_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [2] = {
        .osalMatchId = CONVEYOR_3_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [3] = {
        .osalMatchId = FRONT_LEFT_BRUSH_MATCH_ID,
        .actionType = ACTION_ROTATION,
    },
    [4] = {
        .osalMatchId = FRONT_RIGHT_BRUSH_MATCH_ID,
        .actionType = ACTION_ROTATION,
    },
    [5] = {
        .osalMatchId = BACK_LEFT_BRUSH_MATCH_ID,
        .actionType = ACTION_ROTATION,
    },
    [6] = {
        .osalMatchId = BACK_RIGHT_BRUSH_MATCH_ID,
        .actionType = ACTION_ROTATION,
    },
    [7] = {
        .osalMatchId = FRONT_LEFT_MOVE_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [8] = {
        .osalMatchId = FRONT_RIGHT_MOVE_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [9] = {
        .osalMatchId = BACK_LEFT_MOVE_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [10] = {
        .osalMatchId = BACK_RIGHT_MOVE_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [11] = {
        .osalMatchId = LIFTER_MATCH_ID,
        .actionType = ACTION_MOVE,
    },
    [12] = {
        .osalMatchId = TOP_BRUSH_MATCH_ID,
        .actionType = ACTION_ROTATION,
    },
    [13] = {
        .osalMatchId = GATE_1_MACH_ID,
        .actionType = ACTION_MOVE,
    },
    [14] = {
        .osalMatchId = GATE_3_MACH_ID,
        .actionType = ACTION_MOVE,
    },
};

//公有开关信息
// static Type_SwRegisterInfo_Def Com_Switch_Table[COM_SWITCH_NUMBER][COM_SWITCH_SHAR_NUM] = {
    
// };

int xp_module_debug(char *type, char *fun, char *param);
void switch_driver_control_thread(void* arg);
void brush_current_update_thread(void *arg);
void dev_zero_check_thread(void *arg);
void conveyor_run_crl_thread(void *arg);
void side_brush_follow_thread(void *arg);
void voice_play_thread(void *arg);

/*                                                         =======================                                                         */
/* ========================================================     打印字符查询表     ======================================================== */
/*                                                         =======================                                                         */

//毛刷字符查询表
static const char BrushStrList[][20] = { "BRUSH_TOP", "BRUSH_FRONT_L", "BRUSH_FRONT_R", "BRUSH_BACK_L", "BRUSH_BACK_R"};
static const char* xp_get_brush_str(Type_BrushType_Enum type)
{
    return BrushStrList[type];
}

/*                                                         =======================                                                         */
/* ========================================================      .c初始化接口      ======================================================== */
/*                                                         =======================                                                         */

int xp_service_module_init(void)
{
    aos_task_new("actuator_action_ctr",	switch_driver_control_thread,NULL, 2048);
    aos_task_new("brush_state_update",  brush_current_update_thread, NULL, 1024);
    aos_task_new("dev_zero_check",	    dev_zero_check_thread,       NULL, 1024);
    aos_task_new("conveyor_run_crl",	conveyor_run_crl_thread,     NULL, 1024);
    aos_task_new("side_brush_follow",	side_brush_follow_thread,    NULL, 2048);
    aos_task_new("voice_play",	        voice_play_thread,           NULL, 8192);

    //这里需要初始化报警触压值，不然直接转刷子会认为触压值过大
    for (uint8_t i = 0; i < BRUSH_NUM; i++)
    {
        memset(&brush[i], 0, sizeof(Type_BrushInfo_Def));
        brush[i].pressWarning = (BRUSH_TOP == i) ? TOP_BRUSH_WARNING_CUR : SIDE_BRUSH_WARNING_CUR;
        brush[i].init = 1;
    }

    // for (uint8_t i = 0; i < COM_SWITCH_NUMBER; i++)
    // {
    //     comSwSta[i] = COM_SWITCH_TO_EMPTY;
    // }
    return 0;
}

/*                                                         =======================                                                         */
/* ========================================================        语音播报        ======================================================== */
/*                                                         =======================                                                         */

static aos_sem_t voice_sem;
static aos_mutex_t voice_mutex;
static Type_AgVoice_Enum voiceMode = AG_VOICE_SILENCE;

static uint64_t voiceStartTimeStamp = 0;
uint64_t get_voice_start_time_stamp(void)
{
    return voiceStartTimeStamp;
}

/**
 * @brief       设置语音播报音频
 * @param[in]	mode                
 */
void voice_play_set(Type_AgVoicePos_Enum pos, Type_AgVoice_Enum mode)
{
    if(AG_VOICE_POS_EXIT == pos){
        voiceMode = mode;
        aos_sem_signal(&voice_sem);
    }
    else{
        xp_ag_osal_get()->voice.play(pos, mode);
    }
}

void voice_play_thread(void *arg)
{
    aos_sem_new(&voice_sem, 0);
    aos_mutex_new(&voice_mutex);

    while (1)
    {
        aos_sem_wait(&voice_sem, AOS_WAIT_FOREVER);
        if(0 == aos_mutex_lock(&voice_mutex, 1000)){
            voiceStartTimeStamp = aos_now_ms();
            xp_ag_osal_get()->voice.play(AG_VOICE_POS_EXIT, voiceMode);
        }
        aos_mutex_unlock(&voice_mutex);
    }
}

int module_lock_voice_mutex(uint16_t overTime)
{
    return aos_mutex_lock(&voice_mutex, overTime);
}

int module_unlock_voice_mutex(void)
{
    return aos_mutex_unlock(&voice_mutex);
}

/*                                                         =======================                                                         */
/* ========================================================    各机构首次回零监测   ======================================================== */
/*                                                         =======================                                                         */

//零点机构定义
typedef struct {
    bool                        lifter;
    bool                        frontLeftBrushPutter;
    bool                        frontRightBrushPutter;
    bool                        backLeftBrushPutter;
    bool                        backRightBrushPutter;
    bool                        entryReadyAreaGate;
    bool                        entryWorkAreaGate;
} Type_ZeroCheckType_Def;
Type_ZeroCheckType_Def zeroCheck = {0}; //零点校准

void dev_zero_check_thread(void *arg)
{
    Type_ZeroCheckType_Def  checkValue;

    memset(&checkValue, 1, sizeof(Type_ZeroCheckType_Def));

    while (1)
    {
        if(is_signal_filter_trigger(SIGNAL_LIFTER_UP))      zeroCheck.lifter = true;
        if(is_signal_filter_trigger(SIGNAL_FL_MOVE_ZERO))   zeroCheck.frontLeftBrushPutter = true;
        if(is_signal_filter_trigger(SIGNAL_FR_MOVE_ZERO))   zeroCheck.frontRightBrushPutter = true;
        if(is_signal_filter_trigger(SIGNAL_BL_MOVE_ZERO))   zeroCheck.backLeftBrushPutter = true;
        if(is_signal_filter_trigger(SIGNAL_BR_MOVE_ZERO))   zeroCheck.backRightBrushPutter = true;

        if(0 == memcmp(&zeroCheck, &checkValue, sizeof(Type_ZeroCheckType_Def))){
            LOG_INFO("All zero checked, delete thread");
            aos_task_exit(0);
        }
        aos_msleep(100);
    }
}

/*                                                         =======================                                                         */
/* ========================================================      输送带运动控制    ======================================================== */
/*                                                         =======================                                                         */

static int conveyor_move(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd);

void conveyor_run_crl_thread(void *arg)
{
    uint32_t recoderEncValue = 0;
    uint32_t finishAreaConveyorEnc;
    uint64_t exitTriggerTimeStamp = 0;
    uint64_t exitNotTriggerTimeStamp = 0;
    bool isExitSignalTrigger = false;
    uint8_t encClearCnt = 0;

    while (1)
    {
        if(false == err_need_flag_handle()->isDevRunSta){
            aos_msleep(500);
            continue;
        }
        //输送带1#控制
        if(isNewCarReadyWash){                                          //下一辆车正在进入工作区时，若前面没有车，输送带1#，2#一起动，否则，输送带1#跟随2#启停
            /* 洗车的时候可能会超时清不成功，这里后面尝试在不洗车的时候清 */
            if(!isClearConveyorEncFinish){
                uint32_t workAreaConveyorEnc = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                //多次清除无效后就不清了，下次再清，洗车流程里，这里会一直等待 isClearConveyorEncFinish 为true，所以这里等待时间不宜过长
                if(workAreaConveyorEnc < 50 || encClearCnt++ > 20){     //扫码启动后，若码盘值较大，则清零，避免溢出（清除时，输送待可能在动，所以这里给到一定余量）
                    isClearConveyorEncFinish = true;                    //清零结束后再动作输送带
                    char *tempBuf = aos_malloc(50);
                    char *valueBuf = aos_malloc(50 * SUPPORT_WASH_NUM_MAX);
                    memset(tempBuf, 0, 50);
                    memset(valueBuf, 0, 50 * SUPPORT_WASH_NUM_MAX);
                    for (uint8_t i = 0; i < SUPPORT_WASH_NUM_MAX; i++)  //前方工作区域的车辆位置信息增加偏移值
                    {
                        if(0 == carWash[i].tailPos) continue;           //没有车尾信息，认为车辆不在工作区内，不记录偏移值
                        carWash[i].headMoveValue += recoderEncValue;
                        carWash[i].tailMoveValue += recoderEncValue;
                        sprintf(tempBuf, "Id %d move value: head %d, tail %d; ", i, carWash[i].headMoveValue, carWash[i].tailMoveValue);
                        strcat(valueBuf, tempBuf);
                    }
                    if(*valueBuf)    LOG_UPLOAD("%s", valueBuf);
                    aos_free(tempBuf);
                    aos_free(valueBuf);
                }
                else{
                    recoderEncValue = workAreaConveyorEnc;
                    clear_dev_encoder(CONVEYOR_2_MATCH_ID);
                    clear_dev_encoder(CONVEYOR_3_MATCH_ID);         //输送带1、3的脉冲值也清零，但因为不需要保证每次成功，不进行成功检测
                    // clear_dev_encoder(CONVEYOR_1_MATCH_ID);
                    LOG_UPLOAD("Clear conveyor Enc...");
                }
            }
            else{
                if(0 == washCarNum){            //没有车洗，1#输送带不动
                    if(!is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID)){
                        conveyor_move(CRL_SECTION_1, CMD_STILL);
                    }
                }
                if(1 == washCarNum && 0 == carWash[headWashCarId].headPos){ //只有一辆车洗，车头进入工作区前，1#输送带启动
                    if(is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID)
                    && is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) && is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)){
                        conveyor_move(CRL_SECTION_1, CMD_FORWARD);
                    }
                }
                else{                           //有多辆车洗或只有一辆车已进入工作区，1#输送带跟随2#
                    if(is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) && is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)){
                        if(!is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID) && is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)){
                            conveyor_move(CRL_SECTION_1, CMD_STILL);
                        }
                        else if(is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID) && !is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)){
                            conveyor_move(CRL_SECTION_1, CMD_FORWARD);
                        }
                        else{
                            //输送带1#，2#动作相同，不重复执行
                        }
                    }
                    else if(!is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID)){
                        conveyor_move(CRL_SECTION_1, CMD_STILL);
                    }
                }
            }
        }
        else{
            encClearCnt = 0;
            if(!is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID)){
                conveyor_move(CRL_SECTION_1, CMD_STILL);
            }
        }
        //输送带2#控制
        if(0 == washCarNum){            //没有车洗，2#输送带不动
            if(!is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)){
                conveyor_move(CRL_SECTION_2, CMD_STILL);
            }
        }
        if(1 == washCarNum && 0 == carWash[headWashCarId].headPos){ //只有一辆车洗，2#输送带在车辆进入工作区前一直启动
            if(is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)
            && is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) && is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)){
                conveyor_move(CRL_SECTION_2, CMD_FORWARD);
            }
        }
        else{
            //工作区有车，启停由洗车流程控制
        }
        //输送带3#控制
        if(is_signal_filter_trigger(SIGNAL_EXIT)){          //出口光电触发时，跟随2#输送带工作
            if(!is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID) && is_dev_move_sta_idle(CONVEYOR_3_MATCH_ID)){
                conveyor_move(CRL_SECTION_3, CMD_FORWARD);
            }
            else if(is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID) && !is_dev_move_sta_idle(CONVEYOR_3_MATCH_ID)){
                conveyor_move(CRL_SECTION_3, CMD_STILL);
            }
            else{
                //输送带2#，3#动作相同，不重复执行
            }
            if(carWash[headWashCarId].isCarMoveCompleteArea && carWash[headWashCarId].headOffsetPos > washProcPos.startBackBrush + 100){
                //车辆输送到完成区时间过长则认为异常（这里有可能因为后车洗车头停那）
                if(get_diff_ms(exitNotTriggerTimeStamp) > 140000){
                    LOG_UPLOAD("Car move to complete area over time");
                }
                finishAreaConveyorEnc = xp_osal_get_dev_pos(CONVEYOR_3_MATCH_ID);
                isExitSignalTrigger = true;
                exitTriggerTimeStamp = aos_now_ms();
            }
        }
        else{
            exitNotTriggerTimeStamp = aos_now_ms();
            if(isExitSignalTrigger){
                //离开出口光电时，输送带3如果是停的, 那认为车已经完全越过工作区，不再进行输送
                if(is_dev_move_sta_idle(CONVEYOR_3_MATCH_ID)){
                    isExitSignalTrigger = false;
                    carWash[headWashCarId].isCarMoveCompleteArea = false;
                    LOG_UPLOAD("Car arrive ahead of conveyor move");
                }
                else if((xp_osal_get_dev_pos(CONVEYOR_3_MATCH_ID) - finishAreaConveyorEnc > 100)       //一个脉冲6mm左右
                || is_signal_filter_trigger(SIGNAL_CAR_WHEEL_OUT) || get_diff_ms(exitTriggerTimeStamp) > 10000){
                    conveyor_move(CRL_SECTION_3, CMD_STILL);        //车辆离开出口光电一段距离或者时间后，停止输送带3#
                    isExitSignalTrigger = false;
                    carWash[headWashCarId].isCarMoveCompleteArea = false;
                    LOG_UPLOAD("Car conveyor move to complete area done");
                }
            }
            else{           //没有车辆到达出口，输送带3#停止
                if(!is_dev_move_sta_idle(CONVEYOR_3_MATCH_ID)){
                    conveyor_move(CRL_SECTION_3, CMD_STILL);
                }
            }
        }
        aos_msleep(50);
    }
}

/*                                                         =======================                                                         */
/* ========================================================    单个设备驱动接口    ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       公有开关指令控制
 * @param[in]	swSta               公有开关切换状态
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @param[in]	isSafe              是否安全移动
 */
/* static void com_switch_set_cmd_run(Type_ComSwitchSta_Enum swSta, Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, bool isSafe)
{} */

/**
 * @brief       公有开关指令控制
 * @param[in]	swSta               公有开关切换状态
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令(这里的值不管正负，只代表速度，在osal.c中会根据距离判断方向)
 * @param[in]	pos                 移动到的位置
 */
/* static void com_switch_set_cmd_pos(Type_ComSwitchSta_Enum swSta, Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, int16_t pos)
{} */


/**
 * @brief       私有开关指令控制（给定速度旋转）
 * @param[in]	priSw               私有开关Id
 * @param[in]	matchId             匹配的驱动Id
 * @param[in]	cmd                 控制指令
 */
static void pri_switch_set_cmd_rotation(Type_PriSwitchIndex_Enum priSw, Type_DriverIndex_Enum matchId, Type_DriverCmd_Enum cmd)
{
    Pri_Switch_Table[priSw].actionType = ACTION_ROTATION;   //配置机构的动作类型
    priSwCmd[priSw].cmd         = cmd;
    priSwCmd[priSw].isCmd       = true;

    cmdExeSta[matchId].isCmdMoveGapTime  = (0 == cmd) ? false : true;
    cmdExeSta[matchId].cmdStartTimeStamp = aos_now_ms();
}

/**
 * @brief       私有开关指令控制（给定速度移动）
 * @param[in]	priSw               私有开关Id
 * @param[in]	matchId              匹配的驱动Id
 * @param[in]	cmd                 控制指令
 * @param[in]	isSafe              是否安全移动
 */
static void pri_switch_set_cmd_move(Type_PriSwitchIndex_Enum priSw, Type_DriverIndex_Enum matchId, Type_DriverCmd_Enum cmd, bool isSafe)
{
    Pri_Switch_Table[priSw].actionType = (isSafe ? ACTION_MOVE : ACTION_FORCE_MOVE); //配置机构的动作类型
    priSwCmd[priSw].cmd         = cmd;
    priSwCmd[priSw].isCmd       = true;

    cmdExeSta[matchId].isCmdMoveGapTime  = (0 == cmd) ? false : true;
    cmdExeSta[matchId].cmdStartTimeStamp = aos_now_ms();
}

/**
 * @brief       私有开关指令控制（给定速度移动到某位置）
 * @param[in]	priSw               私有开关Id
 * @param[in]	matchId              匹配的驱动Id
 * @param[in]	cmd                 控制指令
 * @param[in]	pos                 移动到的位置
 */
static void pri_switch_set_cmd_pos(Type_PriSwitchIndex_Enum priSw, Type_DriverIndex_Enum matchId, Type_DriverCmd_Enum cmd, int16_t pos)
{
    Pri_Switch_Table[priSw].actionType = ACTION_MOVE_POSE;
    priSwCmd[priSw].pos         = pos;
    priSwCmd[priSw].cmd         = cmd;
    priSwCmd[priSw].isCmd       = true;

    cmdExeSta[matchId].isCmdMoveGapTime  = (0 == cmd) ? false : true;
    cmdExeSta[matchId].cmdStartTimeStamp = aos_now_ms();
}

/**
 * @brief       私有开关指令控制（给定速度移动一段时间）
 * @param[in]	priSw               私有开关Id
 * @param[in]	matchId              匹配的驱动Id
 * @param[in]	cmd                 控制指令
 * @param[in]	time                移动的时间
 */
static void pri_switch_set_cmd_time(Type_PriSwitchIndex_Enum priSw, Type_DriverIndex_Enum matchId, Type_DriverCmd_Enum cmd, uint16_t time)
{
    Pri_Switch_Table[priSw].actionType = ACTION_MOVE_TIME;
    priSwCmd[priSw].time        = time;
    priSwCmd[priSw].cmd         = cmd;
    priSwCmd[priSw].isCmd       = true;

    cmdExeSta[matchId].isCmdMoveGapTime  = (0 == cmd) ? false : true;
    cmdExeSta[matchId].cmdStartTimeStamp = aos_now_ms();
}

/**
 * @brief       顶刷旋转控制
 * @param[in]	cmd                 控制指令
 * @return      int
 */
static int top_brush_rotation(Type_DriverCmd_Enum cmd)
{
    pri_switch_set_cmd_rotation(PRI_SWITCH_TOP_BRUSH, TOP_BRUSH_MATCH_ID, cmd);
    return 0;
}

/**
 * @brief       前侧刷旋转控制
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @return      int
 */
static int front_side_brush_rotation(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd)
{
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        pri_switch_set_cmd_rotation(PRI_SWITCH_FRONT_LEFT_BRUSH, FRONT_LEFT_BRUSH_MATCH_ID, cmd);
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        pri_switch_set_cmd_rotation(PRI_SWITCH_FRONT_RIGHT_BRUSH, FRONT_RIGHT_BRUSH_MATCH_ID, cmd);
    }
    return 0;
}

/**
 * @brief       后侧刷旋转控制
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @return      int
 */
static int back_side_brush_rotation(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd)
{
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        pri_switch_set_cmd_rotation(PRI_SWITCH_BACK_LEFT_BRUSH, BACK_LEFT_BRUSH_MATCH_ID, cmd);
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        pri_switch_set_cmd_rotation(PRI_SWITCH_BACK_RIGHT_BRUSH, BACK_RIGHT_BRUSH_MATCH_ID, cmd);
    }
    return 0;
}

/**
 * @brief       传送带控制
 * @param[in]	type                控制类型（部件1/部件2/部件3/所有部件）
 * @param[in]	cmd                 控制指令
 * @return      int                 
 */
static int conveyor_move(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd)
{
    if(CRL_SECTION_1 == type || CRL_ALL_SECTION == type){
        pri_switch_set_cmd_move(PRI_SWITCH_CONVEYOR_1, CONVEYOR_1_MATCH_ID, cmd, true);
    }
    if(CRL_SECTION_2 == type || CRL_ALL_SECTION == type){
        pri_switch_set_cmd_move(PRI_SWITCH_CONVEYOR_2, CONVEYOR_2_MATCH_ID, cmd, true);
    }
    if(CRL_SECTION_3 == type || CRL_ALL_SECTION == type){
        pri_switch_set_cmd_move(PRI_SWITCH_CONVEYOR_3, CONVEYOR_3_MATCH_ID, cmd, true);
    }
    return 0;
}

/**
 * @brief       升降控制
 * @param[in]	cmd                 控制指令
 * @param[in]	isSafe              是否安全移动
 * @return      int                 
 */
static int lifter_move(Type_DriverCmd_Enum cmd, bool isSafe)
{
    pri_switch_set_cmd_move(PRI_SWITCH_LIFTER, LIFTER_MATCH_ID, cmd, isSafe);
    return 0;
}

/**
 * @brief       升降移动到固定位置
 * @param[in]	cmd                 控制指令
 * @param[in]	pos                 移动到的位置
 * @return      int                 
 */
static int lifter_move_pos(Type_DriverCmd_Enum cmd, int16_t pos)
{
    if(false == zeroCheck.lifter){
        LOG_UPLOAD("Lifter not check zero, can not move to pos");
        return -1;
    }

    pri_switch_set_cmd_pos(PRI_SWITCH_LIFTER, LIFTER_MATCH_ID, cmd, pos);
    return 0;
}

/**
 * @brief       升降移动固定时间
 * @param[in]	cmd                 控制指令
 * @param[in]	time                移动时间
 * @return      int                 
 */
static int lifter_move_time(Type_DriverCmd_Enum cmd, uint16_t time)
{
    pri_switch_set_cmd_time(PRI_SWITCH_LIFTER, LIFTER_MATCH_ID, cmd, time);
    return 0;
}

/**
 * @brief       前侧刷开合控制（按指定速度移动）
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @param[in]	isSafe              是否安全移动
 * @return      int
 */
static int front_side_brush_move(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, bool isSafe)
{
    if(cmd > 0){
        if(CRL_ONLY_LEFT == type && false == zeroCheck.frontLeftBrushPutter){
            LOG_WARN("Front Side left putter not check zero, can not move forward");
            return -1;
        }
        else if(CRL_ONLY_RIGHT == type && false == zeroCheck.frontRightBrushPutter){
            LOG_WARN("Front Side right putter not check zero, can not move forward");
            return -2;
        }
        else if(CRL_BOTH == type && (false == zeroCheck.frontLeftBrushPutter || false == zeroCheck.frontRightBrushPutter)){
            LOG_WARN("Front Side both putter not check zero, can not move forward");
            return -3;
        }
    }
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        pri_switch_set_cmd_move(PRI_SWITCH_FRONT_LEFT_PUTTER, FRONT_LEFT_MOVE_MATCH_ID, cmd, isSafe);
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        pri_switch_set_cmd_move(PRI_SWITCH_FRONT_RIGHT_PUTTER, FRONT_RIGHT_MOVE_MATCH_ID, cmd, isSafe);
    }
    return 0;
}

/**
 * @brief       前侧刷开合控制（移动到指定距离）
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @param[in]	pos                 移动到的位置
 * @return      int
 */
static int front_side_brush_move_pos(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, int16_t pos)
{
    if(cmd > 0){
        if(CRL_ONLY_LEFT == type && false == zeroCheck.frontLeftBrushPutter){
            LOG_WARN("Front Side left putter not check zero, can not move forward");
            return -1;
        }
        else if(CRL_ONLY_RIGHT == type && false == zeroCheck.frontRightBrushPutter){
            LOG_WARN("Front Side right putter not check zero, can not move forward");
            return -2;
        }
        else if(CRL_BOTH == type && (false == zeroCheck.frontLeftBrushPutter || false == zeroCheck.frontRightBrushPutter)){
            LOG_WARN("Front Side both putter not check zero, can not move forward");
            return -3;
        }
    }
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        if(0 == pos){       //位置为0（限位零点）时，按速度移动控制，避免脉冲计数不对导致没有回到零点
            pri_switch_set_cmd_move(PRI_SWITCH_FRONT_LEFT_PUTTER, FRONT_LEFT_MOVE_MATCH_ID, CMD_BACKWARD, true);
        }
        else{
            pri_switch_set_cmd_pos(PRI_SWITCH_FRONT_LEFT_PUTTER, FRONT_LEFT_MOVE_MATCH_ID, cmd, pos);
        }
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        if(0 == pos){
            pri_switch_set_cmd_move(PRI_SWITCH_FRONT_RIGHT_PUTTER, FRONT_RIGHT_MOVE_MATCH_ID, CMD_BACKWARD, true);
        }
        else{
            pri_switch_set_cmd_pos(PRI_SWITCH_FRONT_RIGHT_PUTTER, FRONT_RIGHT_MOVE_MATCH_ID, cmd, pos);
        }
    }
    return 0;
}

/**
 * @brief       前侧刷开合控制（移动固定时间）
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @param[in]	time                移动的时间
 * @return      int
 */
static int front_side_brush_move_time(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, uint16_t time)
{
    if(cmd > 0){
        if(CRL_ONLY_LEFT == type && false == zeroCheck.frontLeftBrushPutter){
            LOG_WARN("Front Side left putter not check zero, can not move forward");
            return -1;
        }
        else if(CRL_ONLY_RIGHT == type && false == zeroCheck.frontRightBrushPutter){
            LOG_WARN("Front Side right putter not check zero, can not move forward");
            return -2;
        }
        else if(CRL_BOTH == type && (false == zeroCheck.frontLeftBrushPutter || false == zeroCheck.frontRightBrushPutter)){
            LOG_WARN("Front Side both putter not check zero, can not move forward");
            return -3;
        }
    }
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        pri_switch_set_cmd_time(PRI_SWITCH_FRONT_LEFT_PUTTER, FRONT_LEFT_MOVE_MATCH_ID, cmd, time);
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        pri_switch_set_cmd_time(PRI_SWITCH_FRONT_RIGHT_PUTTER, FRONT_RIGHT_MOVE_MATCH_ID, cmd, time);
    }
    return 0;
}

/**
 * @brief       后侧刷开合控制（按指定速度移动）
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @param[in]	isSafe              是否安全移动
 * @return      int
 */
static int back_side_brush_move(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, bool isSafe)
{
    if(cmd > 0){
        if(CRL_ONLY_LEFT == type && false == zeroCheck.backLeftBrushPutter){
            LOG_WARN("Back Side left putter not check zero, can not move forward");
            return -1;
        }
        else if(CRL_ONLY_RIGHT == type && false == zeroCheck.backRightBrushPutter){
            LOG_WARN("Back Side right putter not check zero, can not move forward");
            return -2;
        }
        else if(CRL_BOTH == type && (false == zeroCheck.backLeftBrushPutter || false == zeroCheck.backRightBrushPutter)){
            LOG_WARN("Back Side both putter not check zero, can not move forward");
            return -3;
        }
    }
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        pri_switch_set_cmd_move(PRI_SWITCH_BACK_LEFT_PUTTER, BACK_LEFT_MOVE_MATCH_ID, cmd, isSafe);
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        pri_switch_set_cmd_move(PRI_SWITCH_BACK_RIGHT_PUTTER, BACK_RIGHT_MOVE_MATCH_ID, cmd, isSafe);
    }
    return 0;
}

/**
 * @brief       后侧刷开合控制（移动到指定距离）
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @param[in]	pos                 移动到的位置
 * @return      int
 */
static int back_side_brush_move_pos(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, int16_t pos)
{
    if(cmd > 0){
        if(CRL_ONLY_LEFT == type && false == zeroCheck.backLeftBrushPutter){
            LOG_WARN("Back Side left putter not check zero, can not move forward");
            return -1;
        }
        else if(CRL_ONLY_RIGHT == type && false == zeroCheck.backRightBrushPutter){
            LOG_WARN("Back Side right putter not check zero, can not move forward");
            return -2;
        }
        else if(CRL_BOTH == type && (false == zeroCheck.backLeftBrushPutter || false == zeroCheck.backRightBrushPutter)){
            LOG_WARN("Back Side both putter not check zero, can not move forward");
            return -3;
        }
    }
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        pri_switch_set_cmd_pos(PRI_SWITCH_BACK_LEFT_PUTTER, BACK_LEFT_MOVE_MATCH_ID, cmd, pos);
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        pri_switch_set_cmd_pos(PRI_SWITCH_BACK_RIGHT_PUTTER, BACK_RIGHT_MOVE_MATCH_ID, cmd, pos);
    }
    return 0;
}

/**
 * @brief       后侧刷开合控制（移动固定时间）
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	cmd                 控制指令
 * @param[in]	time                移动的时间
 * @return      int
 */
static int back_side_brush_move_time(Type_CrlType_Enum type, Type_DriverCmd_Enum cmd, uint16_t time)
{
    if(cmd > 0){
        if(CRL_ONLY_LEFT == type && false == zeroCheck.backLeftBrushPutter){
            LOG_WARN("Back Side left putter not check zero, can not move forward");
            return -1;
        }
        else if(CRL_ONLY_RIGHT == type && false == zeroCheck.backRightBrushPutter){
            LOG_WARN("Back Side right putter not check zero, can not move forward");
            return -2;
        }
        else if(CRL_BOTH == type && (false == zeroCheck.backLeftBrushPutter || false == zeroCheck.backRightBrushPutter)){
            LOG_WARN("Back Side both putter not check zero, can not move forward");
            return -3;
        }
    }
    if(CRL_ONLY_LEFT == type || CRL_BOTH == type){
        pri_switch_set_cmd_time(PRI_SWITCH_BACK_LEFT_PUTTER, BACK_LEFT_MOVE_MATCH_ID, cmd, time);
    }
    if(CRL_ONLY_RIGHT == type || CRL_BOTH == type){
        pri_switch_set_cmd_time(PRI_SWITCH_BACK_RIGHT_PUTTER, BACK_RIGHT_MOVE_MATCH_ID, cmd, time);
    }
    return 0;
}

/**
 * @brief       道闸开关
 * @param[in]	type                
 * @param[in]	cmd                 
 * @return      int                 
 */
int gate_change_state(Type_CrlType_Enum type, int cmd)
{
    if(cmd > 0)         cmd = CMD_FORWARD;
    else if(cmd < 0)    cmd = CMD_BACKWARD;
    else                cmd = CMD_STILL;

    if(CRL_SECTION_1 == type || CRL_ALL_SECTION == type){
        pri_switch_set_cmd_move(PRI_SWITCH_GATE_1, GATE_1_MACH_ID, cmd, true);
    }
    if(CRL_SECTION_2 == type || CRL_ALL_SECTION == type){
        osal_dev_io_state_change(BOARD4_OUTPUT_WARNING_BOARD, cmd < 0 ? IO_ENABLE : IO_DISABLE);
    }
    if(CRL_SECTION_3 == type || CRL_ALL_SECTION == type){
        pri_switch_set_cmd_move(PRI_SWITCH_GATE_3, GATE_3_MACH_ID, cmd, true);
    }
    return 0;
}

/*                                                         ====================                                                         */
/* ======================================================      动作执行模块      ======================================================= */
/* ====================================================== (static)非外部调用接口 ======================================================= */
/*                                                         ====================                                                         */

/**
 * @brief       机构回到零点位置模型
 * @param[in]	devType             需要回待机位的机构类型
 * @return      int                 
 */
static int module_dev_reset(Type_ZeroCheckType_Def *devType)
{
    static Type_ModuleExeSta_Def moduleSta = {0};
    static uint8_t checkSignalStableCnt = 0;

    if(!stepSta.isModuleDriverExecuted){                            //未在执行状态时执行驱动
        stepSta.isModuleDriverExecuted = true;
        moduleSta.isComplete = false;
        moduleSta.timeStamp = aos_now_ms();

        char *pC = aos_malloc(100);
        if(devType->lifter){
            lifter_move(CMD_BACKWARD, true);
            if(pC)  strcat(pC,"Lifter ");
        }
        if(devType->frontLeftBrushPutter){
            front_side_brush_move(CRL_ONLY_LEFT, CMD_BACKWARD, true);
            if(pC)  strcat(pC,"FrontLeft ");
        }
        if(devType->frontRightBrushPutter){
            front_side_brush_move(CRL_ONLY_RIGHT, CMD_BACKWARD, true);
            if(pC)  strcat(pC,"FrontRight ");
        }
        if(devType->backLeftBrushPutter){
            back_side_brush_move(CRL_ONLY_LEFT, CMD_BACKWARD, true);
            if(pC)  strcat(pC,"BackLeft ");
        }
        if(devType->backRightBrushPutter){
            back_side_brush_move(CRL_ONLY_RIGHT, CMD_BACKWARD, true);
            if(pC)  strcat(pC,"BackRight ");
        }
        if(devType->entryReadyAreaGate){
            gate_change_state(CRL_SECTION_1, CMD_FORWARD);
            if(pC)  strcat(pC,"Gate1# ");
        }
        if(devType->entryWorkAreaGate){
            osal_dev_io_state_change(BOARD4_OUTPUT_WARNING_BOARD, IO_ENABLE);       //STOP警示牌是边沿触发，这里先置低电平
            if(pC)  strcat(pC,"Gate2# ");
        }
        LOG_UPLOAD("%sstart reset~~~", pC);
        aos_free(pC);
    }

    if(moduleSta.isComplete) return RET_COMPLETE;           //避免完成后重复调用，一直保持完成状态

    //根据限位判断机构是否重置到位
    uint16_t resetSta = 0;
    if(devType->lifter && !is_signal_filter_trigger(SIGNAL_LIFTER_UP)){
        if(is_dev_move_sta_idle(LIFTER_MATCH_ID)){          //未重置到位，但机构已处于空闲状态，重新驱动
            lifter_move(CMD_BACKWARD, true);
        }
        resetSta |= 0x0001;
    }
    if(devType->frontLeftBrushPutter && !is_signal_filter_trigger(SIGNAL_FL_MOVE_ZERO)){
        if(is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID)){
            front_side_brush_move(CRL_ONLY_LEFT, CMD_BACKWARD, true);
        }
        resetSta |= 0x0002;
    }
    if(devType->frontRightBrushPutter && !is_signal_filter_trigger(SIGNAL_FR_MOVE_ZERO)){
        if(is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID)){
            front_side_brush_move(CRL_ONLY_RIGHT, CMD_BACKWARD, true);
        }
        resetSta |= 0x0004;
    }
    if(devType->backLeftBrushPutter && !is_signal_filter_trigger(SIGNAL_BL_MOVE_ZERO)){
        if(is_dev_move_sta_idle(BACK_LEFT_MOVE_MATCH_ID)){
            back_side_brush_move(CRL_ONLY_LEFT, CMD_BACKWARD, true);
        }
        resetSta |= 0x0008;
    }
    if(devType->backRightBrushPutter && !is_signal_filter_trigger(SIGNAL_BR_MOVE_ZERO)){
        if(is_dev_move_sta_idle(BACK_RIGHT_MOVE_MATCH_ID)){
            back_side_brush_move(CRL_ONLY_RIGHT, CMD_BACKWARD, true);
        }
        resetSta |= 0x0010;
    }
    // 这里不做关到位检测，避免有车在预备区挡住了全进光电导致道闸落不下来
    // if(devType->entryReadyAreaGate && !is_signal_filter_trigger(SIGNAL_GATE_1_CLOSE)){
    //     resetSta |= 0x0020;
    // }
    if(devType->entryWorkAreaGate && (is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) || is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN))){
        resetSta |= 0x0040;
    }

    if(devType->entryWorkAreaGate && get_diff_ms(moduleSta.timeStamp) > 1000){
        osal_dev_io_state_change(BOARD4_OUTPUT_WARNING_BOARD, IO_DISABLE);
    }

    if(0 == resetSta){
        if(checkSignalStableCnt++ > 3){     
            LOG_UPLOAD("module_dev_reset complete !");
            moduleSta.isComplete = true;
            return RET_COMPLETE;
        }
    }
    else{
        checkSignalStableCnt = 0;
    }

    if(get_diff_ms(moduleSta.timeStamp) > 20000 + stepSta.pauseTime){
        LOG_UPLOAD("module_dev_reset over time, reset status 0x%X", resetSta);
        if(resetSta & 0x0001)   set_error_state(329, true);
        if(resetSta & 0x0002)   set_error_state(8229, true);
        if(resetSta & 0x0004)   set_error_state(8230, true);
        if(resetSta & 0x0008)   set_error_state(8231, true);
        if(resetSta & 0x0010)   set_error_state(8232, true);
        if(resetSta & 0x0020)   set_error_state(8233, true);
        if(resetSta & 0x0040)   set_error_state(8234, true);
        return ERR_TIMEOUT;
    }
    return NOR_CONTINUE;
}

/**
 * @brief       侧刷回到虚拟零位（兼容回到零点会打到毛刷，影响电流的情况）
 * @return      int                 
 */
static int module_side_putters_open(void)
{
    static Type_ModuleExeSta_Def moduleSta = {0};

    if(!stepSta.isModuleDriverExecuted){
        stepSta.isModuleDriverExecuted = true;
        moduleSta.isComplete = false;
        moduleSta.timeStamp = aos_now_ms();
        front_side_brush_move_pos(CRL_BOTH, CMD_BACKWARD, SIDE_BRUSH_OPEN_POS);
    }

    if(stepSta.isComplete) return RET_COMPLETE;

    if(is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID) && is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID)){
        //未到位的话重试
        if(xp_osal_get_dev_pos(FRONT_LEFT_MOVE_MATCH_ID) > SIDE_BRUSH_OPEN_POS + MOVE_POS_ERR 
        || xp_osal_get_dev_pos(FRONT_RIGHT_MOVE_MATCH_ID) > SIDE_BRUSH_OPEN_POS + MOVE_POS_ERR){
            front_side_brush_move_pos(CRL_BOTH, CMD_BACKWARD, SIDE_BRUSH_OPEN_POS);
        }
        else{
            moduleSta.isComplete = true;
            return RET_COMPLETE;
        }
    }

    if(get_diff_ms(moduleSta.timeStamp) > 15000 + moduleSta.pauseTime){
        LOG_WARN("module_side_putters_open over time");
        return ERR_TIMEOUT;
    }
    return NOR_CONTINUE;
}

/**
 * @brief       毛刷基准电流值校准
 * @param[in]	brushType               校准的毛刷类型
 * @param[in]	isCarIntrude            车辆是否闯入
 * @return      int                 
 */
static int module_brush_current_calibrate(Type_BrushType_Enum brushType, bool isCarIntrude)
{
    static Type_ModuleExeSta_Def moduleSta = {0};
    static uint8_t recordCaliCnt = 0;
    static int32_t topBrushRecordPosNum = 0;
    static bool isStartRecord = false;
    static bool isStartCali = false;
    static int16_t currentSum[BRUSH_NUM] = {0};
    static int16_t recordCurrent[BRUSH_NUM][BRUSH_CALI_READ_CNT] = {0};
    int32_t lifterPos = 0;

    if(!stepSta.isModuleDriverExecuted){
        stepSta.isModuleDriverExecuted = true;
        moduleSta.isComplete = false;
        moduleSta.timeStamp = aos_now_ms();
        LOG_UPLOAD("module_brush_current_calibrate brushType %d start~~~", brushType);

        switch (brushType)
        {
        case BRUSH_TOP:
            top_brush_rotation(CMD_BACKWARD);
            water_system_control(WATER_TOP, true);
            currentSum[BRUSH_TOP] = 0;
            topBrushRecordPosNum = 0;
            break;
        case BRUSH_SIDE_FRONT:
            if(!is_signal_filter_trigger(SIGNAL_FL_MOVE_ZERO) || !is_signal_filter_trigger(SIGNAL_FR_MOVE_ZERO)){
                LOG_UPLOAD("Front side brush not in the init pos, EXIT !");
                return -2;
            }
            front_side_brush_rotation(CRL_ONLY_LEFT, CMD_FORWARD);
            front_side_brush_rotation(CRL_ONLY_RIGHT, CMD_BACKWARD);
            water_system_control(WATER_FRONT_SIDE, true);
            // front_side_brush_move_pos(CRL_BOTH, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
            front_side_brush_move_time(CRL_BOTH, CMD_FORWARD, isCarIntrude ? 800 : 5500);
            currentSum[BRUSH_FRONT_LEFT] = 0;
            currentSum[BRUSH_FRONT_RIGHT] = 0;
            break;
        case BRUSH_SIDE_BACK:
            if(!is_signal_filter_trigger(SIGNAL_BL_MOVE_ZERO) || !is_signal_filter_trigger(SIGNAL_BR_MOVE_ZERO)){
                LOG_UPLOAD("Back side brush not in the init pos, EXIT !");
                return -2;
            }
            back_side_brush_rotation(CRL_ONLY_LEFT, CMD_FORWARD);
            back_side_brush_rotation(CRL_ONLY_RIGHT, CMD_BACKWARD);
            water_system_control(WATER_BACK_SIDE, true);
            // back_side_brush_move_pos(CRL_BOTH, CMD_FORWARD, 30);
            back_side_brush_move_time(CRL_BOTH, CMD_FORWARD, isCarIntrude ? 800 : 3500);
            currentSum[BRUSH_BACK_LEFT] = 0;
            currentSum[BRUSH_BACK_RIGHT] = 0;
            break;
        default:
            return -1;
            break;
        }
        isStartRecord = false;
        recordCaliCnt = 0;
    }

    if(moduleSta.isComplete) return RET_COMPLETE;               //避免完成后重复调用，一直保持完成状态

    if(get_diff_ms(moduleSta.timeStamp) > 20000 + moduleSta.pauseTime){
        LOG_WARN("module_brush_current_calibrate over time");
        return ERR_TIMEOUT;
    }
    if(!isStartRecord){                                         //等待毛刷旋转稳定后采样电流
        if(get_diff_ms(moduleSta.timeStamp) > 2500){
            isStartRecord = true;
            if(BRUSH_TOP == brushType){
                lifter_move(CMD_FORWARD, true);
                isStartCali = false;
            }
            else{
                isStartCali = true;
            }
            moduleSta.timeStamp = aos_now_ms();
        }
        return NOR_CONTINUE;
    }
    else{
        if(!isStartCali && get_diff_ms(moduleSta.timeStamp) < 2000){
            //记录顶刷在一定高度时的电流值（顶刷在一定高度会打到结构框架，在这段区域的电流值判定需要做一定处理）AG上其实不用，兼容愿景7
            lifterPos = xp_osal_get_dev_pos(LIFTER_MATCH_ID);
            if(lifterPos >= 0 && lifterPos < TOP_BRUSH_RECORD_CUR_AREA){
                topBrushPosCurrent[lifterPos] = brush[BRUSH_TOP].current;
                //填补没有记录的数组
                for (uint8_t i = topBrushRecordPosNum; i < lifterPos; i++)
                {
                    topBrushPosCurrent[i] = brush[BRUSH_TOP].current;
                }
                topBrushRecordPosNum = lifterPos + 1;
            }
            else{
                //填补没有记录的数组
                if(topBrushRecordPosNum != TOP_BRUSH_RECORD_CUR_AREA){
                    for (uint8_t i = topBrushRecordPosNum; i < TOP_BRUSH_RECORD_CUR_AREA; i++)
                    {
                        topBrushPosCurrent[i] = brush[BRUSH_TOP].current;
                    }
                }
                isStartCali = true;

                char *tempBuf = aos_malloc(10);
                char *valueBuf = aos_malloc(10 * TOP_BRUSH_RECORD_CUR_AREA);
                memset(valueBuf, 0, 10 * TOP_BRUSH_RECORD_CUR_AREA);
                for (uint8_t i = 0; i < TOP_BRUSH_RECORD_CUR_AREA; i++)
                {
                    sprintf(tempBuf, "%d ", topBrushPosCurrent[i]);
                    strcat(valueBuf, tempBuf);
                }
                LOG_DEBUG("Record lifter pos current %s", valueBuf);
                aos_free(tempBuf);
                aos_free(valueBuf);
            }
        }
        else{
            if(!isStartCali){
                isStartCali = true;
                LOG_UPLOAD("Lifter encoder maybe error, no recorder brush top pos current");
            }
            if(recordCaliCnt < BRUSH_CALI_READ_CNT){
                if(get_diff_ms(moduleSta.timeStamp) > 300){
                    moduleSta.timeStamp = aos_now_ms();
                    if(BRUSH_TOP == brushType){
                        recordCurrent[BRUSH_TOP][recordCaliCnt] = brush[BRUSH_TOP].current;
                        currentSum[BRUSH_TOP] += brush[BRUSH_TOP].current;
                    }
                    else if(BRUSH_SIDE_FRONT == brushType){
                        recordCurrent[BRUSH_FRONT_LEFT][recordCaliCnt] = brush[BRUSH_FRONT_LEFT].current;
                        recordCurrent[BRUSH_FRONT_RIGHT][recordCaliCnt] = brush[BRUSH_FRONT_RIGHT].current;
                        currentSum[BRUSH_FRONT_LEFT] += brush[BRUSH_FRONT_LEFT].current;
                        currentSum[BRUSH_FRONT_RIGHT] += brush[BRUSH_FRONT_RIGHT].current;
                    }
                    else if(BRUSH_SIDE_BACK == brushType){
                        recordCurrent[BRUSH_BACK_LEFT][recordCaliCnt] = brush[BRUSH_BACK_LEFT].current;
                        recordCurrent[BRUSH_BACK_RIGHT][recordCaliCnt] = brush[BRUSH_BACK_RIGHT].current;
                        currentSum[BRUSH_BACK_LEFT] += brush[BRUSH_BACK_LEFT].current;
                        currentSum[BRUSH_BACK_RIGHT] += brush[BRUSH_BACK_RIGHT].current;
                    }
                    recordCaliCnt++;
                }
                return NOR_CONTINUE;
            }
            else{
                char *bufTemp = aos_malloc(10);
                char *bufValue = aos_malloc(10 * BRUSH_CALI_READ_CNT);
                memset(bufValue, 0, 10 * BRUSH_CALI_READ_CNT);      //需清零，避免空间内有字符，导致连接后溢出
                if(BRUSH_TOP == brushType){
                    brush[BRUSH_TOP].baseCurrent = currentSum[BRUSH_TOP] / BRUSH_CALI_READ_CNT;
                    for (uint8_t i = 0; i < BRUSH_CALI_READ_CNT; i++)
                    {
                        sprintf(bufTemp, "%d ", recordCurrent[BRUSH_TOP][i]);
                        strcat(bufValue, bufTemp);
                    }
                    LOG_DEBUG("Top base current %d, ===Record current %s", brush[BRUSH_TOP].baseCurrent, bufValue);
                    //顶刷控制参数
                    brush[BRUSH_TOP].pressTouchcar  = brush[BRUSH_TOP].baseCurrent  + 8;
                    brush[BRUSH_TOP].pressL         = brush[BRUSH_TOP].baseCurrent  + 13;      //自由跟随状态下，阈值下限值正常，使吃毛深度合适
                    brush[BRUSH_TOP].pressL_NoBW    = brush[BRUSH_TOP].baseCurrent  + 15;      //跟随不允许向上，阈值较下限值高，能及时向下贴合车身
                    brush[BRUSH_TOP].pressH_NoFW    = brush[BRUSH_TOP].baseCurrent  + 20;      //跟随不允许向下，阈值较上限值低，不要刷太深
                    brush[BRUSH_TOP].pressH         = brush[BRUSH_TOP].baseCurrent  + 22;      //自由跟随状态下，阈值上限值偏高，加大阈值范围，防止点头
                    brush[BRUSH_TOP].pressProtect   = brush[BRUSH_TOP].baseCurrent  + 50;
                }
                else if(BRUSH_SIDE_FRONT == brushType){
                    brush[BRUSH_FRONT_LEFT].baseCurrent = currentSum[BRUSH_FRONT_LEFT] / BRUSH_CALI_READ_CNT;
                    for (uint8_t i = 0; i < BRUSH_CALI_READ_CNT; i++)
                    {
                        sprintf(bufTemp, "%d ", recordCurrent[BRUSH_FRONT_LEFT][i]);
                        strcat(bufValue, bufTemp);
                    }
                    LOG_DEBUG("Front left base current %d, ===Record current %s", brush[BRUSH_FRONT_LEFT].baseCurrent, bufValue);
                    //左前刷控制参数
                    brush[BRUSH_FRONT_LEFT].pressTouchcar   = brush[BRUSH_FRONT_LEFT].baseCurrent  + 10;
                    brush[BRUSH_FRONT_LEFT].pressL_NoBW     = brush[BRUSH_FRONT_LEFT].baseCurrent  + 10;
                    brush[BRUSH_FRONT_LEFT].pressL          = brush[BRUSH_FRONT_LEFT].baseCurrent  + 50;
                    brush[BRUSH_FRONT_LEFT].pressH_NoFW     = brush[BRUSH_FRONT_LEFT].baseCurrent  + 15;
                    brush[BRUSH_FRONT_LEFT].pressH          = brush[BRUSH_FRONT_LEFT].baseCurrent  + 60;
                    brush[BRUSH_FRONT_LEFT].pressProtect    = brush[BRUSH_FRONT_LEFT].baseCurrent  + 70;

                    brush[BRUSH_FRONT_RIGHT].baseCurrent = currentSum[BRUSH_FRONT_RIGHT] / BRUSH_CALI_READ_CNT;
                    memset(bufValue, 0, 10 * BRUSH_CALI_READ_CNT);
                    for (uint8_t i = 0; i < BRUSH_CALI_READ_CNT; i++)
                    {
                        sprintf(bufTemp, "%d ", recordCurrent[BRUSH_FRONT_RIGHT][i]);
                        strcat(bufValue, bufTemp);
                    }
                    LOG_DEBUG("Front right base current %d, ===Record current %s", brush[BRUSH_FRONT_RIGHT].baseCurrent, bufValue);
                    //右前刷控制参数
                    brush[BRUSH_FRONT_RIGHT].pressTouchcar  = brush[BRUSH_FRONT_RIGHT].baseCurrent + 10;
                    brush[BRUSH_FRONT_RIGHT].pressL_NoBW    = brush[BRUSH_FRONT_RIGHT].baseCurrent + 10;
                    brush[BRUSH_FRONT_RIGHT].pressL         = brush[BRUSH_FRONT_RIGHT].baseCurrent + 50;
                    brush[BRUSH_FRONT_RIGHT].pressH_NoFW    = brush[BRUSH_FRONT_RIGHT].baseCurrent + 15;
                    brush[BRUSH_FRONT_RIGHT].pressH         = brush[BRUSH_FRONT_RIGHT].baseCurrent + 60;
                    brush[BRUSH_FRONT_RIGHT].pressProtect   = brush[BRUSH_FRONT_RIGHT].baseCurrent + 70;
                }
                else if(BRUSH_SIDE_BACK == brushType){
                    brush[BRUSH_BACK_LEFT].baseCurrent = currentSum[BRUSH_BACK_LEFT] / BRUSH_CALI_READ_CNT;
                    for (uint8_t i = 0; i < BRUSH_CALI_READ_CNT; i++)
                    {
                        sprintf(bufTemp, "%d ", recordCurrent[BRUSH_BACK_LEFT][i]);
                        strcat(bufValue, bufTemp);
                    }
                    LOG_DEBUG("Back left base current %d, ===Record current %s", brush[BRUSH_BACK_LEFT].baseCurrent, bufValue);
                    //左后刷控制参数
                    brush[BRUSH_BACK_LEFT].pressTouchcar    = brush[BRUSH_BACK_LEFT].baseCurrent  + 10;
                    brush[BRUSH_BACK_LEFT].pressL_NoBW      = brush[BRUSH_BACK_LEFT].baseCurrent  + 10;
                    brush[BRUSH_BACK_LEFT].pressL           = brush[BRUSH_BACK_LEFT].baseCurrent  + 20;
                    brush[BRUSH_BACK_LEFT].pressH_NoFW      = brush[BRUSH_BACK_LEFT].baseCurrent  + 15;
                    brush[BRUSH_BACK_LEFT].pressH           = brush[BRUSH_BACK_LEFT].baseCurrent  + 30;
                    brush[BRUSH_BACK_LEFT].pressProtect     = brush[BRUSH_BACK_LEFT].baseCurrent  + 40;

                    brush[BRUSH_BACK_RIGHT].baseCurrent = currentSum[BRUSH_BACK_RIGHT] / BRUSH_CALI_READ_CNT;
                    memset(bufValue, 0, 10 * BRUSH_CALI_READ_CNT);
                    for (uint8_t i = 0; i < BRUSH_CALI_READ_CNT; i++)
                    {
                        sprintf(bufTemp, "%d ", recordCurrent[BRUSH_BACK_RIGHT][i]);
                        strcat(bufValue, bufTemp);
                    }
                    LOG_DEBUG("Back right base current %d, ===Record current %s", brush[BRUSH_BACK_RIGHT].baseCurrent, bufValue);
                    //右后刷控制参数
                    brush[BRUSH_BACK_RIGHT].pressTouchcar   = brush[BRUSH_BACK_RIGHT].baseCurrent + 10;
                    brush[BRUSH_BACK_RIGHT].pressL_NoBW     = brush[BRUSH_BACK_RIGHT].baseCurrent + 10;
                    brush[BRUSH_BACK_RIGHT].pressL          = brush[BRUSH_BACK_RIGHT].baseCurrent + 20;
                    brush[BRUSH_BACK_RIGHT].pressH_NoFW     = brush[BRUSH_BACK_RIGHT].baseCurrent + 15;
                    brush[BRUSH_BACK_RIGHT].pressH          = brush[BRUSH_BACK_RIGHT].baseCurrent + 30;
                    brush[BRUSH_BACK_RIGHT].pressProtect    = brush[BRUSH_BACK_RIGHT].baseCurrent + 40;
                }
                aos_free(bufTemp);
                aos_free(bufValue);
                moduleSta.isComplete = true;
                return RET_COMPLETE;
            }
        }
    }
    return NOR_CONTINUE;
}

/**
 * @brief       侧刷同时移动到固定位置模块
 * @param[in]	pos                 移动的目标位置
 * @return      int                
 */
static int module_side_brush_both_move_to_position(Type_SideBrushPos_Enum pos)
{
    static Type_ModuleExeSta_Def moduleSta = {0};
    static bool isLeftInPosition  = false;
    static bool isRightInPosition = false;
    static int32_t leftPutterEncoder = 0;
    static int32_t rightPutterEncoder = 0;
    static uint64_t bothMoveTimeStamp = 0;
    static bool isBothMoveDone = false;

    if(!stepSta.isModuleDriverExecuted){
        stepSta.isModuleDriverExecuted = true;
        moduleSta.isComplete = false;
        moduleSta.timeStamp = aos_now_ms();

        if(SIDE_BRUSH_POS_MIDDLE == pos){
            isSideBrushCantMoveToPose = false;          //移到中间时默认侧刷能到达指定位置
        }
        else{
            if(true == isSideBrushCantMoveToPose){
                LOG_UPLOAD("Now pos can not move side putter to target pos, finish this module");
                return RET_COMPLETE;
            }
        }
        isLeftInPosition   = false;
        isRightInPosition  = false;
        isBothMoveDone     = false;
        leftPutterEncoder  = xp_osal_get_dev_pos(FRONT_LEFT_MOVE_MATCH_ID);
        rightPutterEncoder = xp_osal_get_dev_pos(FRONT_RIGHT_MOVE_MATCH_ID);
        
        switch(pos)
        {
            case SIDE_BRUSH_POS_MIDDLE:
                LOG_UPLOAD("side brush both move to middle start~~~");
                front_side_brush_move_pos(CRL_BOTH, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
                break;
            case SIDE_BRUSH_POS_LEFT:
                LOG_UPLOAD("side brush both move to left start~~~");
                // front_side_brush_move_pos(CRL_ONLY_LEFT, CMD_BACKWARD, PUTTER_GO_SELF_SIDE_OFFSET);
                // front_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_FORWARD, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                front_side_brush_move_time(CRL_ONLY_LEFT, CMD_BACKWARD, 2500);
                front_side_brush_move_time(CRL_ONLY_RIGHT, CMD_FORWARD, 2500);
                break;
            case SIDE_BRUSH_POS_RIGHT:
                LOG_UPLOAD("side brush both move to right start~~~");
                // front_side_brush_move_pos(CRL_ONLY_LEFT, CMD_FORWARD, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                // front_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_BACKWARD, PUTTER_GO_SELF_SIDE_OFFSET);
                front_side_brush_move_time(CRL_ONLY_LEFT, CMD_FORWARD, 5000);
                front_side_brush_move_time(CRL_ONLY_RIGHT, CMD_BACKWARD, 5000);
                break;
            default:
                break;
        }
    }

    if(moduleSta.isComplete) return RET_COMPLETE;       //避免完成后重复调用，一直保持完成状态

    if(get_diff_ms(moduleSta.timeStamp) > 20000 + stepSta.pauseTime){
        LOG_WARN("module_side_brush_both_move_to_position over time");
        return ERR_TIMEOUT;
    }

    if(!isSideBrushCantMoveToPose){
        //毛刷移动过程中，如果有一个毛刷压力过大，则回零，避免在洗车头或车尾的时候，毛刷吃毛过深，导致挤停
        if(brush[BRUSH_FRONT_LEFT].current > (brush[BRUSH_FRONT_LEFT].pressWarning) || brush[BRUSH_FRONT_RIGHT].current > brush[BRUSH_FRONT_RIGHT].pressWarning
        || is_signal_filter_trigger(SIGNAL_FL_BRUSH_CROOKED) || is_signal_filter_trigger(SIGNAL_FR_BRUSH_CROOKED)){
            err_need_flag_handle()->isDetectBrushCroooked = false;  //侧刷触压值过大或触发前后歪时不检测，等待开位后检测
            isSideBrushCantMoveToPose = true;
            LOG_UPLOAD("Side brush current too high, can not move to target position, jump to move zero");
            stepSta.isModuleDriverExecuted = false;
            return module_side_putters_open();  //检测到无法到达目标位置，则不再往目标位置移动，移动到虚拟开位
        }

        // if(is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID))    isLeftInPosition = true;
        // if(is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID))   isRightInPosition = true;

        //有可能开位会误触发一瞬间（不至于脉冲计数清零），导致侧刷移动中途停掉，停止后判断下位置，不在误差范围内则按位置再次移动
        leftPutterEncoder  = xp_osal_get_dev_pos(FRONT_LEFT_MOVE_MATCH_ID);
        rightPutterEncoder = xp_osal_get_dev_pos(FRONT_RIGHT_MOVE_MATCH_ID);
        uint8_t posJudgeErr = 15;
        switch(pos)
        {
            case SIDE_BRUSH_POS_MIDDLE:
                //未来城站，左前侧刷计数老是异常，硬件问题一时无法修复，这个做特殊处理，限制侧刷合中间的时间
                //这个限制基于侧刷是在洗完车身已经合了一定距离后的条件
                if((!isLeftInPosition || !isRightInPosition) && get_diff_ms(moduleSta.timeStamp) > 5000){
                    front_side_brush_move(CRL_BOTH, CMD_STILL, true);
                    LOG_UPLOAD("Side putter encode maybe error, stop move");
                }
                if(!isLeftInPosition){
                    if(is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID)){
                        // if(leftPutterEncoder > PUTTER_GO_MIDDLE_OFFSET - posJudgeErr){
                            isLeftInPosition = true;
                        // }
                        // else{
                        //     front_side_brush_move_pos(CRL_ONLY_LEFT, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
                        // }
                    }
                }
                if(!isRightInPosition){
                    if(is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID)){
                        // if(rightPutterEncoder > PUTTER_GO_MIDDLE_OFFSET - posJudgeErr){
                            isRightInPosition = true;
                        // }
                        // else{
                        //     front_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
                        // }
                    }
                }
                break;
            case SIDE_BRUSH_POS_LEFT:
                if(!isLeftInPosition){
                    if(is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID)){
                        // if(leftPutterEncoder < PUTTER_GO_SELF_SIDE_OFFSET + posJudgeErr){
                            isLeftInPosition = true;
                        // }
                        // else{
                        //     front_side_brush_move_pos(CRL_ONLY_LEFT, CMD_BACKWARD, PUTTER_GO_SELF_SIDE_OFFSET);
                        // }
                    }
                }
                if(!isRightInPosition){
                    if(is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID)){
                        // if(rightPutterEncoder > PUTTER_GO_ANOTHER_SIDE_OFFSET - posJudgeErr){
                            isRightInPosition = true;
                        // }
                        // else{
                        //     front_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_FORWARD, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                        // }
                    }
                }
                break;
            case SIDE_BRUSH_POS_RIGHT:
                if(false == isLeftInPosition){
                    if(is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID)){
                        // if(leftPutterEncoder > PUTTER_GO_ANOTHER_SIDE_OFFSET - posJudgeErr){
                            isLeftInPosition = true;
                        // }
                        // else{
                        //     front_side_brush_move_pos(CRL_ONLY_LEFT, CMD_FORWARD, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                        // }
                    }
                }
                if(false == isRightInPosition){
                    if(is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID)){
                        // if(rightPutterEncoder < PUTTER_GO_SELF_SIDE_OFFSET + posJudgeErr){
                            isRightInPosition = true;
                        // }
                        // else{
                        //     front_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_BACKWARD, PUTTER_GO_SELF_SIDE_OFFSET);
                        // }
                    }
                }
                break;
            default:
                break;
        }

        if(!isBothMoveDone && true == isLeftInPosition && true == isRightInPosition){
            isBothMoveDone = true;
            bothMoveTimeStamp = aos_now_ms();
        }
        if(isBothMoveDone && get_diff_ms(bothMoveTimeStamp) > 1000){             //延迟结束，避免后续调用该接口动作切换太快（保证电机正反转切换停留一段时间）
            LOG_UPLOAD("module_side_brush_both_move_to_position %d completed!", pos);
            moduleSta.isComplete = true;
            return RET_COMPLETE;
        }
        return NOR_CONTINUE;
    }
    else{
        int ret = module_side_putters_open();
        if(ret != NOR_CONTINUE){
            err_need_flag_handle()->isDetectBrushCroooked = true;
            if(RET_COMPLETE == ret
            && (is_signal_filter_trigger(SIGNAL_FL_BRUSH_CROOKED) || is_signal_filter_trigger(SIGNAL_FR_BRUSH_CROOKED))){
                ret = NOR_CONTINUE;     //开到位后，如果检测到前后歪还是触发状态，则在这里等待报警
            }
        }
        return ret;  //检测到无法到达目标位置，则不再往目标位置移动，移动到虚拟开位
    }
}

/**
 * @brief       侧刷换向模型
 * @param[in]	conveyorDir         输送带当前方向
 * @return      int                 
 */
static int module_change_side_brush_rotation(Type_DriverCmd_Enum conveyorDir)
{
    static Type_ModuleExeSta_Def moduleSta = {0};
    static Type_BrushRunMode_Enum frontLeftBrushRunMode;
    static Type_BrushRunMode_Enum frontRightBrushRunMode;
    static Type_BrushRunMode_Enum backLeftBrushRunMode;
    static Type_BrushRunMode_Enum backRightBrushRunMode;
    static int16_t leftBackPutterPos = 0;
    static int16_t rightBackPutterPos = 0;
    static uint8_t changeRotStep = 0;

    if(!stepSta.isModuleDriverExecuted){
        stepSta.isModuleDriverExecuted = true;
        moduleSta.isComplete = false;
        moduleSta.timeStamp = aos_now_ms();
        LOG_UPLOAD("module_change_side_brush_rotation start ~~");
        if(conveyorDir != 0) conveyor_move(CRL_SECTION_2, CMD_STILL);
        frontLeftBrushRunMode   = brush[BRUSH_FRONT_LEFT].runMode;
        frontRightBrushRunMode  = brush[BRUSH_FRONT_RIGHT].runMode;
        backLeftBrushRunMode    = brush[BRUSH_BACK_LEFT].runMode;
        backLeftBrushRunMode    = brush[BRUSH_BACK_RIGHT].runMode;
        brush[BRUSH_FRONT_LEFT].runMode  = BRUSH_MANUAL;
        brush[BRUSH_FRONT_RIGHT].runMode = BRUSH_MANUAL;
        brush[BRUSH_BACK_LEFT].runMode   = BRUSH_MANUAL;
        brush[BRUSH_BACK_RIGHT].runMode  = BRUSH_MANUAL;
        leftBackPutterPos  = xp_osal_get_dev_pos(BACK_LEFT_MOVE_MATCH_ID);
        rightBackPutterPos = xp_osal_get_dev_pos(BACK_RIGHT_MOVE_MATCH_ID);
        back_side_brush_move_time(CRL_BOTH, CMD_BACKWARD, 3000);
        front_side_brush_rotation(CRL_BOTH, CMD_STILL);
        back_side_brush_rotation(CRL_BOTH, CMD_STILL);
        water_system_control(WATER_BACK_SIDE, false);
        changeRotStep = 1;
    }

    if(stepSta.isComplete) return RET_COMPLETE;

    if(1 == changeRotStep && get_diff_ms(moduleSta.timeStamp) > 1500){
        front_side_brush_rotation(CRL_ONLY_LEFT, CMD_BACKWARD);
        front_side_brush_rotation(CRL_ONLY_RIGHT, CMD_FORWARD);
        changeRotStep = 2;
        moduleSta.timeStamp = aos_now_ms();
    }
    else if(2 == changeRotStep && get_diff_ms(moduleSta.timeStamp) > 2000){
        changeRotStep = 3;
        brush[BRUSH_FRONT_LEFT].runMode  = frontLeftBrushRunMode;
        brush[BRUSH_FRONT_RIGHT].runMode = frontRightBrushRunMode;
        moduleSta.timeStamp = aos_now_ms();
        return 1;       //侧刷旋转一定时间后返回值表示前侧刷已经换向完成
    }
    else if(3 == changeRotStep && get_diff_ms(moduleSta.timeStamp) > 12000){    //这个流程中参杂了洗车尾，这个时间需要配合洗车尾时间
        // changeRotStep = 4;
        // back_side_brush_rotation(CRL_ONLY_LEFT, CMD_BACKWARD);
        // back_side_brush_rotation(CRL_ONLY_RIGHT, CMD_FORWARD);
        // water_system_control(WATER_BACK_SIDE, true);
        moduleSta.timeStamp = aos_now_ms();
        // 洗车尾差不多后这个流程就结束，后侧刷不再工作（coneryor速度在洗车尾结束后恢复）
        // conveyor_move(CRL_SECTION_2, conveyorDir);
        changeRotStep = 6;
        LOG_UPLOAD("module_change_side_brush_rotation completed!");
        moduleSta.isComplete = true;
        return RET_COMPLETE;
    }
    else if(4 == changeRotStep && get_diff_ms(moduleSta.timeStamp) > 2000){
        changeRotStep = 5;
        back_side_brush_move_pos(CRL_ONLY_LEFT, CMD_FORWARD, leftBackPutterPos);
        back_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_FORWARD, rightBackPutterPos);
        moduleSta.timeStamp = aos_now_ms();
    }
    else if(5 == changeRotStep 
    && is_dev_move_sta_idle(BACK_LEFT_MOVE_MATCH_ID) && is_dev_move_sta_idle(BACK_RIGHT_MOVE_MATCH_ID)){
        brush[BRUSH_BACK_LEFT].runMode   = backLeftBrushRunMode;
        brush[BRUSH_BACK_RIGHT].runMode  = backLeftBrushRunMode;
        conveyor_move(CRL_SECTION_2, conveyorDir);
        changeRotStep = 6;
        LOG_UPLOAD("module_change_side_brush_rotation completed!");
        moduleSta.isComplete = true;
        return RET_COMPLETE;
    }

    if(get_diff_ms(moduleSta.timeStamp) > 15000 + moduleSta.pauseTime){
        LOG_WARN("module_change_side_brush_rotation over time");
        return ERR_TIMEOUT;
    }
    return NOR_CONTINUE;
}

/*                                                         =======================                                                         */
/* ========================================================    细分步骤调用接口    ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       设备正常归位
 * @return      int                 
 */
int step_dev_back_home(void)
{
    Type_ZeroCheckType_Def checkType = {0};
    
    if(!isDriverExecuted){
        isDriverExecuted = true;
        stepSta.isModuleDriverExecuted = false;
    }

    checkType.lifter                = true;
    checkType.frontLeftBrushPutter  = true;
    checkType.frontRightBrushPutter = true;
    checkType.backLeftBrushPutter   = true;
    checkType.backRightBrushPutter  = true;
    checkType.entryReadyAreaGate    = true;
    checkType.entryWorkAreaGate     = true;

    return module_dev_reset(&checkType);
}

/**
 * @brief       设备报警归位
 * @return      int                 
 */
int step_dev_warning_home(void)
{
    Type_ZeroCheckType_Def checkType = {0};

    if(!isDriverExecuted){
        isDriverExecuted = true;
        stepSta.isModuleDriverExecuted = false;
    }

    checkType.lifter                = true;
    checkType.frontLeftBrushPutter  = true;
    checkType.frontRightBrushPutter = true;
    checkType.backLeftBrushPutter   = true;
    checkType.backRightBrushPutter  = true;
    checkType.entryReadyAreaGate    = false;
    checkType.entryWorkAreaGate     = false;

    return module_dev_reset(&checkType);
}

/**
 * @brief       隧道机洗车步骤
 * @param[in]	completeId          完成的订单Id
 * @return      int                 
 */
int step_dev_wash(uint8_t *completeId)
{
    static Type_SideBrushPos_Enum  sideBrushWashPos = SIDE_BRUSH_POS_MIDDLE;
    uint32_t workAreaConveyorEnc = 0;
    int ret = NOR_CONTINUE;
    static uint8_t printCnt = 0;
    static bool isRearEndProtect = false;
    static bool isRearEndProtectStop = false;
    static bool isLifterDetechProtect = false;
    static bool isLifterDetechProtectStop = false;
    static uint64_t voiceTimeStamp = 0;
    static uint64_t carWashStartTimeStamp = 0;
    static uint64_t carTailLeaveEntrenceTimeStamp = 0;
    static uint64_t carWashTimeStamp[SUPPORT_WASH_NUM_MAX] = {0};   //车辆进入的时间戳
    static uint64_t carProtectTimeStamp = 0;        //防追尾时停止的时间戳
    static uint8_t voiceCnt = 0;
    static bool isCarIntrude = false;
    static bool isPickupTruck = false;
    static uint32_t catrTailTempPos = 0;            //车尾临时值，等待确定后再赋值车尾判定值
    static bool isCarMoveToWash = false;
    static uint32_t recordWorkAreaConveyorEnc = 0;
    static int32_t recordLifterPosValue = 0;
    uint8_t waitOverCnt = 0;

    printCnt++;
    *completeId = 0;        //初始赋值为0，若有完成的订单，赋值完成的车辆Id
    workAreaConveyorEnc = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);

    //这里必须等待清脉冲结束在运行流程，否则会造成位置识别错误
    if(!isClearConveyorEncFinish){
        if(waitOverCnt++ > 200){            //这里的时间需要大于清脉冲的超时时间
            LOG_UPLOAD("Clear conveyor enc over time");
            return ERR_TIMEOUT;
        }
        return NOR_CONTINUE;
    }
    else{
        waitOverCnt = 0;
    }

    if(isNewCarWash){
        carWashStartTimeStamp = aos_now_ms();
        catrTailTempPos = 0;
        isNewCarWash = false;
    }
    
    //记录车头车尾位置
    if(isNewCarReadyWash){
        if(0 == carWash[entryCarIndex].headPos && get_diff_ms(carWashStartTimeStamp) > 120000){
            return ERR_TIMEOUT;
        }
        if(is_signal_filter_trigger(SIGNAL_ENTRANCE)){
            if(!is_dev_move_sta_idle(CONVEYOR_1_MATCH_ID)){     //这里需要等待输送带动起来再去检测
                isCarMoveToWash = true;
                carTailLeaveEntrenceTimeStamp = aos_now_ms();
                if(0 == carWash[entryCarIndex].headPos){
                    carWash[entryCarIndex].headPos = workAreaConveyorEnc;   //记录车头位置
                    LOG_UPLOAD("Car index %d, head pos %d", entryCarIndex, carWash[entryCarIndex].headPos);
                    water_system_control(WATER_HIGH_PRESS_WATER, true);
                    brush[BRUSH_TOP].isReadyCalibrate           = false;
                    brush[BRUSH_TOP].isCalibrated               = false;
                    brush[BRUSH_FRONT_LEFT].isReadyCalibrate    = false;
                    brush[BRUSH_FRONT_LEFT].isCalibrated        = false;
                    brush[BRUSH_FRONT_RIGHT].isReadyCalibrate   = false;
                    brush[BRUSH_FRONT_RIGHT].isCalibrated       = false;
                    brush[BRUSH_BACK_LEFT].isReadyCalibrate     = false;
                    brush[BRUSH_BACK_LEFT].isCalibrated         = false;
                    brush[BRUSH_BACK_RIGHT].isReadyCalibrate    = false;
                    brush[BRUSH_BACK_RIGHT].isCalibrated        = false;
                    carWashTimeStamp[entryCarIndex] = aos_now_ms();
                    isCarIntrude = false;
                    isPickupTruck = false;
                    // set_error_state(8126, false);
                    recordWorkAreaConveyorEnc = 0;
                    recordLifterPosValue = 0;
                }
            }
        }
        // else if(workAreaConveyorEnc - carWash[entryCarIndex].headPos > CAR_MIN_LENGTH){  //车头过一段距离后再检测车尾
        else{       //避免车辆闯入时车尾信息检测延迟，检测完车头直接检测车尾
            if(carWash[entryCarIndex].headPos != 0 && 0 == carWash[entryCarIndex].tailPos){   //有车头位置信息才赋值车尾位置
                //入口光电未遮挡后，记录当前车尾坐标（只记录临时值，避免检测异常，不是真的车尾）
                if(isCarMoveToWash){
                    isCarMoveToWash = false;
                    catrTailTempPos = workAreaConveyorEnc;
                }
                //经过一段时间入口光电状态一直都是未遮挡则认为之前是真的车尾，赋值车尾坐标，否则重新修改临时车尾坐标值
                if(get_diff_ms(carTailLeaveEntrenceTimeStamp) > 1500){
                    carWash[entryCarIndex].tailPos = catrTailTempPos;   //记录车尾位置
                    LOG_UPLOAD("Car index %d, head pos %d, tail pos %d", entryCarIndex, carWash[entryCarIndex].headPos, carWash[entryCarIndex].tailPos);
                    isNewCarReadyWash = false;
                }
            }
        }
    }

    //防闯入判定（因为顶刷校准完成后就会开始前侧刷电流校准，正常前侧刷会移动到中间洗车头的位置，所以需要在前侧刷校准前确定车辆是否闯入）
    if(!isCarIntrude && carWash[entryCarIndex].headOffsetPos > 0 && carWash[entryCarIndex].headOffsetPos < washProcPos.startTopBrush
    && is_signal_filter_trigger(SIGNAL_AVOID_INTRUDE)){
        isCarIntrude = true;
        carWash[entryCarIndex].headMoveValue = washProcPos.startTopBrush;     //闯入后偏移闯入车辆位置，认为已经到了洗车顶位置
        LOG_UPLOAD("Have car intrude !");
        // set_error_state(8126, true);
    }

    //织带松判定（织带松时输送带停止）
    //在进入车辆有车头信息时再判定，避免还没有车头信息时就触发停止，导致输送带一直不动（有线程控制输送带在没有车头信息时会一直控制动）
    if(err_need_flag_handle()->isDetectLifterLooseEnable && carWash[entryCarIndex].headPos != 0 
    && (is_signal_filter_trigger(SIGNAL_LIFTER_LEFT_DETECH) || is_signal_filter_trigger(SIGNAL_LIFTER_RIGHT_DETECH))){
        if(!isLifterDetechProtect){
            isLifterDetechProtect = true;
            isLifterDetechProtectStop = (is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)) ? false : true;
            LOG_UPLOAD("Lifter detech trigger, wait...");
        }
        if(!is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID))  conveyor_move(CRL_SECTION_2, CMD_STILL);
    }
    else{
        if(isLifterDetechProtect){
            LOG_UPLOAD("Lifter detech redress, continue run");
            if(isLifterDetechProtectStop){
                conveyor_move(CRL_SECTION_2, CMD_FORWARD);
            }
        }
        isLifterDetechProtect = false;
    }

    //检测完成区的车辆是否有车防止追尾（防追尾条件：完成光电和防追尾光电触发、出口光电不触发）
    //前车流程在风干步骤及之后时，如果满足防追尾条件认为是同一辆车中间漏光了，不理会
    //只有在前车流程没到风干步骤前，满足防追尾条件，认为是两辆车，允许触发防追尾（在完成区洗完的车会清除该车的所有信息，不算前车）
    if(carWash[headWashCarId].headProc < PROC_START_DYRING){
        if(is_signal_filter_trigger(SIGNAL_FINISH)){
            if(is_signal_filter_trigger(SIGNAL_REAR_END_PROTECT) && !is_signal_filter_trigger(SIGNAL_EXIT)){
                carProtectTimeStamp = aos_now_ms();
                if(!isRearEndProtect){
                    isRearEndProtect = true;
                    isRearEndProtectStop = (is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)) ? false : true;  //如果当前本来就没动，后面就不做恢复
                }
                if(!is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID))  conveyor_move(CRL_SECTION_2, CMD_STILL);
                if(voiceCnt < 5 && get_diff_ms(voiceTimeStamp) > 8000){
                    voiceCnt++;
                    voiceTimeStamp = aos_now_ms();
                    voice_play_set(AG_VOICE_POS_EXIT, AG_VOICE_EXIT_CONGESTION);
                    LOG_UPLOAD("Car in complete area, wait...");
                }
            }
            else{
                isRearEndProtect = false;
            }
        }
        else{
            if(isRearEndProtectStop){
                LOG_UPLOAD("Car move out complete area, continue run");
                conveyor_move(CRL_SECTION_2, CMD_FORWARD);
            }
            isRearEndProtect = false;
            isRearEndProtectStop = false;
            voiceCnt = 0;
        }
    }

    int32_t lifterPos = xp_osal_get_dev_pos(LIFTER_MATCH_ID);
    //车身不同位置顶刷跟随方式变更
    if(carWash[entryCarIndex].headOffsetPos > CAR_MIN_LENGTH / 2 + washProcPos.startTopBrush){
        if(BRUSH_FOLLOW_NO_FORWARD == brush[BRUSH_TOP].runMode){    //大概越过车顶后允许顶刷自由跟随
            brush[BRUSH_TOP].runMode = BRUSH_FREE_FOLLOW;
            LOG_UPLOAD("Top brush change follow mode free");
        }
        if(BRUSH_FREE_FOLLOW == brush[BRUSH_TOP].runMode            //顶刷到车顶下降一定高度后不再允许顶刷上升
        && lifterPos > carWash[entryCarIndex].topLifter
        && lifterPos - carWash[entryCarIndex].topLifter > CAT_TOP_TO_NO_BW_FOLLOW_OFFSET){
            brush[BRUSH_TOP].runMode = BRUSH_FOLLOW_NO_BACKWARD;
            LOG_UPLOAD("Top brush change follow mode no backward");
        }
    }

    //皮卡检测（到最高点向下移动时开始检测下降斜率）
    // if(!isPickupTruck && lifterPos > carWash[entryCarIndex].topLifter){
    //     if(0 == recordWorkAreaConveyorEnc || 0 == recordLifterPosValue){    //首次进入记录初始值
    //         recordWorkAreaConveyorEnc = workAreaConveyorEnc;
    //         recordLifterPosValue = lifterPos;
    //     }
    //     else{
    //         //下降斜率超过 PICKUP_SLOPE_THRESHOLD_VALUE / SLPOE_JUDGE_FREQ_VALUE 不再进行顶刷刷洗
    //         if(lifterPos - recordLifterPosValue > PICKUP_SLOPE_THRESHOLD_VALUE
    //         && workAreaConveyorEnc - recordWorkAreaConveyorEnc < SLPOE_JUDGE_FREQ_VALUE){
    //             LOG_UPLOAD("Is pickup, finish top brush !");
    //             isPickupTruck = true;
    //             set_error_state(8215, true);
    //             brush[BRUSH_TOP].runMode = BRUSH_MANUAL;
    //             brush[BRUSH_TOP].isJogMove = false;
    //             top_brush_rotation(CMD_STILL);
    //             lifter_move(CMD_BACKWARD, true);
    //             water_system_control(WATER_TOP, false);
    //         }
            
    //         if(workAreaConveyorEnc - recordWorkAreaConveyorEnc > SLPOE_JUDGE_FREQ_VALUE){
    //             LOG_DEBUG("Slope %d / %d", lifterPos - recordLifterPosValue, SLPOE_JUDGE_FREQ_VALUE);
    //             recordWorkAreaConveyorEnc = workAreaConveyorEnc;
    //             recordLifterPosValue = lifterPos;
    //         }
    //         LOG_DEBUG("====lifterPos %d, workAreaConveyorEnc %d", lifterPos, workAreaConveyorEnc);
    //     }
    // }

    //轮询判定工作区内各车辆的位置，执行相应的机构（编号0的Id不参与洗车）
    for (uint8_t i = 1; i < SUPPORT_WASH_NUM_MAX; i++)
    {
        if(carWash[i].headPos != 0){
            //洗车超时判定
            if((isRearEndProtect && get_diff_ms(carWashTimeStamp[i]) - get_diff_ms(carProtectTimeStamp) > 300000)
            || (!isRearEndProtect && get_diff_ms(carWashTimeStamp[i]) > 300000)){
                LOG_UPLOAD("Wash car over time");
                return ERR_TIMEOUT;
            }

            //输送带码盘只会正转，不溢出的情况下，数值只增不减，因为每次有车进入工作区，码盘值都会清零，所以不会溢出
            // carWash[i].headMoveValue 为车头位置相对于码盘清零时的位置移动距离值
            carWash[i].headOffsetPos = workAreaConveyorEnc + carWash[i].headMoveValue - carWash[i].headPos; //车头相对于入口光电偏移的距离
            if(0 == printCnt % 30){
                LOG_UPLOAD("Car Index %d relative head entry pos %d", i, carWash[i].headOffsetPos);
            }

            //根据当前车头进入入口光电的距离判定当前处于什么流程（洗车车顶之后的流程不允许按位置距离跳跃）
            if(carWash[i].headProc == PROC_START_WAXWATER && carWash[i].headOffsetPos > washProcPos.startDrying)                        carWash[i].headProc = PROC_START_DYRING;
            else if(carWash[i].headProc == PROC_START_BACK_BRUSH && carWash[i].headOffsetPos > washProcPos.startWaxwater)               carWash[i].headProc = PROC_START_WAXWATER;
            else if(carWash[i].headProc == PROC_START_FRONT_BRUSH && carWash[i].headOffsetPos > washProcPos.startBackBrush
            && (1 == washCarNum || (washCarNum > 1 && carWash[headWashCarId].isBackBrushFinish))){                  //多辆车时后车必需等待前车洗完车尾再开始流程
                carWash[i].headProc = PROC_START_BACK_BRUSH;
            }
            // 前刷启动的流程用毛刷压力值去判定，不用脉冲值（车在输送带上可能打滑）
            // else if(carWash[i].headProc < PROC_START_FRONT_BRUSH && carWash[i].headOffsetPos > washProcPos.startFrontBrush)carWash[i].headProc = PROC_START_FRONT_BRUSH;
            else if(carWash[i].headProc < PROC_START_TOP_BRUSH && carWash[i].headOffsetPos > washProcPos.startTopBrush)                 carWash[i].headProc = PROC_START_TOP_BRUSH;
            else if(carWash[i].headProc < PROC_START_SHAMPOO && carWash[i].headOffsetPos > washProcPos.startShampoo)                    carWash[i].headProc = PROC_START_SHAMPOO;
            else if(carWash[i].headProc < PROC_START_SKIRT_BRUSH_OUT && carWash[i].headOffsetPos > washProcPos.startSkirtBrush + 20)    carWash[i].headProc = PROC_START_SKIRT_BRUSH_OUT;
            else if(carWash[i].headProc < PROC_START_SKIRT_BRUSH_ROTATION && carWash[i].headOffsetPos > washProcPos.startSkirtBrush)    carWash[i].headProc = PROC_START_SKIRT_BRUSH_ROTATION;

            //顶刷电流校准及触碰到车头判定
            if(carWash[i].headProc >= PROC_START_SKIRT_BRUSH_ROTATION && carWash[i].headProc <= PROC_START_TOP_BRUSH){
                if(!brush[BRUSH_TOP].isReadyCalibrate){
                    brush[BRUSH_TOP].isReadyCalibrate = true;
                    stepSta.isModuleDriverExecuted = false;     //重置模型驱动供电流校准使用
                }
                if(brush[BRUSH_TOP].isCalibrated){
                    //若顶刷触碰到车直接跳转到顶刷流程
                    if(carWash[i].headProc < PROC_START_TOP_BRUSH && brush[BRUSH_TOP].current > brush[BRUSH_TOP].pressTouchcar){
                        LOG_UPLOAD("Top brush touch car ahead of pos offset, current now %d, touch %d", brush[BRUSH_TOP].current, brush[BRUSH_TOP].pressTouchcar);
                        //流程跳跃则补全跳过流程的动作
                        osal_dev_io_state_change(BOARD0_OUTPUT_SKIRT_BRUSH_VALVE, IO_ENABLE);
                        water_system_control(WATER_SHAMPOO, true);
                        water_system_control(WATER_SHAMPOO_PIKN, true);
                        water_system_control(WATER_SHAMPOO_GREEN, true);
                        water_system_control(WATER_BASE_PLATE, true);
                        carWash[i].headProc = PROC_START_TOP_BRUSH;
                        /* 顶刷触碰到轿车位置非车头位置，这里不刷新 */
                        // //刷新车头位置（有可能打滑，导致后面比如吹风在车没到的时候就开始吹了）
                        // if(carWash[i].headOffsetPos > washProcPos.startTopBrush){
                        //     carWash[i].headPos += (carWash[i].headOffsetPos - washProcPos.startTopBrush);
                        // }
                        // LOG_UPLOAD("Car Index %d change proc to %d, head offset pos %d, refresh head pos to %d", 
                        // i, carWash[i].headProc, carWash[i].headOffsetPos, carWash[i].headPos);
                    }
                }
                else{
                    //顶刷电流校准（校准过程中，顶刷降到下限位）
                    ret = module_brush_current_calibrate(BRUSH_TOP, isCarIntrude);    //需保证在车辆到达顶刷位置前完成电流校准
                    if(ret < 0){
                        return ret;
                        LOG_UPLOAD("module_brush_current_calibrate top err ret %d",ret);
                    }
                    else if(RET_COMPLETE == ret){
                        ret = NOR_CONTINUE;
                        brush[BRUSH_TOP].isCalibrated = true;
                        //顶刷跟随在校准完就开启，不需要等到位置到达，防止有车闯入的情况
                        brush[BRUSH_TOP].runMode   = BRUSH_FOLLOW_NO_FORWARD;   //顶刷跟随（禁止向下）
                        brush[BRUSH_TOP].isJogMove = true;
                        LOG_UPLOAD("Start top brush follow mode no forward");
                        memset(recordAreaPos, 0, sizeof(recordAreaPos));        //顶刷开始跟随后，清除车高记录数据
                        memset(recordLifterPos, 0, sizeof(recordLifterPos));
                    }
                }
            }
            //前侧刷电流校准及车头判定
            if(carWash[i].headProc >= PROC_START_TOP_BRUSH && carWash[i].headProc < PROC_START_FRONT_BRUSH
            && brush[BRUSH_TOP].isCalibrated){
                if(!brush[BRUSH_FRONT_LEFT].isReadyCalibrate || !brush[BRUSH_FRONT_RIGHT].isReadyCalibrate){
                    brush[BRUSH_FRONT_LEFT].isReadyCalibrate = true;
                    brush[BRUSH_FRONT_RIGHT].isReadyCalibrate = true;
                    stepSta.isModuleDriverExecuted = false;
                }
                if(brush[BRUSH_FRONT_LEFT].isCalibrated && brush[BRUSH_FRONT_RIGHT].isCalibrated){
                    if((carWash[i].headOffsetPos > washProcPos.startFrontBrush 
                    && (brush[BRUSH_FRONT_LEFT].current > brush[BRUSH_FRONT_LEFT].pressTouchcar || brush[BRUSH_FRONT_RIGHT].current > brush[BRUSH_FRONT_RIGHT].pressTouchcar))
                    || brush[BRUSH_FRONT_LEFT].current > brush[BRUSH_FRONT_LEFT].pressProtect || brush[BRUSH_FRONT_RIGHT].current > brush[BRUSH_FRONT_RIGHT].pressProtect){
                        carWash[i].headProc = PROC_START_FRONT_BRUSH;
                        LOG_DEBUG("Front brush touch car, current left %d, right %d", brush[BRUSH_FRONT_LEFT].current, brush[BRUSH_FRONT_RIGHT].current);
                        //刷新车头位置（有可能打滑，导致后面比如后侧刷在车没到的时候就开始合了。提前到的不管，后面的机构晚一点动作没什么风险）
                        if(carWash[i].headOffsetPos > washProcPos.startFrontBrush){
                            carWash[i].headPos += (carWash[i].headOffsetPos - washProcPos.startFrontBrush + 25);//25为毛刷深度补偿值
                        }
                        LOG_UPLOAD("Car Index %d change proc to %d, head offset pos %d, reflash head pos to %d", 
                        i, carWash[i].headProc, carWash[i].headOffsetPos, carWash[i].headPos);
                    }
                }
                else{
                    ret = module_brush_current_calibrate(BRUSH_SIDE_FRONT, isCarIntrude);   //需保证在车辆到达前侧刷位置前完成电流校准
                    if(ret < 0){
                        return ret;
                        LOG_UPLOAD("module_brush_current_calibrate front brush err ret %d", ret);
                    }
                    else if(RET_COMPLETE == ret){
                        ret = NOR_CONTINUE;
                        brush[BRUSH_FRONT_LEFT].isCalibrated = true;
                        brush[BRUSH_FRONT_RIGHT].isCalibrated = true;
                        if(isCarIntrude){
                            carWash[i].headProc = PROC_START_FRONT_BRUSH;
                            LOG_UPLOAD("Car intrude, change proc to start front brush immediately");
                        }
                        // if(xp_osal_get_dev_pos(FRONT_LEFT_MOVE_MATCH_ID) > PUTTER_GO_MIDDLE_OFFSET - MOVE_POS_ERR
                        // && xp_osal_get_dev_pos(FRONT_RIGHT_MOVE_MATCH_ID) > PUTTER_GO_MIDDLE_OFFSET - MOVE_POS_ERR){
                        //     brush[BRUSH_FRONT_LEFT].isCalibrated = true;
                        //     brush[BRUSH_FRONT_RIGHT].isCalibrated = true;
                        // }
                    }
                }
            }
            //后侧刷电流校准(如果前面有车在洗时，需要等前面的车后侧刷流程结束后再进行校准)
            if((1 == washCarNum || (washCarNum > 1 && carWash[headWashCarId].isBackBrushFinish))
            && carWash[i].isFrontBrushWashBody
            && carWash[i].headProc >= PROC_START_FRONT_BRUSH && carWash[i].headProc < PROC_START_BACK_BRUSH
            && brush[BRUSH_FRONT_LEFT].isCalibrated && brush[BRUSH_FRONT_RIGHT].isCalibrated){
                if(!brush[BRUSH_BACK_LEFT].isReadyCalibrate || !brush[BRUSH_BACK_RIGHT].isReadyCalibrate){
                    brush[BRUSH_BACK_LEFT].isReadyCalibrate = true;
                    brush[BRUSH_BACK_RIGHT].isReadyCalibrate = true;
                    stepSta.isModuleDriverExecuted = false;
                }
                if(brush[BRUSH_BACK_LEFT].isCalibrated && brush[BRUSH_BACK_RIGHT].isCalibrated){
                    //
                }
                else{
                    ret = module_brush_current_calibrate(BRUSH_SIDE_BACK, isCarIntrude);    //需保证在车辆到达后侧刷位置前完成电流校准
                    if(ret < 0){
                        return ret;
                        LOG_UPLOAD("module_brush_current_calibrate back brush err ret %d", ret);
                    }
                    else if(RET_COMPLETE == ret){
                        ret = NOR_CONTINUE;
                        brush[BRUSH_BACK_LEFT].isCalibrated = true;
                        brush[BRUSH_BACK_RIGHT].isCalibrated = true;
                    }
                }
            }

            //前侧刷越过车头后自由跟随
            if(carWash[i].isFrontBrushInHeadArea && carWash[i].headOffsetPos > HEAD_OFFSET_FRONT_BRUSH_OVER_HEAD){
                carWash[i].isFrontBrushInHeadArea = false;
                brush[BRUSH_FRONT_LEFT].runMode    = BRUSH_FREE_FOLLOW;
                brush[BRUSH_FRONT_RIGHT].runMode   = BRUSH_FREE_FOLLOW;
                brush[BRUSH_FRONT_LEFT].isJogMove  = true;
                brush[BRUSH_FRONT_RIGHT].isJogMove = true;
            }
            //后侧刷越过车头后自由跟随
            if(carWash[i].isBackBrushInHeadArea && carWash[i].headOffsetPos > washProcPos.startWaxwater){
                carWash[i].isBackBrushInHeadArea  = false;
                brush[BRUSH_BACK_LEFT].runMode    = BRUSH_FREE_FOLLOW;
                brush[BRUSH_BACK_RIGHT].runMode   = BRUSH_FREE_FOLLOW;
                brush[BRUSH_BACK_LEFT].isJogMove  = true;
                brush[BRUSH_BACK_RIGHT].isJogMove = true;
            }

            //判断是否进入了新流程
            carWash[i].isHeadProcChanged = (carWash[i].lastHeadProc == carWash[i].headProc) ? false : true;
            carWash[i].lastHeadProc = carWash[i].headProc;
            if(carWash[i].isHeadProcChanged){
                LOG_UPLOAD("Car Index %d change head proc to %d", i, carWash[i].headProc);
            }

            switch (carWash[i].headProc)
            {
            case PROC_START_SKIRT_BRUSH_ROTATION:   //开始进程群刷旋转
                if(carWash[i].isHeadProcChanged){
                    carWash[i].isHeadProcChanged = false;
                    osal_dev_io_state_change(BOARD4_OUTPUT_LEFT_SKIRT_ROTATION, IO_ENABLE);
                    osal_dev_io_state_change(BOARD4_OUTPUT_RIGHT_SKIRT_ROTATION, IO_ENABLE);
                }
                break;
            case PROC_START_SKIRT_BRUSH_OUT:        //开始进程群刷伸出
                if(carWash[i].isHeadProcChanged){
                    carWash[i].isHeadProcChanged = false;
                    osal_dev_io_state_change(BOARD0_OUTPUT_SKIRT_BRUSH_VALVE, IO_ENABLE);
                }
                break;
            case PROC_START_SHAMPOO:                //开始进程喷香波
                if(carWash[i].isHeadProcChanged){
                    carWash[i].isHeadProcChanged = false;
                    water_system_control(WATER_SHAMPOO, true);
                    water_system_control(WATER_SHAMPOO_PIKN, true);
                    water_system_control(WATER_SHAMPOO_GREEN, true);
                    water_system_control(WATER_BASE_PLATE, true);
                }
                //前轮可能在预备区和工作区输送带间打滑，这里加个超时判定
                if(get_diff_ms(carWashTimeStamp[i]) - get_diff_ms(carProtectTimeStamp) > 120000){
                    LOG_UPLOAD("Car head move to front brush over time");
                    return ERR_TIMEOUT;
                }
                break;
            case PROC_START_TOP_BRUSH:              //开始进程洗车顶
                if(carWash[i].isHeadProcChanged){
                    if(!brush[BRUSH_TOP].isCalibrated && !isCarIntrude){
                        return -6;
                        LOG_UPLOAD("Proc start top brush but not calibrated");
                    }
                    carWash[i].isHeadProcChanged = false;
                    //顶刷跟随在校准完就开启，不需要等到位置到达，防止有车闯入的情况
                    //有车闯入会直接跳到洗车顶，补齐前面的流程动作
                    if(isCarIntrude){
                        osal_dev_io_state_change(BOARD4_OUTPUT_LEFT_SKIRT_ROTATION, IO_ENABLE);
                        osal_dev_io_state_change(BOARD4_OUTPUT_RIGHT_SKIRT_ROTATION, IO_ENABLE);
                        osal_dev_io_state_change(BOARD0_OUTPUT_SKIRT_BRUSH_VALVE, IO_ENABLE);
                        water_system_control(WATER_SHAMPOO, true);
                        water_system_control(WATER_SHAMPOO_PIKN, true);
                        water_system_control(WATER_SHAMPOO_GREEN, true);
                        water_system_control(WATER_BASE_PLATE, true);
                    }
                }
                break;
            case PROC_START_FRONT_BRUSH:                        //开始进程前侧刷洗
                if(carWash[i].isHeadProcChanged){
                    if(!brush[BRUSH_FRONT_LEFT].isCalibrated || !brush[BRUSH_FRONT_RIGHT].isCalibrated){
                        return -7;
                        LOG_UPLOAD("Proc start front side brush but not calibrated");
                    }
                    carWash[i].isHeadProcChanged = false;
                    if(!isCarIntrude){
                        conveyor_move(CRL_SECTION_2, CMD_STILL);
                        stepSta.isModuleDriverExecuted = false;     //重置模型驱动
                        sideBrushWashPos = SIDE_BRUSH_POS_LEFT;     //开始前侧刷时，侧刷已经在中间，不需要再移动到中间
                        isSideBrushCantMoveToPose = false;          //默认侧刷能到达指定位置，避免前面一辆车洗车尾时把该标志位置true了
                        carWash[i].isWashCarHeadFinish = false;
                        water_system_control(WATER_SHAMPOO, false);
                        water_system_control(WATER_SHAMPOO_PIKN, false);
                        water_system_control(WATER_SHAMPOO_GREEN, false);
                        water_system_control(WATER_BASE_PLATE, false);
                        water_system_control(WATER_HIGH_PRESS_WATER, false);
                        brush[BRUSH_TOP].runMode = BRUSH_MANUAL;    //洗车头时，停止顶刷跟随，上升一定距离，避免顶刷长时间刷车身
                        lifter_move_time(CMD_BACKWARD, 1500);
                    }
                    else{
                        carWash[i].isWashCarHeadFinish = true;      //有车闯入不洗车头
                    }
                    carWash[i].isFrontBrushWashBody     = false;
                    carWash[i].isFrontBrushInHeadArea   = false;
                    carWash[i].isBackBrushInHeadArea    = false;
                }
                if(!carWash[i].isWashCarHeadFinish){
                    ret = module_side_brush_both_move_to_position(sideBrushWashPos);
                    if(ret < 0){
                        return ret;
                        LOG_UPLOAD("Front side brush move to pos err ret %d", ret);
                    }
                    else{
                        if(RET_COMPLETE == ret){
                            if(SIDE_BRUSH_POS_MIDDLE == sideBrushWashPos){
                                sideBrushWashPos = SIDE_BRUSH_POS_LEFT;
                                stepSta.isModuleDriverExecuted = false;                 //重置模型驱动
                            }
                            else if(SIDE_BRUSH_POS_LEFT == sideBrushWashPos){
                                sideBrushWashPos = SIDE_BRUSH_POS_RIGHT;
                                stepSta.isModuleDriverExecuted = false;                 //重置模型驱动
                            }
                            else if(SIDE_BRUSH_POS_RIGHT == sideBrushWashPos){
                                LOG_UPLOAD("Front side brush wash car head finish");
                                front_side_brush_move_time(CRL_ONLY_LEFT, CMD_BACKWARD, 5000);
                                //刷洗完车头，侧刷合到一定位置，准备刷洗车头弧度处。左右移动有误差，进行一定补偿
                                // front_side_brush_move_pos(CRL_ONLY_LEFT, CMD_BACKWARD, 30);
                                // front_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_FORWARD, 30 - 5);
                                lifter_move_time(CMD_FORWARD, 1500);                    //这个时间不要大于侧刷移动的时间，略小于上升的时间
                                carWash[i].isWashCarHeadFinish = true;
                                water_system_control(WATER_SHAMPOO, true);
                                water_system_control(WATER_SHAMPOO_PIKN, true);
                                water_system_control(WATER_SHAMPOO_GREEN, true);
                                water_system_control(WATER_BASE_PLATE, true);
                                water_system_control(WATER_HIGH_PRESS_WATER, true);
                            }
                            ret = NOR_CONTINUE;
                        }
                    }
                }
                else{
                    //前侧刷洗完车头后开始清洗车身
                    if(!carWash[i].isFrontBrushWashBody
                    && is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID) && is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID)){
                        carWash[i].isFrontBrushWashBody = true;
                        carWash[i].isFrontBrushInHeadArea = true;
                        conveyor_move(CRL_SECTION_2, CMD_FORWARD);
                        brush[BRUSH_TOP].runMode = BRUSH_FOLLOW_NO_FORWARD;   //车头洗完恢复跟随
                        brush[BRUSH_FRONT_LEFT].runMode    = BRUSH_FOLLOW_NO_FORWARD;
                        brush[BRUSH_FRONT_RIGHT].runMode   = BRUSH_FOLLOW_NO_FORWARD;
                        brush[BRUSH_FRONT_LEFT].isJogMove  = false;
                        brush[BRUSH_FRONT_RIGHT].isJogMove = false;
                    }
                }
                break;
            case PROC_START_BACK_BRUSH:                         //开始进程后侧刷洗
                if(carWash[i].isHeadProcChanged){
                    carWash[i].isHeadProcChanged = false;
                    brush[BRUSH_BACK_LEFT].runMode    = BRUSH_FOLLOW_NO_FORWARD;
                    brush[BRUSH_BACK_RIGHT].runMode   = BRUSH_FOLLOW_NO_FORWARD;
                    brush[BRUSH_BACK_LEFT].isJogMove  = false;
                    brush[BRUSH_BACK_RIGHT].isJogMove = false;
                    carWash[i].isBackBrushInHeadArea  = true;
                    carWash[i].isCarMoveCompleteArea  = true;   //开始后侧刷后即将到达完成光电，准备输送到完成区
                    carWash[i].isBackBrushFinish = false;
                }
                break;
            case PROC_START_WAXWATER:        //开始进程喷蜡水
                if(carWash[i].isHeadProcChanged){
                    carWash[i].isHeadProcChanged = false;
                    water_system_control(WATER_WAXWATER, true);
                    water_system_control(WATER_CLEAR_WATER, true);
                }
                break;
            case PROC_START_DYRING:                             //开始进程吹风
                if(carWash[i].isHeadProcChanged){
                    carWash[i].isHeadProcChanged = false;
                    dryer_work(CRL_ALL_SECTION, true);
                }
                break;
            default:
                break;
            }

            if(carWash[i].tailPos != 0){
                //输送带码盘只会正转，不溢出的情况下，数值只增不减，因为每次有车进入工作区，码盘值都会清零，所以不会溢出
                // carWash[i].tailMoveValue 为车尾位置相对于码盘清零时的位置移动距离值
                carWash[i].tailOffsetPos = workAreaConveyorEnc + carWash[i].tailMoveValue - carWash[i].tailPos; //车尾相对于入口光电偏移的距离

                if(0 == printCnt % 30){
                    LOG_UPLOAD("Car Index %d relative tail entry pos %d", i, carWash[i].tailOffsetPos);
                }
                //根据当前车尾离开入口光电的距离判定当前处于什么流程（不允许按位置距离位置跳跃步骤）
                if(carWash[i].tailProc == PROC_FINISH_WAXWAT && carWash[i].tailOffsetPos > washProcPos.endDrying && !is_signal_filter_trigger(SIGNAL_EXIT)) carWash[i].tailProc = PROC_FINISH_DYRING;
                else if(carWash[i].tailProc == PROC_FINISH_BACK_BRUSH && carWash[i].tailOffsetPos > washProcPos.endWaxwater)    carWash[i].tailProc = PROC_FINISH_WAXWAT;
                else if(carWash[i].tailProc == PROC_FINISH_FRONT_BRUSH && carWash[i].tailOffsetPos > washProcPos.endBackBrush)  carWash[i].tailProc = PROC_FINISH_BACK_BRUSH;
                else if(carWash[i].tailProc == PROC_FINISH_TOP_BRUSH && carWash[i].tailOffsetPos > washProcPos.endFrontBrush)   carWash[i].tailProc = PROC_FINISH_FRONT_BRUSH;
                else if(carWash[i].tailProc == PROC_FINISH_SHAMPOO && carWash[i].tailOffsetPos > washProcPos.endTopBrush)       carWash[i].tailProc = PROC_FINISH_TOP_BRUSH;
                else if(carWash[i].tailProc == PROC_FINISH_SKIRT_BRUSH && carWash[i].tailOffsetPos > washProcPos.endShampoo)    carWash[i].tailProc = PROC_FINISH_SHAMPOO;
                else if(carWash[i].tailProc == PROC_FINISH_HIGH_PUMP && carWash[i].tailOffsetPos > washProcPos.endSkirtBrush)   carWash[i].tailProc = PROC_FINISH_SKIRT_BRUSH;
                else if(carWash[i].tailProc < PROC_FINISH_HIGH_PUMP && carWash[i].tailOffsetPos > washProcPos.endHighPump)      carWash[i].tailProc = PROC_FINISH_HIGH_PUMP;

                //顶刷在车尾提前取消点动
                if(brush[BRUSH_TOP].isJogMove && carWash[i].tailOffsetPos > washProcPos.endTopBrush - 50){
                    brush[BRUSH_TOP].isJogMove = false;
                }
                //侧刷在车尾处提前关闭跟随，一直往里合贴合车尾拐角处
                if(carWash[i].tailProc < PROC_FINISH_FRONT_BRUSH && carWash[i].tailOffsetPos > washProcPos.endFrontBrush - 50){
                    brush[BRUSH_FRONT_LEFT].runMode  = BRUSH_MANUAL;
                    brush[BRUSH_FRONT_RIGHT].runMode = BRUSH_MANUAL;
                    brush[BRUSH_FRONT_LEFT].isJogMove  = false;
                    brush[BRUSH_FRONT_RIGHT].isJogMove = false;
                    if(is_dev_move_sta_idle(FRONT_LEFT_MOVE_MATCH_ID)){
                        front_side_brush_move_pos(CRL_ONLY_LEFT, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
                    }
                    if(is_dev_move_sta_idle(FRONT_RIGHT_MOVE_MATCH_ID)){
                        front_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
                    }
                }
                //后侧刷不洗车尾这里就屏蔽
                // if(carWash[i].tailProc < PROC_FINISH_BACK_BRUSH && carWash[i].tailOffsetPos > washProcPos.endBackBrush - 50){
                //     brush[BRUSH_BACK_LEFT].runMode  = BRUSH_MANUAL;
                //     brush[BRUSH_BACK_RIGHT].runMode = BRUSH_MANUAL;
                //     brush[BRUSH_BACK_LEFT].isJogMove  = false;
                //     brush[BRUSH_BACK_RIGHT].isJogMove = false;
                //     if(is_dev_move_sta_idle(BACK_LEFT_MOVE_MATCH_ID)){
                //         back_side_brush_move_pos(CRL_ONLY_LEFT, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
                //     }
                //     if(is_dev_move_sta_idle(BACK_RIGHT_MOVE_MATCH_ID)){
                //         back_side_brush_move_pos(CRL_ONLY_RIGHT, CMD_FORWARD, PUTTER_GO_MIDDLE_OFFSET);
                //     }
                // }

                //判断是否进入了新流程
                carWash[i].isTailProcChanged = (carWash[i].lastTailProc == carWash[i].tailProc) ? false : true;
                carWash[i].lastTailProc = carWash[i].tailProc;
                if(carWash[i].isTailProcChanged){
                    LOG_UPLOAD("Car Index %d change tail proc to %d", i, carWash[i].tailProc);
                }

                Type_ZeroCheckType_Def checkType;
                switch (carWash[i].tailProc)
                {
                case PROC_FINISH_HIGH_PUMP:                     //结束高压喷水
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        water_system_control(WATER_HIGH_PRESS_WATER, false);
                    }
                    break;
                case PROC_FINISH_SKIRT_BRUSH:                   //结束裙边刷洗
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        osal_dev_io_state_change(BOARD0_OUTPUT_SKIRT_BRUSH_VALVE, IO_DISABLE);
                        osal_dev_io_state_change(BOARD4_OUTPUT_LEFT_SKIRT_ROTATION, IO_DISABLE);
                        osal_dev_io_state_change(BOARD4_OUTPUT_RIGHT_SKIRT_ROTATION, IO_DISABLE);
                    }
                    break;
                case PROC_FINISH_SHAMPOO:                       //结束香波
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        water_system_control(WATER_SHAMPOO, false);
                        water_system_control(WATER_SHAMPOO_PIKN, false);
                        water_system_control(WATER_SHAMPOO_GREEN, false);
                        water_system_control(WATER_BASE_PLATE, false);
                    }
                    break;
                case PROC_FINISH_TOP_BRUSH:                     //结束顶部刷洗
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        brush[BRUSH_TOP].runMode   = BRUSH_MANUAL;      //离开车尾关闭顶刷跟随
                        brush[BRUSH_TOP].isJogMove = false;
                        top_brush_rotation(CMD_STILL);
                        water_system_control(WATER_TOP, false);
                        //从前往后查找前车窗位置（从车头找到车中间位置）
                        carWash[i].headWindowsOffsetHead = 0;
                        for (uint8_t j = (washProcPos.startTopBrush / CAR_POS_RECORD_INFO_ACCURACY);
                            j < (((carWash[i].tailPos - carWash[i].headPos) / 2 + washProcPos.startTopBrush) / CAR_POS_RECORD_INFO_ACCURACY); j++)
                        {
                            if(recordLifterPos[j] != 0 && (recordLifterPos[j] < carWash[i].topLifter + CAR_WINDOWS_TO_TOP_OFFSET)){
                                if(recordAreaPos[j] < washProcPos.startTopBrush){
                                    LOG_UPLOAD("Error! head windows pos [%d]=%d, before head pos %d", j, recordAreaPos[j], washProcPos.startTopBrush);
                                    // return -3;
                                }
                                else{
                                    carWash[i].headWindowsOffsetHead = recordAreaPos[j] - washProcPos.startTopBrush;
                                }
                                break;
                            }
                        }
                        //从后往前查找后车窗位置（从车尾找到车中间位置）
                        carWash[i].tailWindowsOffsetHead = 0;
                        for (uint8_t j = ((carWash[i].tailPos + washProcPos.startTopBrush) / CAR_POS_RECORD_INFO_ACCURACY); \
                            j > (((carWash[i].tailPos - carWash[i].headPos) / 2 + washProcPos.startTopBrush) / CAR_POS_RECORD_INFO_ACCURACY); j--)
                        {
                            if(recordLifterPos[j] != 0 && (recordLifterPos[j] < carWash[i].topLifter + CAR_WINDOWS_TO_TOP_OFFSET)){
                                if(recordAreaPos[j] < washProcPos.startTopBrush){
                                    LOG_UPLOAD("Error! tail windows pos [%d]=%d, before head pos %d", j, recordAreaPos[j], washProcPos.startTopBrush);
                                    // return -4;
                                }
                                else{
                                    carWash[i].tailWindowsOffsetHead = recordAreaPos[j] - washProcPos.startTopBrush;
                                }
                                break;
                            }
                        }
                        LOG_UPLOAD("Find car windows offset head %d, head windows %d, tail windows %d", carWash[i].headPos, carWash[i].headWindowsOffsetHead, carWash[i].tailWindowsOffsetHead);
                        stepSta.isModuleDriverExecuted = false;         //重置模型驱动（用于侧刷归位）
                    }
                    checkType.lifter                = true;
                    checkType.frontLeftBrushPutter  = false;
                    checkType.frontRightBrushPutter = false;
                    checkType.backLeftBrushPutter   = false;
                    checkType.backRightBrushPutter  = false;
                    checkType.entryReadyAreaGate    = false;
                    checkType.entryWorkAreaGate     = false;
                    ret = module_dev_reset(&checkType);         //这里执行有可能来不及返回完成，只是提供一个回零动作
                    if(ret < 0){
                        return ret;
                        LOG_UPLOAD("Top brush reset err ret %d", ret);
                    }
                    else if(RET_COMPLETE == ret){
                        ret = NOR_CONTINUE;
                    }
                    break;
                case PROC_FINISH_FRONT_BRUSH:                   //结束前刷侧边刷洗
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        brush[BRUSH_FRONT_LEFT].runMode     = BRUSH_MANUAL;
                        brush[BRUSH_FRONT_RIGHT].runMode    = BRUSH_MANUAL;
                        brush[BRUSH_FRONT_LEFT].isJogMove   = false;
                        brush[BRUSH_FRONT_RIGHT].isJogMove  = false;
                        conveyor_move(CRL_SECTION_2, CMD_STILL);
                        water_system_control(WATER_WAXWATER, false);    //输送带停止时，暂停喷蜡水
                        water_system_control(WATER_CLEAR_WATER, false);
                        stepSta.isModuleDriverExecuted = false; //重置模型驱动供侧刷换向使用
                        sideBrushWashPos = SIDE_BRUSH_POS_MIDDLE;
                        carWash[i].isWashCarTailFinish = false;
                        carWash[i].isAllChangeBrushRotation = false;
                        carWash[i].isFrontChangeBrushRotation = false;
                    }
                    if(!carWash[i].isAllChangeBrushRotation){
                        bool isDriveExe = false;
                        if(carWash[i].isFrontChangeBrushRotation && !stepSta.isModuleDriverExecuted){
                            isDriveExe = true;
                            stepSta.isModuleDriverExecuted = true;  //前侧刷换向结束之后，不再受理驱动标志（避免洗车尾的驱动标志被这里消耗）
                        }
                        ret = module_change_side_brush_rotation(CMD_STILL);
                        if(ret < 0){
                            return ret;
                            LOG_UPLOAD("Change side brush rotation err ret %d", ret);
                        }
                        else if(1 == ret){
                            carWash[i].isFrontChangeBrushRotation = true;
                            stepSta.isModuleDriverExecuted = false; //重置模型驱动供洗车尾使用
                            ret = NOR_CONTINUE;
                        }
                        else if(RET_COMPLETE == ret){
                            carWash[i].isAllChangeBrushRotation = true;
                            ret = NOR_CONTINUE;
                        }
                        if(isDriveExe)  stepSta.isModuleDriverExecuted = false;     //归还驱动标志
                    }
                    if(!carWash[i].isWashCarTailFinish){
                        if(carWash[i].isFrontChangeBrushRotation){
                            ret = module_side_brush_both_move_to_position(sideBrushWashPos);
                            if(ret < 0){
                                return ret;
                                LOG_UPLOAD("Front side brush move to pos err ret %d", ret);
                            }
                            else if(RET_COMPLETE == ret){
                                if(SIDE_BRUSH_POS_MIDDLE == sideBrushWashPos){
                                    sideBrushWashPos = SIDE_BRUSH_POS_LEFT;
                                    stepSta.isModuleDriverExecuted = false;                 //重置模型驱动
                                }
                                else if(SIDE_BRUSH_POS_LEFT == sideBrushWashPos){
                                    sideBrushWashPos = SIDE_BRUSH_POS_RIGHT;
                                    stepSta.isModuleDriverExecuted = false;                 //重置模型驱动

                                    //有车洗车尾快结束了就开放下一辆车进入
                                    isAllowNextCarInWorkArea = true;
                                    entryCarIndex = 0;                  //允许下一辆车进入后进入车辆的Id编号清零（表示当前无车辆正在进入工作区）
                                }
                                else if(SIDE_BRUSH_POS_RIGHT == sideBrushWashPos){
                                    LOG_UPLOAD("Front side brush wash car tail finish");
                                    stepSta.isModuleDriverExecuted = false;                 //重置模型驱动（用于侧刷归位）
                                    carWash[i].isWashCarTailFinish = true;
                                    carWash[i].isBackBrushFinish = false;
                                    front_side_brush_rotation(CRL_BOTH, CMD_STILL);
                                    water_system_control(WATER_FRONT_SIDE, false);

                                    if(is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)){
                                        isRearEndProtect = false;                       //输送带动作后重新判定是否防追尾保护
                                        conveyor_move(CRL_SECTION_2, CMD_FORWARD);
                                        water_system_control(WATER_WAXWATER, true);     //输送带继续动作后，继续喷蜡水
                                        water_system_control(WATER_CLEAR_WATER, true);
                                    }
                                }
                                ret = NOR_CONTINUE;
                            }
                        }
                    }
                    else if(carWash[i].isAllChangeBrushRotation && carWash[i].isWashCarTailFinish){
                        checkType.lifter                = false;
                        checkType.frontLeftBrushPutter  = true;
                        checkType.frontRightBrushPutter = true;
                        checkType.backLeftBrushPutter   = false;
                        checkType.backRightBrushPutter  = false;
                        checkType.entryReadyAreaGate    = false;
                        checkType.entryWorkAreaGate     = false;
                        ret = module_dev_reset(&checkType);
                        if(ret < 0){
                            return ret;
                            LOG_UPLOAD("Front brush putter reset err ret %d", ret);
                        }
                        //若车辆已经到达完成区域（车辆自己开到了完成区域）则直接跳转到下一流程
                        else if(RET_COMPLETE == ret && !carWash[i].isCarMoveCompleteArea){
                            carWash[i].tailProc = PROC_FINISH_BACK_BRUSH;
                        }
                    }
                    break;
                case PROC_FINISH_BACK_BRUSH:                    //结束后刷侧边刷洗
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        //跟随和点动都提前关闭，避免跟随动作时又马上结束执行回零（这里也要添加，避免车提前离开导致没到位置值就跳转到这里）
                        brush[BRUSH_BACK_LEFT].runMode      = BRUSH_MANUAL;
                        brush[BRUSH_BACK_RIGHT].runMode     = BRUSH_MANUAL;
                        brush[BRUSH_BACK_LEFT].isJogMove    = false;
                        brush[BRUSH_BACK_RIGHT].isJogMove   = false;
                        back_side_brush_rotation(CRL_BOTH, CMD_STILL);
                        water_system_control(WATER_BACK_SIDE, false);
                        stepSta.isModuleDriverExecuted = false; //重置模型驱动（用于侧刷归位）
                    }
                    checkType.lifter                = false;
                    checkType.frontLeftBrushPutter  = false;
                    checkType.frontRightBrushPutter = false;
                    checkType.backLeftBrushPutter   = true;
                    checkType.backRightBrushPutter  = true;
                    checkType.entryReadyAreaGate    = false;
                    checkType.entryWorkAreaGate     = false;
                    ret = module_dev_reset(&checkType);
                    if(ret < 0){
                        return ret;
                        LOG_UPLOAD("Back brush putter reset err ret %d", ret);
                    }
                    else if(RET_COMPLETE == ret){
                        if(!carWash[i].isCarMoveCompleteArea) carWash[i].tailProc = PROC_FINISH_WAXWAT;
                        carWash[i].isBackBrushFinish = true;
                        ret = NOR_CONTINUE;
                    }
                    break;
                case PROC_FINISH_WAXWAT:                            //结束蜡水
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        water_system_control(WATER_WAXWATER, false);
                        water_system_control(WATER_CLEAR_WATER, false);
                    }
                    if(!carWash[i].isCarMoveCompleteArea){
                        carWash[i].tailProc = PROC_FINISH_DYRING;
                    }
                    break;
                case PROC_FINISH_DYRING:                            //结束吹风
                    if(carWash[i].isTailProcChanged){
                        carWash[i].isTailProcChanged = false;
                        // if(1 == washCarNum)  dryer_work(CRL_ALL_SECTION, false);    //有多辆车在工作区时不关闭风机
                        dryer_work(CRL_ALL_SECTION, false);
                        LOG_UPLOAD("Car id %d finish dryer", i);
                    }
                    if(!carWash[i].isCarMoveCompleteArea){
                        headWashCarId = entryCarIndex;              //有车结束后，更换前车Id
                        memset(&carWash[i], 0, sizeof(Type_CarWashInfo_Def));   //完成后清零当前车辆Id存储数据
                        carWash[i].topLifter = INIT_CAR_TOP_POS;
                        LOG_UPLOAD("Car id %d finished, refresh all data", i);
                        *completeId = i;                            //赋值当前完成的车辆Id编号
                        if(washCarNum > 0)  washCarNum--;
                        return RET_COMPLETE;
                    }
                    break;
                default:
                    break;
                }
            }
        }
    }
    return NOR_CONTINUE;
}

/*                                                         ===================                                                         */
/* ========================================================    毛刷跟随控制    ======================================================== */
/*                                                         ===================                                                         */

void side_brush_follow_thread(void *arg)
{
    int16_t lifterPos = 0;
    int16_t brushCurrent = 0;                               //毛刷当前电流
    int16_t brushCurrent_L = 0;                             //毛刷控制下限阈值
    int16_t brushCurrent_H = 0;                             //毛刷控制上限阈值
    Type_DriverCmd_Enum needCmd[BRUSH_NUM] = {0};           //当前需要跟随方向
    Type_DriverCmd_Enum lastNeedCmd[BRUSH_NUM] = {0};
    uint8_t filterCnt[BRUSH_NUM] = {0};
    uint64_t brushMoveTimestamp[BRUSH_NUM] = {0};
    Type_DriverCmd_Enum recordBrushCmdSta[BRUSH_NUM] = {0};
    Type_DriverCmd_Enum conveyorRecordSpeed = CMD_STILL;    //记录输送带设置的速度
    uint8_t limtConveyorLevel[BRUSH_NUM] = {0};             //毛刷限制输送带的速度
    uint8_t pressProtectCnt[BRUSH_NUM] = {0};               //毛刷触压值达到保护值的次数
    uint8_t protectReleaseCnt[BRUSH_NUM] = {0};             //触压保护释放的确认次数
    uint8_t limtConveyorLevelFinal = VELOCITY_NO_LIMIT;     //最终限制输送带的速度，默认最快速度
    uint8_t lastLimitLevel = VELOCITY_NO_LIMIT;
    uint8_t followErrCnt = 0;
    bool isFrontBrushFollow = false;                        //侧刷是否处于跟随状态，基于侧刷同时开启或关闭跟随
    bool isTopBrushFirstBackward = false;

    while(1){
        if(BRUSH_MANUAL == brush[BRUSH_TOP].runMode
        && BRUSH_MANUAL == brush[BRUSH_FRONT_LEFT].runMode && BRUSH_MANUAL == brush[BRUSH_FRONT_RIGHT].runMode
        && BRUSH_MANUAL == brush[BRUSH_BACK_LEFT].runMode && BRUSH_MANUAL == brush[BRUSH_BACK_RIGHT].runMode){
            limtConveyorLevelFinal = VELOCITY_NO_LIMIT;     //取消跟随后，解除输送带速度限制
            lastLimitLevel = limtConveyorLevelFinal;
            followErrCnt = 0;
            isTopBrushFirstBackward = false;
            if(isFrontBrushFollow){
                osal_set_dev_limit_mode(FRONT_LEFT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                osal_set_dev_limit_mode(FRONT_RIGHT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                isFrontBrushFollow = false;
            }
            aos_msleep(300);
            continue;
        }
        else{
            //跟随状态下若无基准电流或检测到毛刷电流为0则报错
            uint8_t i;
            for (i = 0; i < BRUSH_NUM; i++)
            {
                if(BRUSH_MANUAL != brush[i].runMode && (0 == brush[i].baseCurrent || 0 == brush[i].current)){
                    followErrCnt++;
                    break;
                }
            }
            if(i < BRUSH_NUM){
                if(followErrCnt > get_error_overtime(309) / 50){    //50为循环周期
                    set_error_state(309, true);
                    LOG_UPLOAD("Follow err, base currnert T %d, FL %d, FR %d, BL %d, BR %d, now currnert T %d, FL %d, FR %d, BL %d, BR %d",
                    brush[BRUSH_TOP].baseCurrent, brush[BRUSH_FRONT_LEFT].baseCurrent, brush[BRUSH_FRONT_LEFT].baseCurrent,
                    brush[BRUSH_BACK_LEFT].baseCurrent, brush[BRUSH_BACK_RIGHT].baseCurrent,
                    brush[BRUSH_TOP].current, brush[BRUSH_FRONT_LEFT].current, brush[BRUSH_FRONT_LEFT].current,
                    brush[BRUSH_BACK_LEFT].current, brush[BRUSH_BACK_RIGHT].current);
                }
                aos_msleep(50);
                continue;
            }
            else{
                followErrCnt = 0;
            }

            //顶刷不跟随时,首次上升标志清零
            if(BRUSH_MANUAL == brush[BRUSH_TOP].runMode){
                isTopBrushFirstBackward = false;
            }

            //前侧刷跟随状态下，限制正向软限位
            if(brush[BRUSH_FRONT_LEFT].runMode != BRUSH_MANUAL && brush[BRUSH_FRONT_RIGHT].runMode != BRUSH_MANUAL){
                if(!isFrontBrushFollow){
                    isFrontBrushFollow = true;
                    osal_set_dev_limit_mode(FRONT_LEFT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, PUTTER_FOLLOW_MAX_OFFSET);
                    osal_set_dev_limit_mode(FRONT_RIGHT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, PUTTER_FOLLOW_MAX_OFFSET);
                }
            }
            else{
                if(isFrontBrushFollow){
                    isFrontBrushFollow = false;
                    osal_set_dev_limit_mode(FRONT_LEFT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                    osal_set_dev_limit_mode(FRONT_RIGHT_MOVE_MATCH_ID, MODE_SOFT_LIMIT_MAX, 0, PUTTER_GO_ANOTHER_SIDE_OFFSET);
                }
            }

            lifterPos = xp_osal_get_dev_pos(LIFTER_MATCH_ID);
            bool isRearviewMirrors = false;
            for (uint8_t i = 0; i < SUPPORT_WASH_NUM_MAX; i++)
            {
                if(carWash[i].headWindowsOffsetHead != 0
                && carWash[i].headOffsetPos > washProcPos.startFrontBrush + carWash[i].headWindowsOffsetHead + 90
                && carWash[i].headOffsetPos < washProcPos.startFrontBrush + carWash[i].headWindowsOffsetHead + 170){
                    isRearviewMirrors = true;
                    break;
                }
            }
            
            //判断当前毛刷需要动作的方向
            for (uint8_t i = 0; i < BRUSH_NUM; i++)
            {
                if(BRUSH_MANUAL == brush[i].runMode)    continue;
                switch (i)
                {
                case BRUSH_TOP:
                    brushCurrent_L   = (BRUSH_FOLLOW_NO_BACKWARD == brush[i].runMode) ? brush[i].pressL_NoBW : brush[i].pressL;
                    brushCurrent_H   = (BRUSH_FOLLOW_NO_FORWARD == brush[i].runMode) ? brush[i].pressH_NoFW : brush[i].pressH;
                    //顶刷高度较高时，电流上下限值需增加毛刷打到结构件的偏移电流值
                    if(lifterPos < TOP_BRUSH_RECORD_CUR_AREA){
                        brushCurrent_L += (topBrushPosCurrent[lifterPos] - brush[i].baseCurrent);
                        brushCurrent_H += (topBrushPosCurrent[lifterPos] - brush[i].baseCurrent);
                    }
                    break;
                case BRUSH_FRONT_LEFT:
                case BRUSH_FRONT_RIGHT:
                    //前侧刷侧重洗车窗，在后视镜的地方压力阈值同后侧刷（刷洗压力小一点）
                    if(BRUSH_FREE_FOLLOW == brush[i].runMode && !isRearviewMirrors){
                        brushCurrent_L = brush[i].pressL;
                        brushCurrent_H = brush[i].pressH;
                    }
                    else{
                        brushCurrent_L = (BRUSH_FOLLOW_NO_FORWARD == brush[i].runMode) ? brush[i].pressL      : brush[BRUSH_BACK_LEFT].pressL;
                        brushCurrent_H = (BRUSH_FOLLOW_NO_FORWARD == brush[i].runMode) ? brush[i].pressH_NoFW : brush[BRUSH_BACK_LEFT].pressH;
                    }
                    break;
                case BRUSH_BACK_LEFT:
                case BRUSH_BACK_RIGHT:
                    brushCurrent_L = brush[i].pressL;
                    if(BRUSH_FOLLOW_NO_FORWARD == brush[i].runMode) brushCurrent_H = brush[i].pressH_NoFW;
                    else                                            brushCurrent_H = brush[i].pressH;
                    break;
                default:
                    break;
                }
                
                brushCurrent = brush[i].current;
                if(brushCurrent > brushCurrent_H)			needCmd[i] = CMD_BACKWARD;
                else if(brushCurrent < brushCurrent_L)		needCmd[i] = CMD_FORWARD;
                else										needCmd[i] = CMD_STILL;

                filterCnt[i]++;
                if(lastNeedCmd[i] != needCmd[i]){
                    lastNeedCmd[i] = needCmd[i];
                    filterCnt[i] = 0;               //跟随方向改变，滤波值清零
                }

                // if(BRUSH_TOP == i){
                //     LOG_DEBUG("------current %d need cmd %d", brushCurrent, needCmd[i]);
                // }
            }

            //顶刷跟随
            if(brush[BRUSH_TOP].runMode != BRUSH_MANUAL){
                //顶刷首次下降时，检测压力合适增加检测次数，避免在下降中途没碰到车就停了
                uint8_t brushCurrentFitJudgeCnt = DECISION_STILL_CMD_CNT;
                if(!isTopBrushFirstBackward){
                    brushCurrentFitJudgeCnt = 5;
                }
                //记录跟随期间顶刷的最高位置
                lifterPos = xp_osal_get_dev_pos(LIFTER_MATCH_ID);
                //顶刷跟随时一直刷新车顶位置（在顶刷出现上升后再做记录，避免顶刷没有降到底提前开启跟随的情况（如车辆闯入））
                if(isTopBrushFirstBackward && lifterPos < carWash[entryCarIndex].topLifter){
                    carWash[entryCarIndex].topLifter = lifterPos;
                    LOG_INFO("Car top lifter refresh to %d", carWash[entryCarIndex].topLifter);
                }
                //顶刷跟随时一直记录升降和输送带的位置信息（ CAR_POS_RECORD_INFO_ACCURACY 太小，线程循环时间太长，会导致某些数组值没有填充到）
                if(carWash[entryCarIndex].headOffsetPos > SIGNAL_ENTRANCE_TO_TOP_BRUSH_OFFSET 
                && carWash[entryCarIndex].headOffsetPos < CAR_MAX_LENGTH + SIGNAL_ENTRANCE_TO_TOP_BRUSH_OFFSET){
                    recordAreaPos[carWash[entryCarIndex].headOffsetPos / CAR_POS_RECORD_INFO_ACCURACY] = carWash[entryCarIndex].headOffsetPos;
                    recordLifterPos[carWash[entryCarIndex].headOffsetPos / CAR_POS_RECORD_INFO_ACCURACY] = lifterPos;
                }

                if(CMD_BACKWARD == needCmd[BRUSH_TOP] && filterCnt[BRUSH_TOP] > (300/BRUSH_FOLLOW_THREAD_FREQ - 3)){
                    isTopBrushFirstBackward = true;
                    if(brush[BRUSH_TOP].current > brush[BRUSH_TOP].pressProtect){
                        brush[BRUSH_TOP].isPressProtectMove = true;
                        recordBrushCmdSta[BRUSH_TOP] = CMD_BACKWARD;
                        brushMoveTimestamp[BRUSH_TOP] = aos_now_ms();
                        filterCnt[BRUSH_TOP] = 0;
                        lifter_move_time(CMD_BACKWARD, (BRUSH_FREE_FOLLOW == brush[BRUSH_TOP].runMode) ? 300 : 350);
                        LOG_INFO("Top current %d is high to protect, backward", brush[BRUSH_TOP].current);
                    }
                    else if(brush[BRUSH_TOP].runMode != BRUSH_FOLLOW_NO_BACKWARD && filterCnt[BRUSH_TOP] > DECISION_BACKWARD_CMD_CNT
                    && get_diff_ms(brushMoveTimestamp[BRUSH_TOP]) > 1000){     //限制执行的间隔时间
                        brush[BRUSH_TOP].isPressProtectMove = false;
                        recordBrushCmdSta[BRUSH_TOP] = CMD_BACKWARD;
                        brushMoveTimestamp[BRUSH_TOP] = aos_now_ms();
                        filterCnt[BRUSH_TOP] = 0;
                        brush[BRUSH_TOP].isJogMove ? lifter_move_time(CMD_BACKWARD, (BRUSH_FOLLOW_NO_FORWARD == brush[BRUSH_TOP].runMode) ? 500 : 300) \
                        : lifter_move_time(CMD_BACKWARD, 800);
                        // LOG_INFO("Top current %d is high, backward", brush[BRUSH_TOP].current);
                        LOG_INFO("Top current %d is high, base %d, PL %d, PH %d, backward", \
                        brush[BRUSH_TOP].current, brush[BRUSH_TOP].baseCurrent, brush[BRUSH_TOP].pressL, brush[BRUSH_TOP].pressH);
                    }
                }
                else if(brush[BRUSH_TOP].runMode != BRUSH_FOLLOW_NO_FORWARD
                && CMD_FORWARD == needCmd[BRUSH_TOP] && filterCnt[BRUSH_TOP] > (300/BRUSH_FOLLOW_THREAD_FREQ - 1)){
                    //触压过小下降的判定时间小于单次移动的时间，避免动作过程中一动一停，保持连续性
                    if(brush[BRUSH_TOP].current < brush[BRUSH_TOP].pressTouchcar){
                        recordBrushCmdSta[BRUSH_TOP] = CMD_FORWARD;
                        brushMoveTimestamp[BRUSH_TOP] = aos_now_ms(); 
                        filterCnt[BRUSH_TOP] = 0;
                        brush[BRUSH_TOP].isJogMove ? \
                        lifter_move_time(CMD_FORWARD, (brush[BRUSH_TOP].isPressProtectMove) ? 200 : 300) : lifter_move(CMD_FORWARD, true);
                        LOG_INFO("Top current %d is too low, forward", brush[BRUSH_TOP].current);
                        brush[BRUSH_TOP].isPressProtectMove = false;
                    }
                    else if(get_diff_ms(brushMoveTimestamp[BRUSH_TOP]) > 1000 && filterCnt[BRUSH_TOP] > DECISION_FORWARD_CMD_CNT){
                        brush[BRUSH_TOP].isPressProtectMove = false;
                        recordBrushCmdSta[BRUSH_TOP] = CMD_FORWARD;
                        brushMoveTimestamp[BRUSH_TOP] = aos_now_ms();
                        filterCnt[BRUSH_TOP] = 0;
                        brush[BRUSH_TOP].isJogMove ? lifter_move_time(CMD_FORWARD, (BRUSH_FOLLOW_NO_BACKWARD == brush[BRUSH_TOP].runMode) ? 400 : 300) : lifter_move(CMD_FORWARD, true);
                        LOG_INFO("Top current %d is low, forward", brush[BRUSH_TOP].current);
                    }
                }
                else if(CMD_STILL == needCmd[BRUSH_TOP] && filterCnt[BRUSH_TOP] >= brushCurrentFitJudgeCnt
                && recordBrushCmdSta[BRUSH_TOP] != CMD_STILL){
                    recordBrushCmdSta[BRUSH_TOP] = CMD_STILL;
                    filterCnt[BRUSH_TOP] = 0;
                    lifter_move(CMD_STILL, true);
                    LOG_INFO("Top current is fit, lifter still");
                }
                
                //输送带移动速度受限于毛刷电流（升降没到顶时限速，到顶的话靠触压异常值保护）
                if(brush[BRUSH_TOP].current > brush[BRUSH_TOP].pressProtect && !is_signal_filter_trigger(SIGNAL_LIFTER_UP)){
                    if(pressProtectCnt[BRUSH_TOP] > PRESS_PROTECT_COMFIRM_CNT){
                        limtConveyorLevel[BRUSH_TOP] = VELOCITY_LIMIT_LEVEL_STOP;
                        protectReleaseCnt[BRUSH_TOP] = PRESS_PROTECT_REEASE_CNT;
                    }
                    else{                                   //未达到确认次数，不进行限速
                        pressProtectCnt[BRUSH_TOP]++;
                        protectReleaseCnt[BRUSH_TOP] = 0;
                        limtConveyorLevel[BRUSH_TOP] = VELOCITY_NO_LIMIT;
                    }
                }
                else{
                    if(protectReleaseCnt[BRUSH_TOP] > 0){   //未确认释放时，还是限速
                        protectReleaseCnt[BRUSH_TOP]--;
                        limtConveyorLevel[BRUSH_TOP] = VELOCITY_LIMIT_LEVEL_STOP;
                    }
                    else{
                        pressProtectCnt[BRUSH_TOP] = 0;
                        limtConveyorLevel[BRUSH_TOP] = VELOCITY_NO_LIMIT;
                    }
                }
            }
            else{
                limtConveyorLevel[BRUSH_TOP] = VELOCITY_NO_LIMIT;
            }
            //前后侧刷跟随
            for (uint8_t i = 1; i < BRUSH_NUM; i++)
            {
                bool isBrushCrooked = false;
                switch (i)
                {
                case BRUSH_FRONT_LEFT:  isBrushCrooked = is_signal_filter_trigger(SIGNAL_FL_BRUSH_CROOKED) ? true : false; break;
                case BRUSH_FRONT_RIGHT: isBrushCrooked = is_signal_filter_trigger(SIGNAL_FR_BRUSH_CROOKED) ? true : false; break;
                case BRUSH_BACK_LEFT:   isBrushCrooked = is_signal_filter_trigger(SIGNAL_BL_BRUSH_CROOKED) ? true : false; break;
                case BRUSH_BACK_RIGHT:  isBrushCrooked = is_signal_filter_trigger(SIGNAL_BR_BRUSH_CROOKED) ? true : false; break;
                default:
                    break;
                }
                if(brush[i].runMode != BRUSH_MANUAL){
                    if((CMD_BACKWARD == needCmd[i] && filterCnt[i] > DECISION_BACKWARD_CMD_CNT)
                    || isBrushCrooked){
                        recordBrushCmdSta[i] = needCmd[i];
                        if(get_diff_ms(brushMoveTimestamp[i]) > 300 && (brush[i].current > brush[i].pressProtect || isBrushCrooked)){
                            brushMoveTimestamp[i] = aos_now_ms();
                            filterCnt[i] = 0;
                            switch (i)
                            {
                            case BRUSH_FRONT_LEFT:  front_side_brush_move_time(CRL_ONLY_LEFT,  CMD_BACKWARD, 500); break;
                            case BRUSH_FRONT_RIGHT: front_side_brush_move_time(CRL_ONLY_RIGHT, CMD_BACKWARD, 500); break;
                            case BRUSH_BACK_LEFT:   back_side_brush_move_time(CRL_ONLY_LEFT,   CMD_BACKWARD, 500); break;
                            case BRUSH_BACK_RIGHT:  back_side_brush_move_time(CRL_ONLY_RIGHT,  CMD_BACKWARD, 500); break;
                            default:
                                break;
                            }
                            LOG_INFO("%s current %d is high to protect, backward. crooked flag %d", xp_get_brush_str(i), brush[i].current, isBrushCrooked);
                        }
                        else if(get_diff_ms(brushMoveTimestamp[i]) > 1000){     //限制执行的间隔时间
                            brushMoveTimestamp[i] = aos_now_ms();
                            filterCnt[i] = 0;
                            switch (i)
                            {
                            case BRUSH_FRONT_LEFT:  front_side_brush_move_time(CRL_ONLY_LEFT,  CMD_BACKWARD, brush[i].isJogMove ? 400 : 500); break;
                            case BRUSH_FRONT_RIGHT: front_side_brush_move_time(CRL_ONLY_RIGHT, CMD_BACKWARD, brush[i].isJogMove ? 400 : 500); break;
                            case BRUSH_BACK_LEFT:   back_side_brush_move_time(CRL_ONLY_LEFT,   CMD_BACKWARD, brush[i].isJogMove ? 400 : 500); break;
                            case BRUSH_BACK_RIGHT:  back_side_brush_move_time(CRL_ONLY_RIGHT,  CMD_BACKWARD, brush[i].isJogMove ? 400 : 500); break;
                            default:
                                break;
                            }
                            LOG_INFO("%s current %d is high, backward, PL %d, PH %d", xp_get_brush_str(i), brush[i].current, brushCurrent_L, brushCurrent_H);
                        }
                    }
                    else if(CMD_FORWARD == needCmd[i] && filterCnt[i] > DECISION_FORWARD_CMD_CNT
                    && get_diff_ms(brushMoveTimestamp[i]) > 1000){
                        recordBrushCmdSta[i] = needCmd[i];
                        // 侧刷低位作为限位信号会在osal中进行判定停止
                        // bool isBrushDownTrig = true;
                        // switch (i)
                        // {
                        // case BRUSH_FRONT_LEFT:  isBrushDownTrig = (is_signal_filter_trigger(SIGNAL_FL_BRUSH_DOWN)) ? true : false; break;
                        // case BRUSH_FRONT_RIGHT: isBrushDownTrig = (is_signal_filter_trigger(SIGNAL_FR_BRUSH_DOWN)) ? true : false; break;
                        // case BRUSH_BACK_LEFT:   isBrushDownTrig = (is_signal_filter_trigger(SIGNAL_BL_BRUSH_DOWN)) ? true : false; break;
                        // case BRUSH_BACK_RIGHT:  isBrushDownTrig = (is_signal_filter_trigger(SIGNAL_BR_BRUSH_DOWN)) ? true : false; break;
                        // default:
                        //     break;
                        // }
                        // if(isBrushDownTrig){
                        //     LOG_INFO("%s brush down trig, refuse forward", xp_get_brush_str(i));
                        // }
                        // else 
                        if(brush[i].runMode != BRUSH_FOLLOW_NO_FORWARD){
                            brushMoveTimestamp[i] = aos_now_ms();
                            filterCnt[i] = 0;
                            switch (i)
                            {
                            case BRUSH_FRONT_LEFT:  front_side_brush_move_time(CRL_ONLY_LEFT,  CMD_FORWARD, brush[i].isJogMove ? 300 : 700); break;
                            case BRUSH_FRONT_RIGHT: front_side_brush_move_time(CRL_ONLY_RIGHT, CMD_FORWARD, brush[i].isJogMove ? 300 : 700); break;
                            case BRUSH_BACK_LEFT:   back_side_brush_move_time(CRL_ONLY_LEFT,   CMD_FORWARD, brush[i].isJogMove ? 300 : 700); break;
                            case BRUSH_BACK_RIGHT:  back_side_brush_move_time(CRL_ONLY_RIGHT,  CMD_FORWARD, brush[i].isJogMove ? 300 : 700); break;
                            default:
                                break;
                            }
                            LOG_INFO("%s current %d is low, forward, PL %d, PH %d", xp_get_brush_str(i), brush[i].current, brushCurrent_L, brushCurrent_H);
                        }
                    }
                    else if(CMD_STILL == needCmd[i] && filterCnt[i] > DECISION_STILL_CMD_CNT
                    && recordBrushCmdSta[i] != CMD_STILL){
                        recordBrushCmdSta[i] = CMD_STILL;
                        filterCnt[i] = 0;
                        switch (i)
                        {
                        case BRUSH_FRONT_LEFT:  front_side_brush_move_time(CRL_ONLY_LEFT,  CMD_STILL, true); break;
                        case BRUSH_FRONT_RIGHT: front_side_brush_move_time(CRL_ONLY_RIGHT, CMD_STILL, true); break;
                        case BRUSH_BACK_LEFT:   back_side_brush_move_time(CRL_ONLY_LEFT,   CMD_STILL, true); break;
                        case BRUSH_BACK_RIGHT:  back_side_brush_move_time(CRL_ONLY_RIGHT,  CMD_STILL, true); break;
                        default:
                            break;
                        }
                        LOG_INFO("%s current is fit, still", xp_get_brush_str(i));
                    }
                    
                    //输送带移动速度受限于毛刷电流
                    uint8_t putterMatchId = 0;
                    switch (i)
                    {
                    case BRUSH_FRONT_LEFT:  putterMatchId = FRONT_LEFT_MOVE_MATCH_ID; break;
                    case BRUSH_FRONT_RIGHT: putterMatchId = FRONT_RIGHT_MOVE_MATCH_ID; break;
                    case BRUSH_BACK_LEFT:   putterMatchId = BACK_LEFT_MOVE_MATCH_ID; break;
                    case BRUSH_BACK_RIGHT:  putterMatchId = BACK_RIGHT_MOVE_MATCH_ID; break;
                    default:
                        break;
                    }
                    if(brush[i].current > brush[i].pressProtect 
                    && xp_osal_get_dev_pos(putterMatchId) > SIDE_BRUSH_OPEN_POS + MOVE_POS_ERR){
                        if(pressProtectCnt[i] > PRESS_PROTECT_COMFIRM_CNT){
                            limtConveyorLevel[i] = VELOCITY_LIMIT_LEVEL_STOP;
                            protectReleaseCnt[i] = PRESS_PROTECT_REEASE_CNT;
                        }
                        else{                           //未达到确认次数，不进行限速
                            pressProtectCnt[i]++;
                            protectReleaseCnt[i] = 0;
                            limtConveyorLevel[i] = VELOCITY_NO_LIMIT;
                        }
                    }
                    else{
                        if(protectReleaseCnt[i] > 0){   //未确认释放时，还是限速
                            protectReleaseCnt[i]--;
                            limtConveyorLevel[i] = VELOCITY_LIMIT_LEVEL_STOP;
                        }
                        else{
                            pressProtectCnt[i] = 0;
                            limtConveyorLevel[i] = VELOCITY_NO_LIMIT;
                        }
                    }
                }
                else{
                    limtConveyorLevel[i] = VELOCITY_NO_LIMIT;
                }
            }

            /* 输送带2#不受触压值保护限速 */
            /* //查找每个毛刷限制的速度，取最小值
            limtConveyorLevelFinal = VELOCITY_NO_LIMIT;
            for (uint8_t i = 0; i < BRUSH_NUM; i++)
            {
                if(limtConveyorLevel[i] < limtConveyorLevelFinal){
                    limtConveyorLevelFinal = limtConveyorLevel[i];
                }
            }

            if(lastLimitLevel != limtConveyorLevelFinal){     //限制速度调整后，重新配置输送带速度
                if(0 == limtConveyorLevelFinal){
                    if(priSwCmd[PRI_SWITCH_CONVEYOR_2].cmd != 0){
                        conveyorRecordSpeed = priSwCmd[PRI_SWITCH_CONVEYOR_2].cmd;    //记录当前输送带的设置速度（不一定是真实速度）
                    }
                    LOG_UPLOAD("brush current T %d, FL %d, FR %d, BL %d, BR %d, touch protect value, stop conveyor move, record speed %d", 
                    brush[BRUSH_TOP].current, brush[BRUSH_FRONT_LEFT].current, brush[BRUSH_FRONT_RIGHT].current,
                    brush[BRUSH_BACK_LEFT].current, brush[BRUSH_BACK_RIGHT].current, conveyorRecordSpeed);
                    conveyor_move(CRL_SECTION_2, CMD_STILL);
                }
                else if(abs(conveyorRecordSpeed) < limtConveyorLevelFinal){
                    LOG_UPLOAD("recover conveyor speed %d", conveyorRecordSpeed);
                    conveyor_move(CRL_SECTION_2, conveyorRecordSpeed);
                }
                else{
                    LOG_UPLOAD("gantry speed limit to %d", limtConveyorLevelFinal);
                    conveyorRecordSpeed > 0 ? conveyor_move(CRL_SECTION_2, limtConveyorLevelFinal) : conveyor_move(CRL_SECTION_2, -limtConveyorLevelFinal);
                }
                lastLimitLevel = limtConveyorLevelFinal;
            } */
        }
        aos_msleep(BRUSH_FOLLOW_THREAD_FREQ);
    }
}

/*                                                         =======================                                                         */
/* ========================================================      参数读取更新      ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       毛刷电流值更新
 * @param[in]	arg                 
 */
void brush_current_update_thread(void *arg)
{
    uint8_t logCnt = 0;

    while (1)
    {
        // uint64_t timeStamp = aos_now_ms();
        for (uint8_t i = 0; i < BRUSH_NUM; i++)
        {
            int16_t current = 0;
            uint8_t brushMatchId;
            if(0 == i)      brushMatchId = TOP_BRUSH_MATCH_ID;
            else if(1 == i) brushMatchId = FRONT_LEFT_BRUSH_MATCH_ID;
            else if(2 == i) brushMatchId = FRONT_RIGHT_BRUSH_MATCH_ID;
            else if(3 == i) brushMatchId = BACK_LEFT_BRUSH_MATCH_ID;
            else if(4 == i) brushMatchId = BACK_RIGHT_BRUSH_MATCH_ID;
            else            brushMatchId = TOP_BRUSH_MATCH_ID;
            
            current = xp_ag_osal_get()->freq.getCurrent(brushMatchId);
            brush[i].current = (!is_dev_move_sta_idle(brushMatchId) && get_diff_ms(cmdExeSta[brushMatchId].cmdStartTimeStamp) > 1800
                            && current > 0) ? current : 0;
            aos_msleep(1);
        }
        // uint64_t diffTime = get_diff_ms(timeStamp);
        
        if(logCnt++ % 50 == 0){
            // LOG_DEBUG("========Current get freq %lld", diffTime);
            // LOG_DEBUG("----Current TOP %d, FL %d, FR %d, BL %d, BR %d", 
            // brush[BRUSH_TOP].current, brush[BRUSH_FRONT_LEFT].current, brush[BRUSH_FRONT_RIGHT].current, brush[BRUSH_BACK_LEFT].current, brush[BRUSH_BACK_RIGHT].current);
        }

        /* 毛刷电流值过大报警 */
        uint16_t errCodeIndex = 0;
        bool isLimitTrigger = false;
        for (uint8_t i = 0; i < BRUSH_NUM; i++)
        {
            switch (i)
            {
            case BRUSH_TOP:
                errCodeIndex = 8108;
                isLimitTrigger = is_signal_filter_trigger(SIGNAL_LIFTER_UP);
                break;
            case BRUSH_FRONT_LEFT:
                errCodeIndex = 8119;
                isLimitTrigger = is_signal_filter_trigger(SIGNAL_FL_MOVE_ZERO);
                break;
            case BRUSH_FRONT_RIGHT:
                errCodeIndex = 8120;
                isLimitTrigger = is_signal_filter_trigger(SIGNAL_FR_MOVE_ZERO);
                break;
            case BRUSH_BACK_LEFT:
                errCodeIndex = 8121;
                isLimitTrigger = is_signal_filter_trigger(SIGNAL_BL_MOVE_ZERO);
                break;
            case BRUSH_BACK_RIGHT:
                errCodeIndex = 8122;
                isLimitTrigger = is_signal_filter_trigger(SIGNAL_BR_MOVE_ZERO);
                break;
            default:
                break;
            }
            if(brush[i].current > brush[i].pressWarning){
                brush[i].pressTooHighCnt++;
                if((brush[i].pressTooHighCnt > 1500 / BRUSH_CURRENT_UPDATE_THREAD_FREQ && isLimitTrigger)
                || (brush[i].pressTooHighCnt > get_error_overtime(errCodeIndex) / BRUSH_CURRENT_UPDATE_THREAD_FREQ)){
                    set_error_state(errCodeIndex, true);
                    LOG_UPLOAD("%s over press, current %d baseCurrent %d L %d H %d", 
                    xp_get_brush_str(i), brush[i].current, brush[i].baseCurrent, brush[i].pressL, brush[i].pressH);
                }
            }
            else{
                brush[i].pressTooHighCnt = 0;
            }
        }
        aos_msleep(BRUSH_CURRENT_UPDATE_THREAD_FREQ);
    }
}

/*                                                         ======================                                                         */
/* ========================================================    公/私有开关控制    ======================================================== */
/*                                                         ======================                                                         */

/**
 * @brief       判断公有开关是否允许驱动（检查开关切换到位引脚是否反馈完成，且共用的其它设备开关处于断开状态）
 * @param[in]	point               公有开关表格首地址
 * @param[in]	swNum               切换的开关序号
 * @return      bool                
 */
static bool is_com_sw_driver_valid(Type_SwRegisterInfo_Def const *point, Type_ComSwitchSta_Enum swNum)
{
    bool result = true;

    for (uint8_t i = 0; i < COM_SWITCH_SHAR_NUM; i++)
    {
        point += i;
        if(point != NULL){
            if(swNum == i){
                result &= xp_io_read_input_pin(BOARD_ID_RESOLUTION(point->ioIndexSwDone), PIN_ID_RESOLUTION(point->ioIndexSwDone)) ? true : false;
            }
            else{
                result &= xp_io_read_input_pin(BOARD_ID_RESOLUTION(point->ioIndexSwDone), PIN_ID_RESOLUTION(point->ioIndexSwDone)) ? false : true;
            }
        }
        else{
            LOG_UPLOAD("Illegal switch info point");
        }
    }
    
    return result;
}

/**
 * @brief       私有/共有开关切换及驱动控制线程
 * @param[in]	arg
 */
void switch_driver_control_thread(void* arg)
{
    uint8_t i,j;
    Type_SwRegisterInfo_Def *pSwInfo = NULL;
    // Type_SwCrlInfo_Def ComSwCrl_Table[CRL_COM_SW_NUM];
    Type_SwCrlInfo_Def PriSwCrl_Table[CRL_PRI_SW_NUM];

    //公有开关变量初始化
    // for (i = 0; i < CRL_COM_SW_NUM; i++)
    // {
    //     ComSwCrl_Table[i].isSwitch      = false;
    //     ComSwCrl_Table[i].comSwitchSta  = COM_SWITCH_TO_EMPTY;
    //     ComSwCrl_Table[i].info          = &Com_Switch_Table[i][0];     //指针务必指向表格中对应的公有开关首地址
    // }
    //私有开关变量初始化
    for (i = 0; i < CRL_PRI_SW_NUM; i++)
    {
        PriSwCrl_Table[i].isSwitch    = false;
        PriSwCrl_Table[i].info        = &Pri_Switch_Table[i];
    }

    while (1) {
        // for (i = 0; i < CRL_COM_SW_NUM; i++)     //轮询公有开关的状态切换和驱动
        // {
        //     //指令赋值到实际输出有一段时间，用 isCmdMoveGapTime 标志作为当前处于该时间段
        //     for (j = 0; j < COM_SWITCH_SHAR_NUM; j++)
        //     {
        //         if(cmdExeSta[Com_Switch_Table[i][j].osalMatchId].isCmdMoveGapTime 
        //         && get_diff_ms(cmdExeSta[Com_Switch_Table[i][j].osalMatchId].cmdStartTimeStamp) > SWITCH_DELAY_TIME_MS + 500){
        //             cmdExeSta[Com_Switch_Table[i][j].osalMatchId].isCmdMoveGapTime = false;
        //         }
        //     }
        //     //                                          ===================                                          
        //     // =========================================    公有开关切换    =========================================
        //     //                                          ===================                                          
        //     if(ComSwCrl_Table[i].comSwitchSta != comSwSta[i]           //共用开关状态发生改变时进入切换等待状态
        //     || (comSwCmd[i].isCmd && CMD_STILL == comSwCmd[i].cmd)){
        //         for (j = 0; j < COM_SWITCH_SHAR_NUM; j++)
        //         {
        //             pSwInfo = ComSwCrl_Table[i].info + j;       //指向公有开关对应共享设备在注册表格中的地址
        //             if(ComSwCrl_Table[i].comSwitchSta != comSwSta[i]){
        //                 ComSwCrl_Table[i].comSwitchSta = comSwSta[i];
        //                 LOG_INFO("ComSwitch %d switch to %d", i, ComSwCrl_Table[i].comSwitchSta);

        //                 if(COM_SWITCH_TO_EMPTY != ComSwCrl_Table[i].comSwitchSta){
        //                     ComSwCrl_Table[i].isSwitch = true;
        //                     ComSwCrl_Table[i].switchStartT = aos_now_ms();
        //                 }
        //                 xp_io_write_pin(BOARD_ID_RESOLUTION(pSwInfo->ioIndexSw), PIN_ID_RESOLUTION(pSwInfo->ioIndexSw), IO_DISABLE);   //切断共享的设备链接（切换开关断开）
        //             }
        //             switch (pSwInfo->actionType)                //设备输出状态停止
        //             {
        //             case ACTION_MOVE:
        //             case ACTION_FORCE_MOVE:
        //             case ACTION_MOVE_POSE:
        //             case ACTION_MOVE_TIME:
        //                 xp_osal_move_stop(pSwInfo->osalMatchId);
        //                 break;
        //             case ACTION_ROTATION:
        //                 xp_osal_brush_rotation(pSwInfo->osalMatchId, 0);
        //                 break;
        //             default:
        //                 break;
        //             }
        //         }
        //         if(CMD_STILL == comSwCmd[i].cmd)    comSwCmd[i].isCmd = false;      //如果是停止指令，指令执行到此结束
        //     }

        //     //公有开关置空一定时间后，接通相应的设备（切换到空状态不处理）
        //     if (ComSwCrl_Table[i].isSwitch && get_diff_ms(ComSwCrl_Table[i].switchStartT) > SWITCH_DELAY_TIME_MS) {
        //         if(ComSwCrl_Table[i].comSwitchSta >= COM_SWITCH_SHAR_NUM && ComSwCrl_Table[i].comSwitchSta != COM_SWITCH_TO_EMPTY){ //防止指针溢出
        //             goto Err_01;
        //         }
        //         pSwInfo = ComSwCrl_Table[i].info + ComSwCrl_Table[i].comSwitchSta; //指向公有开关对应共享设备在注册表格中的地址
        //         xp_io_write_pin(BOARD_ID_RESOLUTION(pSwInfo->ioIndexSw), PIN_ID_RESOLUTION(pSwInfo->ioIndexSw), IO_ENABLE);
        //         ComSwCrl_Table[i].isSwitch = false;
        //     }

        //     //                                          ===================                                          
        //     // =========================================    公有开关驱动    =========================================
        //     //                                          ===================                                          
            
        //     //公有开关切换完成后若需要执行动作命令，则等接触器吸合后执行驱动
        //     if(!ComSwCrl_Table[i].isSwitch && comSwCmd[i].isCmd && comSwCmd[i].cmd != CMD_STILL \
        //     && get_diff_ms(ComSwCrl_Table[i].switchStartT) > SWITCH_DELAY_TIME_MS + 200)
        //     {
        //         if(ComSwCrl_Table[i].comSwitchSta >= COM_SWITCH_SHAR_NUM && ComSwCrl_Table[i].comSwitchSta != COM_SWITCH_TO_EMPTY){ //防止指针溢出
        //             goto Err_01;
        //         }
        //         pSwInfo = ComSwCrl_Table[i].info;       //指向对应开关表格的首地址
        //         if(is_com_sw_driver_valid(pSwInfo, ComSwCrl_Table[i].comSwitchSta)){
        //             switch (pSwInfo->actionType)        //根据当前开关切换状态的运动类型驱动
        //             {
        //                 case ACTION_MOVE:       xp_osal_move_run(pSwInfo->osalMatchId, comSwCmd[i].cmd); break;
        //                 case ACTION_FORCE_MOVE: xp_osal_force_move_run(pSwInfo->osalMatchId, comSwCmd[i].cmd, DEV_FORCE_MOVE_TIME); break;
        //                 case ACTION_MOVE_POSE:  xp_osal_move_pos(pSwInfo->osalMatchId, comSwCmd[i].cmd, comSwCmd[i].pos); break;
        //                 case ACTION_MOVE_TIME:  xp_osal_move_time(pSwInfo->osalMatchId, comSwCmd[i].cmd, comSwCmd[i].time); break;
        //                 case ACTION_ROTATION:   xp_osal_brush_rotation(pSwInfo->osalMatchId, comSwCmd[i].cmd); break;
        //                 default: break;
        //             }
        //         }else{
        //             // set_error_state( ,true);
        //             LOG_WARN("Com switch not in right driver %d", ComSwCrl_Table[i].comSwitchSta);
        //         }
        //         comSwCmd[i].isCmd = false;
        //     }
        // }

        /*                                          ===================                                          */
        /* =========================================    私有开关驱动    ========================================= */
        /*                                          ===================                                          */
        
        for (i = 0; i < CRL_PRI_SW_NUM; i++)        //轮询所有私有开关，状态有改变则执行驱动
        {
            pSwInfo = PriSwCrl_Table[i].info;

            //指令赋值到实际输出有一段时间，用 isCmdMoveGapTime 标志作为当前处于该时间段
            if(cmdExeSta[pSwInfo->osalMatchId].isCmdMoveGapTime 
            && get_diff_ms(cmdExeSta[pSwInfo->osalMatchId].cmdStartTimeStamp) > SWITCH_DELAY_TIME_MS + 500){
                cmdExeSta[pSwInfo->osalMatchId].isCmdMoveGapTime = false;
            }

            if(priSwCmd[i].isCmd){
                priSwCmd[i].isCmd = false;

                if(priSwCmd[i].cmd == CMD_STILL
                || (priSwCmd[i].cmd > 0 && priSwCmd[i].lastCmd < 0)
                || (priSwCmd[i].cmd < 0 && priSwCmd[i].lastCmd > 0)){     //指令方向不同或停止指令时，需先切断所有相关输出口
                    switch (pSwInfo->actionType)
                    {
                    case ACTION_MOVE:
                    case ACTION_FORCE_MOVE:
                    case ACTION_MOVE_POSE:
                    case ACTION_MOVE_TIME:
                        xp_osal_move_stop(pSwInfo->osalMatchId);
                        break;
                    case ACTION_ROTATION:
                        xp_osal_brush_rotation(pSwInfo->osalMatchId, 0);
                        break;
                    default:
                        break;
                    }

                    //非停止指令则调用驱动
                    if(CMD_STILL != priSwCmd[i].cmd){
                        PriSwCrl_Table[i].isSwitch = true;
                    }

                    //指令发生切换时，重新计切换时间
                    if(priSwCmd[i].cmd != priSwCmd[i].lastCmd){
                        PriSwCrl_Table[i].switchStartT = aos_now_ms();
                    }
                    priSwCmd[i].lastCmd = priSwCmd[i].cmd;
                }
                else{       //方向相同时直接更新速度
                    PriSwCrl_Table[i].isSwitch = true;
                    PriSwCrl_Table[i].switchStartT = aos_now_ms() + SWITCH_DELAY_TIME_MS;  //无等待切换时间
                }
            }

            //这里的等待时间长于osal中状态机轮询等待时间，保证osal中状态机做了一次轮询
            if (PriSwCrl_Table[i].isSwitch && get_diff_ms(PriSwCrl_Table[i].switchStartT) >= SWITCH_DELAY_TIME_MS) {
                switch (pSwInfo->actionType)
                {
                case ACTION_MOVE:       xp_osal_move_run(pSwInfo->osalMatchId, priSwCmd[i].cmd); break;
                case ACTION_FORCE_MOVE: xp_osal_force_move_run(pSwInfo->osalMatchId, priSwCmd[i].cmd, DEV_FORCE_MOVE_TIME); break;
                case ACTION_MOVE_POSE:  xp_osal_move_pos(pSwInfo->osalMatchId, priSwCmd[i].cmd, priSwCmd[i].pos); break;
                case ACTION_MOVE_TIME:  xp_osal_move_time(pSwInfo->osalMatchId, priSwCmd[i].cmd, priSwCmd[i].time); break;
                case ACTION_ROTATION:   xp_osal_brush_rotation(pSwInfo->osalMatchId, priSwCmd[i].cmd); break;
                default: break;
                }
                PriSwCrl_Table[i].isSwitch = false;
            }
        }
        aos_msleep(20);
    }
}
// Err_01:
    // while (1)
    // {
    //     LOG_FATAL("pSwInfo pointer overflow");
    //     // set_error_state( ,true);
    //     if(comSwSta[i] < COM_SWITCH_SHAR_NUM) break;
    //     aos_msleep(2000);
    // }

/*                                                         =======================                                                         */
/* ========================================================        其它接口        ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       判断设备是否处于空闲状态（考虑指令赋值到实际执行的缺口时间）
 * @param[in]	devIndex            
 * @return      bool                
 */
bool is_dev_move_sta_idle(Type_DriverIndex_Enum devIndex)
{
    //指令缺口时间认为机构在非空闲状态
    return (!cmdExeSta[devIndex].isCmdMoveGapTime && (MOTOR_STA_IDLE == xp_osal_get_motor_move_state(devIndex))) ? true : false;
}

/**
 * @brief       置位/清除设备驱动执行标志
 * @param[in]	value               
 */
void set_driver_executed_flag(bool value)
{
    isDriverExecuted = value;
}

/**
 * @brief       赋值暂停时间
 * @param[in]	value               
 */
void set_step_pause_time(uint64_t value)
{
    stepSta.pauseTime = value;
}

/**
 * @brief       设置是否有下一辆车准备进入工作区标志
 * @param[in]	value               
 */
void set_new_car_ready_wash_falg(bool value)
{
    isNewCarReadyWash = value;
    if(true == value) isClearConveyorEncFinish = false;
}

bool get_new_car_ready_wash_falg(void)
{
    return isNewCarReadyWash;
}

/**
 * @brief       设置是否允许下一辆车进入工作区洗车标志
 * @param[in]	value               
 */
void set_is_allow_next_car_wash_flag(bool value)
{
    isAllowNextCarInWorkArea = value;
}

/**
 * @brief       获取当前是否允许下一辆车进入工作区洗车标志
 * @return      bool                
 */
bool get_is_allow_next_car_wash_flag(void)
{
    return isAllowNextCarInWorkArea;
}

/**
 * @brief       赋值新订单的车辆Id
 * @param[in]	newOrderId          
 */
void set_new_order_car_id(uint8_t newOrderId)
{
    if(newOrderId >= 0 && newOrderId < SUPPORT_WASH_NUM_MAX){
        entryCarIndex = newOrderId;
        if(1 == entryCarIndex){         //进来的车辆是编号1，看编号2有没有车尾位置，没有则认为前面没有车
            isNewCarWash = true;
            if(carWash[2].tailPos > 0){
                washCarNum = 2;
                headWashCarId = 2;
            }
            else{
                washCarNum = 1;
                headWashCarId = 1;
            }
        }
        else if(2 == entryCarIndex){    //进来的车辆是编号2，看编号1有没有车尾位置，没有则认为前面没有车
            isNewCarWash = true;
            if(carWash[1].tailPos > 0){
                washCarNum = 2;
                headWashCarId = 1;
            }
            else{
                washCarNum = 1;
                headWashCarId = 2;
            }
        }
        else{
            isNewCarWash = false;
            washCarNum = 0;
            headWashCarId = 0;
        }
        LOG_UPLOAD("New order Start, car num %d, head id %d, entry id %d", washCarNum, headWashCarId, entryCarIndex);
    }
    else{
        LOG_UPLOAD("Illegal car Id");
    }
}

/**
 * @brief       获取洗车流程位置值变量指针
 * @return      Type_ModelSts_Def*  
 */
Type_CarProcPosInfo_Def *get_washProcPos_Obj(void)
{
    return &washProcPos;
}

/**
 * @brief       获取毛刷电流
 * @param[in]	type                
 * @return      int                 
 */
int get_brush_current(Type_BrushType_Enum type)
{
    if(type < BRUSH_NUM){
        return brush[type].current;
    }
    else{
        LOG_WARN("Get current no this type");
        return 0;
    }
}

/**
 * @brief       公共开关置空（切断共用开关的所有输出）
 */
void com_switch_change_empty(void)
{
    // for (uint8_t i = 0; i < COM_SWITCH_NUMBER; i++)     //共用开关的设备只要把总开关置空状态
    // {
    //     comSwSta[i] = COM_SWITCH_TO_EMPTY;
    // }
}

/**
 * @brief       获取当前订单编号的工作状态
 * @param[in]	washId              查询的车辆ID
 * @return      uint16_t            
 */
int get_work_state(uint8_t washId)
{
    uint16_t sta = 0;
    if(washId == entryCarIndex && is_signal_filter_trigger(SIGNAL_GATE_2_LEFT_OPEN) && is_signal_filter_trigger(SIGNAL_GATE_2_RIGHT_OPEN)) sta |= 0x0001;
    if(carWash[washId].headPos != 0 && carWash[washId].tailProc < PROC_FINISH_HIGH_PUMP) sta |= 0x0002;
    if(carWash[washId].headProc >= PROC_START_SKIRT_BRUSH_ROTATION && carWash[washId].tailProc < PROC_FINISH_SKIRT_BRUSH) sta |= 0x0004;
    if(carWash[washId].headProc >= PROC_START_SHAMPOO && carWash[washId].tailProc < PROC_FINISH_SHAMPOO) sta |= 0x0008;
    if(carWash[washId].headProc >= PROC_START_TOP_BRUSH && carWash[washId].tailProc < PROC_FINISH_TOP_BRUSH) sta |= 0x0010;
    if(carWash[washId].headProc >= PROC_START_FRONT_BRUSH && carWash[washId].tailProc < PROC_FINISH_FRONT_BRUSH) sta |= 0x0020;
    if(carWash[washId].headProc >= PROC_START_BACK_BRUSH && carWash[washId].tailProc < PROC_FINISH_BACK_BRUSH) sta |= 0x0040;
    if(carWash[washId].headProc >= PROC_START_WAXWATER && carWash[washId].tailProc < PROC_FINISH_WAXWAT) sta |= 0x0080;
    if(carWash[washId].headProc >= PROC_START_DYRING && carWash[washId].tailProc < PROC_FINISH_DYRING) sta |= 0x0100;
    if(carWash[washId].headProc == PROC_START_FRONT_BRUSH && is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)) sta |= 0x0200;
    if(carWash[washId].tailProc == PROC_FINISH_FRONT_BRUSH && is_dev_move_sta_idle(CONVEYOR_2_MATCH_ID)) sta |= 0x0400;
    return sta;
}

/**
 * @brief       停止所有移动动作
 */
static void stop_all_move(void)
{
    conveyor_move(CRL_ALL_SECTION, CMD_STILL);
    lifter_move(CMD_STILL, true);
    front_side_brush_move(CRL_BOTH, CMD_STILL, true);
    back_side_brush_move(CRL_BOTH, CMD_STILL, true);
    gate_change_state(CRL_SECTION_1, CMD_STILL);
    osal_dev_io_state_change(BOARD0_OUTPUT_SKIRT_BRUSH_VALVE, IO_DISABLE);
    // com_switch_change_empty();
}

/**
 * @brief       停止所有旋转动作
 */
static void stop_all_rotation(void)
{
    top_brush_rotation(CMD_STILL);
    front_side_brush_rotation(CRL_BOTH, CMD_STILL);
    back_side_brush_rotation(CRL_BOTH, CMD_STILL);
    dryer_work(CRL_ALL_SECTION, false);
    osal_dev_io_state_change(BOARD4_OUTPUT_LEFT_SKIRT_ROTATION, IO_DISABLE);
    osal_dev_io_state_change(BOARD4_OUTPUT_RIGHT_SKIRT_ROTATION, IO_DISABLE);
    // com_switch_change_empty();
}

static void stop_all_water(void)
{
    water_system_control(WATER_HIGH_PRESS_WATER, false);
    water_system_control(WATER_ALL, false);
}

/**
 * @brief       异常停止输出
 */
void stop_all_dev_run(void)
{
    stop_all_move();
    stop_all_rotation();
    stop_all_water();
    
    brush[BRUSH_TOP].runMode         = BRUSH_MANUAL;
    brush[BRUSH_FRONT_LEFT].runMode  = BRUSH_MANUAL;
    brush[BRUSH_FRONT_RIGHT].runMode = BRUSH_MANUAL;
    brush[BRUSH_BACK_LEFT].runMode   = BRUSH_MANUAL;
    brush[BRUSH_BACK_RIGHT].runMode  = BRUSH_MANUAL;
}

/**
 * @brief       洗车的一些变量初始化
 */
void wash_crl_variable_init(void)
{
    for (uint8_t i = 0; i < SUPPORT_WASH_NUM_MAX; i++)
    {
        memset(&carWash[i], 0, sizeof(Type_CarWashInfo_Def));
        carWash[i].topLifter = INIT_CAR_TOP_POS;
    }
    
    //这里会把触压报警值也清掉，需要重新赋值
    for (uint8_t i = 0; i < BRUSH_NUM; i++)
    {
        memset(&brush[i], 0, sizeof(Type_BrushInfo_Def));
        brush[i].pressWarning = (BRUSH_TOP == i) ? TOP_BRUSH_WARNING_CUR : SIDE_BRUSH_WARNING_CUR;
    }
}

/**
 * @brief       App前端控制机构设置
 * @param[in]	obj                 
 * @param[in]	cmd                 
 */
void app_crl_dev_set(Type_AppCrlObject_Enum obj, int cmd)
{
    switch (obj)
    {
    case APP_CRL_GATE_1_OPEN:       gate_change_state(CRL_SECTION_1, cmd > 0 ? CMD_BACKWARD : CMD_STILL); break;
    case APP_CRL_GATE_1_CLOSE:      gate_change_state(CRL_SECTION_1, cmd > 0 ? CMD_FORWARD : CMD_STILL); break;
    case APP_CRL_GATE_2_OPEN:       gate_change_state(CRL_SECTION_2, cmd > 0 ? CMD_BACKWARD : CMD_STILL); break;
    case APP_CRL_GATE_2_CLOSE:      gate_change_state(CRL_SECTION_2, cmd > 0 ? CMD_FORWARD : CMD_STILL);  break;
    case APP_CRL_GATE_3_OPEN:       gate_change_state(CRL_SECTION_3, cmd > 0 ? CMD_BACKWARD : CMD_STILL); break;
    case APP_CRL_GATE_3_CLOSE:      gate_change_state(CRL_SECTION_3, cmd > 0 ? CMD_FORWARD : CMD_STILL); break;
    case APP_CRL_CONVEYOR_1:        conveyor_move(CRL_SECTION_1, cmd); break;
    case APP_CRL_CONVEYOR_2:        conveyor_move(CRL_SECTION_2, cmd); break;
    case APP_CRL_CONVEYOR_3:        conveyor_move(CRL_SECTION_3, cmd); break;
    case APP_CRL_CONVEYOR_ALL:       
        conveyor_move(CRL_SECTION_1, cmd);
        conveyor_move(CRL_SECTION_2, cmd);
        conveyor_move(CRL_SECTION_3, cmd);
        break;
    case APP_CRL_HIGH_PRESS_WATER:
        water_system_control(WATER_HIGH_PRESS_WATER, cmd > 0 ? true : false);
        break;
    case APP_CRL_WATER_SHAMPOO:
        water_system_control(WATER_SHAMPOO, cmd > 0 ? true : false);
        water_system_control(WATER_SHAMPOO_PIKN, cmd > 0 ? true : false);
        water_system_control(WATER_SHAMPOO_GREEN, cmd > 0 ? true : false);
        break;
    case APP_CRL_WATER_WAXWATER:    water_system_control(WATER_WAXWATER, cmd > 0 ? true : false); break;
    case APP_CRL_WATER_TOP:         water_system_control(WATER_TOP, cmd > 0 ? true : false); break;
    case APP_CRL_WATER_FRONT_SIDE:  water_system_control(WATER_FRONT_SIDE, cmd > 0 ? true : false); break;
    case APP_CRL_WATER_BACK_SIDE:   water_system_control(WATER_BACK_SIDE, cmd > 0 ? true : false); break;
    case APP_CRL_SKIRT_BRUSH_MOVE:  osal_dev_io_state_change(BOARD0_OUTPUT_SKIRT_BRUSH_VALVE, cmd > 0 ? IO_ENABLE : IO_DISABLE); break;
    case APP_CRL_SKIRT_BRUSH_ROTATIN:
        osal_dev_io_state_change(BOARD4_OUTPUT_LEFT_SKIRT_ROTATION, cmd > 0 ? IO_ENABLE : IO_DISABLE);
        osal_dev_io_state_change(BOARD4_OUTPUT_RIGHT_SKIRT_ROTATION, cmd > 0 ? IO_ENABLE : IO_DISABLE); 
        break;
    case APP_CRL_TOP_BRUSH:         top_brush_rotation(cmd); break;
    case APP_CRL_LIFTER:            lifter_move(cmd, true);; break;
    case APP_CRL_FRONT_LEFT_BRUSH:  front_side_brush_rotation(CRL_ONLY_LEFT, cmd); break;
    case APP_CRL_FRONT_LEFT_MOVE:
        if(cmd > 0){
            front_side_brush_move_pos(CRL_ONLY_LEFT, cmd, PUTTER_GO_MIDDLE_OFFSET);
        }
        else{
            front_side_brush_move(CRL_ONLY_LEFT, cmd, true);
        }
        break;
    case APP_CRL_FRONT_RIGHT_BRUSH: front_side_brush_rotation(CRL_ONLY_RIGHT, cmd); break;
    case APP_CRL_FRONT_RIGHT_MOVE:
        if(cmd > 0){
            front_side_brush_move_pos(CRL_ONLY_RIGHT, cmd, PUTTER_GO_MIDDLE_OFFSET);
        }
        else{
            front_side_brush_move(CRL_ONLY_RIGHT, cmd, true);
        }
        break;
    case APP_CRL_BACK_LEFT_BRUSH:   back_side_brush_rotation(CRL_ONLY_LEFT, cmd); break;
    case APP_CRL_BACK_LEFT_MOVE:
        if(cmd > 0){
            back_side_brush_move_pos(CRL_ONLY_LEFT, cmd, PUTTER_GO_MIDDLE_OFFSET);
        }
        else{
            back_side_brush_move(CRL_ONLY_LEFT, cmd, true);
        }
        break;
    case APP_CRL_BACK_RIGHT_BRUSH:  back_side_brush_rotation(CRL_ONLY_RIGHT, cmd); break;
    case APP_CRL_BACK_RIGHT_MOVE:
        if(cmd > 0){
            back_side_brush_move_pos(CRL_ONLY_RIGHT, cmd, PUTTER_GO_MIDDLE_OFFSET);
        }
        else{
            back_side_brush_move(CRL_ONLY_RIGHT, cmd, true);
        }
        break;
    case APP_CRL_DRYER_1:           dryer_work(CRL_SECTION_1, cmd > 0 ? true : false); break;
    case APP_CRL_DRYER_25:
        dryer_work(CRL_SECTION_2, cmd > 0 ? true : false);
        dryer_work(CRL_SECTION_5, cmd > 0 ? true : false);
        break;
    case APP_CRL_DRYER_34:
        dryer_work(CRL_SECTION_3, cmd > 0 ? true : false);
        dryer_work(CRL_SECTION_4, cmd > 0 ? true : false);
        break;
    // case APP_CRL_DRYER_6:   ; break;
    case APP_CRL_FLOODLIGHT:
        osal_dev_io_state_change(BOARD1_OUTPUT_FLOODLIGHT, cmd > 0 ? IO_ENABLE : IO_DISABLE);
        osal_dev_io_state_change(BOARD3_OUTPUT_FLOODLIGHT, cmd > 0 ? IO_ENABLE : IO_DISABLE);
        osal_dev_io_state_change(BOARD4_OUTPUT_FLOODLIGHT, cmd > 0 ? IO_ENABLE : IO_DISABLE);
        break;
    case APP_CRL_AMBIENT_LIGHT:
        osal_dev_io_state_change(BOARD0_OUTPUT_AMBIENT_LIGHT, cmd > 0 ? IO_ENABLE : IO_DISABLE);
        osal_dev_io_state_change(BOARD4_OUTPUT_AMBIENT_LIGHT, cmd > 0 ? IO_ENABLE : IO_DISABLE);
        break;
    case APP_CRL_SEWAGE_PUMP:       water_system_control(WATER_SEWAGE_PUMP, cmd > 0 ? true : false); break;
    case APP_CRL_HIGH_PUMP:         water_system_control(WATER_HIGH_PUMP, cmd > 0 ? true : false); break;
    case APP_CRL_WINTER_DRAINAGE:   water_system_control(WATER_DRAIN, cmd > 0 ? true : false); break;
    // case APP_CRL_CONVEYOR_1_VALVE:  osal_dev_io_state_change(BOARD1_OUTPUT_WATER_CONVEYOR_1_VALVE, cmd > 0 ? IO_ENABLE : IO_DISABLE); break;
    // case APP_CRL_CONVEYOR_2_VALVE:  osal_dev_io_state_change(BOARD1_OUTPUT_WATER_CONVEYOR_2_VALVE, cmd > 0 ? IO_ENABLE : IO_DISABLE); break;
    // case APP_CRL_CONVEYOR_3_VALVE:  osal_dev_io_state_change(BOARD1_OUTPUT_WATER_CONVEYOR_3_VALVE, cmd > 0 ? IO_ENABLE : IO_DISABLE); break;
    default:
        break;
    }
}

/*                                                         =======================                                                         */
/* ========================================================        调试接口        ======================================================== */
/*                                                         =======================                                                         */
void ag_test_thread(void *arg){
    uint16_t testCnt = 0;
    bool result = true;
    
    while (1)
    {
        testCnt++;
        aos_msleep(100);
    }
}

int xp_module_debug(char *type, char *fun, char *param)
{
    char *ptr = NULL;  
    char *p = NULL;
    char *param_1 = NULL;
    char *param_2 = NULL;
    int ret = NOR_CONTINUE;

    ptr = strtok_r(param, "_", &p);
    if(ptr != NULL) {
        param_1 = ptr;
        LOG_DEBUG("param_1 = %s", param_1);
    }
    ptr = strtok_r(NULL, "_", &p);
    if(ptr != NULL) {
        param_2 = ptr;
        LOG_DEBUG("param_2 = %s", param_2);
    }

    if(strcmp(type, "module") == 0){
        if(strcmp(fun, "conveyor_s") == 0){
            Type_CrlType_Enum num = atoi(param_1);
            if(1 == num)        num = CRL_SECTION_1;
            else if(2 == num)   num = CRL_SECTION_2;
            else if(3 == num)   num = CRL_SECTION_3;
            else num = 0;
            conveyor_move(num, (Type_DriverCmd_Enum)atoi(param_2));
        }
        else if(strcmp(fun, "dryer") == 0){
            Type_CrlType_Enum num = atoi(param_1);
            if(1 == num)        num = CRL_SECTION_1;
            else if(2 == num)   num = CRL_SECTION_2;
            else if(3 == num)   num = CRL_SECTION_3;
            else if(4 == num)   num = CRL_SECTION_4;
            else if(5 == num)   num = CRL_SECTION_5;
            else num = 0;
            dryer_work(num, atoi(param_2));
        }
        else if(strcmp(fun, "floodlight") == 0){
            osal_dev_io_state_change(BOARD1_OUTPUT_FLOODLIGHT, atoi(param_1) ? IO_ENABLE : IO_DISABLE);
            osal_dev_io_state_change(BOARD3_OUTPUT_FLOODLIGHT, atoi(param_1) ? IO_ENABLE : IO_DISABLE);
            osal_dev_io_state_change(BOARD4_OUTPUT_FLOODLIGHT, atoi(param_1) ? IO_ENABLE : IO_DISABLE);
        }
        else if(strcmp(fun, "ambientLight") == 0){
            osal_dev_io_state_change(BOARD0_OUTPUT_AMBIENT_LIGHT, atoi(param_1) ? IO_ENABLE : IO_DISABLE);
            osal_dev_io_state_change(BOARD4_OUTPUT_AMBIENT_LIGHT, atoi(param_1) ? IO_ENABLE : IO_DISABLE);
        }
        else if (strcmp(fun, "water_drain") == 0) {
            water_system_control(WATER_DRAIN, atoi(param_1));
        }
        else if (strcmp(fun, "water_high_pump") == 0) {
            water_system_control(WATER_HIGH_PUMP, atoi(param_1));
        }
        else if (strcmp(fun, "water_low_pump") == 0) {
            water_system_control(WATER_LOW_PUMP, atoi(param_1));
        }
        else if (strcmp(fun, "water_waxwater") == 0) {
            water_system_control(WATER_WAXWATER, atoi(param_1));
        }
        else if (strcmp(fun, "water_shampoo") == 0) {
            water_system_control(WATER_SHAMPOO, atoi(param_1));
            water_system_control(WATER_SHAMPOO_PIKN, atoi(param_1));
            water_system_control(WATER_SHAMPOO_GREEN, atoi(param_1));
        }
        else if (strcmp(fun, "water_top") == 0) {
            water_system_control(WATER_TOP, atoi(param_1));
        }
        else if (strcmp(fun, "water_front_side") == 0) {
            water_system_control(WATER_FRONT_SIDE, atoi(param_1));
        }
        else if (strcmp(fun, "water_back_side") == 0) {
            water_system_control(WATER_BACK_SIDE, atoi(param_1));
        }
        // else if (strcmp(fun, "water_conveyor") == 0) {
        //     if(1 == atoi(param_1)) water_system_control(WATER_CONVEYOR_1, atoi(param_2));
        //     if(2 == atoi(param_1)) water_system_control(WATER_CONVEYOR_2, atoi(param_2));
        //     if(3 == atoi(param_1)) water_system_control(WATER_CONVEYOR_3, atoi(param_2));
        // }
        // else if (strcmp(fun, "water_middle") == 0) {
        //     water_system_control(WATER_MIDDLE, atoi(param_1));
        // }
        else if (strcmp(fun, "front_brush") == 0) {
            front_side_brush_rotation((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2));
        }
        else if (strcmp(fun, "back_brush") == 0) {
            back_side_brush_rotation((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2));
        }
        else if (strcmp(fun, "top_brush") == 0) {
            top_brush_rotation((Type_DriverCmd_Enum)atoi(param_1));
        }
        else if (strcmp(fun, "skirt_brush") == 0) {
            osal_dev_io_state_change(BOARD4_OUTPUT_LEFT_SKIRT_ROTATION, atoi(param_1) > 0 ? IO_ENABLE : IO_DISABLE);
            osal_dev_io_state_change(BOARD4_OUTPUT_RIGHT_SKIRT_ROTATION, atoi(param_1) > 0 ? IO_ENABLE : IO_DISABLE); 
        }
        else if (strcmp(fun, "front_move_s") == 0) {
            if(atoi(param_2) > 0){
                front_side_brush_move_pos((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2), PUTTER_GO_MIDDLE_OFFSET);
            }
            else{
                front_side_brush_move((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2), true);
            }
        }
        else if (strcmp(fun, "front_move_p") == 0) {
            front_side_brush_move_pos((Type_CrlType_Enum)atoi(param_1), CMD_FORWARD, atoi(param_2));
        }
        else if (strcmp(fun, "front_move_t") == 0) {
            front_side_brush_move_time((Type_CrlType_Enum)atoi(param_1), CMD_FORWARD, atoi(param_2));
        }
        else if (strcmp(fun, "front_move_force") == 0) {
            front_side_brush_move((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2), false);
        }
        else if (strcmp(fun, "back_move_s") == 0) {
            if(atoi(param_2) > 0){
                back_side_brush_move_pos((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2), PUTTER_GO_MIDDLE_OFFSET);
            }
            else{
                back_side_brush_move((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2), true);
            }
        }
        else if (strcmp(fun, "back_move_p") == 0) {
            back_side_brush_move_pos((Type_CrlType_Enum)atoi(param_1), CMD_FORWARD, atoi(param_2));
        }
        else if (strcmp(fun, "back_move_t") == 0) {
            back_side_brush_move_time((Type_CrlType_Enum)atoi(param_1), 
            atoi(param_2) > 0 ? CMD_FORWARD : CMD_BACKWARD, abs(atoi(param_2)));
        }
        else if (strcmp(fun, "back_move_force") == 0) {
            back_side_brush_move((Type_CrlType_Enum)atoi(param_1), (Type_DriverCmd_Enum)atoi(param_2), false);
        }
        else if (strcmp(fun, "lifter_s") == 0) {
            lifter_move((Type_DriverCmd_Enum)atoi(param_1), true);
        }
        else if (strcmp(fun, "lifter_p") == 0) {
            lifter_move_pos(CMD_FORWARD, atoi(param_1));
        }
        else if (strcmp(fun, "lifter_t") == 0) {
            lifter_move_time(CMD_FORWARD, atoi(param_1));
        }
        else if (strcmp(fun, "lifter_force") == 0) {
            lifter_move((Type_DriverCmd_Enum)atoi(param_1), false);
        }
        else if (strcmp(fun, "gate_move") == 0) {
            Type_CrlType_Enum num = atoi(param_1);
            if(1 == num)        num = CRL_SECTION_1;
            else if(2 == num)   num = CRL_SECTION_2;
            else if(3 == num)   num = CRL_SECTION_3;
            else num = 0;
            gate_change_state(num, atoi(param_2));
        }
        else if(strcmp(fun, "test") == 0) {
//            aos_task_new("test_thread",	av_test_thread,    NULL, 1024);
        }
    } else {
        return 0;
    }
    return 1;
}

