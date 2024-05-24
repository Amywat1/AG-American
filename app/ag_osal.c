/**
 * @file 	 ag_osal.c
 * @brief
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 *
 * @copyright Copyright (c) 2022  YGL
 *
 */

#include <stdlib.h>
#include "ag_osal.h"
#include "../../../../1km/bsp/bsp.h"
#include "aos/kv.h"
#include "aos/kernel.h"
#include "../../../../1km/driver/transducer/lnovance_md.h"
#include "../../../../1km/driver/display/led_screen.h"
#include "../../../../1km/driver/voice.h"
#include "../../../../1km/common/boardio.h"
#include "../../../../1km/bsp/io_config.h"
#include "../../../../1km/driver/voice/dl485_fl.h"
 // #include "../../../1km/driver/sensor/pt100-485/pt100_485.h"

#define USE_ONBOARD_VOICE           (1)
#define USE_DL485_VOICE             (1)
#define NULL_ENCODER                (0xFFFFFFFF)    //未获取码盘脉冲值时的返回值
#define NONE_REG_INFO               (0xFF)          //无注册信息
#define SIGNAL_TRIGGER_THREAD_FREQ  (20)            //信号触发检测线程的周期
#define SIGNAL_COMFIRM_CNT          (20)            //信号检测确认次数
#define IO_IS_TRIGGER               (-1)            //信号已触发
#define SIGNAL_STABLE               (0)             //信号保持状态
#define IO_NO_TRIGGER               (1)             //信号未触发
#define MOTOR_TABLE_NUM             (sizeof(MotorDev_Table) / sizeof(MotorDev_Table[0]))    //电机列表个数
#define ENCODE_TABLE_NUM            (sizeof(Encoder_Table) / sizeof(Encoder_Table[0]))      //码盘列表个数

static bool boardIoInSta[4]  = {1,1,1,1};           //主板只有4个输入输出点位（默认状态为1）
static bool boardIoOutSta[4] = {1,1,1,1};

static bool isPumpWorking = false;

aos_mutex_t rs485_1_mux;

//设备驱动类型
typedef enum {
	DRIVER_TYPE_VFD_LNOVANCE = 0,		//汇川变频器驱动
	DRIVER_TYPE_KM,						//线圈驱动
	DRIVER_TYPE_TRIGGER,				//脉冲触发驱动
	DRIVER_TYPE_ADDR,					//点位路径触发驱动
} Type_DriverType_Enum;

//设备动作类型
typedef enum {
	ACTION_TYPE_MOVE = 0,				//移动类型
	ACTION_TYPE_ROTATION,				//旋转类型
} Type_ActionType_Enum;

//移动电机控制信息定义
typedef struct {
    Type_MoveState_Enum     state;
    Type_MoveState_Enum     lastState;
    bool                    isFirstSwitch;      //是否首次切换状态

    bool                    isTriggerEnable;    //是否使能引脚触发
    uint16_t                triggerTime;        //触发时间
    Type_OutputIo_Enum      ioIndexTrig;        //触发引脚

    int32_t                 tarPos;
    uint16_t                moveTime;
    uint16_t                forceMoveTime;
    uint32_t                actionOverTime;     //单次动作的最长时间
    uint64_t                timeStamp;

    Type_LimitMode_Enum     limitMode;          //限位模式
    uint16_t                limtMinPos;         //限制移动的最小距离
    uint16_t                limtMaxPos;         //限制移动的最长距离
    uint8_t                 limitTouchedCnt;    //限位触发次数

    Type_OutputIo_Enum      ioIndexEnable;      //驱动使能IO
    Type_InputIo_Enum       ioIndexLimitCW;     //正向限位IO
    Type_InputIo_Enum       ioIndexLimitCCW;    //反向限位IO
} Type_MoveDevInfo_Def;

//电机信息定义
typedef struct {
    Type_ActionType_Enum    actionType;         //动作类型
    Type_DriverIndex_Enum   drvIndex;           //驱动器索引
    Type_DriverType_Enum    drvType;            //驱动器类型
    uint8_t                 matchVfdRegNum;     //用于链接变频器驱动表中注册的某一实体（如匹配 Lnovance_Table[] 哪一组）
    
    Type_OutputIo_Enum      ioIndexCW;          //正转IO
    Type_OutputIo_Enum      ioIndexCCW;         //反转IO
    int                     tarVel;             //目标速度
    int                     lastVel;            //上次的速度
    uint8_t                 retryCnt;           //重试计数

    Type_MoveDevInfo_Def    moveInfo;           //移动电机的设备信息
} Type_MotorInfo_Def;

//码盘信息
typedef struct
{
    int32_t                 encoderValue;       //码盘计数基于识别零点的相对值（未校零时可能为负数）
    int32_t                 lastEncoderValue;   //上次的码盘计数值
    Type_DriverIndex_Enum   drvIndex;           //对应业务主板控制的驱动器索引
    uint8_t                 ioBoardRegIndex;    //对应子板注册设备的索引

    Type_InputIo_Enum       ioIndexPulse;       //脉冲采集IO
    Type_OutputIo_Enum      ioIndexCW;          //控制正向移动IO
    Type_OutputIo_Enum      ioIndexCCW;         //控制反向移动IO
    Type_InputIo_Enum       ioIndexZero;        //初始零位IO
} Type_EncoderInfo_Def;

//IO脉冲触发信息定义（非 MotorDev_Table 里管理的IO）
typedef struct
{
    bool                    isEnable;           //触发IO是否使能
    uint64_t                startTimeStamp;     //初始时间戳
    uint16_t                triggerTime;        //脉冲触发时间
    Type_OutputIo_Enum      ioIndexTrig;        //触发引脚IO索引
    Type_OutputIo_Enum      enable;             //设备使能IO
} Type_IoTriggerInfo_Def;

//脉冲触发IO枚举（非 MotorDev_Table 里管理的IO）
typedef enum {
    TRIGGER_IO_NUM,
} Type_TriggerIo_Enum;

// Type_IoTriggerInfo_Def      triggerIo[TRIGGER_IO_NUM] = {0};

//电机驱动的设备注册表（没有脉冲计数的电机 .moveInfo.limitMode 限位模式统一设为 MODE_SIGNAL_LIMIT 用以区分有无脉冲计数电机）
static Type_MotorInfo_Def MotorDev_Table[] = {
    //负责旋转，无位移的电机（汇川变频器驱动）
    [0] = {                             //左前刷旋转
        .actionType     = ACTION_TYPE_ROTATION,
        .drvIndex       = FRONT_LEFT_BRUSH_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 3,            //对应 Lnovance_Table[] 中的数组序号————有匹配的注册表设备不需要重复定义IO（在变频器驱动注册表中已定义，重复定义以变频器驱动注册表中的定义为准）
    },
    [1] = {                             //右前刷旋转
        .actionType     = ACTION_TYPE_ROTATION,
        .drvIndex       = FRONT_RIGHT_BRUSH_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 4,
    },
    [2] = {                             //左后刷旋转
        .actionType     = ACTION_TYPE_ROTATION,
        .drvIndex       = BACK_LEFT_BRUSH_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 5,
    },
    [3] = {                             //右后刷旋转
        .actionType     = ACTION_TYPE_ROTATION,
        .drvIndex       = BACK_RIGHT_BRUSH_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 6,
    },
    [4] = {                             //顶刷旋转
        .actionType     = ACTION_TYPE_ROTATION,
        .drvIndex       = TOP_BRUSH_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 7,
    },
    //负责旋转，无位移的电机（接触器/继电器驱动）

    //负责位移的电机（汇川变频器驱动）                  /*         ************************************         */
    //负责位移的电机参与状态机轮询                     /* ********注意正反限位与汇川驱动注册表中的实体对应******* */
                                                     /*         ************************************         */
    [5] = {                             //1#输送带
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = CONVEYOR_1_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 0,
        // .ioIndexCW      =            //汇川控制的电机正反转IO索引在 Lnovance_Table[] 中定义
        // .ioIndexCCW     = 
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = MOVE_FOREVER,
        .moveInfo.limitMode      = MODE_SIGNAL_LIMIT,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = MOVE_FOREVER,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = IO_NULL,
        .moveInfo.ioIndexLimitCCW  = IO_NULL,
    },
    [6] = {                             //2#输送带
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = CONVEYOR_2_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 1,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = MOVE_FOREVER,
        .moveInfo.limitMode      = MODE_SIGNAL_LIMIT,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = MOVE_FOREVER,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = IO_NULL,
        .moveInfo.ioIndexLimitCCW  = IO_NULL,
    },
    [7] = {                             //3#输送带
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = CONVEYOR_3_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 2,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = MOVE_FOREVER,
        .moveInfo.limitMode      = MODE_SIGNAL_LIMIT,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = MOVE_FOREVER,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = IO_NULL,
        .moveInfo.ioIndexLimitCCW  = IO_NULL,
    },
    [8] = {                             //升降移动
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = LIFTER_MATCH_ID,
        .drvType        = DRIVER_TYPE_VFD_LNOVANCE,
        .matchVfdRegNum = 8,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = 20000,
        .moveInfo.limitMode      = MODE_SIGNAL_LIMIT,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = MOVE_FOREVER,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = BOARD4_INPUT_LIFTER_DOWN,
        .moveInfo.ioIndexLimitCCW  = BOARD4_INPUT_LIFTER_UP,
    },

    //负责位移的电机（接触器驱动）
    [9] = {                             //左前刷移动
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = FRONT_LEFT_MOVE_MATCH_ID,
        .drvType        = DRIVER_TYPE_KM,
        .matchVfdRegNum = NONE_REG_INFO,
        .ioIndexCW      = BOARD2_OUTPUT_FRONT_LEFT_CLOSE,
        .ioIndexCCW     = BOARD2_OUTPUT_FRONT_LEFT_OPEN,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = 15000,
        .moveInfo.limitMode      = MODE_SOFT_LIMIT_MAX,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = 150,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = BOARD2_INPUT_FRONT_LEFT_BRUSH_DOWN,
        .moveInfo.ioIndexLimitCCW  = BOARD2_INPUT_FRONT_LEFT_MOVE_ZERO,
    },
    [10] = {                            //右前刷移动
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = FRONT_RIGHT_MOVE_MATCH_ID,
        .drvType        = DRIVER_TYPE_KM,
        .matchVfdRegNum = NONE_REG_INFO,
        .ioIndexCW      = BOARD2_OUTPUT_FRONT_RIGHT_CLOSE,
        .ioIndexCCW     = BOARD2_OUTPUT_FRONT_RIGHT_OPEN,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = 15000,
        .moveInfo.limitMode      = MODE_SOFT_LIMIT_MAX,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = 150,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = BOARD2_INPUT_FRONT_RIGHT_BRUSH_DOWN,
        .moveInfo.ioIndexLimitCCW  = BOARD2_INPUT_FRONT_RIGHT_MOVE_ZERO,
    },
    [11] = {                            //左后刷移动
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = BACK_LEFT_MOVE_MATCH_ID,
        .drvType        = DRIVER_TYPE_KM,
        .matchVfdRegNum = NONE_REG_INFO,
        .ioIndexCW      = BOARD2_OUTPUT_BACK_LEFT_CLOSE,
        .ioIndexCCW     = BOARD2_OUTPUT_BACK_LEFT_OPEN,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = 15000,
        .moveInfo.limitMode      = MODE_SOFT_LIMIT_MAX,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = 150,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = BOARD2_INPUT_BACK_LEFT_BRUSH_DOWN,
        .moveInfo.ioIndexLimitCCW  = BOARD2_INPUT_BACK_LEFT_MOVE_ZERO,
    },
    [12] = {                             //右后刷移动
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = BACK_RIGHT_MOVE_MATCH_ID,
        .drvType        = DRIVER_TYPE_KM,
        .matchVfdRegNum = NONE_REG_INFO,
        .ioIndexCW      = BOARD2_OUTPUT_BACK_RIGHT_CLOSE,
        .ioIndexCCW     = BOARD2_OUTPUT_BACK_RIGHT_OPEN,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = 15000,
        .moveInfo.limitMode      = MODE_SOFT_LIMIT_MAX,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = 150,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = BOARD2_INPUT_BACK_RIGHT_BRUSH_DOWN,
        .moveInfo.ioIndexLimitCCW  = BOARD2_INPUT_BACK_RIGHT_MOVE_ZERO,
    },
    [13] = {                             //1#道闸
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = GATE_1_MACH_ID,
        .drvType        = DRIVER_TYPE_KM,
        .matchVfdRegNum = NONE_REG_INFO,
        .ioIndexCW      = BOARD4_OUTPUT_GATE_1_CLOSE,
        .ioIndexCCW     = BOARD4_OUTPUT_GATE_1_OPEN,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = 10000,
        .moveInfo.limitMode      = MODE_SIGNAL_LIMIT,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = MOVE_FOREVER,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        .moveInfo.ioIndexLimitCW   = BOARD5_INPUT_GATE_1_CLOSE_DONE,
        .moveInfo.ioIndexLimitCCW  = BOARD5_INPUT_GATE_1_OPEN_DONE,
    },
    [14] = {                             //3#道闸
        .actionType     = ACTION_TYPE_MOVE,
        .drvIndex       = GATE_3_MACH_ID,
        .drvType        = DRIVER_TYPE_KM,
        .matchVfdRegNum = NONE_REG_INFO,
        .ioIndexCW      = BOARD1_OUTPUT_GATE_3_CLOSE,
        .ioIndexCCW     = BOARD1_OUTPUT_GATE_3_OPEN,
        .tarVel         = 0,
        .lastVel        = 0,

        .moveInfo.state          = MOTOR_STA_IDLE,
        .moveInfo.isFirstSwitch  = true,
        .moveInfo.actionOverTime = 10000,
        .moveInfo.limitMode      = MODE_SIGNAL_LIMIT,
        .moveInfo.limtMinPos     = 0,
        .moveInfo.limtMaxPos     = MOVE_FOREVER,
        
        .moveInfo.ioIndexEnable    = IO_NULL,
        // .moveInfo.ioIndexLimitCW   = BOARD5_INPUT_GATE_3_CLOSE_DONE,
        // .moveInfo.ioIndexLimitCCW  = BOARD5_INPUT_GATE_3_OPEN_DONE,
    },
};

//码盘状态
typedef struct
{
    bool isErr;                                 //码盘计数值是否异常状态
    bool isMoveSta[MOTOR_TABLE_NUM];            //机构是否在移动状态
    Type_DriverIndex_Enum errIndex;             //错误的码盘编号索引
    uint8_t errCnt[MOTOR_TABLE_NUM];
    uint64_t checkTimeStamp[MOTOR_TABLE_NUM];
} Type_EncoderState_Def;

Type_EncoderState_Def   encSta = {0};           //码盘状态

//码盘脉冲计数列表
static Type_EncoderInfo_Def Encoder_Table[] = {
    [0] = {                             //1#输送带
        .encoderValue       = 0,
        .drvIndex           = CONVEYOR_1_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,                    //某块子板注册的设备编号xx，如子板1注册的编号0设备（默认无注册信息，注册的时候赋值）
        .ioIndexPulse       = BOARD0_INPUT_CONVEYOR_1_PULSE,
        // .ioIndexCW          =                                //正反转和零位引脚在初始化的时候赋值 MotorDev_Table[] 中的值
        // .ioIndexCCW         =
        // .ioIndexZero        =
    },
    [1] = {                             //2#输送带
        .encoderValue       = 0,
        .drvIndex           = CONVEYOR_2_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,
        .ioIndexPulse       = BOARD0_INPUT_CONVEYOR_2_PULSE,
    },
    [2] = {                             //3#输送带
        .encoderValue       = 0,
        .drvIndex           = CONVEYOR_3_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,
        .ioIndexPulse       = BOARD0_INPUT_CONVEYOR_3_PULSE,
    },
    [3] = {                             //左前刷移动
        .encoderValue       = 0,
        .drvIndex           = FRONT_LEFT_MOVE_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,
        .ioIndexPulse       = BOARD2_INPUT_FRONT_LEFT_MOVE_PULSE,
    },
    [4] = {                             //右前刷移动
        .encoderValue       = 0,
        .drvIndex           = FRONT_RIGHT_MOVE_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,
        .ioIndexPulse       = BOARD2_INPUT_FRONT_RIGHT_MOVE_PULSE,
    },
    [5] = {                             //左后刷移动
        .encoderValue       = 0,
        .drvIndex           = BACK_LEFT_MOVE_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,
        .ioIndexPulse       = BOARD2_INPUT_BACK_LEFT_MOVE_PULSE,
    },
    [6] = {                             //右后刷移动
        .encoderValue       = 0,
        .drvIndex           = BACK_RIGHT_MOVE_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,
        .ioIndexPulse       = BOARD2_INPUT_BACK_RIGHT_MOVE_PULSE,
    },
    [7] = {                             //升降移动
        .encoderValue       = 0,
        .drvIndex           = LIFTER_MATCH_ID,
        .ioBoardRegIndex    = NONE_REG_INFO,
        .ioIndexPulse       = BOARD4_INPUT_LIFTER_PULSE,
    },
};

//变频器列表
static Type_Lnovance_Def Lnovance_Table[] = {
    //1#输送带
    [0].com.slave_id = 7,
    [0].com.port     = 1,
    [0].com.speed    = 9600,
    [0].isInitOk     = false,

    [0].DI1.boardId = BOARD_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_1_CW),     [0].DI1.pinId = PIN_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_1_CW),
    [0].DI2.boardId = BOARD_NULL,                                           [0].DI2.pinId = IO_NULL,
    [0].DI3.boardId = BOARD_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_1_RESET),  [0].DI3.pinId = PIN_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_1_RESET),

    //2#输送带
    [1].com.slave_id = 8,
    [1].com.port     = 1,
    [1].com.speed    = 9600,
    [1].isInitOk     = false,

    [1].DI1.boardId = BOARD_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_2_CW),     [1].DI1.pinId = PIN_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_2_CW),
    [1].DI2.boardId = BOARD_NULL,                                           [1].DI2.pinId = IO_NULL,
    [1].DI3.boardId = BOARD_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_2_RESET),  [1].DI3.pinId = PIN_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_2_RESET),

    //3#输送带
    [2].com.slave_id = 9,
    [2].com.port     = 1,
    [2].com.speed    = 9600,
    [2].isInitOk     = false,

    [2].DI1.boardId = BOARD_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_3_CW),     [2].DI1.pinId = PIN_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_3_CW),
    [2].DI2.boardId = BOARD_NULL,                                           [2].DI2.pinId = IO_NULL,
    [2].DI3.boardId = BOARD_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_3_RESET),  [2].DI3.pinId = PIN_ID_RESOLUTION(BOARD0_OUTPUT_CONVEYOR_3_RESET),

    //左前刷旋转
    [3].com.slave_id = 2,
    [3].com.port     = 1,
    [3].com.speed    = 9600,
    [3].isInitOk     = false,

    [3].DI1.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_LEFT_BRUSH_CW),       [3].DI1.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_LEFT_BRUSH_CW),
    [3].DI2.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_LEFT_BRUSH_CCW),      [3].DI2.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_LEFT_BRUSH_CCW),
    [3].DI3.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_LEFT_BRUSH_RESET),    [3].DI3.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_LEFT_BRUSH_RESET),

    //右前刷旋转
    [4].com.slave_id = 3,
    [4].com.port     = 1,
    [4].com.speed    = 9600,
    [4].isInitOk     = false,

    [4].DI1.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_CW),      [4].DI1.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_CW),
    [4].DI2.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_CCW),     [4].DI2.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_CCW),
    [4].DI3.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_RESET),   [4].DI3.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_RESET),

    //左后刷旋转
    [5].com.slave_id = 4,
    [5].com.port     = 1,
    [5].com.speed    = 9600,
    [5].isInitOk     = false,

    [5].DI1.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_BACK_LEFT_BRUSH_CW),        [5].DI1.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_BACK_LEFT_BRUSH_CW),
    [5].DI2.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_BACK_LEFT_BRUSH_CCW),       [5].DI2.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_BACK_LEFT_BRUSH_CCW),
    [5].DI3.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_BACK_LEFT_BRUSH_RESET),     [5].DI3.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_BACK_LEFT_BRUSH_RESET),

    //右后刷旋转
    [6].com.slave_id = 5,
    [6].com.port     = 1,
    [6].com.speed    = 9600,
    [6].isInitOk     = false,

    [6].DI1.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_BACK_RIGHT_BRUSH_CW),       [6].DI1.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_BACK_RIGHT_BRUSH_CW),
    [6].DI2.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_BACK_RIGHT_BRUSH_CCW),      [6].DI2.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_BACK_RIGHT_BRUSH_CCW),
    [6].DI3.boardId = BOARD_ID_RESOLUTION(BOARD2_OUTPUT_BACK_RIGHT_BRUSH_RESET),    [6].DI3.pinId = PIN_ID_RESOLUTION(BOARD2_OUTPUT_BACK_RIGHT_BRUSH_RESET),

    //顶刷旋转
    [7].com.slave_id = 1,
    [7].com.port     = 1,
    [7].com.speed    = 9600,
    [7].isInitOk     = false,

    [7].DI1.boardId = BOARD_NULL,                                       [7].DI1.pinId = IO_NULL,
    [7].DI2.boardId = BOARD_ID_RESOLUTION(BOARD4_OUTPUT_TOP_ROTATION),  [7].DI2.pinId = PIN_ID_RESOLUTION(BOARD4_OUTPUT_TOP_ROTATION),
    [7].DI3.boardId = BOARD_NULL,                                       [7].DI3.pinId = IO_NULL,

    //顶刷升降
    [8].com.slave_id = 6,
    [8].com.port     = 1,
    [8].com.speed    = 9600,
    [8].isInitOk     = false,

    [8].DI1.boardId = BOARD_ID_RESOLUTION(BOARD4_OUTPUT_LIFTER_CW),     [8].DI1.pinId = PIN_ID_RESOLUTION(BOARD4_OUTPUT_LIFTER_CW),
    [8].DI2.boardId = BOARD_ID_RESOLUTION(BOARD4_OUTPUT_LIFTER_CCW),    [8].DI2.pinId = PIN_ID_RESOLUTION(BOARD4_OUTPUT_LIFTER_CCW),
    [8].DI3.boardId = BOARD_ID_RESOLUTION(BOARD4_OUTPUT_LIFTER_RESET),  [8].DI3.pinId = PIN_ID_RESOLUTION(BOARD4_OUTPUT_LIFTER_RESET),
};

/* //温度传感器列表
Type_PR3002_Def pr3002_List[] = {
    [0].com.slage_id = 30,
    [0].com.port = 2,
    [0].com.speed = 9600,

    [0].isInitOk = false,

    [1].com.slage_id = 31,
    [1].com.port = 2,
    [1].com.speed = 9600,

    [1].isInitOk = false,
}; */

static Type_Driver_Def agDriver = { 0 };
int osal_debug(char* type, char* fun, char* param);
void xp_osal_encoder_pulse_update_thread(void* arg);
void xp_osal_dev_run_thread(void* arg);
void xp_osal_io_pulse_trigger_thread(void* arg);
void xp_osal_water_system_crl_thread(void* arg);
void signal_trigger_state_thread(void *arg);

void osal_dev_io_state_change(Type_OutputIo_Enum index, bool sta);

static void (*osal_error_upload)(uint16_t code, bool value);
static void (*offline_payment)(uint8_t washMode);


/*                                                         =======================                                                         */
/* ========================================================     打印字符查询表     ======================================================== */
/*                                                         =======================                                                         */

//机构字符查询表
static const char DeviceStrList[][20] = { "BRUSH_FRONT_L", "BRUSH_FRONT_R", "BRUSH_BACK_L", "BRUSH_BACK_R", \
                                          "BRUSH_TOP", "1#_CONVEYOR", "2#_CONVEYOR", "3#_CONVEYOR", "MOVE_LIFTER", \
                                          "MOVE_FRONT_L", "MOVE_FRONT_R", "MOVE_BACK_L", "MOVE_BACK_R", "GATE_1#", "GATE_2#"};
static const char* xp_get_device_str(uint8_t index)
{
    return DeviceStrList[index];
}

//机构码盘字符查询表
static const char EncoderStrList[][20] = { "1#_CONVEYOR", "2#_CONVEYOR", "3#_CONVEYOR", "MOVE_FRONT_L", "MOVE_FRONT_R", \
                                           "MOVE_BACK_L", "MOVE_BACK_R", "MOVE_LIFTER"};
static const char* xp_get_encoder_str(uint8_t index)
{
    return EncoderStrList[index];
}

//状态字符查询表
static const char StateStrList[][10] = { "IDLE", "ROTATION", "MOV", "FMOV", "POS", "TIME", "PAUSE", "RESUME", "STOP", "FAULT" };
static const char* xp_osal_move_get_state_str(Type_MoveState_Enum state)
{
    return StateStrList[state];
}

/*                                                         =======================                                                         */
/* ========================================================     KV值记录和存储     ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       记录KV值
 * @param[in]	pKeyStr             KV key             
 * @param[in]	pData               数据地址
 * @param[in]	len                 数据长度                 
 * @return      int                 
 */
int xp_save_kv_params(char *pKeyStr, const void *pData, int len)
{
    if(pKeyStr && pData){
        int ret = aos_kv_set(pKeyStr, pData, len, 1);
        if(0 == ret)    return 0;
        LOG_UPLOAD("Update kv fail. <%s> <%d>", pKeyStr, ret);
    }
    return -1;
}

/**
 * @brief       读取KV参数,如果不存在则创建,并设置为默认值
 * @param[in]	pKeyStr             KV key             
 * @param[in]	pData               数据地址
 * @param[in]	len                 数据长度指针        
 * @return      int             
 */
int xp_read_creat_kv_params(char *pKeyStr, void *pData, int *len)
{
    if(pKeyStr && pData){
        int ret = aos_kv_get(pKeyStr, pData, len);
        if(0 == ret)    return 0;
        LOG_WARN("Read kv fail. <%s> <%d>", pKeyStr, ret);
        
        ret = aos_kv_set(pKeyStr, pData, *len, 1);
        if(0 == ret)    return 0;
        LOG_UPLOAD("No found Kv and Update kv fail. <%s> <%d>", pKeyStr, ret);
    }
    return -1;
}

/*                                                         =======================                                                         */
/* ========================================================      voice相关接口      ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       onboard_voice_init
 * @return      xp_bool
 */
static xp_bool onboard_voice_init()
{
    return (0 == xp_voice_init()) ? xp_true : xp_false;
}

/**
 * @brief
 * @param[in]	item
 * @return      xp_bool
 */
static xp_bool onboard_voice_play(Type_AgVoice_Enum item)
{
    char voice_file_path[64] = { 0 };
    int sta=0;

    if(AG_VOICE_SILENCE == item){
        sta = xp_voice_stop();
        if(sta != 0){
            xp_voice_stop();
        }
    }
    else{
        sprintf(voice_file_path, "fs:/sdcard/ag_voice/00%d.mp3", item);
        sta = xp_voice_stop();
        if(sta != 0){
            xp_voice_stop();
        }
        xp_voice_start(voice_file_path, 60);
    }
    return xp_true;
}

/**
 * @brief       voice_init
 * @return      int
 */
static int xp_osal_voice_init(void)
{
    bool isSuccess = true;
#if (1 == USE_DL485_VOICE)
    isSuccess &= dl485_voice_init();
#endif
#if (1 == USE_ONBOARD_VOICE)
    isSuccess &= onboard_voice_init();
#endif
    return isSuccess? 0 : -1;
}

/**
 * @brief       voice_play
 * @param[in]	item
 * @return      int
 */
static int xp_osal_voice_play(Type_AgVoicePos_Enum pos, Type_AgVoice_Enum item)
{
    bool isSuccess = true;
#if (1 == USE_DL485_VOICE)
    uint8_t tryCnt = 3;
    if(AG_VOICE_POS_ENTRY == pos){
        isSuccess &= (0 == dl485_voice_play(pos, item)) ? true : false;
        while (!isSuccess && tryCnt--)
        {
            isSuccess &= (0 == dl485_voice_play(pos, item)) ? true : false;
        }
    }
#endif
#if (1 == USE_ONBOARD_VOICE)
    if(AG_VOICE_POS_EXIT == pos){
        isSuccess &= onboard_voice_play(item);
    }
#endif
    return isSuccess? 0 : -1;
}

/*                                                         =======================                                                         */
/* ========================================================      显示相关接口      ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       display_init
 * @return      int
 */
static int xp_osal_display_init(void)
{
    // led_screen_init();
    return 0;
}

/**
 * @brief       display_set
 * @param[in]	code
 * @return      int
 */
static int xp_osal_display_set(Type_AgDisplay_Enum code)
{
    static Type_AgDisplay_Enum lastCode = 0;

    osal_dev_io_state_change(BOARD4_OUTPUT_LED_IN_01, (code & 0x01) ? IO_ENABLE : IO_DISABLE);
    osal_dev_io_state_change(BOARD4_OUTPUT_LED_IN_02, (code & 0x02) ? IO_ENABLE : IO_DISABLE);
    osal_dev_io_state_change(BOARD4_OUTPUT_LED_IN_03, (code & 0x04) ? IO_ENABLE : IO_DISABLE);
    osal_dev_io_state_change(BOARD4_OUTPUT_LED_IN_04, (code & 0x08) ? IO_ENABLE : IO_DISABLE);
    if(lastCode != code){
        LOG_UPLOAD("Display change to %d", code);
    }
    lastCode = code;
    return 0;
}

/*                                                         =======================                                                         */
/* ========================================================     变频器相关接口     ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       freq_init
 * @return      int
 */
static int xp_osal_freq_init(void)
{
    xp_lnovance_init(Lnovance_Table, sizeof(Lnovance_Table) / sizeof(Lnovance_Table[0]));
    return 0;
}

/**
 * @brief       freq_run（这个的ID已经匹配过了，不用再匹配）
 * @param[in]	index                   //变频器的列表索引
 * @param[in]	vel
 * @return      int
 */
static int xp_osal_freq_run(int index, int vel)
{
    return (true == xp_lnovance_run(index, vel)) ? 0 : -1;
}

/**
 * @brief       freq_get_current
 * @param[in]	index                   //电机驱动设备列表索引
 * @return      int
 */
static int xp_osal_freq_get_current(int index)
{
    int16_t current = 0;

    if(0 == aos_mutex_lock(&rs485_1_mux, 100)){
        for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
        {
            if (DRIVER_TYPE_VFD_LNOVANCE == MotorDev_Table[i].drvType && index == MotorDev_Table[i].drvIndex) {
                if(false == xp_lnovance_get_current(MotorDev_Table[i].matchVfdRegNum, &current)){
                    current = 0xFFFF;
                }
                break;
            }
        }
    }
    aos_mutex_unlock(&rs485_1_mux);
    return current;
}

/**
 * @brief       freq_get_state
 * @param[in]	index                   //电机驱动设备列表索引
 * @return      uint16_t
 */
static uint16_t xp_osal_freq_get_state(int index)
{
    uint16_t state = 0;

    if(0 == aos_mutex_lock(&rs485_1_mux, 100)){
        for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
        {
            if (DRIVER_TYPE_VFD_LNOVANCE == MotorDev_Table[i].drvType && index == MotorDev_Table[i].drvIndex) {
                if (false == xp_lnovance_get_state(MotorDev_Table[i].matchVfdRegNum, &state)) {
                    state = 0xFFFF;
                }
                break;
            }
        }
    }
    aos_mutex_unlock(&rs485_1_mux);
    return state;
}

/**
 * @brief       freq_get_error
 * @param[in]	index                   //电机驱动设备列表索引
 * @return      uint32_t
 */
static uint32_t xp_osal_freq_get_error(int index)
{
    uint32_t errCode = 0;

    if(0 == aos_mutex_lock(&rs485_1_mux, 100)){
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (DRIVER_TYPE_VFD_LNOVANCE == MotorDev_Table[i].drvType && index == MotorDev_Table[i].drvIndex) {
            if (false == xp_lnovance_get_error_code(MotorDev_Table[i].matchVfdRegNum, &errCode)) {
                errCode = 0xFFFFFFFF;
            }
            break;
            }
        }
    }
    aos_mutex_unlock(&rs485_1_mux);
    return errCode;
}

/**
 * @brief       freq_clear_error
 * @param[in]	index                   //电机驱动设备列表索引
 * @return      int
 */
static int xp_osal_freq_clear_error(int index)
{
    int ret = -1;

    if(0 == aos_mutex_lock(&rs485_1_mux, 100)){
        for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
        {
            if (DRIVER_TYPE_VFD_LNOVANCE == MotorDev_Table[i].drvType && index == MotorDev_Table[i].drvIndex) {
                ret = xp_lnovance_clear_error(MotorDev_Table[i].matchVfdRegNum);
                break;
            }
        }
    }
    aos_mutex_unlock(&rs485_1_mux);
    return ret;
}

/*                                                         =======================                                                         */
/* ========================================================      .c初始化接口      ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief
 * @return      int
 */
int xp_osal_init(void) {
    int sta = 0;

    osal_dev_io_state_change(BOARD1_OUTPUT_SAFE_RELAY_RESET, IO_ENABLE);
    aos_msleep(1000);
    osal_dev_io_state_change(BOARD1_OUTPUT_SAFE_RELAY_RESET, IO_DISABLE); //主电源上电后电气自锁，使能1S后断开

    //voice config
    agDriver.voice.init = xp_osal_voice_init;
    agDriver.voice.play = xp_osal_voice_play;

    //display config
    agDriver.screen.init    = xp_osal_display_init;
    agDriver.screen.display = xp_osal_display_set;

    //freq config
    agDriver.freq.init          = xp_osal_freq_init;
    agDriver.freq.run           = xp_osal_freq_run;
    agDriver.freq.getCurrent    = xp_osal_freq_get_current;
    agDriver.freq.getState      = xp_osal_freq_get_state;
    agDriver.freq.getError      = xp_osal_freq_get_error;
    agDriver.freq.clearError    = xp_osal_freq_clear_error;

    osal_error_upload = NULL;
    offline_payment = NULL;

    aos_mutex_new(&rs485_1_mux);

    //注册子板的脉冲计数相关引脚
    static uint8_t ioBoardRegDevNum[BOARD_NUMS - 1] = {0};
    uint8_t ioBoardNum = 0;
    for (uint8_t i = 0; i < ENCODE_TABLE_NUM; i++)
    {
        //配置的子板号从脉冲引脚索引号中获取
        for (uint8_t j = 0; j < MOTOR_TABLE_NUM; j++)
        {
            //从 MotorDev_Table[] 中查找对应的码盘信息赋值引脚号
            if(Encoder_Table[i].drvIndex == MotorDev_Table[j].drvIndex){
                if(DRIVER_TYPE_VFD_LNOVANCE == MotorDev_Table[j].drvType){
                    //汇川变频器的 ioIndexCW 和 ioIndexCCW 从 Lnovance_Table[] 中取，已经解析了引脚Id号，后面重新解析一遍也不影响
                    Encoder_Table[i].ioIndexCW  = (Type_OutputIo_Enum)(Lnovance_Table[MotorDev_Table[j].matchVfdRegNum].DI1.pinId);
                    Encoder_Table[i].ioIndexCCW = (Type_OutputIo_Enum)(Lnovance_Table[MotorDev_Table[j].matchVfdRegNum].DI2.pinId);
                }
                else{
                    Encoder_Table[i].ioIndexCW  = MotorDev_Table[j].ioIndexCW;
                    Encoder_Table[i].ioIndexCCW = MotorDev_Table[j].ioIndexCCW;
                }
                Encoder_Table[i].ioIndexZero = MotorDev_Table[j].moveInfo.ioIndexLimitCCW;
                break;
            }
        }
        //配置子板的引脚功能
        ioBoardNum = BOARD_ID_RESOLUTION(Encoder_Table[i].ioIndexPulse);
        xp_io_config_crl_pin(ioBoardNum,
                             PIN_ID_RESOLUTION(Encoder_Table[i].ioIndexZero),
                             PIN_ID_RESOLUTION(Encoder_Table[i].ioIndexPulse),
                             PIN_ID_RESOLUTION(Encoder_Table[i].ioIndexCW),
                             PIN_ID_RESOLUTION(Encoder_Table[i].ioIndexCCW));
        //这个值跟子板分配的设备id号需一致
        if(ioBoardRegDevNum[ioBoardNum - 1] >= MAX_REG_DEV_NUMS){
            LOG_FATAL("Dev num is too much, please check");
            return -1;
        }
        Encoder_Table[i].ioBoardRegIndex = ioBoardRegDevNum[ioBoardNum - 1]++;
    }
    
    aos_task_new("osal_encoder_update", xp_osal_encoder_pulse_update_thread,    NULL, 2048);
    aos_task_new("osal_dev_run",        xp_osal_dev_run_thread,                 NULL, 2048);
    aos_task_new("signal_trigger_sta",	signal_trigger_state_thread,            NULL, 2048);
    aos_task_new("osal_water_crl",      xp_osal_water_system_crl_thread,        NULL, 2048);
    // aos_task_new("osal_io_trigger", xp_osal_io_pulse_trigger_thread, NULL, 1024);

    // agDriver.ht_sens.init = xp_osal_ht_sens_init;
    // agDriver.ht_sens.get = xp_osal_ht_sens_get;
    // agDriver.ht_sens.read = xp_osal_ht_sens_read;

    agDriver.init = 1;

    if (agDriver.voice.init() != 0) {
        sta |= -1;
    }
    if (agDriver.screen.init() != 0) {
        sta |= -2;
    }
    if (agDriver.freq.init() != 0) {
        sta |= -4;
    }
    // if(agDriver.ht_sens.init() != 0){
    //     sta |= -8;
    // }
    println("osal init complete !");
    return sta;
}

/**
 * @brief       osal_deinit
 */
void xp_ag_osal_deinit(void) {
    memset(&agDriver, 0, sizeof(Type_Driver_Def));
}

/**
 * @brief
 * @return      Type_Driver_Def*
 */
Type_Driver_Def* xp_ag_osal_get(void) {
    return &agDriver;
}

// static int xp_osal_ht_sens_init(void)
// {
//     return xp_pr3002_init(&pr3002_List, sizeof(pr3002_List) / sizeof(pr3002_List[0])) ? (0) : (-1);
// }

// /*
// lite temperature get
// */
// static int xp_osal_ht_sens_get(uint8_t listId, float * pHumidity, float * pTemperature)
// {
//     return xp_pr3002_get_H_T(listId, pHumidity, pTemperature) ? (0) : (-1);
// }

// /*
// lite temperature read
// */
// static int xp_osal_ht_sens_read(uint8_t listId, float * pHumidity, float * pTemperature)
// {
//     return xp_pr3002_read_H_T(listId, pHumidity, pTemperature) ? (0) : (-1);
// }

/*                                                         =======================                                                         */
/* ========================================================    传感器触发监测接口   ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       传感器触发检测（1为IO的默认状态值，为0时表示触发）
 * @param[in]	index               IO索引
 * @return      bool                
 */
bool osal_is_io_trigger(Type_InputIo_Enum index)
{
    uint8_t boardId = BOARD_ID_RESOLUTION(index);
    uint8_t pinId   = PIN_ID_RESOLUTION(index);

    if((0 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD_IO_INPUT_PIN_NUM))
    || (1 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD0_INPUT_PIN_NUM))
    || (2 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD1_INPUT_PIN_NUM))
    || (3 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD2_INPUT_PIN_NUM))
    || (4 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD3_INPUT_PIN_NUM))
    || (5 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD4_INPUT_PIN_NUM))){
        LOG_UPLOAD("Index %d board %d is not registered", index, boardId);
        return false;
    }

    if(0 == boardId){
        if(0 == xp_boardIO_get(pinId + 1)){
            boardIoInSta[pinId] = 0;        //提供数组存储状态，用在及时性不高的地方（主板IO是I2C扩展的，不稳定，尽量减少读取）
            return true;
        }
        else{
            boardIoInSta[pinId] = 1;
            return false;
        }
    }
    else{
        //这里只对侧刷低位做特殊处理，需要在动作过程中做限位判断
        if(BOARD2_INPUT_FRONT_LEFT_BRUSH_DOWN == index
        || BOARD2_INPUT_FRONT_RIGHT_BRUSH_DOWN == index
        || BOARD2_INPUT_BACK_LEFT_BRUSH_DOWN == index
        || BOARD2_INPUT_BACK_RIGHT_BRUSH_DOWN == index){
            return 1 == xp_io_read_input_pin(boardId, pinId) ? true : false;
        }
        else{
            return 0 == xp_io_read_input_pin(boardId, pinId) ? true : false;
        }
    }
}

/*                                                         =======================                                                         */
/* ========================================================       水系统控制       ======================================================== */
/*                                                         =======================                                                         */

typedef struct
{
    bool        waterSta[WATER_CRL_NUM];            //水系统状态
    bool        recordSta[WATER_CRL_NUM];           //记录的水系统状态
    uint16_t    matchIo[WATER_CRL_NUM];
    aos_sem_t   water_sem;
} Type_WaterCrlInfo_Def;
Type_WaterCrlInfo_Def waterCrl = {0};

/**
 * @brief       水系统控制，遵循先开阀，再开泵；先关泵，再关阀的原则
 * @param[in]	arg                 
 */
void xp_osal_water_system_crl_thread(void* arg)
{
    bool waterSta[WATER_CRL_NUM];    //线程里使用局部变量，避免全局的改变影响此处判断
    aos_sem_new(&waterCrl.water_sem, 0);

    //只赋值相应的水路IO，其它水系统控制类型保持默认值0
    waterCrl.matchIo[WATER_HIGH_PRESS_WATER]    = BOARD4_OUTPUT_SWING_VALVE;
    waterCrl.matchIo[WATER_WAXWATER]            = BOARD1_OUTPUT_WAXWATER_VALVE;
    waterCrl.matchIo[WATER_SHAMPOO]             = BOARD1_OUTPUT_SHAMPOO_WATER_VALVE;
    waterCrl.matchIo[WATER_TOP]                 = BOARD1_OUTPUT_TOP_WATER_VALVE;
    waterCrl.matchIo[WATER_FRONT_SIDE]          = BOARD1_OUTPUT_FRONT_BRUSH_WATER_VALVE;
    waterCrl.matchIo[WATER_BACK_SIDE]           = BOARD1_OUTPUT_BACK_BRUSH_WATER_VALVE;
    // waterCrl.matchIo[WATER_CONVEYOR_1]  = BOARD1_OUTPUT_WATER_CONVEYOR_1_VALVE;
    // waterCrl.matchIo[WATER_CONVEYOR_2]  = BOARD1_OUTPUT_WATER_CONVEYOR_2_VALVE;
    // waterCrl.matchIo[WATER_CONVEYOR_3]  = BOARD1_OUTPUT_WATER_CONVEYOR_3_VALVE;
    // waterCrl.matchIo[WATER_MIDDLE]      = BOARD1_OUTPUT_MIDDLE_WATER_VALVE;
    waterCrl.matchIo[WATER_SHAMPOO_PIKN]        = BOARD1_OUTPUT_SHAMPOO_PINK_VALVE;
    waterCrl.matchIo[WATER_SHAMPOO_GREEN]       = BOARD1_OUTPUT_SHAMPOO_GREEN_VALVE;
    waterCrl.matchIo[WATER_CLEAR_WATER]         = BOARD1_OUTPUT_CLEAR_WATER_VALVE;
    waterCrl.matchIo[WATER_BASE_PLATE]          = BOARD1_OUTPUT_BASE_PLATE_WASH_VALVE;
    while (1)
    {
        aos_sem_wait(&waterCrl.water_sem, AOS_WAIT_FOREVER);
        memcpy(waterSta, waterCrl.waterSta, sizeof(waterCrl.waterSta));
        if(waterCrl.recordSta[WATER_DRAIN] != waterSta[WATER_DRAIN]){
            waterCrl.recordSta[WATER_DRAIN] = waterSta[WATER_DRAIN];
            if(true == waterSta[WATER_DRAIN]){
                isPumpWorking = false;
                osal_dev_io_state_change(BOARD0_OUTPUT_LOW_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PINK_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_GREEN_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_WAXWATER_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD0_OUTPUT_SHAMPOO_AIR_VALVE, IO_DISABLE);
                aos_msleep(500);
                for (uint8_t i = 0; i < WATER_CRL_NUM; i++)
                {
                    if(waterCrl.matchIo[i]){
                        osal_dev_io_state_change(waterCrl.matchIo[i], IO_DISABLE);
                        waterCrl.recordSta[i] = false;
                    }
                }
                osal_dev_io_state_change(BOARD1_OUTPUT_DRAIN_WATER_VALVE, IO_ENABLE);
            }
            else{
                osal_dev_io_state_change(BOARD1_OUTPUT_DRAIN_WATER_VALVE, IO_DISABLE);
                for (uint8_t i = 0; i < WATER_CRL_NUM; i++)
                {
                    if(waterCrl.matchIo[i]){
                        osal_dev_io_state_change(waterCrl.matchIo[i], IO_DISABLE);
                        waterCrl.recordSta[i] = false;
                    }
                }
            }
            continue;           //操作排水排气时不再操作其它，需要记录当前的水相关状态记录
        }
        
        //高压水只有一路单独控制
        if(waterCrl.recordSta[WATER_HIGH_PRESS_WATER] != waterSta[WATER_HIGH_PRESS_WATER]){
            waterCrl.recordSta[WATER_HIGH_PRESS_WATER] = waterSta[WATER_HIGH_PRESS_WATER];
            if(true == waterSta[WATER_HIGH_PRESS_WATER]){
                osal_dev_io_state_change(BOARD4_OUTPUT_SWING_VALVE, IO_ENABLE);
                aos_msleep(500);
                osal_dev_io_state_change(BOARD4_OUTPUT_TOP_SWING, IO_ENABLE);
                osal_dev_io_state_change(BOARD0_OUTPUT_HIGH_PUMP, IO_ENABLE);
            }
            else{
                osal_dev_io_state_change(BOARD0_OUTPUT_HIGH_PUMP, IO_DISABLE);
                aos_msleep(500);
                osal_dev_io_state_change(BOARD4_OUTPUT_SWING_VALVE, IO_DISABLE);
                osal_dev_io_state_change(BOARD4_OUTPUT_TOP_SWING, IO_DISABLE);
            }
            continue;
        }

        //低压泵的水系统控制
        bool isWaterOpen = false;
        bool isNewWaterOpen = false;                    //看是否有新的阀需要开启
        for (uint8_t i = 0; i < WATER_CRL_NUM; i++)
        {
            if(waterCrl.matchIo[i]){
                isWaterOpen |= waterSta[i];
                if(waterCrl.recordSta[i] != waterSta[i]){
                    waterCrl.recordSta[i] = waterSta[i];
                    if(true == waterSta[i]){
                        isNewWaterOpen = true;
                        //有些水路需要额外使能一些点位
                        if(WATER_WAXWATER == i){
                            osal_dev_io_state_change(BOARD1_OUTPUT_WAXWATER_PUMP, IO_ENABLE);
                            osal_dev_io_state_change(BOARD1_OUTPUT_DRIER_VALVE, IO_ENABLE);
                        }
                        else if(WATER_SHAMPOO == i){
                            osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PUMP, IO_ENABLE);
                        }
                        else if(WATER_SHAMPOO_PIKN == i){
                            osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PINK_PUMP, IO_ENABLE);
                        }
                        else if(WATER_SHAMPOO_GREEN == i){
                            osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_GREEN_PUMP, IO_ENABLE);
                        }
                        if(true == waterSta[WATER_SHAMPOO] || true == waterSta[WATER_SHAMPOO_PIKN] || true == waterSta[WATER_SHAMPOO_GREEN]){
                            osal_dev_io_state_change(BOARD0_OUTPUT_SHAMPOO_AIR_VALVE, IO_ENABLE);
                        }
                        osal_dev_io_state_change(waterCrl.matchIo[i], IO_ENABLE);       //先开启需要开启的水路阀，保持水泵有压力释放口
                    }
                }
            }
        }

        if(isWaterOpen){
            if(isNewWaterOpen)  aos_msleep(200);            //有新水路开时，延时一段时间再关闭不开的水路，避免水路切换瞬间所有水路阀都关闭
            for (uint8_t i = 0; i < WATER_CRL_NUM; i++)
            {
                if(waterCrl.matchIo[i] && false == waterSta[i]){
                    //有些水路需要额外使能一些点位
                    if(WATER_WAXWATER == i){
                        osal_dev_io_state_change(BOARD1_OUTPUT_WAXWATER_PUMP, IO_DISABLE);
                        osal_dev_io_state_change(BOARD1_OUTPUT_DRIER_VALVE, IO_DISABLE);
                    }
                    else if(WATER_SHAMPOO == i){
                        osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PUMP, IO_DISABLE);
                    }
                    else if(WATER_SHAMPOO_PIKN == i){
                        osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PINK_PUMP, IO_DISABLE);
                    }
                    else if(WATER_SHAMPOO_GREEN == i){
                        osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_GREEN_PUMP, IO_DISABLE);
                    }
                    if(false == waterSta[WATER_SHAMPOO] && false == waterSta[WATER_SHAMPOO_PIKN] && false == waterSta[WATER_SHAMPOO_GREEN]){
                        osal_dev_io_state_change(BOARD0_OUTPUT_SHAMPOO_AIR_VALVE, IO_DISABLE);
                    }
                    osal_dev_io_state_change(waterCrl.matchIo[i], IO_DISABLE);
                }
            }
            if(!isPumpWorking){                             //之前没开启水泵就开启水泵
                aos_msleep(300);                            //延时一段时间后开启水泵
                osal_dev_io_state_change(BOARD0_OUTPUT_LOW_PUMP, IO_ENABLE);
            }
            isPumpWorking = true;
        }
        else{
            if(isPumpWorking){
                isPumpWorking = false;
                osal_dev_io_state_change(BOARD0_OUTPUT_LOW_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PINK_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_GREEN_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_WAXWATER_PUMP, IO_DISABLE);
                osal_dev_io_state_change(BOARD0_OUTPUT_SHAMPOO_AIR_VALVE, IO_DISABLE);
                osal_dev_io_state_change(BOARD1_OUTPUT_DRIER_VALVE, IO_DISABLE);
                aos_msleep(500);
                for (uint8_t i = 0; i < WATER_CRL_NUM; i++)
                {
                    if(waterCrl.matchIo[i]){
                        osal_dev_io_state_change(waterCrl.matchIo[i], IO_DISABLE);
                        waterCrl.recordSta[i] = false;
                    }
                }
            }
        }
    }
}

/**
 * @brief       水系统相关控制
 * @param[in]	type                控制类型
 * @param[in]	enable              使能
 * @return      void
 */
void water_system_control(Type_WaterSystem_Enum type, bool enable)
{
    if(WATER_HIGH_PUMP == type){
       osal_dev_io_state_change(BOARD0_OUTPUT_HIGH_PUMP, enable ? IO_ENABLE : IO_DISABLE);
    }
    else if(WATER_LOW_PUMP == type){
       osal_dev_io_state_change(BOARD0_OUTPUT_LOW_PUMP, enable ? IO_ENABLE : IO_DISABLE);
    }
    else if(WATER_SEWAGE_PUMP == type){
    //    osal_dev_io_state_change(BOARD1_OUTPUT_SEWAGE_PUMP, enable ? IO_ENABLE : IO_DISABLE);
    }
    else if(WATER_SHAMPOO_PUMP == type){
       osal_dev_io_state_change(BOARD1_OUTPUT_SHAMPOO_PUMP, enable ? IO_ENABLE : IO_DISABLE);
    }
    else if(WATER_WAXWATER_PUMP == type){
       osal_dev_io_state_change(BOARD1_OUTPUT_WAXWATER_PUMP, enable ? IO_ENABLE : IO_DISABLE);
    }
    else{
        if(WATER_ALL == type){
           waterCrl.waterSta[WATER_HIGH_PRESS_WATER]    = false;
           waterCrl.waterSta[WATER_WAXWATER]            = false;
           waterCrl.waterSta[WATER_SHAMPOO]             = false;
           waterCrl.waterSta[WATER_TOP]                 = false;
           waterCrl.waterSta[WATER_FRONT_SIDE]          = false;
           waterCrl.waterSta[WATER_BACK_SIDE]           = false;
        //    waterCrl.waterSta[WATER_CONVEYOR_1] = false;
        //    waterCrl.waterSta[WATER_CONVEYOR_2] = false;
        //    waterCrl.waterSta[WATER_CONVEYOR_3] = false;
        //    waterCrl.waterSta[WATER_MIDDLE]     = false;
           waterCrl.waterSta[WATER_SHAMPOO_PIKN]        = false;
           waterCrl.waterSta[WATER_SHAMPOO_GREEN]       = false;
           waterCrl.waterSta[WATER_CLEAR_WATER]         = false;
           waterCrl.waterSta[WATER_BASE_PLATE]          = false;
        }
        else{
           waterCrl.waterSta[type] = enable;
        }
        aos_sem_signal(&waterCrl.water_sem);
    }
}

/*                                                         =======================                                                         */
/* ========================================================     信号触发状态监测   ======================================================== */
/*                                                         =======================                                                         */

//这里的信号触发判定时间较长，对于需要立即响应的信号，不取这里的状态判定值（如报警状态和限位触发）
//信号信息匹配表
Type_SignalStaInfo_Def SignalInfo_Table[] = {
//  信号类型                     //对应的检测引脚                         //信号状态       //信号确认次数、靠近时的位置、离开时的位置
    {SIGNAL_GROUND,             BOARD4_INPUT_RESERVE_1,                 SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_ALL_IN,             BOARD4_INPUT_ALL_IN_SIGNAL,             SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_LEFT_SKEW,          BOARD4_INPUT_STOP_LEFT_SKEW,            SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_RIGHT_SKEW,         BOARD4_INPUT_STOP_RIGHT_SKEW,           SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_SUPER_HIGH,         BOARD4_INPUT_SUPER_HIGH,                SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_STOP,               BOARD4_INPUT_CAR_STOP_SIGNAL,           SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_ENTRANCE,           BOARD4_INPUT_ENTRANCE_SIGNAL,           SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_REAR_END_PROTECT,   BOARD0_INPUT_SIGNAL_REAR_END_PROTECT,   SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_AVOID_INTRUDE,      BOARD4_INPUT_AVOID_INTRUDE_SIGNAL,      SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_EXIT,               BOARD0_INPUT_SIGNAL_EXIT,               SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FINISH,             BOARD0_INPUT_SIGNAL_FINISH,             SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_LIFTER_UP,          BOARD4_INPUT_LIFTER_UP,                 SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_LIFTER_DOWN,        BOARD4_INPUT_LIFTER_DOWN,               SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FL_BRUSH_DOWN,      BOARD2_INPUT_FRONT_LEFT_BRUSH_DOWN,     SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FR_BRUSH_DOWN,      BOARD2_INPUT_FRONT_RIGHT_BRUSH_DOWN,    SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BL_BRUSH_DOWN,      BOARD2_INPUT_BACK_LEFT_BRUSH_DOWN,      SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BR_BRUSH_DOWN,      BOARD2_INPUT_BACK_RIGHT_BRUSH_DOWN,     SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FL_MOVE_ZERO,       BOARD2_INPUT_FRONT_LEFT_MOVE_ZERO,      SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FR_MOVE_ZERO,       BOARD2_INPUT_FRONT_RIGHT_MOVE_ZERO,     SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BL_MOVE_ZERO,       BOARD2_INPUT_BACK_LEFT_MOVE_ZERO,       SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BR_MOVE_ZERO,       BOARD2_INPUT_BACK_RIGHT_MOVE_ZERO,      SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FL_BRUSH_CROOKED,   BOARD2_INPUT_FRONT_RIGHT_BRUSH_CROOKED, SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FR_BRUSH_CROOKED,   BOARD2_INPUT_FRONT_RIGHT_BRUSH_CROOKED, SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BL_BRUSH_CROOKED,   BOARD2_INPUT_BACK_RIGHT_BRUSH_CROOKED,  SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BR_BRUSH_CROOKED,   BOARD2_INPUT_BACK_RIGHT_BRUSH_CROOKED,  SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_LEFT_SKIRT_ZERO,    BOARD4_INPUT_FRONT_LEFT_SKIRT_ZERO,     SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_RIGHT_SKIRT_ZERO,   BOARD4_INPUT_FRONT_RIGHT_SKIRT_ZERO,    SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_LIFTER_LEFT_DETECH, BOARD4_INPUT_LIFTER_LEFT_DETECH,        SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_LIFTER_RIGHT_DETECH,BOARD4_INPUT_LIFTER_RIGHT_DETECH,       SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FL_COLLISION,       BOARD4_INPUT_FRONT_LEFT_COLLISION,      SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_FR_COLLISION,       BOARD4_INPUT_FRONT_RIGHT_COLLISION,     SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BL_COLLISION,       BOARD0_INPUT_BACK_LEFT_COLLISION,       SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BR_COLLISION,       BOARD0_INPUT_BACK_RIGHT_COLLISION,      SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_GATE_1_CLOSE,       BOARD5_INPUT_GATE_1_CLOSE_DONE,         SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_GATE_1_OPEN,        BOARD5_INPUT_GATE_1_OPEN_DONE,          SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_GATE_2_LEFT_OPEN,   BOARD5_INPUT_GATE_2_LEFT_OPEN_DONE,     SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_GATE_2_RIGHT_OPEN,  BOARD5_INPUT_GATE_2_RIGHT_OPEN_DONE,    SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BUTTON_RESET,       BOARD0_INPUT_BUTTON_RESET,              SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BUTTON_START,       BOARD0_INPUT_BUTTON_START,              SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BUTTON_STOP,        BOARD0_INPUT_BUTTON_STOP,               SIGNAL_STABLE,  3,    0,  0},
    {SIGNAL_BUTTON_PAUSE,       BOARD0_INPUT_BUTTON_PAUSE_RESUME,       SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_BUTTON_ENTRY_START, BOARD4_INPUT_BUTTON_START_ENTRY,        SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_PICKUP_TRUCK,       BOARD4_INPUT_PICKUP_TRUCK_SIGNAL,       SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_CAR_WHEEL_OUT,      BOARD1_FINISH_AREA_WHEEL_EXIT,          SIGNAL_STABLE,  20,   0,  0},
    {SIGNAL_WASH_MODE_1,        BOARD5_INPUT_WASH_MODE_1,               SIGNAL_STABLE,  5,    0,  0},
    {SIGNAL_WASH_MODE_2,        BOARD5_INPUT_WASH_MODE_2,               SIGNAL_STABLE,  5,    0,  0},
    {SIGNAL_WASH_MODE_3,        BOARD5_INPUT_WASH_MODE_3,               SIGNAL_STABLE,  5,    0,  0},
    {SIGNAL_WASH_MODE_4,        BOARD5_INPUT_WASH_MODE_4,               SIGNAL_STABLE,  5,    0,  0},
};

/**
 * @brief       光电信号触发判断（多次滤波判定）
 * @param[in]	signalType          光电信号类型
 */
static int signal_flip_dir(Type_SignalType_Enum signalType)
{
    static bool lasSignalState[SIGNAL_NUM] = {0};
    static bool isStartConfirmSignal[SIGNAL_NUM] = {0};
    static uint8_t signalFlipStableCnt[SIGNAL_NUM] = {0};
    bool signalState = osal_is_io_trigger(SignalInfo_Table[signalType].matchIo);

    if(false == isStartConfirmSignal[signalType]){
        if(signalState != lasSignalState[signalType]) isStartConfirmSignal[signalType] = true;
        signalFlipStableCnt[signalType] = 0;
    }else{
        // LOG_DEBUG("Check signal %d, now state %d, last_state %d",signalType, signalState, lasSignalState[signalType]);
        if(signalState != lasSignalState[signalType]){
            isStartConfirmSignal[signalType] = false;
        }
        else{
            signalFlipStableCnt[signalType]++;
            if(signalFlipStableCnt[signalType] >= SignalInfo_Table[signalType].trigCnt){
                signalFlipStableCnt[signalType] = SignalInfo_Table[signalType].trigCnt;     //防止溢出

                isStartConfirmSignal[signalType] = false;			                //确认完后退出确认，避免影响下一次检测
                lasSignalState[signalType] = signalState;

                return (true == signalState) ? IO_IS_TRIGGER : IO_NO_TRIGGER;
            }
        }
    }
    lasSignalState[signalType] = signalState;
    return SIGNAL_STABLE;
}

/**
 * @brief       信号触发监测线程（有信号滤波处理）
 * @param[in]	arg                 
 */
void signal_trigger_state_thread(void *arg)
{
    while (1)
    {
        for (uint8_t i = 0; i < SIGNAL_NUM; i++)
        {
            int trigDir = signal_flip_dir(i);
            if(IO_IS_TRIGGER == trigDir){
                switch (i)
                {
                case SIGNAL_ENTRANCE:
                    SignalInfo_Table[i].closePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_ENTRANCE trig close, Pos %d", SignalInfo_Table[i].closePos);
                    break;
                case SIGNAL_REAR_END_PROTECT:
                    SignalInfo_Table[i].closePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_REAR_END_PROTECT trig close, Pos %d", SignalInfo_Table[i].closePos);
                    break;
                case SIGNAL_AVOID_INTRUDE:
                    SignalInfo_Table[i].closePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_AVOID_INTRUDE trig close, Pos %d", SignalInfo_Table[i].closePos);
                    break;
                case SIGNAL_EXIT:
                    SignalInfo_Table[i].closePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_EXIT trig close, Pos %d", SignalInfo_Table[i].closePos);
                    break;
                case SIGNAL_FINISH:
                    SignalInfo_Table[i].closePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_FINISH trig close, Pos %d", SignalInfo_Table[i].closePos);
                    break;
                case SIGNAL_PICKUP_TRUCK:
                    SignalInfo_Table[i].closePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_PICKUP_TRUCK trig close, Pos %d", SignalInfo_Table[i].closePos);
                    break;
                case SIGNAL_WASH_MODE_1:
                    if(offline_payment) offline_payment(1);
                    break;
                case SIGNAL_WASH_MODE_2:
                    if(offline_payment) offline_payment(2);
                    break;
                case SIGNAL_WASH_MODE_3:
                    if(offline_payment) offline_payment(3);
                    break;
                case SIGNAL_WASH_MODE_4:
                    if(offline_payment) offline_payment(4);
                    break;
                default:
                    break;
                }
                SignalInfo_Table[i].trigDir = trigDir;      //赋值当前信号源触发状态
            }
            else if(IO_NO_TRIGGER == trigDir){
                switch (i)
                {
                case SIGNAL_ENTRANCE:
                    SignalInfo_Table[i].leavePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_ENTRANCE trig leave, Pos %d", SignalInfo_Table[i].leavePos);
                    break;
                case SIGNAL_REAR_END_PROTECT:
                    SignalInfo_Table[i].leavePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_REAR_END_PROTECT trig leave, Pos %d", SignalInfo_Table[i].leavePos);
                    break;
                case SIGNAL_AVOID_INTRUDE:
                    SignalInfo_Table[i].leavePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_AVOID_INTRUDE trig leave, Pos %d", SignalInfo_Table[i].leavePos);
                    break;
                case SIGNAL_EXIT:
                    SignalInfo_Table[i].leavePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_EXIT trig leave, Pos %d", SignalInfo_Table[i].leavePos);
                    break;
                case SIGNAL_FINISH:
                    SignalInfo_Table[i].leavePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_FINISH trig leave, Pos %d", SignalInfo_Table[i].leavePos);
                    break;
                case SIGNAL_PICKUP_TRUCK:
                    SignalInfo_Table[i].leavePos = xp_osal_get_dev_pos(CONVEYOR_2_MATCH_ID);
                    LOG_UPLOAD("SIGNAL_PICKUP_TRUCK trig leave, Pos %d", SignalInfo_Table[i].leavePos);
                    break;
                default:
                    break;
                }
                SignalInfo_Table[i].trigDir = trigDir;      //赋值当前信号源触发状态
            }
        }
        aos_msleep(SIGNAL_TRIGGER_THREAD_FREQ);
    }
}

Type_SignalStaInfo_Def *get_signal_handle(Type_SignalType_Enum type)
{
    return &SignalInfo_Table[type];
}

/**
 * @brief       返回信号触发状态（经过滤波）
 * @param[in]	type                
 * @return      bool                
 */
bool is_signal_filter_trigger(Type_SignalType_Enum type)
{
    //对射类光电正常电平状态为0，遮挡后为1认为触发了
    if(SIGNAL_GROUND == type
    || SIGNAL_ALL_IN == type
    // || SIGNAL_LEFT_SKEW == type
    // || SIGNAL_RIGHT_SKEW == type
    || SIGNAL_SUPER_HIGH == type
    || SIGNAL_STOP == type
    // || SIGNAL_ENTRANCE == type
    // || SIGNAL_REAR_END_PROTECT == type
    // || SIGNAL_AVOID_INTRUDE == type
    || SIGNAL_EXIT == type
    || SIGNAL_FINISH == type
    || SIGNAL_LIFTER_LEFT_DETECH == type
    || SIGNAL_LIFTER_RIGHT_DETECH == type
    || SIGNAL_FL_BRUSH_CROOKED == type
    || SIGNAL_FR_BRUSH_CROOKED == type
    || SIGNAL_BL_BRUSH_CROOKED == type
    || SIGNAL_BR_BRUSH_CROOKED == type
    || SIGNAL_FL_COLLISION == type
    || SIGNAL_FR_COLLISION == type
    || SIGNAL_BL_COLLISION == type
    || SIGNAL_BR_COLLISION == type
    || SIGNAL_CAR_WHEEL_OUT == type){
    // || SIGNAL_PICKUP_TRUCK == type
    // || SIGNAL_FL_BRUSH_DOWN == type          //侧刷低位已在IO检测处取反（便于限位停止移动判定）
    // || SIGNAL_FR_BRUSH_DOWN == type
    // || SIGNAL_BL_BRUSH_DOWN == type
    // || SIGNAL_BR_BRUSH_DOWN == type){
        return (IO_IS_TRIGGER == SignalInfo_Table[type].trigDir) ? false : true;
    }
    else{
        return (IO_NO_TRIGGER == SignalInfo_Table[type].trigDir) ? false : true;
    }
}

/*                                                         =======================                                                         */
/* ========================================================   设备单IO输出控制接口  ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       设置注册IO的引脚状态
 * @param[in]	index               
 * @param[in]	sta                 
 */
void osal_dev_io_state_change(Type_OutputIo_Enum index, bool sta)
{
    uint8_t boardId = BOARD_ID_RESOLUTION(index);
    uint8_t pinId   = PIN_ID_RESOLUTION(index);

    if((0 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD_IO_OUTPUT_PIN_NUM))
    || (1 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD0_OUTPUT_PIN_NUM))
    || (2 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD1_OUTPUT_PIN_NUM))
    || (3 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD2_OUTPUT_PIN_NUM))
    || (4 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD3_OUTPUT_PIN_NUM))
    || (5 == boardId && pinId >= PIN_ID_RESOLUTION(BOARD4_OUTPUT_PIN_NUM))){
        LOG_UPLOAD("Index %d board %d is not registered", index, boardId);
        return;
    }

    if(0 == boardId){
        xp_boardIO_set(pinId + 1, sta);
        boardIoOutSta[pinId] = sta ? IO_DISABLE : IO_ENABLE;            //IO默认状态为1，置0时为使能状态
    }
    else{
        xp_io_write_pin(boardId, pinId, sta);
    }
}

/*                                                         =======================                                                         */
/* ========================================================     io脉冲触发控制     ======================================================== */
/*                                                         =======================                                                         */

//只负责电平关闭，使能还是在驱动的时候去使能
void xp_osal_io_pulse_trigger_thread(void *arg)
{
    while(1){
        //非 MotorDev_Table 里管理的脉冲IO控制

        // MotorDev_Table 里管理的脉冲IO控制
        for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
        {
            if (DRIVER_TYPE_TRIGGER == MotorDev_Table[i].drvType && MotorDev_Table[i].moveInfo.isTriggerEnable
            && get_diff_ms(MotorDev_Table[i].moveInfo.timeStamp) > MotorDev_Table[i].moveInfo.triggerTime) {
                MotorDev_Table[i].moveInfo.isTriggerEnable = false;
                xp_io_write_pin(BOARD_ID_RESOLUTION(MotorDev_Table[i].moveInfo.ioIndexTrig), \
                                PIN_ID_RESOLUTION(MotorDev_Table[i].moveInfo.ioIndexTrig), IO_DISABLE);
            }
        }
        aos_msleep(300);
    }
}

/*                                                         =======================                                                         */
/* ========================================================      码盘位置更新      ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       获取指定ID电机的码盘计数值（记录的值）
 * @param[in]	id                  电机驱动索引id
 * @return      int32_t
 */
int32_t xp_osal_get_dev_pos(Type_DriverIndex_Enum id)
{
    for (uint8_t i = 0; i < ENCODE_TABLE_NUM; i++)
    {
        if (id == Encoder_Table[i].drvIndex) {
            return Encoder_Table[i].encoderValue;
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return NULL_ENCODER;
}

/**
 * @brief       更新已注册码盘的位置值（仅支持由子板记录脉冲值的码盘）
 * @param[in]	arg
 */
void xp_osal_encoder_pulse_update_thread(void* arg)
{
    uint8_t logCnt = 0;

    aos_msleep(3000);
    while (1)
    {
        for (uint8_t board = 1; board < BOARD_NUMS; board++)      //第一块子板由1开始
        { 
            for (uint8_t j = 0; j < ENCODE_TABLE_NUM; j++)
            {
                if(board == BOARD_ID_RESOLUTION(Encoder_Table[j].ioIndexPulse)){    //依次查找列表中对应的子板，并更新子板注册设备的码盘值
                    if(Encoder_Table[j].ioBoardRegIndex != NONE_REG_INFO){
                        xp_io_get_dev_encoder(board, Encoder_Table[j].ioBoardRegIndex, &Encoder_Table[j].encoderValue);
                    }
                }
            }
        }
        if(logCnt++ % 100 == 0){
            for (uint8_t i = 0; i < ENCODE_TABLE_NUM; i++)
            {
                if(Encoder_Table[i].lastEncoderValue != Encoder_Table[i].encoderValue){
                    Encoder_Table[i].lastEncoderValue = Encoder_Table[i].encoderValue;
                    LOG_UPLOAD("*****%s encoder upadte to %d", xp_get_encoder_str(i), Encoder_Table[i].encoderValue);
                }
            }
        }
        aos_msleep(10);
    }
}

/*                                                         =======================                                                         */
/* ========================================================       状态机控制       ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       获取电机的运动状态
 * @param[in]	id
 * @return      int
 */
Type_MoveState_Enum xp_osal_get_motor_move_state(Type_DriverIndex_Enum id)
{
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (id == MotorDev_Table[i].drvIndex) {
            return MotorDev_Table[i].moveInfo.state;
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/**
 * @brief       状态机切换
 * @param[in]	pMotor              
 * @param[in]	state               需要切换的状态
 * @return      int                 
 */
static int xp_osal_motor_state_set(Type_MotorInfo_Def* const pMotor, Type_MoveState_Enum state)
{
    Type_MoveState_Enum CurrState = pMotor->moveInfo.state;

    if (pMotor->moveInfo.state != state) {

        /*状态迁移图*/
        //空闲->错误/速度模式移动/强制速度移动/位置模式移动/时间模式移动/旋转
        if (MOTOR_STA_IDLE == pMotor->moveInfo.state
        && (MOTOR_STA_FAULT == state || MOTOR_STA_MOVE == state || MOTOR_STA_MOVE_FORE == state || MOTOR_STA_MOVE_POS == state || MOTOR_STA_MOVE_TIME == state || MOTOR_STA_ROTATION == state)) {
            pMotor->moveInfo.state = state;
        }
        //速度模式移动->空闲/暂停/错误/位置模式移动/强制速度移动/时间模式移动
        else if (MOTOR_STA_MOVE == pMotor->moveInfo.state 
        && (MOTOR_STA_IDLE == state || MOTOR_STA_PAUSE == state || MOTOR_STA_FAULT == state || MOTOR_STA_MOVE_FORE == state || MOTOR_STA_MOVE_POS == state || MOTOR_STA_MOVE_TIME == state)) {
            pMotor->moveInfo.state = state;
        }
        //强制速度移动->空闲/暂停/错误
        else if (MOTOR_STA_MOVE_FORE == pMotor->moveInfo.state 
        && (MOTOR_STA_IDLE == state || MOTOR_STA_PAUSE == state || MOTOR_STA_FAULT == state)) {
            pMotor->moveInfo.state = state;
        }
        //位置模式移动->空闲/暂停/错误/速度模式移动/强制速度移动/时间模式移动
        else if (MOTOR_STA_MOVE_POS == pMotor->moveInfo.state 
        && (MOTOR_STA_IDLE == state || MOTOR_STA_PAUSE == state || MOTOR_STA_FAULT == state || MOTOR_STA_MOVE == state || MOTOR_STA_MOVE_FORE == state || MOTOR_STA_MOVE_TIME == state)) {
            pMotor->moveInfo.state = state;
        }
        //时间模式移动->空闲/暂停/错误/速度模式移动/强制速度移动/位置模式移动
        else if (MOTOR_STA_MOVE_TIME == pMotor->moveInfo.state 
        && (MOTOR_STA_IDLE == state || MOTOR_STA_PAUSE == state || MOTOR_STA_FAULT == state || MOTOR_STA_MOVE == state || MOTOR_STA_MOVE_FORE == state || MOTOR_STA_MOVE_POS == state)) {
            pMotor->moveInfo.state = state;
        }
        //暂停->恢复/错误
        else if (MOTOR_STA_PAUSE == pMotor->moveInfo.state 
        && (MOTOR_STA_RESUME == state || MOTOR_STA_FAULT == state)) {
            pMotor->moveInfo.state = state;
        }
        //恢复->错误/速度模式移动/强制速度移动/位置模式移动/时间模式移动
        else if (MOTOR_STA_RESUME == pMotor->moveInfo.state 
        && (MOTOR_STA_FAULT == state || MOTOR_STA_MOVE == state || MOTOR_STA_MOVE_FORE == state || MOTOR_STA_MOVE_POS == state || MOTOR_STA_MOVE_TIME == state)) {
            pMotor->moveInfo.state = state;
        }
        //旋转->空闲
        else if (MOTOR_STA_ROTATION == pMotor->moveInfo.state && (MOTOR_STA_IDLE == state)) {
            pMotor->moveInfo.state = state;
        }
        //停止->空闲/速度模式移动/强制速度移动/位置模式移动/时间模式移动
        else if (MOTOR_STA_STOP == pMotor->moveInfo.state 
        && (MOTOR_STA_IDLE == state || MOTOR_STA_MOVE == state || MOTOR_STA_MOVE_FORE == state || MOTOR_STA_MOVE_POS == state || MOTOR_STA_MOVE_TIME == state || MOTOR_STA_ROTATION == state)){
            pMotor->moveInfo.state = state;
        }
        else if (MOTOR_STA_STOP == state) {	                                        //任意状态都允许到停止
            pMotor->moveInfo.state = state;
        }
        else {
            printf("%s state switch not allowed, %s -> %s\r\n", xp_get_device_str(pMotor - &MotorDev_Table[0]), xp_osal_move_get_state_str(CurrState), xp_osal_move_get_state_str(state));
            return -1;				            //不允许的状态切换
        }
        if (MOTOR_STA_PAUSE != CurrState) {
            pMotor->moveInfo.lastState = CurrState;
        }
        pMotor->moveInfo.isFirstSwitch = true;  //状态切换成功
        //打印非阻塞 ？？在打印较多的时候可能会从一个线程切换到另外一个线程，导致运行逻辑不符合预期
        //开启下面这个打印后，在切换完状态成功后才会设置目标速度，但是由于非阻塞，在打印过程中切换到了状态轮询线程，
        //判断目标速度为0（其实是还没来得及设置），就又把状态切回了停止状态
        // LOG_DEBUG("%s state switch OK, %s -> %s", xp_get_device_str(pMotor - &MotorDev_Table[0]), xp_osal_move_get_state_str(CurrState), xp_osal_move_get_state_str(state));
    }
    else {					                    //已是当前状态
        printf("%s already in %s state\r\n", xp_get_device_str(pMotor - &MotorDev_Table[0]), xp_osal_move_get_state_str(CurrState));
    }
    return 0;
}

/**
 * @brief       设备驱动总接口
 * @param[in]	pMotor              
 * @return      int                 
 */
static int xp_dev_motor_run(Type_MotorInfo_Def* const pMotor)
{
    if (DRIVER_TYPE_VFD_LNOVANCE == pMotor->drvType) {       //汇川变频器驱动
        return xp_osal_freq_run(pMotor->matchVfdRegNum, pMotor->tarVel);
    }
    else if (DRIVER_TYPE_KM == pMotor->drvType) {           //线圈驱动
        if (pMotor->tarVel > 0) {
            if(GATE_1_MACH_ID == pMotor->drvIndex){
                osal_dev_io_state_change(BOARD4_OUTPUT_GATE_1_STOP, IO_DISABLE);
            }
            else if(GATE_3_MACH_ID == pMotor->drvIndex){
                osal_dev_io_state_change(BOARD0_OUTPUT_GATE_3_STOP, IO_DISABLE);
            }
            osal_dev_io_state_change(pMotor->ioIndexCW, IO_ENABLE);
            osal_dev_io_state_change(pMotor->ioIndexCCW, IO_DISABLE);
        }
        else if (pMotor->tarVel < 0) {
            if(GATE_1_MACH_ID == pMotor->drvIndex){
                osal_dev_io_state_change(BOARD4_OUTPUT_GATE_1_STOP, IO_DISABLE);
            }
            else if(GATE_3_MACH_ID == pMotor->drvIndex){
                osal_dev_io_state_change(BOARD0_OUTPUT_GATE_3_STOP, IO_DISABLE);
            }
            osal_dev_io_state_change(pMotor->ioIndexCW, IO_DISABLE);
            osal_dev_io_state_change(pMotor->ioIndexCCW, IO_ENABLE);
        }
        else {
            if(GATE_1_MACH_ID == pMotor->drvIndex){
                osal_dev_io_state_change(BOARD4_OUTPUT_GATE_1_STOP, IO_ENABLE);
            }
            else if(GATE_3_MACH_ID == pMotor->drvIndex){
                osal_dev_io_state_change(BOARD0_OUTPUT_GATE_3_STOP, IO_ENABLE);
            }
            osal_dev_io_state_change(pMotor->ioIndexCW, IO_DISABLE);
            osal_dev_io_state_change(pMotor->ioIndexCCW, IO_DISABLE);
        }
    }
    else if(DRIVER_TYPE_TRIGGER == pMotor->drvType){        //脉冲触发驱动
        if (pMotor->tarVel > 0) {
            osal_dev_io_state_change(pMotor->ioIndexCW, IO_ENABLE);   //立即使能IO，由线程计时关闭
            osal_dev_io_state_change(pMotor->ioIndexCCW, IO_DISABLE);
            pMotor->moveInfo.isTriggerEnable    = true;
            pMotor->moveInfo.ioIndexTrig        = pMotor->ioIndexCW;
        }
        else if (pMotor->tarVel < 0) {
            osal_dev_io_state_change(pMotor->ioIndexCW, IO_DISABLE);
            osal_dev_io_state_change(pMotor->ioIndexCCW, IO_ENABLE);
            pMotor->moveInfo.isTriggerEnable    = true;
            pMotor->moveInfo.ioIndexTrig        = pMotor->ioIndexCCW;
        }
        else {
            osal_dev_io_state_change(pMotor->ioIndexCW, IO_DISABLE);
            osal_dev_io_state_change(pMotor->ioIndexCCW, IO_DISABLE);
            pMotor->moveInfo.isTriggerEnable    = false;
        }
    }
    else if(DRIVER_TYPE_ADDR == pMotor->drvType){           //驱动器路径触发驱动
        //
    }
    else{
        LOG_UPLOAD("No this driver type, please check");
        return -1;
    }

    return 0;
}

/**
 * @brief       判定移动设备是否触发限位
 * @param[in]	pMotor
 * @return      bool
 */
static bool is_dev_limit_touched(Type_MotorInfo_Def* const pMotor)
{
    if(ACTION_TYPE_ROTATION == pMotor->actionType) return false;        //负责旋转动作的电机无限位

    if (pMotor->tarVel > 0) {           //正向移动时
        //无限位传感器，则按定义的最大位置作为限位
        if(IO_NULL == pMotor->moveInfo.ioIndexLimitCW) {
            if(pMotor->moveInfo.limtMaxPos == MOVE_FOREVER){            //无软硬限位保护
                return false;
            }
            else{
                return (xp_osal_get_dev_pos(pMotor->drvIndex) != NULL_ENCODER && xp_osal_get_dev_pos(pMotor->drvIndex) >= pMotor->moveInfo.limtMaxPos) ? true : false;
            }
        }
        //有限位传感器的机构，如果设定为软限位，则软限位位置和硬限位位置都得停
        else{
            //道闸有多个不允许执行关闭的限位信号，在这里特殊处理
            if(GATE_1_MACH_ID == pMotor->drvIndex){
                if(is_signal_filter_trigger(SIGNAL_ALL_IN)){
                // || is_signal_filter_trigger(SIGNAL_GROUND)){
                    return true;
                }else{}//继续判断限位
            }
            //皮卡检测光电触发不允许顶刷下降，在这里特殊处理
            else if(LIFTER_MATCH_ID == pMotor->drvIndex){
                if(is_signal_filter_trigger(SIGNAL_PICKUP_TRUCK)){
                    return true;
                }else{}//继续判断限位
            }
            if((MODE_SOFT_LIMIT_MAX == pMotor->moveInfo.limitMode || MODE_SOFT_LIMIT_MIN_MAX == pMotor->moveInfo.limitMode)
            && xp_osal_get_dev_pos(pMotor->drvIndex) != NULL_ENCODER && xp_osal_get_dev_pos(pMotor->drvIndex) >= pMotor->moveInfo.limtMaxPos){
                return true;
            }
            return osal_is_io_trigger(pMotor->moveInfo.ioIndexLimitCW) ? true : false;
        }
    }
    else if (pMotor->tarVel < 0) {      //反向移动时
        //无限位传感器，则无保护
        if(IO_NULL == pMotor->moveInfo.ioIndexLimitCCW) {
            return false;
        }
        else{
            if((MODE_SOFT_LIMIT_MIN == pMotor->moveInfo.limitMode || MODE_SOFT_LIMIT_MIN_MAX == pMotor->moveInfo.limitMode)
            && xp_osal_get_dev_pos(pMotor->drvIndex) != NULL_ENCODER && xp_osal_get_dev_pos(pMotor->drvIndex) <= pMotor->moveInfo.limtMinPos){
                return true;
            }
            return osal_is_io_trigger(pMotor->moveInfo.ioIndexLimitCCW) ? true : false;
        }
    }
    return false;
}

/**
 * @brief       判定移动设备是否触发限位（有一定滤波次数）
 * @param[in]	pMotor
 * @return      bool
 */
static bool is_dev_limit_touched_filter(Type_MotorInfo_Def* const pMotor)
{
    if(is_dev_limit_touched(pMotor)){
        if(pMotor->moveInfo.limitTouchedCnt < 3){
            pMotor->moveInfo.limitTouchedCnt++;
            return false;
        }
        else{
            return true;
        }
    }
    else{
        pMotor->moveInfo.limitTouchedCnt = 0;
        return false;
    }
}

/**
 * @brief       注册的电机运动控制线程
 * @param[in]	arg
 */
void xp_osal_dev_run_thread(void* arg)
{
    int32_t  devPos = 0;
    int32_t  lastDevPos[MOTOR_TABLE_NUM] = {0};
    uint64_t pauseTime = 0;
    uint64_t pauseStartStamp = 0;
    Type_MotorInfo_Def *pMotor = NULL;

    /* 为了避免机构在动作过程中限位信号误检测导致停止，需要增加滤波，由 is_dev_limit_touched_filter 函数判定，
    首次上电 limitTouchedCnt 为零时，若机构在限位位置，会导致机构即使在限位处也会动一下，所以这里赋一个较大初始值*/
    /* 对于子板的io信号，可以在子板程序做好滤波，但子板程序无法远程更新，所以在主板程序上想办法 */
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        MotorDev_Table[i].moveInfo.limitTouchedCnt = 0xFF;
    }

    while (1)
    {
        for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
        {
            pMotor = &MotorDev_Table[i];
            if(NULL == pMotor){
                LOG_UPLOAD("pMotor is NULL");
                continue;
            }
            
            //旋转结构不参与状态机，只做速度变更，不做其它判断
            if(ACTION_TYPE_ROTATION == pMotor->actionType){
                if (pMotor->lastVel != pMotor->tarVel){
                    if (0 == xp_dev_motor_run(pMotor)) {
                        pMotor->lastVel = pMotor->tarVel;
                        LOG_UPLOAD("%s ROTATION velocity change to %d", xp_get_device_str(i), pMotor->tarVel);
                    }
                }
                continue;
            }

            bool isMoveOverTime = false;
            int  moveOverTimeVel = 0;

            //状态轮询
            if(MOTOR_STA_IDLE == pMotor->moveInfo.state){
                pauseTime = 0;
            }
            else if(MOTOR_STA_MOVE == pMotor->moveInfo.state){
                if (is_dev_limit_touched_filter(pMotor) || 0 == pMotor->tarVel) {
                    if(0 != pMotor->tarVel){
                        LOG_UPLOAD("%s -MOVE- touch limit, stop move", xp_get_device_str(i));
                    }
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);        //在速度方向上的限位触发后或设置速度为0进入停止模式
                }
                else if (pMotor->moveInfo.actionOverTime < MOVE_FOREVER 
                        && get_diff_ms(pMotor->moveInfo.timeStamp) > (pMotor->moveInfo.actionOverTime + pauseTime)) {
                    LOG_UPLOAD("%s -MOVE- over time, stop move", xp_get_device_str(i));
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                    isMoveOverTime = true;
                    moveOverTimeVel = pMotor->tarVel;
                }
                else if (pMotor->lastVel != pMotor->tarVel) {       //仅当在命令发生变化后才对驱动器发出命令
                    if (0 == xp_dev_motor_run(pMotor)) {
                        pMotor->lastVel = pMotor->tarVel;
                        LOG_UPLOAD("%s -MOVE- velocity change to %d", xp_get_device_str(i), pMotor->tarVel);
                    }
                }
            }
            else if(MOTOR_STA_MOVE_FORE == pMotor->moveInfo.state){
                if (get_diff_ms(pMotor->moveInfo.timeStamp) > pMotor->moveInfo.forceMoveTime || 0 == pMotor->tarVel) {
                    if(0 != pMotor->tarVel){
                        LOG_UPLOAD("%s -FORCE MOVE- time out, stop move", xp_get_device_str(i));
                    }
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                }
                else if (get_diff_ms(pMotor->moveInfo.timeStamp) > (pMotor->moveInfo.actionOverTime + pauseTime)) {
                    LOG_UPLOAD("%s -FORCE MOVE- over time, stop move", xp_get_device_str(i));
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                    isMoveOverTime = true;
                    moveOverTimeVel = pMotor->tarVel;
                }
                else if (pMotor->lastVel != pMotor->tarVel) {       //仅当在命令发生变化后才对驱动器发出命令
                    if (0 == xp_dev_motor_run(pMotor)) {
                        pMotor->lastVel = pMotor->tarVel;
                        LOG_UPLOAD("%s -FORCE MOVE- velocity change to %d", xp_get_device_str(i), pMotor->tarVel);
                    }
                }
            }
            else if(MOTOR_STA_MOVE_POS == pMotor->moveInfo.state){
                devPos = xp_osal_get_dev_pos(pMotor->drvIndex);
                if(0 == pMotor->tarVel){
                    LOG_UPLOAD("%s -MOVE POS- vel stop", xp_get_device_str(i));
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                }
                else if(is_dev_limit_touched_filter(pMotor)){
                    //触碰到限位发现位置没到位，则不停止（因为位置脉冲值计数和清零是由子板判定的，主板检测限位触发较灵敏，可能存在主板检测到限位了（实际是干扰信号），
                    //但子板因为滤波时间较长，未把脉冲值清零，这种情况还能避免电机没到位置误停）
                    if(0 == pMotor->retryCnt
                    || (pMotor->tarVel > 0 && devPos >= pMotor->moveInfo.tarPos - MOVE_POS_ERR)
                    || (pMotor->tarVel < 0 && devPos <= pMotor->moveInfo.tarPos + MOVE_POS_ERR)){
                        xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                        LOG_UPLOAD("%s -MOVE POS- touch limit, retry remain cnt %d", xp_get_device_str(i), pMotor->retryCnt);
                    }
                    if(pMotor->retryCnt)    pMotor->retryCnt--;
                }
                else if (get_diff_ms(pMotor->moveInfo.timeStamp) > (pMotor->moveInfo.actionOverTime + pauseTime)) {
                    LOG_UPLOAD("%s -MOVE POS- over time, stop move", xp_get_device_str(i));
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                    isMoveOverTime = true;
                    moveOverTimeVel = pMotor->tarVel;
                }
                else if (pMotor->lastVel != pMotor->tarVel) {       //仅当在命令发生变化后才对驱动器发出命令
                    if (0 == xp_dev_motor_run(pMotor)) {
                        pMotor->lastVel = pMotor->tarVel;
                        LOG_UPLOAD("%s -MOVE POS- velocity change to %d", xp_get_device_str(i), pMotor->tarVel);
                    }
                }
                else {
                    if(pMotor->tarVel > 0 && devPos >= pMotor->moveInfo.tarPos){
                        LOG_DEBUG("%s -MOVE POS- %d done, now pos %d", xp_get_device_str(i), pMotor->moveInfo.tarPos, devPos);
                        xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                    }
                    else if(pMotor->tarVel < 0 && devPos <= pMotor->moveInfo.tarPos){
                        LOG_DEBUG("%s -MOVE POS- %d done, now pos %d", xp_get_device_str(i), pMotor->moveInfo.tarPos, devPos);
                        xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                    }
                }
            }
            else if(MOTOR_STA_MOVE_TIME == pMotor->moveInfo.state){
                if (is_dev_limit_touched_filter(pMotor) || 0 == pMotor->tarVel) {
                    if(0 != pMotor->tarVel){
                        LOG_UPLOAD("%s -MOVE TIME- touch limit, stop move", xp_get_device_str(i));
                    }
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                }
                else if (get_diff_ms(pMotor->moveInfo.timeStamp) > pMotor->moveInfo.moveTime) {
                    if(0 != pMotor->tarVel){
                        LOG_UPLOAD("%s -MOVE TIME- time out, stop move", xp_get_device_str(i));
                    }
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                }
                else if (get_diff_ms(pMotor->moveInfo.timeStamp) > (pMotor->moveInfo.actionOverTime + pauseTime)) {
                    LOG_UPLOAD("%s -MOVE TIME- over time, stop move", xp_get_device_str(i));
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                    isMoveOverTime = true;
                    moveOverTimeVel = pMotor->tarVel;
                }
                else if (pMotor->lastVel != pMotor->tarVel) {       //仅当在命令发生变化后才对驱动器发出命令
                    if (0 == xp_dev_motor_run(pMotor)) {
                        pMotor->lastVel = pMotor->tarVel;
                        LOG_UPLOAD("%s -MOVE TIME- velocity change to %d", xp_get_device_str(i), pMotor->tarVel);
                    }
                }
            }
            else if(MOTOR_STA_PAUSE == pMotor->moveInfo.state){
                if (pMotor->moveInfo.isFirstSwitch) {
                    pMotor->lastVel = pMotor->tarVel;
                    pMotor->tarVel = 0;
                    if (0 == xp_dev_motor_run(pMotor)) {
                        pMotor->moveInfo.isFirstSwitch = false;
                        pauseStartStamp = aos_now_ms();
                    }
                    else{
                        LOG_UPLOAD("Motor pause failed");
                    }
                }
                else {
                    pauseTime = get_diff_ms(pauseStartStamp);
                }
            }
            else if(MOTOR_STA_RESUME == pMotor->moveInfo.state){
                pMotor->tarVel = pMotor->lastVel;
                xp_osal_motor_state_set(pMotor, pMotor->moveInfo.lastState);
            }
            else if(MOTOR_STA_FAULT == pMotor->moveInfo.state){
                //
            }
            //切换到stop状态立即执行
            if(MOTOR_STA_STOP == pMotor->moveInfo.state){
                pMotor->tarVel = 0;
                if (0 == xp_dev_motor_run(pMotor)) {
                    pMotor->lastVel = 0;
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_IDLE);
                    LOG_DEBUG("%s stop success", xp_get_device_str(i));
                }
                else{
                    LOG_UPLOAD("%s change stop err", xp_get_device_str(i));
                }
            }

            //无脉冲计数的电机跳过后面的判定
            if(ACTION_TYPE_ROTATION == pMotor->actionType || pMotor->moveInfo.limitMode == MODE_SIGNAL_LIMIT){
                continue;
            }
            devPos = xp_osal_get_dev_pos(pMotor->drvIndex);
            /* 码盘计数异常检测（电机在运动过程中多次检测码盘值无变化或非运动状态下多次检测码盘值变化，认为异常） */
            if(MOTOR_STA_MOVE == pMotor->moveInfo.state || MOTOR_STA_MOVE_FORE == pMotor->moveInfo.state 
            || MOTOR_STA_MOVE_POS == pMotor->moveInfo.state || MOTOR_STA_MOVE_TIME == pMotor->moveInfo.state){
                if(!encSta.isMoveSta[i]){          //运动状态改变，清除错误计数，避免切换过程中误计
                    encSta.errCnt[i] = 0;
                    encSta.isMoveSta[i] = true;
                }
            }
            else{
                if(encSta.isMoveSta[i]){
                    encSta.errCnt[i] = 0;
                    encSta.isMoveSta[i] = false;
                }
            }
            //龙门15Hz 4s走24个脉冲，167ms走一个脉冲，判定周期大于这个值，保证移动状态下，每次判定脉冲值都有变化
            if(devPos != NULL_ENCODER && get_diff_ms(encSta.checkTimeStamp[i]) > 400){
                encSta.checkTimeStamp[i] = aos_now_ms();
                if(MOTOR_STA_MOVE == pMotor->moveInfo.state || MOTOR_STA_MOVE_FORE == pMotor->moveInfo.state 
                || MOTOR_STA_MOVE_POS == pMotor->moveInfo.state || MOTOR_STA_MOVE_TIME == pMotor->moveInfo.state){
                    (lastDevPos[i] == devPos) ? encSta.errCnt[i]++ : (encSta.errCnt[i] = 0);
                }
                else{
                    (lastDevPos[i] != devPos) ? encSta.errCnt[i]++ : (encSta.errCnt[i] = 0);
                }
                if(encSta.errCnt[i] > 5){
                    encSta.errCnt[i] = 0;
                    xp_osal_motor_state_set(pMotor, MOTOR_STA_STOP);
                    if(osal_error_upload != NULL){
                        switch (pMotor->drvIndex)
                        {
                        case CONVEYOR_1_MATCH_ID:       osal_error_upload(8112, true);   break;
                        case CONVEYOR_2_MATCH_ID:       osal_error_upload(8113, true);   break;
                        case CONVEYOR_3_MATCH_ID:       osal_error_upload(8114, true);   break;
                        case LIFTER_MATCH_ID:           osal_error_upload(300, true);    break;
                        case FRONT_LEFT_MOVE_MATCH_ID:  osal_error_upload(8225, true);   break;
                        case FRONT_RIGHT_MOVE_MATCH_ID: osal_error_upload(8226, true);   break;
                        case BACK_LEFT_MOVE_MATCH_ID:   osal_error_upload(8227, true);   break;
                        case BACK_RIGHT_MOVE_MATCH_ID:  osal_error_upload(8228, true);   break;
                        default:
                            break;
                        }
                    }
                    LOG_UPLOAD("%s encoder err, dev state %d, dev pos %d", xp_get_device_str(i), pMotor->moveInfo.state, devPos);
                }
                lastDevPos[i] = devPos;
            }
            //动作超时异常判定
            if(osal_error_upload != NULL && isMoveOverTime){
                if(LIFTER_MATCH_ID == pMotor->drvIndex){
                    if(moveOverTimeVel > 0)      osal_error_upload(8031, true);
                    else if(moveOverTimeVel < 0) osal_error_upload(8030, true);
                }
                else if(FRONT_LEFT_MOVE_MATCH_ID == pMotor->drvIndex){
                    if(moveOverTimeVel < 0)      osal_error_upload(8032, true);
                }
                else if(FRONT_RIGHT_MOVE_MATCH_ID == pMotor->drvIndex){
                    if(moveOverTimeVel < 0)      osal_error_upload(8033, true);
                }
                else if(BACK_LEFT_MOVE_MATCH_ID == pMotor->drvIndex){
                    if(moveOverTimeVel < 0)      osal_error_upload(8034, true);
                }
                else if(BACK_RIGHT_MOVE_MATCH_ID == pMotor->drvIndex){
                    if(moveOverTimeVel < 0)      osal_error_upload(8035, true);
                }
                else if(GATE_1_MACH_ID == pMotor->drvIndex){
                    if(moveOverTimeVel > 0)      osal_error_upload(8040, true);
                    else if(moveOverTimeVel < 0) osal_error_upload(8041, true);
                }
            }
        }
        aos_msleep(10);
    }
}

/*                                                         =======================                                                         */
/* ========================================================    电机运动控制接口    ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       毛刷旋转
 * @param[in]	id                  电机驱动索引id
 * @param[in]	vel                 毛刷速度
 * @return      int
 */
int xp_osal_brush_rotation(Type_DriverIndex_Enum id, int vel)
{
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_ROTATION == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            MotorDev_Table[i].tarVel = vel;
            LOG_DEBUG("Rotation %s speed change to %d", xp_get_device_str(i), MotorDev_Table[i].tarVel);
            return xp_osal_motor_state_set(&MotorDev_Table[i], (0 == MotorDev_Table[i].tarVel) ? MOTOR_STA_IDLE : MOTOR_STA_ROTATION);
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/**
 * @brief       电机按指定速度移动（有限位保护）
 * @param[in]	id                  电机驱动索引id
 * @param[in]	vel                 移动速度
 * @return      int                 
 */
int xp_osal_move_run(Type_DriverIndex_Enum id, int vel)
{
    int ret = -1;

    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            ret = xp_osal_motor_state_set(&MotorDev_Table[i], MOTOR_STA_MOVE);
            if(0 == ret){
                MotorDev_Table[i].tarVel = vel;
                MotorDev_Table[i].moveInfo.timeStamp = aos_now_ms();
                LOG_UPLOAD("Move %s speed change to %d", xp_get_device_str(i), MotorDev_Table[i].tarVel);
            }
            return ret;
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/**
 * @brief       强制电机按指定速度移动一段时间（无限位保护）
 * @param[in]	id                  电机驱动索引id
 * @param[in]	vel                 移动速度
 * @param[in]	time                移动时间
 * @return      int
 */
int xp_osal_force_move_run(Type_DriverIndex_Enum id, int vel, uint16_t time)
{
    int ret = -1;
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            ret = xp_osal_motor_state_set(&MotorDev_Table[i], MOTOR_STA_MOVE_FORE);
            if(0 == ret){
                MotorDev_Table[i].tarVel = vel;
                MotorDev_Table[i].moveInfo.forceMoveTime = time;
                MotorDev_Table[i].moveInfo.timeStamp = aos_now_ms();
                LOG_DEBUG("Force move %s speed change to %d, move time %d", xp_get_device_str(i), MotorDev_Table[i].tarVel, MotorDev_Table[i].moveInfo.forceMoveTime);
            }
            return ret;
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}
/**
 * @brief       电机按指定速度移动到指定位置
 * @param[in]	id                  电机驱动索引id
 * @param[in]	vel                 移动速度
 * @param[in]	tarPos              移动目标位置
 * @return      int
 */
int xp_osal_move_pos(Type_DriverIndex_Enum id, int vel, int32_t tarPos)
{
    int32_t nowPos = 0;
    int ret = -1;

    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            ret = xp_osal_motor_state_set(&MotorDev_Table[i], MOTOR_STA_MOVE_POS);
            if(0 == ret){
                nowPos = xp_osal_get_dev_pos(id);
                if(tarPos > nowPos){
                    MotorDev_Table[i].tarVel = abs(vel);
                }
                else if(tarPos < nowPos){
                    MotorDev_Table[i].tarVel = -abs(vel);
                }
                else{
                    MotorDev_Table[i].tarVel = 0;
                    LOG_DEBUG("Pos move %s in right pos already", xp_get_device_str(i));
                }
                MotorDev_Table[i].moveInfo.tarPos = tarPos;
                MotorDev_Table[i].moveInfo.timeStamp = aos_now_ms();
                //若目标距离与当前距离相差不大，就不移动
                if(abs(tarPos - nowPos) <= MOVE_POS_ERR){
                    MotorDev_Table[i].tarVel = 0;
                    MotorDev_Table[i].retryCnt = 0;
                    LOG_INFO("%s now pos close tar pos, cancle move", xp_get_device_str(i));
                }
                else{
                    MotorDev_Table[i].retryCnt = 10;        //移动到目标位置避免中间误触限位，增加重试
                    LOG_UPLOAD("Pos move %s speed change to %d, pos %d to targ pos %d", xp_get_device_str(i), MotorDev_Table[i].tarVel, nowPos, MotorDev_Table[i].moveInfo.tarPos);
                }
            }
            return ret;
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/**
 * @brief       电机按指定速度移动一定时间
 * @param[in]	id                  电机驱动索引id
 * @param[in]	vel                 移动速度
 * @param[in]	time                移动时间
 * @return      int
 */
int xp_osal_move_time(Type_DriverIndex_Enum id, int vel, uint16_t time)
{
    int ret = -1;
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            ret = xp_osal_motor_state_set(&MotorDev_Table[i], MOTOR_STA_MOVE_TIME);
            if(0 == ret){
                MotorDev_Table[i].tarVel = vel;
                MotorDev_Table[i].moveInfo.moveTime = time;
                MotorDev_Table[i].moveInfo.timeStamp = aos_now_ms();
                LOG_UPLOAD("Time move %s speed change to %d, move time %d", xp_get_device_str(i), MotorDev_Table[i].tarVel, MotorDev_Table[i].moveInfo.moveTime);
            }
            return ret;
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/**
 * @brief       电机暂停移动
 * @param[in]	id                  电机驱动索引id
 * @return      int
 */
int xp_osal_move_pause(Type_DriverIndex_Enum id)
{
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            LOG_DEBUG("%s pause", xp_get_device_str(i));
            return xp_osal_motor_state_set(&MotorDev_Table[i], MOTOR_STA_PAUSE);
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/**
 * @brief       电机继续移动
 * @param[in]	id                  电机驱动索引id
 * @return      int
 */
int xp_osal_move_resume(Type_DriverIndex_Enum id)
{
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            LOG_DEBUG("%s resume", xp_get_device_str(i));
            return xp_osal_motor_state_set(&MotorDev_Table[i], MOTOR_STA_RESUME);
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/**
 * @brief       电机停止移动
 * @param[in]	id                  电机驱动索引id
 * @return      int
 */
int xp_osal_move_stop(Type_DriverIndex_Enum id)
{
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            return xp_osal_motor_state_set(&MotorDev_Table[i], MOTOR_STA_STOP);
        }
    }
    LOG_UPLOAD("Driver Index %d, not found, please check...", id);
    return -1;
}

/*                                                         =======================                                                         */
/* ========================================================        其它接口        ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       风机控制
 * @param[in]	type                控制类型（仅左侧/仅右侧/两侧）
 * @param[in]	enable              使能/失能
 * @return      int
 */
void dryer_work(Type_CrlType_Enum type, bool enable)
{
    if(CRL_SECTION_1 == type || CRL_ALL_SECTION == type){
        osal_dev_io_state_change(BOARD0_OUTPUT_DRYER_1_START, enable ? IO_ENABLE : IO_DISABLE);
    }
    if(CRL_SECTION_2 == type || CRL_ALL_SECTION == type){
        osal_dev_io_state_change(BOARD0_OUTPUT_DRYER_1_START, enable ? IO_ENABLE : IO_DISABLE);
    }
    if(CRL_SECTION_3 == type || CRL_ALL_SECTION == type){
        osal_dev_io_state_change(BOARD0_OUTPUT_DRYER_3_START, enable ? IO_ENABLE : IO_DISABLE);
    }
    if(CRL_SECTION_4 == type || CRL_ALL_SECTION == type){
        osal_dev_io_state_change(BOARD0_OUTPUT_DRYER_5_START, enable ? IO_ENABLE : IO_DISABLE);
    }
    if(CRL_SECTION_5 == type || CRL_ALL_SECTION == type){
        osal_dev_io_state_change(BOARD0_OUTPUT_DRYER_5_START, enable ? IO_ENABLE : IO_DISABLE);
    }
}

/**
 * @brief       获取主板输入IO状态
 * @param[in]	pin                 
 * @return      bool                
 */
bool get_board_input_io(uint8_t pin)
{
    return (pin < BOARD_IO_INPUT_PIN_NUM) ? boardIoInSta[pin] : 1;
}

/**
 * @brief       获取主板输出IO状态
 * @param[in]	pin                 
 * @return      bool                
 */
bool get_board_output_io(uint8_t pin)
{
    return ((pin < BOARD_IO_OUTPUT_PIN_NUM)) ? boardIoOutSta[pin] : 1;
}

/**
 * @brief       获取水泵工作状态
 * @return      bool                
 */
bool get_pump_work_status_flag(void)
{
    return isPumpWorking;
}

/**
 * @brief       设置机构的限位模式
 * @param[in]	id                  机构索引
 * @param[in]	mode                限位模式
 * @param[in]	minPos              软限位最小位置
 * @param[in]	maxPos              软限位最大位置
 * @return      int                 
 */
int osal_set_dev_limit_mode(Type_DriverIndex_Enum id, Type_LimitMode_Enum mode, uint16_t minPos, uint16_t maxPos)
{
    for (uint8_t i = 0; i < MOTOR_TABLE_NUM; i++)
    {
        if (ACTION_TYPE_MOVE == MotorDev_Table[i].actionType && id == MotorDev_Table[i].drvIndex) {
            MotorDev_Table[i].moveInfo.limitMode = mode;
            if(MODE_SOFT_LIMIT_MIN == mode){
                MotorDev_Table[i].moveInfo.limtMinPos = minPos;
                LOG_UPLOAD("%s set soft min limit, minPos %d", xp_get_device_str(i), minPos);
            }
            else if(MODE_SOFT_LIMIT_MAX == mode){
                MotorDev_Table[i].moveInfo.limtMaxPos = maxPos;
                LOG_UPLOAD("%s set soft max limit, maxPos %d", xp_get_device_str(i), maxPos);
            }
            else if(MODE_SOFT_LIMIT_MIN_MAX == mode){
                MotorDev_Table[i].moveInfo.limtMinPos = minPos;
                MotorDev_Table[i].moveInfo.limtMaxPos = maxPos;
                LOG_UPLOAD("%s set soft limit, minPos %d, maxPos %d", xp_get_device_str(i), minPos, maxPos);
            }
            else{
                LOG_UPLOAD("%s set signal limit", xp_get_device_str(i));
            }
            return 0;
        }
    }
    LOG_UPLOAD("Set soft limit not found dev");
    return -1;
}

/**
 * @brief       码盘值清零
 * @param[in]	id                  
 * @return      int                 
 */
int clear_dev_encoder(Type_DriverIndex_Enum id)
{
    for (uint8_t i = 0; i < ENCODE_TABLE_NUM; i++)
    {
        if (id == Encoder_Table[i].drvIndex) {
            return xp_io_clear_encoder(BOARD_ID_RESOLUTION(Encoder_Table[i].ioIndexPulse), Encoder_Table[i].ioBoardRegIndex);
        }
    }
    LOG_UPLOAD("Driver Index not found, please check...");
    return -1;
}

/**
 * @brief       错误上报回调
 * @param[in]	callback            
 */
void osal_error_upload_callback_regist(void (*callback)(uint16_t code, bool value))
{
    osal_error_upload = callback;
}

/**
 * @brief       线下收费机启动回调
 * @param[in]	callback            
 */
void offline_payment_callback_regist(void (*callback)(uint8_t washMode))
{
    offline_payment = callback;
}

/*                                                         =======================                                                         */
/* ========================================================       debug接口       ======================================================== */
/*                                                         =======================                                                         */

int osal_debug(char* type, char* fun, char* param)
{
    if (strcmp(type, "osal") != 0) return 0;
    if (strcmp(fun, "voice_play") == 0) {
#if (1 == USE_ONBOARD_VOICE)
        onboard_voice_play((Type_AgVoice_Enum)atoi(param));
#else
        dl485_voice_play(AG_VOICE_POS_EXIT, (Type_AgVoice_Enum)atoi(param));
#endif
    }
    else if (strcmp(fun, "voice_entry") == 0) {
        dl485_voice_play(AG_VOICE_POS_ENTRY, (Type_AgVoice_Enum)atoi(param));
    }
    else if (strcmp(fun, "voice_exit") == 0) {
        dl485_voice_play(AG_VOICE_POS_EXIT, (Type_AgVoice_Enum)atoi(param));
    }
    else if (strcmp(fun, "display") == 0) {
        xp_osal_display_set((Type_AgDisplay_Enum)atoi(param));
    }
    return 1;
}

