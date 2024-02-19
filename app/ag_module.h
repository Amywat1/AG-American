/**
 * @file 	 ag_module.h
 * @brief       
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 * 
 * @copyright Copyright (c) 2022  YGL
 * 
 */

#include <stdlib.h>
#include "stdbool.h"
#include "ag_osal.h"

#ifndef __AG_MODULE_H
#define __AG_MODULE_H

//返回值定义（-1 ~ -90字段预留给非通用异常返回值）
#define NOR_CONTINUE        (100)               //返回值正常，继续运行
#define RET_PAUSE    	    (99)                //传感器异常，返回暂停等待
#define RET_COMPLETE        (0)                 //返回完成标志
#define ERR_ALARM           (-99)               //有告警的错误报警
#define ERR_TIMEOUT         (-100)              //超时报错

//app前端可控制的对象枚举（单个机构单一动作）
typedef enum{
	APP_CRL_GATE_1_OPEN = 0,
	APP_CRL_GATE_1_CLOSE,
	APP_CRL_GATE_2_OPEN,
	APP_CRL_GATE_2_CLOSE,
	APP_CRL_CONVEYOR_1,
	APP_CRL_CONVEYOR_2,
	APP_CRL_CONVEYOR_3,
	APP_CRL_CONVEYOR_ALL,
	APP_CRL_HIGH_PRESS_WATER,
	APP_CRL_WATER_SHAMPOO,
	APP_CRL_WATER_WAXWATER,
	APP_CRL_WATER_TOP,
	APP_CRL_WATER_FRONT_SIDE,
	APP_CRL_WATER_BACK_SIDE,
	APP_CRL_SKIRT_BRUSH_MOVE,
	APP_CRL_SKIRT_BRUSH_ROTATIN,
	APP_CRL_TOP_BRUSH,
	APP_CRL_LIFTER,
	APP_CRL_FRONT_LEFT_BRUSH,
	APP_CRL_FRONT_LEFT_MOVE,
	APP_CRL_FRONT_RIGHT_BRUSH,
	APP_CRL_FRONT_RIGHT_MOVE,
	APP_CRL_BACK_LEFT_BRUSH,
	APP_CRL_BACK_LEFT_MOVE,
	APP_CRL_BACK_RIGHT_BRUSH,
	APP_CRL_BACK_RIGHT_MOVE,
	APP_CRL_DRYER_1,
	APP_CRL_DRYER_25,
	APP_CRL_DRYER_34,
	APP_CRL_DRYER_6,
	APP_CRL_FLOODLIGHT,
	APP_CRL_AMBIENT_LIGHT,
	APP_CRL_SEWAGE_PUMP,
	APP_CRL_HIGH_PUMP,
	APP_CRL_WINTER_DRAINAGE,
	APP_CRL_CONVEYOR_1_VALVE,
	APP_CRL_CONVEYOR_2_VALVE,
	APP_CRL_CONVEYOR_3_VALVE,
	APP_CRL_NUM,
} Type_AppCrlObject_Enum;

//停车状态
typedef enum{
	PARK_EMPTY = 0,
	PARK_TOO_BACK,				//停车太后
	PARK_TOO_FRONT,
	PARK_TOO_LEFT,
	PARK_TOO_RIGHT,
	PARK_TOO_LONG,
	PARK_OK,
} Type_ParkState_Enum;

//毛刷枚举
typedef enum{
	BRUSH_TOP = 0,
	BRUSH_FORNT_LEFT,
	BRUSH_FORNT_RIGHT,
	BRUSH_BACK_LEFT,
	BRUSH_BACK_RIGHT,
	BRUSH_NUM,
	BRUSH_SIDE_FRONT,
	BRUSH_SIDE_BACK,
} Type_BrushType_Enum;

//毛刷刷洗跟随状态
typedef enum {
	BRUSH_MANUAL = 0,							//手动控制，不跟随电流
    BRUSH_FREE_FOLLOW,							//根据电流自由跟随
    BRUSH_FOLLOW_NO_FORWARD,
    BRUSH_FOLLOW_NO_BACKWARD,
} Type_BrushRunMode_Enum;

//毛刷对象结构体定义
typedef struct{
	bool 					init;               //init state
	bool					isPressProtectMove;	//是否达到触压保护值后退

	int16_t					current;			//检测电流值
	int16_t 				baseCurrent;        //空转基准值
	int16_t 				pressL;             //下限值，<该值需进给
	int16_t 				pressH;             //上限值，>该值需退回
	int16_t 				pressL_NoBW;        //毛刷不允许后退时的下限值，<该值前进
	int16_t 				pressH_NoFW;        //毛刷不允许前进时的上限值，>该值后退
    int16_t 				pressTouchcar;      //触碰到车的压力值
	int16_t 				pressProtect;       //保护值，>该值需有保护性动作（如停止龙门移动）
	int16_t 				pressWarning;       //告警电流，>该值退出洗车流程并告警

    uint8_t 				pressTooHighCnt;    //触压异常计数值

	Type_BrushRunMode_Enum	runMode;			//跟随状态

    bool 					isJogMove;          //毛刷是否点动移动
	bool					isReadyCalibrate;	//是否准备进行电流校准
	bool					isCalibrated;		//是否已进行电流校准
} Type_BrushInfo_Def;

extern uint64_t get_voice_start_time_stamp(void);
extern void module_lock_voice_mutex(uint16_t overTime);
extern void module_unlock_voice_mutex(void);
extern int xp_service_module_init(void);
extern void voice_play_set(Type_AgVoicePos_Enum pos, Type_AgVoice_Enum mode);
extern bool is_dev_move_sta_idle(Type_DriverIndex_Enum devIndex);
extern void set_driver_executed_flag(bool value);
extern void set_step_pause_time(uint64_t value);
extern int get_brush_current(Type_BrushType_Enum type);
extern void stop_all_dev_run(void);
extern void app_crl_dev_set(Type_AppCrlObject_Enum obj, int cmd);
extern int gate_change_state(Type_CrlType_Enum type, int cmd);

extern int step_dev_back_home(void);
extern int step_dev_warning_home(void);
extern int step_dev_wash(uint8_t *completeId);

extern void set_new_car_ready_wash_falg(bool value);
extern bool get_new_car_ready_wash_falg(void);
extern void set_is_allow_next_car_wash_flag(bool value);
extern bool get_is_allow_next_car_wash_flag(void);
extern void set_new_order_car_id(uint8_t newOrderId);
extern int  get_work_state(uint8_t washId);
extern void wash_crl_variable_init(void);
#endif


