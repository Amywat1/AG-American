/**
 * @file 	 ag_osal.h
 * @brief
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 *
 * @copyright Copyright (c) 2022  YGL
 *
 */

#ifndef __AG_OSAL_H
#define __AG_OSAL_H

#include "log_level.h"
#include "../../../1km/bsp/tool.h"
#include "io_config.h"

#define	IO_ENABLE						(0)								//使能电平
#define IO_DISABLE						(!IO_ENABLE)					//失能电平
#define	IO_NULL							(31U)							//未配置的IO（引脚状态由u32类型定义，实际引脚号为0~23，24~31做虚拟IO引脚表示无该引脚定义）
#define MOVE_POS_ERR					(3)								//移动到固定位置时的允许误差
#define MOVE_FOREVER					(0xFFFF)						//无目标位置限制
#define DIFF_BOARD_IO_VALUE				(100)							//不同线路板IO的区分差值
#define BOARD_ID_RESOLUTION(id)     	(id / DIFF_BOARD_IO_VALUE)      //解析线路板id
#define PIN_ID_RESOLUTION(id)       	(id % DIFF_BOARD_IO_VALUE)      //解析线路板引脚id

//动作机构对应osal中的驱动Index
#define FRONT_LEFT_BRUSH_MATCH_ID       (DRIVER_VFD_SHIHLIN_ID_0)
#define FRONT_RIGHT_BRUSH_MATCH_ID      (DRIVER_VFD_SHIHLIN_ID_1)
#define BACK_LEFT_BRUSH_MATCH_ID        (DRIVER_VFD_SHIHLIN_ID_2)
#define BACK_RIGHT_BRUSH_MATCH_ID       (DRIVER_VFD_SHIHLIN_ID_3)
#define TOP_BRUSH_MATCH_ID              (DRIVER_VFD_SHIHLIN_ID_4)
#define CONVEYOR_1_MATCH_ID             (DRIVER_VFD_SHIHLIN_ID_5)
#define CONVEYOR_2_MATCH_ID             (DRIVER_VFD_SHIHLIN_ID_6)
#define CONVEYOR_3_MATCH_ID             (DRIVER_VFD_SHIHLIN_ID_7)
#define LIFTER_MATCH_ID                 (DRIVER_VFD_SHIHLIN_ID_8)
#define FRONT_LEFT_MOVE_MATCH_ID      	(DRIVER_KM_ID_0)
#define FRONT_RIGHT_MOVE_MATCH_ID     	(DRIVER_KM_ID_1)
#define BACK_LEFT_MOVE_MATCH_ID       	(DRIVER_KM_ID_2)
#define BACK_RIGHT_MOVE_MATCH_ID      	(DRIVER_KM_ID_3)
#define GATE_1_MACH_ID       			(DRIVER_KM_ID_4)
#define GATE_3_MACH_ID       			(DRIVER_KM_ID_5)

 // 输入口枚举
 /***********************************************************************************/
typedef enum {
	// 业务主板
	BOARD_IO_INPUT_PIN_NUM = 0,					//输入IO使用个数

	// 0号IO扩展板
	BOARD0_INPUT_BUTTON_START = 1*DIFF_BOARD_IO_VALUE,	//手动开始按钮
	BOARD0_INPUT_BUTTON_STOP,				//手动停止按钮
	BOARD0_INPUT_BUTTON_RESET,				//手动复位按钮
	BOARD0_INPUT_POWER_ON,					//主电路有电（急停回路）
	BOARD0_INPUT_BUTTON_PAUSE_RESUME,		//暂停/继续====
	BOARD0_INPUT_RESERVE_1,					//
	BOARD0_INPUT_HIGH_PRESS_PUMP_WORK,		//高压泵工作====
	BOARD0_INPUT_LOW_PRESS_PUMP_WORK,		//低压泵工作====
	BOARD0_INPUT_BACK_LEFT_COLLISION,		//左后防撞
	BOARD0_INPUT_BACK_RIGHT_COLLISION,		//右后防撞
	BOARD0_INPUT_SIGNAL_REAR_END_PROTECT,	//防追尾光电
	BOARD0_INPUT_SIGNAL_EXIT,				//出口光电
	BOARD0_INPUT_SIGNAL_FINISH,				//完成光电
	BOARD0_INPUT_CONVEYOR_1_PULSE,			//输送带1#脉冲（预备区）====
	BOARD0_INPUT_CONVEYOR_2_PULSE,			//输送带2#脉冲（工作区）====
	BOARD0_INPUT_CONVEYOR_3_PULSE,			//输送带3#脉冲（完成区）====
	BOARD0_INPUT_CONVEYOR_1_ERR,			//输送带1#异常
	BOARD0_INPUT_CONVEYOR_2_ERR,			//输送带2#异常
	BOARD0_INPUT_CONVEYOR_3_ERR,			//输送带3#异常
	BOARD0_INPUT_DRYER_1_ERR,				//风机1异常
	BOARD0_INPUT_DRYER_2_ERR,				//风机2异常
	BOARD0_INPUT_DRYER_3_ERR,				//风机3异常
	BOARD0_INPUT_DRYER_4_ERR,				//风机4异常
	BOARD0_INPUT_DRYER_5_ERR,				//风机5异常
	BOARD0_INPUT_PIN_NUM,
	// 1号IO扩展板
	BOARD1_INPUT_RESERVE_1 = 2*DIFF_BOARD_IO_VALUE,	//6#风机异常（预留）====
	BOARD1_INPUT_RESERVE_2,
	BOARD1_INPUT_SHAMPOO_LESS_BULE,				//缺香波（蓝色）====
	BOARD1_INPUT_SHAMPOO_LESS_GREEN,			//缺香波（绿色）====
	BOARD1_INPUT_SHAMPOO_LIQUID_PINK,			//缺香波（粉色）====
	BOARD1_INPUT_WAXWATER_LESS,					//缺蜡水====
	BOARD1_INPUT_CLEANER_LESS,					//缺泥沙松动剂====
	BOARD1_INPUT_DRIER_LESS,					//缺助干剂====
	BOARD1_INPUT_RESERVE_3,
	BOARD1_INPUT_RESERVE_4,
	BOARD1_INPUT_RESERVE_5,
	BOARD1_INPUT_RESERVE_6,
	BOARD1_INPUT_RESERVE_7,
	BOARD1_INPUT_RESERVE_8,
	BOARD1_INPUT_RESERVE_9,
	BOARD1_INPUT_RESERVE_10,
	BOARD1_INPUT_RESERVE_11,
	BOARD1_INPUT_RESERVE_12,
	BOARD1_INPUT_RESERVE_13,
	BOARD1_INPUT_RESERVE_14,
	BOARD1_INPUT_RESERVE_15,
	BOARD1_FINISH_AREA_WHEEL_EXIT,				//出口处轮子到达输送带边缘====
	BOARD1_INPUT_WATER_PRESS_LOW_1,				//水压不足1====
	BOARD1_INPUT_WATER_PRESS_LOW_2,				//水压不足2====
	BOARD1_INPUT_PIN_NUM,
	// 2号IO扩展板
	BOARD2_INPUT_RESERVE_1 = 3*DIFF_BOARD_IO_VALUE,
	BOARD2_INPUT_RESERVE_2,
	BOARD2_INPUT_RESERVE_3,
	BOARD2_INPUT_RESERVE_4,
	BOARD2_INPUT_FRONT_LEFT_MOVE_ERR,		//左前刷横移异常
	BOARD2_INPUT_FRONT_RIGHT_MOVE_ERR,		//右前刷横移异常
	BOARD2_INPUT_BACK_LEFT_MOVE_ERR,		//左后刷横移异常
	BOARD2_INPUT_BACK_RIGHT_MOVE_ERR,		//右后刷横移异常
	BOARD2_INPUT_RESERVE_5,
	BOARD2_INPUT_FRONT_LEFT_MOVE_ZERO,		//左前刷开位
	BOARD2_INPUT_FRONT_LEFT_BRUSH_DOWN,		//左前刷低位
	BOARD2_INPUT_FRONT_LEFT_MOVE_PULSE,		//左前刷脉冲
	BOARD2_INPUT_FRONT_RIGHT_BRUSH_CROOKED,	//右前刷前后歪====
	BOARD2_INPUT_FRONT_RIGHT_MOVE_ZERO,		//右前刷开位
	BOARD2_INPUT_FRONT_RIGHT_BRUSH_DOWN,	//右前刷低位
	BOARD2_INPUT_FRONT_RIGHT_MOVE_PULSE,	//右前刷脉冲
	BOARD2_INPUT_RESERVE_6,
	BOARD2_INPUT_BACK_LEFT_MOVE_ZERO,		//左后刷开位
	BOARD2_INPUT_BACK_LEFT_BRUSH_DOWN,		//左后刷低位
	BOARD2_INPUT_BACK_LEFT_MOVE_PULSE,		//左后刷脉冲
	BOARD2_INPUT_BACK_RIGHT_BRUSH_CROOKED,	//右后刷前后歪====
	BOARD2_INPUT_BACK_RIGHT_MOVE_ZERO,		//右后刷开位
	BOARD2_INPUT_BACK_RIGHT_BRUSH_DOWN,		//右后刷低位
	BOARD2_INPUT_BACK_RIGHT_MOVE_PULSE,		//右后刷脉冲
	BOARD2_INPUT_PIN_NUM,
	// 3号IO扩展板
	BOARD3_INPUT_FRONT_LEFT_ROTATION_ERR = 4*DIFF_BOARD_IO_VALUE,		//左前刷旋转异常
	BOARD3_INPUT_FRONT_RIGHT_ROTATION_ERR,		//右前刷旋转异常
	BOARD3_INPUT_BACK_LEFT_ROTATION_ERR,		//左后刷旋转异常
	BOARD3_INPUT_BACK_RIGHT_ROTATION_ERR,		//右后刷旋转异常
	BOARD3_INPUT_PIN_NUM,
	// 4号IO扩展板
	BOARD4_INPUT_RESERVE_1 = 5*DIFF_BOARD_IO_VALUE,
	BOARD4_INPUT_RESERVE_2,
	BOARD4_INPUT_RESERVE_3,
	BOARD4_INPUT_RESERVE_4,
	BOARD4_INPUT_BUTTON_START_ENTRY,		//入口处启动按钮====
	BOARD4_INPUT_RESERVE_6,
	BOARD4_INPUT_ALL_IN_SIGNAL,				//全进光电
	BOARD4_INPUT_PICKUP_TRUCK_SIGNAL,		//皮卡检测光电
	BOARD4_INPUT_FRONT_LEFT_COLLISION,		//左前防撞
	BOARD4_INPUT_FRONT_RIGHT_COLLISION,		//右前防撞
	BOARD4_INPUT_AVOID_INTRUDE_SIGNAL,		//防闯光电
	BOARD4_INPUT_ENTRANCE_SIGNAL,			//入口光电
	BOARD4_INPUT_CAR_STOP_SIGNAL,			//停车光电
	BOARD4_INPUT_SUPER_HIGH,				//超高
	BOARD4_INPUT_STOP_LEFT_SKEW,			//左停偏光电
	BOARD4_INPUT_STOP_RIGHT_SKEW,			//右停偏光电
	BOARD4_INPUT_LIFTER_UP,					//顶刷上限位
	BOARD4_INPUT_LIFTER_DOWN,				//顶刷下限位
	BOARD4_INPUT_LIFTER_PULSE,				//顶刷脉冲
	BOARD4_INPUT_LIFTER_LEFT_DETECH,		//左织带松
	BOARD4_INPUT_LIFTER_RIGHT_DETECH,		//右织带松
	BOARD4_INPUT_FRONT_LEFT_SKIRT_ZERO,		//左前群刷开位
	BOARD4_INPUT_FRONT_RIGHT_SKIRT_ZERO,	//右前群刷开位
	BOARD4_INPUT_RESERVE_7,
	BOARD4_INPUT_PIN_NUM,
	// 5号IO扩展板
	BOARD5_INPUT_GATE_2_LEFT_OPEN_DONE = 6*DIFF_BOARD_IO_VALUE,	//2#号左道闸开到位
	BOARD5_INPUT_GATE_2_RIGHT_OPEN_DONE,	//2#号右道闸开到位
	BOARD5_INPUT_RESERVE_1,
	BOARD5_INPUT_RESERVE_2,
	BOARD5_INPUT_RESERVE_3,
	BOARD5_INPUT_RESERVE_4,
	BOARD5_INPUT_GATE_1_CLOSE_DONE,			//1#号道闸关到位
	BOARD5_INPUT_GATE_1_OPEN_DONE,			//1#号道闸开到位
	BOARD5_INPUT_TOP_BRUSH_ERR,				//顶刷旋转故障
	BOARD5_INPUT_LIFTER_ERR,				//顶刷上下移动故障
	BOARD5_INPUT_TOP_SWING_ERR,				//顶部摆淋故障
	BOARD5_INPUT_FRONT_LEFT_SKIRT_ERR,		//左前裙边刷故障
	BOARD5_INPUT_FRONT_RIGHT_SKIRT_ERR,		//右前裙边刷故障
	BOARD5_INPUT_WATER_PRESS_LOW_3,			//水压不足====
	BOARD5_INPUT_SIGNAL_GATE_1_PROTECT,		//1号闸处光电（无地感时做道闸关保护）
	BOARD5_INPUT_GIVE_UP_WASH,				//放弃洗车，允许客户从机器通道内离开
	BOARD5_INPUT_WASH_MODE_1,				//高档
	BOARD5_INPUT_WASH_MODE_2,
	BOARD5_INPUT_WASH_MODE_3,				//普通
	BOARD5_INPUT_WASH_MODE_4,
	BOARD5_INPUT_PIN_NUM,
} Type_InputIo_Enum;

// 板输出口枚举
/***********************************************************************************/

typedef enum {
	// 业务主板
	// BOARD_IO_OUTPUT_DEV_POWER_ON = 0,		//主电路上电
	BOARD_IO_OUTPUT_PIN_NUM,

	// 0号IO扩展板
	BOARD0_OUTPUT_BUTTON_LIGHT_START = 1*DIFF_BOARD_IO_VALUE,		//手动开始灯====
	BOARD0_OUTPUT_BUTTON_LIGHT_STOP,		//手动停止灯====
	BOARD0_OUTPUT_BUTTON_LIGHT_RESET,		//复位按钮灯====
	BOARD0_OUTPUT_AMBIENT_LIGHT,			//照明灯（射灯）====
	BOARD0_OUTPUT_RESERVE_1,
	BOARD0_OUTPUT_RESERVE_2,
	BOARD0_OUTPUT_RESERVE_3,
	BOARD0_OUTPUT_GATE_3_STOP,				//道闸3#停====
	BOARD0_OUTPUT_CONVEYOR_1_CW,			//待洗电机正转
	BOARD0_OUTPUT_CONVEYOR_1_RESET,			//待洗电机复位====
	BOARD0_OUTPUT_CONVEYOR_2_CW,			//工作电机正转
	BOARD0_OUTPUT_CONVEYOR_2_RESET,			//工作电机复位
	BOARD0_OUTPUT_CONVEYOR_3_CW,			//完成电机正转
	BOARD0_OUTPUT_CONVEYOR_3_RESET,			//完成电机复位====
	BOARD0_OUTPUT_HIGH_PUMP,				//高压泵
	BOARD0_OUTPUT_LOW_PUMP,					//低压泵
	BOARD0_OUTPUT_DRYER_3_START,			//风机3启动
	BOARD0_OUTPUT_DRYER_3_RESET,			//风机3复位====
	BOARD0_OUTPUT_DRYER_1_START,			//风机1启动
	BOARD0_OUTPUT_DRYER_1_RESET,			//风机1复位====
	BOARD0_OUTPUT_DRYER_5_START,			//风机5启动
	BOARD0_OUTPUT_DRYER_5_RESET,			//风机5复位====
	BOARD0_OUTPUT_SHAMPOO_AIR_VALVE,		//香波气阀====
	BOARD0_OUTPUT_SKIRT_BRUSH_VALVE,		//群刷气阀
	BOARD0_OUTPUT_PIN_NUM,
	// 1号IO扩展板
	BOARD1_OUTPUT_FLOODLIGHT = 2*DIFF_BOARD_IO_VALUE,	//照明灯====
	BOARD1_OUTPUT_CLEANER_PUMP,				//泥沙松动剂计量泵====
	BOARD1_OUTPUT_DRIER_PUMP,				//助干剂计量泵====
	BOARD1_OUTPUT_GREEN_LIGHT_EXIT,			//出口绿灯
	BOARD1_OUTPUT_RED_LIGHT_EXIT,			//出口红灯
	BOARD1_OUTPUT_GATE_3_OPEN,				//道闸3#起
	BOARD1_OUTPUT_GATE_3_CLOSE,				//道闸3#落
	BOARD1_OUTPUT_SAFE_RELAY_RESET,			//安全继电器复位
	BOARD1_OUTPUT_SHAMPOO_PUMP,				//香波泡沫计量泵（蓝）
	BOARD1_OUTPUT_SHAMPOO_GREEN_PUMP,		//香波泡沫计量泵（绿）普通的
	BOARD1_OUTPUT_SHAMPOO_PINK_PUMP,		//香波泡沫计量泵（粉）镀膜
	BOARD1_OUTPUT_WAXWATER_PUMP,			//蜡水计量泵
	BOARD1_OUTPUT_CLEANER_VALVE,			//泥沙松动剂水阀
	BOARD1_OUTPUT_SHAMPOO_PINK_VALVE,		//香波水阀（粉）
	BOARD1_OUTPUT_TOP_WATER_VALVE,			//顶刷水
	BOARD1_OUTPUT_BACK_BRUSH_WATER_VALVE,	//后侧刷水
	BOARD1_OUTPUT_FRONT_BRUSH_WATER_VALVE,	//前侧刷水
	BOARD1_OUTPUT_CLEAR_WATER_VALVE,		//清水
	BOARD1_OUTPUT_DRIER_VALVE,				//助干剂
	BOARD1_OUTPUT_BASE_PLATE_WASH_VALVE,	//底盘水
	BOARD1_OUTPUT_WAXWATER_VALVE,			//蜡水
	BOARD1_OUTPUT_SHAMPOO_WATER_VALVE,		//香波水阀（蓝）
	BOARD1_OUTPUT_SHAMPOO_GREEN_VALVE,		//香波水阀（绿）
	BOARD1_OUTPUT_DRAIN_WATER_VALVE,		//排水气阀====			第二路香波气阀
	BOARD1_OUTPUT_PIN_NUM,
	// 2号IO扩展板
	BOARD2_OUTPUT_BACK_LEFT_CLOSE = 3*DIFF_BOARD_IO_VALUE,	//左后刷合
	BOARD2_OUTPUT_BACK_LEFT_OPEN,			//左后刷开
	BOARD2_OUTPUT_BACK_RIGHT_CLOSE,			//右后刷合
	BOARD2_OUTPUT_BACK_RIGHT_OPEN,			//右后刷开
	BOARD2_OUTPUT_FRONT_LEFT_MOVE_RESET,	//左前刷横移复位====
	BOARD2_OUTPUT_FRONT_RIGHT_MOVE_RESET,	//右前刷横移复位====
	BOARD2_OUTPUT_BACK_LEFT_MOVE_RESET,		//左后刷横移复位====
	BOARD2_OUTPUT_BACK_RIGHT_MOVE_RESET,	//右后刷横移复位====
	BOARD2_OUTPUT_FRONT_LEFT_BRUSH_CW,		//左前刷旋转（正转）
	BOARD2_OUTPUT_FRONT_LEFT_BRUSH_CCW,		//左前刷旋转（反转）
	BOARD2_OUTPUT_FRONT_LEFT_BRUSH_RESET,	//左前刷旋转（复位）====
	BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_CW,		//右前刷旋转（正转）
	BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_CCW,	//右前刷旋转（反转）
	BOARD2_OUTPUT_FRONT_RIGHT_BRUSH_RESET,	//右前刷旋转（复位）
	BOARD2_OUTPUT_BACK_LEFT_BRUSH_CW,		//左后刷旋转（正转）
	BOARD2_OUTPUT_BACK_LEFT_BRUSH_CCW,		//左后刷旋转（反转）
	BOARD2_OUTPUT_BACK_LEFT_BRUSH_RESET,	//左后刷旋转（复位）====
	BOARD2_OUTPUT_BACK_RIGHT_BRUSH_CW,		//右后刷旋转（正转）
	BOARD2_OUTPUT_BACK_RIGHT_BRUSH_CCW,		//右后刷旋转（反转）
	BOARD2_OUTPUT_BACK_RIGHT_BRUSH_RESET,	//右后刷旋转（复位）====
	BOARD2_OUTPUT_FRONT_LEFT_CLOSE,			//左前刷合
	BOARD2_OUTPUT_FRONT_LEFT_OPEN,			//左前刷开
	BOARD2_OUTPUT_FRONT_RIGHT_CLOSE,		//右前刷合
	BOARD2_OUTPUT_FRONT_RIGHT_OPEN,			//右前刷开
	BOARD2_OUTPUT_PIN_NUM,
	// 3号IO扩展板
	BOARD3_OUTPUT_RESERVE_1 = 4*DIFF_BOARD_IO_VALUE,
	BOARD3_OUTPUT_RESERVE_2,
	BOARD3_OUTPUT_RESERVE_3,
	BOARD3_OUTPUT_RESERVE_4,
	BOARD3_OUTPUT_FLOODLIGHT,				//照明灯====
	BOARD3_OUTPUT_PIN_NUM,

	// 4号IO扩展板
	BOARD4_OUTPUT_LED_IN_01 = 5*DIFF_BOARD_IO_VALUE,	//LED_IN_01===
	BOARD4_OUTPUT_LED_IN_02,				//LED_IN_02===
	BOARD4_OUTPUT_LED_IN_03,				//LED_IN_03====
	BOARD4_OUTPUT_RESERVE_1,
	BOARD4_OUTPUT_RESERVE_2,
	BOARD4_OUTPUT_RESERVE_3,
	BOARD4_OUTPUT_RESERVE_4,
	BOARD4_OUTPUT_LED_IN_04,				//LED_IN_04====
	BOARD4_OUTPUT_LIFTER_CW,				//顶刷升降（正转）
	BOARD4_OUTPUT_LIFTER_CCW,				//顶刷升降（反转）
	BOARD4_OUTPUT_LIFTER_RESET,				//顶刷升降（复位）====
	BOARD4_OUTPUT_LEFT_SKIRT_ROTATION,		//左前群刷正转
	BOARD4_OUTPUT_RIGHT_SKIRT_ROTATION,		//右前群刷正转
	BOARD4_OUTPUT_TOP_SWING,				//顶部喷淋摆动
	BOARD4_OUTPUT_TOP_ROTATION,				//顶刷旋转
	BOARD4_OUTPUT_FLOODLIGHT,				//照明灯====
	BOARD4_OUTPUT_GATE_1_OPEN,				//道闸1#开
	BOARD4_OUTPUT_GATE_1_CLOSE,				//道闸1#关
	BOARD4_OUTPUT_GATE_1_STOP,				//道闸1#停
	BOARD4_OUTPUT_WARNING_BOARD,			//Stop警示牌
	BOARD4_OUTPUT_AMBIENT_LIGHT,			//照明灯（射灯）====
	BOARD4_OUTPUT_LEFT_CONVEYOR_VALVE,		//左链板冲洗阀====
	BOARD4_OUTPUT_RIGHT_CONVEYOR_VALVE,		//右链板冲洗阀====
	BOARD4_OUTPUT_SWING_VALVE,				//摆动喷淋阀
	BOARD4_OUTPUT_PIN_NUM,
	//5号IO扩展板
	BOARD5_OUTPUT_TOP_ROTATION_CCW = 6*DIFF_BOARD_IO_VALUE,	//顶刷旋转（反转预留）
	BOARD5_OUTPUT_TOP_ROTATION_RESET,		//顶刷旋转变频器复位====
	BOARD5_OUTPUT_TOP_SWING_CCW,			//顶部喷淋摆动（反转预留）
	BOARD5_OUTPUT_TOP_SWING_RESET,			//顶部喷淋摆动变频器复位====
	BOARD5_OUTPUT_RESERVE_1,
	BOARD5_OUTPUT_LEFT_SKIRT_RESET,			//左侧裙边刷变频器复位
	BOARD5_OUTPUT_RESERVE_3,
	BOARD5_OUTPUT_RIGHT_SKIRT_RESET,		//右侧裙边刷变频器复位
	BOARD5_OUTPUT_SIGNAL_LAMP_GREEN,		//信号灯柱——绿色====
	BOARD5_OUTPUT_SIGNAL_LAMP_YELLOW,		//信号灯柱——黄色====
	BOARD5_OUTPUT_SIGNAL_LAMP_RED,			//信号灯柱——红色====
	BOARD5_OUTPUT_MACHINE_IDEL,				//机器空闲
	BOARD5_OUTPUT_PIN_NUM,
} Type_OutputIo_Enum;

/***********************************************************************************/

typedef enum {
	BOARD_NONE = 0,
	IO_BOARD0,							//子板调用的接口id号从1开始
	IO_BOARD1,
	IO_BOARD2,
	IO_BOARD3,
	IO_BOARD4,
	IO_BOARD5,
	BOARD_NUMS,
	BOARD_NULL = 255,					//空主板定义
} Type_Ioboards_Enum;

//voice module
/***********************************************************************************/
typedef enum {
	AG_VOICE_SILENCE = 0,               // 静音
	AG_VOICE_CAR_FORWARD,               // 继续向前				（预备区）
	AG_VOICE_SLOW_BACKOFF,              // 缓慢后退				（预备区）
	AG_VOICE_CAR_LEFT,                  // 停车偏左				（预备区）
	AG_VOICE_CAR_RIGHT,                 // 停车偏右				（预备区）
	AG_VOICE_CAR_STOP,                  // 请停车				（预备区）
	AG_VOICE_WASH_START,            	// 开始洗车				（预备区）
	AG_VOICE_CAR_TOO_LONG,            	// 车辆超长				（预备区）
	// AG_VOICE_LEAVE_AT_ONCE,            	// 后方来车，请立即驶离	 （完成区）
	AG_VOICE_RESERVE,
	AG_VOICE_OVER_HEIGHT,               // 车辆超高				（预备区）
	AG_VOICE_COMPLETED,                 // 洗车已完成			（完成区）
	AG_VOICE_EXIT_CONGESTION,			// 出口拥堵				（完成区）
	AG_VOICE_WASH_EXCEPTION,            // 洗车异常				（预备区）
	AG_VOICE_HOMING,                    // 归位中				（预备区）
	AG_VOICE_PLEASE_PAY,				// 请扫码付费			（预备区）
	AG_VOICE_GO_READY_AREA,				// 请驶入洗车预备区	 	 （预备区）
} Type_AgVoice_Enum;

typedef enum {
	AG_VOICE_POS_ENTRY = 0,				//入口处语音
	AG_VOICE_POS_EXIT,					//出口处语音
} Type_AgVoicePos_Enum;

typedef struct {
	int (*init)();                      //voice module init
	int (*play)(Type_AgVoicePos_Enum pos, Type_AgVoice_Enum item);     //voice module play set code number
} Type_Voice_Def;
/***********************************************************************************/

//dispaly module
/***********************************************************************************/
typedef enum {
	// AG_PAUSE_SERVICE = 0,               // 暂停服务
	// AG_CAR_FORWARD,                     // 请前进
	// AG_CAR_BACKOFF,                     // 缓慢后退
	// AG_CAR_STOP,                        // 请停车
	// AG_CAR_TRANSFER,                    // 车辆传送
	// AG_CAR_READY_TRANSFER,              // 即将传送
	// AG_CAR_WAIT,                    	// 请稍等
	// AG_CAR_TOO_LONG,                    // 车辆超长
	// AG_SERVICE_EXCEPTION,               // 服务异常
	// AG_CAR_LEFT_SKEW,                   // 左侧停偏
	// AG_CAR_RIGHT_SKEW,                  // 右侧停偏
	// AG_DEV_RESET,                  		// 复位中
	// AG_CAR_MOVEE_ON,                  	// 继续前进
	// AG_SCAN_CODE,                     	// 请扫码
	// AG_OVER_HEIGHT,                     // 车辆超高
	// AG_SOON_TO_OPEN,					// 即将开业
	AG_PAUSE_SERVICE = 0,               // Closed
	AG_CAR_FORWARD,                     // Forward
	AG_CAR_MOVEE_ON,                  	// Continue Forwar
	AG_CAR_BACKOFF,                     // Back Up
	AG_CAR_STOP,                        // Stop
	AG_CAR_PARK_BRAKE,                  // Parking Brake
	AG_SCAN_CODE,                     	// Scan Code
	AG_CAR_READY_TRANSFER,              // Conveyor Starting
	AG_CAR_WASH_STARTING,               // Wash Starting
	AG_DEV_RESET,                  		// Resetting
	AG_CAR_LEFT_SKEW,                   // Move Right
	AG_CAR_RIGHT_SKEW,                  // Move Left
	AG_OVER_HEIGHT,                     // Overheight
	AG_CAR_TOO_LONG,                    // Overlength
	AG_CAR_WAIT,                    	// Please Wait
	AG_SERVICE_EXCEPTION,               // Out of Order
} Type_AgDisplay_Enum;

typedef struct {
	int (*init)();                      //display module init
	int (*display)(Type_AgDisplay_Enum);//display module set code number
} Type_Display_Def;
/***********************************************************************************/

//freq module
/***********************************************************************************/
typedef struct {
	int (*init)();                      //freq module init
	int (*run)(int, int);               //freq set run speed
	int (*getCurrent)(int);             //freq get current
	uint16_t(*getState)(int);           //freq get state
	uint32_t(*getError)(int);           //freq get error
	int (*clearError)(int);             //freq clear error
} Type_Freq_Def;
/***********************************************************************************/

//电机状态机枚举
typedef enum {
    MOTOR_STA_IDLE = 0,
    MOTOR_STA_ROTATION,
    MOTOR_STA_MOVE,
    MOTOR_STA_MOVE_FORE,
    MOTOR_STA_MOVE_POS,
    MOTOR_STA_MOVE_TIME,
    MOTOR_STA_PAUSE,
    MOTOR_STA_RESUME,
    MOTOR_STA_STOP,
    MOTOR_STA_FAULT,
} Type_MoveState_Enum;

//控制类型枚举
typedef enum {
	CRL_NULL = 0,
	//对称机构（如左右侧刷）
	CRL_ONLY_LEFT,
	CRL_ONLY_RIGHT,
	CRL_BOTH,
	//非对称机构，部件序号（如传送带1，2，3）
	CRL_SECTION_1,
	CRL_SECTION_2,
	CRL_SECTION_3,
	CRL_SECTION_4,
	CRL_SECTION_5,
	CRL_ALL_SECTION,
} Type_CrlType_Enum;
//限位模式枚举
typedef enum {
    MODE_SIGNAL_LIMIT = 0,
    MODE_SOFT_LIMIT_MIN,			//软限位模式在触碰到正/反向限位传感器后同样会停止
	MODE_SOFT_LIMIT_MAX,
	MODE_SOFT_LIMIT_MIN_MAX,
} Type_LimitMode_Enum;

//水相关控制量枚举
typedef enum {
	WATER_DRAIN = 0,
	WATER_DRAIN_WINTER,
	WATER_HIGH_PUMP,
	WATER_LOW_PUMP,
	WATER_SEWAGE_PUMP,
	WATER_SHAMPOO_PUMP,
	WATER_WAXWATER_PUMP,
	WATER_SWING_WATER,
	WATER_CLEAR_CONVEYOR_L,		//底盘冲洗
	WATER_CLEAR_CONVEYOR_R,
	WATER_WAXWATER,
	WATER_SHAMPOO,
	WATER_TOP,
	WATER_FRONT_SIDE,
	WATER_BACK_SIDE,
	// WATER_CONVEYOR_1,
	// WATER_CONVEYOR_2,
	// WATER_CONVEYOR_3,
	// WATER_MIDDLE,
	WATER_SHAMPOO_PIKN,
	WATER_SHAMPOO_GREEN,
	WATER_CLEAR_WATER,
	WATER_BASE_PLATE,
	WATER_ALL,
	WATER_CRL_NUM,
} Type_WaterSystem_Enum;

//设备驱动索引号
typedef enum {
    //士林变频器
	DRIVER_VFD_SHIHLIN_ID_0 = 0,
	DRIVER_VFD_SHIHLIN_ID_1,
	DRIVER_VFD_SHIHLIN_ID_2,
	DRIVER_VFD_SHIHLIN_ID_3,
	DRIVER_VFD_SHIHLIN_ID_4,
	DRIVER_VFD_SHIHLIN_ID_5,
	DRIVER_VFD_SHIHLIN_ID_6,
	DRIVER_VFD_SHIHLIN_ID_7,
	DRIVER_VFD_SHIHLIN_ID_8,

    //线圈（接触器/继电器）
	DRIVER_KM_ID_0,
	DRIVER_KM_ID_1,
	DRIVER_KM_ID_2,
	DRIVER_KM_ID_3,
	DRIVER_KM_ID_4,
	DRIVER_KM_ID_5,
	DRIVER_NUM,
} Type_DriverIndex_Enum;

//信号枚举
typedef enum{
    SIGNAL_GROUND = 0,
    SIGNAL_ALL_IN,
    SIGNAL_LEFT_SKEW,
    SIGNAL_RIGHT_SKEW,
	SIGNAL_SUPER_HIGH,
    SIGNAL_STOP,
    SIGNAL_ENTRANCE,
    SIGNAL_REAR_END_PROTECT,
	SIGNAL_AVOID_INTRUDE,
    SIGNAL_EXIT,
    SIGNAL_FINISH,
    SIGNAL_LIFTER_UP,
    SIGNAL_LIFTER_DOWN,
    SIGNAL_FL_BRUSH_DOWN,
    SIGNAL_FR_BRUSH_DOWN,
    SIGNAL_BL_BRUSH_DOWN,
    SIGNAL_BR_BRUSH_DOWN,
    SIGNAL_FL_MOVE_ZERO,
    SIGNAL_FR_MOVE_ZERO,
    SIGNAL_BL_MOVE_ZERO,
    SIGNAL_BR_MOVE_ZERO,
    SIGNAL_FL_BRUSH_CROOKED,
    SIGNAL_FR_BRUSH_CROOKED,
    SIGNAL_BL_BRUSH_CROOKED,
    SIGNAL_BR_BRUSH_CROOKED,
    SIGNAL_LEFT_SKIRT_ZERO,
    SIGNAL_RIGHT_SKIRT_ZERO,
    SIGNAL_LIFTER_LEFT_DETECH,
    SIGNAL_LIFTER_RIGHT_DETECH,
	SIGNAL_FL_COLLISION,
    SIGNAL_FR_COLLISION,
    SIGNAL_BL_COLLISION,
    SIGNAL_BR_COLLISION,
	SIGNAL_GATE_1_CLOSE,
	SIGNAL_GATE_1_OPEN,
	SIGNAL_GATE_2_LEFT_OPEN,
	SIGNAL_GATE_2_RIGHT_OPEN,
	SIGNAL_BUTTON_RESET,
	SIGNAL_BUTTON_START,
	SIGNAL_BUTTON_STOP,
	SIGNAL_BUTTON_PAUSE,
	SIGNAL_BUTTON_ENTRY_START,
	SIGNAL_PICKUP_TRUCK,
	SIGNAL_CAR_WHEEL_OUT,
	SIGNAL_WASH_MODE_1,
	SIGNAL_WASH_MODE_2,
	SIGNAL_WASH_MODE_3,
	SIGNAL_WASH_MODE_4,
	SIGNAL_GATE_1_PROTECT,
	SIGNAL_GIVE_UP_WASH,
    SIGNAL_NUM,
} Type_SignalType_Enum;
//信号信息
typedef struct{
	Type_SignalType_Enum	signalType;
    Type_InputIo_Enum   	matchIo;
	int                 	trigDir;
	uint8_t					trigCnt;
    uint32_t            	closePos;
    uint32_t            	leavePos;
} Type_SignalStaInfo_Def;

typedef struct {
	int init;                           //init status
	Type_Voice_Def      voice;			//语音
	Type_Display_Def    screen;			//显示
	Type_Freq_Def       freq;           //变频器
	// LITE_ht_sens ht_sens;
} Type_Driver_Def;


/***********************************************************************************/
//状态获取接口
extern Type_Driver_Def* xp_ag_osal_get(void);							//获取osal的控制对象
extern Type_SignalStaInfo_Def *get_signal_handle(Type_SignalType_Enum type);	//获取传感器信号的句柄
extern bool is_signal_filter_trigger(Type_SignalType_Enum type);		//获取传感器是否处于触发状态（有滤波）
extern int32_t xp_osal_get_dev_pos(Type_DriverIndex_Enum id);			//获取码盘机构的位置
extern Type_MoveState_Enum xp_osal_get_motor_move_state(Type_DriverIndex_Enum id);	//获取电机的运动状态
extern bool get_board_input_io(uint8_t pin);							//获取主板的输入IO状态（非真实，只是设置的状态）
extern bool get_board_output_io(uint8_t pin);							//获取主板的输出IO状态（非真实，只是设置的状态）
/***********************************************************************************/
//机构驱动接口
extern int xp_osal_brush_rotation(Type_DriverIndex_Enum id, int vel);
extern int xp_osal_move_run(Type_DriverIndex_Enum id, int vel);
extern int xp_osal_force_move_run(Type_DriverIndex_Enum id, int vel, uint16_t time);
extern int xp_osal_move_pos(Type_DriverIndex_Enum id, int vel, int32_t pos);
extern int xp_osal_move_time(Type_DriverIndex_Enum id, int vel, uint16_t time);
extern int xp_osal_move_pause(Type_DriverIndex_Enum id);
extern int xp_osal_move_resume(Type_DriverIndex_Enum id);
extern int xp_osal_move_stop(Type_DriverIndex_Enum id);
extern void dryer_work(Type_CrlType_Enum type, bool enable);
extern void water_system_control(Type_WaterSystem_Enum type, bool enable);
extern void osal_dev_io_state_change(Type_OutputIo_Enum index, bool sta);	//输出点位状态控制
/***********************************************************************************/
//其它接口
extern int xp_save_kv_params(char *pKeyStr, const void *pData, int len);
extern int xp_read_creat_kv_params(char *pKeyStr, void *pData, int *len);
extern bool get_pump_work_status_flag(void);
extern bool osal_is_io_trigger(Type_InputIo_Enum index);
extern int osal_set_dev_limit_mode(Type_DriverIndex_Enum id, Type_LimitMode_Enum mode, uint16_t minPos, uint16_t maxPos);
extern int clear_dev_encoder(Type_DriverIndex_Enum id);
extern void osal_error_upload_callback_regist(void (*callback)(uint16_t code, bool value));
extern void offline_payment_callback_regist(void (*callback)(uint8_t washMode));
#endif
