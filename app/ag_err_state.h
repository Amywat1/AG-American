/**
 * @file 	 ag_err_state.h
 * @brief       
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 * 
 * @copyright Copyright (c) 2022  YGL
 * 
 */
#ifndef __AG_ERR_STATE_H
#define __AG_ERR_STATE_H

#include <stdint.h>
#include "stdbool.h"

//错误处理模式
typedef enum {
    E_ERROR,                        //致命错误，立即停机上报                                    归位成功后清除报警
    E_WARNING,                      //警告错误，立即上报，工作区内车辆清洗结束后报警还在就停机     归位成功后清除报警
    E_NOTICE,                       //通知错误，上报但不停机                                    归位成功后清除报警
    E_NOTICE_AUTO_CLRAR,            //通知错误，上报但不停机 （报警根据检测状态清除）             归位成功不做状态清除
    ERROR_LEVEL_NUM,                //错误等级数量
} Type_ErrDealMode_Enum;

//错误信息定义
typedef struct {
    uint16_t                code;               //错误代码
    bool                    isErr;              //错误状态
    bool                    isTrigger;          //错误是否触发
    uint16_t                statusCnt;          //错误状态计数
    uint16_t                errOverTime;        //报警发生检测时间(ms)
    uint16_t                errRecoverTime;     //报警恢复检测时间(ms)
    Type_ErrDealMode_Enum   dealMode;           //错误处理模式
} Type_ErrStaInfo_Def;

//异常处理的标志位状态定义
typedef struct {
    bool isDevIdleSta;                  //设备是否处于待机状态
    bool isDevRunSta;                   //设备是否处于洗车状态
    bool isMqttConnected;               //是否联网
    bool isAllCollisionEnable;          //防撞检测总使能
    bool isFrontLeftCollisionEnable;    //左前竖防撞检测使能
    bool isFrontRightCollisionEnable;   //右前竖防撞检测使能
    bool isBackLeftCollisionEnable;     //左后竖防撞检测使能
    bool isBackRightCollisionEnable;    //右后竖防撞检测使能
    bool isDetectLifterLooseEnable;     //升降织带检测使能
    bool isDetectWaterPressEnable;      //水压检测使能
    bool isDetectSuperHighEnable;       //超高检测使能
    bool isDetectShampooEnable;         //香波检测功能
    bool isDetectWaxwaterEnable;        //蜡水检测功能
    bool isDetectCleanerLess;           //泥沙松动剂检测功能
    bool isDetectDyierLess;             //助干剂检测功能
    bool isDetectSewageEnable;          //污水液位检测功能
    bool isDetectGate1Enable;           //道闸1检测功能
    bool isDetectBrushCroooked;         //侧刷前后歪检测功能
    bool isSetElectricalReset;          //安全继电器是否复位
} Type_ErrStateFlag_Def;

/***********************************************************************************/
//回调
extern void xp_error_deal_callback_regist(void (*callback)(uint16_t code, Type_ErrDealMode_Enum mode)); //错误处理回调
extern void xp_error_upload_callback_regist(int (*callback)(char *arg));    //错误上报回调
/***********************************************************************************/
//状态获取或设置
extern Type_ErrStateFlag_Def *err_need_flag_handle(void);
extern bool get_allow_back_home_flag(void);
extern bool get_emc_power_off_sta(void);
extern uint32_t get_error_overtime(uint16_t errCode);
extern void set_error_state(uint16_t errCode, bool status);
extern bool get_error_state(uint16_t errCode);
extern bool get_attention_err_status(void);

#endif

