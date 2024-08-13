/**
 * @file 	 ag_err_state.c
 * @brief       
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 * 
 * @copyright Copyright (c) 2022  YGL
 * 
 */
#include "ag_err_state.h"
#include "ag_osal.h"

#define NDEFINE_TIME                    (0)         //未定义时间
#define VFD_NUM                         (9)         //变频器数量
#define VFD_DETECT_FREQ_TIME            (1000)      //变频器的循环检测时间(ms)
#define ERR_CHECK_SLEEP_TIME            (10)        //错误检测的等待时间(ms)
#define FREQ_ERR_T(T)                   (T)*ERR_CHECK_SLEEP_TIME/VFD_DETECT_FREQ_TIME             //设置变频器报警时间
#define ERR_TABLE_NUMBERS               (sizeof(ErrInfo_Table) / sizeof(ErrInfo_Table[0]))      //报警个数
#define EMC_MATCH_TABLE_ID              (0)         //急停报警在报警数组里的序号
#define ERR_CANT_BACK_HOME              (ErrInfo_Table[EMC_MATCH_TABLE_ID].isErr)

//错误信息查询表
//单个变频器的读取会阻塞30ms左右，所以实际超时时间会比定义的超时时间长
//流程超时类报警这里只做上报处理，停止动作在service.c中会处理
Type_ErrStaInfo_Def ErrInfo_Table[] = {
//  代码号   错误状态     错误触发    状态计数 触发时间             恢复时间             处理方式
    {8100,   false,      false,      0,      20,                 500,                E_ERROR},   //紧急停止按下,请复位-------->急停统一放第一行，便于管理
    // {8001,   false,      false,      0,      500,                500,                E_ERROR},   //1#道闸电机过载
    // {8002,   false,      false,      0,      500,                500,                E_ERROR},   //2#道闸电机过载
    // {8003,   false,      false,      0,      500,                500,                E_WARNING}, //顶部摆动电机过载（报警但延迟停机）
    // {8004,   false,      false,      0,      500,                500,                E_WARNING}, //左侧摆动电机过载
    // {8005,   false,      false,      0,      500,                500,                E_WARNING}, //右侧摆动电机过载
    // {8006,   false,      false,      0,      500,                500,                E_ERROR},   //高压泵过载
    // {8007,   false,      false,      0,      500,                500,                E_ERROR},   //低压泵过载（到达风干环节了再报警）
    // {8008,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机1电机过载（报警不停机）
    // {8009,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机2&5电机故障（报警不停机）
    // {8010,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机3&4电机故障（报警不停机）
    // {8013,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机6电机过载（报警不停机）
    {8014,   false,      false,      0,      500,                500,                E_WARNING}, //裙边电机过载（报警但延迟停机）
    // {8015,   false,      false,      0,      500,                500,                E_ERROR},   //左前刷开合电机过载
    // {8016,   false,      false,      0,      500,                500,                E_ERROR},   //右前刷开合电机过载
    // {8017,   false,      false,      0,      500,                500,                E_ERROR},   //左后刷开合电机过载
    // {8018,   false,      false,      0,      500,                500,                E_ERROR},   //右后刷开合电机过载
    {8019,   false,      false,      0,      500,                500,                E_ERROR},   //顶刷旋转变频器故障
    {8020,   false,      false,      0,      500,                500,                E_ERROR},   //顶刷上下变频器故障
    {8021,   false,      false,      0,      500,                500,                E_ERROR},   //1#输送带变频器故障
    {8022,   false,      false,      0,      500,                500,                E_ERROR},   //2#输送带变频器故障
    {8023,   false,      false,      0,      500,                500,                E_ERROR},   //3#输送带变频器故障
    {8024,   false,      false,      0,      500,                500,                E_ERROR},   //左前刷旋转变频器故障
    {8025,   false,      false,      0,      500,                500,                E_ERROR},   //右前刷旋转变频器故障
    {8026,   false,      false,      0,      500,                500,                E_ERROR},   //左后刷旋转变频器故障
    {8027,   false,      false,      0,      500,                500,                E_ERROR},   //右后刷旋转变频器故障
    {8030,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //顶刷上移超时未检测到限位信号（报警但延迟停机）
    {8031,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //顶刷下移超时未检测到限位信号（报警但延迟停机）
    {8032,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //左前刷开超时未检测到限位（报警但延迟停机）
    {8033,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //右前刷开超时未检测到限位（报警但延迟停机）
    {8034,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //左后刷开超时未检测到限位（报警但延迟停机）
    {8035,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //右后刷开超时未检测到限位（报警但延迟停机）
    {8036,   false,      false,      0,      2000,               500,                E_ERROR},   //顶刷左侧松动报警
    {8037,   false,      false,      0,      2000,               500,                E_ERROR},   //顶刷右侧松动报警
    // {8038,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //左裙边刷气缸回超时未检测到限位信号（报警但延迟停机）
    // {8039,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //右裙边刷气缸回超时未检测到限位信号（报警但延迟停机）
    {8040,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //1#道闸落杆超时未检测到限位信号（报警但延迟停机）
    {8041,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //1#道闸抬杆超时未检测到限位信号（报警但延迟停机）
    {8042,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //2#道闸落杆超时未检测到限位信号（报警但延迟停机）
    {8043,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //2#道闸抬杆超时未检测到限位信号（报警但延迟停机）
    {8050,   false,      false,      0,      1000,               500,                E_ERROR},   //左前侧刷歪报警（拉到开位继续洗，开位还报的话停机）
    {8051,   false,      false,      0,      1000,               500,                E_ERROR},   //右前侧刷歪报警（拉到开位继续洗）
    {8052,   false,      false,      0,      1000,               500,                E_ERROR},   //左后侧刷歪报警（拉到开位继续洗）
    {8053,   false,      false,      0,      1000,               500,                E_ERROR},   //右后侧刷歪报警（拉到开位继续洗）
    {8054,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_ERROR},   //顶刷旋转通讯异常
    {8055,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_ERROR},   //左前刷通讯异常
    {8056,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_ERROR},   //右前刷通讯异常
    {8057,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_ERROR},   //左后刷通讯异常
    {8058,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_ERROR},   //右后刷通讯异常
    {8059,   false,      false,      0,      20,                 500,                E_ERROR},   //左前防撞报警
    {8060,   false,      false,      0,      20,                 500,                E_ERROR},   //右前防撞报警
    {8061,   false,      false,      0,      20,                 500,                E_ERROR},   //左后防撞报警
    {8062,   false,      false,      0,      20,                 500,                E_ERROR},   //右后防撞报警
    {8063,   false,      false,      0,      500,                500,                E_NOTICE},  //语音模块通讯异常（报警但延迟停机）
    {8064,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_NOTICE},  //1#输送带通讯异常（报警但延迟停机）
    {8065,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_NOTICE},  //2#输送带通讯异常（报警但延迟停机）
    {8066,   false,      false,      0,      FREQ_ERR_T(3000),   FREQ_ERR_T(3000),   E_NOTICE},  //3#输送带通讯异常（报警但延迟停机）
    // {8067,   false,      false,      0,      500,                500,                E_NOTICE},  //风机2&5通讯异常（报警但延迟停机）
    // {8068,   false,      false,      0,      500,                500,                E_NOTICE},  //风机3&4通讯异常（报警但延迟停机）
    // {8069,   false,      false,      0,      20,                 500,                E_ERROR},   //压力波信号触发
    {8101,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //归位未完成请检查信号
    {8102,   false,      false,      0,      60000,              500,                E_NOTICE_AUTO_CLRAR},   //缺香波（蓝色）
    {8103,   false,      false,      0,      60000,              500,                E_NOTICE_AUTO_CLRAR},   //缺蜡水
    {8104,   false,      false,      0,      60000,              500,                E_NOTICE_AUTO_CLRAR},   //污水高液位
    // {8105,   false,      false,      0,      500,                500,                E_WARNING}, //相序报警
    {8106,   false,      false,      0,      5000,               500,                E_WARNING}, //水压不足
    {8107,   false,      false,      0,      500,                500,                E_WARNING}, //高度超高
    {8108,   false,      false,      0,      2000,               500,                E_WARNING}, //顶刷旋转电流超高
    {8109,   false,      false,      0,      500,                500,                E_WARNING}, //车头位置丢失报警（侧刷合30S未检测到车头）
    {8110,   false,      false,      0,      500,                500,                E_WARNING}, //前侧刷启动时低位报警（洗车头过程中侧刷合检测）
    {8111,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //洗车超时报警
    {8112,   false,      false,      0,      500,                500,                E_NOTICE},  //1#输送带码盘开关故障
    {8113,   false,      false,      0,      500,                500,                E_ERROR},   //2#输送带码盘开关故障
    {8114,   false,      false,      0,      500,                500,                E_ERROR},   //3#输送带码盘开关故障
    {8115,   false,      false,      0,      500,                500,                E_NOTICE},  //工作区防追尾被遮挡（待机的时候超10S报警）
    {8045,   false,      false,      0,      500,                500,                E_ERROR},   //地感信号故障（待机的时候超30S报警）
    // {8046,   false,      false,      0,      500,                500,                E_ERROR},   //左闸压力波信号故障
    // {8047,   false,      false,      0,      500,                500,                E_ERROR},   //右闸压力波信号故障
    {8048,   false,      false,      0,      500,                500,                E_ERROR},   //前刷低位故障（所有状态5S灭的就报警）
    {8049,   false,      false,      0,      500,                500,                E_ERROR},   //后刷低位故障（所有状态5S灭的就报警）
    {8116,   false,      false,      0,      500,                500,                E_ERROR},   //1#道闸开关同时亮
    {8117,   false,      false,      0,      500,                500,                E_ERROR},   //2#道闸开关同时亮
    {8118,   false,      false,      0,      500,                500,                E_ERROR},   //顶刷上下同时亮
    {8119,   false,      false,      0,      2000,               500,                E_ERROR},   //左前刷旋转电流异常
    {8120,   false,      false,      0,      2000,               500,                E_ERROR},   //右前刷旋转电流异常
    {8121,   false,      false,      0,      2000,               500,                E_ERROR},   //左后刷旋转电流异常
    {8122,   false,      false,      0,      2000,               500,                E_ERROR},   //右后刷旋转电流异常
    // {8123,   false,      false,      0,      500,                500,                E_WARNING}, //后裙边电机过载（报警但延迟停机）
    // {8124,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //左后裙边刷气缸回超时未检测到限位信号（报警但延迟停机）
    // {8125,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_WARNING}, //右后裙边刷气缸回超时未检测到限位信号（报警但延迟停机）
    // {8126,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE_AUTO_CLRAR},  //有车辆闯入
    {273,    false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //用户使用手机APP停止
    {300,    false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_ERROR},   //顶刷计数异常
    {309,    false,      false,      0,      2000,               NDEFINE_TIME,       E_ERROR},   //毛刷跟随异常
    {310,    false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_ERROR},   //左前刷计数异常
    {311,    false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_ERROR},   //右前刷计数异常
    {312,    false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_ERROR},   //左后刷计数异常
    {313,    false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_ERROR},   //右后刷计数异常
    {500,    false,      false,      0,      1000,               NDEFINE_TIME,       E_ERROR},   //第一块子板掉线
    {501,    false,      false,      0,      1000,               NDEFINE_TIME,       E_ERROR},   //第二块子板掉线
    {502,    false,      false,      0,      1000,               NDEFINE_TIME,       E_ERROR},   //第三块子板掉线
    {503,    false,      false,      0,      1000,               NDEFINE_TIME,       E_ERROR},   //第四块子板掉线
    {504,    false,      false,      0,      1000,               NDEFINE_TIME,       E_ERROR},   //第五块子板掉线
    {505,    false,      false,      0,      1000,               NDEFINE_TIME,       E_ERROR},   //第六块子板掉线
    {8300,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //工作区输送带停止时间超时
    {8200,   false,      false,      0,      60000,              500,                E_NOTICE_AUTO_CLRAR},   //缺香波（绿色）
    {8201,   false,      false,      0,      60000,              500,                E_NOTICE_AUTO_CLRAR},   //缺香波（粉色）
    {8202,   false,      false,      0,      60000,              500,                E_NOTICE_AUTO_CLRAR},   //缺泥沙松动剂
    {8203,   false,      false,      0,      60000,              500,                E_NOTICE_AUTO_CLRAR},   //缺助干剂
    {8204,   false,      false,      0,      500,                500,                E_ERROR},   //左前刷开合电机故障
    {8205,   false,      false,      0,      500,                500,                E_ERROR},   //右前刷开合电机故障
    {8206,   false,      false,      0,      500,                500,                E_ERROR},   //左后刷开合电机故障
    {8207,   false,      false,      0,      500,                500,                E_ERROR},   //右后刷开合电机故障
    {8208,   false,      false,      0,      500,                500,                E_WARNING}, //顶部摆动变频器故障（报警但延迟停机）
    {8209,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机1变频器故障（报警不停机）
    {8210,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机2变频器故障（报警不停机）
    {8211,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机3变频器故障（报警不停机）
    {8212,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机4变频器故障（报警不停机）
    {8213,   false,      false,      0,      500,                500,                E_NOTICE_AUTO_CLRAR},   //风机5变频器故障（报警不停机）
    {8214,   false,      false,      0,      500,                500,                E_WARNING}, //裙边刷变频器故障（报警但延迟停机）
    {329,    false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //升降归位超时  （这里的单个超时报警只通知，处理的话有一个总的归位超时报警会处理）
    {8229,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //左前刷归位超时
    {8230,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //右前刷归位超时
    {8231,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //左后刷归位超时
    {8232,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //右后刷归位超时
    {8233,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //1#道闸归位超时
    {8234,   false,      false,      0,      NDEFINE_TIME,       NDEFINE_TIME,       E_NOTICE},  //2#道闸归位超时
};

//错误状态定义
typedef struct {
    bool init;                                  //异常初始化
    bool isAlloweBackHome;                      //是否允许归位
    bool isEmcSta;                              //是否是急停状态（急停刚恢复未稳定时也是急停状态）
    bool isNeedAttention;                       //是否有需要关注的报警
    char aggregate[500];
    aos_queue_t status;                         //错误状态消息队列
} Type_ErrInfo_Def;
static Type_ErrInfo_Def error = {0};

static Type_ErrStateFlag_Def errNeedFlag = {0};

static void (*xp_error_deal_callback)(uint16_t code, Type_ErrDealMode_Enum mode);
static int (*xp_error_upload_callback)(char *arg);
static void xp_error_upload_thread(void *arg);
static void xp_check_error_thread(void *arg);

/*                                                         =======================                                                         */
/* ========================================================        回调相关        ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       注册报警处理函数
 * @param[in]	callback            
 */
void xp_error_deal_callback_regist(void (*callback)(uint16_t code, Type_ErrDealMode_Enum mode))
{
    xp_error_deal_callback = callback;
}

/**
 * @brief       注册报警上传函数
 * @param[in]	callback            
 */
void xp_error_upload_callback_regist(int (*callback)(char *arg))
{
    xp_error_upload_callback = callback;
}

/**
 * @brief       错误上报回调
 * @param[in]	code                
 */
void osal_error_upload_callback(uint16_t code, bool value)
{
    for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
    {
        if(ErrInfo_Table[i].code == code){
            if((8040 == code || 8041 == code) && !errNeedFlag.isDetectGate1Enable){
                ErrInfo_Table[i].isErr = false;
            }
            else{
                ErrInfo_Table[i].isErr = value;
            }
            break;
        }
    }
}

/*                                                         =======================                                                         */
/* ========================================================         初始化         ======================================================== */
/*                                                         =======================                                                         */

/**
 * @brief       错误更新初始化
 * @param[in]	mq                  
 * @return      int                 
 */
int xp_error_state_init(void)
{
    aos_task_t id;

    xp_error_deal_callback = NULL;
    xp_error_upload_callback = NULL;

    if(aos_task_new("error_upload", xp_error_upload_thread, NULL, 8192) != 0){
        return -1;
    }
    if(aos_task_new_ext(&id,"check_error", xp_check_error_thread, NULL, 8192, 30) != 0){
        return -2;
    }

    osal_error_upload_callback_regist(osal_error_upload_callback);

    void *errStaBuff = aos_malloc(sizeof(ErrInfo_Table)*5);
    if(aos_queue_new(&error.status, errStaBuff, sizeof(ErrInfo_Table)*5, sizeof(ErrInfo_Table)) != 0){
        return -3;
    }

    error.init = 1;
    error.isAlloweBackHome = true;
    error.isEmcSta = true;
    LOG_INFO("error module init success~");
    return 0;
}

/**
 * @brief       报警监测
 * @param[in]	isEmc               当前是否急停状态
 * @return      bool                
 */
static bool check_ag_error(bool isEmc){
    static uint8_t vfdDetectCnt = VFD_NUM;          //需要检测的变频器数量
    static uint16_t vfdErrSta[VFD_NUM] = {0};
    static uint8_t vfdFreqCnt = 0;
    Type_DriverIndex_Enum errDev;
    bool isDetectVfd = false;
    bool errSta = false;

    if(0 == vfdFreqCnt++ % (VFD_DETECT_FREQ_TIME/VFD_NUM/ERR_CHECK_SLEEP_TIME)){
        //设备在工作状态或者急停状态不检测变频器通讯相关异常（变频器控制都是用点位控制，通讯只是用于读取报警值还有电流值）
        isDetectVfd = (errNeedFlag.isDevRunSta || isEmc) ? false : true;
        //有变频器通讯异常后，一直检测单个变频器通讯，避免通讯不上线程阻塞导致多个变频器通讯检测混乱
        if(get_error_state(8054))      vfdDetectCnt = TOP_BRUSH_MATCH_ID;
        else if(get_error_state(8055)) vfdDetectCnt = FRONT_LEFT_BRUSH_MATCH_ID;
        else if(get_error_state(8056)) vfdDetectCnt = FRONT_RIGHT_BRUSH_MATCH_ID;
        else if(get_error_state(8057)) vfdDetectCnt = BACK_LEFT_BRUSH_MATCH_ID;
        else if(get_error_state(8058)) vfdDetectCnt = BACK_RIGHT_BRUSH_MATCH_ID;
        else if(get_error_state(8064)) vfdDetectCnt = CONVEYOR_1_MATCH_ID;
        else if(get_error_state(8065)) vfdDetectCnt = CONVEYOR_2_MATCH_ID;
        else if(get_error_state(8066)) vfdDetectCnt = CONVEYOR_3_MATCH_ID;
        else{
            vfdDetectCnt = (vfdDetectCnt < VFD_NUM - 1) ? ++vfdDetectCnt : 0;
        }
    }
    else{
        isDetectVfd = false;
    }

    //循环判定所有报警的状态
    for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
    {
        bool isError = false;
        switch (ErrInfo_Table[i].code)
        {
        case 8019:   //顶刷旋转变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD5) && osal_is_io_trigger(BOARD5_INPUT_TOP_BRUSH_ERR)) ? true : false;
            break;
        case 8020:   //顶刷上下变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD5) && osal_is_io_trigger(BOARD5_INPUT_LIFTER_ERR)) ? true : false;
            break;
        case 8021:   //1#输送带变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_CONVEYOR_1_ERR)) ? true : false;
            break;
        case 8022:   //2#输送带变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_CONVEYOR_2_ERR)) ? true : false;
            break;
        case 8023:   //3#输送带变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_CONVEYOR_3_ERR)) ? true : false;
            break;
        case 8024:   //左前刷旋转变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD3) && osal_is_io_trigger(BOARD3_INPUT_FRONT_LEFT_ROTATION_ERR)) ? true : false;
            break;
        case 8025:   //右前刷旋转变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD3) && osal_is_io_trigger(BOARD3_INPUT_FRONT_RIGHT_ROTATION_ERR)) ? true : false;
            break;
        case 8026:   //左后刷旋转变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD3) && osal_is_io_trigger(BOARD3_INPUT_BACK_LEFT_ROTATION_ERR)) ? true : false;
            break;
        case 8027:   //右后刷旋转变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD3) && osal_is_io_trigger(BOARD3_INPUT_BACK_RIGHT_ROTATION_ERR)) ? true : false;
            break;
        case 8036:   //顶刷左侧松动报警
            isError = (errNeedFlag.isDetectLifterLooseEnable
                    && xp_io_get_online_sta(IO_BOARD4) && !osal_is_io_trigger(BOARD4_INPUT_LIFTER_LEFT_DETECH)) ? true : false;
            break;
        case 8037:   //顶刷右侧松动报警
            isError = (errNeedFlag.isDetectLifterLooseEnable
                    && xp_io_get_online_sta(IO_BOARD4) && !osal_is_io_trigger(BOARD4_INPUT_LIFTER_RIGHT_DETECH)) ? true : false;
            break;
        // case 8050:   //左前侧刷歪报警
        //     isError = (errNeedFlag.isDetectBrushCroooked
        //             && xp_io_get_online_sta(IO_BOARD2) && !osal_is_io_trigger(BOARD2_INPUT_FRONT_RIGHT_BRUSH_CROOKED)) ? true : false;
        //     break;
        case 8051:   //右前侧刷歪报警
            isError = (errNeedFlag.isDetectBrushCroooked
                    && xp_io_get_online_sta(IO_BOARD2) && !osal_is_io_trigger(BOARD2_INPUT_FRONT_RIGHT_BRUSH_CROOKED)) ? true : false;
            break;
        // case 8052:   //左后侧刷歪报警
        //     isError = (errNeedFlag.isDetectBrushCroooked
        //             && xp_io_get_online_sta(IO_BOARD2) && !osal_is_io_trigger(BOARD2_INPUT_BACK_RIGHT_BRUSH_CROOKED)) ? true : false;
        //     break;
        case 8053:   //右后侧刷歪报警
            isError = (errNeedFlag.isDetectBrushCroooked
                    && xp_io_get_online_sta(IO_BOARD2) && !osal_is_io_trigger(BOARD2_INPUT_BACK_RIGHT_BRUSH_CROOKED)) ? true : false;
            break;
        case 8054:   //顶刷旋转通讯异常
            if(!isDetectVfd || vfdDetectCnt != TOP_BRUSH_MATCH_ID)   continue;
            vfdErrSta[TOP_BRUSH_MATCH_ID] = xp_ag_osal_get()->freq.getState(TOP_BRUSH_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[TOP_BRUSH_MATCH_ID]) ? true : false;
            break;
        case 8055:   //左前刷通讯异常
            if(!isDetectVfd || vfdDetectCnt != FRONT_LEFT_BRUSH_MATCH_ID)   continue;
            vfdErrSta[FRONT_LEFT_BRUSH_MATCH_ID] = xp_ag_osal_get()->freq.getState(FRONT_LEFT_BRUSH_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[FRONT_LEFT_BRUSH_MATCH_ID]) ? true : false;
            break;
        case 8056:   //右前刷通讯异常
            if(!isDetectVfd || vfdDetectCnt != FRONT_RIGHT_BRUSH_MATCH_ID)   continue;
            vfdErrSta[FRONT_RIGHT_BRUSH_MATCH_ID] = xp_ag_osal_get()->freq.getState(FRONT_RIGHT_BRUSH_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[FRONT_RIGHT_BRUSH_MATCH_ID]) ? true : false;
            break;
        case 8057:   //左后刷通讯异常
            if(!isDetectVfd || vfdDetectCnt != BACK_LEFT_BRUSH_MATCH_ID)   continue;
            vfdErrSta[BACK_LEFT_BRUSH_MATCH_ID] = xp_ag_osal_get()->freq.getState(BACK_LEFT_BRUSH_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[BACK_LEFT_BRUSH_MATCH_ID]) ? true : false;
            break;
        case 8058:   //右后刷通讯异常
            if(!isDetectVfd || vfdDetectCnt != BACK_RIGHT_BRUSH_MATCH_ID)   continue;
            vfdErrSta[BACK_RIGHT_BRUSH_MATCH_ID] = xp_ag_osal_get()->freq.getState(BACK_RIGHT_BRUSH_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[BACK_RIGHT_BRUSH_MATCH_ID]) ? true : false;
            break;
        case 8059:   //左前防撞报警
            isError = (errNeedFlag.isAllCollisionEnable && errNeedFlag.isFrontLeftCollisionEnable 
                    && xp_io_get_online_sta(IO_BOARD4) && !osal_is_io_trigger(BOARD4_INPUT_FRONT_LEFT_COLLISION)) ? true : false;
            break;
        case 8060:   //右前防撞报警
            isError = (errNeedFlag.isAllCollisionEnable && errNeedFlag.isFrontRightCollisionEnable 
                    && xp_io_get_online_sta(IO_BOARD4) && !osal_is_io_trigger(BOARD4_INPUT_FRONT_RIGHT_COLLISION)) ? true : false;
            break;
        case 8061:   //左后防撞报警
            isError = (errNeedFlag.isAllCollisionEnable && errNeedFlag.isBackLeftCollisionEnable 
                    && xp_io_get_online_sta(IO_BOARD4) && !osal_is_io_trigger(BOARD0_INPUT_BACK_LEFT_COLLISION)) ? true : false;
            break;
        case 8062:   //右后防撞报警
            isError = (errNeedFlag.isAllCollisionEnable && errNeedFlag.isBackRightCollisionEnable 
                    && xp_io_get_online_sta(IO_BOARD0) && !osal_is_io_trigger(BOARD0_INPUT_BACK_RIGHT_COLLISION)) ? true : false;
            break;
        // case 8063:   //语音模块通讯异常
        //     isError = 
        //     break;
        case 8064:    //1#输送带通讯异常
            if(!isDetectVfd || vfdDetectCnt != CONVEYOR_1_MATCH_ID)   continue;
            vfdErrSta[CONVEYOR_1_MATCH_ID] = xp_ag_osal_get()->freq.getState(CONVEYOR_1_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[CONVEYOR_1_MATCH_ID]) ? true : false;
            break;
        case 8065:    //2#输送带通讯异常
            if(!isDetectVfd || vfdDetectCnt != CONVEYOR_2_MATCH_ID)   continue;
            vfdErrSta[CONVEYOR_2_MATCH_ID] = xp_ag_osal_get()->freq.getState(CONVEYOR_2_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[CONVEYOR_2_MATCH_ID]) ? true : false;
            break;
        case 8066:    //3#输送带通讯异常
            if(!isDetectVfd || vfdDetectCnt != CONVEYOR_3_MATCH_ID)   continue;
            vfdErrSta[CONVEYOR_3_MATCH_ID] = xp_ag_osal_get()->freq.getState(CONVEYOR_3_MATCH_ID);
            isError = (0xFFFF == vfdErrSta[CONVEYOR_3_MATCH_ID]) ? true : false;
            break;
        // case 8067:   //风机2&5通讯异常
        //     isError = ;
        //     break;
        // case 8068:   //风机3&4通讯异常
        //     isError = ;
        //     break;
        // case 8069:   //压力波信号触发
        //     isError = ;
        //     break;
        case 8100:   //紧急停止按下,请复位（用主电路有电信号做判断，不直接检测急停按钮信号）
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_POWER_ON)) ? true : false;
            break;
        case 8102:   //缺香波（蓝色）
            isError = (errNeedFlag.isDetectShampooEnable
                    && xp_io_get_online_sta(IO_BOARD1) && osal_is_io_trigger(BOARD1_INPUT_SHAMPOO_LESS_BULE)) ? true : false;
            break;
        case 8103:   //缺蜡水
            isError = (errNeedFlag.isDetectWaxwaterEnable
                    && xp_io_get_online_sta(IO_BOARD1) && osal_is_io_trigger(BOARD1_INPUT_WAXWATER_LESS)) ? true : false;
            break;
        case 8106:   //水压不足
            isError = (errNeedFlag.isDetectWaterPressEnable
                    && true == get_pump_work_status_flag() && xp_io_get_online_sta(IO_BOARD1) 
                    && osal_is_io_trigger(BOARD1_INPUT_WATER_PRESS_LOW_1) && osal_is_io_trigger(BOARD1_INPUT_WATER_PRESS_LOW_2)) ? true : false;
            break;
        case 8107:   //高度超高
            isError = (errNeedFlag.isDetectSuperHighEnable 
                    && xp_io_get_online_sta(IO_BOARD4) && !osal_is_io_trigger(BOARD4_INPUT_SUPER_HIGH)) ? true : false;
            break;
        // case 8109:   //车头位置丢失报警
        //     isError = ;
        //     break;
        // case 8110:   //前侧刷启动时低位报警
        //     isError = ;
        //     break;
        // case 8115:   //工作区防追尾被遮挡
        //     isError = ;
        //     break;
        // case 8045:   //地感信号故障
        //     isError = ;
        //     break;
        // case 8046:   //左闸压力波信号故障
        //     isError = ;
        //     break;
        // case 8047:   //右闸压力波信号故障
        //     isError = ;
        //     break;
        // case 8048:   //前刷低位故障
        //     isError = ;
        //     break;
        // case 8049:   //后刷低位故障
        //     isError = ;
        //     break;
        case 8116:   //1#道闸开关同时亮
            isError = (xp_io_get_online_sta(IO_BOARD5) 
                    && osal_is_io_trigger(BOARD5_INPUT_GATE_1_OPEN_DONE)
                    && osal_is_io_trigger(BOARD5_INPUT_GATE_1_CLOSE_DONE)) ? true : false;
            break;
        case 8118:   //顶刷上下同时亮
            isError = (xp_io_get_online_sta(IO_BOARD4) 
                    && osal_is_io_trigger(BOARD4_INPUT_LIFTER_UP)
                    && osal_is_io_trigger(BOARD4_INPUT_LIFTER_DOWN)) ? true : false;
            break;
        case 500:    //第一块子板掉线
            isError = (!xp_io_get_online_sta(IO_BOARD0)) ? true : false;
            break;
        case 501:    //第二块子板掉线
            isError = (!xp_io_get_online_sta(IO_BOARD1)) ? true : false;
            break;
        case 502:    //第三块子板掉线
            isError = (!xp_io_get_online_sta(IO_BOARD2)) ? true : false;
            break;
        case 503:    //第四块子板掉线
            isError = (!xp_io_get_online_sta(IO_BOARD3)) ? true : false;
            break;
        case 504:    //第五块子板掉线
            isError = (!xp_io_get_online_sta(IO_BOARD4)) ? true : false;
            break;
        case 505:    //第六块子板掉线
            isError = (!xp_io_get_online_sta(IO_BOARD5)) ? true : false;
            break;
        case 8200:   //缺香波（绿色）
            isError = (errNeedFlag.isDetectShampooEnable
                    && xp_io_get_online_sta(IO_BOARD1) && osal_is_io_trigger(BOARD1_INPUT_SHAMPOO_LESS_GREEN)) ? true : false;
            break;
        case 8201:   //缺香波（粉色）
            isError = (errNeedFlag.isDetectShampooEnable
                    && xp_io_get_online_sta(IO_BOARD1) && osal_is_io_trigger(BOARD1_INPUT_SHAMPOO_LIQUID_PINK)) ? true : false;
            break;
        case 8202:   //缺泥沙松动剂
            isError = (errNeedFlag.isDetectCleanerLess
                    && xp_io_get_online_sta(IO_BOARD1) && osal_is_io_trigger(BOARD1_INPUT_CLEANER_LESS)) ? true : false;
            break;
        case 8203:   //缺助干剂
            isError = (errNeedFlag.isDetectDyierLess
                    && xp_io_get_online_sta(IO_BOARD1) && osal_is_io_trigger(BOARD1_INPUT_DRIER_LESS)) ? true : false;
            break;
        case 8204:   //左前刷开合电机故障
            isError = (xp_io_get_online_sta(IO_BOARD2) && osal_is_io_trigger(BOARD2_INPUT_FRONT_LEFT_MOVE_ERR)) ? true : false;
            break;
        case 8205:   //右前刷开合电机故障
            isError = (xp_io_get_online_sta(IO_BOARD2) && osal_is_io_trigger(BOARD2_INPUT_FRONT_RIGHT_MOVE_ERR)) ? true : false;
            break;
        case 8206:   //左后刷开合电机故障
            isError = (xp_io_get_online_sta(IO_BOARD2) && osal_is_io_trigger(BOARD2_INPUT_BACK_LEFT_MOVE_ERR)) ? true : false;
            break;
        case 8207:   //右后刷开合电机故障
            isError = (xp_io_get_online_sta(IO_BOARD2) && osal_is_io_trigger(BOARD2_INPUT_BACK_RIGHT_MOVE_ERR)) ? true : false;
            break;
        case 8208:   //顶部摆动变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD5) && osal_is_io_trigger(BOARD5_INPUT_TOP_SWING_ERR)) ? true : false;
            break;
        case 8209:   //风机1变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_DRYER_1_ERR)) ? true : false;
            break;
        case 8210:   //风机2变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_DRYER_2_ERR)) ? true : false;
            break;
        case 8211:   //风机3变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_DRYER_3_ERR)) ? true : false;
            break;
        case 8212:   //风机4变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_DRYER_4_ERR)) ? true : false;
            break;
        case 8213:   //风机5变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD0) && osal_is_io_trigger(BOARD0_INPUT_DRYER_5_ERR)) ? true : false;
            break;
        case 8014:   //裙边刷变频器故障
            isError = (xp_io_get_online_sta(IO_BOARD5) 
            && (osal_is_io_trigger(BOARD5_INPUT_FRONT_LEFT_SKIRT_ERR) || osal_is_io_trigger(BOARD5_INPUT_FRONT_RIGHT_SKIRT_ERR))) ? true : false;
            break;
        default:
            continue;
            break;
        }
        errSta |= isError;

        if(isError){
            if(!ErrInfo_Table[i].isErr){
                if(!ErrInfo_Table[i].isTrigger){
                    ErrInfo_Table[i].statusCnt = 0;
                }
                ErrInfo_Table[i].statusCnt++;
                if(ErrInfo_Table[i].statusCnt >= (ErrInfo_Table[i].errOverTime / ERR_CHECK_SLEEP_TIME)){
                    ErrInfo_Table[i].isErr = true;
                }
            }
            ErrInfo_Table[i].isTrigger = true;
        }
        else{
            if(ErrInfo_Table[i].isErr){
                if(ErrInfo_Table[i].isTrigger){
                    ErrInfo_Table[i].statusCnt = 0;
                }
                ErrInfo_Table[i].statusCnt++;
                if(ErrInfo_Table[i].statusCnt >= (ErrInfo_Table[i].errRecoverTime / ERR_CHECK_SLEEP_TIME)){
                    ErrInfo_Table[i].isErr = false;
                }
            }
            ErrInfo_Table[i].isTrigger = false;
        }
    }
    return errSta;
}

/**
 * @brief       错误代码字符串获取
 * @param[in]	code                错误代码
 * @param[in]	num                 待判定的报警个数          
 */
static char *xp_errState_string(Type_ErrStaInfo_Def const *code, uint8_t num)
{
    char buf[256] = {0};
    char temp_buf[10] = {0};
    uint8_t errNums = 0;        //错误个数

    for(uint8_t i = 0; i < num; i++){
        if(code != NULL && code->isErr){
            if(errNums > 0){
                strcat(buf,",");
            }
            sprintf(temp_buf, "%d", code->code);
            strcat(buf, temp_buf);
            errNums++;
        }
        code++;

        if(strlen(buf) > 200){
            break;
        }
    }
    
    if(0 == errNums){
        strcpy(error.aggregate, "0");
    }
    else{
        strcpy(error.aggregate, buf);
    }
    return error.aggregate;
}

/**
 * @brief       错误上传
 * @param[in]	code                错误代码
 * @param[in]	num                 待判定的报警个数
 * @return      int                 
 */
static int xp_error_upload(Type_ErrStaInfo_Def *code, uint8_t num){
    char postBuf[512] = {0};

    if(error.init != 1){
        LOG_UPLOAD("error module no init~");
        return -1;
    }

    char *j = xp_errState_string(code, num);
    sprintf(postBuf,"{\"error\":\"%s\"}", j);
    LOG_INFO("error json:%s", postBuf);

    if(xp_error_upload_callback != NULL)
    {
        return xp_error_upload_callback(postBuf);
    }
    return -2;
}

/**
 * @brief       远程错误更新线程
 * @param[in]	arg                 
 */
static void xp_error_upload_thread(void *arg)
{
    uint32_t len;
    Type_ErrStaInfo_Def errCode[ERR_TABLE_NUMBERS] = {0};
    int ret;

    aos_msleep(1000);        //等待初始化
    if(error.init != 1){
        LOG_UPLOAD("Error module no init~ exit thread~");
        aos_task_exit(0);
    }

    while(1){
        aos_queue_recv(&error.status, AOS_WAIT_FOREVER, &errCode, &len);
        ret = xp_error_upload(errCode, ERR_TABLE_NUMBERS);
        if(ret != 0){
            LOG_UPLOAD("Error upload fault, ret %d", ret);
        }
    }
}

/**
 * @brief       错误检查线程
 * @param[in]	arg                 
 */
static void xp_check_error_thread(void *arg)
{
    bool dealErr[ERR_TABLE_NUMBERS] = {0};      //处理的报警状态存储值
    bool uploadErr[ERR_TABLE_NUMBERS] = {0};    //上传的报警状态存储值
    uint64_t powerOnHoldTimeStamp = 0;
    uint8_t errPrintCylCnt = 0;

    aos_msleep(5000);
    memset(uploadErr, 1, sizeof(uploadErr));    //初始化上报的错误记录值全为1（避免断电前有报警，重新上电后没报警，没有上传空报警，导致上次的报警状态一直存在）
    while(1)
    {
        if(false == xp_ag_osal_get()->init){
            aos_msleep(500);
            LOG_INFO("Wait osal init...");
            continue;
        }

        check_ag_error(error.isEmcSta);
        if(ErrInfo_Table[EMC_MATCH_TABLE_ID].isErr){
            error.isEmcSta = true;
            if(is_signal_filter_trigger(SIGNAL_BUTTON_RESET) || errNeedFlag.isSetElectricalReset){ //检测到复位按钮后急停复位
                errNeedFlag.isSetElectricalReset = false;
                osal_dev_io_state_change(BOARD1_OUTPUT_SAFE_RELAY_RESET, IO_ENABLE);
                aos_msleep(500);
                osal_dev_io_state_change(BOARD1_OUTPUT_SAFE_RELAY_RESET, IO_DISABLE);
                powerOnHoldTimeStamp = aos_now_ms();
            }
        }
        else{
            if(!error.isEmcSta){
                powerOnHoldTimeStamp = aos_now_ms();
            }
            else{
                if(get_diff_ms(powerOnHoldTimeStamp) > 5000){       //上电后等待一段时间再检测被急停断电的设备
                    error.isEmcSta = false;
                }
            }
        }

        bool isUploadErrTrig = false;
        error.isNeedAttention = false;
        errPrintCylCnt++;
        for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
        {
            //判定有没有需要关注的报警，除自动清除类报警外都认为是错误状态，归位成功后进行清除
            if(ErrInfo_Table[i].isErr){
                if(ErrInfo_Table[i].dealMode != E_NOTICE_AUTO_CLRAR) error.isNeedAttention = true;
                if(0 == errPrintCylCnt % 100) LOG_DEBUG("Have err trigger, code %d", ErrInfo_Table[i].code);
            }
            //判定有没有需要处理的错误状态改变
            if((E_ERROR == ErrInfo_Table[i].dealMode || E_WARNING == ErrInfo_Table[i].dealMode)
            && dealErr[i] != ErrInfo_Table[i].isErr){
                if(ErrInfo_Table[i].isErr){            //有新的报警状态才处理
                    if(xp_error_deal_callback != NULL){
                        xp_error_deal_callback(ErrInfo_Table[i].code, ErrInfo_Table[i].dealMode);
                    }
                    //变频器相关报警在异常时读取报警代码
                    if(8019 == ErrInfo_Table[i].code) LOG_UPLOAD("TOP_BRUSH err code %d", xp_ag_osal_get()->freq.getError(TOP_BRUSH_MATCH_ID));
                    if(8020 == ErrInfo_Table[i].code) LOG_UPLOAD("LIFTER err code %d", xp_ag_osal_get()->freq.getError(LIFTER_MATCH_ID));
                    if(8021 == ErrInfo_Table[i].code) LOG_UPLOAD("CONVEYOR_1 err code %d", xp_ag_osal_get()->freq.getError(CONVEYOR_1_MATCH_ID));
                    if(8022 == ErrInfo_Table[i].code) LOG_UPLOAD("CONVEYOR_2 err code %d", xp_ag_osal_get()->freq.getError(CONVEYOR_2_MATCH_ID));
                    if(8023 == ErrInfo_Table[i].code) LOG_UPLOAD("CONVEYOR_3 err code %d", xp_ag_osal_get()->freq.getError(CONVEYOR_3_MATCH_ID));
                    if(8024 == ErrInfo_Table[i].code) LOG_UPLOAD("FRONT_LEFT_BRUSH err code %d", xp_ag_osal_get()->freq.getError(FRONT_LEFT_BRUSH_MATCH_ID));
                    if(8025 == ErrInfo_Table[i].code) LOG_UPLOAD("FRONT_RIGHT_BRUSH err code %d", xp_ag_osal_get()->freq.getError(FRONT_RIGHT_BRUSH_MATCH_ID));
                    if(8026 == ErrInfo_Table[i].code) LOG_UPLOAD("BACK_LEFT_BRUSH err code %d", xp_ag_osal_get()->freq.getError(BACK_LEFT_BRUSH_MATCH_ID));
                    if(8027 == ErrInfo_Table[i].code) LOG_UPLOAD("BACK_RIGHT_BRUSH err code %d", xp_ag_osal_get()->freq.getError(BACK_RIGHT_BRUSH_MATCH_ID));
                    if(8204 == ErrInfo_Table[i].code){
                        uint32_t errCode = 0xFFFFFFFF;
                        xp_lnovance_get_error_code(9, &errCode);
                        LOG_UPLOAD("FRONT_LEFT_MOVE err code %d", errCode);
                    }
                    if(8205 == ErrInfo_Table[i].code){
                        uint32_t errCode = 0xFFFFFFFF;
                        xp_lnovance_get_error_code(10, &errCode);
                        LOG_UPLOAD("FRONT_RIGHT_MOVE err code %d", errCode);
                    }
                    if(8206 == ErrInfo_Table[i].code){
                        uint32_t errCode = 0xFFFFFFFF;
                        xp_lnovance_get_error_code(11, &errCode);
                        LOG_UPLOAD("BACK_LEFT_MOVE err code %d", errCode);
                    }
                    if(8207 == ErrInfo_Table[i].code){
                        uint32_t errCode = 0xFFFFFFFF;
                        xp_lnovance_get_error_code(12, &errCode);
                        LOG_UPLOAD("BACK_RIGHT_MOVE err code %d", errCode);
                    }
                    if(8208 == ErrInfo_Table[i].code){
                        uint32_t errCode = 0xFFFFFFFF;
                        xp_lnovance_get_error_code(13, &errCode);
                        LOG_UPLOAD("TOP_SWING err code %d", errCode);
                    }
                    if(8214 == ErrInfo_Table[i].code){
                        uint32_t errCode = 0xFFFFFFFF;
                        xp_lnovance_get_error_code(14, &errCode);
                        LOG_UPLOAD("SKIRT_BRUSH_LEFT err code %d", errCode);
                        errCode = 0xFFFFFFFF;
                        aos_msleep(10);             //临时添加
                        xp_lnovance_get_error_code(15, &errCode);
                        LOG_UPLOAD("SKIRT_BRUSH_RIGHT err code %d", errCode);
                    }
                }
                dealErr[i] = ErrInfo_Table[i].isErr;
            }
            //判定是否有需要上传的报警更新
            if(uploadErr[i] != ErrInfo_Table[i].isErr){
                isUploadErrTrig = true;
            }
        }

        if(error.isNeedAttention){
            //部分报警需解除后才能归位
            error.isAlloweBackHome = (ERR_CANT_BACK_HOME) ? false : true;

            if(errNeedFlag.isDevIdleSta){                                   //待机状态清除所有报警
                for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
                {
                    if(E_NOTICE_AUTO_CLRAR != ErrInfo_Table[i].dealMode){   //仅通知自动清除的报警类型不清除
                        ErrInfo_Table[i].isErr = false;
                        ErrInfo_Table[i].statusCnt = 0;
                        dealErr[i] = false;
                    }
                }
                LOG_UPLOAD("Clear all err");
            }
        }
        else{
            error.isAlloweBackHome = true;
        }
        
        //有错误更新时重新上传报警代码
        if(isUploadErrTrig && errNeedFlag.isMqttConnected){
            aos_queue_send(&error.status, ErrInfo_Table, sizeof(ErrInfo_Table));       //采用消息队列的方式存储报警信息依次上传
            for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
            {
                uploadErr[i] = ErrInfo_Table[i].isErr;
            }
        }
        aos_msleep(ERR_CHECK_SLEEP_TIME);
    }
}

/*                                                         =======================                                                         */
/* ========================================================        其它接口        ======================================================== */
/*                                                         =======================                                                         */

Type_ErrStateFlag_Def *err_need_flag_handle(void)
{
    return &errNeedFlag;
}

/**
 * @brief       获取是否允许归位标志
 * @return      bool                
 */
bool get_allow_back_home_flag(void)
{
    return error.isAlloweBackHome;
}

/**
 * @brief       获取急停断电状态
 * @return      bool                
 */
bool get_emc_power_off_sta(void)
{
    return error.isEmcSta;
}

/**
 * @brief       获取报警的超时时间
 * @param[in]	errCode             
 * @return      uint32_t            
 */
uint32_t get_error_overtime(uint16_t errCode)
{
    for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
    {
        if(ErrInfo_Table[i].code == errCode){
            return ErrInfo_Table[i].errOverTime;
        }
    }
    return 0;
}

/**
 * @brief       设置报警状态
 * @param[in]	errCode             
 * @param[in]	status              
 */
void set_error_state(uint16_t errCode, bool status)
{
    for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
    {
        if(ErrInfo_Table[i].code == errCode){
            ErrInfo_Table[i].isErr = status;
            if(false == status) ErrInfo_Table[i].statusCnt = 0;
            LOG_UPLOAD("Set error code %d = %d", ErrInfo_Table[i].code, status);
            break;
        }
    }
}

/**
 * @brief       获取报警对应报警状态
 * @param[in]	errCode             
 * @return      bool                
 */
bool get_error_state(uint16_t errCode)
{
    for (uint8_t i = 0; i < ERR_TABLE_NUMBERS; i++)
    {
        if(ErrInfo_Table[i].code == errCode){
            return ErrInfo_Table[i].isErr;
        }
    }
    return false;
}

/**
 * @brief       获取是否有需要关注的报警状态
 * @return      bool                
 */
bool get_attention_err_status(void)
{
    return error.isNeedAttention;
}

