/**
 * @file 	 log_level.h
 * @brief       
 * @author 	 wangweihu
 * @version  1.0
 * @date 	 2022-11-30
 * 
 * @copyright Copyright (c) 2022  YGL
 * 
 */
#ifndef __LOG_LEVEL_H
#define __LOG_LEVEL_H

#include <stdio.h>
#include "../../../../1km/bsp/cli_cmd.h"

/*打印等级*/
#define HI_LOG(levle, format, args...) \
	do{ \
		printo("[%s] %s [%d]: "format"\r\n", levle, __FN__, __LINE__, ##args);\
	}while(0)

#define UP_LOG(levle, format, args...) \
	do{ \
		print_up_log("--%d: "format"", __LINE__, ##args);\
		printo("[%s] %s [%d]: "format"\r\n", levle, __FN__, __LINE__, ##args);\
	}while(0)


#define LOG_LEVEL 5  //打印级别控制的宏定义

#if(LOG_LEVEL == 0)
	#define LOG_UPLOAD(format, args...) UP_LOG("UPLOAD", format, args)
	#define LOG_FATAL(format, args...)
	#define LOG_ERROR(format, args...)
	#define LOG_WARN(format, args...)
	#define LOG_INFO(format, args...)
	#define LOG_DEBUG(format, args...)
#elif(LOG_LEVEL == 1)
	#define LOG_UPLOAD(format, args...) UP_LOG("UPLOAD", format, args)
	#define LOG_FATAL(format, args...) HI_LOG("FATAL", format, args)
	#define LOG_ERROR(format, args...)
	#define LOG_WARN(format, args...)
	#define LOG_INFO(format, args...)
	#define LOG_DEBUG(format, args...)
#elif(LOG_LEVEL == 2)
	#define LOG_UPLOAD(format, args...) UP_LOG("UPLOAD", format, args)
	#define LOG_FATAL(format, args...) HI_LOG("FATAL", format, args)
	#define LOG_ERROR(format, args...) HI_LOG("ERROR", format, args)
	#define LOG_WARN(format, args...)
	#define LOG_INFO(format, args...)
	#define LOG_DEBUG(format, args...)
#elif(LOG_LEVEL == 3)
	#define LOG_UPLOAD(format, args...) UP_LOG("UPLOAD", format, args)
	#define LOG_FATAL(format, args...) HI_LOG("FATAL", format, args)
	#define LOG_ERROR(format, args...) HI_LOG("ERROR", format, args)
	#define LOG_WARN(format, args...) HI_LOG("WARN", format, args)
	#define LOG_INFO(format, args...)
	#define LOG_DEBUG(format, args...)
#elif(LOG_LEVEL == 4)
	#define LOG_UPLOAD(format, args...) UP_LOG("UPLOAD", format, args)
	#define LOG_FATAL(format, args...) HI_LOG("FATAL", format, args)
	#define LOG_ERROR(format, args...) HI_LOG("ERROR", format, args)
	#define LOG_WARN(format, args...) HI_LOG("WARN", format, args)
	#define LOG_INFO(format, args...) HI_LOG("INFO", format, args)
	#define LOG_DEBUG(format, args...)
#elif(LOG_LEVEL == 5)
	#define LOG_UPLOAD(format, args...) UP_LOG("UPLOAD", format, args)
	#define LOG_FATAL(format, args...) HI_LOG("FATAL", format, args)
	#define LOG_ERROR(format, args...) HI_LOG("ERROR", format, args)
	#define LOG_WARN(format, args...) HI_LOG("WARN", format, args)
	#define LOG_INFO(format, args...) HI_LOG("INFO", format, args)
	#define LOG_DEBUG(format, args...) HI_LOG("DEBUG", format, args)
#endif

#endif
