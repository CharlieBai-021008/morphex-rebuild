/**
 * @file hexapod_remote_control.h
 * @brief 六足机器人遥控控制头文件
 */

#ifndef __HEXAPOD_REMOTE_CONTROL_H
#define __HEXAPOD_REMOTE_CONTROL_H

#include "hexapod_control.h"
#include "main.h"
#include "sbus.h"
#include <stdbool.h>

/* 定义 */
#define SBUS_BUFFER_SIZE 25         // SBUS帧大小
#define CONTROL_UPDATE_PERIOD 20     // 控制更新周期(ms)

/* 运动控制阈值 */
#define STICK_MIDDLE 1000           // 摇杆中间值
#define STICK_DEADZONE 100          // 摇杆死区


/* 函数声明 */
/**
 * @brief 遥控控制初始化
 */
void RemoteControl_Init(void);

/**
 * @brief 处理遥控器控制信号
 */
void RemoteControl_Process(void);

/**
 * @brief 主循环中调用此函数处理遥控控制
 */
void RemoteControl_MainLoop(void);

#endif /* __HEXAPOD_REMOTE_CONTROL_H */
