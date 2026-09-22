/**
 * @file sphere_control.h
 * @brief 球形机器人控制模式头文件
 */

#ifndef __SPHERE_CONTROL_H
#define __SPHERE_CONTROL_H

#include "main.h"
#include "servo.h"
#include "servo2.h"
#include "hexapod_control.h"
#include <math.h>
#include <stdbool.h>

// 球形机构参数
#define SHELL_SERVO_COUNT 24         // 外壳舵机数量
#define SHELL_SERVO_PER_HALF 12      // 每半球的舵机数量
#define SLICES_COUNT 12              // 切片总数量
#define SLICES_PER_HALF 6            // 每半球的切片数量
#define SERVOS_PER_SLICE 2           // 每个切片的舵机数量
#define ROTATION_TIME_MS 2000        // 转动时间

// 半球定义
typedef enum {
    LEFT_HALF = 0,                   // 左半球
    RIGHT_HALF = 1                   // 右半球
} SphereHalf;

// 舵机角度范围
#define SHELL_SERVO_MIN_ANGLE 0      // 最小角度（完全收缩）
#define SHELL_SERVO_MAX_ANGLE 270    // 最大角度（完全展开）

// 球形态舵机初始角度定义
#define SHELL_UPPER_SERVO_INIT_ANGLE 50.91f  // 切片上部舵机初始角度
#define SHELL_LOWER_SERVO_INIT_ANGLE 116.87f // 切片下部舵机初始角度

// 球形模式状态
typedef enum {
    SPHERE_STATE_IDLE,              // 空闲状态
    SPHERE_STATE_TRANSFORMING,      // 转换状态
    SPHERE_STATE_SPHERE,            // 球形状态
    SPHERE_STATE_HEXAPOD,            // 六足状态
	SPHERE_STATE_DISPLAY            // 展示状态
} SphereState;

// 运动方向
typedef enum {
    SPHERE_MOVE_STOP,               // 停止
    SPHERE_MOVE_FORWARD,            // 前进
    SPHERE_MOVE_BACKWARD,           // 后退
    SPHERE_MOVE_LEFT,               // 左转
    SPHERE_MOVE_RIGHT,              // 右转
    SPHERE_MOVE_STRETCH_RIGHT,      // 向右伸展
    SPHERE_MOVE_STRETCH_LEFT        // 向左伸展
} SphereMoveDirection;

// 外壳舵机ID结构
typedef struct {
    uint8_t upperServo;              // 上部舵机ID
    uint8_t lowerServo;              // 下部舵机ID
} ShellServoIDs;

// 函数声明
void Sphere_Init(void);
void Sphere_Process(void);
SphereState Sphere_GetState(void);
bool Sphere_TransformToSphere(void);
bool Sphere_TransformToHexapod(void);
bool Sphere_TransformToDisplay(void);
void Sphere_Roll(SphereMoveDirection direction, uint16_t speed);
void Sphere_Stop(void);
void Sphere_SetShellServosAngle(uint8_t angle);
void Sphere_ControlSingleSlice(SphereHalf half, uint8_t sliceIndex, uint8_t angle);
void Sphere_AdjustShellSlices(uint8_t startAngle, uint8_t endAngle, uint16_t timeMs);

// 内部函数声明

static void _Sphere_RollForward(uint16_t speed);
static void _Sphere_RollBackward(uint16_t speed);
static void _Sphere_RollLeft(uint16_t speed);
static void _Sphere_RollRight(uint16_t speed);
static void _Sphere_StretchRight(uint16_t speed);
static void _Sphere_StretchLeft(uint16_t speed);


// 外部变量声明
extern const ShellServoIDs leftShellServoIDs[SLICES_PER_HALF];  // 左半球舵机ID
extern const ShellServoIDs rightShellServoIDs[SLICES_PER_HALF]; // 右半球舵机ID
extern SphereState sphereState;

#endif /* __SPHERE_CONTROL_H */
