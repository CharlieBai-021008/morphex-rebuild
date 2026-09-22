#ifndef __HEXAPOD_CONTROL_H
#define __HEXAPOD_CONTROL_H

#include "main.h"
#include "kinematics.h"
#include "servo.h"
#include <math.h>

// 机器人结构参数
#define LEG_COUNT 6
#define L1 61.86f     // 髋关节长度(mm)
#define L2 87.02f     // 大腿长度(mm)
#define L3 117.1f     // 小腿长度(mm)
#define BASE_RADIUS 134.45f  // 底盘中心到髋关节距离(mm)

// 腿部初始位置偏移（相对于髋关节）
#define INIT_X_OFFSET 0.0f     // x方向初始偏移(mm)
#define INIT_Y_OFFSET 130.0f     // y方向初始偏移(mm)
#define INIT_Z_OFFSET -130      // z方向初始偏移(mm)

// 步态参数
#define STEP_HEIGHT 40.0f    // 抬腿高度(mm)
#define STEP_LENGTH 50.0f    // 步长(mm)
/*#define SERVO_MOVE_TIME 200  // 舵机运动时间(ms)*/
extern uint16_t SERVO_MOVE_TIME;   // 声明为外部变量，可在运行时修改

// 旋转参数
#define ROTATION_ANGLE 15.0f*M_PI/ 180.0f  // 默认旋转角度(弧度)

// 舵机控制参数
#define SERVO_MIN_PULSE 500    // 最小脉冲宽度(us)
#define SERVO_MAX_PULSE 2500   // 最大脉冲宽度(us)
#define SERVO_MIN_ANGLE 0    // 最小角度(度)
#define SERVO_MAX_ANGLE 270     // 最大角度(度)

// 腿部编号
typedef enum {
    LEFT_FRONT = 0,
    LEFT_MIDDLE = 1,
    LEFT_BACK = 2,
    RIGHT_FRONT = 3,
    RIGHT_MIDDLE = 4,
    RIGHT_BACK = 5
} LegID;

// 舵机ID结构
typedef struct {
    uint8_t hip;     // 髋关节舵机ID
    uint8_t thigh;   // 大腿舵机ID
    uint8_t knee;    // 膝关节舵机ID
} LegServoIDs;

// 舵机组控制结构
typedef struct {
    uint8_t ids[LEG_COUNT * 3];        // 存储所有舵机ID
    uint16_t positions[LEG_COUNT * 3];  // 存储所有舵机位置
    uint16_t times[LEG_COUNT * 3];      // 存储所有舵机运动时间
    int servo_count;                    // 实际使用的舵机数量
} ServoGroupControl;

// 3D位置结构体
typedef struct {
    double x;
    double y;
    double z;
} Position3D;

// 关节角度结构体
typedef struct {
    double theta1;  // 髋关节角度
    double theta2;  // 大腿角度
    double theta3;  // 膝关节角度
} JointAngles;

// 各腿相对于底盘中心的角度（单位：弧度）
static const float LEG_ANGLES[LEG_COUNT] = {
    30.0f * M_PI / 180.0f,   // 左前腿 30度
    90.0f * M_PI / 180.0f,   // 左中腿 90度
    150.0f * M_PI / 180.0f,  // 左后腿 150度
    -30.0f * M_PI / 180.0f,  // 右前腿 -30度
    -90.0f * M_PI / 180.0f,  // 右中腿 -90度
    -150.0f * M_PI / 180.0f  // 右后腿 -150度
};

// 舵机安装角度偏移（单位：度）
static const float SERVO_MOUNT_OFFSETS[LEG_COUNT][3] = {
    {28.3f, 90.0f, 77.9f},   // 左前腿  (髋关节、大腿、膝关节)
    {28.3f, 90.0f, 77.9f},   // 左中腿
    {28.3f, 90.0f, 77.9f},   // 左后腿
    {28.3f, 90.0f, 77.9f},   // 右前腿
    {28.3f, 90.0f, 77.9f},   // 右中腿
    {28.3f, 90.0f, 77.9f}    // 右后腿
};

// 全局变量
extern Position3D legOffsets[LEG_COUNT];    // 腿部位置偏移
extern Position3D legPositions[LEG_COUNT];  // 当前腿部位置
extern JointAngles legAngles[LEG_COUNT];    // 当前关节角度

// 腿部舵机ID映射表
extern const LegServoIDs legServoIDs[LEG_COUNT];

// 函数声明
void InitHexapod(void);
void CalculateLegControl(LegID leg, double x, double y, double z, ServoGroupControl* group_control, int start_index);
void CalculateGroupControl(const LegID* legs, int leg_count, double x, double y, double z, ServoGroupControl* group_control);
void TripleStepForward(void);
void TripleStepBackward(void);
void TestLeftFrontLeg(void);
uint16_t AngleToPWM(double angle);
void CalculateLocalPosition(LegID leg, double* local_x, double* local_y, double* local_z);
void TripleStepLeft(void);
void TripleStepRight(void);
void TripleStepRotateCounterClockwise(void);
void TripleStepRotateClockwise(void);
void SetSpecificJointAnglesStepwise(void);
void SetLegsToAngles(const LegID* legs, int leg_count, double theta1, double theta2, double theta3, uint16_t move_time);
void SetLegsToAngles2(const LegID* legs, int leg_count, double theta1, double theta2, double theta3, uint16_t move_time);

#endif /* __HEXAPOD_CONTROL_JIAN_H */
