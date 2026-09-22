/**
 * @file sphere_control.c
 * @brief 球形机器人控制模式实现
 */

#include "sphere_control.h"
#include "hexapod_control.h"
#include <stdio.h>

/* 全局变量 */
SphereState sphereState = SPHERE_STATE_HEXAPOD; // 默认为六足状态
SphereMoveDirection currentMoveDirection = SPHERE_MOVE_STOP;
uint16_t currentSpeed = 0;
bool isRolling = false;

/* 切片舵机ID映射表（需要根据实际硬件配置修改） */
// 左半球舵机ID (每个切片2个舵机, 共6个切片)
const ShellServoIDs leftShellServoIDs[SLICES_PER_HALF] = {
    {13, 12},    // 左半球切片1 (上、下舵机ID)
    {23, 22},    // 左半球切片2
    {33, 32},    // 左半球切片3
    {43, 42},    // 左半球切片4
    {53, 52},   // 左半球切片5
    {63, 62}   // 左半球切片6
};

// 右半球舵机ID (每个切片2个舵机, 共6个切片)
const ShellServoIDs rightShellServoIDs[SLICES_PER_HALF] = {
    {15, 14},  // 右半球切片1 (上、下舵机ID)
    {25, 24},  // 右半球切片2
    {35, 34},  // 右半球切片3
    {45, 44},  // 右半球切片4
    {55, 54},  // 右半球切片5
    {65, 64}   // 右半球切片6
};

/* 舵机角度与PWM脉宽转换函数 */
static uint16_t ShellAngleToPWM(double angle) {
    // 将角度映射到PWM范围
    uint16_t pwm = (uint16_t)((angle - SERVO_MIN_ANGLE) *
                              (SERVO_MAX_PULSE - SERVO_MIN_PULSE) /
                              (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) + 1500);
    return pwm;
}

/**
 * @brief 初始化球形控制模块
 */
void Sphere_Init(void) {
    // 初始化舵机控制
    Servo2_Init();
    printf("球形控制初始化完成\r\n");
}

/**
 * @brief 获取当前球形状态
 * @return 当前状态
 */
SphereState Sphere_GetState(void) {
    return sphereState;
}

/**
 * @brief 主循环中处理球形控制
 */
void Sphere_Process(void) {
    // 处理接收到的数据
    Servo2_ProcessRxData();

    // 如果在滚动状态，维持滚动运动
    if (isRolling && sphereState == SPHERE_STATE_SPHERE) {
        static uint32_t lastUpdateTime = 0;
        uint32_t currentTime = HAL_GetTick();

        // 每100ms更新一次滚动控制，以维持持续运动
        if (currentTime - lastUpdateTime >= 100) {
            lastUpdateTime = currentTime;

            // 根据当前方向调整舵机位置实现滚动运动
            switch (currentMoveDirection) {
                case SPHERE_MOVE_FORWARD:
                    // 前进滚动控制
                    _Sphere_RollForward(currentSpeed);
                    break;

                case SPHERE_MOVE_BACKWARD:
                    // 后退滚动控制
                    _Sphere_RollBackward(currentSpeed);
                    break;

                case SPHERE_MOVE_LEFT:
                    // 左转滚动控制
                    _Sphere_RollLeft(currentSpeed);
                    break;

                case SPHERE_MOVE_RIGHT:
                    // 右转滚动控制
                    _Sphere_RollRight(currentSpeed);
                    break;

                case SPHERE_MOVE_STRETCH_RIGHT:
                    // 向右伸展控制
                    _Sphere_StretchRight(currentSpeed);
                    break;

                case SPHERE_MOVE_STRETCH_LEFT:
                    // 向左伸展控制
                    _Sphere_StretchLeft(currentSpeed);
                    break;

                case SPHERE_MOVE_STOP:
                default:
                    // 停止滚动
                    isRolling = false;
                    break;
            }
        }
    }
}

/**
 * @brief 设置所有外壳舵机到指定角度，左半球使用servo，右半球使用servo2
 * @param angle 目标角度（0-270度）
 */
void Sphere_SetShellServosAngle(uint8_t angle) {
    uint8_t leftIds[SHELL_SERVO_PER_HALF];       // 左半球舵机ID数组
    uint16_t leftPositions[SHELL_SERVO_PER_HALF];
    uint16_t leftTimes[SHELL_SERVO_PER_HALF];

    uint8_t rightIds[SHELL_SERVO_PER_HALF];      // 右半球舵机ID数组
    uint16_t rightPositions[SHELL_SERVO_PER_HALF];
    uint16_t rightTimes[SHELL_SERVO_PER_HALF];

    int leftServoIndex = 0;
    int rightServoIndex = 0;

    // 限制角度范围
    if (angle > SHELL_SERVO_MAX_ANGLE) {
        angle = SHELL_SERVO_MAX_ANGLE;
    } else if (angle < SHELL_SERVO_MIN_ANGLE) {
        angle = SHELL_SERVO_MIN_ANGLE;
    }

    // 计算PWM值
    uint16_t pwm = ShellAngleToPWM(angle);

    // 准备控制左半球所有舵机 (使用Servo控制)
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        leftIds[leftServoIndex] = leftShellServoIDs[i].upperServo;
        leftIds[leftServoIndex+1] = leftShellServoIDs[i].lowerServo;
        leftPositions[leftServoIndex] = pwm;
        leftPositions[leftServoIndex+1] = pwm;
        leftTimes[leftServoIndex] = ROTATION_TIME_MS;
        leftTimes[leftServoIndex+1] = ROTATION_TIME_MS;
        leftServoIndex += 2;
    }

    // 准备控制右半球所有舵机 (使用Servo2控制)
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        rightIds[rightServoIndex] = rightShellServoIDs[i].upperServo;
        rightIds[rightServoIndex+1] = rightShellServoIDs[i].lowerServo;
        rightPositions[rightServoIndex] = pwm;
        rightPositions[rightServoIndex+1] = pwm;
        rightTimes[rightServoIndex] = ROTATION_TIME_MS;
        rightTimes[rightServoIndex+1] = ROTATION_TIME_MS;
        rightServoIndex += 2;
    }

    // 发送命令控制所有舵机
    // 左半球使用Servo控制，可能需要分批次
    Servo_SetMultiPosition(leftIds, leftPositions, leftTimes, leftServoIndex);

    // 短暂延时，确保命令发送不冲突
    HAL_Delay(20);

    // 右半球使用Servo2控制
    Servo2_SetMultiPosition(rightIds, rightPositions, rightTimes, rightServoIndex);
}

/**
 * @brief 控制单个切片的舵机角度，左半球使用servo，右半球使用servo2
 * @param half 半球选择（左/右）
 * @param sliceIndex 切片索引（0-5）
 * @param angle 目标角度（0-270度）
 */
void Sphere_ControlSingleSlice(SphereHalf half, uint8_t sliceIndex, uint8_t angle) {
    if (sliceIndex >= SLICES_PER_HALF) {
        return;
    }

    // 限制角度范围
    if (angle > SHELL_SERVO_MAX_ANGLE) {
        angle = SHELL_SERVO_MAX_ANGLE;
    } else if (angle < SHELL_SERVO_MIN_ANGLE) {
        angle = SHELL_SERVO_MIN_ANGLE;
    }

    // 计算PWM值
    uint16_t pwm = ShellAngleToPWM(angle);

    // 根据半球选择获取对应的舵机ID和控制方法
    if (half == LEFT_HALF) {
        // 左半球使用servo控制
        uint8_t ids[2];
        uint16_t positions[2];
        uint16_t times[2];

        // 设置ID
        ids[0] = leftShellServoIDs[sliceIndex].upperServo;
        ids[1] = leftShellServoIDs[sliceIndex].lowerServo;

        // 设置位置
        positions[0] = pwm;
        positions[1] = pwm;

        // 设置时间
        times[0] = ROTATION_TIME_MS;
        times[1] = ROTATION_TIME_MS;

        // 控制上下两个舵机
        Servo_SetMultiPosition(ids, positions, times, 2);
    } else {
        // 右半球使用servo2控制
        uint8_t ids[2];
        uint16_t positions[2];
        uint16_t times[2];

        // 设置ID
        ids[0] = rightShellServoIDs[sliceIndex].upperServo;
        ids[1] = rightShellServoIDs[sliceIndex].lowerServo;

        // 设置位置
        positions[0] = pwm;
        positions[1] = pwm;

        // 设置时间
        times[0] = ROTATION_TIME_MS;
        times[1] = ROTATION_TIME_MS;

        // 控制上下两个舵机
        Servo2_SetMultiPosition(ids, positions, times, 2);
    }
}

/**
 * @brief 平滑调整外壳切片角度，左半球使用servo，右半球使用servo2
 * @param startAngle 起始角度
 * @param endAngle 结束角度
 * @param timeMs 变换时间（毫秒）
 */
void Sphere_AdjustShellSlices(uint8_t startAngle, uint8_t endAngle, uint16_t timeMs) {
    // 计算步数，大约每50ms一步
    uint16_t steps = timeMs / 50;
    if (steps < 1) steps = 1;

    // 计算每步的角度变化
    float angleStep = (float)(endAngle - startAngle) / steps;

    // 逐步调整角度
    for (uint16_t i = 0; i < steps; i++) {
        uint8_t currentAngle = (uint8_t)(startAngle + angleStep * i);

        // 使用修改后的函数，它会分别使用servo和servo2控制左右半球
        Sphere_SetShellServosAngle(currentAngle);
        HAL_Delay(50);
    }

    // 确保最终设置到目标角度
    Sphere_SetShellServosAngle(endAngle);
}

/**
 * @brief 变形为球形状态
 * @return 是否成功变形
 */
bool Sphere_TransformToSphere(void) {
    // 检查当前状态
    if (sphereState == SPHERE_STATE_SPHERE) {
        return true; // 已经是球形状态
    }

    // 修复：允许从任何状态变形为球形（除了正在变形中）
    if (sphereState == SPHERE_STATE_TRANSFORMING) {
        return false; // 正在变形中，不能开始新的变形
    }

    // 设置状态为变形中
    sphereState = SPHERE_STATE_TRANSFORMING;
    printf("开始变形为球形...\r\n");

    // 首先停止当前的滚动运动
    isRolling = false;
    currentMoveDirection = SPHERE_MOVE_STOP;

    // 步骤1: 首先将六足机器人的腿部收缩，准备变形
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};  // Triangle group 1
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK}; // Triangle group 2
    const LegID allLegs[] = {LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK,
                                     RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK};

    SetLegsToAngles(group1,  3, 90.0, -80.0, -80.0, 1000);
    SetLegsToAngles(group1,  3, 90.0, 40.0 , -70.0, 1000);
    SetLegsToAngles(group2,  3, 90.0, 0    , -80.0, 1000);
    SetLegsToAngles(group2,  3, 90.0, 40.0 , -70.0, 1000);
    SetLegsToAngles(allLegs, 6, 90.0, 50   , 0    , 1000);
    Servo_SetAngle(90, 1000);
    SetLegsToAngles(allLegs, 6, 81.89, 116.87, 50.91, 1000);
    Servo_SetAngle(30, 1000);
    HAL_Delay(SERVO_MOVE_TIME + 50); // 额外等待50ms确保命令完成

    // 设置状态为球形
    sphereState = SPHERE_STATE_SPHERE;
    printf("变形为球形完成！\r\n");

    return true;
}

/**
 * @brief 变形为六足状态
 * @return 是否成功变形
 */
bool Sphere_TransformToHexapod(void) {
    // 检查当前状态
    if (sphereState == SPHERE_STATE_HEXAPOD) {
        return true; // 已经是六足状态
    }

    // 修复：允许从任何状态变形为六足（除了正在变形中）
    if (sphereState == SPHERE_STATE_TRANSFORMING) {
        return false; // 正在变形中，不能开始新的变形
    }

    // 设置状态为变形中
    sphereState = SPHERE_STATE_TRANSFORMING;
    printf("开始变形为六足...\r\n");

    // 步骤1: 停止滚动
    isRolling = false;
    currentMoveDirection = SPHERE_MOVE_STOP;

    HAL_Delay(200);

    uint8_t allIds[SHELL_SERVO_PER_HALF];
    uint16_t allPositions[SHELL_SERVO_PER_HALF];
    uint16_t allTimes[SHELL_SERVO_PER_HALF];

    // 设置所有舵机的目标位置
    int index = 0;
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        // 上部舵机
        allIds[index] = rightShellServoIDs[i].upperServo;
        allPositions[index] = 1500 + (uint16_t)((SHELL_UPPER_SERVO_INIT_ANGLE * 2000.0f) / 270.0f);
        allTimes[index] = SERVO_MOVE_TIME;
        index++;

        // 下部舵机
        allIds[index] = rightShellServoIDs[i].lowerServo;
        allPositions[index] = 1500 + (uint16_t)((SHELL_LOWER_SERVO_INIT_ANGLE * 2000.0f) / 270.0f);
        allTimes[index] = SERVO_MOVE_TIME;
        index++;
    }

    // 同时发送命令控制所有右半球舵机
    Servo2_SetMultiPosition(allIds, allPositions, allTimes, SHELL_SERVO_PER_HALF);


    // 步骤2: 设置六足机器人到初始站立姿态
    const LegID allLegs[] = {
        LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK,
        RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK    };
	const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};  // Triangle group 1
	const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK}; // Triangle group 2

    Servo_SetAngle(90, 1000);
    SetLegsToAngles(allLegs, 6, 61.7, 50   , -70.0, 1000);
    Servo_SetAngle(120, 1000);
    SetLegsToAngles(group2,  3, 61.7, 40.0 , -70.0, 1000);
    SetLegsToAngles(group2,  3, 61.7, 0    , -90.0, 1000);
    SetLegsToAngles(group1,  3, 61.7, 40.0 , -70.0, 1000);
    SetLegsToAngles(group1,  3, 61.7, 0    , -90.0, 1000);
    Servo_SetAngle(169.2, 1000);
    // 设置为站立姿态
    SetLegsToAngles(allLegs, 6, 61.7, 0, 0, 1000);
    HAL_Delay(1500); // 等待动作完成

    // 设置状态为六足
    sphereState = SPHERE_STATE_HEXAPOD;
    printf("变形为六足完成！\r\n");

    return true;
}

bool Sphere_TransformToDisplay(void) {
    // 检查当前状态
    if (sphereState == SPHERE_STATE_DISPLAY) {
        return true; // 已经是展示状态
    }

    // 修复：允许从任何状态变形为展示（除了正在变形中）
    if (sphereState == SPHERE_STATE_TRANSFORMING) {
        return false; // 正在变形中，不能开始新的变形
    }

    printf("开始变形为展示状态...\r\n");

    // 设置状态为变形中
    sphereState = SPHERE_STATE_TRANSFORMING;

    // 首先停止当前的滚动运动
    isRolling = false;
    currentMoveDirection = SPHERE_MOVE_STOP;

    // 保存原始舵机运动时间
    uint16_t original_move_time = SERVO_MOVE_TIME;

    // 定义腿部分组
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
    const LegID allLegs[] = {LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK,
                             RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK};


    Servo_SetAngle(169.2, 1000);
    SetLegsToAngles(allLegs, 6, 61.7, 0   , -70.0, 1000);

    HAL_Delay(SERVO_MOVE_TIME + 500);

    uint8_t allIds[SHELL_SERVO_PER_HALF];
    uint16_t allPositions[SHELL_SERVO_PER_HALF];
    uint16_t allTimes[SHELL_SERVO_PER_HALF];

    // 设置所有舵机的目标位置
    int index = 0;
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        // 上部舵机
        allIds[index] = rightShellServoIDs[i].upperServo;
        allPositions[index] = 1500 + (uint16_t)((10.0f * 2000.0f) / 270.0f);
        allTimes[index] = SERVO_MOVE_TIME;
        index++;

        // 下部舵机
        allIds[index] = rightShellServoIDs[i].lowerServo;
        allPositions[index] = 1500 + (uint16_t)((50.0f * 2000.0f) / 270.0f);
        allTimes[index] = SERVO_MOVE_TIME;
        index++;
    }

    // 同时发送命令控制所有右半球舵机
    Servo2_SetMultiPosition(allIds, allPositions, allTimes, SHELL_SERVO_PER_HALF);

    // 等待舵机运动完成
    HAL_Delay(SERVO_MOVE_TIME + 100);


    // 恢复原始舵机运动时间
    SERVO_MOVE_TIME = original_move_time;

    // 设置状态为展示
    sphereState = SPHERE_STATE_DISPLAY;
    printf("机器人现处于展示状态\r\n");

    return true;
}

/**
 * @brief 滚动控制函数
 * @param direction 滚动方向
 * @param speed 滚动速度（0-100）
 */
void Sphere_Roll(SphereMoveDirection direction, uint16_t speed) {
    // 检查当前状态
    if (sphereState != SPHERE_STATE_SPHERE) {
        printf("错误: 只能在球形状态下滚动\r\n");
        return;
    }

    // 限制速度范围
    if (speed > 100) {
        speed = 100;
    }

    // 更新当前状态
    currentMoveDirection = direction;
    currentSpeed = speed;
    isRolling = (direction != SPHERE_MOVE_STOP);

    if (direction == SPHERE_MOVE_STOP) {
        printf("停止滚动\r\n");
        Sphere_Stop();
    } else {
        // 根据方向启动相应的滚动动作
        switch (direction) {
            case SPHERE_MOVE_FORWARD:
                _Sphere_RollForward(speed);
                printf("向前滚动，速度: %d%%\r\n", speed);
                break;

            case SPHERE_MOVE_BACKWARD:
                _Sphere_RollBackward(speed);
                printf("向后滚动，速度: %d%%\r\n", speed);
                break;

            case SPHERE_MOVE_LEFT:
                _Sphere_RollLeft(speed);
                printf("向左滚动，速度: %d%%\r\n", speed);
                break;

            case SPHERE_MOVE_RIGHT:
                _Sphere_RollRight(speed);
                printf("向右滚动，速度: %d%%\r\n", speed);
                break;

            case SPHERE_MOVE_STRETCH_RIGHT:
                _Sphere_StretchRight(speed);
                printf("向右伸展，速度: %d%%\r\n", speed);
                break;

            case SPHERE_MOVE_STRETCH_LEFT:
                _Sphere_StretchLeft(speed);
                printf("向左伸展，速度: %d%%\r\n", speed);
                break;

            default:
                isRolling = false;
                printf("未知的滚动方向\r\n");
                break;
        }
    }
}

/**
 * @brief 停止滚动，左半球使用servo，右半球使用servo2
 */
void Sphere_Stop(void) {
    // 设置所有切片舵机到中心位置，以停止滚动
    uint8_t leftIds[SHELL_SERVO_PER_HALF];       // 左半球舵机ID数组
    uint16_t leftPositions[SHELL_SERVO_PER_HALF];
    uint16_t leftTimes[SHELL_SERVO_PER_HALF];

    uint8_t rightIds[SHELL_SERVO_PER_HALF];      // 右半球舵机ID数组
    uint16_t rightPositions[SHELL_SERVO_PER_HALF];
    uint16_t rightTimes[SHELL_SERVO_PER_HALF];

    int leftServoIndex = 0;
    int rightServoIndex = 0;

    uint16_t centerPWM = ShellAngleToPWM(SHELL_SERVO_MAX_ANGLE);

    // 准备控制左半球所有舵机 (使用Servo控制)
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        leftIds[leftServoIndex] = leftShellServoIDs[i].upperServo;
        leftIds[leftServoIndex+1] = leftShellServoIDs[i].lowerServo;
        leftPositions[leftServoIndex] = ShellAngleToPWM(SHELL_UPPER_SERVO_INIT_ANGLE);
        leftPositions[leftServoIndex+1] = ShellAngleToPWM(SHELL_LOWER_SERVO_INIT_ANGLE);
        leftTimes[leftServoIndex] = 500; // 快速停止
        leftTimes[leftServoIndex+1] = 500;
        leftServoIndex += 2;
    }

    // 准备控制右半球所有舵机 (使用Servo2控制)
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        rightIds[rightServoIndex] = rightShellServoIDs[i].upperServo;
        rightIds[rightServoIndex+1] = rightShellServoIDs[i].lowerServo;
        rightPositions[rightServoIndex] = ShellAngleToPWM(SHELL_UPPER_SERVO_INIT_ANGLE);
        rightPositions[rightServoIndex+1] = ShellAngleToPWM(SHELL_LOWER_SERVO_INIT_ANGLE);
        rightTimes[rightServoIndex] = 500;
        rightTimes[rightServoIndex+1] = 500;
        rightServoIndex += 2;
    }

    // 发送命令控制所有舵机
    // 左半球使用Servo控制
    Servo_SetMultiPosition(leftIds, leftPositions, leftTimes, leftServoIndex);

    // 短暂延时，确保命令发送不冲突
    HAL_Delay(20);

    // 右半球使用Servo2控制
    Servo2_SetMultiPosition(rightIds, rightPositions, rightTimes, rightServoIndex);

    // 更新状态
    currentMoveDirection = SPHERE_MOVE_STOP;
    isRolling = false;
}

/* 内部滚动控制函数 *///大修

/**
 * @brief 向前滚动实现 - 每组关于中心对称的两个切片完成伸出和缩回过程
 * @param speed 速度（0-100）
 */
static void _Sphere_RollForward(uint16_t speed) {
    // 使用定义的初始角度
    float baseUpperAngle = SHELL_UPPER_SERVO_INIT_ANGLE;
    float baseLowerAngle = SHELL_LOWER_SERVO_INIT_ANGLE;

    // 动作时间应随速度调整
    uint16_t actionTime = 300; // 固定动作时间

    // 定义三组切片 - 每组两个关于中心对称的切片
    // 第一组：切片0和5（前端与后端）
    uint8_t group1Ids[8] = {
        leftShellServoIDs[0].lowerServo,  // 左前端下部
        leftShellServoIDs[0].upperServo,  // 左前端上部
        leftShellServoIDs[5].lowerServo,  // 左后端下部
        leftShellServoIDs[5].upperServo,  // 左后端上部
        rightShellServoIDs[0].lowerServo, // 右前端下部
        rightShellServoIDs[0].upperServo, // 右前端上部
        rightShellServoIDs[5].lowerServo, // 右后端下部
        rightShellServoIDs[5].upperServo  // 右后端上部
    };

    // 第二组：切片1和4（前中部与后中部）
    uint8_t group2Ids[8] = {
        leftShellServoIDs[2].lowerServo,  // 左中前部下部
        leftShellServoIDs[2].upperServo,  // 左中前部上部
        leftShellServoIDs[3].lowerServo,  // 左中后部下部
        leftShellServoIDs[3].upperServo,  // 左中后部上部
        rightShellServoIDs[2].lowerServo, // 右中前部下部
        rightShellServoIDs[2].upperServo, // 右中前部上部
        rightShellServoIDs[3].lowerServo, // 右中后部下部
        rightShellServoIDs[3].upperServo  // 右中后部上部


    };

    // 第三组：切片2和3（中前部与中后部）
    uint8_t group3Ids[8] = {
        leftShellServoIDs[1].lowerServo,  // 左前中部下部
		leftShellServoIDs[1].upperServo,  // 左前中部上部
		leftShellServoIDs[4].lowerServo,  // 左后中部下部
		leftShellServoIDs[4].upperServo,  // 左后中部上部
		rightShellServoIDs[1].lowerServo, // 右前中部下部
		rightShellServoIDs[1].upperServo, // 右前中部上部
		rightShellServoIDs[4].lowerServo, // 右后中部下部
		rightShellServoIDs[4].upperServo  // 右后中部上部
    };

    // 伸出位置 - 前端折叠，后端展开（与后退相反）
    uint16_t extendedPositions[8] = {
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0), // 上部展开
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0), // 上部展开
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0), // 上部展开
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0)  // 上部展开
    };

    // 缩回位置 - 恢复到基础角度
    uint16_t retractedPositions[8] = {
        ShellAngleToPWM(116.87), // 下部恢复
        ShellAngleToPWM(50.91),  // 上部恢复
        ShellAngleToPWM(116.87), // 下部恢复
        ShellAngleToPWM(50.91),  // 上部恢复
        ShellAngleToPWM(116.87), // 下部恢复
        ShellAngleToPWM(50.91),  // 上部恢复
        ShellAngleToPWM(116.87), // 下部恢复
        ShellAngleToPWM(50.91)   // 上部恢复
    };

    uint16_t times[8] = {actionTime, actionTime, actionTime, actionTime, actionTime, actionTime, actionTime, actionTime};

    // 实现简单的循环滚动模式
    static uint8_t currentGroup = 0;     // 当前激活的组
    static bool isExtending = true;      // 当前是伸出还是缩回阶段

    // === 调试信息 ===
    printf("\n=== 前进运动 - 速度: %d%%, 时间: %dms ===\n", speed, actionTime);
    printf("当前组: %d, 状态: %s\n", currentGroup, isExtending ? "伸出" : "缩回");

    // 根据组别打印组信息
    const char* groupNames[3] = {
        "前端(0)与后端(5)",
        "前中部(1)与后中部(4)",
        "中前部(2)与中后部(3)"
    };
    printf("控制切片组: %s\n", groupNames[currentGroup]);

    // 打印舵机信息
    uint8_t* currentIds;
    switch (currentGroup) {
        case 0: currentIds = group1Ids; break;
        case 1: currentIds = group2Ids; break;
        case 2: currentIds = group3Ids; break;
    }

    // 控制当前组的伸出或缩回
    if (isExtending) {
        // 伸出阶段 - 当前组折叠/伸出
        switch (currentGroup) {
            case 0: // 第一组伸出 (前端和后端)
                // 前端上部舵机折叠，后端下部舵机展开
                printf("执行: 前端折叠, 后端展开\n");
                Servo_SetMultiPosition(&group1Ids[0], &extendedPositions[0], &times[0], 4);
                HAL_Delay(10);
                Servo2_SetMultiPosition(&group1Ids[4], &extendedPositions[4], &times[4], 4);
                break;

            case 1: // 第二组伸出 (前中部和后中部)
                // 前中部上部舵机折叠，后中部下部舵机展开
                printf("执行: 前中部折叠, 后中部展开\n");
                Servo_SetMultiPosition(&group2Ids[0], &extendedPositions[0], &times[0], 4);
                HAL_Delay(10);
                Servo2_SetMultiPosition(&group2Ids[4], &extendedPositions[4], &times[4], 4);
                break;

            case 2: // 第三组伸出 (中前部和中后部)
                // 中前部上部舵机折叠，中后部下部舵机展开
                printf("执行: 中前部折叠, 中后部展开\n");
                Servo_SetMultiPosition(&group3Ids[0], &extendedPositions[0], &times[0], 4);
                HAL_Delay(10);
                Servo2_SetMultiPosition(&group3Ids[4], &extendedPositions[4], &times[4], 4);
                break;
        }
    } else {
        // 缩回阶段 - 当前组恢复
        switch (currentGroup) {
            case 0: // 第一组缩回 (前端和后端)
                printf("执行: 前端和后端恢复\n");
                Servo_SetMultiPosition(&group1Ids[0], &retractedPositions[0], &times[0], 4);
                HAL_Delay(10);
                Servo2_SetMultiPosition(&group1Ids[4], &retractedPositions[4], &times[4], 4);
                break;

            case 1: // 第二组缩回 (前中部和后中部)
                printf("执行: 前中部和后中部恢复\n");
                Servo_SetMultiPosition(&group2Ids[0], &retractedPositions[0], &times[0], 4);
                HAL_Delay(10);
                Servo2_SetMultiPosition(&group2Ids[4], &retractedPositions[4], &times[4], 4);
                break;

            case 2: // 第三组缩回 (中前部和中后部)
                printf("执行: 中前部和中后部恢复\n");
                Servo_SetMultiPosition(&group3Ids[0], &retractedPositions[0], &times[0], 4);
                HAL_Delay(10);
                Servo2_SetMultiPosition(&group3Ids[4], &retractedPositions[4], &times[4], 4);
                break;
        }
    }

    printf("等待动作完成: %dms\n", actionTime + 50);
    // 等待动作完成
    HAL_Delay(actionTime + 50);

    // 在伸出和缩回之间切换
    if (!isExtending) {
        // 如果刚完成缩回，则切换到下一组
        currentGroup = (currentGroup + 1) % 3;
        printf("下一组将是: %d\n", currentGroup);
    }

    // 切换伸出/缩回状态
    isExtending = !isExtending;
    printf("下一状态将是: %s\n", isExtending ? "伸出" : "缩回");
    printf("=== 前进运动周期结束 ===\n\n");
}
/**
 * @brief 向后滚动实现 - 每组关于中心对称的两个切片完成伸出和缩回过程
 * @param speed 速度（0-100）
 */
static void _Sphere_RollBackward(uint16_t speed) {
    // 使用定义的初始角度
    float baseUpperAngle = SHELL_UPPER_SERVO_INIT_ANGLE;
    float baseLowerAngle = SHELL_LOWER_SERVO_INIT_ANGLE;

    // 动作时间应随速度调整
    uint16_t actionTime = 300; // 速度越快，动作时间越短


    // 定义三组切片 - 每组两个关于中心对称的切片
    // 第一组：切片5和0（后端与前端）- 后退时从后开始
    uint8_t group1Ids[8] = {
        leftShellServoIDs[5].lowerServo,  // 左后端下部
		leftShellServoIDs[5].upperServo,  // 左后端下部
		leftShellServoIDs[0].lowerServo,  // 左后端下部
        leftShellServoIDs[0].upperServo,  // 左前端上部
		rightShellServoIDs[5].lowerServo, // 右后端下部
        rightShellServoIDs[5].upperServo,  // 右前端上部
		rightShellServoIDs[0].lowerServo, // 右后端下部
		rightShellServoIDs[0].upperServo  // 右前端上部
    };

    // 第二组：切片4和1（后中部与前中部）
    uint8_t group2Ids[8] = {
        leftShellServoIDs[4].lowerServo,  // 左后中部下部
        leftShellServoIDs[4].upperServo,  // 左后中部下部
		leftShellServoIDs[1].lowerServo,  // 左前中部上部
		leftShellServoIDs[1].upperServo,  // 左前中部上部
        rightShellServoIDs[4].lowerServo, // 右后中部下部
		rightShellServoIDs[4].upperServo, // 右后中部下部
        rightShellServoIDs[1].lowerServo,  // 右前中部上部
        rightShellServoIDs[1].upperServo  // 右前中部上部
    };

    // 第三组：切片3和2（中后部与中前部）
    uint8_t group3Ids[8] = {
    	leftShellServoIDs[3].lowerServo,  // 左后中部下部
    	leftShellServoIDs[3].upperServo,  // 左后中部下部
    	leftShellServoIDs[2].lowerServo,  // 左前中部上部
    	leftShellServoIDs[2].upperServo,  // 左前中部上部
    	rightShellServoIDs[3].lowerServo, // 右后中部下部
    	rightShellServoIDs[3].upperServo, // 右后中部下部
    	rightShellServoIDs[2].lowerServo,  // 右前中部上部
    	rightShellServoIDs[2].upperServo  // 右前中部上部
    };

    // 伸出位置 - 折叠角度（后退时与前进相反）
    uint16_t extendedPositions[8] = {
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0), // 上部展开
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0),  // 上部展开
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0), // 上部展开
        ShellAngleToPWM(70.0), // 下部折叠
        ShellAngleToPWM(0)  // 上部展开
    };

    // 缩回位置 - 恢复到基础角度
    uint16_t retractedPositions[8] = {
        ShellAngleToPWM(116.87), // 下部折叠
        ShellAngleToPWM(50.91), // 上部展开
        ShellAngleToPWM(116.87), // 下部折叠
        ShellAngleToPWM(50.91),  // 上部展开
        ShellAngleToPWM(116.87), // 下部折叠
        ShellAngleToPWM(50.91), // 上部展开
        ShellAngleToPWM(116.87), // 下部折叠
        ShellAngleToPWM(50.91)  // 上部展开
    };

    uint16_t times[8] = {actionTime, actionTime, actionTime, actionTime,actionTime, actionTime, actionTime, actionTime};

    // 实现简单的循环滚动模式
    static uint8_t currentGroup = 0;     // 当前激活的组
    static bool isExtending = true;      // 当前是伸出还是缩回阶段

    // === 调试信息 ===
    printf("\n=== 后退运动 - 速度: %d%%, 时间: %dms ===\n", speed, actionTime);
    printf("当前组: %d, 状态: %s\n", currentGroup, isExtending ? "伸出" : "缩回");

    // 根据组别打印组信息
    const char* groupNames[3] = {
        "后端(5)与前端(0)",
        "后中部(4)与前中部(1)",
        "中后部(3)与中前部(2)"
    };
    printf("控制切片组: %s\n", groupNames[currentGroup]);

    // 打印舵机信息
    uint8_t* currentIds;
    switch (currentGroup) {
        case 0: currentIds = group1Ids; break;
        case 1: currentIds = group2Ids; break;
        case 2: currentIds = group3Ids; break;
    }


    // 控制当前组的伸出或缩回
    if (isExtending) {
        // 伸出阶段 - 当前组折叠/伸出
        switch (currentGroup) {
            case 0: // 第一组伸出 (后端和前端)
                // 后端下部舵机折叠，前端上部舵机展开
                printf("执行: 后端折叠, 前端展开\n");
                    Servo_SetMultiPosition(&group1Ids[0], &extendedPositions[0], &times[0], 4);
                    HAL_Delay(10);
                    Servo2_SetMultiPosition(&group1Ids[4], &extendedPositions[4], &times[4], 4);
                break;

            case 1: // 第二组伸出 (后中部和前中部)
                // 后中部下部舵机折叠，前中部上部舵机展开
                printf("执行: 后中部折叠, 前中部展开\n");
                    Servo_SetMultiPosition(&group2Ids[0], &extendedPositions[0], &times[0], 4);
                    HAL_Delay(10);
                    Servo2_SetMultiPosition(&group2Ids[4], &extendedPositions[4], &times[4], 4);
                break;

            case 2: // 第三组伸出 (中后部和中前部)
                // 中后部下部舵机折叠，中前部上部舵机展开
                printf("执行: 中后部折叠, 中前部展开\n");
                    Servo_SetMultiPosition(&group3Ids[0], &extendedPositions[0], &times[0], 4);
                    HAL_Delay(10);
                    Servo2_SetMultiPosition(&group3Ids[4], &extendedPositions[4], &times[4], 4);
                break;
        }
    } else {
        // 缩回阶段 - 当前组恢复
        switch (currentGroup) {
            case 0: // 第一组缩回 (后端和前端)
                printf("执行: 后端和前端恢复\n");
                    Servo_SetMultiPosition(&group1Ids[0], &retractedPositions[0], &times[0], 4);
                    HAL_Delay(10);
                    Servo2_SetMultiPosition(&group1Ids[4], &retractedPositions[4], &times[4], 4);
                break;

            case 1: // 第二组缩回 (后中部和前中部)
                printf("执行: 后中部和前中部恢复\n");
                    Servo_SetMultiPosition(&group2Ids[0], &retractedPositions[0], &times[0], 4);
                    HAL_Delay(10);
                    Servo2_SetMultiPosition(&group2Ids[4], &retractedPositions[4], &times[2], 4);
                break;

            case 2: // 第三组缩回 (中后部和中前部)
                printf("执行: 中后部和中前部恢复\n");
                    Servo_SetMultiPosition(&group3Ids[0], &retractedPositions[0], &times[0], 4);
                    HAL_Delay(10);
                    Servo2_SetMultiPosition(&group3Ids[4], &retractedPositions[4], &times[4], 4);
                break;
        }
    }

    printf("等待动作完成: %dms\n", actionTime + 50);
    // 等待动作完成
    HAL_Delay(actionTime + 50);

    // 在伸出和缩回之间切换
    if (!isExtending) {
        // 如果刚完成缩回，则切换到下一组
        currentGroup = (currentGroup + 1) % 3;
        printf("下一组将是: %d\n", currentGroup);
    }

    // 切换伸出/缩回状态
    isExtending = !isExtending;
    printf("下一状态将是: %s\n", isExtending ? "伸出" : "缩回");
    printf("=== 后退运动周期结束 ===\n\n");
}

/**
 * @brief 向左滚动实现 - 展开左侧所有切片
 * @param speed 速度（0-100）
 */
static void _Sphere_RollLeft(uint16_t speed) {
    // 设置动作时间基于速度
    uint16_t actionTime = 300; // 速度越快，动作时间越短
    printf("\n=== 向左滚动===\n");

    // 右半球ID数组和位置数组 - 全部收缩
    uint8_t rightIds[SHELL_SERVO_PER_HALF];
    uint16_t rightRetractPositions[SHELL_SERVO_PER_HALF];
    uint16_t rightTimes[SHELL_SERVO_PER_HALF];

    int rightIndex = 0;
    // 准备右半球舵机控制参数 - 全部收缩
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        // 上部舵机
        rightIds[rightIndex] = rightShellServoIDs[i].upperServo;
        rightRetractPositions[rightIndex] = ShellAngleToPWM(31.0);
        rightTimes[rightIndex] = actionTime;
        rightIndex++;

        // 下部舵机
        rightIds[rightIndex] = rightShellServoIDs[i].lowerServo;
        rightRetractPositions[rightIndex] = ShellAngleToPWM(49.39);
        rightTimes[rightIndex] = actionTime;
        rightIndex++;
    }
    // 控制右半球舵机收缩
    Servo2_SetMultiPosition(rightIds, rightRetractPositions, rightTimes, rightIndex);

    printf("等待动作完成: %dms\n", actionTime + 50);
    HAL_Delay(actionTime + 50);
    printf("=== 向左滚动结束 ===\n\n");
}

/**
 * @brief 向右滚动实现 - 展开右侧所有切片
 * @param speed 速度（0-100）
 */
static void _Sphere_RollRight(uint16_t speed) {
    // 设置动作时间基于速度
    uint16_t actionTime = 300;

    printf("\n=== 向右滚动 ===\n");

    // 左半球ID数组和位置数组 - 全部收缩
    uint8_t leftIds[SHELL_SERVO_PER_HALF];
    uint16_t leftRetractPositions[SHELL_SERVO_PER_HALF];
    uint16_t leftTimes[SHELL_SERVO_PER_HALF];
    int leftIndex = 0;
    // 准备左半球舵机控制参数 - 全部收缩
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        // 上部舵机
        leftIds[leftIndex] = leftShellServoIDs[i].upperServo;
        leftRetractPositions[leftIndex] = ShellAngleToPWM(31.0);
        leftTimes[leftIndex] = actionTime;
        leftIndex++;

        // 下部舵机
        leftIds[leftIndex] = leftShellServoIDs[i].lowerServo;
        leftRetractPositions[leftIndex] = ShellAngleToPWM(49.39);
        leftTimes[leftIndex] = actionTime;
        leftIndex++;
    }
    // 控制左半球舵机收缩
    Servo_SetMultiPosition(leftIds, leftRetractPositions, leftTimes, leftIndex);


    printf("等待动作完成: %dms\n", actionTime + 50);
    HAL_Delay(actionTime + 50);
    printf("=== 向右滚动结束 ===\n\n");
}


/**有BUG，暂时用不了
 * @brief 向右伸展实现 - 操作不同位置的切片组以实现旋转运动
 * @param speed 速度（0-100）
 */
static void _Sphere_StretchRight(uint16_t speed) {
    // 设置动作时间基于速度
    uint16_t actionTime = 500;

    printf("\n=== 向右伸展 ===\n");

    // 右半球ID数组和位置数组 - 伸展
    uint8_t rightIds[SLICES_PER_HALF];
    uint16_t rightPositions[SLICES_PER_HALF];
    uint16_t rightTimes[SLICES_PER_HALF];

    int rightIndex = 0;


    // 准备右半球后部切片的舵机控制参数 - 伸展
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        // 上部舵机
        rightIds[rightIndex] = rightShellServoIDs[i].upperServo;
        rightPositions[rightIndex] = ShellAngleToPWM(44.5);
        rightTimes[rightIndex] = actionTime;
        rightIndex++;

        // 下部舵机
        rightIds[rightIndex] = rightShellServoIDs[i].lowerServo;
        rightPositions[rightIndex] = ShellAngleToPWM(-30.5);
        rightTimes[rightIndex] = actionTime;
        rightIndex++;
    }


    // 控制右半球后部舵机收缩
    Servo2_SetMultiPosition(rightIds, rightPositions, rightTimes, rightIndex);

    printf("等待动作完成: %dms\n", actionTime + 50);
    HAL_Delay(actionTime + 50);
    printf("=== 向右伸展结束 ===\n\n");
}

/**有BUG，暂时用不了
 * @brief 向左伸展实现 - 操作不同位置的切片组以实现旋转运动
 * @param speed 速度（0-100）
 */
static void _Sphere_StretchLeft(uint16_t speed) {
    // 设置动作时间基于速度
    uint16_t actionTime = 500;

    printf("\n=== 向左伸展 ===\n");

    // 左半球ID数组和位置数组 - 伸展
    uint8_t leftIds[SLICES_PER_HALF];
    uint16_t leftPositions[SLICES_PER_HALF];
    uint16_t leftTimes[SLICES_PER_HALF];

    int leftIndex = 0;

    // 准备左半球后部切片的舵机控制参数 - 伸展
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        // 上部舵机
        leftIds[leftIndex] = leftShellServoIDs[i].upperServo;
        leftPositions[leftIndex] = ShellAngleToPWM(44.5);
        leftTimes[leftIndex] = actionTime;
        leftIndex++;

        // 下部舵机
        leftIds[leftIndex] = leftShellServoIDs[i].lowerServo;
        leftPositions[leftIndex] = ShellAngleToPWM(-30.5);
        leftTimes[leftIndex] = actionTime;
        leftIndex++;
    }


    // 控制左半球后部舵机伸展
    Servo_SetMultiPosition(leftIds, leftPositions, leftTimes, leftIndex);

    // 短暂延时，确保命令发送不冲突
    HAL_Delay(10);

    printf("等待动作完成: %dms\n", actionTime + 50);
    HAL_Delay(actionTime + 50);
    printf("=== 向左伸展结束 ===\n\n");
}
