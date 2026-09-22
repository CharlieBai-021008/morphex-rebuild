/**
 * @file robot_initialization.c
 * @brief 全面初始化六足/球形变形机器人的所有31个舵机
 */

#include "hexapod_control.h"
#include "sphere_control.h"
#include "servo.h"
#include "servo2.h"

/**
 * @brief 全面初始化机器人所有31个舵机到稳定站立姿态
 *
 * 该函数初始化所有31个舵机，包括:
 * - 六足部分的18个舵机（6条腿 × 3个关节）
 * - 球形模式的12个舵机（覆盖外壳的切片舵机）
 * - 中央控制舵机（1个）
 *
 * 初始化采用分步骤、分组进行，确保机器人稳定性
 */
void InitializeAllServos(void) {
    printf("\n===== 开始初始化所有31个舵机 =====\n");

    // 定义所有腿组
    const LegID allLegs[] = {
        LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK,
        RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK
    };

    // 定义三角形腿组（用于分批次控制）
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
//
    // 步骤1: 初始化系统参数
    InitHexapod();
    Sphere_Init();
    printf("系统参数初始化完成\n");
//
//    // 设置较长的舵机运动时间，确保平稳移动
      uint16_t original_move_time = SERVO_MOVE_TIME;
      SERVO_MOVE_TIME = 1000;
//

    // 步骤2: 底盘展开
    printf("\n----- 第一阶段: 底盘展开 -----\n");
    Servo_SetAngle(169.2, SERVO_MOVE_TIME);
    HAL_Delay(SERVO_MOVE_TIME+100); // 等待完成

    // 步骤3: 初始化右半球外壳舵机 - 所有舵机同时控制
    printf("\n----- 第二阶段: 初始化球形外壳舵机组 -----\n");
    // 准备所有右半球舵机的控制参数
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

    // 等待舵机运动完成
    HAL_Delay(SERVO_MOVE_TIME + 100);

    // 步骤4: 初始化六足舵机组
    printf("\n----- 第三阶段: 初始化六足舵机组 -----\n");
    SetLegsToAngles(allLegs, 6, 61.7, 0, 0, SERVO_MOVE_TIME);
    HAL_Delay(SERVO_MOVE_TIME+100); // 等待完成


    // 步骤5: 设置舵机状态标志
    sphereState = SPHERE_STATE_HEXAPOD; // 设置为六足模式

    // 恢复默认舵机运动时间
    SERVO_MOVE_TIME = original_move_time;

    printf("\n===== 所有31个舵机初始化完成! =====\n");
    printf("机器人当前处于六足模式，所有舵机已设置到初始位置\n");
    printf("初始化完成，机器人准备就绪!\n");
}

/**
 * @brief 验证所有舵机功能
 *
 * 该函数按顺序测试所有31个舵机，每个舵机做小幅度运动
 * 用于检查所有舵机是否正常工作
 */
void TestAllServos(void) {
    printf("\n===== 开始测试所有31个舵机 =====\n");

    uint16_t original_move_time = SERVO_MOVE_TIME;
    SERVO_MOVE_TIME = 800;

    // 1. 测试中央控制舵机
    printf("测试中央控制舵机 (ID: 30)...\n");
    Servo_SetAngle(30, 80);
    HAL_Delay(1000);
    Servo_SetAngle(30, 100);
    HAL_Delay(1000);
    Servo_SetAngle(30, 90);
    HAL_Delay(1000);

    // 2. 测试每条腿的三个舵机
    for (int leg = 0; leg < LEG_COUNT; leg++) {
        printf("测试腿 %d 的舵机 (IDs: %d, %d, %d)...\n",
               leg, legServoIDs[leg].hip, legServoIDs[leg].thigh, legServoIDs[leg].knee);

        // 测试髋关节
        Servo_SetAngle(legServoIDs[leg].hip, 85);
        HAL_Delay(800);
        Servo_SetAngle(legServoIDs[leg].hip, 95);
        HAL_Delay(800);
        Servo_SetAngle(legServoIDs[leg].hip, 90);
        HAL_Delay(800);

        // 测试大腿关节
        Servo_SetAngle(legServoIDs[leg].thigh, 40);
        HAL_Delay(800);
        Servo_SetAngle(legServoIDs[leg].thigh, 50);
        HAL_Delay(800);
        Servo_SetAngle(legServoIDs[leg].thigh, 45);
        HAL_Delay(800);

        // 测试膝关节
        Servo_SetAngle(legServoIDs[leg].knee, 130);
        HAL_Delay(800);
        Servo_SetAngle(legServoIDs[leg].knee, 140);
        HAL_Delay(800);
        Servo_SetAngle(legServoIDs[leg].knee, 135);
        HAL_Delay(800);
    }

    // 3. 测试左半球外壳舵机
    printf("\n测试左半球外壳舵机...\n");
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        printf("测试左半球切片 %d 的舵机 (IDs: %d, %d)...\n",
               i, leftShellServoIDs[i].upperServo, leftShellServoIDs[i].lowerServo);

        // 测试上部舵机 (小幅度动作)
        uint8_t ids[1] = {leftShellServoIDs[i].upperServo};
        uint16_t positions[1] = {500 + (uint16_t)(((SHELL_UPPER_SERVO_INIT_ANGLE + 10) * 2000.0f) / 270.0f)};
        uint16_t times[1] = {800};
        Servo_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);

        positions[0] = 500 + (uint16_t)((SHELL_UPPER_SERVO_INIT_ANGLE * 2000.0f) / 270.0f);
        Servo_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);

        // 测试下部舵机 (小幅度动作)
        ids[0] = leftShellServoIDs[i].lowerServo;
        positions[0] = 500 + (uint16_t)(((SHELL_LOWER_SERVO_INIT_ANGLE + 10) * 2000.0f) / 270.0f);
        Servo_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);

        positions[0] = 500 + (uint16_t)((SHELL_LOWER_SERVO_INIT_ANGLE * 2000.0f) / 270.0f);
        Servo_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);
    }

    // 4. 测试右半球外壳舵机
    printf("\n测试右半球外壳舵机...\n");
    for (int i = 0; i < SLICES_PER_HALF; i++) {
        printf("测试右半球切片 %d 的舵机 (IDs: %d, %d)...\n",
               i, rightShellServoIDs[i].upperServo, rightShellServoIDs[i].lowerServo);

        // 测试上部舵机 (小幅度动作)
        uint8_t ids[1] = {rightShellServoIDs[i].upperServo};
        uint16_t positions[1] = {500 + (uint16_t)(((SHELL_UPPER_SERVO_INIT_ANGLE + 10) * 2000.0f) / 270.0f)};
        uint16_t times[1] = {800};
        Servo2_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);

        positions[0] = 500 + (uint16_t)((SHELL_UPPER_SERVO_INIT_ANGLE * 2000.0f) / 270.0f);
        Servo2_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);

        // 测试下部舵机 (小幅度动作)
        ids[0] = rightShellServoIDs[i].lowerServo;
        positions[0] = 500 + (uint16_t)(((SHELL_LOWER_SERVO_INIT_ANGLE + 10) * 2000.0f) / 270.0f);
        Servo2_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);

        positions[0] = 500 + (uint16_t)((SHELL_LOWER_SERVO_INIT_ANGLE * 2000.0f) / 270.0f);
        Servo2_SetMultiPosition(ids, positions, times, 1);
        HAL_Delay(800);
    }

    // 恢复默认舵机运动时间
    SERVO_MOVE_TIME = original_move_time;

    printf("\n===== 所有31个舵机测试完成! =====\n");
}
