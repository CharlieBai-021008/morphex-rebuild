#include "hexapod_control.h"
#include "servo2.h"
#include <stdbool.h>

// 全局变量定义
Position3D legOffsets[LEG_COUNT];    // 腿部位置偏移
Position3D legPositions[LEG_COUNT];  // 当前腿部位置
JointAngles legAngles[LEG_COUNT];    // 当前关节角度
uint16_t SERVO_MOVE_TIME = 200;      // 舵机运动时间默认值为200ms

// 腿部舵机ID映射表（示例值，需要根据实际硬件配置修改）
const LegServoIDs legServoIDs[LEG_COUNT] = {
    {11, 12, 13},    // 左前腿
    {31, 32, 33},    // 左中腿
    {51, 52, 53},    // 左后腿
    {21, 22, 23}, // 右前腿
    {41, 42, 43}, // 右中腿
    {61, 62, 63}  // 右后腿
};


// 角度转PWM值的函数
uint16_t AngleToPWM(double angle) {

    // 使用线性映射：Y = (X - X1)*(Y2 - Y1)/(X2 - X1) + Y1
    uint16_t pwm = (uint16_t)((angle - SERVO_MIN_ANGLE) * 
                              (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 
                              (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE));
    
    return pwm;
}



// 初始化函数实现
void InitHexapod(void) {
    // 计算每条腿的基准位置偏移
    for(int i = 0; i < LEG_COUNT; i++) {
        legOffsets[i].x = INIT_X_OFFSET * cos(LEG_ANGLES[i]) - INIT_Y_OFFSET * sin(LEG_ANGLES[i]) ;
        legOffsets[i].y = INIT_X_OFFSET * sin(LEG_ANGLES[i]) + INIT_Y_OFFSET * cos(LEG_ANGLES[i]) ;
        legOffsets[i].z = INIT_Z_OFFSET ;
        
        // 初始化当前位置为偏移位置
        legPositions[i] = legOffsets[i];
        
        // 初始化关节角度为0
        legAngles[i].theta1 = 0.0;
        legAngles[i].theta2 = 0.0;
        legAngles[i].theta3 = 0.0;
    }
}


// 计算局部坐标系下的位置
void CalculateLocalPosition(LegID leg, double* local_x, double* local_y, double* local_z) {
    // 获取当前腿的全局位置
    double x = legPositions[leg].x;
    double y = legPositions[leg].y;
    double z = legPositions[leg].z;
    
    // 获取当前腿的安装角度
    double angle = LEG_ANGLES[leg];
    
    // 使用旋转矩阵进行坐标转换
    // 从全局坐标系转换到局部坐标系
    *local_x = x * cos(-angle) - y * sin(-angle);
    *local_y = x * sin(-angle) + y * cos(-angle);
    *local_z = z;
}

// 计算单条腿的舵机控制值但不立即执行
void CalculateLegControl(LegID leg, double x, double y, double z, ServoGroupControl* group_control, int start_index) {
    //设置目标位置
    legPositions[leg].x = legOffsets[leg].x + x;
    legPositions[leg].y = legOffsets[leg].y + y;
    legPositions[leg].z = z;

    // 计算局部坐标
    double local_x, local_y, local_z;
    CalculateLocalPosition(leg, &local_x, &local_y, &local_z);

    // 计算逆运动学
    double theta1, theta2, theta3;
    inverse_kinematics(
        local_x,
        local_y,
        local_z,
        L1, L2, L3,
        &theta1, &theta2, &theta3
    );



    // 存储角度
    legAngles[leg].theta1 = theta1;
    legAngles[leg].theta2 = theta2;
    legAngles[leg].theta3 = theta3;

    // 添加安装角度偏移并转换为PWM值
    uint16_t pwm1 = 500 + AngleToPWM(225.0f - theta1 + SERVO_MOUNT_OFFSETS[leg][0]) ;
    uint16_t pwm2 = 500 + AngleToPWM(225.0f - theta2 - SERVO_MOUNT_OFFSETS[leg][1]);
    uint16_t pwm3 = 500 + AngleToPWM(135.0f + theta3 - SERVO_MOUNT_OFFSETS[leg][2]);

    // 存储到控制组
    group_control->ids[start_index] = legServoIDs[leg].hip;
    group_control->ids[start_index + 1] = legServoIDs[leg].thigh;
    group_control->ids[start_index + 2] = legServoIDs[leg].knee;
    group_control->positions[start_index] = pwm1;
    group_control->positions[start_index + 1] = pwm2;
    group_control->positions[start_index + 2] = pwm3;
    group_control->times[start_index] = SERVO_MOVE_TIME;
    group_control->times[start_index + 1] = SERVO_MOVE_TIME;
    group_control->times[start_index + 2] = SERVO_MOVE_TIME;


}

// 计算一组腿的控制值
void CalculateGroupControl(const LegID* legs, int leg_count, double x, double y, double z, ServoGroupControl* group_control) {
    group_control->servo_count = 0;
    for(int i = 0; i < leg_count; i++) {
        CalculateLegControl(legs[i], x, y, z, group_control, group_control->servo_count);
        group_control->servo_count += 3; // 每条腿3个舵机
    }
}

// 三角步态实现
void TripleStepForward(void) {
    // 定义三角步态分组
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
    ServoGroupControl group_control = {0};  // 初始化控制组

    // 步骤1: 抬起第一组腿
    CalculateGroupControl(group1, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 1 - Lift Group1, LEFT_FRONT angles: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           legAngles[LEFT_FRONT].theta1,
           legAngles[LEFT_FRONT].theta2,
           legAngles[LEFT_FRONT].theta3);

    // 步骤2: 第一组腿向前移，第二组腿向后移
    // 计算第一组腿的前移
    CalculateGroupControl(group1, 3, 0, STEP_LENGTH/2, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 计算第二组腿的后移
    CalculateGroupControl(group2, 3, 0, -STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 2 - Move Group1 Forward, LEFT_FRONT angles: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           legAngles[LEFT_FRONT].theta1,
           legAngles[LEFT_FRONT].theta2,
           legAngles[LEFT_FRONT].theta3);

    // 步骤3: 放下第一组腿
    CalculateGroupControl(group1, 3, 0, STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 3 - Lower Group1, LEFT_FRONT angles: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           legAngles[LEFT_FRONT].theta1,
           legAngles[LEFT_FRONT].theta2,
           legAngles[LEFT_FRONT].theta3);

    // 步骤4: 抬起第二组腿
    CalculateGroupControl(group2, 3, 0, -STEP_LENGTH/2, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 4 - Lift Group2, LEFT_FRONT angles: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           legAngles[LEFT_FRONT].theta1,
           legAngles[LEFT_FRONT].theta2,
           legAngles[LEFT_FRONT].theta3);

    // 步骤5: 第二组腿向前移，第一组腿向后移
    // 计算第二组腿的前移
    CalculateGroupControl(group2, 3, 0, STEP_LENGTH/2, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 计算第一组腿的后移
    CalculateGroupControl(group1, 3, 0, -STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 5 - Move Group1 Backward, LEFT_FRONT angles: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           legAngles[LEFT_FRONT].theta1,
           legAngles[LEFT_FRONT].theta2,
           legAngles[LEFT_FRONT].theta3);

    // 步骤6: 放下第二组腿
    CalculateGroupControl(group2, 3, 0, STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 6 - Lower Group2, LEFT_FRONT angles: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           legAngles[LEFT_FRONT].theta1,
           legAngles[LEFT_FRONT].theta2,
           legAngles[LEFT_FRONT].theta3);
}

// 三角步态实现 - 后退移动
void TripleStepBackward(void) {
    // 使用与前进相同的三角步态分组
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
    ServoGroupControl group_control = {0};  // 初始化控制组

    // 步骤1: 抬起第一组腿
    CalculateGroupControl(group1, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("步骤1 - 抬起第一组腿\n");

    // 步骤2: 第一组腿向后移动,第二组腿向前移动(与前进步态相反)
    // 第一组腿向后移动
    CalculateGroupControl(group1, 3, 0, -STEP_LENGTH/2, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 第二组腿向前移动
    CalculateGroupControl(group2, 3, 0, STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("步骤2 - 第一组腿向后移动\n");

    // 步骤3: 放下第一组腿
    CalculateGroupControl(group1, 3, 0, -STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("步骤3 - 放下第一组腿\n");

    // 步骤4: 抬起第二组腿
    CalculateGroupControl(group2, 3, 0, STEP_LENGTH/2, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("步骤4 - 抬起第二组腿\n");

    // 步骤5: 第二组腿向后移动,第一组腿向前移动
    // 第二组腿向后移动
    CalculateGroupControl(group2, 3, 0, -STEP_LENGTH/2, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 第一组腿向前移动
    CalculateGroupControl(group1, 3, 0, STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("步骤5 - 第二组腿向后移动\n");

    // 步骤6: 放下第二组腿
    CalculateGroupControl(group2, 3, 0, -STEP_LENGTH/2, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("步骤6 - 放下第二组腿\n");
}

// 三角步态实现 - 向左直走
void TripleStepLeft(void) {
    // 定义三角步态分组（与向前走相同的分组）
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
    ServoGroupControl group_control = {0};  // 初始化控制组

    // 步骤1: 抬起第一组腿
    CalculateGroupControl(group1, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 1 - 抬起第一组腿\n");

    // 步骤2: 第一组腿向左移，第二组腿向右移
    // 注意：x坐标表示侧向移动，正值为向右
    // 计算第一组腿的左移
    CalculateGroupControl(group1, 3, -STEP_LENGTH/2, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 计算第二组腿的右移
    CalculateGroupControl(group2, 3, STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 2 - 第一组腿向左移，第二组腿向右移\n");

    // 步骤3: 放下第一组腿
    CalculateGroupControl(group1, 3, -STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 3 - 放下第一组腿\n");

    // 步骤4: 抬起第二组腿
    CalculateGroupControl(group2, 3, STEP_LENGTH/2, 0,INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 4 - 抬起第二组腿\n");

    // 步骤5: 第二组腿向左移，第一组腿向右移
    // 计算第二组腿的左移
    CalculateGroupControl(group2, 3, -STEP_LENGTH/2, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 计算第一组腿的右移
    CalculateGroupControl(group1, 3, STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 5 - 第二组腿向左移，第一组腿向右移\n");

    // 步骤6: 放下第二组腿
    CalculateGroupControl(group2, 3, -STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 6 - 放下第二组腿\n");
}

// 三角步态实现 - 向右直走
void TripleStepRight(void) {
    // 定义三角步态分组（与向前走相同的分组）
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
    ServoGroupControl group_control = {0};  // 初始化控制组

    // 步骤1: 抬起第一组腿
    CalculateGroupControl(group1, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 1 - 抬起第一组腿\n");

    // 步骤2: 第一组腿向右移，第二组腿向左移
    // 注意：x坐标表示侧向移动，正值为向右
    // 计算第一组腿的右移
    CalculateGroupControl(group1, 3, STEP_LENGTH/2, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 计算第二组腿的左移
    CalculateGroupControl(group2, 3, -STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 2 - 第一组腿向右移，第二组腿向左移\n");

    // 步骤3: 放下第一组腿
    CalculateGroupControl(group1, 3, STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 3 - 放下第一组腿\n");

    // 步骤4: 抬起第二组腿
    CalculateGroupControl(group2, 3, -STEP_LENGTH/2, 0,INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 4 - 抬起第二组腿\n");

    // 步骤5: 第二组腿向右移，第一组腿向左移
    // 计算第二组腿的右移
    CalculateGroupControl(group2, 3, STEP_LENGTH/2, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    // 计算第一组腿的左移
    CalculateGroupControl(group1, 3, -STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 5 - 第二组腿向右移，第一组腿向左移\n");

    // 步骤6: 放下第二组腿
    CalculateGroupControl(group2, 3, STEP_LENGTH/2, 0, INIT_Z_OFFSET, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 6 - 放下第二组腿\n");
}

// 三角步态实现 - 原地逆时针旋转
void TripleStepRotateCounterClockwise(void) {
    // 定义三角步态分组（与其他步态相同的分组）
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
    ServoGroupControl group_control = {0};  // 初始化控制组

    // 旋转步长 - 相对于腿部初始位置的弧度偏移
    float rotationAngle = STEP_LENGTH / BASE_RADIUS; // 弧度单位

    // 步骤1: 抬起第一组腿
    CalculateGroupControl(group1, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 1 - 抬起第一组腿\n");

    // 步骤2: 第一组腿逆时针旋转，第二组腿顺时针旋转
    // 对第一组腿计算旋转后的位置
    double x_offset, y_offset;
    group_control.servo_count = 0;

    for(int i = 0; i < 3; i++) {
        LegID leg = group1[i];
        // 计算旋转偏移 - 逆时针方向
        x_offset = -legOffsets[leg].y * sin(rotationAngle);
        y_offset = legOffsets[leg].x * sin(rotationAngle);

        // 为抬起的腿设置新位置（带高度）
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET + STEP_HEIGHT, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    group_control.servo_count = 0;

    // 第二组腿 - 顺时针小角度旋转（地面支撑腿）
    for(int i = 0; i < 3; i++) {
        LegID leg = group2[i];
        // 计算旋转偏移 - 顺时针方向（与步骤5对应）
        x_offset = legOffsets[leg].y * sin(rotationAngle);
        y_offset = -legOffsets[leg].x * sin(rotationAngle);

        // 为支撑腿设置新位置
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 2 - 第一组腿逆时针旋转，第二组腿顺时针旋转\n");

    // 步骤3: 放下第一组腿
    group_control.servo_count = 0;
    for(int i = 0; i < 3; i++) {
        LegID leg = group1[i];
        // 使用相同的偏移，但更改高度
        x_offset = -legOffsets[leg].y * sin(rotationAngle);
        y_offset = legOffsets[leg].x * sin(rotationAngle);

        // 放下腿
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 3 - 放下第一组腿\n");

    // 步骤4: 抬起第二组腿
    CalculateGroupControl(group2, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 4 - 抬起第二组腿\n");

    // 步骤5: 第二组腿逆时针旋转，第一组腿顺时针旋转
    group_control.servo_count = 0;

    // 第二组腿 - 逆时针旋转（抬起的腿）
    for(int i = 0; i < 3; i++) {
        LegID leg = group2[i];
        // 计算旋转偏移 - 逆时针方向
        x_offset = -legOffsets[leg].y * sin(rotationAngle);
        y_offset = legOffsets[leg].x * sin(rotationAngle);

        // 为抬起的腿设置新位置（带高度）
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET + STEP_HEIGHT, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    group_control.servo_count = 0;

    // 第一组腿 - 顺时针小角度旋转（地面支撑腿）
    for(int i = 0; i < 3; i++) {
        LegID leg = group1[i];
        // 计算旋转偏移 - 顺时针方向
        x_offset = legOffsets[leg].y * sin(rotationAngle);
        y_offset = -legOffsets[leg].x * sin(rotationAngle);

        // 为支撑腿设置新位置
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 5 - 第二组腿逆时针旋转，第一组腿顺时针旋转\n");

    // 步骤6: 放下第二组腿
    group_control.servo_count = 0;
    for(int i = 0; i < 3; i++) {
        LegID leg = group2[i];
        // 使用相同的偏移，但更改高度
        x_offset = -legOffsets[leg].y * sin(rotationAngle);
        y_offset = legOffsets[leg].x * sin(rotationAngle);

        // 放下腿
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 6 - 放下第二组腿\n");
}

// 三角步态实现 - 原地顺时针旋转
void TripleStepRotateClockwise(void) {
    // 定义三角步态分组（与其他步态相同的分组）
    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK};
    ServoGroupControl group_control = {0};  // 初始化控制组

    // 旋转步长 - 相对于腿部初始位置的弧度偏移
    float rotationAngle = STEP_LENGTH / BASE_RADIUS; // 弧度单位

    // 步骤1: 抬起第一组腿
    CalculateGroupControl(group1, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 1 - 抬起第一组腿\n");

    // 步骤2: 第一组腿顺时针旋转，第二组腿逆时针旋转
    // 对第一组腿计算旋转后的位置
    double x_offset, y_offset;
    group_control.servo_count = 0;

    for(int i = 0; i < 3; i++) {
        LegID leg = group1[i];
        // 计算旋转偏移 - 顺时针方向
        x_offset = legOffsets[leg].y * sin(rotationAngle);
        y_offset = -legOffsets[leg].x * sin(rotationAngle);

        // 为抬起的腿设置新位置（带高度）
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET + STEP_HEIGHT, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    group_control.servo_count = 0;

    // 第二组腿 - 逆时针小角度旋转（地面支撑腿）
    for(int i = 0; i < 3; i++) {
        LegID leg = group2[i];
        // 计算旋转偏移 - 顺时针方向（与步骤5对应）
        x_offset = -legOffsets[leg].y * sin(rotationAngle);
        y_offset = legOffsets[leg].x * sin(rotationAngle);

        // 为支撑腿设置新位置
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 2 - 第一组腿顺时针旋转，第二组腿逆时针旋转\n");

    // 步骤3: 放下第一组腿
    group_control.servo_count = 0;
    for(int i = 0; i < 3; i++) {
        LegID leg = group1[i];
        // 使用相同的偏移，但更改高度
        x_offset = legOffsets[leg].y * sin(rotationAngle);
        y_offset = -legOffsets[leg].x * sin(rotationAngle);

        // 放下腿
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 3 - 放下第一组腿\n");

    // 步骤4: 抬起第二组腿
    CalculateGroupControl(group2, 3, 0, 0, INIT_Z_OFFSET + STEP_HEIGHT, &group_control);
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 4 - 抬起第二组腿\n");

    // 步骤5: 第二组腿顺时针旋转，第一组腿逆时针旋转
    group_control.servo_count = 0;

    // 第二组腿 - 顺时针旋转（抬起的腿）
    for(int i = 0; i < 3; i++) {
        LegID leg = group2[i];
        // 计算旋转偏移 - 逆时针方向
        x_offset = legOffsets[leg].y * sin(rotationAngle);
        y_offset = -legOffsets[leg].x * sin(rotationAngle);

        // 为抬起的腿设置新位置（带高度）
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET + STEP_HEIGHT, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    group_control.servo_count = 0;

    // 第一组腿 - 逆时针小角度旋转（地面支撑腿）
    for(int i = 0; i < 3; i++) {
        LegID leg = group1[i];
        // 计算旋转偏移 - 逆时针方向
        x_offset = -legOffsets[leg].y * sin(rotationAngle);
        y_offset = legOffsets[leg].x * sin(rotationAngle);

        // 为支撑腿设置新位置
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 5 - 第二组腿顺时针旋转，第一组腿逆时针旋转\n");

    // 步骤6: 放下第二组腿
    group_control.servo_count = 0;
    for(int i = 0; i < 3; i++) {
        LegID leg = group2[i];
        // 使用相同的偏移，但更改高度
        x_offset = legOffsets[leg].y * sin(rotationAngle);
        y_offset = -legOffsets[leg].x * sin(rotationAngle);

        // 放下腿
        CalculateLegControl(leg, x_offset, y_offset, INIT_Z_OFFSET, &group_control, group_control.servo_count);
        group_control.servo_count += 3;
    }

    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);
    HAL_Delay(SERVO_MOVE_TIME);
    printf("Step 6 - 放下第二组腿\n");
}

/**
 * @brief 分步设置六足机器人所有腿到指定角度
 *
 * 此函数将0号舵机设置为30度，
 * 并将每条腿的第一个关节设为60度，第二个关节设为-63.13度，第三个关节设为62.95度
 * 控制分为4步完成，避免同时控制太多舵机导致卡死
 */
void SetSpecificJointAnglesStepwise(void) {




    const LegID group1[] = {LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK};  // Triangle group 1
    const LegID group2[] = {RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK}; // Triangle group 2
    const LegID allLegs[] = {LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK,
                                 RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK};

    SetLegsToAngles(group1,  3, 90.0, -20.0, 20.0, 1000);
    SetLegsToAngles(group1,  3, 90.0, 100.0, 70.0, 1000);
    SetLegsToAngles(group2,  3, 90.0, -20.0, 20.0, 1000);
    SetLegsToAngles(group2,  3, 90.0, 100.0, 70.0, 1000);
    SetLegsToAngles(allLegs, 6, 81.89, 116.87, 50.91, 1000);

    Servo_SetAngle(30, 1000);
    HAL_Delay(SERVO_MOVE_TIME + 50); // 额外等待50ms确保命令完成
}


/**
 * @brief 控制指定的多条腿的三个关节依次移动到指定角度
 *
 * @param legs 要控制的腿的ID数组
 * @param leg_count 要控制的腿的数量
 * @param theta1 髋关节目标角度(度)
 * @param theta2 大腿关节目标角度(度)
 * @param theta3 膝关节目标角度(度)
 * @param move_time 每个关节的运动时间(ms)
 */
void SetLegsToAngles(const LegID* legs, int leg_count, double theta1, double theta2, double theta3, uint16_t move_time) {
    ServoGroupControl group_control = {0};

    printf("依次设置 %d 条腿的三个关节到指定角度: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           leg_count, theta1, theta2, theta3);

    // 步骤1: 首先控制所有指定腿的髋关节
    printf("步骤1: 设置髋关节 (theta1=%.2f)\n", theta1);
    group_control.servo_count = 0;

    for (int i = 0; i < leg_count; i++) {
        LegID leg = legs[i];

        // 髋关节
        group_control.ids[group_control.servo_count] = legServoIDs[leg].hip;
        uint16_t pwm1 = 500 + AngleToPWM(225.0f - theta1);
        group_control.positions[group_control.servo_count] = pwm1;
        group_control.times[group_control.servo_count] = move_time;
        group_control.servo_count++;

        // 更新内部角度记录
        legAngles[leg].theta1 = theta1;
    }

    // 发送命令控制髋关节舵机
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);

    // 等待髋关节舵机运动完成
    HAL_Delay(move_time + 50);  // 额外等待50ms确保命令完成

    // 步骤2: 控制所有指定腿的大腿关节
        printf("步骤2: 设置大腿关节 (theta2=%.2f)\n", theta2);
        group_control.servo_count = 0;

        for (int i = 0; i < leg_count; i++) {
            LegID leg = legs[i];

            // 大腿关节 - 应用舵机安装偏移
            group_control.ids[group_control.servo_count] = legServoIDs[leg].thigh;
            uint16_t pwm2 = 500 + AngleToPWM(135.0f+theta2);
            group_control.positions[group_control.servo_count] = pwm2;
            group_control.times[group_control.servo_count] = move_time;
            group_control.servo_count++;

            // 更新内部角度记录
            legAngles[leg].theta2 = theta2;
        }

        // 发送命令控制大腿关节舵机
        Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);

        // 等待大腿关节舵机运动完成
        HAL_Delay(move_time + 50);  // 额外等待50ms确保命令完成


    // 步骤3: 最后控制所有指定腿的膝关节
    printf("步骤3: 设置膝关节 (theta3=%.2f)\n", theta3);
    group_control.servo_count = 0;

    for (int i = 0; i < leg_count; i++) {
        LegID leg = legs[i];

        // 膝关节 - 应用舵机安装偏移
        group_control.ids[group_control.servo_count] = legServoIDs[leg].knee;
        uint16_t pwm3 = 500 + AngleToPWM(135.0+theta3); /*- SERVO_MOUNT_OFFSETS[leg][2]*/
        group_control.positions[group_control.servo_count] = pwm3;
        group_control.times[group_control.servo_count] = move_time;
        group_control.servo_count++;

        // 更新内部角度记录
        legAngles[leg].theta3 = theta3;
    }

    // 发送命令控制膝关节舵机
    Servo_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);

    // 等待膝关节舵机运动完成
    HAL_Delay(move_time + 50);  // 额外等待50ms确保命令完成




    printf("所有关节角度设置完成！\n");
}

/**
 * @brief 控制指定的多条腿的三个关节依次移动到指定角度
 *
 * @param legs 要控制的腿的ID数组
 * @param leg_count 要控制的腿的数量
 * @param theta1 髋关节目标角度(度)
 * @param theta2 大腿关节目标角度(度)
 * @param theta3 膝关节目标角度(度)
 * @param move_time 每个关节的运动时间(ms)
 */
void SetLegsToAngles2(const LegID* legs, int leg_count, double theta1, double theta2, double theta3, uint16_t move_time) {
    ServoGroupControl group_control = {0};

    printf("依次设置 %d 条腿的三个关节到指定角度: theta1=%.2f, theta2=%.2f, theta3=%.2f\n",
           leg_count, theta1, theta2, theta3);

    // 步骤1: 首先控制所有指定腿的髋关节
    printf("步骤1: 设置髋关节 (theta1=%.2f)\n", theta1);
    group_control.servo_count = 0;

    for (int i = 0; i < leg_count; i++) {
        LegID leg = legs[i];

        // 髋关节
        group_control.ids[group_control.servo_count] = legServoIDs[leg].hip;
        uint16_t pwm1 = 500 + AngleToPWM(225.0f - theta1);
        group_control.positions[group_control.servo_count] = pwm1;
        group_control.times[group_control.servo_count] = move_time;
        group_control.servo_count++;

        // 更新内部角度记录
        legAngles[leg].theta1 = theta1;
    }

    // 发送命令控制髋关节舵机
    Servo2_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);

    // 等待髋关节舵机运动完成
    HAL_Delay(move_time + 50);  // 额外等待50ms确保命令完成

    // 步骤2: 控制所有指定腿的大腿关节
        printf("步骤2: 设置大腿关节 (theta2=%.2f)\n", theta2);
        group_control.servo_count = 0;

        for (int i = 0; i < leg_count; i++) {
            LegID leg = legs[i];

            // 大腿关节 - 应用舵机安装偏移
            group_control.ids[group_control.servo_count] = legServoIDs[leg].thigh;
            uint16_t pwm2 = 500 + AngleToPWM(135.0f+theta2);
            group_control.positions[group_control.servo_count] = pwm2;
            group_control.times[group_control.servo_count] = move_time;
            group_control.servo_count++;

            // 更新内部角度记录
            legAngles[leg].theta2 = theta2;
        }

        // 发送命令控制大腿关节舵机
        Servo2_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);

        // 等待大腿关节舵机运动完成
        HAL_Delay(move_time + 50);  // 额外等待50ms确保命令完成


    // 步骤3: 最后控制所有指定腿的膝关节
    printf("步骤3: 设置膝关节 (theta3=%.2f)\n", theta3);
    group_control.servo_count = 0;

    for (int i = 0; i < leg_count; i++) {
        LegID leg = legs[i];

        // 膝关节 - 应用舵机安装偏移
        group_control.ids[group_control.servo_count] = legServoIDs[leg].knee;
        uint16_t pwm3 = 500 + AngleToPWM(135.0+theta3); /*- SERVO_MOUNT_OFFSETS[leg][2]*/
        group_control.positions[group_control.servo_count] = pwm3;
        group_control.times[group_control.servo_count] = move_time;
        group_control.servo_count++;

        // 更新内部角度记录
        legAngles[leg].theta3 = theta3;
    }

    // 发送命令控制膝关节舵机
    Servo2_SetMultiPosition(group_control.ids, group_control.positions, group_control.times, group_control.servo_count);

    // 等待膝关节舵机运动完成
    HAL_Delay(move_time + 50);  // 额外等待50ms确保命令完成




    printf("所有关节角度设置完成！\n");
}
