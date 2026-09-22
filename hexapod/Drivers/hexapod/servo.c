#include "servo.h"
#include "main.h"
#include "tim.h"

// 假设使用UART2进行通信
extern UART_HandleTypeDef huart2;

// 定义通信管理结构体
static ServoComm_t servoComm = {
    .state = SERVO_STATE_IDLE,
    .txSize = 0,
    .rxSize = 0,
    .rxExpectedSize = 0,
    .timeoutTick = 0,
    .responseRequired = false
};

// 用于存储读取操作的结果
static uint16_t readPosition = 0;
static float readTemp = 0.0f;
static float readVoltage = 0.0f;
static char readVersion[20] = {0};

// 初始化舵机通信
void Servo_Init(void) {
    // 启动UART接收中断，接收单个字符
    HAL_UART_Receive_IT(&huart2, &servoComm.rxBuffer[0], 1);
}

// 处理舵机通信状态，应在主循环中定期调用
void Servo_Process(void) {
    // 检查是否超时
    if (servoComm.state == SERVO_STATE_WAITING) {
        if (HAL_GetTick() > servoComm.timeoutTick) {
            servoComm.state = SERVO_STATE_TIMEOUT;
        }
    }

    // 处理超时情况
    if (servoComm.state == SERVO_STATE_TIMEOUT) {
        // 重置状态
        servoComm.state = SERVO_STATE_IDLE;
        servoComm.rxSize = 0;
        // 重新开始接收
        HAL_UART_Receive_IT(&huart2, &servoComm.rxBuffer[0], 1);
    }

    // 解析接收到的响应
    if (servoComm.state == SERVO_STATE_RECEIVED) {
        // 解析响应，根据最后发送的命令类型处理
        // 这里需要根据您的具体应用场景进行扩展
        servoComm.rxBuffer[servoComm.rxSize] = '\0'; // 确保字符串结束

        char* response = (char*)servoComm.rxBuffer;

        // 检查响应类型，这里仅做示例
        if (strstr(response, "PRAD") != NULL) {
            // 解析位置读取响应
            sscanf(response, "#%*3dP%4d!", &readPosition);
        } else if (strstr(response, "PRTV") != NULL) {
            // 解析温度和电压读取响应
            uint8_t t, v;
            sscanf(response, "#%*3dT%2dV%2d!", &t, &v);
            readTemp = (float)t;
            readVoltage = (float)v / 10.0f;
        } else if (strstr(response, "PVER") != NULL) {
            // 解析版本信息
            strcpy(readVersion, response);
        }

        // 重置状态
        servoComm.state = SERVO_STATE_IDLE;
        servoComm.rxSize = 0;

        // 重新开始接收
        HAL_UART_Receive_IT(&huart2, &servoComm.rxBuffer[0], 1);
    }
}

// UART接收完成回调，需要在HAL_UART_RxCpltCallback中调用
void Servo_UART_RxCallback(void) {
    // 收到一个字符
    servoComm.rxSize++;

    // 检查是否是响应结束符'!'
    if (servoComm.rxBuffer[servoComm.rxSize - 1] == '!') {
        if (servoComm.state == SERVO_STATE_WAITING) {
            servoComm.state = SERVO_STATE_RECEIVED;
        }
    } else {
        // 继续接收下一个字符
        HAL_UART_Receive_IT(&huart2, &servoComm.rxBuffer[servoComm.rxSize], 1);
    }
}

// UART发送完成回调，需要在HAL_UART_TxCpltCallback中调用
void Servo_UART_TxCallback(void) {
    if (servoComm.responseRequired) {
        // 如果需要等待响应，则设置状态为等待
        servoComm.state = SERVO_STATE_WAITING;
        servoComm.timeoutTick = HAL_GetTick() + 100; // 100ms超时
    } else {
        // 如果不需要响应，则直接设置为空闲
        servoComm.state = SERVO_STATE_IDLE;
    }
}

// 发送命令到舵机，带响应选项
void Servo_SendCommand(const char* cmd, bool needResponse) {
    // 如果当前正在通信，则等待
    while (servoComm.state != SERVO_STATE_IDLE) {
        Servo_Process();
    }

    // 复制命令到发送缓冲区
    strcpy((char*)servoComm.txBuffer, cmd);
    servoComm.txSize = strlen(cmd);
    servoComm.responseRequired = needResponse;

    // 开始发送
    servoComm.state = SERVO_STATE_SENDING;
    HAL_UART_Transmit_IT(&huart2, servoComm.txBuffer, servoComm.txSize);
}

// 发送原始数据，带响应选项
void Servo_SendCommandRaw(const uint8_t* data, uint16_t size, bool needResponse) {
    // 如果当前正在通信，则等待
    while (servoComm.state != SERVO_STATE_IDLE) {
        Servo_Process();
    }

    // 复制命令到发送缓冲区
    memcpy(servoComm.txBuffer, data, size);
    servoComm.txSize = size;
    servoComm.responseRequired = needResponse;

    // 开始发送
    servoComm.state = SERVO_STATE_SENDING;
    HAL_UART_Transmit_IT(&huart2, servoComm.txBuffer, servoComm.txSize);
}

// 检查舵机通信是否空闲
bool Servo_IsIdle(void) {
    return (servoComm.state == SERVO_STATE_IDLE);
}

// 控制单个舵机位置
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t time) {
    char cmd[20];
    // 确保参数在有效范围内
    if(position < SERVO_PWM_MIN) position = SERVO_PWM_MIN;
    if(position > SERVO_PWM_MAX) position = SERVO_PWM_MAX;
    if(time > SERVO_TIME_MAX) time = SERVO_TIME_MAX;

    sprintf(cmd, "#%03dP%04dT%04d!", id, position, time);
    Servo_SendCommand(cmd, false); // 不需要响应
}

// 控制多个舵机位置
void Servo_SetMultiPosition(const uint8_t* ids, const uint16_t* positions, const uint16_t* times, uint8_t count) {
    char cmd[256] = "{";
    char temp[30];

    for(uint8_t i = 0; i < count; i++) {
        sprintf(temp, "#%03dP%04dT%04d!", ids[i], positions[i], times[i]);
        strcat(cmd, temp);
    }
    strcat(cmd, "}");
    Servo_SendCommand(cmd, false); // 不需要响应
}

// 读取舵机位置
uint16_t Servo_ReadPosition(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPRAD!", id);

    // 发送命令并等待响应
    Servo_SendCommand(cmd, true);

    // 等待响应处理完成
    while (servoComm.state != SERVO_STATE_IDLE && servoComm.state != SERVO_STATE_TIMEOUT) {
        Servo_Process();
    }

    return readPosition;
}

// 停止舵机
void Servo_Stop(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPDST!", id);
    Servo_SendCommand(cmd, false);
}

// 暂停舵机
void Servo_Pause(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPDPT!", id);
    Servo_SendCommand(cmd, false);
}

// 继续运动
void Servo_Continue(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPDCT!", id);
    Servo_SendCommand(cmd, false);
}

// 修改舵机ID
void Servo_SetID(uint8_t oldID, uint8_t newID) {
    char cmd[20];
    if(newID <= SERVO_ID_MAX) {
        sprintf(cmd, "#%03dPID%03d!", oldID, newID);
        Servo_SendCommand(cmd, false);
    }
}

// 设置工作模式
void Servo_SetMode(uint8_t id, ServoMode mode) {
    char cmd[20];
    sprintf(cmd, "#%03dPMOD%d!", id, mode);
    Servo_SendCommand(cmd, false);
}

// 释放扭力
void Servo_ReleaseTorque(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPULK!", id);
    Servo_SendCommand(cmd, false);
}

// 恢复扭力
void Servo_RestoreTorque(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPULR!", id);
    Servo_SendCommand(cmd, false);
}

// 读取温度和电压
void Servo_ReadTempAndVoltage(uint8_t id, float* temp, float* voltage) {
    char cmd[20];
    sprintf(cmd, "#%03dPRTV!", id);

    // 发送命令并等待响应
    Servo_SendCommand(cmd, true);

    // 等待响应处理完成
    while (servoComm.state != SERVO_STATE_IDLE && servoComm.state != SERVO_STATE_TIMEOUT) {
        Servo_Process();
    }

    *temp = readTemp;
    *voltage = readVoltage;
}

// 读取版本信息
void Servo_ReadVersion(uint8_t id, char* version) {
    char cmd[20];
    sprintf(cmd, "#%03dPVER!", id);

    // 发送命令并等待响应
    Servo_SendCommand(cmd, true);

    // 等待响应处理完成
    while (servoComm.state != SERVO_STATE_IDLE && servoComm.state != SERVO_STATE_TIMEOUT) {
        Servo_Process();
    }

    strcpy(version, readVersion);
}

void Servo_SetAngle(uint8_t target_angle, uint32_t time_ms)
{
    // 限制目标角度范围
    if(target_angle > 180) target_angle = 180;

    // 如果时间为0或当前角度等于目标角度，则立即设置
    if(time_ms == 0 || g_current_angle == target_angle)
    {
        uint32_t pulse_width = 500 + (uint32_t)((target_angle * 2000) / 180);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width);
        g_current_angle = target_angle; // 更新当前角度
        return;
    }

    // 计算角度变化量和步进次数
    int16_t angle_diff = (int16_t)target_angle - (int16_t)g_current_angle;
    uint32_t steps = time_ms / 20; // 每20ms更新一次，大约是舵机的控制周期

    // 防止步数为0
    if(steps == 0) steps = 1;

    // 计算每步的角度变化量(使用浮点数以提高精度)
    float angle_step = (float)angle_diff / steps;

    // 逐步移动舵机
    for(uint32_t i = 0; i < steps; i++)
    {
        // 计算当前步的角度
        float current_step_angle = g_current_angle + angle_step * (i + 1);
        uint8_t step_angle = (uint8_t)(current_step_angle + 0.5f); // 四舍五入

        // 计算脉冲宽度并设置
        uint32_t pulse_width = 500 + (uint32_t)((step_angle * 2000) / 180);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width);

        // 等待20ms
        HAL_Delay(20);
    }

    // 最后确保设置为目标角度(避免累积误差)
    uint32_t final_pulse_width = 500 + (uint32_t)((target_angle * 2000) / 180);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, final_pulse_width);

    // 更新当前角度
    g_current_angle = target_angle;
}
