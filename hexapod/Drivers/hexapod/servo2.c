#include "main.h"
#include "servo2.h"

// 使用UART3进行通信
extern UART_HandleTypeDef huart3;

// 中断模式接收缓冲区
uint8_t Servo2_RxBuffer[SERVO2_RX_BUFFER_SIZE];
uint8_t Servo2_RxData;
uint8_t Servo2_RxComplete = 0;
uint16_t Servo2_RxIndex = 0;

// 初始化函数
void Servo2_Init(void) {
    // 启动UART3接收中断
    HAL_UART_Receive_IT(&huart3, &Servo2_RxData, 1);
}

// 接收中断回调函数
void Servo2_RxCallback(void) {
    // 将接收到的数据存入缓冲区
    if (Servo2_RxIndex < SERVO2_RX_BUFFER_SIZE) {
        Servo2_RxBuffer[Servo2_RxIndex++] = Servo2_RxData;

        // 检查是否接收到结束符'!'
        if (Servo2_RxData == '!') {
            Servo2_RxComplete = 1;
        }
    } else {
        // 缓冲区溢出，重置索引
        Servo2_RxIndex = 0;
    }

    // 继续接收下一个字节
    HAL_UART_Receive_IT(&huart3, &Servo2_RxData, 1);
}

// 处理接收到的数据
void Servo2_ProcessRxData(void) {
    if (Servo2_RxComplete) {
        // 数据处理逻辑根据具体应用添加

        // 处理完成后重置状态
        Servo2_RxIndex = 0;
        Servo2_RxComplete = 0;
    }
}

// 发送命令到舵机
void Servo2_SendCommand(const char* cmd) {
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), 100);
}

// 等待舵机响应
uint8_t Servo2_WaitForResponse(char* response, uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();

    // 重置接收状态
    Servo2_RxComplete = 0;
    Servo2_RxIndex = 0;

    // 等待接收完成或超时
    while (!Servo2_RxComplete) {
        if (HAL_GetTick() - startTime > timeout) {
            return HAL_TIMEOUT;
        }
        // 可以添加延时
        HAL_Delay(1);
    }

    // 复制接收到的数据到response
    if (Servo2_RxIndex <= SERVO2_RX_BUFFER_SIZE) {
        memcpy(response, Servo2_RxBuffer, Servo2_RxIndex);
        response[Servo2_RxIndex] = '\0'; // 确保字符串结束
        return HAL_OK;
    }

    return HAL_ERROR;
}

// 控制单个舵机位置
void Servo2_SetPosition(uint8_t id, uint16_t position, uint16_t time) {
    char cmd[20];
    // 确保参数在有效范围内
    if(position < SERVO2_PWM_MIN) position = SERVO2_PWM_MIN;
    if(position > SERVO2_PWM_MAX) position = SERVO2_PWM_MAX;
    if(time > SERVO2_TIME_MAX) time = SERVO2_TIME_MAX;

    sprintf(cmd, "#%03dP%04dT%04d!", id, position, time);
    Servo2_SendCommand(cmd);
}

// 控制多个舵机位置
void Servo2_SetMultiPosition(const uint8_t* ids, const uint16_t* positions, const uint16_t* times, uint8_t count) {
    char cmd[256] = "{";
    char temp[30];

    for(uint8_t i = 0; i < count; i++) {
        sprintf(temp, "#%03dP%04dT%04d!", ids[i], positions[i], times[i]);
        strcat(cmd, temp);
    }
    strcat(cmd, "}");
    Servo2_SendCommand(cmd);
}

// 读取舵机位置
uint16_t Servo2_ReadPosition(uint8_t id) {
    char cmd[20];
    char response[20];
    sprintf(cmd, "#%03dPRAD!", id);
    Servo2_SendCommand(cmd);

    if(Servo2_WaitForResponse(response, 100) == HAL_OK) {
        uint16_t pos;
        sscanf(response, "#%*3dP%4d!", &pos);
        return pos;
    }
    return 0;
}

// 停止舵机
void Servo2_Stop(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPDST!", id);
    Servo2_SendCommand(cmd);
}

// 暂停舵机
void Servo2_Pause(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPDPT!", id);
    Servo2_SendCommand(cmd);
}

// 继续运动
void Servo2_Continue(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPDCT!", id);
    Servo2_SendCommand(cmd);
}

// 修改舵机ID
void Servo2_SetID(uint8_t oldID, uint8_t newID) {
    char cmd[20];
    if(newID <= SERVO2_ID_MAX) {
        sprintf(cmd, "#%03dPID%03d!", oldID, newID);
        Servo2_SendCommand(cmd);
    }
}

// 设置工作模式
void Servo2_SetMode(uint8_t id, Servo2Mode mode) {
    char cmd[20];
    sprintf(cmd, "#%03dPMOD%d!", id, mode);
    Servo2_SendCommand(cmd);
}

// 释放扭力
void Servo2_ReleaseTorque(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPULK!", id);
    Servo2_SendCommand(cmd);
}

// 恢复扭力
void Servo2_RestoreTorque(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPULR!", id);
    Servo2_SendCommand(cmd);
}

// 读取温度和电压
void Servo2_ReadTempAndVoltage(uint8_t id, float* temp, float* voltage) {
    char cmd[20];
    char response[20];
    sprintf(cmd, "#%03dPRTV!", id);
    Servo2_SendCommand(cmd);

    if(Servo2_WaitForResponse(response, 100) == HAL_OK) {
        uint8_t t, v;
        sscanf(response, "#%*3dT%2dV%2d!", &t, &v);
        *temp = (float)t;
        *voltage = (float)v / 10.0f;
    }
}

// 读取版本信息
void Servo2_ReadVersion(uint8_t id, char* version) {
    char cmd[20];
    sprintf(cmd, "#%03dPVER!", id);
    Servo2_SendCommand(cmd);
    Servo2_WaitForResponse(version, 100);
}

// 读取ID
uint8_t Servo2_ReadID(uint8_t id) {
    char cmd[20];
    char response[20];
    sprintf(cmd, "#%03dPRID!", id);
    Servo2_SendCommand(cmd);

    if(Servo2_WaitForResponse(response, 100) == HAL_OK) {
        uint8_t newId;
        sscanf(response, "#%3d!", &newId);
        return newId;
    }
    return 0;
}

// 读取模式
Servo2Mode Servo2_ReadMode(uint8_t id) {
    char cmd[20];
    char response[20];
    sprintf(cmd, "#%03dPRMD!", id);
    Servo2_SendCommand(cmd);

    if(Servo2_WaitForResponse(response, 100) == HAL_OK) {
        uint8_t mode;
        sscanf(response, "#%*3dMOD%1d!", &mode);
        return (Servo2Mode)mode;
    }
    return (Servo2Mode)0;
}

// 设置启动模式
void Servo2_SetStartupMode(uint8_t id, Servo2StartupMode mode) {
    char cmd[20];
    sprintf(cmd, "#%03dPSMD%d!", id, mode);
    Servo2_SendCommand(cmd);
}

// 读取启动模式
Servo2StartupMode Servo2_ReadStartupMode(uint8_t id) {
    char cmd[20];
    char response[20];
    sprintf(cmd, "#%03dPRSM!", id);
    Servo2_SendCommand(cmd);

    if(Servo2_WaitForResponse(response, 100) == HAL_OK) {
        uint8_t mode;
        sscanf(response, "#%*3dSMD%1d!", &mode);
        return (Servo2StartupMode)mode;
    }
    return (Servo2StartupMode)0;
}

// 设置波特率
void Servo2_SetBaudRate(uint8_t id, Servo2BaudRate baud) {
    char cmd[20];
    sprintf(cmd, "#%03dPBAD%d!", id, baud);
    Servo2_SendCommand(cmd);
}

// 校准
void Servo2_Calibrate(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPCLB!", id);
    Servo2_SendCommand(cmd);
}

// 设置最小位置
void Servo2_SetMinPosition(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPMIN!", id);
    Servo2_SendCommand(cmd);
}

// 设置最大位置
void Servo2_SetMaxPosition(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPMAX!", id);
    Servo2_SendCommand(cmd);
}

// 恢复出厂设置但保留ID
void Servo2_FactoryResetKeepID(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPRST!", id);
    Servo2_SendCommand(cmd);
}

// 恢复出厂设置包括ID
void Servo2_FactoryResetAll(uint8_t id) {
    char cmd[20];
    sprintf(cmd, "#%03dPRSA!", id);
    Servo2_SendCommand(cmd);
}
