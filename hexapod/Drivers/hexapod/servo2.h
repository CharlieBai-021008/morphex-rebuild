#ifndef __SERVO2_H
#define __SERVO2_H

#include "main.h"
#include "string.h"
#include "stdio.h"

// 舵机参数范围定义
#define SERVO2_ID_MIN        0
#define SERVO2_ID_MAX        254
#define SERVO2_BROADCAST_ID  255
#define SERVO2_PWM_MIN       500
#define SERVO2_PWM_MAX       2500
#define SERVO2_TIME_MIN      0
#define SERVO2_TIME_MAX      9999

// 舵机工作模式
typedef enum {
    SERVO2_MODE_270_CW = 1,    // 舵机模式,270度,顺时针
    SERVO2_MODE_270_CCW = 2,   // 舵机模式,270度,逆时针
    SERVO2_MODE_180_CW = 3,    // 舵机模式,180度,顺时针
    SERVO2_MODE_180_CCW = 4,   // 舵机模式,180度,逆时针
    SERVO2_MODE_360_TURN_CW = 5,  // 马达模式,360度定圈,顺时针
    SERVO2_MODE_360_TURN_CCW = 6, // 马达模式,360度定圈,逆时针
    SERVO2_MODE_360_TIME_CW = 7,  // 马达模式,360度定时,顺时针
    SERVO2_MODE_360_TIME_CCW = 8  // 马达模式,360度定时,逆时针
} Servo2Mode;

// 舵机启动模式
typedef enum {
    SERVO2_STARTUP_TO_MIDDLE = 1,  // 开机旋转到启动位置
    SERVO2_STARTUP_KEEP_POS = 2,   // 开机保持当前位置
    SERVO2_STARTUP_RELEASE = 3     // 开机无力
} Servo2StartupMode;

// 波特率选项
typedef enum {
    SERVO2_BAUD_9600 = 1,
    SERVO2_BAUD_19200 = 2,
    SERVO2_BAUD_38400 = 3,
    SERVO2_BAUD_57600 = 4,
    SERVO2_BAUD_115200 = 5,
    SERVO2_BAUD_128000 = 6,
    SERVO2_BAUD_256000 = 7,
    SERVO2_BAUD_1000000 = 8
} Servo2BaudRate;

// 通信缓冲区定义
#define SERVO2_RX_BUFFER_SIZE 64
extern uint8_t Servo2_RxBuffer[SERVO2_RX_BUFFER_SIZE];
extern uint8_t Servo2_RxData;
extern uint8_t Servo2_RxComplete;
extern uint16_t Servo2_RxIndex;

// 基本控制函数
void Servo2_Init(void);
void Servo2_SetPosition(uint8_t id, uint16_t position, uint16_t time);
void Servo2_SetMultiPosition(const uint8_t* ids, const uint16_t* positions, const uint16_t* times, uint8_t count);
uint16_t Servo2_ReadPosition(uint8_t id);
void Servo2_Stop(uint8_t id);
void Servo2_Pause(uint8_t id);
void Servo2_Continue(uint8_t id);

// 配置函数
void Servo2_SetID(uint8_t oldID, uint8_t newID);
uint8_t Servo2_ReadID(uint8_t id);
void Servo2_SetMode(uint8_t id, Servo2Mode mode);
Servo2Mode Servo2_ReadMode(uint8_t id);
void Servo2_SetStartupMode(uint8_t id, Servo2StartupMode mode);
Servo2StartupMode Servo2_ReadStartupMode(uint8_t id);
void Servo2_SetBaudRate(uint8_t id, Servo2BaudRate baud);

// 力矩控制
void Servo2_ReleaseTorque(uint8_t id);
void Servo2_RestoreTorque(uint8_t id);

// 校准和限位
void Servo2_Calibrate(uint8_t id);
void Servo2_SetMinPosition(uint8_t id);
void Servo2_SetMaxPosition(uint8_t id);

// 恢复出厂设置
void Servo2_FactoryResetKeepID(uint8_t id);
void Servo2_FactoryResetAll(uint8_t id);

// 状态读取
void Servo2_ReadTempAndVoltage(uint8_t id, float* temp, float* voltage);
void Servo2_ReadVersion(uint8_t id, char* version);

// 辅助函数
void Servo2_SendCommand(const char* cmd);
uint8_t Servo2_WaitForResponse(char* response, uint32_t timeout);
void Servo2_ProcessRxData(void);
void Servo2_RxCallback(void);

#endif /* __SERVO2_H */
