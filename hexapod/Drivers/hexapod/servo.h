#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

// 舵机参数范围定义
#define SERVO_ID_MIN        0
#define SERVO_ID_MAX        254
#define SERVO_BROADCAST_ID  255
#define SERVO_PWM_MIN       500
#define SERVO_PWM_MAX       2500
#define SERVO_TIME_MIN      0
#define SERVO_TIME_MAX      9999

// 串口接收缓冲区大小
#define UART_RX_BUFFER_SIZE 64
#define UART_TX_BUFFER_SIZE 256

// 舵机工作模式
typedef enum {
    SERVO_MODE_270_CW = 1,    // 舵机模式,270度,顺时针
    SERVO_MODE_270_CCW = 2,   // 舵机模式,270度,逆时针
    SERVO_MODE_180_CW = 3,    // 舵机模式,180度,顺时针
    SERVO_MODE_180_CCW = 4,   // 舵机模式,180度,逆时针
    SERVO_MODE_360_TURN_CW = 5,  // 马达模式,360度定圈,顺时针
    SERVO_MODE_360_TURN_CCW = 6, // 马达模式,360度定圈,逆时针
    SERVO_MODE_360_TIME_CW = 7,  // 马达模式,360度定时,顺时针
    SERVO_MODE_360_TIME_CCW = 8  // 马达模式,360度定时,逆时针
} ServoMode;

// 舵机启动模式
typedef enum {
    SERVO_STARTUP_TO_MIDDLE = 1,  // 开机旋转到启动位置
    SERVO_STARTUP_KEEP_POS = 2,   // 开机保持当前位置
    SERVO_STARTUP_RELEASE = 3     // 开机无力
} ServoStartupMode;

// 波特率选项
typedef enum {
    SERVO_BAUD_9600 = 1,
    SERVO_BAUD_19200 = 2,
    SERVO_BAUD_38400 = 3,
    SERVO_BAUD_57600 = 4,
    SERVO_BAUD_115200 = 5,
    SERVO_BAUD_128000 = 6,
    SERVO_BAUD_256000 = 7,
    SERVO_BAUD_1000000 = 8
} ServoBaudRate;

// 舵机通信状态
typedef enum {
    SERVO_STATE_IDLE,        // 空闲
    SERVO_STATE_SENDING,     // 正在发送
    SERVO_STATE_WAITING,     // 等待响应
    SERVO_STATE_RECEIVED,    // 已接收响应
    SERVO_STATE_TIMEOUT      // 超时
} ServoState;

// 舵机通信管理结构体
typedef struct {
    ServoState state;                          // 当前通信状态
    uint8_t txBuffer[UART_TX_BUFFER_SIZE];     // 发送缓冲区
    uint8_t rxBuffer[UART_RX_BUFFER_SIZE];     // 接收缓冲区
    uint16_t txSize;                           // 发送数据大小
    uint16_t rxSize;                           // 已接收数据大小
    uint16_t rxExpectedSize;                   // 期望接收数据大小
    uint32_t timeoutTick;                      // 超时计时器
    bool responseRequired;                     // 是否需要响应
} ServoComm_t;

static uint8_t g_current_angle = 90; // 默认初始角度为90度

// 基本控制函数
void Servo_Init(void);
void Servo_Process(void);
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t time);
void Servo_SetMultiPosition(const uint8_t* ids, const uint16_t* positions, const uint16_t* times, uint8_t count);
uint16_t Servo_ReadPosition(uint8_t id);
void Servo_Stop(uint8_t id);
void Servo_Pause(uint8_t id);
void Servo_Continue(uint8_t id);

// 配置函数
void Servo_SetID(uint8_t oldID, uint8_t newID);
uint8_t Servo_ReadID(uint8_t id);
void Servo_SetMode(uint8_t id, ServoMode mode);
ServoMode Servo_ReadMode(uint8_t id);
void Servo_SetStartupMode(uint8_t id, ServoStartupMode mode);
ServoStartupMode Servo_ReadStartupMode(uint8_t id);
void Servo_SetBaudRate(uint8_t id, ServoBaudRate baud);

// 力矩控制
void Servo_ReleaseTorque(uint8_t id);
void Servo_RestoreTorque(uint8_t id);

// 校准和限位
void Servo_Calibrate(uint8_t id);
void Servo_SetMinPosition(uint8_t id);
void Servo_SetMaxPosition(uint8_t id);

// 恢复出厂设置
void Servo_FactoryResetKeepID(uint8_t id);
void Servo_FactoryResetAll(uint8_t id);

// 状态读取
void Servo_ReadTempAndVoltage(uint8_t id, float* temp, float* voltage);
void Servo_ReadVersion(uint8_t id, char* version);

// 通信函数
void Servo_SendCommand(const char* cmd, bool needResponse);
void Servo_SendCommandRaw(const uint8_t* data, uint16_t size, bool needResponse);
bool Servo_IsIdle(void);
void Servo_UART_RxCallback(void);
void Servo_UART_TxCallback(void);

void Servo_SetAngle(uint8_t target_angle, uint32_t time_ms);

#endif /* __SERVO_H */
