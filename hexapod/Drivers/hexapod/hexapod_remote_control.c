/*
*
 * @file hexapod_remote_control.c
 * @brief 六足/球形机器人遥控控制实现
 * 使用SBUS接收机接收遥控器信号，并通过中断方式处理
 * 增加了球形模式的控制功能（24舵机版本）
 * 支持带误差容忍的三位拨档开关控制模式
*/


#include "hexapod_remote_control.h"
#include "hexapod_control.h"
#include "sphere_control.h"
#include <stdlib.h>

/* 外部定义*/
extern UART_HandleTypeDef huart6;  // 假设UART6用于SBUS接收
extern SBUS_CH_Struct SBUS_CH;     // SBUS通道结构体

/* 变量*/
static uint8_t sbus_rx_buffer[SBUS_BUFFER_SIZE];    // SBUS接收缓冲区
static uint8_t sbus_frame_buffer[SBUS_BUFFER_SIZE]; // SBUS帧缓冲区
static bool sbus_frame_ready = false;               // SBUS帧准备好标志
static bool remote_control_enabled = true;          // 遥控控制启用标志
static uint32_t last_control_time = 0;              // 上次控制时间
static uint8_t lastModePosition = 0;                // 上次三位拨档位置 (0-低, 1-中, 2-高)

// 三位拨档开关位置定义
#define SWITCH_LOW     0
#define SWITCH_MIDDLE  1
#define SWITCH_HIGH    2

/*
*
 * @brief 获取三位拨档开关位置
 * @param value SBUS通道值
 * @return 开关位置(0-低, 1-中, 2-高)
*/

static uint8_t GetSwitchPosition(uint16_t value) {
    // 针对三位拨档开关，大致将整个范围分成三段
    // 通常SBUS范围是172-1811

    // 定义各位置的中心值
    const uint16_t LOW_CENTER = 350;     // 低位中心值
    const uint16_t MIDDLE_CENTER = 1000;  // 中位中心值
    const uint16_t HIGH_CENTER = 1650;    // 高位中心值

    // 定义误差范围
    const uint16_t ERROR_MARGIN = 150;    // 误差范围

    // 根据与中心值的接近程度判断位置
    if (abs((int)value - LOW_CENTER) < ERROR_MARGIN) {
        return SWITCH_LOW;
    } else if (abs((int)value - MIDDLE_CENTER) < ERROR_MARGIN) {
        return SWITCH_MIDDLE;
    } else if (abs((int)value - HIGH_CENTER) < ERROR_MARGIN) {
        return SWITCH_HIGH;
    } else {
        // 如果不在任何误差范围内，根据距离最近的中心值确定位置
        int distLow = abs((int)value - LOW_CENTER);
        int distMiddle = abs((int)value - MIDDLE_CENTER);
        int distHigh = abs((int)value - HIGH_CENTER);

        if (distLow <= distMiddle && distLow <= distHigh) {
            return SWITCH_LOW;
        } else if (distMiddle <= distLow && distMiddle <= distHigh) {
            return SWITCH_MIDDLE;
        } else {
            return SWITCH_HIGH;
        }
    }
}

/**
 * @brief 遥控控制初始化 (修改版本)
 */
void RemoteControl_Init(void) {
    // 以中断方式开始接收SBUS数据
    HAL_UART_Receive_IT(&huart6, sbus_rx_buffer, SBUS_BUFFER_SIZE);

    printf("遥控控制初始化完成\r\n");
    printf("左摇杆: 前/后/左/右移动\r\n");
    printf("右摇杆: 左/右旋转/伸展\r\n");
    printf("通道5: 舵机运动时间\r\n");
    printf("通道6 (三位拨档): 模式选择\r\n");
    printf("  - 低位: 展示模式 (机器人展示动作序列)\r\n");
    printf("  - 中位: 六足模式 (传统六足行走)\r\n");
    printf("  - 高位: 球形模式 (球形滚动)\r\n");
}

/**
 * @brief UART接收完成回调函数
 * @param huart UART句柄*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART2) // 如果是 UART2 中断
    {
        Servo_UART_RxCallback(); // 调用舵机驱动接收回调
    }
    else if(huart->Instance == USART3) // 如果是 UART3 中断
    {
        Servo2_RxCallback(); // 调用第2组舵机驱动接收回调
    }
    else if (huart->Instance == huart6.Instance) {
        // 复制接收到的数据到帧缓冲区
        memcpy(sbus_frame_buffer, sbus_rx_buffer, SBUS_BUFFER_SIZE);
        sbus_frame_ready = true;

        // 重新开始接收下一帧数据
        HAL_UART_Receive_IT(&huart6, sbus_rx_buffer, SBUS_BUFFER_SIZE);
    }
}

// UART 发送完成中断回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) // 如果是 UART2 中断
    {
        Servo_UART_TxCallback(); // 调用舵机驱动发送回调
    }
}

/**
 * @brief 处理遥控器控制信号 (修改版本)
 */
void RemoteControl_Process(void) {
    uint32_t current_time = HAL_GetTick();

    // 检查是否有新的SBUS帧
    if (sbus_frame_ready) {
        // 更新SBUS通道值
        update_sbus(sbus_frame_buffer);
        sbus_frame_ready = false;

        // 每隔一定时间处理控制命令，避免过于频繁的控制更新
        if (current_time - last_control_time >= CONTROL_UPDATE_PERIOD) {
            last_control_time = current_time;

            // 检查遥控器是否连接
            if (SBUS_CH.ConnectState) {
                // 计算舵机运动时间（通道5控制）
                uint16_t servo_move_time = (uint16_t)sbus_to_Range(SBUS_CH.CH5, 50, 200);
                SERVO_MOVE_TIME = servo_move_time;

                // 获取三位拨档开关位置（通道6）
                uint8_t currentModePosition = GetSwitchPosition(SBUS_CH.CH6);

                // 标记是否发生了模式切换
                bool modeChanged = false;

                // 处理模式切换和变形（通过通道6的三位拨档触发）
                // 只在拨档位置变化时触发变形
                if (currentModePosition != lastModePosition) {
                    // 当前模式
                    SphereState currentState = Sphere_GetState();

                    // 根据拨档位置和当前状态决定动作
                    switch (currentModePosition) {
                        case SWITCH_LOW: // 低位 - 展示模式
                            if (currentState != SPHERE_STATE_DISPLAY) {
                                printf("触发: 切换到展示模式\r\n");
                                Sphere_TransformToDisplay();
                                modeChanged = true;
                            }
                            break;

                        case SWITCH_MIDDLE: // 中位 - 六足模式
                            if (currentState != SPHERE_STATE_HEXAPOD) {
                                printf("触发: 切换到六足模式\r\n");
                                Sphere_TransformToHexapod();
                                modeChanged = true;
                            }
                            break;

                        case SWITCH_HIGH: // 高位 - 球形模式
                            if (currentState != SPHERE_STATE_SPHERE) {
                                printf("触发: 切换到球形模式\r\n");
                                Sphere_TransformToSphere();
                                modeChanged = true;
                            }
                            break;
                    }

                    // 更新模式位置
                    lastModePosition = currentModePosition;
                }

                // 获取摇杆输入
                int16_t y_stick = SBUS_CH.CH3 - STICK_MIDDLE;  // 前后方向
                int16_t x_stick = SBUS_CH.CH4 - STICK_MIDDLE;  // 左右方向
                int16_t rot_stick = SBUS_CH.CH1 - STICK_MIDDLE;  // 旋转方向

                // 根据当前模式和摇杆输入执行相应动作
                SphereState currentState = Sphere_GetState();

                // 仅在变形完成状态下处理运动控制，且没有发生模式切换
                if (currentState != SPHERE_STATE_TRANSFORMING && !modeChanged) {
                    // 六足模式控制逻辑
                    if (currentState == SPHERE_STATE_HEXAPOD) {
                        // 前后移动优先
                        if (abs(y_stick) > STICK_DEADZONE && abs(x_stick) <= STICK_DEADZONE && abs(rot_stick) <= STICK_DEADZONE) {
                            if (y_stick > 0) {
                                printf("六足模式: 前进\r\n");
                                TripleStepForward();
                            } else {
                                printf("六足模式: 后退\r\n");
                                TripleStepBackward();
                            }
                        }
                        // 左右平移
                        else if (abs(x_stick) > STICK_DEADZONE && abs(y_stick) <= STICK_DEADZONE && abs(rot_stick) <= STICK_DEADZONE) {
                            if (x_stick > 0) {
                                printf("六足模式: 右移\r\n");
                                TripleStepRight();
                            } else {
                                printf("六足模式: 左移\r\n");
                                TripleStepLeft();
                            }
                        }
                        // 旋转
                        else if (abs(rot_stick) > STICK_DEADZONE && abs(x_stick) <= STICK_DEADZONE && abs(y_stick) <= STICK_DEADZONE) {
                            if (rot_stick > 0) {
                                printf("六足模式: 顺时针旋转\r\n");
                                TripleStepRotateClockwise();
                            } else {
                                printf("六足模式: 逆时针旋转\r\n");
                                TripleStepRotateCounterClockwise();
                            }
                        }
                    }
                    // 球形模式控制逻辑
                    else if (currentState == SPHERE_STATE_SPHERE) {
                        // 计算滚动速度，基于摇杆偏移量
                        uint16_t speed = 0;

                        // 前后滚动优先（左摇杆上下）
                        if (abs(y_stick) > STICK_DEADZONE && abs(x_stick) <= STICK_DEADZONE && abs(rot_stick) <= STICK_DEADZONE) {
                            speed = (abs(y_stick) * 100) / (2000 - STICK_DEADZONE);
                            if (speed > 100) speed = 100;

                            if (y_stick > 0) {
                                printf("球形模式: 前滚, 速度: %d%%\r\n", speed);
                                Sphere_Roll(SPHERE_MOVE_FORWARD, speed);
                            } else {
                                printf("球形模式: 后滚, 速度: %d%%\r\n", speed);
                                Sphere_Roll(SPHERE_MOVE_BACKWARD, speed);
                            }
                        }
                        // 左右滚动（左摇杆左右）
                        else if (abs(x_stick) > STICK_DEADZONE && abs(y_stick) <= STICK_DEADZONE && abs(rot_stick) <= STICK_DEADZONE) {
                            speed = (abs(x_stick) * 100) / (2000 - STICK_DEADZONE);
                            if (speed > 100) speed = 100;

                            if (x_stick > 0) {
                                printf("球形模式: 右滚, 速度: %d%%\r\n", speed);
                                Sphere_Roll(SPHERE_MOVE_RIGHT, speed);
                            } else {
                                printf("球形模式: 左滚, 速度: %d%%\r\n", speed);
                                Sphere_Roll(SPHERE_MOVE_LEFT, speed);
                            }
                        }
                        // 左右伸展（右摇杆左右）
                        else if (abs(rot_stick) > STICK_DEADZONE && abs(x_stick) <= STICK_DEADZONE && abs(y_stick) <= STICK_DEADZONE) {
                            speed = (abs(rot_stick) * 100) / (2000 - STICK_DEADZONE);
                            if (speed > 100) speed = 100;

                            if (rot_stick > 0) {
                                printf("球形模式: 向右伸展, 速度: %d%%\r\n", speed);
                                Sphere_Roll(SPHERE_MOVE_STRETCH_RIGHT, speed);
                            } else {
                                printf("球形模式: 向左伸展, 速度: %d%%\r\n", speed);
                                Sphere_Roll(SPHERE_MOVE_STRETCH_LEFT, speed);
                            }
                        }
                        // 摇杆回中则停止
                        else if (abs(x_stick) <= STICK_DEADZONE && abs(y_stick) <= STICK_DEADZONE && abs(rot_stick) <= STICK_DEADZONE) {
                            Sphere_Roll(SPHERE_MOVE_STOP, 0);
                        }
                    }
                    // 展示模式控制逻辑 (修改版本 - 只有在没有模式切换时才处理)
                    else if (currentState == SPHERE_STATE_DISPLAY) {
                        // 在展示模式下，只有当摇杆输入足够大且没有发生模式切换时，才触发重新播放展示序列
                        if (abs(y_stick) > STICK_DEADZONE*2 || abs(x_stick) > STICK_DEADZONE*2 || abs(rot_stick) > STICK_DEADZONE*2) {
                            printf("展示模式: 重新播放展示序列\r\n");
                            Sphere_TransformToDisplay(); // 重新播放展示序列
                        }
                    }
                }

                // 打印通道值用于调试（每秒一次）
                static uint32_t lastPrintTime = 0;
                if (current_time - lastPrintTime >= 1000) {
                    lastPrintTime = current_time;
                    printf("SBUS CH1:%d CH2:%d CH3:%d CH4:%d CH5:%d CH6:%d\r\n",
                           SBUS_CH.CH1, SBUS_CH.CH2, SBUS_CH.CH3, SBUS_CH.CH4, SBUS_CH.CH5, SBUS_CH.CH6);

                    // 显示当前模式
                    const char* modeNames[] = {"展示模式", "六足模式", "球形模式"};
                    printf("当前拨档: %s\r\n", modeNames[currentModePosition]);

                    // 显示当前状态
                    const char* stateNames[] = {"空闲", "变形中", "球形", "六足", "展示"};
                    printf("当前状态: %s\r\n", stateNames[Sphere_GetState()]);
                }
            } else {
                // 遥控器未连接，执行失控保护
                printf("遥控器未连接，执行失控保护\r\n");
                // 停止所有运动
                if (Sphere_GetState() == SPHERE_STATE_SPHERE) {
                    Sphere_Roll(SPHERE_MOVE_STOP, 0);
                }
            }
        }
    }

    // 处理球形控制过程
    Sphere_Process();
}




/**
 * @brief 主循环中调用此函数处理遥控控制*/

void RemoteControl_MainLoop(void) {
    while (1) {
        // 处理遥控控制
        RemoteControl_Process();

        // 处理舵机通信
        Servo_Process();

        // 延时一小段时间，避免CPU占用过高
        HAL_Delay(1);
    }
}
