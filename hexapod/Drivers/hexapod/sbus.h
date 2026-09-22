#ifndef __SBUS_H
#define __SBUS_H

#include "main.h"



#define SBUS_FRAME_SIZE		25		// SBUS每帧字节数
#define SBUS_INPUT_CHANNELS	16		// SBUS通道数

// 以下参数不同的遥控器可能不一样，要核对后更改，尤其是 SBUS_CONNECT_FLAG
#define SBUS_RANGE_MIN 200.0f		// SBUS最小值
#define SBUS_RANGE_MAX 1800.0f		// SBUS最大值
#define SBUS_RANGE_MIDDLE 1000.0f	// SBUS中值
#define SBUS_CONNECT_FLAG 0x00		// SBUS连接位，乐迪遥控器是0x00表示正常连接
#define SBUS_TARGET_MIN 1000.0f		// SBUS转换为PWM，最小值1000
#define SBUS_TARGET_MAX 2000.0f		// SBUS转换为PWM，最大值2000

#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

typedef struct
{
    uint16_t signal[25];
	uint16_t CH1;//通道1数值
	uint16_t CH2;//通道2数值
	uint16_t CH3;//通道3数值
	uint16_t CH4;//通道4数值
	uint16_t CH5;//通道5数值
	uint16_t CH6;//通道6数值
    uint16_t CH7;//通道7数值
    uint16_t CH8;//通道8数值
    uint16_t CH9;//通道9数值
    uint16_t CH10;//通道10数值
	uint16_t CH11;//通道10数值
	uint16_t CH12;//通道10数值
	uint16_t CH13;//通道10数值
	uint16_t CH14;//通道10数值
	uint16_t CH15;//通道10数值
	uint16_t CH16;//通道10数值
	uint8_t ConnectState;//遥控器与接收器连接状态 0=未连接，1=正常连接
}SBUS_CH_Struct;

extern SBUS_CH_Struct SBUS_CH;
//SBUS信号解析相关函数
uint8_t update_sbus(uint8_t *buf);
uint16_t sbus_to_pwm(uint16_t sbus_value);
float sbus_to_Range(uint16_t sbus_value, float p_min, float p_max);
#endif
