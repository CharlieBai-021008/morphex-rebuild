#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <math.h>
#include <stdio.h>

// 定义PI常量
#ifndef PI
#define PI 3.141592653589793
#endif

/**
 * @brief 逆运动学计算函数
 *
 * @param x 目标点的x坐标
 * @param y 目标点的y坐标
 * @param z 目标点的z坐标
 * @param L1 机械臂第一段的长度
 * @param L2 机械臂第二段的长度
 * @param L3 机械臂第三段的长度
 * @param theta1 输出：肩关节的旋转角度（单位：度）
 * @param theta2 输出：髋关节的旋转角度（单位：度）
 * @param theta3 输出：膝关节的旋转角度（单位：度）
 *
 * @note 该函数会检查目标点是否在机械臂的工作范围内，超出范围时会打印提示。
 */
void inverse_kinematics(double x, double y, double z, double L1, double L2, double L3, double *theta1, double *theta2, double *theta3);

#endif /* __K_H */
