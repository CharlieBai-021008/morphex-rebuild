/**
 * @file robot_initialization.h
 * @brief 六足/球形变形机器人的舵机初始化和测试函数
 *
 * 该头文件提供了初始化和测试变形机器人全部31个舵机的函数声明。
 * 包括六足部分18个舵机(6条腿×3个关节)、球形模式12个舵机(外壳切片)和中央控制舵机(1个)。
 */

#ifndef ROBOT_INITIALIZATION_H
#define ROBOT_INITIALIZATION_H

#ifdef __cplusplus
extern "C" {
#endif

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
void InitializeAllServos(void);

/**
 * @brief 验证所有舵机功能
 *
 * 该函数按顺序测试所有31个舵机，每个舵机做小幅度运动
 * 用于检查所有舵机是否正常工作
 */
void TestAllServos(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_INITIALIZATION_H */
