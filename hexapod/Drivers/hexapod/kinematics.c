#include <stdio.h>
#include <math.h>

// 定义PI值
#define PI 3.141592653589793

// 定义逆运动学函数
void inverse_kinematics(double x, double y, double z, double L1, double L2, double L3, double *theta1, double *theta2, double *theta3) {
    // 计算肩关节（水平面上的旋转角度）
    *theta1 = atan2(y, x);

    // 计算投影到x-z平面的距离
    double d = sqrt(x * x + y * y) - L1;

    // 计算从肩到目标点的距离
    double D = sqrt(d * d + z * z);

    // 判断是否超出工作范围
    if (D > (L2 + L3) || D < fabs(L2 - L3)) {
        printf("目标点超出工作范围！\n");
        return;
    }

    // 使用余弦定理计算膝关节角度
    double cos_theta3 = (L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3);
    *theta3 = acos(cos_theta3); // 膝关节角度

    // 使用余弦定理计算髋关节角度的一部分
    double cos_phi = (L2 * L2 + D * D - L3 * L3) / (2 * L2 * D);
    double phi = acos(cos_phi);

    // 计算髋关节到目标点的角度
    double alpha = atan2(z, d);

    // 髋关节角度
    *theta2 = phi + alpha;

    // 转换为角度制（如果需要）
    *theta1 *= 180.0 / PI;
    *theta2 *= 180.0 / PI;
    *theta3 *= 180.0 / PI;
}

//int main() {
//    // 输入目标点坐标和关节长度
//    double x=100, y=100, z=-60;
//
//
//    double L1 = 61.86; // 第一段长度
//    double L2 = 87.02; // 第二段长度
//    double L3 = 117.1; // 第三段长度
//
//    // 定义输出关节角度
//    double theta1, theta2, theta3;
//
//    // 调用逆运动学函数
//    inverse_kinematics(x, y, z, L1, L2, L3, &theta1, &theta2, &theta3);
//
//    // 输出结果
//    printf("关节角度:\n");
//    printf("theta1 (肩关节) = %.2f°\n", theta1);
//    printf("theta2 (髋关节) = %.2f°\n", theta2);
//    printf("theta3 (膝关节) = %.2f°\n", theta3);
//
//    return 0;
//}
