#ifndef __DELTA_KINEMATICS_H__
#define __DELTA_KINEMATICS_H__

#include "stm32f4xx.h"

// =====================================================================
//  Delta 机械臂几何参数（已确认实测值，电机为 GDW G92 舵机）
//  命名约定与 Delta-X 固件保持一致：
//    RD_F  : 静平台（大平台）铰接点所在圆半径 SB     (mm)
//    RD_E  : 动平台（小平台）铰接点所在圆半径        (mm)
//    RD_RF : 主动大臂长度 L                         (mm)
//    RD_RE : 从动小臂长度 l（并联杆）                (mm)
//  注：RD_F/RD_E 采用本运动学约定的"半径参数"，若你的 500/350 指
//      的是三角板边长或直径，请按比例换算（直径则除以2）。
// =====================================================================
#define DELTA_RD_F    500.0f    // 静平台半径参数 SB
#define DELTA_RD_E    350.0f    // 动平台半径参数
#define DELTA_RD_RF   150.0f    // 主动大臂长度
#define DELTA_RD_RE   200.0f    // 从动小臂长度

// 复位（Home）时三个主动臂相对水平面的倾角，单位度，向下为负
// -5度位于当前几何的工作空间中央（末端约 Z=-40mm，Z向下为负）
// 注意：不要再沿用Delta-X的-38.5度——按当前臂长(150/200)该角度已接近浅边界奇异
#define DELTA_THETA1_HOME  -5.0f    // 实际安装后标定
#define DELTA_THETA2_HOME  -5.0f
#define DELTA_THETA3_HOME  -5.0f

// 主动臂倾角允许范围（度），用于限位保护
// 按当前参数实测工作空间：中心Z约 -30~-85mm，横向仅约+-20mm
// 画半径10mm的圆需要臂抬到水平面以上约+23度，故上限放到+25
// 【警告】theta>0表示主动臂高于水平面，上电前请手动确认机械上不会干涉！
#define DELTA_THETA_MIN    -90.0f
#define DELTA_THETA_MAX     25.0f

// 工作空间限制（mm），超出即拒绝运动，防止奇异姿态
#define DELTA_AREA_X_MAX    170.0f   // TODO: 按实际工作空间修改
#define DELTA_AREA_Y_MAX    170.0f
#define DELTA_AREA_Z_MAX    402.0f

typedef struct
{
    float X;
    float Y;
    float Z;
} DeltaPoint;

typedef struct
{
    float Theta1;
    float Theta2;
    float Theta3;
} DeltaAngle;

// 初始化（预计算常量）
void DeltaKinematics_Init(void);

// 正解：三个主动臂角度(度) -> 末端位置(mm)，Z轴向下为正方向参考 Delta-X
u8 DeltaKinematics_Forward(DeltaAngle angle, DeltaPoint *point);

// 反解：末端位置(mm) -> 三个主动臂角度(度)
u8 DeltaKinematics_Inverse(DeltaPoint point, DeltaAngle *angle);

#endif
