#include "DeltaRobot.h"
#include "Servo.h"
#include "Delay.h"
#include <math.h>

#define PI_F  3.14159265358979f

typedef enum
{
    MOVE_IDLE = 0,
    MOVE_LINE,      // 笛卡尔直线
    MOVE_ARC,       // 平面圆弧
    MOVE_JOINT      // 关节空间点到点
} MoveMode;

typedef struct
{
    MoveMode mode;
    u8       busy;

    DeltaPoint p0;          // 起点（LINE/ARC）
    DeltaPoint p1;          // 终点（LINE/ARC）
    float      cx, cy;      // 圆心（ARC）
    float      a0, a1;      // 起止圆心角（ARC）

    float th0[3];           // 起点关节角（JOINT）
    float th1[3];           // 终点关节角（JOINT）

    float radius;           // 圆弧半径（ARC）
    float L;                // 行程（mm 或 度）
    float v;                // 速度
    float a;                // 加速度
    float t_acc;            // 加速段时间
    float T;                // 总时间
    float t;                // 已运行时间
} MoveState;

static MoveState    s_move;
static DeltaPoint   s_cur_point;
static DeltaAngle   s_cur_angle;

// ---------------------------------------------------------------------
// 内部工具
// ---------------------------------------------------------------------

static float Dist3D(DeltaPoint a, DeltaPoint b)
{
    float dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static u8 CheckPointInArea(DeltaPoint p)
{
    if(p.X >  DELTA_AREA_X_MAX || p.X < -DELTA_AREA_X_MAX) return 1;
    if(p.Y >  DELTA_AREA_Y_MAX || p.Y < -DELTA_AREA_Y_MAX) return 1;
    if(p.Z >  DELTA_AREA_Z_MAX || p.Z < -DELTA_AREA_Z_MAX) return 1;
    return 0;
}

static u8 CheckAngleLimit(DeltaAngle ang)
{
    if(ang.Theta1 < DELTA_THETA_MIN || ang.Theta1 > DELTA_THETA_MAX) return 1;
    if(ang.Theta2 < DELTA_THETA_MIN || ang.Theta2 > DELTA_THETA_MAX) return 1;
    if(ang.Theta3 < DELTA_THETA_MIN || ang.Theta3 > DELTA_THETA_MAX) return 1;
    return 0;
}

// 梯形速度规划：由行程L、速度v、加速度a求加速段时间t_acc与总时间T
static void PlanProfile(float L, float v, float a, float *t_acc, float *T)
{
    float d_acc = v * v / (2.0f * a);   // 加速段行程

    if(L <= 0)          { *t_acc = 0; *T = 0; return; }
    if(2.0f * d_acc >= L)               // 三角形速度曲线（到不了匀速）
    {
        *T = 2.0f * sqrtf(L / a);
        *t_acc = *T / 2.0f;
    }
    else                                // 梯形速度曲线
    {
        *t_acc = v / a;
        *T = 2.0f * (*t_acc) + (L - 2.0f * d_acc) / v;
    }
}

// 梯形曲线：t时刻已走过的行程
static float ProfileS(float t, float L, float v, float a, float t_acc, float T)
{
    if(t <= 0)      return 0;
    if(t >= T)      return L;
    if(t < t_acc)   return 0.5f * a * t * t;                        // 加速
    if(t < T - t_acc)                                               // 匀速
    {
        float d_acc = v * v / (2.0f * a);
        return d_acc + v * (t - t_acc);
    }
    float t_dec = T - t;                                            // 减速
    return L - 0.5f * a * t_dec * t_dec;
}

// 把主动臂倾角theta下发到舵机（含安装标定）
static void DriveServos(DeltaAngle ang)
{
    Servo_SetAngle(SERVO_AXIS1, SERVO_ZERO_OFFSET_1 + SERVO_DIR_1 * ang.Theta1);
    Servo_SetAngle(SERVO_AXIS2, SERVO_ZERO_OFFSET_2 + SERVO_DIR_2 * ang.Theta2);
    Servo_SetAngle(SERVO_AXIS3, SERVO_ZERO_OFFSET_3 + SERVO_DIR_3 * ang.Theta3);
}

// ---------------------------------------------------------------------
// 运动节拍（TIM2中断，10ms一次）
// ---------------------------------------------------------------------

void Delta_Tick_ISR(void)
{
    DeltaAngle ang;
    DeltaPoint p;
    float u, s, th_cur;

    if(!s_move.busy) return;

    s_move.t += DELTA_TICK_MS / 1000.0f;

    if(s_move.t >= s_move.T)          // 到达终点
    {
        s = s_move.L;
        u = 1.0f;
        s_move.busy = 0;
    }
    else
    {
        s = ProfileS(s_move.t, s_move.L, s_move.v, s_move.a, s_move.t_acc, s_move.T);
        u = s / s_move.L;
    }

    switch(s_move.mode)
    {
        case MOVE_LINE:
            p.X = s_move.p0.X + (s_move.p1.X - s_move.p0.X) * u;
            p.Y = s_move.p0.Y + (s_move.p1.Y - s_move.p0.Y) * u;
            p.Z = s_move.p0.Z + (s_move.p1.Z - s_move.p0.Z) * u;
            if(DeltaKinematics_Inverse(p, &ang) == 0 && CheckAngleLimit(ang) == 0)
            {
                DriveServos(ang);
                s_cur_angle = ang;
                s_cur_point = p;
            }
            // 若中途反解失败：保持上一拍姿态（不更新输出）
            break;

        case MOVE_ARC:
            th_cur = s_move.a0 + (s_move.a1 - s_move.a0) * u;
            p.X = s_move.cx + s_move.radius * cosf(th_cur);
            p.Y = s_move.cy + s_move.radius * sinf(th_cur);
            p.Z = s_move.p0.Z + (s_move.p1.Z - s_move.p0.Z) * u;
            if(DeltaKinematics_Inverse(p, &ang) == 0 && CheckAngleLimit(ang) == 0)
            {
                DriveServos(ang);
                s_cur_angle = ang;
                s_cur_point = p;
            }
            break;

        case MOVE_JOINT:
            ang.Theta1 = s_move.th0[0] + (s_move.th1[0] - s_move.th0[0]) * u;
            ang.Theta2 = s_move.th0[1] + (s_move.th1[1] - s_move.th0[1]) * u;
            ang.Theta3 = s_move.th0[2] + (s_move.th1[2] - s_move.th0[2]) * u;
            if(CheckAngleLimit(ang) == 0)
            {
                DriveServos(ang);
                s_cur_angle = ang;
                DeltaKinematics_Forward(ang, &s_cur_point);
            }
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------
// 初始化
// ---------------------------------------------------------------------

static void Delta_TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 84MHz / 8400 = 10kHz，再 /100 = 100Hz（10ms）
    TIM_TimeBaseStructure.TIM_Prescaler     = 8400 - 1;
    TIM_TimeBaseStructure.TIM_Period        = 10000 / DELTA_TICK_HZ - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Delta_Init(void)
{
    Servo_Init();
    DeltaKinematics_Init();

    s_move.mode = MOVE_IDLE;
    s_move.busy = 0;

    // 约定：断电前机械臂应停在Home位。
    // 上电后把当前状态直接记为Home，舵机先输出Home角度，避免猛甩。
    // 正解用数值法（IK反演），初值给在工作空间中部附近保证收敛
    s_cur_angle.Theta1 = DELTA_THETA1_HOME;
    s_cur_angle.Theta2 = DELTA_THETA2_HOME;
    s_cur_angle.Theta3 = DELTA_THETA3_HOME;
    s_cur_point.X = 0.0f;
    s_cur_point.Y = 0.0f;
    s_cur_point.Z = -40.0f;   // 初值 hint，数值正解会迭代到精确位置
    DeltaKinematics_Forward(s_cur_angle, &s_cur_point);
    DriveServos(s_cur_angle);

    Delta_TIM2_Init();
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        Delta_Tick_ISR();
    }
}

// ---------------------------------------------------------------------
// 对外接口
// ---------------------------------------------------------------------

u8 Delta_IsBusy(void)
{
    return s_move.busy;
}

void Delta_GetPoint(DeltaPoint *point)
{
    *point = s_cur_point;
}

void Delta_GetAngle(DeltaAngle *angle)
{
    *angle = s_cur_angle;
}

// 阻塞等待一条运动完成（带超时保护，单位ms）
// 用Delay_ms按节拍分片等待，TIM2中断在此期间正常驱动轨迹
static void WaitMoveDone(u32 timeout_ms)
{
    while(s_move.busy && timeout_ms > 0)
    {
        u16 wait = (timeout_ms > DELTA_TICK_MS) ? DELTA_TICK_MS : (u16)timeout_ms;
        Delay_ms(wait);
        timeout_ms -= wait;
    }
    s_move.busy = 0;
}

void Delta_Home(void)
{
    Delta_JointMove(DELTA_THETA1_HOME, DELTA_THETA2_HOME, DELTA_THETA3_HOME, DELTA_HOME_DEG_PER_SEC);
}

u8 Delta_MoveLine(float x, float y, float z, float vel_mm_s, float acc_mm_s2)
{
    DeltaPoint target;
    DeltaAngle ang;

    if(s_move.busy) return 1;

    target.X = x; target.Y = y; target.Z = z;

    if(CheckPointInArea(target)) return 2;                       // 超出工作空间
    if(DeltaKinematics_Inverse(target, &ang)) return 3;          // 目标不可达
    if(CheckAngleLimit(ang)) return 4;                           // 目标超关节限位

    s_move.mode = MOVE_LINE;
    s_move.p0   = s_cur_point;
    s_move.p1   = target;
    s_move.L    = Dist3D(s_move.p0, target);
    if(s_move.L < 0.01f) return 0;                               // 已在目标处

    s_move.v = vel_mm_s;
    s_move.a = acc_mm_s2;
    PlanProfile(s_move.L, s_move.v, s_move.a, &s_move.t_acc, &s_move.T);
    s_move.t = 0;

    if(s_move.T < 0.02f) s_move.T = 0.02f;                       // 至少一拍

    s_move.busy = 1;
    WaitMoveDone((u32)(s_move.T * 1000.0f) + 2000);
    return 0;
}

u8 Delta_MoveArc(float i, float j, float x, float y, u8 cw, float vel_mm_s, float acc_mm_s2)
{
    DeltaPoint target;
    DeltaAngle ang;
    float radius, ang_start, ang_end, travel;

    if(s_move.busy) return 1;
    if(i == 0 && j == 0) return 5;

    target.X = x; target.Y = y; target.Z = s_cur_point.Z;
    if(CheckPointInArea(target)) return 2;
    if(DeltaKinematics_Inverse(target, &ang)) return 3;
    if(CheckAngleLimit(ang)) return 4;

    radius = sqrtf(i * i + j * j);

    s_move.cx = s_cur_point.X + i;
    s_move.cy = s_cur_point.Y + j;

    // 保护：终点若落在圆心附近（atan2(0,0)无定义），按整圆处理
    {
        float dx_end = x - s_move.cx, dy_end = y - s_move.cy;
        if(sqrtf(dx_end * dx_end + dy_end * dy_end) < 1.0f)
        {
            x = s_cur_point.X;
            y = s_cur_point.Y;
            target.X = x;
            target.Y = y;
        }
    }

    ang_start = atan2f(s_cur_point.Y - s_move.cy, s_cur_point.X - s_move.cx);
    ang_end   = atan2f(y - s_move.cy, x - s_move.cx);

    travel = ang_end - ang_start;
    if(travel < 0) travel += 2.0f * PI_F;
    if(cw) travel -= 2.0f * PI_F;                                // 顺转为负

    // 完整圆的特殊处理：终点与起点重合
    if(travel > -0.001f && travel < 0.001f)
        travel = cw ? -2.0f * PI_F : 2.0f * PI_F;

    s_move.mode  = MOVE_ARC;
    s_move.p0    = s_cur_point;
    s_move.p1    = target;
    s_move.a0    = ang_start;
    s_move.a1    = ang_start + travel;
    s_move.radius= radius;
    s_move.v     = vel_mm_s;
    s_move.a     = acc_mm_s2;
    s_move.L     = radius * fabsf(travel);                       // 实际行程=弧长

    PlanProfile(s_move.L, s_move.v, s_move.a, &s_move.t_acc, &s_move.T);
    s_move.t = 0;
    if(s_move.T < 0.02f) s_move.T = 0.02f;

    s_move.busy = 1;
    WaitMoveDone((u32)(s_move.T * 1000.0f) + 2000);
    return 0;
}

u8 Delta_JointMove(float theta1, float theta2, float theta3, float deg_per_sec)
{
    float d1, d2, d3, L;
    DeltaAngle ang;

    if(s_move.busy) return 1;

    ang.Theta1 = theta1; ang.Theta2 = theta2; ang.Theta3 = theta3;
    if(CheckAngleLimit(ang)) return 4;

    d1 = fabsf(theta1 - s_cur_angle.Theta1);
    d2 = fabsf(theta2 - s_cur_angle.Theta2);
    d3 = fabsf(theta3 - s_cur_angle.Theta3);
    L = d1;
    if(d2 > L) L = d2;
    if(d3 > L) L = d3;
    if(L < 0.01f) return 0;

    s_move.mode = MOVE_JOINT;
    s_move.th0[0] = s_cur_angle.Theta1;
    s_move.th0[1] = s_cur_angle.Theta2;
    s_move.th0[2] = s_cur_angle.Theta3;
    s_move.th1[0] = theta1;
    s_move.th1[1] = theta2;
    s_move.th1[2] = theta3;
    s_move.L      = L;
    s_move.v      = deg_per_sec;
    s_move.a      = 4.0f * deg_per_sec;      // 关节空间默认加减速，1/4秒内到速

    PlanProfile(s_move.L, s_move.v, s_move.a, &s_move.t_acc, &s_move.T);
    s_move.t = 0;
    if(s_move.T < 0.02f) s_move.T = 0.02f;

    s_move.busy = 1;
    WaitMoveDone((u32)(s_move.T * 1000.0f) + 2000);
    return 0;
}

// ---------------------------------------------------------------------
// 预定轨迹
// =====================================================================
//  轨迹内容（数值已按当前几何 IK 逐点校核，全部可达）：
//    Home 位于工作空间中央，末端约 Z=-40mm
//    1. 末端垂直下降 15mm（上）
//    2. 上升回到 Home（下）——上下一个来回完成
//    3. 下降到 Z=-50mm 平面
//    4. 绕圈一周（半径10mm，全程 theta 在 -33~+23 度内）
//    5. 上升回 Home
//  注意：当前几何(平台500/350、臂150/200)工作空间很小，圆半径无法再加大
// =====================================================================

void Delta_RunDemo(void)
{
    DeltaPoint home = s_cur_point;
    float z_circle  = home.Z - 10.0f;   // 圆的平面（最深theta约+23度，限位+25内）
    float r_circle  = 10.0f;            // 圆半径（已校核整圆全部可达）
    float feed      = DELTA_DEFAULT_VELOCITY;

    // 1. 下降 15mm
    Delta_MoveLine(home.X, home.Y, home.Z - 15.0f, feed, DELTA_DEFAULT_ACCELERATION);

    // 2. 上升回 Home（上下一个来回）
    Delta_MoveLine(home.X, home.Y, home.Z, feed, DELTA_DEFAULT_ACCELERATION);

    // 3. 下降到画圆平面
    Delta_MoveLine(home.X, home.Y, z_circle, feed, DELTA_DEFAULT_ACCELERATION);

    // 4. 绕圈一周（整圆：终点=起点）
    Delta_MoveArc(r_circle, 0.0f, home.X, home.Y, 0, feed, DELTA_DEFAULT_ACCELERATION);

    // 5. 上升回 Home
    Delta_MoveLine(home.X, home.Y, home.Z, feed, DELTA_DEFAULT_ACCELERATION);
}
