#include "DeltaKinematics.h"
#include <math.h>

// 移植自 Delta-X-Firmware DeltaKinematics.cpp，坐标系与公式保持一致
// Z 轴以静平台中心为原点、竖直向下为负（与参考固件相同）

#define PI_F          3.14159265358979f
#define RAD_TO_DEG_F  (180.0f / PI_F)

#define SIN120_F  0.866025403784439f
#define COS120_F  -0.5f
#define TAN30_F   0.577350269189626f

static float s_y0_, s_y1_;
static float s_RF_pow2, s_RE_pow2;

void DeltaKinematics_Init(void)
{
    s_y0_     = 0.5f * TAN30_F * DELTA_RD_E;
    s_y1_     = -0.5f * TAN30_F * DELTA_RD_F;
    s_RF_pow2 = DELTA_RD_RF * DELTA_RD_RF;
    s_RE_pow2 = DELTA_RD_RE * DELTA_RD_RE;
}

// 正解：数值法（高斯-牛顿求 IK(P)=angle 的解）
// 说明：闭式FK与IK的球交点存在分支二义性，某些几何参数下会选错根导致正逆解不自洽。
// 这里直接用IK做数值反演，从初值*point出发迭代，保证正逆解严格同一分支。
// 初值通常传入当前位置（热启动，3~5次迭代收敛）；失败时尝试若干固定初值。
u8 DeltaKinematics_Forward(DeltaAngle angle, DeltaPoint *point)
{
    static const float z_starts[4] = {0.0f, -50.0f, -75.0f, -110.0f};
    float p[3], res[3], J[3][3], delta[3];
    DeltaAngle a;
    DeltaPoint pb;
    u8 s, iter, i, j, conv;
    const float eps = 0.05f;     // 数值微分步长(mm)
    const float res_tol = 0.02f; // 收敛容差(度)

    for(s = 0; s < 5; s++)
    {
        // 选择初值：先用户传入的热启动，再尝试一组固定初值
        if(s == 0)
        {
            p[0] = point->X; p[1] = point->Y; p[2] = point->Z;
        }
        else
        {
            p[0] = 0.0f; p[1] = 0.0f; p[2] = z_starts[s - 1];
            if(s == 1) p[2] = point->Z;   // 同深度再试一次
        }

        conv = 0;
        for(iter = 0; iter < 30; iter++)
        {
            pb.X = p[0]; pb.Y = p[1]; pb.Z = p[2];
            if(DeltaKinematics_Inverse(pb, &a)) break;   // 该点不可达，放弃此初值

            res[0] = a.Theta1 - angle.Theta1;
            res[1] = a.Theta2 - angle.Theta2;
            res[2] = a.Theta3 - angle.Theta3;

            if(fabsf(res[0]) < res_tol && fabsf(res[1]) < res_tol && fabsf(res[2]) < res_tol)
            {
                conv = 1;
                break;
            }

            // 数值雅可比（列j：对第j个坐标微分）
            for(j = 0; j < 3; j++)
            {
                float pj = p[j];
                p[j] = pj + eps;
                pb.X = p[0]; pb.Y = p[1]; pb.Z = p[2];
                if(DeltaKinematics_Inverse(pb, &a)) { p[j] = pj; break; }
                if(j == 0)      { J[0][0] = (a.Theta1 - angle.Theta1 - res[0]) / eps; J[1][0] = (a.Theta2 - angle.Theta2 - res[1]) / eps; J[2][0] = (a.Theta3 - angle.Theta3 - res[2]) / eps; }
                else if(j == 1) { J[0][1] = (a.Theta1 - angle.Theta1 - res[0]) / eps; J[1][1] = (a.Theta2 - angle.Theta2 - res[1]) / eps; J[2][1] = (a.Theta3 - angle.Theta3 - res[2]) / eps; }
                else            { J[0][2] = (a.Theta1 - angle.Theta1 - res[0]) / eps; J[1][2] = (a.Theta2 - angle.Theta2 - res[1]) / eps; J[2][2] = (a.Theta3 - angle.Theta3 - res[2]) / eps; }
                p[j] = pj;
            }
            if(j < 3) break;   // 雅可比探测失败，放弃此初值

            // 解 J*delta = -res （克莱姆法则）
            {
                float m[3][3];
                float det, d0, d1, d2;

                for(i = 0; i < 3; i++)
                    for(j = 0; j < 3; j++)
                        m[i][j] = J[i][j];

                det = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
                    - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
                    + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
                if(fabsf(det) < 1e-9f) break;   // 奇异，放弃此初值

                d0 = (-res[0])*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
                   - m[0][1]*((-res[1])*m[2][2]-m[1][2]*(-res[2]))
                   + m[0][2]*((-res[1])*m[2][1]-m[1][1]*(-res[2]));
                d1 = m[0][0]*((-res[1])*m[2][2]-m[1][2]*(-res[2]))
                   - (-res[0])*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
                   + m[0][2]*(m[1][0]*(-res[2])-(-res[1])*m[2][0]);
                d2 = m[0][0]*(m[1][1]*(-res[2])-(-res[1])*m[2][1])
                   - m[0][1]*(m[1][0]*(-res[2])-(-res[1])*m[2][0])
                   + (-res[0])*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);

                delta[0] = d0 / det;
                delta[1] = d1 / det;
                delta[2] = d2 / det;
            }

            // 步长限幅，防止跳出可达域
            for(i = 0; i < 3; i++)
            {
                if(delta[i] >  30.0f) delta[i] =  30.0f;
                if(delta[i] < -30.0f) delta[i] = -30.0f;
                p[i] += delta[i];
            }
        }

        if(conv)
        {
            point->X = p[0];
            point->Y = p[1];
            point->Z = p[2];
            return 0;
        }
    }

    return 1;   // 所有初值均未收敛
}

static u8 AngleThetaCalc(float x0, float y0, float z0, float *theta)
{
    float y1 = s_y1_;
    y0 -= s_y0_;

    // z = a + b*y
    float a = (x0 * x0 + y0 * y0 + z0 * z0 + s_RF_pow2 - s_RE_pow2 - y1 * y1) / (2.0f * z0);
    float b = (y1 - y0) / z0;

    // 判别式
    float d = -(a + b * y1) * (a + b * y1) + b * b * s_RF_pow2 + s_RF_pow2;
    if(d < 0) return 1;   // 该点不可达

    float yj = (y1 - a * b - sqrtf(d)) / (b * b + 1.0f);
    float zj = a + b * yj;

    *theta = atanf(-zj / (y1 - yj)) * RAD_TO_DEG_F + ((yj > y1) ? 180.0f : 0.0f);

    return 0;
}

u8 DeltaKinematics_Inverse(DeltaPoint point, DeltaAngle *angle)
{
    if(AngleThetaCalc(point.X, point.Y, point.Z, &angle->Theta1)) return 1;
    if(AngleThetaCalc(point.X * COS120_F + point.Y * SIN120_F,
                      point.Y * COS120_F - point.X * SIN120_F,
                      point.Z, &angle->Theta2)) return 1;
    if(AngleThetaCalc(point.X * COS120_F - point.Y * SIN120_F,
                      point.Y * COS120_F + point.X * SIN120_F,
                      point.Z, &angle->Theta3)) return 1;
    return 0;
}
