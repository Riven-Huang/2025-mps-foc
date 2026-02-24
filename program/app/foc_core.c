#include "foc_core.h"

// --- 增量式/位置式 PI 计算 ---
void PID_Calc(PID_Controller *pid)
{
    float error = pid->Ref - pid->Fbk;
    
    // 积分项
    pid->Ui += pid->Ki * error;
    
    // 积分抗饱和 (Anti-windup)
    if (pid->Ui > pid->OutMax) pid->Ui = pid->OutMax;
    else if (pid->Ui < pid->OutMin) pid->Ui = pid->OutMin;
    
    // 计算输出
    pid->Out = (pid->Kp * error) + pid->Ui;
    
    // 输出限幅
    if (pid->Out > pid->OutMax) pid->Out = pid->OutMax;
    else if (pid->Out < pid->OutMin) pid->Out = pid->OutMin;
}

// --- Clarke 变换 (3相 -> 2相静止) ---
void FOC_Clarke(FOC_State *foc)
{
    foc->Ialpha = foc->Ia;
    foc->Ibeta  = (foc->Ia + 2.0f * foc->Ib) * 0.577350269f; // 1/sqrt(3)
}

// --- Park 变换 (2相静止 -> 2相旋转) ---
void FOC_Park(FOC_State *foc)
{
    float s = sinf(foc->Theta_Elec);
    float c = cosf(foc->Theta_Elec);
    
    foc->Id =  foc->Ialpha * c + foc->Ibeta * s;
    foc->Iq = -foc->Ialpha * s + foc->Ibeta * c;
}

// --- 反 Park 变换 (2相旋转 -> 2相静止) ---
void FOC_InvPark(FOC_State *foc)
{
    float s = sinf(foc->Theta_Elec);
    float c = cosf(foc->Theta_Elec);
    
    foc->Valpha = foc->Vd * c - foc->Vq * s;
    foc->Vbeta  = foc->Vd * s + foc->Vq * c;
}

// --- SVPWM (简单标准版) + 反Clarke变换 ---
// 将 Valpha, Vbeta 转换为 DutyA, DutyB, DutyC
void FOC_SVPWM(FOC_State *foc)
{
    // 使用简化的 SPWM + 3次谐波注入 (等效于 SVPWM)
    // 这种方法代码最少，适合入门，效果与标准7段SVPWM几乎一样
    
    // 1. 归一化 (假设最大电压矢量是 DC_Bus / sqrt(3))
    // 系数取决于你的母线电压采样是否准确，这里简化处理，直接用 Valpha/Vbeta 算占空比
    // 假设 Vd, Vq 输出范围是 -Vbus/2 到 +Vbus/2
    
    float v_bus = foc->DC_Bus_V;
    if(v_bus < 1.0f) v_bus = 12.0f; // 防止除0，默认12V
    
    float inv_vbus = 1.0f / v_bus;
    
    // 逆 Clarke 变换得到 Va, Vb, Vc (暂存)
    float Va = foc->Valpha;
    float Vb = (-0.5f * foc->Valpha) + (0.866025f * foc->Vbeta);
    float Vc = (-0.5f * foc->Valpha) - (0.866025f * foc->Vbeta);
    
    // 3次谐波注入 (零序分量注入) -> 扩展 15% 电压利用率
    float Vmin, Vmax, Vcom;
    
    if (Va < Vb) { Vmin = Va; Vmax = Vb; } else { Vmin = Vb; Vmax = Va; }
    if (Vc < Vmin) Vmin = Vc;
    if (Vc > Vmax) Vmax = Vc;
    
    Vcom = -0.5f * (Vmax + Vmin);
    
    // 加上注入电压并归一化到 0~1
    foc->Duty_A = ((Va + Vcom) * inv_vbus) + 0.5f;
    foc->Duty_B = ((Vb + Vcom) * inv_vbus) + 0.5f;
    foc->Duty_C = ((Vc + Vcom) * inv_vbus) + 0.5f;
    
    // 最终限幅
    if(foc->Duty_A > 0.95f) foc->Duty_A = 0.95f; else if(foc->Duty_A < 0.05f) foc->Duty_A = 0.05f;
    if(foc->Duty_B > 0.95f) foc->Duty_B = 0.95f; else if(foc->Duty_B < 0.05f) foc->Duty_B = 0.05f;
    if(foc->Duty_C > 0.95f) foc->Duty_C = 0.95f; else if(foc->Duty_C < 0.05f) foc->Duty_C = 0.05f;
}

//
// End of File
//
