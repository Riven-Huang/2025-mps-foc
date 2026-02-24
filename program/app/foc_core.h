#ifndef BSP_FOC_CORE
#define BSP_FOC_CORE

// #include "driverlib.h"
// #include "device.h"
// #include "board.h"

#include <math.h>
#include <stdint.h>

// --- PI 控制器结构体 ---
typedef struct {
    float  Ref;         // 参考值 (输入)
    float  Fbk;         // 反馈值 (输入)
    float  Out;         // 输出值
    float  Kp;          // 比例
    float  Ki;          // 积分
    float  Ui;          // 积分累加项
    float  OutMax;      // 输出上限
    float  OutMin;      // 输出下限
} PID_Controller;

// --- FOC 全局状态结构体 ---
typedef struct {
    // 1. 传感器数据
    float  Ia;          // A相电流 (A)
    float  Ib;          // B相电流 (A)
    float  Ic;          // C相电流 (A)
    float  DC_Bus_V;    // 母线电压 (V)
    
    // 2. 角度与速度
    float  Theta_Elec;  // 电角度 (0 ~ 2PI)
    float  Theta_Mech;  // 机械角度 (0 ~ 2PI)
    float  Speed_RPM;   // 机械转速
    
    // 3. Clarke/Park 变换中间变量
    float  Ialpha;
    float  Ibeta;
    float  Id;
    float  Iq;
    
    // 4. 电流环设定值
    float  Id_Ref;      // 励磁电流 (通常为0)
    float  Iq_Ref;      // 转矩电流 (来自速度环)
    
    // 5. 反变换输出
    float  Vd;
    float  Vq;
    float  Valpha;
    float  Vbeta;
    
    // 6. SVPWM 占空比 (0.0 ~ 1.0)
    float  Duty_A;
    float  Duty_B;
    float  Duty_C;
    
} FOC_State;

// 函数声明
void PID_Calc(PID_Controller *pid);
void FOC_Clarke(FOC_State *foc);
void FOC_Park(FOC_State *foc);
void FOC_InvPark(FOC_State *foc);
void FOC_SVPWM(FOC_State *foc);

#endif // BSP_FOC_CORE
//
// End of File
//
