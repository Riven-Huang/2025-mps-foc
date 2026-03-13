//#############################################################################
// FILE:   foc_main.c
// TITLE:  FOC Main Control Loop
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"

// user included Files
#include "foc_core.h"
#include "soft_spi_ma600.h"
#include <math.h>

// --- 宏定义 ---
#define PI              3.1415926535f
#define TWO_PI          6.283185307f
#define POLE_PAIRS      14.0f   // 极对数
#define PWM_PERIOD      2500    // 20kHz
#define CURRENT_SCALE   0.012207f

#define VOLTAGE_LIMIT   6.0f

// --- 全局控制对象 ---
FOC_State       foc;
PID_Controller  pid_id;
PID_Controller  pid_iq;
PID_Controller  pid_spd;
PID_Controller  pid_pos;

// --- 状态机 ---
enum {
    STATE_CALIB_CURRENT = 0,
    STATE_ALIGN_ENCODER,
    STATE_CLOSED_LOOP,
    STATE_ERROR
} Sys_State = STATE_CALIB_CURRENT;

// --- 控制模式定义 ---
enum {
    CTRL_MODE_CURRENT = 0, // 纯力矩模式
    CTRL_MODE_SPEED,       // 速度模式
    CTRL_MODE_POSITION     // 位置模式
};

// --- 变量 ---
int   calib_cnt = 0;
float offset_Ia = 0.0f, offset_Ib = 0.0f;
float align_angle_offset = 0.0f; // 机械角度零点偏差
uint16_t loop_cnt = 0;           // 分频计数器

// --- 调试与观测变量 (全局) ---
uint16_t Ia_Raw = 0; // 定义在全局，方便在 Expressions 窗口观察
uint16_t Ib_Raw = 0;
uint16_t Ic_Raw = 0; 
uint16_t Vbus_Raw = 0;
float    raw_theta_mech = 0.0f;
float    MA600_dir = -1.0f;
static float old_theta_mech = 0.0f; // 用于测速

// --- 电机参数 ---
float control_freq = 20000;
float singlephase_R = 1.45f/2.0f;
float singlephase_L = 0.823f/2.0f/1000;

// --- 用户控制接口 ---
volatile int Control_Mode = CTRL_MODE_SPEED; 
volatile float User_Iq_Ref = 0.0f;    // Current Ref
volatile float User_Speed_Ref = 0.0f; // Speed Ref (RPM)
volatile float User_Pos_Ref = 0.0f;   // Position Ref (Rad)
volatile uint8_t Run_Flag = 1;  
volatile float Speed_Filter_K = 0.002f; 

float Speed_RPM_Raw = 0.0f;
float test_vq = 0;

    float Id_disp, Iq_disp; 

//
// Main
//
void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();
    C2000Ware_libraries_init();

    // --- user init ---
    MA600_SoftSPI_Init();

    // --- PID 参数初始化 ---
    // 1. 电流环 (10khz)
    pid_id.Kp = 0.00f;  pid_id.Ki = 0.0f; pid_id.OutMax = 5.0f; pid_id.OutMin = -5.0f; // 输出电压Vd,暂时未用
    pid_iq.Kp = 0.00f;  pid_iq.Ki = 0.0f; pid_iq.OutMax = 5.0f; pid_iq.OutMin = -5.0f; // 输出电压Vq
    // 2. 速度环 (2khz)
    pid_spd.Kp = 0.1f; pid_spd.Ki = 0.005f; pid_spd.OutMax = VOLTAGE_LIMIT; pid_spd.OutMin = -VOLTAGE_LIMIT; // 输出电流Iq
    // 3. 位置环 (1khz)
    pid_pos.Kp = 20.0f;  pid_pos.Ki = 0.0f;   pid_pos.OutMax = 500.0f; pid_pos.OutMin = -500.0f; // 输出速度RPM

    // 暂时先随便给一个，后续改成adc校准的
    foc.DC_Bus_V = 31.5f;

    // start the system
    GPIO_writePin(mp6539_nSLEEP,1);
    DEVICE_DELAY_US(500000);
    
    // init complete
    EINT;
    ERTM;

    //
    // IDLE loop. Just sit and loop forever (optional):
    //
    for(;;)
    {
        // --- screen ---
        GPIO_togglePin(LED1);
        DEVICE_DELAY_US(500000);
        // Enter code here
    }
}

// ADC 中断服务函数 (High Frequency Loop - e.g., 20kHz)
__interrupt void INT_IA_IC_ADCA_INT1_ISR(void) 
{

    // 1. 快速采样 (First-level)
    Ia_Raw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0); 
    Ib_Raw = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
    raw_theta_mech = MA600_dir * MA600_ReadAngle_SoftSPI(); 
    Vbus_Raw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
    Ic_Raw = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);

    switch(Sys_State)
    {
        // --- 状态0: 电流零点校准 ---
        case STATE_CALIB_CURRENT:
            offset_Ia += Ia_Raw;
            offset_Ib += Ib_Raw;
            calib_cnt++;

            // 关断输出
            foc.Duty_A = 0.0f; foc.Duty_B = 0.0f; foc.Duty_C = 0.0f;

            if(calib_cnt >= 1000) {
                offset_Ia /= 1000.0f;
                offset_Ib /= 1000.0f;
                calib_cnt = 0;
                Sys_State = STATE_ALIGN_ENCODER; 
            }
            break;

        // --- 状态1: 编码器对齐 ---
        case STATE_ALIGN_ENCODER:
            // 强拖到电角度 0 度
            foc.Duty_A = 0.6f; 
            foc.Duty_B = 0.4f;
            foc.Duty_C = 0.4f;

            calib_cnt++;
            if(calib_cnt >= 20000) { // 1秒
                align_angle_offset = raw_theta_mech; 
                Sys_State = STATE_CLOSED_LOOP; 
            }
            break;

        // --- 状态2: 闭环控制 ---
        case STATE_CLOSED_LOOP:
            // [数据清洗]
            foc.Ia = ((float)Ia_Raw - offset_Ia) * CURRENT_SCALE;
            foc.Ib = -((float)Ib_Raw - offset_Ib) * CURRENT_SCALE; // ！！！电流定义方向一定要一样 ！！！

            // [角度处理]
            foc.Theta_Mech = raw_theta_mech - align_angle_offset;
            if(foc.Theta_Mech < 0) foc.Theta_Mech += TWO_PI; // 归一化到 0-2PI
            
            foc.Theta_Elec = foc.Theta_Mech * POLE_PAIRS;
            while(foc.Theta_Elec > TWO_PI) foc.Theta_Elec -= TWO_PI;
            while(foc.Theta_Elec < 0.0f)   foc.Theta_Elec += TWO_PI;

            // [速度计算] (简单微分)
            float diff = foc.Theta_Mech - old_theta_mech;
            if(diff > 3.14159f) diff -= 6.28f;       // 过零点处理
            else if(diff < -3.14159f) diff += 6.28f;
            
            // Speed (RPM) = diff(rad) * Fs * 60 / 2PI
            // Fs = 20000Hz.  K = 20000 * 9.549 = 190985.9
            Speed_RPM_Raw = diff * 190985.9f; 
            foc.Speed_RPM = (1.0f - Speed_Filter_K) * foc.Speed_RPM + (Speed_Filter_K * Speed_RPM_Raw);
            old_theta_mech = foc.Theta_Mech;

            FOC_Clarke(&foc);
            FOC_Park(&foc);

             // 简单的滤波用于显示
            Id_disp = 0.95f * Id_disp + 0.05f * foc.Id;
            Iq_disp = 0.95f * Iq_disp + 0.05f * foc.Iq;

            // [三环控制逻辑]
            loop_cnt++;if(loop_cnt>30)loop_cnt=0;

            float Vq_Target = 0.0f;

            switch(Control_Mode)
            {
                // --- 模式A: 位置模式 ---
                case CTRL_MODE_POSITION:
                    // 降频 20:1 (1kHz)
                    if(loop_cnt >= 20) 
                    {
                        loop_cnt = 0; // 复位计数
                        pid_pos.Ref = User_Pos_Ref;
                        pid_pos.Fbk = foc.Theta_Mech;
                        PID_Calc(&pid_pos);
                        
                        // 位置环输出 -> 速度环输入
                        pid_spd.Ref = pid_pos.Out; 
                        
                        // 级联执行速度环
                        pid_spd.Fbk = foc.Speed_RPM;
                        PID_Calc(&pid_spd);
                        foc.Iq_Ref = pid_spd.Out;
                    }
                    foc.Id_Ref = 0.0f; 
                    Vq_Target = pid_spd.Out; 
                    break;

                // --- 模式B: 速度模式 ---
                case CTRL_MODE_SPEED:
                    // 降频 10:1 (2kHz)
                    // 注意：这里使用取模运算或单独的计数逻辑，避免冲突
                    // 为了简单，直接复用 loop_cnt，但要小心位置模式切换过来时的初值
                    if(loop_cnt >= 10) 
                    {
                        loop_cnt = 0; 
                        
                        pid_spd.Ref = User_Speed_Ref; 
                        pid_spd.Fbk = foc.Speed_RPM;
                        PID_Calc(&pid_spd);         
                        foc.Iq_Ref = pid_spd.Out;
                    }
                    foc.Id_Ref = 0.0f; 
                    Vq_Target = pid_spd.Out;
                    break;

                // --- 模式C: 力矩模式 ---
                case CTRL_MODE_CURRENT:
                    // [修改] 此时变成了 "电压开环模式"
                    // User_Iq_Ref 现在被当作 Voltage Ref 使用
                    Vq_Target = User_Iq_Ref; 
                    break;   
            }

            if(Vq_Target > VOLTAGE_LIMIT) Vq_Target = VOLTAGE_LIMIT;
            if(Vq_Target < -VOLTAGE_LIMIT) Vq_Target = -VOLTAGE_LIMIT;

            foc.Vd = 0.0f;
            foc.Vq = Vq_Target;

            FOC_InvPark(&foc);
            FOC_SVPWM(&foc);
            break;
    }

    if(Run_Flag == 1)
    {
        EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(foc.Duty_A * PWM_PERIOD));
        EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(foc.Duty_B * PWM_PERIOD));
        EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(foc.Duty_C * PWM_PERIOD));
    }else {
        EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
        EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, 0);
        EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 0);
    }


    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


//
// End of File
//
