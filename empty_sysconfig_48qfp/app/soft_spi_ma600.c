#include "soft_spi_ma600.h"
#include "driverlib.h" // 必须包含以获取寄存器定义

// =============================================================
// 1. 引脚号定义 (根据你提供的)
// =============================================================
#define PIN_CS    16   // Port A
#define PIN_CLK   32   // Port B
#define PIN_MOSI  33   // Port B
#define PIN_MISO  24   // Port A

// =============================================================
// 2. 预计算掩码 (0-31位)
// =============================================================
#define MASK_CS   (1UL << (PIN_CS   % 32)) // 1 << 16
#define MASK_CLK  (1UL << (PIN_CLK  % 32)) // 1 << 0
#define MASK_MOSI (1UL << (PIN_MOSI % 32)) // 1 << 1
#define MASK_MISO (1UL << (PIN_MISO % 32)) // 1 << 24

// =============================================================
// 3. 寄存器物理地址指针 (核弹级优化，支持混合端口)
// =============================================================
// Port A (GPIO 0-31) 寄存器组指针
#define PORT_A_REGS  ((volatile uint32_t *)((uintptr_t)GPIODATA_BASE))
// Port B (GPIO 32-63) 寄存器组指针
#define PORT_B_REGS  ((volatile uint32_t *)((uintptr_t)GPIODATA_BASE) + GPIO_DATA_REGS_STEP)

// 初始化
void MA600_SoftSPI_Init(void)
{
    // 这里使用 DriverLib 函数初始化，慢一点没关系，保证配置正确
    // 确保在 SysConfig 里把这些引脚设为对应的 Input/Output
    GPIO_writePin(PIN_CS, 1);
    GPIO_writePin(PIN_CLK, 1);
    GPIO_writePin(PIN_MOSI, 0);
}

// -------------------------------------------------------------
// 极速 SPI 读取 (混合端口版)
// -------------------------------------------------------------
#pragma CODE_SECTION(MA600_ReadRaw_Fast, ".TI.ramfunc");

uint16_t MA600_ReadRaw_Fast(void)
{
    uint16_t data_in = 0;
    int i;
    
    // 1. 拉低 CS (GPIO 16 -> Port A)
    PORT_A_REGS[GPIO_GPxCLEAR_INDEX] = MASK_CS;
    
    asm(" RPT #10 || NOP"); // 稍作延时保证 CS 建立时间

    // 2. 循环 16 次
    for(i = 0; i < 16; i++)
    {
        // --- CLK 拉低 (GPIO 32 -> Port B) ---
        PORT_B_REGS[GPIO_GPxCLEAR_INDEX] = MASK_CLK;
        
        // MOSI 写 0 (GPIO 33 -> Port B)
        PORT_B_REGS[GPIO_GPxCLEAR_INDEX] = MASK_MOSI; 
        
        asm(" RPT #4 || NOP"); // 保持低电平

        // --- CLK 拉高 (GPIO 32 -> Port B) ---
        PORT_B_REGS[GPIO_GPxSET_INDEX] = MASK_CLK;
        
        data_in <<= 1;
        
        // 读取 MISO (GPIO 24 -> Port A)
        // 注意：这里读的是 Port A 的 DAT 寄存器
        if(PORT_A_REGS[GPIO_GPxDAT_INDEX] & MASK_MISO)
        {
            data_in |= 1;
        }
        
        asm(" RPT #4 || NOP"); // 保持高电平
    }

    // 3. 拉高 CS (GPIO 16 -> Port A)
    PORT_A_REGS[GPIO_GPxSET_INDEX] = MASK_CS;
    
    asm(" RPT #20 || NOP"); // CS 恢复时间

    return data_in;
}

// 封装
float MA600_ReadAngle_SoftSPI(void)
{
    uint16_t raw = MA600_ReadRaw_Fast();
    // 转换为弧度 (0 ~ 2PI)
    return (float)raw * 0.0000958738f; 
}

