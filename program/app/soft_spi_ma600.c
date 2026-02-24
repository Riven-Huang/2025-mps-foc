#include "soft_spi_ma600.h"
#include "driverlib.h"

// =============================================================
// 1. 引脚号定义 (已更新)
// =============================================================
#define PIN_CS    33   // Port B (GPIO 32-63)
#define PIN_CLK   32   // Port B (GPIO 32-63)
#define PIN_MOSI  24   // Port A (GPIO 0-31)  <-- 注意这里变了
#define PIN_MISO  16   // Port A (GPIO 0-31)  <-- 注意这里变了

// =============================================================
// 2. 预计算掩码 (0-31位)
// =============================================================
// 33 % 32 = 1 (Bit 1 of Port B)
#define MASK_CS   (1UL << (PIN_CS   % 32)) 
// 32 % 32 = 0 (Bit 0 of Port B)
#define MASK_CLK  (1UL << (PIN_CLK  % 32)) 
// 24 % 32 = 24 (Bit 24 of Port A)
#define MASK_MOSI (1UL << (PIN_MOSI % 32)) 
// 16 % 32 = 16 (Bit 16 of Port A)
#define MASK_MISO (1UL << (PIN_MISO % 32)) 

// =============================================================
// 3. 寄存器物理地址指针
// =============================================================
#define PORT_A_REGS  ((volatile uint32_t *)((uintptr_t)GPIODATA_BASE))
#define PORT_B_REGS  ((volatile uint32_t *)((uintptr_t)GPIODATA_BASE) + GPIO_DATA_REGS_STEP)

// 初始化
void MA600_SoftSPI_Init(void)
{
    // ⚠️重要：请务必确保在 main 或这里配置了正确的 GPIO 方向
    // GPIO33 (CS) -> Output
    // GPIO32 (CLK) -> Output
    // GPIO24 (MOSI) -> Output
    // GPIO16 (MISO) -> Input (建议开启上拉 PULLUP)
    
    GPIO_writePin(PIN_CS, 1);
    GPIO_writePin(PIN_CLK, 1);
    GPIO_writePin(PIN_MOSI, 0);
}

// -------------------------------------------------------------
// 极速 SPI 读取 (已修正端口归属)
// -------------------------------------------------------------
#pragma CODE_SECTION(MA600_ReadRaw_Fast, ".TI.ramfunc");

uint16_t MA600_ReadRaw_Fast(void)
{
    uint16_t data_in = 0;
    int i;
    
    // 1. 拉低 CS (GPIO 33 -> Port B)
    // 【修改点】原先是 PORT_A_REGS，现在 CS 在 Port B
    PORT_B_REGS[GPIO_GPxCLEAR_INDEX] = MASK_CS;
    
    asm(" RPT #10 || NOP"); 

    // 2. 循环 16 次
    for(i = 0; i < 16; i++)
    {
        // --- CLK 拉低 (GPIO 32 -> Port B) ---
        // CLK 还在 Port B，保持不变
        PORT_B_REGS[GPIO_GPxCLEAR_INDEX] = MASK_CLK;
        
        // MOSI 写 0 (GPIO 24 -> Port A)
        // 【修改点】原先是 PORT_B_REGS，现在 MOSI 在 Port A
        PORT_A_REGS[GPIO_GPxCLEAR_INDEX] = MASK_MOSI; 
        
        asm(" RPT #4 || NOP"); 

        // --- CLK 拉高 (GPIO 32 -> Port B) ---
        PORT_B_REGS[GPIO_GPxSET_INDEX] = MASK_CLK;
        
        data_in <<= 1;
        
        // 读取 MISO (GPIO 16 -> Port A)
        // MISO 还在 Port A (16)，保持 PORT_A_REGS
        if(PORT_A_REGS[GPIO_GPxDAT_INDEX] & MASK_MISO)
        {
            data_in |= 1;
        }
        
        asm(" RPT #4 || NOP"); 
    }

    // 3. 拉高 CS (GPIO 33 -> Port B)
    // 【修改点】改为 Port B
    PORT_B_REGS[GPIO_GPxSET_INDEX] = MASK_CS;
    
    asm(" RPT #20 || NOP"); 

    return data_in;
}

// 封装
float MA600_ReadAngle_SoftSPI(void)
{
    uint16_t raw = MA600_ReadRaw_Fast();
    return (float)raw * 0.0000958738f; 
}
