#include "bsp_tim.h"

uint16_t foc_cputimer_0_cnt = 0;
__interrupt void INT_FOC_CPUTIMER0_ISR(void)
{
    foc_cputimer_0_cnt++;
    if(foc_cputimer_0_cnt % 10000 == 0)
    {

    }
    

    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

uint16_t foc_cputimer_1_cnt = 0;
__interrupt void INT_FOC_CPUTIMER1_ISR(void)
{
    foc_cputimer_1_cnt++;
    if(foc_cputimer_1_cnt % 10000 == 0)
    {

    }
    

    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
    // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

uint16_t foc_cputimer_2_cnt = 0;
__interrupt void INT_FOC_CPUTIMER2_ISR(void)
{
    foc_cputimer_2_cnt++;
    if(foc_cputimer_2_cnt % 10000 == 0)
    {

    }
    

    CPUTimer_clearOverflowFlag(CPUTIMER2_BASE);
    // Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
// End of File
//
