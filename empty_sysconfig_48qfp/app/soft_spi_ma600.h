#ifndef BSP_SOFT_SPI_MA600_H
#define BSP_SOFT_SPI_MA600_H

#include "driverlib.h"
#include "device.h"
#include "board.h"

// software pin config


void MA600_SoftSPI_Init(void);
float MA600_ReadAngle_SoftSPI(void);
uint16_t MA600_ReadRaw_Fast(void);

#endif // BSP_SOFT_SPI_MA600_H
//
// End of File
//
