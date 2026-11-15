/* Host stand-in for the Kinetis SDK 1.1 clock manager. See board.h. */
#ifndef SIM_KSDK_FSL_CLOCK_MANAGER_H
#define SIM_KSDK_FSL_CLOCK_MANAGER_H

#include <stdint.h>

void CLOCK_SYS_EnableSpiClock(uint32_t instance);

#endif
