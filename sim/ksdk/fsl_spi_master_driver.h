/* Host stand-in for the Kinetis SDK 1.1 SPI master driver. See board.h. */
#ifndef SIM_KSDK_FSL_SPI_MASTER_DRIVER_H
#define SIM_KSDK_FSL_SPI_MASTER_DRIVER_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
	kStatus_SPI_Success = 0
} spi_status_t;

typedef enum { kSpiClockPolarity_ActiveHigh, kSpiClockPolarity_ActiveLow } spi_clock_polarity_t;
typedef enum { kSpiClockPhase_FirstEdge, kSpiClockPhase_SecondEdge } spi_clock_phase_t;
typedef enum { kSpiMsbFirst, kSpiLsbFirst } spi_shift_direction_t;

typedef struct {
	uint32_t		bitsPerSec;
	spi_clock_polarity_t	polarity;
	spi_clock_phase_t	phase;
	spi_shift_direction_t	direction;
} spi_master_user_config_t;

typedef struct {
	uint32_t unused;
} spi_master_state_t;

spi_status_t SPI_DRV_MasterInit(uint32_t instance, spi_master_state_t *state);
void SPI_DRV_MasterConfigureBus(uint32_t instance, const spi_master_user_config_t *config,
				uint32_t *calculatedBaudRate);
spi_status_t SPI_DRV_MasterTransferBlocking(uint32_t instance, const spi_master_user_config_t *config,
					    const uint8_t *sendBuffer, uint8_t *receiveBuffer,
					    size_t transferByteCount, uint32_t timeout);

#endif
