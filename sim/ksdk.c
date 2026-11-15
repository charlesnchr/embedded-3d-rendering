/*
 * Host definitions of the Kinetis SDK calls that the firmware makes. GPIO
 * writes update a table of pin levels. SPI transfers go to the SSD1331 model,
 * as commands or as pixel data depending on the level of the D/C pin. The
 * clock, pin multiplexing and delay calls do nothing.
 */
#include <stdarg.h>
#include <stdio.h>

#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_spi_master_driver.h"
#include "sim.h"

const gpio_output_pin_user_config_t ledPins[] = { { kGpioLED1 } };

ssd1331_t simDisplay;
int simConsole;

/*
 * The firmware never drives D/C to a known level before its first command. It
 * toggles the pin around each command byte and sets it high for pixel data, so
 * the level it finds at start-up has to be high for the display to work.
 */
static uint8_t pinLevel[kGpioPinCount] = { [kGpioDC] = 1 };

void
GPIO_DRV_Init(const void *inputPins, const gpio_output_pin_user_config_t *outputPins)
{
	(void)inputPins;
	(void)outputPins;
}

void
GPIO_DRV_SetPinOutput(uint32_t pin)
{
	pinLevel[pin] = 1;
}

void
GPIO_DRV_ClearPinOutput(uint32_t pin)
{
	pinLevel[pin] = 0;
}

void
GPIO_DRV_TogglePinOutput(uint32_t pin)
{
	pinLevel[pin] ^= 1;
}

void
PORT_HAL_SetMuxMode(uint32_t portBase, uint32_t pin, int mux)
{
	(void)portBase;
	(void)pin;
	(void)mux;
}

void
CLOCK_SYS_EnableSpiClock(uint32_t instance)
{
	(void)instance;
}

void
hardware_init(void)
{
}

void
dbg_uart_init(void)
{
}

void
OSA_Init(void)
{
}

void
OSA_TimeDelay(uint32_t milliseconds)
{
	(void)milliseconds;
}

spi_status_t
SPI_DRV_MasterInit(uint32_t instance, spi_master_state_t *state)
{
	(void)instance;
	(void)state;
	return kStatus_SPI_Success;
}

void
SPI_DRV_MasterConfigureBus(uint32_t instance, const spi_master_user_config_t *config,
			   uint32_t *calculatedBaudRate)
{
	(void)instance;
	*calculatedBaudRate = config->bitsPerSec;
}

/*
 * The model looks at D/C only. It ignores chip select and reset. The firmware
 * holds kGpioRST low while it sends pixels, and the panel on the bench kept
 * showing the picture, so that pin was not wired to the reset input the way
 * its name suggests. The pin map of the board is not part of this repository.
 *
 * Nothing is connected to MISO, so the receive buffer is left as it is.
 */
spi_status_t
SPI_DRV_MasterTransferBlocking(uint32_t instance, const spi_master_user_config_t *config,
			       const uint8_t *sendBuffer, uint8_t *receiveBuffer,
			       size_t transferByteCount, uint32_t timeout)
{
	(void)instance;
	(void)config;
	(void)receiveBuffer;
	(void)timeout;

	if (pinLevel[kGpioDC])
		sim_pixels_written(ssd1331_write_data(&simDisplay, sendBuffer, transferByteCount));
	else
		ssd1331_write_command(&simDisplay, sendBuffer, transferByteCount);

	return kStatus_SPI_Success;
}

int
sim_console_printf(const char *format, ...)
{
	va_list arguments;
	int written = 0;

	if (simConsole) {
		va_start(arguments, format);
		written = vfprintf(stderr, format, arguments);
		va_end(arguments);
	}

	return written;
}
