/*
 * Host stand-in for the Kinetis SDK 1.1 board support that the firmware in
 * Implementations/C includes. It declares the SDK names the firmware uses and
 * nothing else. The definitions are in sim/ksdk.c.
 */
#ifndef SIM_KSDK_BOARD_H
#define SIM_KSDK_BOARD_H

#include <stddef.h>
#include <stdint.h>

/* GPIO pins. The numbers only have to be distinct. */
enum {
	kGpioLED1,
	kGpioOC,	/* chip select of the display */
	kGpioDC,	/* data (high) or command (low) */
	kGpioRST,
	kGpioPinCount
};

typedef struct {
	uint32_t pinName;
} gpio_output_pin_user_config_t;

extern const gpio_output_pin_user_config_t ledPins[];

void GPIO_DRV_Init(const void *inputPins, const gpio_output_pin_user_config_t *outputPins);
void GPIO_DRV_SetPinOutput(uint32_t pin);
void GPIO_DRV_ClearPinOutput(uint32_t pin);
void GPIO_DRV_TogglePinOutput(uint32_t pin);

/* Pin multiplexing. */
#define PORTA_BASE	0x40049000u
enum { kPortMuxAlt3 = 3 };
void PORT_HAL_SetMuxMode(uint32_t portBase, uint32_t pin, int mux);

/* Start-up and timing. */
void hardware_init(void);
void dbg_uart_init(void);
void OSA_Init(void);
void OSA_TimeDelay(uint32_t milliseconds);

#endif
