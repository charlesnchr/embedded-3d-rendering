/*
 * Host stand-in for the Kinetis SDK 1.1 debug console. The SDK header routes
 * printf to the debug UART. Here it goes to sim_console_printf, which stays
 * quiet unless the simulator was started with --console.
 */
#ifndef SIM_KSDK_FSL_DEBUG_CONSOLE_H
#define SIM_KSDK_FSL_DEBUG_CONSOLE_H

int sim_console_printf(const char *format, ...);

#define printf sim_console_printf

#endif
