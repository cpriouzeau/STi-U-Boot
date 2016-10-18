/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#include <common.h>
#include <command.h>
#include <stm/soc.h>
#include <stm/socregs.h>
#include <asm/io.h>
#include <stm/pio.h>
#include <stm/sysconf.h>

extern void stm_uart_init(struct stm_uart_config *stm_uart_config)
{
        /* Route SBC_UART0 via PIO3 for TX, RX, CTS & RTS (Alternative #1) */
        PIOALT(stm_uart_config->output.port,
		stm_uart_config->output.pin,
		stm_uart_config->output.alt,
		stm_pad_direction_output);      /* SBC_UART0-TX */
        PIOALT(stm_uart_config->input.port,
		stm_uart_config->input.pin,
		stm_uart_config->input.alt,
		stm_pad_direction_input);      /* SBC_UART0-TX */
}
