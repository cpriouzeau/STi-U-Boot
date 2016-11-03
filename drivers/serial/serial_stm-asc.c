/*
 * drivers/serial/stm-asc.c
 *
 * Support for Serial I/O using STMicroelectronics' on-chip ASC.
 *
 *  Copyright (c) 2004,2008-2013  STMicroelectronics Limited
 *  Sean McGoogan <Sean.McGoogan@st.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include "common.h"
#include <serial.h>
#include "asm/io.h"
#include <stm/pio.h>
#include <stm/socregs.h>
#include <stm/clk.h>

#define CS7		0000040
#define CS8		0000060
#define CSIZE		0000060
#define CSTOPB		0000100
#define CREAD		0000200
#define PARENB		0000400
#define PARODD		0001000
#define HUPCL		0002000
#define CLOCAL		0004000

#define BAUDMODE	0x00001000
#define CTSENABLE	0x00000800
#define RXENABLE	0x00000100
#define RUN		0x00000080
#define STOPBIT		0x00000008
#define MODE		0x00000001
#define MODE_7BIT_PAR	0x0003
#define MODE_8BIT_PAR	0x0007
#define MODE_8BIT	0x0001
#define STOP_1BIT	0x0008
#define PARITYODD	0x0020

#define STA_NKD		0x0400
#define STA_TF		0x0200
#define STA_RHF		0x0100
#define STA_TOI		0x0080
#define STA_TNE		0x0040
#define STA_OE		0x0020
#define STA_FE		0x0010
#define STA_PE		0x0008
#define	STA_THE		0x0004
#define STA_TE		0x0002
#define STA_RBF		0x0001


#define UART_BAUDRATE_OFFSET	0x00
#define UART_TXBUFFER_OFFSET	0x04
#define UART_RXBUFFER_OFFSET	0x08
#define UART_CONTROL_OFFSET	0x0C
#define UART_INTENABLE_OFFSET	0x10
#define UART_STATUS_OFFSET	0x14
#define UART_GUARDTIME_OFFSET	0x18
#define UART_TIMEOUT_OFFSET	0x1C
#define UART_TXRESET_OFFSET	0x20
#define UART_RXRESET_OFFSET	0x24
#define UART_RETRIES_OFFSET	0x28

/* SBC */
#define UART_ENABLE			0x4
#define UART_LPM_CONFIG1	(ST_WKUP_REGS_BASE + UART_ENABLE)

#define UART_BAUDRATE_REG	(CONFIG_SYS_STM_ASC_BASE + UART_BAUDRATE_OFFSET)
#define UART_TXBUFFER_REG	(CONFIG_SYS_STM_ASC_BASE + UART_TXBUFFER_OFFSET)
#define UART_RXBUFFER_REG	(CONFIG_SYS_STM_ASC_BASE + UART_RXBUFFER_OFFSET)
#define UART_CONTROL_REG	(CONFIG_SYS_STM_ASC_BASE + UART_CONTROL_OFFSET)
#define UART_INTENABLE_REG	(CONFIG_SYS_STM_ASC_BASE + UART_INTENABLE_OFFSET)
#define UART_STATUS_REG		(CONFIG_SYS_STM_ASC_BASE + UART_STATUS_OFFSET)
#define UART_GUARDTIME_REG	(CONFIG_SYS_STM_ASC_BASE + UART_GUARDTIME_OFFSET)
#define UART_TIMEOUT_REG	(CONFIG_SYS_STM_ASC_BASE + UART_TIMEOUT_OFFSET)
#define UART_TXRESET_REG	(CONFIG_SYS_STM_ASC_BASE + UART_TXRESET_OFFSET)
#define UART_RXRESET_REG	(CONFIG_SYS_STM_ASC_BASE + UART_RXRESET_OFFSET)
#define UART_RETRIES_REG	(CONFIG_SYS_STM_ASC_BASE + UART_RETRIES_OFFSET)


/*---- Values for the BAUDRATE Register -----------------------*/

#define PCLK			(stm_get_uart_clk_rate())
#define BAUDRATE_VAL_M0(bps)	(PCLK / (16 * (bps)))
#define BAUDRATE_VAL_M1(bps)	((((bps * (1 << 14))+ (1<<13)) / (PCLK/(1 << 6))))

/*
 * MODE 0
 *                       ICCLK
 * ASCBaudRate =   ----------------
 *                   baudrate * 16
 *
 * MODE 1
 *                   baudrate * 16 * 2^16
 * ASCBaudRate =   ------------------------
 *                          ICCLK
 *
 * NOTE:
 * Mode 1 should be used for baudrates of 19200, and above, as it
 * has a lower deviation error than Mode 0 for higher frequencies.
 * Mode 0 should be used for all baudrates below 19200.
 */


#ifdef CONFIG_HWFLOW
static int hwflow = 0;		/* turned off by default */
#endif	/* CONFIG_HWFLOW */

/* Note: the argument order for "outl()" is swapped, w.r.t. writel() */
#define p2_inl(addr)		__raw_readl(addr)
#define p2_outl(addr,v)		__raw_writel(v, addr)


/* busy wait until it is safe to send a char */
static inline void tx_char_ready (void)
{
	unsigned long status;

	do {
		status = p2_inl (UART_STATUS_REG);
	} while (status & STA_TF);
}

/* initialize the ASC */
static int stm_asc_serial_init(void)
{
	const int cflag = CREAD | HUPCL | CLOCAL | CSTOPB | CS8 | PARODD;
	unsigned long val;
	int baud = gd->baudrate;
	int t, mode=1;

	val = p2_inl (0x94b5104);
	p2_outl(0x94b5104,val | 0x1800);

	switch (baud) {
#if 0
	case 0:
		t = -1;
		break;
	case 2400:
		t = BAUDRATE_VAL_M0(2400);
		mode = 0;
		break;
	case 4800:
		t = BAUDRATE_VAL_M0(4800);
		mode = 0;
		break;
#endif
	case 9600:
		t = BAUDRATE_VAL_M0(9600);
		mode = 0;
		break;
	case 19200:
		t = BAUDRATE_VAL_M1(19200);
		break;
	case 38400:
		t = BAUDRATE_VAL_M1(38400);
		break;
	case 57600:
		t = BAUDRATE_VAL_M1(57600);
		break;
	default:
		printf ("ASC: unsupported baud rate: %d, using 115200 instead.\n", baud);
	case 115200:
		t = BAUDRATE_VAL_M1(115200);
		break;
	}

	/* wait for end of current transmission */
	tx_char_ready ();

	/* disable the baudrate generator */
	val = p2_inl (UART_CONTROL_REG);
	p2_outl (UART_CONTROL_REG, (val & ~RUN));

	/* set baud generator reload value */
	p2_outl (UART_BAUDRATE_REG, t);

	/* reset the RX & TX buffers */
	p2_outl (UART_TXRESET_REG, 1);
	p2_outl (UART_RXRESET_REG, 1);

	/* build up the value to be written to CONTROL */
	val = RXENABLE | RUN;

	/* set character length */
	if ((cflag & CSIZE) == CS7)
		val |= MODE_7BIT_PAR;
	else {
		if (cflag & PARENB)
			val |= MODE_8BIT_PAR;
		else
			val |= MODE_8BIT;
	}

	/* set stop bit */
	/* it seems no '0 stop bits' option is available: by default
	 * we get 0.5 stop bits */
	if (cflag & CSTOPB)
		val |= STOP_1BIT;

	/* odd parity */
	if (cflag & PARODD)
		val |= PARITYODD;

#ifdef CONFIG_HWFLOW
	/*  set flow control */
	if (hwflow)
		val |= CTSENABLE;
#endif	/* CONFIG_HWFLOW */

	/* set baud generator mode */
	if (mode)
		val |= BAUDMODE;

	/* finally, write value and enable ASC */
	p2_outl (UART_CONTROL_REG, val);
	return 0;
}

/* returns TRUE if a char is available, ready to be read */
static int stm_asc_serial_tstc(void)
{
	unsigned long status;

	status = p2_inl (UART_STATUS_REG);
	return (status & STA_RBF);
}

/* blocking function, that returns next char */
static int stm_asc_serial_getc(void)
{
	char ch;

	/* polling wait: for a char to be read */
	while (!stm_asc_serial_tstc());

	/* read char, now that we know we have one */
	ch = p2_inl (UART_RXBUFFER_REG);

	/* return consumed char to the caller */
	return ch;
}

/* write write out a single char */
static void stm_asc_serial_putc(char ch)
{
	/* Stream-LF to CR+LF conversion */
	if (ch == 10)
		stm_asc_serial_putc('\r');

	/* wait till safe to write next char */
	tx_char_ready ();

	/* finally, write next char */
	p2_outl (UART_TXBUFFER_REG, ch);
}

/* called to adjust baud-rate */
static void stm_asc_serial_setbrg(void)
{
	/* just re-initialize ASC */
	stm_asc_serial_init();
}

#ifdef CONFIG_HWFLOW
extern int hwflow_onoff (int on)
{
	switch (on) {
	case 0:
	default:
		break;		/* return current */
	case 1:
		hwflow = 1;	/* turn on */
		stm_asc_serial_init();
		break;
	case -1:
		hwflow = 0;	/* turn off */
		stm_asc_serial_init();
		break;
	}
	return hwflow;
}
#endif	/* CONFIG_HWFLOW */

static struct serial_device stm_asc_serial_drv = {
	.name	= "stm_asc",
	.start	= stm_asc_serial_init,
	.stop	= NULL,
	.setbrg	= stm_asc_serial_setbrg,
	.putc	= stm_asc_serial_putc,
	.puts	= default_serial_puts,
	.getc	= stm_asc_serial_getc,
	.tstc	= stm_asc_serial_tstc,
};

extern void stm_asc_serial_initialize(void)
{
	serial_register(&stm_asc_serial_drv);
}

/*
 * If we are also using DTF (JTAG), we probably want that driver
 * to dominate, hence we define this function as "__weak".
 */
extern __weak struct serial_device *default_serial_console(void)
{
	return &stm_asc_serial_drv;
}
