/*
 * (c) 2010-2013 STMicroelectronics Limited
 *
 * Author: Pawel Moll <pawel.moll@st.com>
 *         Sean McGoogan <Sean.McGoogan@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */



#ifndef __INCLUDE_STM_PIO_CONTROL_H
#define __INCLUDE_STM_PIO_CONTROL_H

struct stm_pio_control_retime_config {
	int retime:2;
	int clk:3;		/* was previously "clk1notclk0:2" */
	int clknotdata:2;
	int double_edge:2;
	int invertclk:2;
	int delay:5;		/* was previously "delay_input:3" */
};


/*
 * list of valid values for the "clk" bitfield above
 */
#define STM_RETIME_VALUE_CLK_0			(0)
#define STM_RETIME_VALUE_CLK_1			(1)
	/* if the H/W bitfield is 2-bits wide ? */
#define STM_RETIME_VALUE_CLK_2		(2)
#define STM_RETIME_VALUE_CLK_3		(3)

/*
 * list of valid values for the "delay" bitfield above
 */
#define STM_RETIME_VALUE_DELAY_0		(0)	/* no delay */
#define STM_RETIME_VALUE_DELAY_300		(1)	/* delay 0.3 ns */
#define STM_RETIME_VALUE_DELAY_500		(2)	/* delay 0.5 ns */
#define STM_RETIME_VALUE_DELAY_750		(3)	/* delay 0.75 ns */
#define STM_RETIME_VALUE_DELAY_1000		(4)	/* delay 1.0 ns */
#define STM_RETIME_VALUE_DELAY_1250		(5)	/* delay 1.25 ns */
#define STM_RETIME_VALUE_DELAY_1500		(6)	/* delay 1.5 ns */
#define STM_RETIME_VALUE_DELAY_1750		(7)	/* delay 1.75 ns */
#define STM_RETIME_VALUE_DELAY_2000		(8)	/* delay 2.0 ns */
#define STM_RETIME_VALUE_DELAY_2250		(9)	/* delay 2.25 ns */
#define STM_RETIME_VALUE_DELAY_2500		(10)	/* delay 2.5 ns */
#define STM_RETIME_VALUE_DELAY_2750		(11)	/* delay 2.75 ns */
#define STM_RETIME_VALUE_DELAY_3000		(12)	/* delay 3.0 ns */
#define STM_RETIME_VALUE_DELAY_3250		(13)	/* delay 3.25 ns */
		/* Note: 3.25 ns is the a maximum delay available */


/* 	Generic Retime Padlogic possible modes
 * Refer to GRP Functional specs (ADCS 8198257) for more details */

/* B Mode
 * Bypass retime with optional delay */
#define	RET_BYPASS(_delay) (&(struct stm_pio_control_retime_config){ \
		.retime = 0, \
		.clk = STM_RETIME_VALUE_CLK_0, \
		.clknotdata = 0, \
		.double_edge = 0, \
		.invertclk = 0, \
		.delay = STM_RETIME_VALUE_DELAY_ ## _delay, \
		})
/*
 * (R0, R1, R0D, R1D modes )
 * single-edge data non inverted clock, retime data with clk */
#define	RET_SE_NICLK_IO(_delay, _clk) (&(struct stm_pio_control_retime_config){ \
		.retime = 1, \
		.clk = STM_RETIME_VALUE_CLK_ ## _clk, \
		.clknotdata = 0, \
		.double_edge = 0, \
		.invertclk = 0, \
		.delay = STM_RETIME_VALUE_DELAY_ ## _delay, \
		})

/* RIV0, RIV1, RIV0D, RIV1D modes
 * single-edge data inverted clock, retime data with clk */
#define	RET_SE_ICLK_IO(_delay, _clk) (&(struct stm_pio_control_retime_config){ \
		.retime = 1, \
		.clk = STM_RETIME_VALUE_CLK_ ## _clk, \
		.clknotdata = 0, \
		.double_edge = 0, \
		.invertclk = 1, \
		.delay = STM_RETIME_VALUE_DELAY_ ## _delay, \
		})

/* R0E, R1E, R0ED, R1ED modes
 * double-edge data, retime data with clk */
#define	RET_DE_IO(_delay, _clk) (&(struct stm_pio_control_retime_config){ \
		.retime = 1, \
		.clk = STM_RETIME_VALUE_CLK_ ## _clk, \
		.clknotdata = 0, \
		.double_edge = 1, \
		.invertclk = 0, \
		.delay = STM_RETIME_VALUE_DELAY_ ## _delay, \
		})

/* CIV0, CIV1 modes with inverted clock
 * Retiming the clk pins will park clock & reduce the noise within the core. */
#define RET_ICLK(_delay, _clk) (&(struct stm_pio_control_retime_config){ \
		.retime = 1, \
		.clk = STM_RETIME_VALUE_CLK_ ## _clk, \
		.clknotdata = 1, \
		.double_edge = 0, \
		.invertclk = 1, \
		.delay = STM_RETIME_VALUE_DELAY_ ## _delay, \
		})

/* CLK0, CLK1 modes with non-inverted clock
 * Retiming the clk pins will park clock & reduce the noise within the core. */
#define RET_NICLK(_delay, _clk) (&(struct stm_pio_control_retime_config){ \
		.retime = 1, \
		.clk = STM_RETIME_VALUE_CLK_ ## _clk, \
		.clknotdata = 1, \
		.double_edge = 0, \
		.invertclk = 0, \
		.delay = STM_RETIME_VALUE_DELAY_ ## _delay, \
		})

#endif	/* __INCLUDE_STM_PIO_CONTROL_H */

