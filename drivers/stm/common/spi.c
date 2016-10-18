/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#include <common.h>
#include <stm/soc.h>
#include <stm/socregs.h>
#include <stm_spi_fsm.h>

struct stm_spifsm_platform spi_fsm_data;
extern void stm_configure_spi(void)
{

#if defined(CONFIG_STM_FSM_SPI)
	STM_PIOALT_SELECT(SPI_nCS,  1);	/* SPI_nCS */
	STM_PIOALT_SELECT(SPI_CLK,  1);	/* SPI_CLK */
	STM_PIOALT_SELECT(SPI_MOSI, 1);	/* SPI_D0 */
	STM_PIOALT_SELECT(SPI_MISO, 1);	/* SPI_D1 */
		/* following only really needed for x4 mode */
	STM_PIOALT_SELECT(SPI_nWP,  1);	/* SPI_D2 */
	STM_PIOALT_SELECT(SPI_HOLD, 1);	/* SPI_D3 */

       /* SoC/IP capablities
        * TODO: Imran: I am taking these values from kernel bootlog.
        * Should it change for other STxH410 based platforms */

       spi_fsm_data.base = CONFIG_SYS_STM_SPI_FSM_BASE;

       /* SoC/IP Capabilities */
       spi_fsm_data.capabilities.no_read_repeat = 1;
       spi_fsm_data.capabilities.no_write_repeat = 1;
       spi_fsm_data.capabilities.read_status_bug = spifsm_read_status_clkdiv4;

       spi_fsm_data.capabilities.dual_mode = 1;
       spi_fsm_data.capabilities.no_sw_reset = 0;
       spi_fsm_data.capabilities.quad_mode = 1;
       spi_fsm_data.capabilities.addr_32bit = 0; /* Set it depending on flash size */
       spi_fsm_data.capabilities.no_clk_div_4 = 0; /* Set it depending on flash size */
       spi_fsm_data.capabilities.no_poll_mode_change = 0; /* Set it depending on flash size */
       spi_fsm_data.capabilities.dummy_on_write = 0; /* Set it depending on flash size */

       stm_spifsm_register_device(&spi_fsm_data);
#elif defined(CONFIG_SOFT_SPI)
	/*
	 * We want to use "bit-banging" for SPI (not SSC, nor FSM).
	 */

	/* route PIO (alternate #0) */
	STID325_PIOALT_SELECT(SPI_MISO, 0);	/* SPI_MISO */
	STID325_PIOALT_SELECT(SPI_MOSI, 0);	/* SPI_MOSI */
	STID325_PIOALT_SELECT(SPI_nCS,  0);	/* SPI_nCS */
	STID325_PIOALT_SELECT(SPI_CLK,  0);	/* SPI_CLK */

	/* set PIO directionality */
	SET_PIO_PIN2(SPI_MISO, STPIO_IN);	/* SPI_MISO */
	SET_PIO_PIN2(SPI_MOSI, STPIO_OUT);	/* SPI_MOSI */
	SET_PIO_PIN2(SPI_nCS,  STPIO_OUT);	/* SPI_nCS */
	SET_PIO_PIN2(SPI_CLK,  STPIO_OUT);	/* SPI_CLK */

	/* drive outputs with sensible initial values */
	STPIO_SET_PIN2(SPI_MOSI, 0);		/* deassert SPI_MOSI */
	STPIO_SET_PIN2(SPI_nCS,  1);		/* deassert SPI_nCS */
	STPIO_SET_PIN2(SPI_CLK,  1);		/* assert SPI_CLK */

#else
#error Which DRIVER to use for SPI ?
#endif
}

#if defined(CONFIG_SOFT_SPI)
extern void stm_spi_scl(const int val)
{
	STPIO_SET_PIN2(SPI_CLK, val ? 1 : 0);
}

extern void stm_spi_sda(const int val)
{
	STPIO_SET_PIN2(SPI_MOSI, val ? 1 : 0);
}

extern unsigned char stm_spi_read(void)
{
	return STPIO_GET_PIN2(SPI_MISO);
}
#endif	/* CONFIG_SOFT_SPI */

#if defined(CONFIG_SOFT_SPI) || defined(CONFIG_STM_SSC_SPI)
/*
 * A pair of functions to assert and de-assert the SPI
 * chip select line, for the given SPI "slave" device.
 *
 * This is used by both the S/W "bit-banging" and the
 * H/W SSC drivers (but not the H/W FSM driver).
 *
 * We only support *one* SPI device, so we just ignore
 * the "slave" parameter. This may change later...
 */
extern void spi_cs_activate(struct spi_slave * const slave)
{
	/* assert SPI CSn */
	STPIO_SET_PIN2(SPI_nCS, 0);

	/* wait 1us for CSn assert to propagate  */
	udelay(1);
}
extern void spi_cs_deactivate(struct spi_slave * const slave)
{
	/* DE-assert SPI CSn */
	STPIO_SET_PIN2(SPI_nCS, 1);
}
#endif	/* defined(CONFIG_SOFT_SPI) || defined(CONFIG_STM_SSC_SPI) */



/*-----------------------------------------------------------------------
 * Determine if a SPI chipselect is valid.
 * This function is provided by the board if the low-level SPI driver
 * needs it to determine if a given chipselect is actually valid.
 *
 * Returns: 1 if bus:cs identifies a valid chip on this board, 0
 * otherwise.
 *
 * Note: Initially, we will only accept CS #0, on BUS #0 (i.e. 0:0).
 */
extern int spi_cs_is_valid(
       const unsigned int bus,
       const unsigned int cs)
{
       if ( (bus==0) && (cs==0) )      /* is it 0:0 ? */
       {
               return 1;               /* PASS - bus:cs is "OK" */
       }
       else
       {
               return 0;               /* FAIL - bus:cs is NOT ok */
       }
}


