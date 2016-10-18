/*
 * (C) Copyright 2008-2014 STMicroelectronics.
 *
 * Sean McGoogan <Sean.McGoogan@st.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <stm/soc.h>
#include <stm/socregs.h>
#include <asm/io.h>
#include <stm/pio.h>
#include <stm/sysconf.h>
#include <stm/pio-control.h>
#include <stm/usb.h>
#include <stm/stbus.h>

void flashWriteEnable(void)
{
	/* Enable Vpp for writing to flash */
	/* Nothing to do! */
}

void flashWriteDisable(void)
{
	/* Disable Vpp for writing to flash */
	/* Nothing to do! */
}


/*
 *	MII1: PIO0[7] = M_RGMII1_notRESET
 */
#define GMII_PHY_NOT_RESET	0, 7

#ifndef CONFIG_SYS_DCACHE_OFF
extern void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

extern int board_early_init_f(void)
{
	/* Setup PIOs for ASC device */
#ifndef CONFIG_STM_B2260
	stm_uart_init(&stm_uart_config[0]);
#else
	stm_uart_init(&stm_uart_config[3]);
#endif
        return 0;
}


#ifdef CONFIG_DRIVER_NET_STM_GMAC
extern void stmac_phy_reset(void)
{
	/*
	 * Reset the Ethernet PHY/Switch.
	 */
	STPIO_SET_PIN2(GMII_PHY_NOT_RESET, 1);
	udelay(10000);				/* 10 ms */
	STPIO_SET_PIN2(GMII_PHY_NOT_RESET, 0);
	udelay(10000);				/* 10 ms */
	STPIO_SET_PIN2(GMII_PHY_NOT_RESET, 1);
	udelay(10000);				/* 10 ms */
}
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */


extern int board_init(void)
{
#ifdef CONFIG_DRIVER_NET_STM_GMAC
	/*
	 * Configure the Ethernet PHY/Switch Reset signal
	 */
	SET_PIO_PIN2(GMII_PHY_NOT_RESET, STPIO_OUT);

#if defined(CONFIG_STMAC_RTL8211E)	/* Realtek RTL8211E */
	stm_configure_ethernet(&(struct stm_ethernet_config) {
			.mode = stm_ethernet_mode_rgmii,
			.ext_clk = 0,
			.phy_bus = 1, });
#else
#error Unknown PHY type associated with STM GMAC #1
#endif
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */

#if defined(CONFIG_CMD_I2C)
	stm_configure_i2c();
#endif

#if defined(CONFIG_CMD_NAND)
	/*
	 * Configure the PIO signals (in the FlashSS) for NAND.
	 */
	stm_configure_nand();
#endif	/* CONFIG_CMD_NAND */

	return 0;
}


int checkboard (void)
{
	printf("\n\nBoard: %s-%s-%s [ARM] \n", STM_BOARD, CONFIG_BOARDREV_STRING, STM_SOC);

#if defined(CONFIG_SPI)
	/*
	 * Configure for the SPI Serial Flash.
	 */
	stm_configure_spi();
#endif	/* CONFIG_SPI */

#if defined(CONFIG_DRIVER_NET_STM_GMAC)
	/* Hard Reset the PHY -- do after we have configured the MAC */
	stmac_phy_reset();
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */

	return 0;
}

#if defined(CONFIG_DRIVER_NET_STM_GMAC)
extern void stmac_set_mac_speed(int speed)
{
	unsigned int val;
	/*
	 * Manage the MAC speed.
	 * Ethernet clock needs to be adjusted depending on the speed of
	 * the link:
	 * 1Gbps : 125MHz
	 * 100 Mbps : 25MHz
	 * 10 Mbps : 2.5MHz
	 *
	 * Now there are two approaches to achieve this. Either we change the
	 * setting for 660MHz quad frequency synthesizer (address:0x09103310)
	 * or we change the findivider(address:0x091031cc) accordingly for
	 * ethernet clock. I am taking the second approach since it is more
	 * modular.
	 *
	 */

	val=readl(0x091031cc);
	val=val&0xFFFFFFC0;
	if (speed==1000)
	{
	    /* 125Mhz */
	    val=val|0x0;
  
	}
	else if (speed == 100)
	{
	    /* 25Mhz */
	    val=val|0x4;
	}
	else if (speed == 10)
	{
	    /* 2.5Mhz */
	    val=val|0x31;
	}
	writel(val,0x091031cc);

}
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */



