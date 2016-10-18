/*
 * (C) Copyright 2008-2014 STMicroelectronics.
 *
 * Stuart Menefy <stuart.menefy@st.com>
 * Sean McGoogan <Sean.McGoogan@st.com>
 * Pawel Moll <pawel.moll@st.com>
 * Youssef TRIKI <youssef.triki@st.com>
 * Imran Khan <imran.khan@st.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <stm/soc.h>
#include <stm/stxh410_family.h>
#include <asm/io.h>
#include <stm/pio.h>
#include <stm/stbus.h>
#include <sdhci.h>
#include <stm/sysconf.h>
#include <stm/pad.h>
#include <stm/pio-control.h>
#include <mmc.h>
#include <ata.h>
#include <spi.h>
#include <netdev.h>
#include <stm/stm-sdhci.h>

DECLARE_GLOBAL_DATA_PTR;

#undef  BUG_ON
#define BUG_ON(condition) do { if ((condition)!=0) BUG(); } while(0)

#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))

#if defined(CONFIG_STM_IR)
#define IRB_MODE_IR	0
#define IRB_MODE_UHF	1
#define IRB_MODE_TX	2
#define IRB_MODE_TX_OD	3
#endif

#define TRUE			1
#define FALSE			0
	/*
	 * Define the following only if we need to debug the PIO
	 * configurations, including alternate function usage, and re-timings.
	 * Then set 'debug_pad_configs' to TRUE as appropriate.
	 * NOTE: do *not* enable this before the "console" is up and running!
	 */
//#define DEBUG_PAD_CONFIGS
#ifdef DEBUG_PAD_CONFIGS
volatile int debug_pad_configs = 0;
#endif
void reset_cpu(ulong addr)
{
	/* set Pull up need more elaboration -> TRIKIY */
	unsigned long sysconf = readl(0x9600000);
	SET_SYSCONF_BIT(sysconf,0,0);
	writel(sysconf, 0x9600000);
}
static void stm_clocks(void)
{
	/*
	 * Ideally, we should probe to determine all the clock frequencies.
	 * However, for simplicity, we will simply hard-wire the values
	 * that U-Boot will use for computing the clock dividers later.
	 * WARNING: Getting these values wrong may result in strange behaviour!
	 *
	 * Note: for the ASC in the SBC, we expect this will always be 30MHz,
	 *       otherwise we expect the ASC to be 200MHz.
	 */
#if (CONFIG_SYS_STM_ASC_BASE==STXH410_SBC_ASC0_BASE) || (CONFIG_SYS_STM_ASC_BASE==STXH410_SBC_ASC1_BASE)
	gd->arch.stm_uart_frq =  30ul * 1000000ul;	/*  30 MHz */
#else
	gd->arch.stm_uart_frq =  200ul * 1000000ul;	/*  200 MHz */
#endif
        gd->arch.stm_comm_frq = 100ul * 1000000ul;      /* 100 MHz */
        gd->arch.stm_sbc_comm_frq = 30ul * 1000000ul;   /* 30 MHz */
}


/*
 * PIO alternative Function selector
 */
extern void stm_pioalt_select(int port, const int pin, const int alt)
{
	unsigned long sysconf, *sysconfReg;

#ifdef DEBUG_PAD_CONFIGS
	if (debug_pad_configs)
		printf("%s(port=%d, pin=%d, alt=%d)\n", __func__, port, pin, alt);
	BUG_ON(pin < 0 || pin > 7);
	BUG_ON(alt < 0 || alt > 7);
#endif

	switch (port)
	{
	case 0 ... 5:		/* in "SBC Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(0);
		sysconfReg += port;
		break;
	case 10 ... 20:		/* in "FRONT Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(1000);
		sysconfReg += port - 10;
		break;
	case 30 ... 35:		/* in "REAR Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(2000);
		sysconfReg += port - 30;
		break;
	case 40 ... 42:		/* in "FLASH Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(3000);
		sysconfReg += port - 40;
		break;
	default:
		BUG();
		return;
	}

#ifdef DEBUG_PAD_CONFIGS
	if (debug_pad_configs)
		printf("ante: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BITS(sysconf, TRUE, pin*4,(pin*4)+3, alt,alt);
	writel(sysconf, sysconfReg);
#ifdef DEBUG_PAD_CONFIGS
	if (debug_pad_configs)
		printf("post: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
}

/* Pad configuration */

extern void stm_pioalt_pad(int port, const int pin,
		const enum stm_pad_gpio_direction direction)
{
	int bit;
	int oe=0, pu=0, od=0;
	unsigned long sysconf, *sysconfReg;

	/*
	 * NOTE: The PIO configuration for the PIO pins in the
	 * "FLASH Bank" are different from all the other banks!
	 * Specifically, the output-enable pad control register
	 * (SYS_CFG_3040) and the pull-up pad control register
	 * (SYS_CFG_3050), are both classed as being "reserved".
	 * Hence, we do not write to these registers to configure
	 * the OE and PU features for PIOs in this bank. However,
	 * the open-drain pad control register (SYS_CFG_3060)
	 * follows the style of the other banks, and so we can
	 * treat that register normally.
	 *
	 * Being pedantic, we should configure the PU and PD features
	 * in the "FLASH Bank" explicitly instead using the four
	 * SYS_CFG registers: 3080, 3081, 3085, and 3086. However, this
	 * would necessitate passing in the alternate function number
	 * to this function, and adding some horrible complexity here.
	 * Alternatively, we could just perform 4 32-bit "pokes" to
	 * these four SYS_CFG registers early in the initialization.
	 * In practice, these four SYS_CFG registers are correct
	 * after a reset, and U-Boot does not need to change them, so
	 * we (cheat and) rely on these registers being correct.
	 * WARNING: Please be aware of this (pragmatic) behaviour!
	 */
	int flashSS = 0;	/* bool: PIO in the Flash Sub-System ? */

	switch (direction) {
	case stm_pad_direction_input:
		oe = 0; pu = 0; od = 0;
		break;
	case stm_pad_direction_input_with_pullup:
		oe = 0; pu = 1; od = 0;
		break;
	case stm_pad_direction_output:
		oe = 1; pu = 0; od = 0;
		break;
	case stm_pad_direction_bidir_no_pullup:
		oe = 1; pu = 0; od = 1;
		break;
	default:
		BUG();
		break;
	}

#ifdef DEBUG_PAD_CONFIGS
	if (debug_pad_configs)
		printf("%s(port=%d, pin=%d, oe=%d, pu=%d, od=%d)\n", __func__, port, pin, oe, pu, od);
	BUG_ON(pin < 0 || pin > 7);
#endif

	switch (port)
	{
	case 0 ... 5:		/* in "SBC Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(40);
		sysconfReg += port / 4;
		break;
	case 10 ... 20:		/* in "FRONT Bank" */
		port -= 10;
		sysconfReg = (unsigned long*)STXH410_SYSCFG(1040);
		sysconfReg += port / 4;
		break;
	case 30 ... 35:		/* in "REAR Bank" */
		port -= 30;
		sysconfReg = (unsigned long*)STXH410_SYSCFG(2040);
		sysconfReg += port / 4;
		break;
	case 40 ... 42:		/* in "FLASH Bank" */
		port -= 40;
		sysconfReg = (unsigned long*)STXH410_SYSCFG(3040);
		sysconfReg += port / 4;
		flashSS = 1;	/* pin is in the Flash Sub-System */
		break;
	default:
		BUG();
		return;
	}

	bit = ((port * 8) + pin) % 32;

		/* set the "Output Enable" pad control */
	if (!flashSS)	/* but, do nothing if in the FlashSS */
	{
#ifdef DEBUG_PAD_CONFIGS
		if (debug_pad_configs)
			printf("ante: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
		sysconf = readl(sysconfReg);
		SET_SYSCONF_BIT(sysconf, oe, bit);
		writel(sysconf, sysconfReg);
#ifdef DEBUG_PAD_CONFIGS
		if (debug_pad_configs)
			printf("post: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
	}

	sysconfReg += 10;	/* skip to next set of syscfg registers */

		/* set the "Pull Up" pad control */
	if (!flashSS)	/* but, do nothing if in the FlashSS */
	{
#ifdef DEBUG_PAD_CONFIGS
		if (debug_pad_configs)
			printf("ante: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
		sysconf = readl(sysconfReg);
		SET_SYSCONF_BIT(sysconf, pu, bit);
		writel(sysconf, sysconfReg);
#ifdef DEBUG_PAD_CONFIGS
		if (debug_pad_configs)
			printf("post: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
	}

	sysconfReg += 10;	/* skip to next set of syscfg registers */

		/* set the "Open Drain Enable" pad control */
#ifdef DEBUG_PAD_CONFIGS
	if (debug_pad_configs)
		printf("ante: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, od, bit);
	writel(sysconf, sysconfReg);
#ifdef DEBUG_PAD_CONFIGS
	if (debug_pad_configs)
		printf("post: *%p = 0x%08lx\n", sysconfReg, *sysconfReg);
#endif
}

void stm_configure_pios(
	const struct stm_pad_pin * const pad_config,
	const size_t num_pads)
{
	size_t i;

		/* now configure all the PIOs */
	for (i = 0; i < num_pads; i++)
	{
		const struct stm_pad_pin * const pad = &pad_config[i];
		const int portno = pad->pio.port;
		const int pinno = pad->pio.pin;

#ifdef DEBUG_PAD_CONFIGS
	if (debug_pad_configs)
		printf("%2u: PIO%03u[%u] %-7s, alt=%u, retime=%p\n",
			i+1,
			portno, pinno,
			(pad->direction==stm_pad_direction_input) ? "in" :
				(pad->direction==stm_pad_direction_input_with_pullup) ? "in+pu" :
				(pad->direction==stm_pad_direction_output) ? "out" :
				(pad->direction==stm_pad_direction_bidir_no_pullup) ? "bidir" :
				(pad->direction==stm_pad_direction_bidir_with_pullup) ? "bidir+pu" :
				(pad->direction==stm_pad_direction_ignored) ? "ignore" :
				"BAD-BAD",
			pad->pio.alt,
			pad->retime
		);
#endif

		if (pad->direction == stm_pad_direction_ignored)
			continue;	/* skip all "ignored" pads */

		stm_pioalt_select(portno, pinno, pad->pio.alt);
		stm_pioalt_pad(portno, pinno, pad->direction);
		if (pad->retime)
			stm_pioalt_retime(portno, pinno, pad->retime, pad->direction);
	}
}


/* PIO retiming setup */

extern void stm_pioalt_retime(int port, const int pin,
		const struct stm_pio_control_retime_config * const cfg,
		const enum stm_pad_gpio_direction direction)
{
	unsigned long sysconf, *sysconfReg;
	unsigned innotout = 0;

#ifdef DEBUG_PAD_CONFIGS
	BUG_ON(!cfg);
	if (debug_pad_configs)
		printf("%s(port=%d, pin=%d, retime=%d, clk=%d, "
				"clknotdata=%d, double_edge=%d, invertclk=%d, "
				"delay=%d, direction=%s)\n", __func__, port, pin,
				cfg->retime, cfg->clk, cfg->clknotdata,
				cfg->double_edge, cfg->invertclk, cfg->delay,
				(direction==stm_pad_direction_input) ? "in" :
					(direction==stm_pad_direction_input_with_pullup) ? "in+pu" :
					(direction==stm_pad_direction_output) ? "out" :
					(direction==stm_pad_direction_bidir_no_pullup) ? "bidir" :
					(direction==stm_pad_direction_ignored) ? "ignore" :
					"BAD-BAD"
				);
	BUG_ON(pin < 0 || pin > 7);
	BUG_ON(cfg->retime < 0);	/* the "don't care" semantic is deprecated */
	BUG_ON(cfg->clk < 0);		/* the "don't care" semantic is deprecated */
	BUG_ON(cfg->clknotdata < 0);	/* the "don't care" semantic is deprecated */
	BUG_ON(cfg->double_edge < 0);	/* the "don't care" semantic is deprecated */
	BUG_ON(cfg->invertclk < 0);	/* the "don't care" semantic is deprecated */
	BUG_ON(cfg->delay < 0);		/* the "don't care" semantic is deprecated */
#endif

	switch (direction)
	{
	case stm_pad_direction_input:
	case stm_pad_direction_input_with_pullup:
		innotout = 1;
		break;
	case stm_pad_direction_output:
	case stm_pad_direction_bidir_no_pullup:
		innotout = 0;
		break;
	default:
		BUG();
		break;
	}

	switch (port)
	{
	case 0 ... 5:		/* in "SBC Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(100);
		sysconfReg += (port-0) * 8;
		break;
	case 10 ... 20:		/* in "FRONT Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(1100);
		sysconfReg += (port-10) * 8;
		break;
	case 30 ... 35:		/* in "REAR Bank" */
		sysconfReg = (unsigned long*)STXH410_SYSCFG(2100);
		sysconfReg += (port-30) * 8;
		break;
	case 40 ... 42:		/* in "FLASH Bank" */
		/* QQQ: for the time being, treat them as BUG()! */
	default:
		BUG();
		return;
	}

	sysconfReg += pin;

	/* read the "old" value from the system configuration register */
	sysconf = readl(sysconfReg);

	if (cfg->clk >= 0)
	{			/* map value to 2 adjacent bits */
		SET_SYSCONF_BITS(sysconf, TRUE, 0,1, cfg->clk,cfg->clk);
	}

	if (cfg->clknotdata >= 0)
	{
		SET_SYSCONF_BIT(sysconf, cfg->clknotdata, 2);
	}

	if (cfg->delay >= 0)
	{			/* map value to 4 adjacent bits */
		SET_SYSCONF_BITS(sysconf, TRUE, 3,6, cfg->delay,cfg->delay);
	}

	SET_SYSCONF_BIT(sysconf, innotout, 7);

	if (cfg->double_edge >= 0)
	{
		SET_SYSCONF_BIT(sysconf, cfg->double_edge, 8);
	}

	if (cfg->invertclk >= 0)
	{
		SET_SYSCONF_BIT(sysconf, cfg->invertclk, 9);
	}

	if (cfg->retime >= 0)
	{
		SET_SYSCONF_BIT(sysconf, cfg->retime, 10);
	}

	/* write the "new" value to the system configuration register */
	writel(sysconf, sysconfReg);
}


/*--------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------*/


extern int stmac_default_pbl(void)
{
	return 32;
}

#define GMAC_AHB_CONFIG		0x2000
void stm_ethernet_bus_setup(void)
{
	/* Configure the bridge to generate more efficient STBus traffic.
	 *
	 * Cut Version	| Ethernet AD_CONFIG[21:0]
	 *	1.0	|	0x00264207
	 */
	writel(0x00264207, CONFIG_SYS_STM_STMAC_BASE + GMAC_AHB_CONFIG);
}

extern int arch_cpu_init(void)
{
	stm_clocks();

	gd->arch.stm_devid = *STM_SYSCONF_DEVICEID;

	return 0;
}

extern void set_phy_speed_rmii_mode()
{
    unsigned int val;
    /*
     * Configure the PHY Clock:
     * CLKS_GMAC0_PHY (PIO13[5]) or CLKS_ETH1_PHY (PIO2[3]) should
     * be 50MHz for RMII mode (as appropriate).
     * Note: We rely on the GDB "pokes" to set the frequency for us ...
     */
    /* Set 50MHZ phy clock for RMII mode */
    val=readl(0x09103310);
    val=val&0xFE000000;
    /* 50Mhz */
    val=val|0x013a6666;
    writel(val,0x09103310);
}
struct stm_sdhci_info stm_sdhci_info[] = {
{
    .regbase = (void*)CONFIG_SYS_MMC0_BASE,
    .quirks = SDHCI_QUIRK_BROKEN_VOLTAGE | SDHCI_QUIRK_BROKEN_R1B | SDHCI_QUIRK_WAIT_SEND_CMD | SDHCI_QUIRK_32BIT_DMA_ADDR | SDHCI_QUIRK_NO_HISPD_BIT,
    .max_speed = 50*1000*1000,
    .min_speed = 400*1000,
    .voltages = MMC_VDD_165_195,
    .host_caps = MMC_MODE_DDR_52MHz,
},
{
    .regbase = (void*)CONFIG_SYS_MMC1_BASE,
    .quirks = SDHCI_QUIRK_BROKEN_VOLTAGE | SDHCI_QUIRK_BROKEN_R1B | SDHCI_QUIRK_WAIT_SEND_CMD | SDHCI_QUIRK_32BIT_DMA_ADDR | SDHCI_QUIRK_NO_HISPD_BIT,
    .max_speed = 50*1000*1000,
    .min_speed = 400*1000,
    .voltages = MMC_VDD_32_33 | MMC_VDD_33_34,
    .host_caps = MMC_MODE_DDR_52MHz,
},
};


#if defined(CONFIG_STM_IR)
extern int irb_init()
{
	static const struct {
		struct {
			unsigned char port, pin, alt;
		} mode;
	} ir_pins[] = {
		{ .mode = {  4, 0, 2 }},/*IR*/
		{ .mode = {  4, 1, 2 }},/*UHF*/
		{ .mode = {  4, 2, 2 }},/*TX*/
		{ .mode = {  4, 3, 2 }},/*TX_OD*/
	};

	/* Enable IRB in normal mode */
	unsigned long * const sysconfReg = (unsigned long*)STXH410_LPM_SYSCFG(1);
	unsigned long sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, TRUE, 6); /* Enable IRB */
	SET_SYSCONF_BIT(sysconf, FALSE, 7); /* Enable Normal mode */
	writel(sysconf, sysconfReg);

	/* Set the IRB in IR receiver mode */
	stm_pioalt_select(ir_pins[IRB_MODE_IR].mode.port,
			    ir_pins[IRB_MODE_IR].mode.pin,
			    ir_pins[IRB_MODE_IR].mode.alt);
	stm_pioalt_pad(ir_pins[IRB_MODE_IR].mode.port,
			  ir_pins[IRB_MODE_IR].mode.pin, stm_pad_direction_input);

	return 0;
}
#endif /*defined(CONFIG_STM_IR)*/
