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
#include <asm/io.h>
#include <stm/pio.h>
#include <stm/stbus.h>
#include <stm/sysconf.h>
#include <stm/pad.h>
#include <stm/pio-control.h>
#include <stm/stm-sdhci.h>
#include <mmc.h>
#include <ata.h>
#include <spi.h>
#include <netdev.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_STM_SDHCI)


	/*
	 * Returns the *raw* "CD" (Card Detect) siginal
	 * for the appropriate MMC port #.
	 * Note: The PIO should have a pull-up attached.
	 * returns:	0 for card present, 1 for no card.
	 */
extern int stm_mmc_getcd(const int port)
{
	if(mmc_pio_getcd[port].pio_port)
		return STPIO_GET_PIN(STM_PIO_BASE(mmc_pio_getcd[port].pio_port), mmc_pio_getcd[port].pio_pin);
	else
	    return 0;
}
#if defined(CONFIG_STM_STXH410)
static u32 force_load_address(volatile u32 * volatile ptr)
{
       // ptr is volatile to tell the compiler not to optimize anything around ptr :
       //   no assumptions should be done in compiler about ptr values.

       // *ptr is volatile to tell the compiler not to optimize anything around *ptr :
       //   even if this value is known (already loaded), the load should be performed.

       return *ptr;
}
#endif

static void stm_enable_mmc(const int port, const u32 regbase)
{
	if (port != 0)
	{
		unsigned long sysconf = readl(SYSCONF(SYSCONF_MMC_ENABLE_MMC1));
		SET_SYSCONF_BIT(sysconf,1,SYSCONF_MMC1_ENABLE_BIT);
		writel(sysconf, SYSCONF(SYSCONF_MMC_ENABLE_MMC1));
	}
	else	/* MMC #0 (i.e. the eMMC bootable device) */
	{
		unsigned long bootif = readl(SYSCONF(SYSCONF_BOOT_MODE));
		printf("%s:",((bootif >> 10) & 1)?"Fast boot":"boot");
                if (((bootif >> 4) & 0x7) == 0x7)
		{
		    printf("eMMC: ");
		    bootif = (bootif & 0xc) >> 2;
		    switch (bootif)
		    {
			case 0x0:
			    printf("%s\n", "8xbit - div4");
			    goto boot_disable;
			    break;
			case 0x1:
			    printf("%s\n", "8xbit - div2");
			    goto boot_disable;
			    break;
			case 0x2:
			    printf("%s\n", "1xbit - div4");
			    goto boot_disable;
			    break;
			case 0x3:
			    printf("%s\n", "1xbit - div2");
			    goto boot_disable;
			    break;
			default:
			    return;
		    }
		} else {
		    printf(" non-eMMC.\n");
		    return;
		}
boot_disable:
		/*
		 * When we are booting from eMMC, then the H/W has a
		 * requirement that there must be at least one eMMC
		 * boot-mode "transaction", before we can initialize
		 * the eMMC driver safely.
		 * Hence, when in boot-from-eMMC mode, we will perform
		 * a single "dummy" read to the boot region. We simply
		 * read from physical address zero, and discard the
		 * result.  This read only needs to be performed once.
		 * This is only really an issue when booting via GDB/JTAG,
		 * and when the mode-pins are in boot-from-eMMC mode,
		 * when the boot-controller has not been exercised yet.
		 */
#if defined(CONFIG_STM_STXH410)
		force_load_address(0x0);; /* timeout FW */
		force_load_address(0x0);; /* timeout FW */
#endif /* CONFIG_STM_STXH410 */
		/* Now boot disable is safe*/
		writel( (readl(regbase + TOP_FLASHSS_CONFIG) & (~TOP_FLASHSS_CONFIG_CFG_EMMC_NOT_EMI)),\
			regbase + TOP_FLASHSS_CONFIG);

	}
}

/**
 * stm_mmc_core_config: configure the Arasan HC
 * @ioaddr: base address
 * Description: this function is to configure the Arasan MMC HC.
 * This should be called when the system starts in case of, on the SoC,
 * it is needed to configure the host controller.
 * This happens on some SoCs, i.e. StiH410, where the MMC0 inside the flashSS
 * needs to be configured as MMC 4.5 to have full capabilities.
 * W/o these settings the SDHCI could configure and use the embedded controller
 * with limited features.
 */
static void stm_mmc_core_config(const int port, const u32 regbase)
{
#ifdef MMC_CORE_DEBUG
	printf("\nmmc%d: %x core config in default ...\n", port, regbase);
	printf("cfg1 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_1));
	printf("cfg2 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_2));
	printf("cfg3 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_3));
	printf("cfg4 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_4));
	printf("cfg5 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_5));
	printf("cfg6 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_6));
	printf("cfg7 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_7));
	printf("cfg8 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_8));
#endif /*MMC_CORE_DEBUG*/

	writel(STM_FLASHSS_MMC_CORE_CONFIG_1, regbase + FLASHSS_MMC_CORE_CONFIG_1);

	(port)? \
		writel(STM_FLASHSS_SDCARD_CORE_CONFIG2, regbase + FLASHSS_MMC_CORE_CONFIG_2): \
		writel(STM_FLASHSS_MMC_CORE_CONFIG2, regbase + FLASHSS_MMC_CORE_CONFIG_2);

	(port)? \
		writel(STM_FLASHSS_SDCARD_CORE_CONFIG3, regbase + FLASHSS_MMC_CORE_CONFIG_3): \
		writel(STM_FLASHSS_MMC_CORE_CONFIG3, regbase + FLASHSS_MMC_CORE_CONFIG_3);

	writel(STM_FLASHSS_MMC_CORE_CONFIG4,regbase + FLASHSS_MMC_CORE_CONFIG_4);

#ifdef MMC_CORE_DEBUG
	printf("mmc%d: %x: core config ...\n", port, regbase);
	printf("cfg1 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_1));
	printf("cfg2 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_2));
	printf("cfg3 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_3));
	printf("cfg4 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_4));
	printf("cfg5 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_5));
	printf("cfg6 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_6));
	printf("cfg7 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_7));
	printf("cfg8 0x%x\n", readl(regbase + FLASHSS_MMC_CORE_CONFIG_8));
#endif /*MMC_CORE_DEBUG*/
}

static void stm_configure_mmc(const int port, const u32 regbase)
{
	const struct stm_pad_pin * pad_config;
	size_t num_pads;

	stm_enable_mmc(port, regbase);
	stm_mmc_core_config(port, regbase);
	pad_config = stm_mmc_pad_configs[port];
	num_pads = ARRAY_SIZE(stm_mmc_pad_configs[port]);
		/* now configure all the PIOs */
	stm_configure_pios(pad_config, num_pads);
	set_mmc_pulup_config(port);
}
extern int cpu_mmc_init(bd_t *bis)
{
	int ret = 0;


#if defined(CONFIG_STM_SDHCI_0)
	stm_configure_mmc(0, CONFIG_SYS_MMC0_BASE);
	ret |= stm_sdhci_init(0, &stm_sdhci_info[0]);
#endif	/* CONFIG_STM_SDHCI_0 */

#if defined (CONFIG_STM_SDHCI_1)
	stm_configure_mmc(1, CONFIG_SYS_MMC1_BASE);
	ret |= stm_sdhci_init(1, &stm_sdhci_info[1]);
#endif	/* CONFIG_STM_SDHCI_1 */

	return ret;
}
void set_mmc_pulup_config(int port )
{
    if(port)
	return;
    /*
     * For booting, the standard requires the CMD and DATAx
     * lines all have a pull-up attached. As a cost saving,
     * these pull-ups might not be on the board, so we will
     * explicitly enable the pad's pull-ups in the SoC.
     */
    unsigned long sysconf = readl(SYSCONF(SYSCONF_MMC0_PIO_CONF));
#ifdef MMC_CORE_DEBUG
    printf("mmc%d: pull up config at reset 0x%lx\n", port, sysconf);
#endif
    SET_SYSCONF_BIT(sysconf,1,7);	/* CMD */
    SET_SYSCONF_BIT(sysconf,1,8);	/* DATA0 */
    SET_SYSCONF_BIT(sysconf,1,9);	/* DATA1 */
    SET_SYSCONF_BIT(sysconf,1,10);	/* DATA2 */
    SET_SYSCONF_BIT(sysconf,1,11);	/* DATA3 */
    SET_SYSCONF_BIT(sysconf,1,12);	/* DATA4 */
    SET_SYSCONF_BIT(sysconf,1,13);	/* DATA5 */
    SET_SYSCONF_BIT(sysconf,1,14);	/* DATA6 */
    SET_SYSCONF_BIT(sysconf,1,15);	/* DATA7 */
    
    writel(sysconf, SYSCONF(SYSCONF_MMC0_PIO_CONF));
#ifdef MMC_CORE_DEBUG
    printf("mmc%d: pull up config 0x%x\n", port, readl(SYSCONF(SYSCONF_MMC0_PIO_CONF)));
#endif
#endif	/* CONFIG_STM_SDHCI_0 */
}



