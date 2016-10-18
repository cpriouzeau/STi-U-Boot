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
#include <asm/io.h>
#include <stm/pio.h>
#include <stm/stbus.h>
#include <stm/sysconf.h>
#include <stm/pad.h>
#include <stm/pio-control.h>
#include <netdev.h>

extern int stmac_eth_register(const int id,
		        const u32 base_addr);


/* ETH MAC pad configuration */
void stm_configure_ethernet(
	const struct stm_ethernet_config * const config)
{
	struct stm_pad_pin * pad_config;
	struct stm_pad_pin * phy_clock;
	const struct stm_pad_sysconf * sys_configs;
	size_t num_pads, num_sys;

	BUG_ON(!config);

	switch (config->mode) {
	case stm_ethernet_mode_mii:
		pad_config = ethernet_mii_pad_configs;
		num_pads = num_ethernet_mii_pad_configs;
		sys_configs = ethernet_mii_pad_sysconfs;
		num_sys = num_ethernet_mii_pad_sysconfs;
		phy_clock = stm_gmac_find_phy_clock(pad_config, num_pads);
		if (config->ext_clk)
			stm_pad_set_pio_ignored(phy_clock);
		else
			stm_pad_set_pio_out(phy_clock, 1);
		break;
	case stm_ethernet_mode_rmii:
		pad_config = ethernet_rmii_pad_configs;
		num_pads = num_ethernet_rmii_pad_configs;
		sys_configs = ethernet_rmii_pad_sysconfs;
		num_sys = num_ethernet_rmii_pad_sysconfs;

		/* we assume we *always* use the internal clock for RMII! */
		BUG_ON(config->ext_clk!= 0);

		set_phy_speed_rmii_mode();
		phy_clock = stm_gmac_find_phy_clock(pad_config, num_pads);
		stm_pad_set_pio_out(phy_clock, 1);
		break;
	case stm_ethernet_mode_rgmii:
		/* This mode is similar to GMII (GTX) except the data
		 * buses are reduced down to 4 bits and the 2 error
		 * signals are removed. The data rate is maintained by
		 * using both edges of the clock. This also explains
		 * the different retiming configuration for this mode.
		 */
		pad_config = ethernet_rgmii_pad_configs;
		num_pads = num_ethernet_rgmii_pad_configs;
		sys_configs = ethernet_rgmii_pad_sysconfs;
		num_sys = num_ethernet_rgmii_pad_sysconfs;
		phy_clock = stm_gmac_find_phy_clock(pad_config, num_pads);
		stm_pad_set_pio_out(phy_clock, 4);
		break;

	case stm_ethernet_mode_reverse_mii:
		BUG();		/* assume not required in U-Boot */
		return;

	case stm_ethernet_mode_gmii:
		BUG();		/* assume not required in U-Boot */
		return;

	default:
		BUG();
		return;
	}

		/* now configure all the PIOs */
	stm_configure_pios(pad_config, num_pads);

		/* now configure the relevant SYS_CONFIGs */
	stm_configure_sysconfs(sys_configs, num_sys);

		/* need to do this *after* the SYSCONF initialization! */
	stm_ethernet_bus_setup();
}


	/*
	 * SoC-specific function to register all ST-MAC/GMAC controllers.
	 * The function "board_eth_init()" should ideally be defined
	 * for *each* board.  However, "cpu_eth_init()" can be used
	 * as a generic fall-back, for all boards of a given CPU type.
	 * In any event, "board_eth_init()" will have a higher priority,
	 * so define it, if you want to override the following SoC function.
	 */
int cpu_eth_init(bd_t * const bis)
{
	stmac_eth_register(1, CONFIG_SYS_STM_STMAC_BASE);

	return 0;
}

