/*
 * (C) Copyright 2008-2014 STMicroelectronics.
 *
 * Sean McGoogan <Sean.McGoogan@st.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#ifndef __INCLUDE_STM_SOC_H
#define __INCLUDE_STM_SOC_H


#include <stm/pad.h>
#include <stm/sysconf.h>
#include <stm/stxh410reg.h>

#ifndef CONFIG_BOARDREV_STRING
#define CONFIG_BOARDREV_STRING "revx"
#endif
struct pio{
	const unsigned char port, pin;
	unsigned char alt;
};

struct stm_uart_config {
	struct pio output;
	struct pio input;
};
struct stm_mmc_pio_getcd {
    int pio_port;
    int pio_pin;
};
extern void stm_pioalt_pad(
	int port,
	const int pin,
	const enum stm_pad_gpio_direction direction);
extern void stm_pioalt_select(
	const int port,
	const int pin,
	const int alt);
extern void stm_pioalt_retime(
	const int port,
	const int pin,
	const struct stm_pio_control_retime_config * const cfg,
	const enum stm_pad_gpio_direction direction);
#define STM_PIOALT_SELECT(PAIR, ALT) stm_pioalt_select(PAIR, (ALT))
struct ssc_pios
{
        struct
        {
                char port;
                char pin;
        }       pio[2];
};
const extern struct ssc_pios ssc_pios[];
/*
 * STMAC data types
 */
struct stm_ethernet_config {
	enum {
		stm_ethernet_mode_mii,
		stm_ethernet_mode_gmii,
		stm_ethernet_mode_rmii,
		stm_ethernet_mode_rgmii,
		stm_ethernet_mode_reverse_mii
	} mode;
	int ext_clk : 1;	/* boolean */
	int phy_bus : 5;	/* 0x00 .. 0x1f */
};


/*
 * common call-back functions for STMAC.
 */
extern int  stmac_default_pbl (void);
extern void stmac_set_mac_speed (int speed);
extern void stmac_phy_reset (void);

/*
 * IRB initialization functions.
 */
extern int irb_init(void);

/*
 * STMAC initialization functions.
 */
extern void stm_configure_ethernet(
	const struct stm_ethernet_config * const config);

/*
 * SATA initialization functions.
 */
extern void stm_sata_miphy_init(void);
extern void stm_sata_miphy_deassert_des_reset(void);
extern int  stm_sata_probe(void);


/*
 * USB initialization functions.
 */
extern int  stm_usb_init(const int port);
extern int  stm_usb_stop(const int port);


/*
 * SPI initialization functions.
 */
extern void		stm_configure_spi(void);


/*
 * Software "bit-banging" functions for SPI accesses.
 */
extern void		stm_spi_scl(const int val);
extern void		stm_spi_sda(const int val);
extern unsigned char	stm_spi_read(void);

/*
 * I2C initialization functions.
 */
extern void		stm_configure_i2c(void);


/*
 * Software "bit-banging" functions for I2C accesses.
 */
extern void		stm_i2c_scl(const int val);
extern void		stm_i2c_sda(const int val);
extern int		stm_i2c_read(void);

/*
 * NAND initialization functions.
 */
extern void		stm_configure_nand(void);


/*
 * MMC/SD initialization functions.
 */
extern int stm_mmc_getcd(const int port);


/*
 * Common functions for STMicroelectronics' SoCs.
 */
extern int arch_cpu_init(void);
extern void switch_on_leddec(void);
extern int switch_on_cm_led(void);
extern int switch_off_cm_led(void);

extern void set_phy_speed_rmii_mode(void);
extern void stm_uart_init(struct stm_uart_config *stm_uart_config);
void stm_configure_pios(
        const struct stm_pad_pin * const pad_config,
        const size_t num_pads);
extern void stm_ethernet_bus_setup(void);
extern int ssc_pios_array_size;
extern void stm_configure_spi(void);
extern struct stm_mmc_pio_getcd mmc_pio_getcd[];
extern const struct stm_pad_pin stm_mmc_pad_configs[][10];
void set_mmc_pulup_config(int port );

extern struct stm_pad_pin ethernet_mii_pad_configs[];
extern struct stm_pad_sysconf ethernet_mii_pad_sysconfs[];
extern struct stm_pad_pin ethernet_rgmii_pad_configs[];
extern struct stm_pad_sysconf ethernet_rgmii_pad_sysconfs[];
extern struct stm_pad_pin ethernet_rmii_pad_configs[];
extern struct stm_pad_sysconf ethernet_rmii_pad_sysconfs[];
extern struct stm_uart_config stm_uart_config[];
extern struct stm_mmc_pio_getcd mmc_pio_getcd[];
extern const struct stm_pad_pin stm_mmc_pad_configs[][10];
extern const struct ssc_pios ssc_pios[];
extern int num_ethernet_mii_pad_configs;
extern int num_ethernet_mii_pad_sysconfs;
extern int num_ethernet_rgmii_pad_configs;
extern int num_ethernet_rgmii_pad_sysconfs;
extern int num_ethernet_rmii_pad_configs;
extern int num_ethernet_rmii_pad_sysconfs;
extern struct stm_pad_pin stm_usb3_pad_configs[];
extern struct stm_pad_pin stm_usb2_pad_configs[];
extern struct stm_pad_pin stm_usb3_pad_deconfigs[];
extern struct stm_pad_pin stm_usb2_pad_deconfigs[];

extern int num_stm_usb3_pad_configs;
extern int num_stm_usb2_pad_configs;
extern int num_stm_usb3_pad_deconfigs;
extern int num_stm_usb2_pad_deconfigs;

#include <stm/stm-sdhci.h>
extern int stm_sdhci_init(const int port, struct stm_sdhci_info *);
extern struct stm_sdhci_info stm_sdhci_info[];
#endif	/* __INCLUDE_STM_SOC_H */

