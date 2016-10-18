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
#ifdef CONFIG_DM
#include <dm.h>
#ifdef CONFIG_DM_USB
#include <dm/platform_data/ehci_stm.h>
#include <dm/platform_data/xhci_stm.h>
#endif
#endif
#include <stm/soc.h>
#include <asm/io.h>
#include <stm/pio.h>
#include <stm/stbus.h>
#include <stm/sysconf.h>
#include <stm/pad.h>
#include <stm/soc.h>
#include <stm/pio-control.h>


/* Reg glue registers */
#define USB2_CLKRST_CTRL 0x00
#define aux_clk_en(n) ((n)<<0)
#define sw_pipew_reset_n(n) ((n)<<4)
#define ext_cfg_reset_n(n) ((n)<<8)
#define xhci_revision(n) ((n)<<12)

#define USB2_VBUS_MNGMNT_SEL1 0x2C

/*
 * 2'b00 : Override value from Reg 0x30 is selected
 * 2'b01 : utmiotg_vbusvalid from usb3_top top is selected
 * 2'b10 : pipew_powerpresent from PIPEW instance is selected
 * 2'b11 : value is 1'b0
 */
#define SEL_OVERRIDE_VBUSVALID(n) ((n)<<0)
#define SEL_OVERRIDE_POWERPRESENT(n) ((n)<<4)
#define SEL_OVERRIDE_BVALID(n) ((n)<<8)

#define USB2_VBUS_MNGMNT_VAL1 0x30
#define OVERRIDE_VBUSVALID_VAL (1 << 0)
#define OVERRIDE_POWERPRESENT_VAL (1 << 4)
#define OVERRIDE_BVALID_VAL (1 << 8)


#define MIPHY_VERSION   0xfe
#define MIPHY_REVISION  0xff

#define p4_inb(addr)        __raw_readb(addr)

static void stm_miphy_pipe_write(u32 addr, u32 data)
{
	writeb(data & 0xff,  MIPHY_USB3_PIPEW_BASE + addr);
}

static void stm_miphy_write(u8 addr, u8 data)
{
	void __iomem *base;
	base = (unsigned char*)( MIPHY_USB3_UPORT_BASE + addr);
	writeb(data , (volatile unsigned char *) base);
}

static void miphy_setup(void)
{
	/* Putting Macro in reset */
	stm_miphy_write( 0x00, 0x01);
	stm_miphy_write( 0x00, 0x03);

	/* Wait for a while */
	stm_miphy_write( 0x00, 0x01);
	stm_miphy_write( 0x04, 0x1C);

	/* PLL calibration */
	stm_miphy_write( 0xEB, 0x1D);
	stm_miphy_write( 0x0D, 0x1E);
	stm_miphy_write( 0x0F, 0x00);
	stm_miphy_write( 0xC4, 0x70);
	stm_miphy_write( 0xC9, 0x02);
	stm_miphy_write( 0xCA, 0x02);
	stm_miphy_write( 0xCB, 0x02);
	stm_miphy_write( 0xCC, 0x0A);

	/* Writing The PLL Ratio */
	stm_miphy_write( 0xD4, 0xA6);
	stm_miphy_write( 0xD5, 0xAA);
	stm_miphy_write( 0xD6, 0xAA);
	stm_miphy_write( 0xD7, 0x04);
	stm_miphy_write( 0xD3, 0x00);

	/* Writing The Speed Rate */
	stm_miphy_write( 0x0F, 0x00);
	stm_miphy_write( 0x0E, 0x0A);

	/* Rx Channel compensation and calibration */
	stm_miphy_write( 0xC2, 0x1C); /* Disable AUto Calibration for DC gain */
	stm_miphy_write( 0x97, 0x51); /* Enable Fine Cal */
	stm_miphy_write( 0x98, 0x70); /* Enable Fine Cal */
	stm_miphy_write( 0x99, 0x5F);
	stm_miphy_write( 0x9A, 0x22);
	stm_miphy_write( 0x9F, 0x0E); /* Enable Fine Cal */
	stm_miphy_write( 0x7A, 0x05); /* VGA GAIN, EQ GAIN set manaually */
	stm_miphy_write( 0x7F, 0x78); /* EQU_GAIN_MAN = +5dB  */

	stm_miphy_write( 0x30, 0x1B);

	/* Enable GENSEL_SEL and SSC */
	/* TX_SEL=0 swing preemp forced by pipe registres */
	stm_miphy_write( 0x0A, 0x11);

	/* MIPHY Bias boost */
	stm_miphy_write( 0x63, 0x00);
	stm_miphy_write( 0x64, 0xA7);

	/* TX compensation offset to re-center TX impedance */
	stm_miphy_write( 0x42, 0x02);

	/* SSC modulation */
	stm_miphy_write( 0x0C, 0x04);

	/* Enable RX autocalibration */
	stm_miphy_write( 0x2B, 0x01);

	/* MIPHY TX control */
	stm_miphy_write(0x0F, 0x00); 
	stm_miphy_write(0xE5, 0x5A); 
	stm_miphy_write(0xE6, 0xA0); 
	stm_miphy_write(0xE4, 0x3C);
	stm_miphy_write(0xE6, 0xA1); 
	stm_miphy_write(0xE3, 0x00); 
	stm_miphy_write(0xE3, 0x02); 
	stm_miphy_write(0xE3, 0x00);

	/* Rx PI controller settings */
	stm_miphy_write(0x78, 0xCA);

	/* MIPHY RX input bridge control*/
	/* INPUT_BRIDGE_EN_SW=1, manual input bridge control[0]=1 */
	stm_miphy_write( 0xCD, 0x21);
	stm_miphy_write( 0xCD, 0x29);
	stm_miphy_write( 0xCE, 0x1A);

	/* MIPHY Reset*/
	stm_miphy_write( 0x00, 0x01);
	stm_miphy_write( 0x00, 0x00); /* RST_APPLI_SW=0 */
	stm_miphy_write( 0x01, 0x04);
	stm_miphy_write( 0x01, 0x05);
	stm_miphy_write( 0xE9, 0x00);
	stm_miphy_write( 0x0D, 0x1E);
	stm_miphy_write( 0x3A, 0x40);
	stm_miphy_write( 0x01, 0x01);
	stm_miphy_write( 0x01, 0x00);
	stm_miphy_write( 0xE9, 0x40);
	stm_miphy_write( 0x0F, 0x00);
	stm_miphy_write( 0x0B, 0x00);
	stm_miphy_write( 0x62, 0x00);
	stm_miphy_write( 0x0F, 0x00);
	stm_miphy_write( 0xE3, 0x02);
	stm_miphy_write( 0x26, 0xa5);
	stm_miphy_write( 0x0F, 0x00);

	/* PIPE Wrapper Configuration */
	stm_miphy_pipe_write(0x23, 0X68);
	stm_miphy_pipe_write(0x24, 0X61);
	stm_miphy_pipe_write(0x26, 0X68);
	stm_miphy_pipe_write(0x27, 0X61);
	stm_miphy_pipe_write( 0x29, 0X18);
	stm_miphy_pipe_write( 0x2A, 0X61);

	/*pipe Wrapper usb3 TX swing de-emph margin PREEMPH[7:4], SWING[3:0] */
	stm_miphy_pipe_write( 0x68, 0x67); /* margin 0 */
	stm_miphy_pipe_write( 0x69, 0x0D); /* margin 1 */
	stm_miphy_pipe_write( 0x6A, 0x67); /* margin 2 */
	stm_miphy_pipe_write( 0x6B, 0x0D); /* margin 3 */
	stm_miphy_pipe_write( 0x6C, 0x67); /* margin 4 */
	stm_miphy_pipe_write( 0x6D, 0x0D); /* margin 5 */
	stm_miphy_pipe_write( 0x6E, 0x67); /* margin 6 */
	stm_miphy_pipe_write( 0x6F, 0x0D); /* margin 7 */
}

static inline u32 dwc3_stm_readl(u32 offset)
{
	return readl(STM_DWC3_GLUE_BASE + offset);
}

static inline void dwc3_stm_writel(u32 offset, u32 value)
{
    writel(value, STM_DWC3_GLUE_BASE + offset);
}

static void st_dwc3_init(void)
{
	u32 reg;

	reg = dwc3_stm_readl(USB2_CLKRST_CTRL);

	reg |= aux_clk_en(1) | ext_cfg_reset_n(1) | xhci_revision(1);
	reg &= ~sw_pipew_reset_n(1);
	dwc3_stm_writel(USB2_CLKRST_CTRL, reg);

	reg = dwc3_stm_readl(USB2_VBUS_MNGMNT_SEL1);
	reg |= SEL_OVERRIDE_VBUSVALID(1) | SEL_OVERRIDE_POWERPRESENT(1) |
	SEL_OVERRIDE_BVALID(1);

	dwc3_stm_writel(USB2_VBUS_MNGMNT_SEL1, reg);
	udelay(100);

	reg = dwc3_stm_readl(USB2_CLKRST_CTRL);
	reg |= sw_pipew_reset_n(1);

	dwc3_stm_writel(USB2_CLKRST_CTRL, reg);

	/* Use this delay to adjust for MiPHY HFCready assertion variation */
	udelay(1000);

}

static void st_dwc3_stop(void)
{
	u32 reg;

	reg = dwc3_stm_readl(USB2_CLKRST_CTRL);
	reg &= ~sw_pipew_reset_n(1);
	dwc3_stm_writel(USB2_CLKRST_CTRL, reg);

	reg = dwc3_stm_readl(USB2_VBUS_MNGMNT_SEL1);
	reg &= ~SEL_OVERRIDE_VBUSVALID(1); 
	reg &= ~SEL_OVERRIDE_POWERPRESENT(1);
        reg &= ~SEL_OVERRIDE_BVALID(1);
	dwc3_stm_writel(USB2_VBUS_MNGMNT_SEL1, reg);

	reg = dwc3_stm_readl(USB2_CLKRST_CTRL);
	reg &= ~aux_clk_en(1); 
	reg &= ~ext_cfg_reset_n(1);
	reg &= ~xhci_revision(1);
	reg |= sw_pipew_reset_n(1);
	dwc3_stm_writel(USB2_CLKRST_CTRL, reg);

	/* Use this delay to adjust for MiPHY HFCready assertion variation */
	udelay(1000);
}

void stm_usb3_deassert(void)
{
	unsigned long * sysconfReg ;
	unsigned long sysconf ;

	/* Power up USB3 port..*/
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5001);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 6);
	writel(sysconf, sysconfReg);
    	udelay(1000);

}

void stm_usb3_assert(void)
{
	unsigned long * sysconfReg ;
	unsigned long sysconf ;

	/* Power down USB3 port..*/
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5001);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 6);
	writel(sysconf, sysconfReg);
    	udelay(1000);

}

static void deconfigure_usb3_pio(void)
{
	struct stm_pad_pin * pad_config;
	size_t num_pads;
	pad_config = stm_usb3_pad_deconfigs;
	num_pads = num_stm_usb3_pad_deconfigs;
		/* now configure all the PIOs */
	stm_configure_pios(pad_config, num_pads);
}

static void deconfigure_usb2_pio(void)
{
	struct stm_pad_pin * pad_config;
	size_t num_pads;
	pad_config = stm_usb2_pad_deconfigs;
	num_pads = num_stm_usb2_pad_deconfigs;
		/* now configure all the PIOs */
	stm_configure_pios(pad_config, num_pads);
}

static void configure_usb3_pio(void)
{
	struct stm_pad_pin * pad_config;
	size_t num_pads;
	pad_config = stm_usb3_pad_configs;
	num_pads = num_stm_usb3_pad_configs;
		/* now configure all the PIOs */
	stm_configure_pios(pad_config, num_pads);
}

static void configure_usb2_pio(void)
{
	struct stm_pad_pin * pad_config;
	size_t num_pads;
	pad_config = stm_usb2_pad_configs;
	num_pads = num_stm_usb2_pad_configs;
		/* now configure all the PIOs */
	stm_configure_pios(pad_config, num_pads);
}

static void stm_picophy_assert(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 5);
	SET_SYSCONF_BIT(sysconf, 1, 6);
	SET_SYSCONF_BIT(sysconf, 1, 7);
	writel(sysconf, sysconfReg);
}

static void stm_picophy_deassert(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 5);
	SET_SYSCONF_BIT(sysconf, 0, 6);
	SET_SYSCONF_BIT(sysconf, 0, 7);
	writel(sysconf, sysconfReg);
}

static void stm_miphy_deassert(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 22);
	writel(sysconf, sysconfReg);
}

static void stm_miphy_assert(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 22);
	writel(sysconf, sysconfReg);
}

static void usb3_miphy_init(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5071);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 2);
	SET_SYSCONF_BIT(sysconf, 0, 1);
	SET_SYSCONF_BIT(sysconf, 0, 0);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 22);
	writel(sysconf, sysconfReg);
}

static void usb3_miphy_stop(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 22);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5071);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 2);
	SET_SYSCONF_BIT(sysconf, 1, 1);
	SET_SYSCONF_BIT(sysconf, 1, 0);
	writel(sysconf, sysconfReg);
}
#if defined(CONFIG_STM_STXH301)
static void stm_usb2_deassert1(void)
{
        unsigned long *sysconfReg;
        unsigned long sysconf;

        /* Power up..*/
        sysconfReg = (unsigned long*)STXH410_SYSCFG(5001);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf, 0, 5); /* usb2_1 */
        writel(sysconf, sysconfReg);
        udelay(1000);

        sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf, 1, 29);
        writel(sysconf, sysconfReg);

        /*sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf, 1, 30);
        writel(sysconf, sysconfReg);*/

        sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
        sysconf = readl(sysconfReg);

        /*PICOPHY_REFCLKSEL*/
        SET_SYSCONF_BIT(sysconf,0 , 0);
        SET_SYSCONF_BIT(sysconf,1 , 1);
        writel(sysconf, sysconfReg);

        /* PICOPHY_FSEL */
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 2);
        SET_SYSCONF_BIT(sysconf,0 , 3);
        SET_SYSCONF_BIT(sysconf,0 , 4);
        writel(sysconf, sysconfReg);

        /* Port parameters override */
        sysconfReg = (unsigned long*)STXH410_SYSCFG(5063);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 0);
        SET_SYSCONF_BIT(sysconf,0 , 1);
        SET_SYSCONF_BIT(sysconf,1 , 2);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 3);
        SET_SYSCONF_BIT(sysconf,1 , 4);
        SET_SYSCONF_BIT(sysconf,0 , 5);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 6);
        SET_SYSCONF_BIT(sysconf,1 , 7);
        SET_SYSCONF_BIT(sysconf,0 , 8);
        SET_SYSCONF_BIT(sysconf,0 , 9);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 10);
        SET_SYSCONF_BIT(sysconf,0 , 11);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 12);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 13);
        SET_SYSCONF_BIT(sysconf,0 , 14);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 15);
        SET_SYSCONF_BIT(sysconf,1 , 16);
        SET_SYSCONF_BIT(sysconf,0 , 17);
        SET_SYSCONF_BIT(sysconf,0 , 18);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 19);
        SET_SYSCONF_BIT(sysconf,1 , 20);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 21);
        SET_SYSCONF_BIT(sysconf,0 , 22);
        writel(sysconf, sysconfReg);

        /* Start port1 */
        sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 6);
        writel(sysconf, sysconfReg);
}
#endif
static void stm_usb2_deassert(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	/* Power up..*/
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5001);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 4); /* usb2_0 */
	SET_SYSCONF_BIT(sysconf, 0, 5); /* usb2_1 */
	writel(sysconf, sysconfReg);
    	udelay(1000);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 28);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 29);
	writel(sysconf, sysconfReg);

	/*sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 30);
	writel(sysconf, sysconfReg);*/

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	
	/*PICOPHY_REFCLKSEL*/
	SET_SYSCONF_BIT(sysconf,0 , 0);
	SET_SYSCONF_BIT(sysconf,1 , 1);
	writel(sysconf, sysconfReg);

	/* PICOPHY_FSEL */
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 2);
	SET_SYSCONF_BIT(sysconf,0 , 3);
	SET_SYSCONF_BIT(sysconf,0 , 4);
	writel(sysconf, sysconfReg);

	/* Port parameters override for port usb2_0 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5062);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 0);
	SET_SYSCONF_BIT(sysconf,0 , 1);
	SET_SYSCONF_BIT(sysconf,1 , 2);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 3);
	SET_SYSCONF_BIT(sysconf,1 , 4);
	SET_SYSCONF_BIT(sysconf,0 , 5);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 6);
	SET_SYSCONF_BIT(sysconf,1 , 7);
	SET_SYSCONF_BIT(sysconf,0 , 8);
	SET_SYSCONF_BIT(sysconf,0 , 9);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 10);
	SET_SYSCONF_BIT(sysconf,0 , 11);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 12);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 13);
	SET_SYSCONF_BIT(sysconf,0 , 14);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 15);
	SET_SYSCONF_BIT(sysconf,1 , 16);
	SET_SYSCONF_BIT(sysconf,0 , 17);
	SET_SYSCONF_BIT(sysconf,0 , 18);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 19);
	SET_SYSCONF_BIT(sysconf,1 , 20);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 21);
	SET_SYSCONF_BIT(sysconf,0 , 22);
	writel(sysconf, sysconfReg);


	/* Port parameters override for port usb2_1 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5063);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 0);
	SET_SYSCONF_BIT(sysconf,0 , 1);
	SET_SYSCONF_BIT(sysconf,1 , 2);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 3);
	SET_SYSCONF_BIT(sysconf,1 , 4);
	SET_SYSCONF_BIT(sysconf,0 , 5);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 6);
	SET_SYSCONF_BIT(sysconf,1 , 7);
	SET_SYSCONF_BIT(sysconf,0 , 8);
	SET_SYSCONF_BIT(sysconf,0 , 9);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 10);
	SET_SYSCONF_BIT(sysconf,0 , 11);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 12);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 13);
	SET_SYSCONF_BIT(sysconf,0 , 14);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 15);
	SET_SYSCONF_BIT(sysconf,1 , 16);
	SET_SYSCONF_BIT(sysconf,0 , 17);
	SET_SYSCONF_BIT(sysconf,0 , 18);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 19);
	SET_SYSCONF_BIT(sysconf,1 , 20);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 21);
	SET_SYSCONF_BIT(sysconf,0 , 22);
	writel(sysconf, sysconfReg);

	/* Start port0 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 5);
	writel(sysconf, sysconfReg);

	/* Start port1 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 6);
	writel(sysconf, sysconfReg);
}
#if defined(CONFIG_STM_STXH301)
static void stm_usb2_assert1(void)
{
        unsigned long *sysconfReg;
        unsigned long sysconf;

        /* Stop port1 */
        /*sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 6);
        writel(sysconf, sysconfReg);*/

        /* Reset port parameters override */
        sysconfReg = (unsigned long*)STXH410_SYSCFG(5063);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 21);
        SET_SYSCONF_BIT(sysconf,1 , 22);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 19);
        SET_SYSCONF_BIT(sysconf,0 , 20);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 15);
        SET_SYSCONF_BIT(sysconf,0 , 16);
        SET_SYSCONF_BIT(sysconf,1 , 17);
        SET_SYSCONF_BIT(sysconf,1 , 18);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 13);
        SET_SYSCONF_BIT(sysconf,1 , 14);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 12);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 10);
        SET_SYSCONF_BIT(sysconf,1 , 11);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 6);
        SET_SYSCONF_BIT(sysconf,0 , 7);
        SET_SYSCONF_BIT(sysconf,1 , 8);
        SET_SYSCONF_BIT(sysconf,1 , 9);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 3);
        SET_SYSCONF_BIT(sysconf,0 , 4);
        SET_SYSCONF_BIT(sysconf,1 , 5);
        writel(sysconf, sysconfReg);

        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,1 , 0);
        SET_SYSCONF_BIT(sysconf,1 , 1);
        SET_SYSCONF_BIT(sysconf,0 , 2);
        writel(sysconf, sysconfReg);

        sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
        sysconf = readl(sysconfReg);

        /* PICOPHY_FSEL */
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf,0 , 2);
        SET_SYSCONF_BIT(sysconf,1 , 3);
        SET_SYSCONF_BIT(sysconf,1 , 4);
        writel(sysconf, sysconfReg);

        /*PICOPHY_REFCLKSEL*/
        SET_SYSCONF_BIT(sysconf,1 , 0);
        SET_SYSCONF_BIT(sysconf,0 , 1);
        writel(sysconf, sysconfReg);

        sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf, 0, 30);
        writel(sysconf, sysconfReg);

        sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf, 0, 29);
        writel(sysconf, sysconfReg);

        /* Power down..*/
        sysconfReg = (unsigned long*)STXH410_SYSCFG(5001);
        sysconf = readl(sysconfReg);
        SET_SYSCONF_BIT(sysconf, 1, 5);
        writel(sysconf, sysconfReg);
        udelay(1000);
}
#endif
static void stm_usb2_assert(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	/* Power down port... */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 4); /* usb2_0 */
	SET_SYSCONF_BIT(sysconf,1 , 5); /* usb2_1 */
	writel(sysconf, sysconfReg);

	/* Stop port1 */
	/*sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 6);
	writel(sysconf, sysconfReg);*/

	/* Reset port parameters override for usb2_1 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5063);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 21);
	SET_SYSCONF_BIT(sysconf,1 , 22);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 19);
	SET_SYSCONF_BIT(sysconf,0 , 20);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 15);
	SET_SYSCONF_BIT(sysconf,0 , 16);
	SET_SYSCONF_BIT(sysconf,1 , 17);
	SET_SYSCONF_BIT(sysconf,1 , 18);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 13);
	SET_SYSCONF_BIT(sysconf,1 , 14);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 12);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 10);
	SET_SYSCONF_BIT(sysconf,1 , 11);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 6);
	SET_SYSCONF_BIT(sysconf,0 , 7);
	SET_SYSCONF_BIT(sysconf,1 , 8);
	SET_SYSCONF_BIT(sysconf,1 , 9);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 3);
	SET_SYSCONF_BIT(sysconf,0 , 4);
	SET_SYSCONF_BIT(sysconf,1 , 5);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 0);
	SET_SYSCONF_BIT(sysconf,1 , 1);
	SET_SYSCONF_BIT(sysconf,0 , 2);
	writel(sysconf, sysconfReg);

	/* Reset port parameters override for usb2_0 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5062);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 21);
	SET_SYSCONF_BIT(sysconf,1 , 22);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 19);
	SET_SYSCONF_BIT(sysconf,0 , 20);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 15);
	SET_SYSCONF_BIT(sysconf,0 , 16);
	SET_SYSCONF_BIT(sysconf,1 , 17);
	SET_SYSCONF_BIT(sysconf,1 , 18);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 13);
	SET_SYSCONF_BIT(sysconf,1 , 14);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 12);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 10);
	SET_SYSCONF_BIT(sysconf,1 , 11);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 6);
	SET_SYSCONF_BIT(sysconf,0 , 7);
	SET_SYSCONF_BIT(sysconf,1 , 8);
	SET_SYSCONF_BIT(sysconf,1 , 9);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 3);
	SET_SYSCONF_BIT(sysconf,0 , 4);
	SET_SYSCONF_BIT(sysconf,1 , 5);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 0);
	SET_SYSCONF_BIT(sysconf,1 , 1);
	SET_SYSCONF_BIT(sysconf,0 , 2);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	
	/* PICOPHY_FSEL */
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 2);
	SET_SYSCONF_BIT(sysconf,1 , 3);
	SET_SYSCONF_BIT(sysconf,1 , 4);
	writel(sysconf, sysconfReg);

	/*PICOPHY_REFCLKSEL*/
	SET_SYSCONF_BIT(sysconf,1 , 0);
	SET_SYSCONF_BIT(sysconf,0 , 1);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 30);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 29);
	writel(sysconf, sysconfReg);

	/* Power down..*/
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5001);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 4); /* usb2_0 */
	SET_SYSCONF_BIT(sysconf, 1, 5); /* usb2_1 */
	writel(sysconf, sysconfReg);
    	udelay(1000);
}


static void usb2_port_init(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 1, 30);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	
	/*PICOPHY_REFCLKSEL*/
	SET_SYSCONF_BIT(sysconf,0 , 0);
	SET_SYSCONF_BIT(sysconf,1 , 1);
	writel(sysconf, sysconfReg);

	/* PICOPHY_FSEL */
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 2);
	SET_SYSCONF_BIT(sysconf,0 , 3);
	SET_SYSCONF_BIT(sysconf,0 , 4);
	writel(sysconf, sysconfReg);

	/* Port parameters overriding */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5064);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 0);
	SET_SYSCONF_BIT(sysconf,0 , 1);
	SET_SYSCONF_BIT(sysconf,1 , 2);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 3);
	SET_SYSCONF_BIT(sysconf,1 , 4);
	SET_SYSCONF_BIT(sysconf,0 , 5);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 6);
	SET_SYSCONF_BIT(sysconf,1 , 7);
	SET_SYSCONF_BIT(sysconf,0 , 8);
	SET_SYSCONF_BIT(sysconf,0 , 9);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 10);
	SET_SYSCONF_BIT(sysconf,0 , 11);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 12);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 13);
	SET_SYSCONF_BIT(sysconf,0 , 14);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 15);
	SET_SYSCONF_BIT(sysconf,1 , 16);
	SET_SYSCONF_BIT(sysconf,0 , 17);
	SET_SYSCONF_BIT(sysconf,0 , 18);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 19);
	SET_SYSCONF_BIT(sysconf,1 , 20);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 21);
	SET_SYSCONF_BIT(sysconf,0 , 22);
	writel(sysconf, sysconfReg);

	
	/* Start port2 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 7);
	writel(sysconf, sysconfReg);
}

static void usb2_port_stop(void)
{
	unsigned long *sysconfReg;
	unsigned long sysconf;

	/* Stop port2 */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 7);
	writel(sysconf, sysconfReg);

	/* Reset port parameter overriding parameters */
	sysconfReg = (unsigned long*)STXH410_SYSCFG(5064);
	sysconf = readl(sysconfReg);

	SET_SYSCONF_BIT(sysconf,0 , 21);
	SET_SYSCONF_BIT(sysconf,1 , 22);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 19);
	SET_SYSCONF_BIT(sysconf,0 , 20);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 15);
	SET_SYSCONF_BIT(sysconf,0 , 16);
	SET_SYSCONF_BIT(sysconf,1 , 17);
	SET_SYSCONF_BIT(sysconf,1 , 18);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 13);
	SET_SYSCONF_BIT(sysconf,1 , 14);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,1 , 12);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 10);
	SET_SYSCONF_BIT(sysconf,1 , 11);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 6);
	SET_SYSCONF_BIT(sysconf,0 , 7);
	SET_SYSCONF_BIT(sysconf,1 , 8);
	SET_SYSCONF_BIT(sysconf,1 , 9);
	writel(sysconf, sysconfReg);

	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 3);
	SET_SYSCONF_BIT(sysconf,0 , 4);
	SET_SYSCONF_BIT(sysconf,1 , 5);
	writel(sysconf, sysconfReg);
	
	SET_SYSCONF_BIT(sysconf,1 , 0);
	SET_SYSCONF_BIT(sysconf,1 , 1);
	SET_SYSCONF_BIT(sysconf,0 , 2);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5061);
	sysconf = readl(sysconfReg);

	/* PICOPHY_FSEL */
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf,0 , 2);
	SET_SYSCONF_BIT(sysconf,1 , 3);
	SET_SYSCONF_BIT(sysconf,1 , 4);
	writel(sysconf, sysconfReg);

	/*PICOPHY_REFCLKSEL*/
	SET_SYSCONF_BIT(sysconf,1 , 0);
	SET_SYSCONF_BIT(sysconf,0 , 1);
	writel(sysconf, sysconfReg);

	sysconfReg = (unsigned long*)STXH410_SYSCFG(5132);
	sysconf = readl(sysconfReg);
	SET_SYSCONF_BIT(sysconf, 0, 30);
	writel(sysconf, sysconfReg);
}

int init_done = 0;
extern int stm_usb_init(const int port)
{
        init_done = 1;
	stm_picophy_assert();
	configure_usb2_pio();
	stm_usb2_deassert();
	stm_miphy_deassert();
	configure_usb3_pio();
	stm_usb3_deassert();
	usb3_miphy_init();
	/* Program the MiPHY internal registers */
	miphy_setup();
	usb2_port_init();
	st_dwc3_init();
	return 0;
}

extern int stm_usb_stop(const int port)
{
        if(init_done == 0)
            return 0;
        if(init_done != CONFIG_USB_MAX_CONTROLLER_COUNT) {
            init_done++;
            return 0;
        }
        init_done = 0;
        st_dwc3_stop();
	usb2_port_stop();
	usb3_miphy_stop();
	stm_usb3_assert();
	deconfigure_usb3_pio();
	stm_miphy_assert();
	stm_usb2_assert();
	deconfigure_usb2_pio();
	stm_picophy_deassert();
	return 0;
}

#ifdef CONFIG_DM_USB

#ifdef STM_USB2_SUPPORT

#ifndef CONFIG_STM_STXH301
static const struct stm_ehci_platdata ehci1_platdata = {
	.hcd_base = CONFIG_SYS_USB2_0_BASE,
	.name = "stm_ehci1",
};

U_BOOT_DEVICE(stm_ehci1) = {
	.name = "ehci_stm",
	.platdata = &ehci1_platdata,
};
#endif

static const struct stm_ehci_platdata ehci2_platdata = {
	.hcd_base = CONFIG_SYS_USB2_1_BASE,
	.name = "stm_ehci2",
};

U_BOOT_DEVICE(stm_ehci2) = {
	.name = "ehci_stm",
	.platdata = &ehci2_platdata,
};
#endif

#ifdef STM_USB3_SUPPORT
static const struct stm_xhci_platdata xhci1_platdata = {
	.hcd_base = STM_DWC3_CORE_BASE,
};

U_BOOT_DEVICE(stm_xhci1) = {
	.name = "xhci_stm",
	.platdata = &xhci1_platdata,
};
#endif

#endif

