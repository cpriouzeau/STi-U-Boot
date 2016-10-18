#include <common.h>
#include <command.h>
#include <stm/soc.h>
#include <stm/stxh410reg.h>
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

struct stm_pad_pin ethernet_mii_pad_configs[] = {
			DATA_OUT(0, 0, 1, RET_SE_NICLK_IO(0, 0)),/* TXD[0] */
			DATA_OUT(0, 1, 1, RET_SE_NICLK_IO(0, 0)),/* TXD[1] */
			DATA_OUT(0, 2, 1, RET_SE_NICLK_IO(0, 0)),/* TXD[2] */
			DATA_OUT(0, 3, 1, RET_SE_NICLK_IO(0, 0)),/* TXD[3] */
			DATA_OUT(0, 4, 1, RET_SE_NICLK_IO(0, 0)),/* TXER */
			DATA_OUT(0, 5, 1, RET_SE_NICLK_IO(0, 0)),/* TXEN */
			CLOCK_IN(0, 6, 1, RET_NICLK(0, 0)),/* TXCLK */
			DATA_IN(0, 7, 1, RET_BYPASS(1000)),/* COL */
			DATA_OUT(1, 0, 1, RET_BYPASS(1500)),/* MDIO */
			CLOCK_OUT(1, 1, 1, RET_NICLK(0, 0)),/* MDC */
			DATA_IN(1, 2, 1, RET_BYPASS(1000)),/* CRS */
			DATA_IN(1, 3, 1, RET_BYPASS(0)),/* MDINT */
			DATA_IN(1, 4, 1, RET_SE_NICLK_IO(0, 0)),/* RXD[0] */
			DATA_IN(1, 5, 1, RET_SE_NICLK_IO(0, 0)),/* RXD[1] */
			DATA_IN(1, 6, 1, RET_SE_NICLK_IO(0, 0)),/* RXD[2] */
			DATA_IN(1, 7, 1, RET_SE_NICLK_IO(0, 0)),/* RXD[3] */
			DATA_IN(2, 0, 1, RET_SE_NICLK_IO(0, 0)),/* RXDV */
			DATA_IN(2, 1, 1, RET_SE_NICLK_IO(0, 0)),/* RX_ER */
			CLOCK_IN(2, 2, 1, RET_NICLK(0, 0)),/* RXCLK */
			PHY_CLOCK(2, 3, 1, RET_NICLK(0, 0)),/* PHYCLK */
};
int num_ethernet_mii_pad_configs = ARRAY_SIZE(ethernet_mii_pad_configs);
struct stm_pad_sysconf ethernet_mii_pad_sysconfs[] = {
			/* PWR_DWN_REQ */
			STM_PAD_SYSCONF(SYSCONF(4032), 0, 0, 0),
			/* EN_GMAC1 */
			STM_PAD_SYSCONF(SYSCONF(4032), 1, 1, 1),
			/* MIIx_PHY_SEL */
			STM_PAD_SYSCONF(SYSCONF(4032), 2, 4, 0),
			/* ENMIIx */
			STM_PAD_SYSCONF(SYSCONF(4032), 5, 5, 1),
			/* TXCLK_NOT_CLK125 */
			STM_PAD_SYSCONF(SYSCONF(4032), 6, 6, 1),
			/* INT_NOEXT_PHYCLK */
			STM_PAD_SYSCONF(SYSCONF(4032), 7, 7, 0),
			/* TX_RETIMING_CLK */
			STM_PAD_SYSCONF(SYSCONF(4032), 8, 8, 0),
			/* SBC_RESET_PER */
			STM_PAD_SYSCONF(SYSCONF(4002), 4, 4, 1),
};
int num_ethernet_mii_pad_sysconfs = ARRAY_SIZE(ethernet_mii_pad_sysconfs);

struct stm_pad_pin ethernet_rgmii_pad_configs[] = {
			DATA_OUT(0, 0, 1, RET_DE_IO(0, 0)),/* TXD[0] */
			DATA_OUT(0, 1, 1, RET_DE_IO(0, 0)),/* TXD[1] */
			DATA_OUT(0, 2, 1, RET_DE_IO(0, 0)),/* TXD[2] */
			DATA_OUT(0, 3, 1, RET_DE_IO(0, 0)),/* TXD[3] */
			DATA_OUT(0, 5, 1, RET_DE_IO(0, 0)),/* TXEN */
#ifndef CONFIG_STM_B2260
			CLOCK_IN(0, 6, 1, RET_NICLK(0, 0)),/* TXCLK */
#endif
			MDIO(1, 0, 1, RET_BYPASS(0)),/* MDIO */
			MDC(1, 1, 1, RET_NICLK(0, 0)),/* MDC */
			DATA_IN(1, 4, 1, RET_DE_IO(0, 0)),/* RXD[0] */
			DATA_IN(1, 5, 1, RET_DE_IO(0, 0)),/* RXD[1] */
			DATA_IN(1, 6, 1, RET_DE_IO(0, 0)),/* RXD[2] */
			DATA_IN(1, 7, 1, RET_DE_IO(0, 0)),/* RXD[3] */
			DATA_IN(2, 0, 1, RET_DE_IO(0, 0)),/* RXDV */
			CLOCK_IN(2, 2, 1, RET_NICLK(0, 0)),/* RXCLK */
			PHY_CLOCK(2, 3, 4, RET_NICLK(1750, 1)), /* PHYCLK */
			CLOCK_IN(3, 7, 4, RET_NICLK(0, 0)),/* 125MHz input clock */
};
int num_ethernet_rgmii_pad_configs = ARRAY_SIZE(ethernet_rgmii_pad_configs);
struct stm_pad_sysconf ethernet_rgmii_pad_sysconfs[] = {
                        /* SBC_RESET_PER */
                        STM_PAD_SYSCONF(SYSCONF(4002), 4, 4, 1),
                        /* PWR_DWN_REQ */
                        STM_PAD_SYSCONF(SYSCONF(4032), 0, 0, 0),
                        /* EN_GMAC1 */
                        STM_PAD_SYSCONF(SYSCONF(4032), 1, 1, 1),
                        /* MIIx_PHY_SEL */
                        STM_PAD_SYSCONF(SYSCONF(4032), 2, 4, 1),
                        /* ENMIIx */
                        STM_PAD_SYSCONF(SYSCONF(4032), 5, 5, 1),
                        /* TXCLK_NOT_CLK125 */
                        STM_PAD_SYSCONF(SYSCONF(4032), 6, 6, 0),
                        /* INT_NOEXT_PHYCLK */
                        STM_PAD_SYSCONF(SYSCONF(4032), 7, 7, 1),
                        /* TX_RETIME_CLK */
                        STM_PAD_SYSCONF(SYSCONF(4032), 8, 8, 1),
};
int num_ethernet_rgmii_pad_sysconfs = ARRAY_SIZE(ethernet_rgmii_pad_sysconfs);
struct stm_pad_pin ethernet_rmii_pad_configs[] = {
                        DATA_OUT(0, 0, 1, RET_SE_NICLK_IO(0, 0)),/* TXD[0] */
                        DATA_OUT(0, 1, 1, RET_SE_NICLK_IO(0, 0)),/* TXD[1] */
                        DATA_OUT(0, 5, 1, RET_SE_NICLK_IO(0, 0)),/* TXEN */
                        DATA_OUT(1, 0, 1, RET_BYPASS(0)),/* MDIO */
                        CLOCK_OUT(1, 1, 1, RET_NICLK(0, 0)),/* MDC */
                        DATA_IN(1, 3, 1, RET_BYPASS(0)),/* MDINT */
                        DATA_IN(1, 4, 1, RET_SE_NICLK_IO(0, 1)),/* RXD[0] */
                        DATA_IN(1, 5, 1, RET_SE_NICLK_IO(0, 1)),/* RXD[1] */
                        DATA_IN(2, 0, 1, RET_SE_NICLK_IO(0, 1)),/* RXDV */
                        DATA_IN(2, 1, 1, RET_SE_NICLK_IO(0, 0)),/* RX_ER */
                        PHY_CLOCK(2, 3, 1, RET_NICLK(0, 0)),/* PHYCLK */
};
int num_ethernet_rmii_pad_configs = ARRAY_SIZE(ethernet_rmii_pad_configs);
struct stm_pad_sysconf ethernet_rmii_pad_sysconfs[] = {
                        /* PWR_DWN_REQ */
                        STM_PAD_SYSCONF(SYSCONF(4032), 0, 0, 0),
                        /* EN_GMAC1 */
                        STM_PAD_SYSCONF(SYSCONF(4032), 1, 1, 1),
                        /* MIIx_PHY_SEL */
                        STM_PAD_SYSCONF(SYSCONF(4032), 2, 4, 4),
                        /* ENMIIx */
                        STM_PAD_SYSCONF(SYSCONF(4032), 5, 5, 1),
                        /* TXCLK_NOT_CLK125 */
                        STM_PAD_SYSCONF(SYSCONF(4032), 6, 6, 0),
                        /* INT_NOEXT_PHYCLK */
                        STM_PAD_SYSCONF(SYSCONF(4032), 7, 7, 1),
                        /* TX_RETIME_CLK */
                        STM_PAD_SYSCONF(SYSCONF(4032), 8, 8, 1),
                        /* SBC_RESET_PER */
                        STM_PAD_SYSCONF(SYSCONF(4002), 4, 4, 1),
};
int num_ethernet_rmii_pad_sysconfs = ARRAY_SIZE(ethernet_rmii_pad_sysconfs);
struct stm_uart_config stm_uart_config[] = {
	{{3,4,1},{3,5,1}},
	{{2,6,3},{2,7,3}},
	{{17,0,1},{17,1,1}},
	{{16,0,1},{16,1,1}},
	{{31,3,1},{31,4,1}},
};
struct stm_mmc_pio_getcd mmc_pio_getcd[] = {
#ifdef CONFIG_STM_B2260
{42,4},
#else
{0,0},
#endif
{19,0}
};
const struct stm_pad_pin stm_mmc_pad_configs[][10] = {
#ifdef CONFIG_STM_B2260
			{
                        MMC_DATA_IN_PU(41, 0, 2, NULL),  /* DATA[0] */
                        MMC_DATA_IN_PU(41, 1, 2, NULL ), /* DATA[1] */
                        MMC_DATA_IN_PU(41, 2, 2, NULL),  /* DATA[2] */
                        MMC_DATA_IN_PU(41, 3, 2, NULL ), /* DATA[3] */
                        MMC_DATA_IN_PU(40, 7, 2, NULL ), /* CMD */
                        MMC_CLOCK_OUT(40, 6, 2,  NULL ), /* Clock */
                        MMC_OUT(42, 2, 2),               /* MMC Card PWR */
                        MMC_OUT(42, 0, 2),               /* MMC LED ON */
                        MMC_DATA_IN_PU(42, 4, 2, NULL),  /* MMC Card Detect */
                        MMC_IN(42, 5, 2),                /* MMC Write Protect */
			}
#else
			{
                        MMC_DATA_IN_PU(41, 0, 1, NULL), /* DATA[0] */
                        MMC_DATA_IN_PU(41, 1, 1, NULL), /* DATA[1] */
                        MMC_DATA_IN_PU(41, 2, 1, NULL), /* DATA[2] */
                        MMC_DATA_IN_PU(41, 3, 1, NULL), /* DATA[3] */
                        MMC_DATA_IN_PU(41, 4, 1, NULL), /* DATA[4] */
                        MMC_DATA_IN_PU(41, 5, 1, NULL), /* DATA[5] */
                        MMC_DATA_IN_PU(41, 6, 1, NULL), /* DATA[6] */
                        MMC_DATA_IN_PU(41, 7, 1, NULL), /* DATA[7] */
                        MMC_DATA_IN_PU(40, 7, 1, NULL), /* CMD */
                        MMC_CLOCK_OUT(40, 6, 1, NULL),  /* Clock */ 
			}
#endif
			,
			{
                        MMC_DATA_IN_PU(19, 4, 5, RET_BYPASS(3250)),     /* DATA[0] */
                        MMC_DATA_IN_PU(19, 5, 5, RET_BYPASS(3250)),     /* DATA[1] */
                        MMC_DATA_IN_PU(19, 6, 5, RET_BYPASS(3250)),     /* DATA[2] */
                        MMC_DATA_IN_PU(19, 7, 5, RET_BYPASS(3250)),     /* DATA[3] */
                        MMC_DATA_IN_PU(19, 2, 5, RET_BYPASS(3250)),     /* CMD */
                        MMC_CLOCK_OUT(19, 3, 5, RET_NICLK2(3250,1)),    /* Clock */
                        MMC_OUT(16, 7, 6),                              /* MMC Card PWR */
                        MMC_OUT(16, 6, 6),                              /* MMC LED ON */
                        MMC_DATA_IN_PU(19, 0, 6, NULL),                 /* MMC Card Detect */
                        MMC_IN(19, 1, 6),                               /* MMC Write Protect */
			}
};

const struct ssc_pios ssc_pios[6+3] =
{
	{ { /* SSC0 */
		{ 10, 5 }, /* SCLK */
		{ 10, 6 }, /* MTSR */
	} }, { { /* SSC1 */
		{ 11, 0 }, /* SCLK */
		{ 11, 1 }, /* MTSR */
	} }, { { /* SSC2 */
		{ 15, 5 }, /* SCLK */
		{ 15, 6 }, /* MTSR */
	} }, { { /* SSC3 */
		{ 18, 5 }, /* SCLK */
		{ 18, 6 }, /* MTSR */
	} }, { { /* SSC4 */
		{ 30, 0 }, /* SCLK */
		{ 30, 1 }, /* MTSR */
       } }, { { /* SSC5 */
               { 34, 3 }, /* SCLK */
               { 34, 4 }, /* MTSR */
       } }, { { /* SBC_SSC10 */
               { 4, 5 }, /* SCLK */
               { 4, 6 }, /* MTSR */
       } }, { { /* SBC_SSC11 */
               { 5, 0 }, /* SCLK */
               { 5, 1 }, /* MTSR */
       } }, { { /* SBC_SSC12 */
               { 5, 7 }, /* SCLK */
               { 5, 6 }, /* MTSR */
       } },
};
int ssc_pios_array_size = ARRAY_SIZE(ssc_pios);
/* PAD configuration to configure USB3 PIOs */
struct stm_pad_pin stm_usb3_pad_configs[] = {
                DATA_IN(35, 4, 1, RET_BYPASS(0)),/* OC-DETECT */
                DATA_OUT(35, 5, 1, RET_BYPASS(0)),/* PWR-ENABLE */
                DATA_IN(35, 6, 1, RET_BYPASS(0)),/* VBUS-VALID */
};
/* PAD configuration to configure USB2 PIOs */
struct stm_pad_pin stm_usb2_pad_configs[] = {
                DATA_IN(35, 0, 1, RET_BYPASS(0)),/* USB_20:OC-DETECT */
                DATA_OUT(35, 1, 1, RET_BYPASS(0)),/* USB_20:PWR-ENABLE */
                DATA_IN(35, 2, 1, RET_BYPASS(0)),/* USB_21:OC-DETECT */
                DATA_OUT(35, 3, 1, RET_BYPASS(0)),/* USB_21:PWR-ENABLE */
};

/* PAD configuration to deconfigure USB3 PIOs */
struct stm_pad_pin stm_usb3_pad_deconfigs[] = {
                DATA_OUT(35, 4, 0, RET_BYPASS(0)),/* OC-DETECT */
                DATA_IN(35, 5, 0, RET_BYPASS(0)),/* PWR-ENABLE */
                DATA_OUT(35, 6, 0, RET_BYPASS(0)),/* VBUS-VALID */
};

/* PAD configuration to deconfigure USB2 PIOs */
struct stm_pad_pin stm_usb2_pad_deconfigs[] = {
                DATA_OUT(35, 0, 0, RET_BYPASS(0)),/* USB_20:OC-DETECT */
                DATA_IN(35, 1, 0, RET_BYPASS(0)),/* USB_20:PWR-ENABLE */
                DATA_OUT(35, 2, 0, RET_BYPASS(0)),/* USB_21:OC-DETECT */
                DATA_IN(35, 3, 0, RET_BYPASS(0)),/* USB_21:PWR-ENABLE */
};
int num_stm_usb3_pad_configs = ARRAY_SIZE(stm_usb3_pad_configs);
int num_stm_usb2_pad_configs = ARRAY_SIZE(stm_usb2_pad_configs);
int num_stm_usb3_pad_deconfigs = ARRAY_SIZE(stm_usb3_pad_deconfigs);
int num_stm_usb2_pad_deconfigs = ARRAY_SIZE(stm_usb2_pad_deconfigs);
