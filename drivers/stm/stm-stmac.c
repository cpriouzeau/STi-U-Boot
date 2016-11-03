/*
 *  Copyright (c) 2006-2013 STMicroelectronics Limited
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
 * author(s): Andy Sturges (andy.sturges@st.com)
 *            Sean McGoogan <Sean.McGoogan@st.com>
 */

#include <common.h>

#if defined(CONFIG_DRIVER_NETSTMAC) || defined(CONFIG_DRIVER_NET_STM_GMAC)

#include <command.h>
#include <stm/soc.h>
#include <stm/socregs.h>
#include <asm/io.h>
#include <net.h>
#include <malloc.h>
#include <miiphy.h>
#include <asm/dma-mapping.h>
#include "stm-stmac.h"
#include <netdev.h>


#if defined(CONFIG_ST40)			/* for ST40 cores … */
#	include <asm/addrspace.h>
#	include <asm/pmb.h>
#elif defined(CONFIG_ARM)			/* for ARM cores … */
#	define L1_CACHE_BYTES	32		/* Assume L1 cache line is 32-bytes */
#	define PHYSADDR(x)	((u32)(x))	/* identity mapped */
#	define P2SEGADDR(x)	PHYSADDR(x)	/* identity mapped */
#endif						/* ST40 || ARM */


#if defined(CONFIG_CMD_NET)

/* #define DEBUG */
#ifdef DEBUG
#	define PRINTK(args...) printf(args)
#else
#	define PRINTK(args...)
#endif

/* do we want to put the PHY in loop-back mode ? */
/* #define CONFIG_PHY_LOOPBACK */

/* do we want to dump the first 14-bytes of each
 * RX/TX ethernet packet (i.e. the IEEE 802.3 MAC,
 * (or RFC 894/1042) encapsulation header ? */
/* #define DUMP_ENCAPSULATION_HEADER */

/* prefix to use for diagnostics */
#ifdef CONFIG_DRIVER_NETSTMAC
#	define STMAC	"STM-MAC: "
#else
#	define STMAC	"STM-GMAC: "
#endif /* CONFIG_DRIVER_NETSTMAC */

#define CONFIG_DMA_RX_SIZE 8
#define CONFIG_DMA_TX_SIZE 1	/* Only ever use 1 tx buffer */

static volatile stmac_dma_des *dma_tx;
static volatile stmac_dma_des *dma_rx;
static int cur_rx;
static int eth_phy_addr;

#define MAX_ETH_FRAME_SIZE	1536
#define MAX_PAUSE_TIME (MAC_FLOW_CONTROL_PT_MASK>>MAC_FLOW_CONTROL_PT_SHIFT)

static void stmac_mii_write(struct eth_device * const dev, int phy_addr, int reg, int value);
static unsigned int stmac_mii_read(struct eth_device * const dev, int phy_addr, int reg);

/* DMA structure */
struct dma_t
{
	uchar rx_buff[CONFIG_DMA_RX_SIZE * (PKTSIZE_ALIGN + 2 * L1_CACHE_BYTES) ];
	stmac_dma_des desc_rx[CONFIG_DMA_RX_SIZE];
	stmac_dma_des desc_tx[CONFIG_DMA_TX_SIZE];
	uchar _dummy[L1_CACHE_BYTES];
} __attribute__ ((aligned(L1_CACHE_BYTES)))*dma;

static void *rx_packets[CONFIG_DMA_RX_SIZE];

/* private data structure */
struct stmac_private
{
	u32	id;		/* unique ID of MAC */
};

#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)

/* ----------------------------------------------------------------------------
				 Phy interface
   ---------------------------------------------------------------------------*/

#if defined(CONFIG_STMAC_STE10XP)	/* ST STe10xp */

/* STe100p phy identifier values */
#define STE100P_PHY_ID		0x1c040011u
#define STE100P_PHY_ID_MASK	0xffffffffu

/* STe101p phy identifier values */
#define STE101P_PHY_ID		0x00061c50u
#define STE101P_PHY_ID_MASK	0xfffffff0u

/******************************************************************************
 * IEEE Standard 802.3-2002 vendor specific registers (0x10-0x1e) STe10xP
 *****************************************************************************/
#define MII_XCIIS		0x11	/* Config info & int status register */
#define MII_XIE			0x12	/* Interrupt enable register */
#define MII_100CTR		0x13	/* 100BaseX control register */
#define MII_XMC			0x14	/* Mode control register */

/******************************************************************************
 * 100BaseX Auxiliary Status register defines
 *****************************************************************************/
#define XCIIS_FIFO_OVR		0x0800	/* FIFO Overrun */
#define XCIIS_SPEED		0x0200	/* Speed */
#define XCIIS_DUPLEX		0x0100	/* Duplex */
#define XCIIS_PAUSE		0x0080	/* Pause */
#define XCIIS_ANEG_INT		0x0040	/* Auto Negotiation Interrupt */
#define XCIIS_RFAULT		0x0020	/* Remote Fault Interrupt */
#define XCIIS_LDOWN		0x0010	/* Link Down Interrupt */
#define XCIIS_LCWR		0x0008	/* Link Code Word Received Interrupt */
#define XCIIS_PFAULT		0x0004	/* Parallel Detection Fault */
#define XCIIS_ANEG_PAGE		0x0002	/* Auto Negotiation Page Rec Intr */
#define XCIIS_REF_INTR		0x0001	/* Ref Interrupt */

/******************************************************************************
 * XCVR Mode Control register defines
 *****************************************************************************/
#define XMC_LDETECT		0x0800	/* Link Detect */
#define XMC_PHY_ADDR_MSK	0x00f8	/* PHY Address Mask */
#define XMC_PHY_ADDR_SHIFT	3	/* PHY Address Mask */
#define XMC_PRE_SUP		0x0002	/* Preamble Suppression */
#define PHY_ADDR_MSK		XMC_PHY_ADDR_MSK	/* PHY Address Mask */
#define PHY_ADDR_SHIFT		XMC_PHY_ADDR_SHIFT	/* PHY Address Mask */

/* MII mode */
#define MII_TSTAT_SMII		0x1000
#define MII_TSTAT_RMII		0x0800
#define MII_TSTAT_MII		0x0400

#elif defined(CONFIG_STMAC_LAN8700)	/* SMSC LAN8700 */

/* SMSC LAN8700 phy identifier values */
#define LAN8700_PHY_ID		0x0007c0c0u
#define LAN8700_PHY_ID_MASK	0xfffffff0u

#define SPECIAL_MODE_REG	0x12	/* Special Modes Register */
#define PHY_ADDR_MSK		0x001f	/* PHY Address Mask */
#define PHY_ADDR_SHIFT		0	/* PHY Address Mask */

#elif defined(CONFIG_STMAC_LAN8710)	/* SMSC LAN8710/20 */

/* SMSC LAN8710 phy identifier values */
#define LAN8710_PHY_ID		0x0007c0f0u
#define LAN8710_PHY_ID_MASK	0xfffffff0u

#define SPECIAL_MODE_REG	0x12	/* Special Modes Register */
#define PHY_ADDR_MSK		0x001f	/* PHY Address Mask */
#define PHY_ADDR_SHIFT		0	/* PHY Address Mask */

#elif defined(CONFIG_STMAC_DP83865)	/* Nat Semi DP83865 */

/* Nat Semi DP83865 phy identifier values */
#define DP83865_PHY_ID		0x20005c70u
#define DP83865_PHY_ID_MASK	0xfffffff0u

#define PHY_SUP_REG		0x1f	/* PHY Support Register */
#define PHY_ADDR_MSK		0x001f	/* PHY Address Mask */
#define PHY_ADDR_SHIFT		0	/* PHY Address Mask */

#elif defined(CONFIG_STMAC_KSZ8041)	/* Micrel KSZ8041 */

/* Micrel KSZ8041 phy identifier values */
#define KSZ8041_PHY_ID		0x00221512u /* 1512 = A3, 1513 = A4 */
#define KSZ8041_PHY_ID_MASK	0xfffffffeu

#elif defined(CONFIG_STMAC_KSZ8081)    /* Micrel KSZ8081 */

/* Micrel KSZ8081 phy identifier values */
#define KSZ8081_PHY_ID         0x00221560u
#define KSZ8081_PHY_ID_MASK    0x00fffff0u

#elif defined(CONFIG_STMAC_IP1001)	/* IC+ IP1001 */

/* IC+ IP1001 phy identifier values */
#define IP1001_PHY_ID		0x02430d90u
#define IP1001_PHY_ID_MASK	0xfffffff0u

#define IP1001_SPEC_CTRL_STATUS	16	/* PHY Specific Control & Status Register #1 */
#define IP1001_RXPHASE_SEL	(1u<<0)	/* RXPHASE_SEL = bit 16[0] */
#define IP1001_TXPHASE_SEL	(1u<<1)	/* TXPHASE_SEL = bit 16[1] */

#elif defined(CONFIG_STMAC_IP101A) || defined(CONFIG_STMAC_IP101G)	/* IC+ IP101A or IP101G */

/* IC+ IP101A and IP101G phy identifier values are the *same*! */
#define IP101x_PHY_ID		0x02430c54u
#define IP101x_PHY_ID_MASK	0xffffffffu
#define IP101G_PAGE_CONTROL_REG	20	/* Page Control Register */

#elif defined(CONFIG_STMAC_78Q2123)	/* TERIDIAN 78Q2123 */

/* TERIDIAN 78Q2123 phy identifier values */
#define TERIDIAN_PHY_ID		0x000e7230u
#define TERIDIAN_PHY_ID_MASK	0xfffffff0u

#elif defined(CONFIG_STMAC_RTL8201E)	/* REALTEK RTL8201E(L) */

/* REALTEK RTL8201E(L) phy identifier values */
#define RTL8201E_PHY_ID		0x001cc815u
#define RTL8201E_PHY_ID_MASK	0xffffffffu

#define PHY_TEST_REG		0x19	/* PHY Support Register */
#define PHY_ADDR_MSK		0x180	/* PHY Address Mask */
#define PHY_ADDR_SHIFT		7	/* PHY Address Mask */

#elif defined(CONFIG_STMAC_RTL8211E)	/* REALTEK RTL8211E(G) */

/* REALTEK RTL8211E(G) phy identifier values */
#define RTL8211E_PHY_ID		0x001cc915u
#define RTL8211E_PHY_ID_MASK	0xffffffffu
#if !defined(CONFIG_STMAC_LINK_STATUS_TIMEOUT)
#	define CONFIG_STMAC_LINK_STATUS_TIMEOUT	(CONFIG_SYS_HZ/20)	/* 50 ms*/
#endif	/* CONFIG_STMAC_LINK_STATUS_TIMEOUT */

#elif defined(CONFIG_STMAC_MARVELL)	/* MARVELL 88EC060 */

/* MARVELL 88EC060 phy identifier values */
#define MARVELL_PHY_ID		0x01410DD1u
#define MARVELL_PHY_ID_MASK	0xffffffffu

#elif defined(CONFIG_STMAC_GENERIC)	/* a "generic" PHY */

/* This will match *any* (real) PHY ID (i.e. not all-ones) */
#define GENERIC_PHY_ID		0xffffffffu
#define GENERIC_PHY_ID_MASK	0xffffffffu

#else
#error Need to define which PHY to use
#endif


/* MII mode */
#define MII_ADVERTISE_PAUSE	0x0400	/* supports the pause command */


#if !defined(CONFIG_STMAC_USE_FIXED_PHY)
static void stmac_set_mac_mii_cap(struct eth_device * const dev, int full_duplex, unsigned int speed);
#endif	/* !CONFIG_STMAC_USE_FIXED_PHY */

#if !defined(CONFIG_PHY_LOOPBACK) && !defined(CONFIG_STMAC_USE_FIXED_PHY)
static int stmac_phy_negotiate(struct eth_device * const dev, int phy_addr)
{
	uint now, tmp, status;

	status = 0;

	tmp = stmac_mii_read(dev, phy_addr, MII_BMCR);
	tmp |= (BMCR_ANENABLE | BMCR_ANRESTART);
	stmac_mii_write(dev, phy_addr, MII_BMCR, tmp);

	now = get_timer(0);
	while (get_timer(now) < CONFIG_STMAC_AUTONEG_TIMEOUT) {
		status = stmac_mii_read(dev, phy_addr, MII_BMSR);
		if (status & BMSR_ANEGCOMPLETE) {
			break;
		}

		/* Restart auto-negotiation if remote fault */
		if (status & BMSR_RFAULT) {
			printf(STMAC "PHY remote fault detected\n");
			/* Restart auto-negotiation */
			printf(STMAC "PHY restarting auto-negotiation\n");
			stmac_mii_write(dev, phy_addr, MII_BMCR,
					 BMCR_ANENABLE | BMCR_ANRESTART);
		}
	}

	if (!(status & BMSR_ANEGCOMPLETE)) {
		printf(STMAC "PHY auto-negotiate timed out\n");
	}

	if (status & BMSR_RFAULT) {
		printf(STMAC "PHY remote fault detected\n");
	}

	return (1);
}

static unsigned int stmac_phy_check_speed(struct eth_device * const dev, int phy_addr)
{
	unsigned int status;
	int full_duplex = 0;
	int speed = 0;

	/* Read Status register */
	status = stmac_mii_read(dev, phy_addr, MII_BMSR);

#if defined(CONFIG_STMAC_LINK_STATUS_TIMEOUT)
	/*
	 * For the RealTek RTL8211E(G) (and potentially other PHYs),
	 * empirically, we need to wait a little while after the
	 * auto-negotiation has completed, before the "Link Status"
	 * bitfield in the BMSR actually becomes asserted!
	 * So, read it again, until it is asserted, or we timeout.
	 */
	ulong now = get_timer(0);
	while ( !(status & BMSR_LSTATUS) &&
		(get_timer(now) < CONFIG_STMAC_LINK_STATUS_TIMEOUT) ) {
		status = stmac_mii_read(dev, phy_addr, MII_BMSR);
	}
#endif	/* CONFIG_STMAC_LINK_STATUS_TIMEOUT */

	printf(STMAC);

	/* Check link status.  If 0, default to 100 Mbps. */
	if ((status & BMSR_LSTATUS) == 0) {
		printf("*Warning* no link detected\n");
		return 1;
	} else {
		int negotiated = stmac_mii_read(dev, phy_addr, MII_LPA);

		if (negotiated & LPA_100FULL) {
			printf("100Mbs full duplex link detected\n");
			full_duplex = 1;
			speed = 100;
		} else if (negotiated & LPA_100HALF) {
			printf("100Mbs half duplex link detected\n");
			full_duplex = 0;
			speed = 100;
		} else if (negotiated & LPA_10FULL) {
			printf("10Mbs full duplex link detected\n");
			full_duplex = 1;
			speed = 10;
		} else {
			printf("10Mbs half duplex link detected\n");
			full_duplex = 0;
			speed = 10;
		}
	}
	stmac_set_mac_mii_cap(dev, full_duplex, speed);
	return 0;
}
#endif	/* !CONFIG_PHY_LOOPBACK && !CONFIG_STMAC_USE_FIXED_PHY */

/* Automatically gets and returns the PHY device */
static unsigned int stmac_phy_get_addr(struct eth_device * const dev)
{
	unsigned int i, id;
#if defined(CONFIG_STMAC_IP101A) || defined(CONFIG_STMAC_IP101G)
	const char * deviceName = "IP101A";	/* assume an "A" part */
#endif

	for (i = 0; i < 32; i++) {
		unsigned int phyaddr = (i + 1u) % 32u;
		unsigned int id1 = stmac_mii_read(dev, phyaddr, MII_PHYSID1);
		unsigned int id2 = stmac_mii_read(dev, phyaddr, MII_PHYSID2);
		id  = (id1 << 16) | (id2);

		/* Make sure it is a valid (known) identifier */
#if defined(CONFIG_STMAC_STE10XP)
		if ((id & STE101P_PHY_ID_MASK) == STE101P_PHY_ID) {
			printf(STMAC "STe101P found\n");
			return phyaddr;
		} else if ((id & STE100P_PHY_ID_MASK) == STE100P_PHY_ID) {
			printf(STMAC "STe100P found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_LAN8700)
		if ((id & LAN8700_PHY_ID_MASK) == LAN8700_PHY_ID) {
			printf(STMAC "SMSC LAN8700 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_LAN8710)
		if ((id & LAN8710_PHY_ID_MASK) == LAN8710_PHY_ID) {
			printf(STMAC "SMSC LAN8710/20 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_DP83865)
		if ((id & DP83865_PHY_ID_MASK) == DP83865_PHY_ID) {
			printf(STMAC "NS DP83865 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_KSZ8041)
		if ((id & KSZ8041_PHY_ID_MASK) == KSZ8041_PHY_ID) {
			printf(STMAC "Micrel KSZ8041 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_KSZ8081)
		if ((id & KSZ8081_PHY_ID_MASK) == KSZ8081_PHY_ID) {
			printf(STMAC "Micrel KSZ8081 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_IP1001)
		if ((id & IP1001_PHY_ID_MASK) == IP1001_PHY_ID) {
			printf(STMAC "IC+ IP1001 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_IP101A) || defined(CONFIG_STMAC_IP101G)
		if ((id & IP101x_PHY_ID_MASK) == IP101x_PHY_ID) {
			const unsigned int page = stmac_mii_read(dev, phyaddr, IP101G_PAGE_CONTROL_REG);
			if (page == 16)		/* page 16 ? */
				deviceName = "IP101G";	/* assume a "G" part */
			printf(STMAC "IC+ %s found\n", deviceName);
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_78Q2123)
		if ((id & TERIDIAN_PHY_ID_MASK) == TERIDIAN_PHY_ID) {
			printf(STMAC "TERIDIAN 78Q2123 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_RTL8201E)
		if ((id & RTL8201E_PHY_ID_MASK) == RTL8201E_PHY_ID) {
			printf(STMAC "REALTEK RTL8201E(L) found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_RTL8211E)
		if ((id & RTL8211E_PHY_ID_MASK) == RTL8211E_PHY_ID) {
			printf(STMAC "REALTEK RTL8211E(G) found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_MARVELL)
		if ((id & MARVELL_PHY_ID_MASK) == MARVELL_PHY_ID) {
			printf(STMAC "MARVELL 88EC060 found\n");
			return phyaddr;
		}
#elif defined(CONFIG_STMAC_GENERIC)
		if ((id & GENERIC_PHY_ID_MASK) != GENERIC_PHY_ID) {
			printf(STMAC "Generic PHY found (ID=0x%08x)\n",id);
			return phyaddr;
		}
#else
#error Need to define which PHY to use
#endif	/* CONFIG_STMAC_STE10XP */
	}

	printf(STMAC "Unable to find a known PHY!\n");

	/* write out the IDs of all PHYs who respond */
	for (i = 0; i < 32; i++) {
		unsigned int phyaddr = i;
		unsigned int id1 = stmac_mii_read(dev, phyaddr, MII_PHYSID1);
		unsigned int id2 = stmac_mii_read(dev, phyaddr, MII_PHYSID2);
		id  = (id1 << 16) | (id2);
		if (id != ~0u) {	/* not all ones */
			printf(STMAC "info: PHY at address=0x%02x has ID=0x%08x\n", phyaddr, id);
		}
	}

	return (-1);
}

static int stmac_phy_init(struct eth_device * const dev)
{
#if !defined(CONFIG_STMAC_USE_FIXED_PHY)
	uint advertised_caps, value;
#endif	/* CONFIG_STMAC_USE_FIXED_PHY */

	/* Obtain the PHY's address/id */
	eth_phy_addr = stmac_phy_get_addr(dev);
	if (eth_phy_addr < 0)
		return -1;

#if defined(CONFIG_STMAC_USE_FIXED_PHY)
	/*
	 * Using a similar convention to linux, we effectively treat
	 * the STMAC as if it were connected to a "fixed PHY". This
	 * means that at do not try and control it via writing to
	 * the traditional IEEE PHY register set. Specifically, we
	 * do not try and "reset" it, nor effect the AN status here.
	 * Hence, we just "return", without doing anything more here!
	 */
#if defined(CONFIG_PHY_LOOPBACK)
	#error CONFIG_PHY_LOOPBACK is not supported with a "fixed PHY"!
#endif	/* CONFIG_PHY_LOOPBACK */

#else	/* CONFIG_STMAC_USE_FIXED_PHY */

	/* Now reset the PHY we just found */
	if (miiphy_reset(dev->name, eth_phy_addr)< 0) {
		PRINTK(STMAC "PHY reset failed!\n");
		return -1;
	}

	/* test for H/W address disagreement with the assigned address */
#if defined(CONFIG_STMAC_STE10XP)
	value = stmac_mii_read(dev, eth_phy_addr, MII_XMC);
#elif defined(CONFIG_STMAC_LAN8700) || defined(CONFIG_STMAC_LAN8710)
	value = stmac_mii_read(dev, eth_phy_addr, SPECIAL_MODE_REG);
#elif defined(CONFIG_STMAC_DP83865)
	value = stmac_mii_read(dev, eth_phy_addr, PHY_SUP_REG);
#elif defined(CONFIG_STMAC_RTL8201E)
	value = stmac_mii_read(dev, eth_phy_addr, PHY_TEST_REG);
#elif defined(CONFIG_STMAC_RTL8211E)
	/* The REALTEK RTL8211E(G) does not appear to support
	 * reading the H/W PHY address from any register.  */
#	define CONFIG_STMAC_BYPASS_ADDR_MISMATCH
#elif defined(CONFIG_STMAC_KSZ8041) || defined (CONFIG_STMAC_KSZ8081)
	/* The Micrel KSZ8041 does not appear to support
	 * reading the H/W PHY address from any register.  */
#	define CONFIG_STMAC_BYPASS_ADDR_MISMATCH
#elif defined(CONFIG_STMAC_IP1001)
	/* The IC+ IP1001 does not appear to support
	 * reading the H/W PHY address from any register.  */
#	define CONFIG_STMAC_BYPASS_ADDR_MISMATCH
#elif defined(CONFIG_STMAC_IP101A) || defined(CONFIG_STMAC_IP101G)
	/* The IC+ IP101 does not appear to support
	 * reading the H/W PHY address from any register.  */
#	define CONFIG_STMAC_BYPASS_ADDR_MISMATCH
#elif defined(CONFIG_STMAC_78Q2123)
	/* The TERIDIAN 78Q2123 does not appear to support
	 * reading the H/W PHY address from any register.  */
#	define CONFIG_STMAC_BYPASS_ADDR_MISMATCH
#elif defined(CONFIG_STMAC_MARVELL)
	/* The MARVEL 88EC060 does not appear to support
	 * reading the H/W PHY address from any register.  */
#	define CONFIG_STMAC_BYPASS_ADDR_MISMATCH
#elif defined(CONFIG_STMAC_GENERIC)
	/* For the GENERIC PHY, we assume it does not support
	 * reading the H/W PHY address from any register.  */
#	define CONFIG_STMAC_BYPASS_ADDR_MISMATCH
#else
#error Need to define which PHY to use
#endif
#if !defined(CONFIG_STMAC_BYPASS_ADDR_MISMATCH)
	value = (value & PHY_ADDR_MSK) >> PHY_ADDR_SHIFT;
	if (value != eth_phy_addr) {
		printf(STMAC "PHY address mismatch with hardware (hw %d != %d)\n",
			value,
			eth_phy_addr);
	}
#endif

	/* Read the ANE Advertisement register */
	advertised_caps = stmac_mii_read(dev, eth_phy_addr, MII_ADVERTISE);

	/* Copy our capabilities from MII_BMSR to MII_ADVERTISE */
	value = stmac_mii_read(dev, eth_phy_addr, MII_BMSR);

	/* Set the advertised capabilities */
	if (value & BMSR_100BASE4)
		advertised_caps |= ADVERTISE_100BASE4;
	if (value & BMSR_100FULL)
		advertised_caps |= ADVERTISE_100FULL;
	if (value & BMSR_100HALF)
		advertised_caps |= ADVERTISE_100HALF;
	if (value & BMSR_10FULL)
		advertised_caps |= ADVERTISE_10FULL;
	if (value & BMSR_10HALF)
		advertised_caps |= ADVERTISE_10HALF;

#ifdef CONFIG_STMAC_FLOWCTRL
	advertised_caps |= MII_ADVERTISE_PAUSE;
#else
	advertised_caps &= ~MII_ADVERTISE_PAUSE;
#endif

	/* Update our Auto-Neg Advertisement Register */
	stmac_mii_write(dev, eth_phy_addr, MII_ADVERTISE, advertised_caps);

	/*
	 * For Gigabit capable PHYs, then we will disable the
	 * ability to auto-negotiate at 1000BASE-T (Gigabit).
	 * Once ST's SoCs are capable of Gigabit, then we will review!
	 */
#if defined(CONFIG_STMAC_IP1001) || defined(CONFIG_STMAC_MARVELL) || defined(CONFIG_STMAC_RTL8211E)
	value = stmac_mii_read(dev, eth_phy_addr, MII_GBCR);
	value &= ~(GBCR_1000HALF|GBCR_1000FULL);
	stmac_mii_write(dev, eth_phy_addr, MII_GBCR, value);
#endif

	/*
	 * On a specific board, in RGMII mode, with the IC+ IP1001 PHY, an additional
	 * delay of ~2ns should be added to Rx and Tx, to adjust the RX/TX clock phases.
	 * QQQ: This code-hackery should really be in a board-specific file!
	 */
#if defined(CONFIG_STMAC_IP1001)
	value = stmac_mii_read(dev, eth_phy_addr, IP1001_SPEC_CTRL_STATUS);
	value |= (IP1001_RXPHASE_SEL | IP1001_TXPHASE_SEL);
	stmac_mii_write(dev, eth_phy_addr, IP1001_SPEC_CTRL_STATUS, value);
#endif	/* CONFIG_STMAC_IP1001 */

#ifdef CONFIG_PHY_LOOPBACK

	/* put the PHY in loop-back mode, if required */
	printf( STMAC "Forcing PHY loop-back at full-duplex, 100Mbps\n");
	value = stmac_mii_read(dev, eth_phy_addr, MII_BMCR);
	value |= BMCR_LOOPBACK;		/* enable loop-back mode (in the PHY) */
	value &= ~BMCR_ANENABLE;	/* disable auto-negotiation */
	value &= ~BMCR_SPEED_MASK;	/* clear all speed bits */
	value |= BMCR_SPEED100;		/* set speed to 100Mbps */
	value |= BMCR_FULLDPLX;		/* enable full-duplex */
	stmac_mii_write(dev, eth_phy_addr, MII_BMCR, value);
	/* ensure the write completes! */
	(void)stmac_mii_read(dev, eth_phy_addr, MII_BMCR);

	/* set the MAC capabilities appropriately */
	stmac_set_mac_mii_cap(dev, 1, 100);	/* 100Mbps, full-duplex */

#else	/* CONFIG_PHY_LOOPBACK */

	/* auto-negotiate with remote link partner */
	stmac_phy_negotiate(dev, eth_phy_addr);
	stmac_phy_check_speed(dev, eth_phy_addr);

#endif	/* CONFIG_PHY_LOOPBACK */

#endif	/* CONFIG_STMAC_USE_FIXED_PHY */

	return 0;
}


/* ----------------------------------------------------------------------------
				 MII Interface
   ---------------------------------------------------------------------------*/

#if !defined(CONFIG_STMAC_USE_FIXED_PHY)
static int stmac_mii_poll_busy(struct eth_device * const dev)
{
	/* arm simple, non interrupt dependent timer */
	ulong now = get_timer(0);
	while (get_timer(now) < CONFIG_STMAC_MII_POLL_BUSY_DELAY) {
		if (!(STMAC_READ(MAC_MII_ADDR) & MAC_MII_ADDR_BUSY)) {
			return 1;
		}
	}
	printf(STMAC "stmac_mii_busy timeout\n");
	return (0);
}
#endif	/* CONFIG_STMAC_USE_FIXED_PHY */

static void stmac_mii_write(struct eth_device * const dev, int phy_addr, int reg, int value)
{
#if 0
	printf("QQQ: %s(addr=%u, reg=%u, value=0x%04x)\n", __FUNCTION__, phy_addr, reg, value);
#endif

#if defined(CONFIG_STMAC_USE_FIXED_PHY)

	printf("Error: \"mii write\" is not supported for this PHY\n");

#else	/* CONFIG_STMAC_USE_FIXED_PHY */

	int mii_addr;

	/* Select register */
	mii_addr =
		((phy_addr & MAC_MII_ADDR_PHY_MASK) << MAC_MII_ADDR_PHY_SHIFT)
		| ((reg & MAC_MII_ADDR_REG_MASK) << MAC_MII_ADDR_REG_SHIFT)
		| MAC_MII_ADDR_WRITE | MAC_MII_ADDR_BUSY;

	stmac_mii_poll_busy(dev);

	/* Set the MII address register to write */
	STMAC_WRITE(value, MAC_MII_DATA);
	STMAC_WRITE(mii_addr, MAC_MII_ADDR);

	stmac_mii_poll_busy(dev);

#if defined(CONFIG_STMAC_STE10XP)	/* ST STE10xP PHY */
	/* QQQ: is the following actually needed ? */
	(void)stmac_mii_read(dev, phy_addr, reg);
#endif	/* CONFIG_STMAC_STE10XP */

#endif	/* CONFIG_STMAC_USE_FIXED_PHY */
}

static unsigned int stmac_mii_read(struct eth_device * const dev, int phy_addr, int reg)
{
	unsigned int val;

#if defined(CONFIG_STMAC_USE_FIXED_PHY)

	printf("Error: \"mii read\" is not supported for this PHY\n");

#else	/* CONFIG_STMAC_USE_FIXED_PHY */

	const unsigned int mii_addr =
		((phy_addr & MAC_MII_ADDR_PHY_MASK) << MAC_MII_ADDR_PHY_SHIFT)
		| ((reg & MAC_MII_ADDR_REG_MASK) << MAC_MII_ADDR_REG_SHIFT)
		| MAC_MII_ADDR_BUSY;

	/* Select register */
	stmac_mii_poll_busy(dev);

	STMAC_WRITE(mii_addr, MAC_MII_ADDR);

	stmac_mii_poll_busy(dev);

	/* Return read value */
	val = STMAC_READ(MAC_MII_DATA);

#endif	/* CONFIG_STMAC_USE_FIXED_PHY */

#if 0
	printf("QQQ: %s(addr=%u, reg=%u) --> value=0x%04x)\n", __FUNCTION__, phy_addr, reg, val);
#endif
	return val;
}


/* ----------------------------------------------------------------------------
				MII Call-Back Functions
   ---------------------------------------------------------------------------*/


#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII)
/* define external interface to mii, through miiphy_register() */
static int stmac_miiphy_read(const char *devname, unsigned char addr, unsigned char reg, unsigned short *value)
{
	struct eth_device * const dev = eth_get_dev_by_name(devname);
	*value = stmac_mii_read(dev, addr, reg);
#if 0
	printf("QQQ: %s(addr=%u, reg=%u) --> value=0x%04x)\n", __FUNCTION__, addr, reg, *value);
#endif
	return 0;
}

static int stmac_miiphy_write(const char *devname, unsigned char addr, unsigned char reg, unsigned short value)
{
	struct eth_device * const dev = eth_get_dev_by_name(devname);
#if 0
	printf("QQQ: %s(addr=%u, reg=%u, value=0x%04x)\n", __FUNCTION__, addr, reg, value);
#endif
	stmac_mii_write(dev, addr, reg, value);
	return 0;
}
#endif	/* CONFIG_MII || CONFIG_CMD_MII */


/* ----------------------------------------------------------------------------
				 MAC CORE Interface
   ---------------------------------------------------------------------------*/

#ifdef DEBUG
static void gmac_dump_regs(struct eth_device * const dev)
{
	int i;
	static const char fmt[] =
		"\tReg No. %2d (offset 0x%03x): 0x%08x\n";
	static const char header[] =
		"\t----------------------------------------------\n"
		"\t  %s registers (base addr = 0x%8x)\n"
		"\t----------------------------------------------\n";

	printf(header, "MAC CORE", (unsigned int)dev->iobase);
	for (i = 0; i < 18; i++) {
		int offset = i * 4;
		printf(fmt, i, offset, STMAC_READ(offset));
	}

	printf(header, "MAC DMA", (unsigned int)dev->iobase);
	for (i = 0; i < 9; i++) {
		int offset = i * 4;
		printf(fmt, i, (DMA_BUS_MODE + offset),
			STMAC_READ(DMA_BUS_MODE + offset));
	}

#ifdef CONFIG_DRIVER_NET_STM_GMAC
#if defined(STM_GMAC_AHB2STBUS_BASE)
	printf("\tSTBus bridge register (0x%08x) = 0x%08x\n",
		(unsigned int)(dev->iobase + STM_GMAC_AHB2STBUS_BASE),
		STMAC_READ(STM_GMAC_AHB2STBUS_BASE));
#endif	/* STM_GMAC_AHB2STBUS_BASE */
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */
}
#endif	/* DEBUG */

static void stmac_set_mac_addr(struct eth_device * const dev, unsigned char *Addr)
{
	unsigned long data;

	data = (Addr[5] << 8) | Addr[4];
	STMAC_WRITE(data, MAC_ADDR_HIGH);
	data = (Addr[3] << 24) | (Addr[2] << 16) | (Addr[1] << 8) | Addr[0];
	STMAC_WRITE(data, MAC_ADDR_LOW);
}

static int stmac_get_mac_addr(struct eth_device * const dev, unsigned char *addr)
{
	unsigned int hi_addr, lo_addr;

	/* Read the MAC address from the hardware */
	hi_addr = (unsigned int)STMAC_READ(MAC_ADDR_HIGH);
	lo_addr = (unsigned int)STMAC_READ(MAC_ADDR_LOW);

	/* only if all 48 bits are '1', then it is an invalid MAC address */
	if (((hi_addr & 0x0000FFFFU) == 0x0000FFFFU) && (lo_addr == 0xFFFFFFFFU))
		return 0;

	/* Extract the MAC address from the high and low words */
	addr[0] = lo_addr & 0xffu;
	addr[1] = (lo_addr >> 8) & 0xffu;
	addr[2] = (lo_addr >> 16) & 0xffu;
	addr[3] = (lo_addr >> 24) & 0xffu;
	addr[4] = hi_addr & 0xffu;
	addr[5] = (hi_addr >> 8) & 0xffu;

	return 1;
}

static void stmac_mac_enable(struct eth_device * const dev)
{
	unsigned int value = (unsigned int)STMAC_READ(MAC_CONTROL);

	PRINTK(STMAC "MAC RX/TX enabled\n");

	/* set: TE (transmitter enable), RE (receive enable) */
	value |= (MAC_CONTROL_TE | MAC_CONTROL_RE);

#ifdef CONFIG_DRIVER_NETSTMAC
	/* and RA (receive all mode) */
//	value |= MAC_CONTROL_RA;	/* QQQ: suspect we can delete this */
#endif	/* CONFIG_DRIVER_NETSTMAC */

	STMAC_WRITE(value, MAC_CONTROL);
	return;
}

static void stmac_mac_disable(struct eth_device * const dev)
{
	unsigned int value = (unsigned int)STMAC_READ(MAC_CONTROL);

	PRINTK(STMAC "MAC RX/TX disabled\n");

	value &= ~(MAC_CONTROL_TE | MAC_CONTROL_RE);

#ifdef CONFIG_DRIVER_NETSTMAC
//	value &= ~MAC_CONTROL_RA;	/* QQQ: suspect we can delete this */
#endif	/* CONFIG_DRIVER_NETSTMAC */

	STMAC_WRITE(value, MAC_CONTROL);
	return;
}

static void stmac_set_rx_mode(struct eth_device * const dev)
{
	unsigned int value = (unsigned int)STMAC_READ(MAC_CONTROL);

#ifdef CONFIG_DRIVER_NETSTMAC
	PRINTK(STMAC "MAC address perfect filtering only mode\n");
	value &= ~(MAC_CONTROL_PM | MAC_CONTROL_PR | MAC_CONTROL_IF |
		   MAC_CONTROL_HO | MAC_CONTROL_HP);
#endif	/* CONFIG_DRIVER_NETSTMAC */

	STMAC_WRITE(0x0, MAC_HASH_HIGH);
	STMAC_WRITE(0x0, MAC_HASH_LOW);

	STMAC_WRITE(value, MAC_CONTROL);

	return;
}

#if !defined(CONFIG_STMAC_USE_FIXED_PHY)
static void stmac_set_mac_mii_cap(struct eth_device * const dev, int full_duplex, unsigned int speed)
{
	unsigned int flow = (unsigned int)STMAC_READ(MAC_FLOW_CONTROL);
	unsigned int ctrl = (unsigned int)STMAC_READ(MAC_CONTROL);

	PRINTK(STMAC "%s(full_duplex=%d, speed=%u)\n", __FUNCTION__, full_duplex, speed);

	if (!(full_duplex)) {	/* Half Duplex */
#ifdef CONFIG_DRIVER_NETSTMAC
		flow &= ~(MAC_FLOW_CONTROL_FCE | MAC_FLOW_CONTROL_PT_MASK |
			  MAC_FLOW_CONTROL_PCF);
		ctrl &= ~MAC_CONTROL_F;
		ctrl |= MAC_CONTROL_DRO;
#endif	/* CONFIG_DRIVER_NETSTMAC */
#ifdef CONFIG_DRIVER_NET_STM_GMAC
		flow &= ~(MAC_FLOW_CONTROL_TFE | MAC_FLOW_CONTROL_PT_MASK |
			  MAC_FLOW_CONTROL_RFE);
		ctrl &= ~MAC_CONTROL_DM;
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */
	} else {		/* Full Duplex */
#ifdef CONFIG_DRIVER_NETSTMAC
		ctrl |= MAC_CONTROL_F;
		ctrl &= ~MAC_CONTROL_DRO;
		flow |= MAC_FLOW_CONTROL_FCE | MAC_FLOW_CONTROL_PCF;
#endif	/* CONFIG_DRIVER_NETSTMAC */
#ifdef CONFIG_DRIVER_NET_STM_GMAC
		ctrl |= MAC_CONTROL_DM;
		flow |= MAC_FLOW_CONTROL_TFE | MAC_FLOW_CONTROL_RFE;
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */
		flow |= (MAX_PAUSE_TIME << MAC_FLOW_CONTROL_PT_SHIFT);
	}

#ifdef CONFIG_DRIVER_NETSTMAC
	/* use MII */
	ctrl &= ~MAC_CONTROL_PS;
#endif	/* CONFIG_DRIVER_NETSTMAC */

#ifdef CONFIG_DRIVER_NET_STM_GMAC
	switch (speed) {
	case 1000:		/* Gigabit */
		ctrl &= ~MAC_CONTROL_PS;
		break;
	case 100:		/* 100Mbps */
		ctrl |= MAC_CONTROL_PS | MAC_CONTROL_FES;
		break;
	case 10:		/* 10Mbps */
		ctrl |= MAC_CONTROL_PS;
		ctrl &= ~MAC_CONTROL_FES;
		break;
	}
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */

	//STMAC_WRITE(flow, MAC_FLOW_CONTROL);
	STMAC_WRITE(ctrl, MAC_CONTROL);

	/* ensure the SoC knows the correct speed */
	stmac_set_mac_speed(speed);

	return;
}
#endif	/* !CONFIG_STMAC_USE_FIXED_PHY */

/* This function provides the initial setup of the MAC controller */
static void stmac_mac_core_init(struct eth_device * const dev)
{
	unsigned int value;

	/* Set the MAC control register with our default value */
	value = (unsigned int)STMAC_READ(MAC_CONTROL);
	value |= MAC_CORE_INIT;
	STMAC_WRITE(value, MAC_CONTROL);

#ifdef CONFIG_DRIVER_NET_STM_GMAC
#if defined(STM_GMAC_AHB2STBUS_BASE)
	/* STBus Bridge Configuration */
	STMAC_WRITE(STM_GMAC_AHB2STBUS_CONFIG, STM_GMAC_AHB2STBUS_BASE);
#endif	/* STM_GMAC_AHB2STBUS_BASE */

	/* Freeze MMC counters */
	STMAC_WRITE(MMC_COUNTER_FREEZE, MMC_CONTROL);

	/* Mask all interrupts */
	STMAC_WRITE(~0u, MAC_INT_MASK);
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */

	return;
}

/* ----------------------------------------------------------------------------
 *			DESCRIPTORS functions
 * ---------------------------------------------------------------------------*/

#ifdef DEBUG
static void display_dma_desc_ring(volatile const stmac_dma_des * p, int size)
{
	int i;
	for (i = 0; i < size; i++)
		printf("\t%d [0x%x]: "
			"desc0=0x%08x, desc1=0x%08x, buffer1=0x%08x\n",
			i, (unsigned int)&p[i].des01.u.des0,
			p[i].des01.u.des0, p[i].des01.u.des1,
			(unsigned int)p[i].des2);
}
#endif	/* DEBUG */

static void init_rx_desc(volatile stmac_dma_des * p,
	unsigned int ring_size, void **buffers)
{
	int i;
	stmac_dma_des * first = (stmac_dma_des *)p;
	stmac_dma_des * next  = (stmac_dma_des *)p;

	for (i = 0; i < ring_size; i++) {
		p->des01.u.des0 = p->des01.u.des1 = 0;
		p->des01.rx.own = 1;
		p->des01.rx.buffer1_size = PKTSIZE_ALIGN;
		p->des01.rx.disable_ic = 1;
		p->des01.rx.second_address_chained = 1; // Set mode in chained mode rather than ring (this enable us to be same size as cache line)
		if (i == ring_size - 1) {
        		p->des01.rx.end_ring = 1;
			p->des3 =  ((void *)(PHYSADDR (first)));
		}
		else {
			next = (stmac_dma_des *)p;
			next++;
			p->des3 =  ((void *)(PHYSADDR (next)));
		}
		p->des2 = ((void *) (PHYSADDR (buffers[i])));
		p++;
	}
	return;
}

static void init_tx_desc(volatile stmac_dma_des * p, unsigned int ring_size)
{
	int i;

	for (i = 0; i < ring_size; i++) {
		p->des01.u.des0 = p->des01.u.des1 = 0;
		if (i == ring_size - 1)
			p->des01.tx.end_ring = 1;
		p->des2 = NULL;
		p->des3 = NULL;
		p++;
	}
	return;
}

int allocate_dma_descriptors(void)
{
	static int dma_desc_avlbl;
	if(dma_desc_avlbl == 0){
		printf(STMAC "Allocating coherent descriptors \n");
		dma  = (struct dma_t *)noncached_alloc(sizeof(struct dma_t), 4*1024*1024);
		if(dma == NULL){
			printf(STMAC "Unable to allocate coherent descriptors \n");
			return -1;
		} else { 		
			printf("Successfully allocated coherent descriptors \n");
			dma_desc_avlbl = 1;
		}
	} else {
		printf(STMAC "Using already allocated descriptors.\n");
	}
	return 0;
}
/* Allocate and init the TX and RX descriptors rings.
 * The driver uses the 'implicit' scheme for implementing the TX/RX DMA
 * linked lists. */

static void init_dma_desc_rings(void)
{
	int i;
	unsigned int start_address;

	PRINTK(STMAC "allocate and init the DMA RX/TX lists\n");

	if(allocate_dma_descriptors() < 0){
		PRINTK(STMAC "Unable to allocate dma descriptors. \n");
		return;
	}
		
	/* Allocate memory for the DMA RX/TX buffer descriptors */
	dma_rx = (volatile stmac_dma_des *)  (&dma->desc_rx[0]);
	dma_tx = (volatile stmac_dma_des *)  (&dma->desc_tx[0]);

	cur_rx = 0;

	if ((dma_rx == NULL) || (dma_tx == NULL) ||
	    (((u32)dma_rx % L1_CACHE_BYTES) != 0) ||
	    (((u32)dma_tx % L1_CACHE_BYTES) != 0)) {
		printf(STMAC "ERROR allocating the DMA Tx/Rx desc\n");
		return;
	}

	/* Note: we want to pass CACHED addresses to init_rx_desc() */
	for (i = 0; i < CONFIG_DMA_RX_SIZE; i++){
		/* Set received buffer address aligned on cache line size */
		//start_address = ((unsigned int)dma.rx_buff + (PKTSIZE_ALIGN + L1_CACHE_BYTES * 2 ) * i + L1_CACHE_BYTES ) & ~(L1_CACHE_BYTES-1) ; 
		start_address = ((unsigned int)dma->rx_buff + (PKTSIZE_ALIGN + L1_CACHE_BYTES * 2 ) * i) & ~(L1_CACHE_BYTES-1) ; 
		rx_packets[i] = (void *)(start_address);
	}

	/* Initialize the contents of the DMA buffers */
	init_rx_desc(dma_rx, CONFIG_DMA_RX_SIZE, rx_packets);
	init_tx_desc(dma_tx, CONFIG_DMA_TX_SIZE);

#ifdef DEBUG
	printf(STMAC "RX descriptor ring:\n");
	display_dma_desc_ring(dma_rx, CONFIG_DMA_RX_SIZE);
	printf(STMAC "TX descriptor ring:\n");
	display_dma_desc_ring(dma_tx, CONFIG_DMA_TX_SIZE);
#endif

	return;
}

/* Release and free the descriptor resources. */
static void free_dma_desc_resources(void)
{
	dma_tx = NULL;
	dma_rx = NULL;
	return;
}

/* ----------------------------------------------------------------------------
				DMA FUNCTIONS
 * ---------------------------------------------------------------------------*/

/* DMA SW reset.
 *  NOTE1: the MII_TxClk and the MII_RxClk must be active before this
 *	   SW reset otherwise the MAC core won't exit the reset state.
 *  NOTE2: after a SW reset all interrupts are disabled */

static void stmac_dma_reset(struct eth_device * const dev)
{
	unsigned int value;

	value = (unsigned int)STMAC_READ(DMA_BUS_MODE);
	value |= DMA_BUS_MODE_SFT_RESET;

	STMAC_WRITE(value, DMA_BUS_MODE);

	while ((STMAC_READ(DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET)) {
	}

	return;
}

/* START/STOP the DMA TX/RX processes */
static void stmac_dma_start_tx(struct eth_device * const dev)
{
	unsigned int value;

	value = (unsigned int)STMAC_READ(DMA_CONTROL);
	value |= DMA_CONTROL_ST;
	STMAC_WRITE(value, DMA_CONTROL);

	return;
}

static void stmac_dma_stop_tx(struct eth_device * const dev)
{
	unsigned int value;

	value = (unsigned int)STMAC_READ(DMA_CONTROL);
	value &= ~DMA_CONTROL_ST;
	STMAC_WRITE(value, DMA_CONTROL);

	return;
}
static void stmac_dma_start_rx(struct eth_device * const dev)
{
	unsigned int value;

	value = (unsigned int)STMAC_READ(DMA_CONTROL);
	value |= (DMA_CONTROL_SR | DMA_CONTROL_DFF);
	STMAC_WRITE(value, DMA_CONTROL);

	return;
}

static void stmac_dma_stop_rx(struct eth_device * const dev)
{
	unsigned int value;

	value = (unsigned int)STMAC_READ(DMA_CONTROL);
	value &= ~DMA_CONTROL_SR;
	STMAC_WRITE(value, DMA_CONTROL);

	return;
}

static void stmac_eth_stop_tx(struct eth_device * const dev)
{
	stmac_dma_stop_tx(dev);

	return;
}

/* The DMA init function performs:
 * - the DMA RX/TX SW descriptors initialization
 * - the DMA HW controller initialization
 * NOTE: the DMA TX/RX processes will be started in the 'open' method. */

static int stmac_dma_init(struct eth_device * const dev)
{
	/* Note: PHYSADDR() needs the CACHED address (not the PHYSICAL one) */
	stmac_dma_des * const dma_rx = (stmac_dma_des *)(&dma->desc_rx[0]);
	stmac_dma_des * const dma_tx = (stmac_dma_des *)(&dma->desc_tx[0]);

	PRINTK(STMAC "DMA Core setup\n");

	/* Enable Application Access by writing to DMA CSR0 */
	STMAC_WRITE(DMA_BUS_MODE_DEFAULT |
		     (stmac_default_pbl() << DMA_BUS_MODE_PBL_SHIFT),
		     DMA_BUS_MODE);

	/* Disable interrupts */
	STMAC_WRITE(0, DMA_INTR_ENA);

	/* The base address of the RX/TX descriptor */
	STMAC_WRITE ((unsigned int)(PHYSADDR (dma_tx)), DMA_TX_BASE_ADDR);
	STMAC_WRITE ((unsigned int)(PHYSADDR (dma_rx)), DMA_RCV_BASE_ADDR);

	return (0);
}

static int check_tx_error_summary(const stmac_dma_des * const p)
{
	int ret = 0;	/* assume there are no errors */

	if (unlikely(p->des01.tx.error_summary)) {
		if (unlikely(p->des01.tx.loss_carrier)) {
			PRINTK(STMAC "TX: loss_carrier error\n");
		}
		if (unlikely(p->des01.tx.no_carrier)) {
			PRINTK(STMAC "TX: no_carrier error\n");
		}
		if (unlikely(p->des01.tx.late_collision)) {
			PRINTK(STMAC "TX: late_collision error\n");
		}
		if (unlikely(p->des01.tx.excessive_collisions)) {
			PRINTK(STMAC "TX: excessive_collisions\n");
		}
		if (unlikely(p->des01.tx.excessive_deferral)) {
			PRINTK(STMAC "TX: excessive_deferral\n");
		}
		if (unlikely(p->des01.tx.underflow_error)) {
			PRINTK(STMAC "TX: underflow error\n");
		}
#ifdef CONFIG_DRIVER_NET_STM_GMAC
		if (unlikely(p->des01.tx.jabber_timeout)) {
			PRINTK(STMAC "TX: jabber_timeout error\n");
		}
		if (unlikely(p->des01.tx.frame_flushed)) {
			PRINTK(STMAC "TX: frame_flushed error\n");
		}
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */
		ret = -1;
	}

	if (unlikely(p->des01.tx.deferred)) {
		PRINTK(STMAC "TX: deferred\n");
		ret = -1;
	}
#ifdef CONFIG_DRIVER_NETSTMAC
	if (unlikely(p->des01.tx.heartbeat_fail)) {
		PRINTK(STMAC "TX: heartbeat_fail\n");
		ret = -1;
	}
#endif	/* CONFIG_DRIVER_NETSTMAC */
#ifdef CONFIG_DRIVER_NET_STM_GMAC
	if (unlikely(p->des01.tx.payload_error)) {
		PRINTK(STMAC "TX Addr/Payload csum error\n");
		ret = -1;
	}
	if (unlikely(p->des01.tx.ip_header_error)) {
		PRINTK(STMAC "TX IP header csum error\n");
		ret = -1;
	}
	if (p->des01.tx.vlan_frame) {
		PRINTK(STMAC "TX: VLAN frame\n");
	}
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */

#ifdef DEBUG
	if (ret != 0) {
		printf(STMAC "%s() returning %d\n", __FUNCTION__, ret);
	}
#endif

	return (ret);
}

static int check_rx_error_summary(const stmac_dma_des * const p)
{
	int ret = 0;	/* assume there are no errors */

	if (unlikely(p->des01.rx.error_summary)) {
		if (unlikely(p->des01.rx.descriptor_error)) {
			/* frame doesn't fit within the current descriptor. */
			PRINTK(STMAC "RX: descriptor error\n");
		}
		if (unlikely(p->des01.rx.crc_error)) {
			PRINTK(STMAC "RX: CRC error\n");
		}
#ifdef CONFIG_DRIVER_NETSTMAC
		if (unlikely(p->des01.rx.partial_frame_error)) {
			PRINTK(STMAC "RX: partial_frame_error\n");
		}
		if (unlikely(p->des01.rx.runt_frame)) {
			PRINTK(STMAC "RX: runt_frame\n");
		}
		if (unlikely(p->des01.rx.frame_too_long)) {
			PRINTK(STMAC "RX: frame_too_long\n");
		}
		if (unlikely(p->des01.rx.collision)) {
			PRINTK(STMAC "RX: collision\n");
		}
#endif	/* CONFIG_DRIVER_NETSTMAC */
#ifdef CONFIG_DRIVER_NET_STM_GMAC
		if (unlikely(p->des01.rx.overflow_error)) {
			PRINTK(STMAC "RX: Overflow error\n");
		}
		if (unlikely(p->des01.rx.late_collision)) {
			PRINTK(STMAC "RX: late_collision\n");
		}
		if (unlikely(p->des01.rx.receive_watchdog)) {
			PRINTK(STMAC "RX: receive_watchdog error\n");
		}
		if (unlikely(p->des01.rx.error_gmii)) {
			PRINTK(STMAC "RX: GMII error\n");
		}
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */
		ret = -1;
	}

	if (unlikely(p->des01.rx.length_error)) {
		PRINTK(STMAC "RX: length_error error\n");
		ret = -1;
	}
	if (unlikely(p->des01.rx.dribbling)) {
		PRINTK(STMAC "RX: dribbling error\n");
		ret = -1;
	}
#ifdef CONFIG_DRIVER_NET_STM_GMAC
	if (unlikely(p->des01.rx.filtering_fail)) {
		PRINTK(STMAC "RX: filtering_fail error\n");
		ret = -1;
	}
#endif	/* CONFIG_DRIVER_NET_STM_GMAC */
#ifdef CONFIG_DRIVER_NETSTMAC
	if (unlikely(p->des01.rx.last_descriptor == 0)) {
		PRINTK(STMAC "RX: Oversized Ethernet "
			"frame spanned multiple buffers\n");
		ret = -1;
	}
	if (unlikely(p->des01.rx.mii_error)) {
		PRINTK(STMAC "RX: MII error\n");
		ret = -1;
	}
#endif	/* CONFIG_DRIVER_NETSTMAC */

#ifdef DEBUG
	if (ret != 0) {
		printf(STMAC "%s() returning %d\n", __FUNCTION__, ret);
	}
#endif

	return (ret);
}

static int stmac_eth_tx(struct eth_device * const dev, volatile uchar * data, int len)
{
	volatile stmac_dma_des *p = dma_tx;
	uint now = get_timer(0);
	uint status = 0;
	u32 end_ring;
	unsigned long aligned_data;

	while (p->des01.tx.own
	       && (get_timer(now) < CONFIG_STMAC_TX_TIMEOUT)) {
		;
	}

	if (p->des01.tx.own) {
		printf(STMAC "tx timeout - no desc available\n");
		return -1;
	}

	aligned_data = (unsigned long)data;
	/* Make sure data is in real memory */
	flush_dcache_all();
	p->des2 = (void *)PHYSADDR(ALIGN(aligned_data, L1_CACHE_BYTES));

	/* Clean and set the TX descriptor */
	end_ring = p->des01.tx.end_ring;
	p->des01.u.des0 = p->des01.u.des1 = 0;
	p->des01.tx.interrupt = 1;
	p->des01.tx.first_segment = 1;
	p->des01.tx.last_segment = 1;
	p->des01.tx.end_ring = end_ring;
	p->des01.tx.buffer1_size = len;
	p->des01.tx.own = 1;

#ifdef DEBUG
	PRINTK("\n" STMAC "TX %s(data=0x%08x, len=%d)\n",
		__FUNCTION__, (unsigned int)data, len);
	display_dma_desc_ring(dma_tx, CONFIG_DMA_TX_SIZE);
#endif

	/* CSR1 enables the transmit DMA to check for new descriptor */
	STMAC_WRITE(DMA_STATUS_TI, DMA_STATUS);
	STMAC_WRITE(1, DMA_XMT_POLL_DEMAND);
	now = get_timer(0);
	while (get_timer(now) < CONFIG_STMAC_TX_TIMEOUT) {
		status = STMAC_READ(DMA_STATUS);
		if (status & DMA_STATUS_TI)
			break;
	}
	if (!(status & DMA_STATUS_TI)) {
		printf(STMAC "tx timeout\n");
	}

	return check_tx_error_summary((stmac_dma_des *)p);
}

/* Receive function */
static void stmac_eth_rx(struct eth_device * const dev)
{
	int frame_len = 0;
	volatile stmac_dma_des *drx;

	/* select the RX descriptor to use */
	drx = dma_rx + cur_rx;

	if ((cur_rx < 0) || (cur_rx >= CONFIG_DMA_RX_SIZE)) {
		printf(STMAC "%s: [dma drx = 0x%x, cur_rx=%d]\n", __FUNCTION__,
			(unsigned int)drx, cur_rx);
#ifdef DEBUG
		display_dma_desc_ring(dma_rx, CONFIG_DMA_RX_SIZE);
#endif	/* DEBUG */
	}

	if (!(drx->des01.rx.own) && (drx->des01.rx.last_descriptor)) {
#ifdef DEBUG
		PRINTK(STMAC "RX descriptor ring:\n");
		display_dma_desc_ring(dma_rx, CONFIG_DMA_RX_SIZE);
#endif

		/* Check if the frame was not successfully received */
		if (check_rx_error_summary((stmac_dma_des *)drx) < 0) {
			drx->des01.rx.own = 1;
		} else if (drx->des01.rx.first_descriptor
			   && drx->des01.rx.last_descriptor) {

			/* FL (frame length) indicates the length in byte including
			 * the CRC */
			frame_len = drx->des01.rx.frame_length;
			if ((frame_len >= 0) && (frame_len <= PKTSIZE_ALIGN)) {

#if defined(DEBUG) || defined(CONFIG_PHY_LOOPBACK) || defined(DUMP_ENCAPSULATION_HEADER)
				const unsigned char * const p = rx_packets[cur_rx];
				printf("\nRX[%d]:  0x%08x DA=%pM SA=%pM Type=%04x\n",
					cur_rx, (unsigned int)p, p, p+6, p[12]<<8|p[13]);
#endif
				memcpy((void*)net_rx_packets[0], rx_packets[cur_rx],
					frame_len);
				net_process_received_packet(net_rx_packets[0], frame_len);
			} else {
				printf(STMAC "%s: Framelen %d too long\n",
					__FUNCTION__, frame_len);
			}
			drx->des01.rx.own = 1;
#ifdef DEBUG
			PRINTK(STMAC "%s: frame received \n", __FUNCTION__);
#endif
		} else {
			printf(STMAC "%s: very long frame received\n",
				__FUNCTION__);
		}

		/* advance to the next RX descriptor (for next time) */
		if (drx->des01.rx.end_ring)
			cur_rx = 0;	/* wrap, to first */
		else
			cur_rx++;	/* advance to next */

	} else {
		STMAC_WRITE(1, DMA_RCV_POLL_DEMAND);	/* request input */
	}
	return;
}

static int stmac_get_ethaddr(struct eth_device * const dev)
{
	int rom_valid, env_valid;
	uchar v_env_mac[6], v_rom_mac[6], *v_mac;

	/*
	 * Get the MAC address from the environment variable "ethaddr".
	 * Returns 1, if environment exists, and the MAC is valid.
	 */
	env_valid = eth_getenv_enetaddr("ethaddr", v_env_mac);

	/*
	 * Get the MAC address from the ROM via the MAC/GMAC hardware.
	 * Returns 1, if the MAC is valid.
	 */
	rom_valid = stmac_get_mac_addr(dev, v_rom_mac);

	if (!env_valid) {	/* if NO valid MAC in the environment */
		if (rom_valid) {	/* but ROM is valid */
			eth_setenv_enetaddr("ethaddr", v_mac=v_rom_mac);
		} else {	/* invalid env, and bad ROM - give up! */
			printf("\n*** ERROR: ethaddr is NOT set !!\n");
			return (-1);
		}
	} else {		/* good env, don't care ROM */
		v_mac = v_env_mac;	/* always use a good env over a ROM */
	}

	if (env_valid && rom_valid) {	/* if both env and ROM are good */
		if (memcmp(v_env_mac, v_rom_mac, 6) != 0) {
			printf("\nWarning: MAC addresses don't match:\n");
			printf("\tHW MAC address:  %pM\n", v_rom_mac);
			printf("\t\"ethaddr\" value: %pM\n", v_env_mac);
		}
	}

	printf("Using MAC Address %pM\n", v_mac);
	stmac_set_mac_addr(dev, v_mac);	/* update H/W (volatile only) */

	return (0);
}

static int stmac_reset_eth(struct eth_device * const dev)
{
	/* MAC Software reset */
	stmac_dma_reset(dev);		/* Must be done early  */

	/*
	 * set MAC address (in H/W), and sync it with the global
	 * U-Boot environment variable "ethaddr".
	 */
	if (stmac_get_ethaddr(dev) < 0) {
		return (-1);
	}

	if (stmac_phy_init(dev) < 0) {
		printf(STMAC "ERROR: no PHY detected\n");
		return -1;
	}

	init_dma_desc_rings();

	stmac_mac_core_init(dev);
	stmac_dma_init(dev);

	stmac_set_rx_mode(dev);

	stmac_mac_enable(dev);

	stmac_dma_start_rx(dev);
	stmac_dma_start_tx(dev);

#ifdef DEBUG
	gmac_dump_regs(dev);
#endif

	STMAC_WRITE(1, DMA_RCV_POLL_DEMAND);	/* request input */

	return (0);
}


/* ----------------------------------------------------------------------------
				Ethernet API Call-Back Functions
   ---------------------------------------------------------------------------*/


static int stmac_init(
	struct eth_device * const dev,
	bd_t * const bis)
{
	PRINTK(STMAC "entering %s()\n", __FUNCTION__);

	return stmac_reset_eth(dev);
}

static void stmac_halt(struct eth_device * const dev)
{
	PRINTK(STMAC "entering %s()\n", __FUNCTION__);

	/* Reset the TX/RX processes */
	stmac_dma_stop_rx(dev);
	stmac_eth_stop_tx(dev);

	/* Disable the MAC core */
	stmac_mac_disable(dev);

	/* Free buffers */
	free_dma_desc_resources();
}

/* Get a data block via Ethernet */
static int stmac_rx(struct eth_device * const dev)
{
	stmac_eth_rx(dev);

	return 1;
}

/* Send a data block via Ethernet. */
static int stmac_send(
	struct eth_device * const dev,
	void * const packet,
	const int length)
{
	PRINTK(STMAC "entering %s()\n", __FUNCTION__);
#if defined(DEBUG) || defined(CONFIG_PHY_LOOPBACK) || defined(DUMP_ENCAPSULATION_HEADER)
	const unsigned char * const p = (const unsigned char*)packet;
	printf("TX   :  0x%08x DA=%pM SA=%pM Type=%04x\n",
		(unsigned int)p, p, p+6, p[12]<<8|p[13]);
#endif

	return stmac_eth_tx(dev, packet, length);
}

/* ----------------------------------------------------------------------------
				Ethernet Registration Function
   ---------------------------------------------------------------------------*/


extern int stmac_eth_register(
	const int id,
	const u32 base_addr)
{
	struct stmac_private *priv;
	struct eth_device *dev;

	PRINTK(STMAC "entering %s()\n", __FUNCTION__);

	/* allocate memory for the 2 new structures */
	priv = malloc(sizeof(*priv));
	if (priv == NULL)
		return -1;
	dev = malloc(sizeof(*dev));
	if (dev == NULL) {
		free(priv);
		return -1;
	}

	/* initialize the "device" structure */
	memset(dev, 0, sizeof(*dev));
	sprintf(dev->name, "%s-%u", "stmac", id);
	dev->iobase = base_addr;

	/* initialize the "private" structure */
	dev->priv = priv;
	priv->id  = id;

	/* also set up the call-back functions */
	dev->init = stmac_init;
	dev->halt = stmac_halt;
	dev->send = stmac_send;
	dev->recv = stmac_rx;
//	dev->write_hwaddr = stmac_write_hwaddr;

	/* now register the ethernet device */
	eth_register(dev);

#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII)
	/* finally, add support for MII */
//	miiphy_register(dev->name, stmac_miiphy_read, stmac_miiphy_write);
#endif	/* CONFIG_MII || CONFIG_CMD_MII */

	return 0;
}


#endif /* CONFIG_CMD_NET */

#endif /* CONFIG_DRIVER_NETSTMAC || CONFIG_DRIVER_NET_STM_GMAC */
