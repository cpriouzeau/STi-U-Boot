/*
 * (C) Copyright 2008-2014 STMicroelectronics.
 *
 * Sean McGoogan <Sean.McGoogan@st.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SYS_TEXT_BASE 0x7D600000
#define CONFIG_SYS_SDRAM_BASE 0x40000000
#define CONFIG_SYS_SDRAM_SIZE 0x3FE00000
#define CONFIG_NR_DRAM_BANKS		1		/* Number of memory banks */

#define CONFIG_SYS_NONCACHED_MEMORY (4*1024*1024)  
#define STM_BOARD "B2260"
#define CONFIG_SYS_MAXARGS         16      /* max number of command args */

#define CONFIG_SYS_MAX_FLASH_BANKS                1

#define CONFIG_SYS_CBSIZE               1024    /* Console I/O Buffer Size  */

#define CONFIG_STM_FSM_SPI_FLASH

#define CONFIG_SYS_BOOTM_LEN        (64 << 20)	/* Increase max gunzip size */

#define BOARD b2260

#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C

#undef CONFIG_DM_USB

#define CONFIG_STM_USB
#define STM_USB3_SUPPORT
#define STM_USB2_SUPPORT
#define CONFIG_USE_EHCI
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#ifdef CONFIG_DM_USB
#define CONFIG_DM
#define CONFIG_DM_DEVICE_REMOVE
#endif
#define CONFIG_SPI_FLASH		/* define for SPI serial flash */
#define CONFIG_STM_SDHCI_0		/* define for MMC #0 (MMC/SD) */
#define CONFIG_SPI_FLASH_ST		/* ST N25Q256 */
#define CONFIG_STM_ASC_SERIAL		/* use a ST ASC UART */
#define CONFIG_SYS_STM_ASC_BASE STXH410_ASC1_BASE
#define CONFIG_DRIVER_NET_STM_GMAC
#define CONFIG_MACH_STM_STXH410_A9SS_VCORE_HACK
#undef CONFIG_STM_SATA
#define CONFIG_SYS_NO_FLASH
#define CONFIG_STM_FSM_SPI_FLASH                 /* Use the H/W FSM for SPI, to use
                                                 * bit bang mode define CONFIG_SOFT_SPI here.
                                                 * SPI flash controller with SSC is not being
                                                 * provided yet.*/


#define CONFIG_ENV_OVERWRITE
#define STM_I2C_BUS_NUM			5
#define CONFIG_SYS_CPU_CLK		1000000000	/* 1 GHz */
#define CONFIG_STM_SPI_CLOCKDIV		4
#define CONFIG_MACH_TYPE		4747
#define CONFIG_L2_OFF					/* Disable the L2 caches */



#define CONFIG_DMA_COHERENT
#define CONFIG_DMA_COHERENT_SIZE	(4*1024*1024)

#if defined(CONFIG_USE_XHCI) &&  defined (CONFIG_USB_XHCI_PCI)
#	undef	CONFIG_USB_XHCI_STM
#	undef	CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS
#	define	CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS 4 /*The PCI-USB adapter i am using has 4 ports*/
#endif



#if defined(CONFIG_STM_SATA) || defined(CONFIG_STM_USB)
#	define CONFIG_SYS_64BIT_LBA
#	define CONFIG_LBA48
#	define CONFIG_DOS_PARTITION
#	define CONFIG_CMD_EXT2
#endif

#ifdef CONFIG_DRIVER_NET_STM_GMAC
#       define CONFIG_SYS_STM_STMAC_BASE        CONFIG_STM_STMAC_BASE    /* GMAC #1 */
#       define CONFIG_STMAC_RTL8211E           /* RealTek RTL8211E */
#endif  /* CONFIG_DRIVER_NET_STM_GMAC */

#define STM_BLIT_BASE_ADDRESS             0x6050000
#define STM_DMU_BASE_ADDRESS              0x6040000
#define STM_ESRAM_SECURE_BASE_ADDRESS     0x6058000
#define STM_HVA_BASE_ADDRESS              0x6000000
#include <stm/env.h>
#ifndef CONFIG_ENV_SIZE
#define CONFIG_ENV_SIZE 0x4000
#endif
#include <stm/usb.h>
#include <stm/spi_flash.h>
#include <stm/memory.h>
#include <stm/usb.h>
#include <stm/misc.h>
#include <stm/sdhci.h>
#include <stm/i2c.h>
#include <stm/nand.h>
#include <stm/nbs.h>
#include <stm/ir.h>
	
#define PIOALT(port, pin, alt, dir)                     \
do                                                      \
{                                                       \
        stm_pioalt_select((port), (pin), (alt));    \
        stm_pioalt_pad((port), (pin), (dir));       \
} while(0)
#define SYSCONF(_reg)   ((unsigned long*)STXH410_SYSCFG(_reg))
#define CONFIG_SYS_EMI_SPI_BASE 0x00000000

#define SPI_nCS         40, 0           /* SPI Chip-Select */
#define SPI_CLK         40, 1           /* SPI Clock */
#define SPI_MOSI        40, 2           /* D0 / Master Out, Slave In */
#define SPI_MISO        40, 3           /* D1 / Master In, Slave Out */
#define SPI_nWP         40, 4           /* D2 / SPI Write Protect */
#define SPI_HOLD        40, 5           /* D3 / SPI Hold */


#define SYSCONF_MMC_ENABLE_MMC1		5132
#define SYSCONF_MMC1_ENABLE_BIT		3
#define SYSCONF_BOOT_MODE		5561
#define SYSCONF_BOOT_MASK		0x7c
#define SYSCONF_BOOT_MMC		0x78 
#define SYSCONF_MMC0_PIO_CONF 		3080

#define CONFIG_CMD_BOOTA
#endif	/* __CONFIG_H */
