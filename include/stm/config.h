/*
 * Copyright 2009 Freescale Semiconductor, Inc.
 *
 * Copyright 2012-2013 STMicroelectronics Ltd.
 *	Sean McGoogan <Sean.McGoogan@st.com>
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
 *
 */

#ifndef __INCLUDE_STM_CONFIG_H
#define __INCLUDE_STM_CONFIG_H

#define QUOTEME(M)       #M
#define INCLUDE_FILE(M)  QUOTEME(M)

/* Relocation to SDRAM works on all ST40 boards */
#define CONFIG_RELOC_FIXUP_WORKS

/* Enable use of the Logical Memory Blocks (LMB). */
#define CONFIG_LMB

/* Enable the "initrd_high" functionality. */
#define CONFIG_SYS_BOOT_RAMDISK_HIGH

	/*
	 * Increase some macros which control the maximum sizes
	 * of buffers/arrays for processing U-Boot commands.
	 * Building up linux command lines (e.g. bootargs)
	 * often needs these limits raised, as the linux
	 * command lines tend to grow (and grow)!
	 */
#if !defined(CONFIG_SYS_CBSIZE)		/* (input) Console Buffer size */
#	define CONFIG_SYS_CBSIZE	(2*1024)	/* 2KiB */
#endif

#if !defined(CONFIG_SYS_PBSIZE)		/* (output) Print Buffer size */
#	define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)
#endif

#if !defined(CONFIG_SYS_MAXARGS)	/* max number of command arguments */
#	define CONFIG_SYS_MAXARGS	32
#endif

	/*
	 * pull in the {in,out}[s][bwl] family of macros from
	 * the file arch/arm/include/asm/io.h
	 */
#	define __io

	/*
	 * add additional (core-specific) CONFIG_* options.
	 */
#	define CONFIG_ARCH_CPU_INIT	/* call arch_cpu_init() */
#	define CONFIG_DISPLAY_BOARDINFO	/* call checkboard() */
#	define CONFIG_MISC_INIT_R	/* call misc_init_r() */
#	define CONFIG_CMD_CACHE		/* enable "icache" & "dcache" commands */
#	define CONFIG_SETUP_MEMORY_TAGS	/* use ATAGs for Memory Specification */
#	define CONFIG_CMDLINE_TAG	/* use ATAGs for Kernel's Command Line */
#	define CONFIG_INITRD_TAG	/* use ATAGs for an init ramdisk */
#	define CONFIG_CMD_BOOTZ		/* enable zImag boot */

		/* address of the initial Stack Pointer (SP) */
#	define CONFIG_SYS_INIT_SP_ADDR			\
			( CONFIG_SYS_TEXT_BASE -	\
			  CONFIG_SYS_MALLOC_LEN -	\
			  CONFIG_SYS_GBL_DATA_SIZE )

#define CONFIG_STM_SBC_EXIT_CPS_REG		0x094b5120  /* Exit CPS register */

	/*
	 * For all STMicroelectronics board, we now want to use the
	 * "newer" (multi-MAC) network API (CONFIG_NET_MULTI).
	 * The "older" API is deprecated, and about to be removed.
	 */
#define CONFIG_NET_MULTI


	/*
	 * Call board_early_init_f() for STMicroelectronics' boards.
	 * This is mainly for all *early* board-specific PIO
	 * configuration, specifically used to enable the UART (ASC),
	 * for the main serial console (as early as we can).
	 */
#define CONFIG_BOARD_EARLY_INIT_F	/* call board_early_init_f() */


#endif /* __INCLUDE_STM_CONFIG_H */
