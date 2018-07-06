/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */
#define CONFIG_ENV_SIZE_REQUIRED	0x4000 /* 16 KiB of environment data required */

#if defined(CONFIG_SYS_BOOT_FROM_SDUSB)
#	define CONFIG_ENV_IS_NOWHERE /* ENV is stored in volatile RAM */
#	define CONFIG_ENV_SIZE		0x4000
#	if defined(CONFIG_TARGET_B2260_STXH410)
#		define CONFIG_BOOTCOMMAND \
		"setenv load_env_addr 0x40000000;" \
		"setenv env_import_1 '" \
		"if fatls mmc 0:1; then " \
			"if fatsize mmc 0:1 uenv_sd_$board.txt; then " \
				"fatload mmc 0:1 $load_env_addr uenv_sd_$board.txt;" \
			"else " \
				"fatload mmc 0:1 $load_env_addr /$board/uEnv_sd.txt;" \
			"fi;" \
			"env import -t $load_env_addr $filesize;" \
			"boot;" \
		"else run env_import_2;" \
		"fi;';" \
		"setenv env_import_2 '" \
		"usb start;" \
		"if fatls usb 0; then " \
			"if fatsize usb 0 uenv_usb_$board.txt; then " \
				"fatload usb 0 $load_env_addr uenv_usb_$board.txt;" \
			"else " \
				"fatload usb 0 $load_env_addr /$board/uEnv_usb.txt;" \
			"fi;" \
			"env import -t $load_env_addr $filesize;" \
			"boot;" \
		"else echo 'ERROR: No Environment found';" \
		"fi;" \
		"usb stop;';" \
		"run env_import_1;"
#	endif /* if defined(CONFIG_TARGET_B2260_STXH410) */
#	define CONFIG_FAT_WRITE
#	define CONFIG_EXT4_WRITE
#	define CONFIG_CMD_FAT
#	define CONFIG_CMD_EXT4
#	define CONFIG_CMD_EXT4_WRITE
#	define CONFIG_CMD_FS_GENERIC
#	define CONFIG_SUPPORT_VFAT
#else
#	define CONFIG_ENV_IS_NOWHERE	/* ENV is stored in volatile RAM */
#	undef CONFIG_CMD_SAVEENV	/* no need for "saveenv" */
#endif

