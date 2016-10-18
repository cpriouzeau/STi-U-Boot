/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */
#define CONFIG_ENV_SIZE_REQUIRED        0x4000  /* 16 KiB of environment data required */

#if defined(CONFIG_SYS_BOOT_FROM_SDUSB)
#       define CONFIG_ENV_IS_NOWHERE            /* ENV is stored in volatile RAM */
#       define CONFIG_ENV_SIZE		0x4000
#       define CONFIG_BOOTDELAY         3
#       if defined(CONFIG_TARGET_B2260_STXH410)
#           define CONFIG_BOOTCOMMAND		"\\\n"	\
		"setenv load_env_addr 0x40000000;"		"\\\n" \
		"setenv env_import_1 '"		"\\\n" \
		"  if fatls mmc 0:1; then "		"\\\n" \
		"    if fatsize mmc 0:1 uenv_sd_$board.txt; then "		"\\\n" \
		"      fatload mmc 0:1 $load_env_addr uenv_sd_$board.txt;"		"\\\n" \
		"    else "		"\\\n" \
		"      fatload mmc 0:1 $load_env_addr /$board/uEnv_sd.txt;"		"\\\n" \
		"      fatwrite mmc 0:1 $load_env_addr uenv_sd_$board.txt \${filesize};"		"\\\n" \
		"    fi;"		"\\\n" \
		"    env import -t $load_env_addr $filesize;"		"\\\n" \
		"    boot;"		"\\\n" \
		"  else run env_import_2;"		"\\\n" \
		"  fi;';"		"\\\n" \
	        "setenv env_import_2 '"		"\\\n" \
		"  usb start;"		"\\\n" \
		"  if fatls usb 0; then "		"\\\n" \
		"    if fatsize usb 0 uenv_usb_$board.txt; then "		"\\\n" \
		"      fatload usb 0 $load_env_addr uenv_usb_$board.txt;"		"\\\n" \
		"    else "		"\\\n" \
		"      fatload usb 0 $load_env_addr /$board/uEnv_usb.txt;"		"\\\n" \
		"      fatwrite usb 0 $load_env_addr uenv_usb_$board.txt \${filesize};"		"\\\n" \
		"    fi;"		"\\\n" \
		"    env import -t $load_env_addr $filesize;"		"\\\n" \
		"    boot;"		"\\\n" \
		"  else echo 'ERROR: No Environment found';"		"\\\n" \
		"  fi;"		"\\\n" \
		"  usb stop;';"		"\\\n" \
		"run env_import_1;"
#       endif /* if defined(CONFIG_TARGET_B2260_STXH410) */
#       define CONFIG_FAT_WRITE
#       define CONFIG_EXT4_WRITE
#       define CONFIG_CMD_FAT
#       define CONFIG_CMD_EXT4
#       define CONFIG_CMD_EXT4_WRITE
#       define CONFIG_CMD_FS_GENERIC
#       define CONFIG_SUPPORT_VFAT
#else
#       define CONFIG_ENV_IS_NOWHERE            /* ENV is stored in volatile RAM */
#       undef  CONFIG_CMD_SAVEENV               /* no need for "saveenv" */
#endif

