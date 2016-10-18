/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#define CONFIG_BAUDRATE                 115200
#define CONFIG_SYS_BAUDRATE_TABLE       { 9600, 19200, 38400, 57600, 115200 }

#define XSTR(s) STR(s)
#define STR(s) #s

#define CONFIG_SYS_HUSH_PARSER          1
#define CONFIG_HUSH_PARSER		1
#define CONFIG_AUTO_COMPLETE            1
#define CONFIG_SYS_LONGHELP             1                       /* undef to save memory         */
#define CONFIG_SYS_PROMPT_HUSH_PS2      "> "
#define CONFIG_SYS_LOAD_ADDR            CONFIG_SYS_SDRAM_BASE   /* default load address         */
#if !defined(CONFIG_BOOTDELAY)
#define CONFIG_BOOTDELAY                10                      /* default delay before executing bootcmd */
#endif
#define CONFIG_ZERO_BOOTDELAY_CHECK

#define CONFIG_CMDLINE_EDITING


#define CONFIG_EXTRA_ENV_SETTINGS \
                "board=" XSTR(BOARD) "\0" \
                "load_addr=" XSTR(CONFIG_SYS_LOAD_ADDR) "\0"

/*--------------------------------------------------------------
 * Command line configuration.
 */

#include <config_cmd_default.h>

#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_NFS
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_SETEXPR

/*--------------------------------------------------------------
 * Serial console info
 */
/*#define CONFIG_SYS_STM_ASC_BASE STM_ASC_BASE*/   /* SBC_UART0 on J12 (off-board) */

