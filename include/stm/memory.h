/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */
#define PHYS_DDR_1              CONFIG_SYS_SDRAM_BASE
#define PHYS_DDR_1_SIZE         CONFIG_SYS_SDRAM_SIZE

#define CONFIG_SYS_MONITOR_LEN          0x100000      /* Reserve 1 MiB for Monitor */
#define CONFIG_SYS_MALLOC_LEN           0x1800000       /* Reserve 24 MiB for malloc */
#define CONFIG_SYS_GBL_DATA_SIZE        (1024-8)        /* Global data structures */

#define CONFIG_SYS_MEMTEST_START        CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END          (CONFIG_SYS_TEXT_BASE - CONFIG_SYS_MALLOC_LEN - (0x100000))

