/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

/*-----------------------------------------------------------------------
 * eMMC/MMC organization
 */

#if defined(CONFIG_STM_SDHCI_0) || defined(CONFIG_STM_SDHCI_1)
#       define CONFIG_STM_SDHCI
#       define CONFIG_SDHCI
#       define CONFIG_MMC
#       define CONFIG_GENERIC_MMC
#       define CONFIG_CMD_MMC
#       define CONFIG_CMD_GPT
#       define CONFIG_PARTITION_UUIDS
#       define CONFIG_EFI_PARTITION
#       if defined(CONFIG_STM_SDHCI_0)
#               define CONFIG_SUPPORT_EMMC_BOOT
#               define CONFIG_EMMC_BOOT_MODE_BIT        0               /* use 1-bit boot-mode for cut0 */
#       endif   /* CONFIG_STM_SDHCI_0 */
#endif  /* CONFIG_STM_SDHCI_0 || CONFIG_STM_SDHCI_1 */
