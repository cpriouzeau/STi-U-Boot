/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#ifdef CONFIG_CMD_NAND
#       define CONFIG_SYS_MAX_NAND_DEVICE       1
#       define CONFIG_SYS_NAND0_BASE            CONFIG_SYS_EMI_NAND_BASE
#       define CONFIG_SYS_NAND_BASE_LIST        { CONFIG_SYS_NAND0_BASE }

#       define MTDPARTS_NAND                                            \
        "gen_nand.1:"           /* First NAND flash device */           \
        "1M(uboot)"             /* reserve some space for uboot */      \
        ",8M(kernel)"           /* kernel space */                      \
        ",-(rootfs)"            /* rootfs */

#       define MTDIDS_NAND                                              \
        "nand0=gen_nand.1"      /* First NAND flash device */

        /*
         * Enable this, if support for probing of ONFI complaint NAND
         * devices is also required (typically recommended).
         */
#       define CONFIG_SYS_NAND_ONFI_DETECTION   /* define for probing ONFI devices */

        /*
         * With modern large NAND devices, we need to ensure that (if the
         * environment is stored in NAND) that it is in its own dedicated "block".
         * Hence, we need to define the erase size of the NAND device being used,
         * so that the environment section always starts on an erase-block boundary,
         * AND is a multiple of erase blocks.
         * In practice for a NAND device with an erase size of >= 256KiB,
         * a single dedicated block should always be sufficient!
         */
#       define CONFIG_SYS_NAND_ENV_ALIGN       (256 << 10)     /* NAND ENV offset should be multiple of 256KB
								to ensure compatibility with NAND chips of 
								different erase sizes */
        /*
         * Currently, there are 3 main modes to read/write from/to
         * NAND devices on STM SoCs:
         *      1) using the "EMI bit-banging" driver
         *         (can NOT be used with boot-from-NAND)
         *      2) using the H/W Hamming controller (flex-mode) driver
         *         (also supports boot-from-NAND capability)
         *      3) using the H/W BCH controller (multi-bit ECC) driver
         *         (also supports boot-from-NAND capability)
         * Either CONFIG_SYS_STM_NAND_USE_BIT_BANGING, or CONFIG_SYS_STM_NAND_USE_HAMMING,
         * or CONFIG_SYS_STM_NAND_USE_BCH should be defined, to select a single NAND driver.
         * However, the SoCs on the B2089 board uses the "FlashSS" IP block, which does
         * *NOT* support the "EMI bit-banging" driver, hence Hamming or BCH must be used.
         * If we are using FLEX or BCH, we still need to #define the
         * address CONFIG_SYS_EMI_NAND_BASE, although the value is ignored!
         */
#if 0
#       define CONFIG_SYS_STM_NAND_USE_HAMMING                  /* use H/W Hamming ("flex") driver */
#endif
#       define CONFIG_SYS_STM_NAND_USE_BCH                      /* use H/W BCH ("multi-bit") driver */
#       define CONFIG_SYS_EMI_NAND_BASE         0xDEADBEEF      /* for FLEX/BCH, we do not care! */
        
        /*
         * Do we want to read/write NAND Flash compatible with the ST40's
         * NAND Hamming H/W IP block for "boot-mode"? If we want
         * to read/write NAND flash that is meant to support booting
         * from NAND, then we need to use 3 bytes of ECC per 128 byte
         * record.  If so, then define the "CONFIG_SYS_NAND_ECC_HW3_128" macro.
         * Note: do *not* define this if CONFIG_SYS_STM_NAND_USE_BCH is defined,
         * as the Hamming boot-mode ECC is different to that of the BCH.
         */
#       define CONFIG_SYS_NAND_ECC_HW3_128      /* define for "boot-from-NAND" compatibility */
#       if defined(CONFIG_SYS_STM_NAND_USE_BCH)
#       undef CONFIG_SYS_NAND_ECC_HW3_128       /* explicitly un-define if using BCH */
#       endif /* CONFIG_SYS_NAND_ECC_HW3_128 */

        /*
         * Do we want to use STMicroelectronics' proprietary AFM4 (4+3/512)
         * ECC format, instead of Linux's traditional S/W 3/256 ECC?
         * Note: This does *not* enable H/W AFM - you can use STM's
         * "FLEX-mode", it is simply the addition of the AFM4 ECC
         * algorithm+layout that is being supported here.
         * Note: We *can* use this H/W AFM4 (4+3/512) ECC in addition to
         * the H/W "boot-mode" (3+1/128) ECC, on the same NAND device,
         * to partition it, set CONFIG_SYS_NAND_STM_BOOT_MODE_BOUNDARY appropriately.
         */
#       undef CONFIG_SYS_NAND_ECC_AFM4          /* define for AFM4 (4+3/512) ECC compatibility */

        /*
         * If using CONFIG_SYS_NAND_ECC_HW3_128, then we must also define
         * where the (high watermark) boundary is. That is, the
         * NAND offset, below which we are in "boot-mode", and
         * must use 3 bytes of ECC for each 128 byte record.
         * For this offset (and above) we can use any supported
         * ECC configuration (e.g 3/256 S/W, or 3/512 H/W).
         */
#       define CONFIG_SYS_NAND_STM_BOOT_MODE_BOUNDARY (1ul << 20)       /* 1 MiB */

        /*
         * If we want to store the U-boot environment variables in
         * the NAND device, then we also need to specify *where* the
         * environment variables will be stored. Typically this
         * would be immediately after the U-boot monitor itself.
         * However, that *may* be a bad block. Define the following
         * to place the environment in an appropriate good block.
	 */
#       define CONFIG_SYS_NAND_ENV_OFFSET       roundup(CONFIG_SYS_MONITOR_LEN, CONFIG_SYS_NAND_ENV_ALIGN)

/*----------------------------------------------------------------------
 * JFFS2 + MTD Partition support
 */

#       define CONFIG_CMD_JFFS2                 /* enable JFFS2 support */
#       define CONFIG_CMD_UBI
#       define CONFIG_CMD_UBIFS
#       define CONFIG_LZO
#       define CONFIG_RBTREE
#       define CONFIG_MTD_PARTITIONS

#       define CONFIG_CMD_MTDPARTS              /* mtdparts command line support */
#       define CONFIG_MTD_DEVICE                /* needed for mtdparts commands */
#       define CONFIG_JFFS2_NAND                /* JFFS2 support on NAND Flash */
#       define MTDPARTS_DEFAULT "mtdparts=" MTDPARTS_NAND
#       define MTDIDS_DEFAULT   MTDIDS_NAND
#endif  /* CONFIG_CMD_NAND */
