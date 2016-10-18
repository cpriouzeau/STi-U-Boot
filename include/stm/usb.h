/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#ifdef CONFIG_STM_USB
#       define CONFIG_CMD_USB
#       define CONFIG_CMD_FAT
#       define CONFIG_USB_STORAGE
#       define CONFIG_PARTITION_UUIDS
#       define CONFIG_EFI_PARTITION

        /*
         *
         * As u-boot can't support multiple type of controllers(EHCI and XHCI) simultaneously
         * here we need to define CONFIG_USE_XHCI or CONFIG_USE_EHCI depending on wheteher
         * we intend to support USB-3.0 port or USB-2.0 port in the resultant u-boot image.
         * As a default choice I am using USB-3.0 port i.e CONFIG_USE_XHCI and it can be replaced
         * with CONFIG_USE_EHCI if u-boot intends to use USB-2.0 port
         *
         */

#if defined(STM_USB2_SUPPORT)
#       if defined(CONFIG_USE_OHCI)
#		define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#               define CONFIG_USB_OHCI_NEW                              /* enable USB 1.x, via OHCI */
#               define CONFIG_SYS_USB_OHCI_CPU_INIT
#               define CONFIG_SYS_USB_OHCI_REGS_BASE            AHB2STBUS_OHCI_BASE
#               define CONFIG_SYS_USB_OHCI_SLOT_NAME            "ohci"
#               define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS       1
#       elif    defined(CONFIG_USE_EHCI)
#               define CONFIG_SYS_USB2_0_BASE                   CONFIG_USB2_0_BASE
#               define CONFIG_SYS_USB2_1_BASE                   CONFIG_USB2_1_BASE
#               define CONFIG_SYS_USB_BASE                      CONFIG_SYS_USB2_0_BASE
#               define CONFIG_USB_EHCI                          /* enable USB 2.0, via EHCI */
#               define CONFIG_USB_EHCI_STM                      /* use EHCI for STMicroelectronics */
#               define CONFIG_SYS_USB_EHCI_MAX_ROOT_PORTS       2
#       endif   /* use OHCI/EHCI */
#endif

#if defined (STM_USB3_SUPPORT)
#               define CONFIG_USB_XHCI                                  /* enable USB 3.0, via XHCI */
#               define CONFIG_USB_XHCI_STM                              /* use XHCI for STMicroelectronics */
#               define CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS       2
#endif

#endif  /* ifdef CONFIG_STM_USB */


