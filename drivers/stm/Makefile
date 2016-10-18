#
# (C) Copyright 2008,2010-2014 STMicroelectronics.
# Sean McGoogan <Sean.McGoogan@st.com>
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#


# All the STMicroelectronics-specific stuff, which may be ARCH-agnostic
ifdef CONFIG_STM
obj-y += board.o
obj-y += stm-sysconf.o
obj-$(CONFIG_STM_USB) += usb.o
obj-$(CONFIG_STM_FAMILY_STXH410) += stxh410_family/ common/usb.o
obj-$(CONFIG_USB_EHCI_STM) += stm-ehci-host.o
obj-$(CONFIG_USB_XHCI_STM) += stm-xhci-host.o
obj-$(CONFIG_CMD_STTESTS) += cmd_sttests.o st_mtests.o
obj-$(CONFIG_SPI) += common/spi.o
obj-$(CONFIG_STM_ASC_SERIAL) += common/uart.o
obj-$(CONFIG_DRIVER_NETSTMAC)     += stm-stmac.o
obj-$(CONFIG_DRIVER_NET_STM_GMAC) += stm-stmac.o common/eth.o
obj-$(CONFIG_STM_SDHCI) += stm-sdhci.o common/sdhci.o
obj-$(CONFIG_STM_IR) += stm_ir.o
obj-$(CONFIG_SYS_I2C) += common/i2c.o

ifdef CONFIG_CMD_NAND
obj-y += ecc.o stm-nand.o common/nand.o
obj-$(CONFIG_SYS_STM_NAND_USE_BCH) += stm-nand-bch.o
obj-$(CONFIG_SYS_STM_NAND_USE_HAMMING) += stm-nand-flex.o
obj-$(CONFIG_SYS_NAND_SKIP_BAD_BLOCKS_ON_RELOCATING) += cmd_copybbt.o
endif # CONFIG_CMD_NAND

endif # CONFIG_STM
