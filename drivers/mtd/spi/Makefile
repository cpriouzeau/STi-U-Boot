#
# (C) Copyright 2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-$(CONFIG_DM_SPI_FLASH) += sf-uclass.o
obj-$(CONFIG_STM) += stm_fsm_spi/

ifdef CONFIG_SPL_BUILD
obj-$(CONFIG_SPL_SPI_LOAD)	+= spi_spl_load.o
obj-$(CONFIG_SPL_SPI_BOOT)	+= fsl_espi_spl.o
obj-$(CONFIG_SPL_SPI_SUNXI)	+= sunxi_spi_spl.o
endif

obj-$(CONFIG_SPI_FLASH) += sf_probe.o spi_flash.o sf_params.o sf.o
obj-$(CONFIG_SPI_FLASH_DATAFLASH) += sf_dataflash.o
obj-$(CONFIG_SPI_FLASH_MTD) += sf_mtd.o
obj-$(CONFIG_SPI_FLASH_SANDBOX) += sandbox.o
