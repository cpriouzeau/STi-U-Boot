#
# (C) Copyright 2015 STMicroelectronics.
#
# Imran Khan <imran.khan@st.com>
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-$(CONFIG_STM_FSM_SPI_FLASH)	+= stm_spi_fsm.o

ifdef CONFIG_STM_FSM_SPI_FLASH
obj-$(CONFIG_SPI_FLASH_SPANSION)	+= spansion.o
obj-$(CONFIG_SPI_FLASH_STMICRO)		+= stmicro.o
obj-$(CONFIG_SPI_FLASH_MACRONIX)	+= macronix.o
endif

