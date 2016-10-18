/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#if defined(CONFIG_SPI_FLASH)                           /* SPI serial flash present ? */
#       define CONFIG_SPI                               /* enable the SPI driver */
#       define CONFIG_CMD_SF
#	define CONFIG_SPI_FLASH_STMICRO
#	define CONFIG_SPI_FLASH_MACRONIX
#	define CONFIG_SPI_FLASH_SPANSION

#if defined(CONFIG_STM_FSM_SPI_FLASH)
        /* Use H/W FSM SPI Controller (not H/W SSC, nor S/W "bit-banging") */
#       define CONFIG_STM_FSM_SPI                       /* Use the H/W FSM for SPI */
#       define CONFIG_SYS_STM_SPI_CLOCKDIV      CONFIG_STM_SPI_CLOCKDIV       /* set SPI_CLOCKDIV = 4 */
#else

 #      define SPI_SCL(val)     do { stm_spi_scl((val)); } while (0)
 #      define SPI_SDA(val)     do { stm_spi_sda((val)); } while (0)
 #      define SPI_READ         stm_spi_read()

#endif
#endif  /* CONFIG_SPI_FLASH */

