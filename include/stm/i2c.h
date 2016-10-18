/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */
#ifndef __ASSEMBLY__
extern void stm_i2c_scl(const int val);
extern void stm_i2c_sda(const int val);
extern int stm_i2c_read(void);
#endif
#if defined(CONFIG_CMD_I2C)
        /* Note: I2C Bus #7 also "probes" devices when a suitable HDMI device is plugged in to CN6. */
#       define CONFIG_I2C_BUS           STM_I2C_BUS_NUM       /* Use I2C Bus associated with SSC #5 */
#       define CONFIG_SYS_I2C_SLAVE     0x7F    /* I2C slave address - Bogus: master-only in U-Boot */
#       define CONFIG_SOFT_I2C                  /* I2C with S/W bit-banging     */
#       undef  CONFIG_HARD_I2C                  /* I2C withOUT hardware support */

#       define CONFIG_SYS_I2C_SOFT
#       define CONFIG_SYS_I2C_SOFT_SLAVE        0x7F
#       define I2C_ACTIVE                       /* open-drain, nothing to do */
#       define I2C_TRISTATE                     /* open-drain, nothing to do */
#       define I2C_SCL(val)             do { stm_i2c_scl((val)); } while (0)
#       define I2C_SDA(val)             do { stm_i2c_sda((val)); } while (0)
#       define I2C_READ                 stm_i2c_read()

        /*
         * The "BOGOS" for NDELAY() may be calibrated using the
         * following code fragment, and measuring (using an oscilloscope)
         * the frequency of the I2C SCL pin, and adjusting
         * NDELAY_BOGOS, until the SCL is approximately 100 kHz.
         * (100kHz has a period of 5us + 5us).
         *
         *      printf("just toggling I2C SCL (100kHz frequency) ...\n");
         *      while (1)
         *      {
         *              I2C_SCL(1); NDELAY(5000);
         *              I2C_SCL(0); NDELAY(5000);
         *      }
         */
#       define NDELAY_BOGOS             20      /* Empirical measurement for 1ns */
#       define NDELAY(ns)                                               \
                do {                                                    \
                        const unsigned n_bogo = NDELAY_BOGOS;           \
                        const unsigned n_ticks =                        \
                                ((ns)<n_bogo) ? 1u : (ns)/n_bogo;       \
                        volatile unsigned n_count;                      \
                        for(n_count=0; n_count<n_ticks; n_count++)      \
                                ;       /* do nothing */                \
                } while(0)

        /*
         * Note there are 4 * I2C_DELAY per I2C clock cycle
         * So, 100kHz requires an I2C delay of 2,500 ns,
         * whereas, 400 kHz requires an I2C delay of 625 ns.
         * However, this calculation only works if the S/W
         * overhead in I2C bit-banging is negligible - which it is not!
         * So, in practice, either I2C_DELAY or CONFIG_SYS_I2C_SPEED will be lower.
         * The higher the clock frequency, the greater the difference.
         * Empirical measurement/adjustment is recommended.
         */
#        define CONFIG_SYS_I2C_SOFT_SPEED        100000                  /* I2C clock speed (100 kHz) */
#        define I2C_DELAY        do { NDELAY(625*4); } while (0) /* 2,500 ns */

#endif  /* CONFIG_CMD_I2C */

