/*
 *  Copyright (C) 2014 STMicroelectronics Limited
 *     Ram Dayal <ram.dayal@st.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 */

#include <common.h>
#include <command.h>
#include <stm/soc.h>
#include <stm/pio.h>
#include <asm/io.h>

extern void stm_configure_i2c(void)
{
    /*
     * The I2C buses are routed as follows:
     *
     *      Bus       SCL             SDA
     *      ---       ---             ---
     *       0      PIO10[5]        PIO10[6]        SSC #0
     *       1      PIO11[0]        PIO11[1]        SSC #1
     *       2      PIO15[5]        PIO15[6]        SSC #2
     *       3      PIO18[5]        PIO18[6]        SSC #3
     *       4      PIO30[0]        PIO30[1]        SSC #4
     *       5      PIO34[3]        PIO34[4]        SSC #5
     *       6      PIO4[5]         PIO4[6]         SBC_SSC #10
     *       7      PIO5[0]         PIO5[1]         SBC_SSC #11
     *       8      PIO3[7]         PIO3[6]         SBC_SSC #12
     */
    const int scl_port = ssc_pios[CONFIG_I2C_BUS].pio[0].port;
    const int scl_pin  = ssc_pios[CONFIG_I2C_BUS].pio[0].pin;
    const int sda_port = ssc_pios[CONFIG_I2C_BUS].pio[1].port;
    const int sda_pin  = ssc_pios[CONFIG_I2C_BUS].pio[1].pin;
    
    if (CONFIG_I2C_BUS >= ssc_pios_array_size) BUG();
    
    /* route PIO (explicitly alternate #0) */
    stm_pioalt_select(scl_port, scl_pin, 0);                    /* I2C_SCL */
    stm_pioalt_select(sda_port, sda_pin, 0);                    /* I2C_SDA */
    
    /* set up directionality appropriately */
    SET_PIO_PIN(STM_PIO_BASE(scl_port), scl_pin, STPIO_BIDIR);      /* I2C_SCL */
    SET_PIO_PIN(STM_PIO_BASE(sda_port), sda_pin, STPIO_BIDIR);      /* I2C_SDA */
}

extern void stm_i2c_scl(const int val)
{
    /* SSC's SCLK == I2C's SCL */
    const int port = ssc_pios[CONFIG_I2C_BUS].pio[0].port;
    const int pin  = ssc_pios[CONFIG_I2C_BUS].pio[0].pin;
    STPIO_SET_PIN(STM_PIO_BASE(port), pin, (val) ? 1 : 0);
}

extern void stm_i2c_sda(const int val)
{
    /* SSC's MTSR == I2C's SDA */
    const int port = ssc_pios[CONFIG_I2C_BUS].pio[1].port;
    const int pin  = ssc_pios[CONFIG_I2C_BUS].pio[1].pin;
    STPIO_SET_PIN(STM_PIO_BASE(port), pin, (val) ? 1 : 0);
}

extern int stm_i2c_read(void)
{
    /* SSC's MTSR == I2C's SDA */
    const int port = ssc_pios[CONFIG_I2C_BUS].pio[1].port;
    const int pin  = ssc_pios[CONFIG_I2C_BUS].pio[1].pin;
    return STPIO_GET_PIN(STM_PIO_BASE(port), pin);
}
#if defined(CONFIG_I2C_CMD_TREE)
extern unsigned int i2c_get_bus_speed(void)
{
            return CONFIG_SYS_I2C_SPEED;
}
extern int i2c_set_bus_speed(unsigned int speed)
{
            return -1;
}
#endif  /* CONFIG_I2C_CMD_TREE */

