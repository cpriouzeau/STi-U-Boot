/*
 * Copyright (C) STMicroelectronics Ltd. 2008,2010-2014
 *
 * All rights reserved.
 */

/*
 * This is derived from STMicroelectronics gnu toolchain example:
 *   sh-superh-elf/examples/bare/sh4reg/sti7105.h
 */


#ifndef __INCLUDE_STM_STXH410REG_H
#define __INCLUDE_STM_STXH410REG_H
#ifdef CONFIG_STM_STXH410
#define STM_SOC "STxH410"
#endif
#include <stm/regtype.h>
/*----------------------------------------------------------------------------*/
#define    SRC_PLL0  0
#define    SRC_PLL1  1
#define    SRC_FS0  2
#define    SRC_FS1  3
#define    SRC_FS2  4
#define    SRC_FS3  5
#define    SRC_OSC  6

#define CLKGEN_A0_BASE 0x90ff000
#define CLKGEN_C0_BASE 0x9103000
#define CLK_DIV_MMC0 10
#define CLK_DIV_MMC1 11

/*
 * Base addresses for control register banks.
 */

/* STxH410 control registers */
#ifndef STXH410_COMMS_BASE
#define STXH410_COMMS_BASE			0x09800000
#endif

/* FLASHSS Register Configuration Space */
#define CONFIG_STM_SOC_HAS_FLASHSS		/* The FlashSS is present on this SoC */

/* Flash Interface, Control Registers (port 0) */
#ifndef STXH410_FLASH_IF_REG0_BASE
#define STXH410_FLASH_IF_REG0_BASE		0x09020000
#endif
#ifndef CONFIG_SYS_STM_FLASHSS_PORT0_BASE
#define CONFIG_SYS_STM_FLASHSS_PORT0_BASE	STXH410_FLASH_IF_REG0_BASE
#endif

/* Flash Interface, Control Registers (port 1) */
#ifndef STXH410_FLASH_IF_REG1_BASE
#define STXH410_FLASH_IF_REG1_BASE		0x09060000
#endif
#ifndef CONFIG_SYS_STM_FLASHSS_PORT1_BASE
#define CONFIG_SYS_STM_FLASHSS_PORT1_BASE	STXH410_FLASH_IF_REG1_BASE
#endif

/* FSM SPI Controller Base (in the FlashSS) */
#ifndef CONFIG_SYS_STM_SPI_FSM_BASE
#define CONFIG_SYS_STM_SPI_FSM_BASE		(CONFIG_SYS_STM_FLASHSS_PORT0_BASE + 0x2000)
#endif

/* MMC Controller Bases (in the FlashSS) */
#ifndef CONFIG_SYS_MMC0_BASE
#define CONFIG_SYS_MMC0_BASE			(CONFIG_SYS_STM_FLASHSS_PORT1_BASE + 0x0)	/* MMC #0 */
#endif
#ifndef CONFIG_SYS_MMC1_BASE
#define CONFIG_SYS_MMC1_BASE			0x09080000	/* MMC #1 */
#endif


/* Low-Power Block */
#ifndef STXH410_SBC_LPM_BASE
#define STXH410_SBC_LPM_BASE			0x09400000
#endif

/* ARM Cortex-A9 Configuration Registers */
#ifndef STM_A9_CONFIG_BASE
#define STM_A9_CONFIG_BASE			0x08760000
#endif

/*----------------------------------------------------------------------------*/

/* Recommended STBus Bridge Values for GMAC */
#ifndef STM_GMAC_AHB2STBUS_BASE
#define STM_GMAC_AHB2STBUS_BASE			0x2000		/* offset */
#endif
#ifndef STM_GMAC_AHB2STBUS_CONFIG
#define STM_GMAC_AHB2STBUS_CONFIG		0x26c209	/* from validation */
#endif

/*----------------------------------------------------------------------------*/

/* ASC UARTs */
/* in the main "COMMs" block */
#ifndef STXH410_ASC0_BASE
#define STXH410_ASC0_BASE (STXH410_COMMS_BASE + 0x00030000)
#endif
#ifndef STXH410_ASC1_BASE
#define STXH410_ASC1_BASE (STXH410_COMMS_BASE + 0x00031000)
#endif
#ifndef STXH410_ASC2_BASE
#define STXH410_ASC2_BASE (STXH410_COMMS_BASE + 0x00032000)
#endif
#ifndef STXH410_ASC3_BASE
#define STXH410_ASC3_BASE (STXH410_COMMS_BASE + 0x00033000)
#endif
#ifndef STXH410_ASC4_BASE
#define STXH410_ASC4_BASE (STXH410_COMMS_BASE + 0x00034000)
#endif
/* in the "SBC" block */
#ifndef STXH410_SBC_ASC0_BASE
#define STXH410_SBC_ASC0_BASE (STXH410_SBC_LPM_BASE + 0x00130000)
#endif
#ifndef STXH410_SBC_ASC1_BASE
#define STXH410_SBC_ASC1_BASE (STXH410_SBC_LPM_BASE + 0x00131000)
#endif


/* SSCs */
/* in the main "COMMs" block */
#ifndef STXH410_SSC0_BASE
#define STXH410_SSC0_BASE (STXH410_COMMS_BASE + 0x00040000)
#endif
#ifndef STXH410_SSC1_BASE
#define STXH410_SSC1_BASE (STXH410_COMMS_BASE + 0x00041000)
#endif
#ifndef STXH410_SSC2_BASE
#define STXH410_SSC2_BASE (STXH410_COMMS_BASE + 0x00042000)
#endif
#ifndef STXH410_SSC3_BASE
#define STXH410_SSC3_BASE (STXH410_COMMS_BASE + 0x00043000)
#endif
#ifndef STXH410_SSC4_BASE
#define STXH410_SSC4_BASE (STXH410_COMMS_BASE + 0x00044000)
#endif
#ifndef STXH410_SSC5_BASE
#define STXH410_SSC5_BASE (STXH410_COMMS_BASE + 0x00045000)
#endif
#ifndef STXH410_SSC6_BASE
#define STXH410_SSC6_BASE (STXH410_COMMS_BASE + 0x00046000)
#endif
#ifndef STXH410_SSC7_BASE
#define STXH410_SSC7_BASE (STXH410_COMMS_BASE + 0x00047000)
#endif
/* in the "SBC" block */
#ifndef STXH410_SBC_SSC0_BASE
#define STXH410_SBC_SSC0_BASE (STXH410_SBC_LPM_BASE + 0x00140000)
#endif
#ifndef STXH410_SBC_SSC1_BASE
#define STXH410_SBC_SSC1_BASE (STXH410_SBC_LPM_BASE + 0x00141000)
#endif
#ifndef STXH410_SBC_SSC2_BASE
#define STXH410_SBC_SSC2_BASE (STXH410_SBC_LPM_BASE + 0x00142000)
#endif

/*----------------------------------------------------------------------------*/

/* PIOs */
#ifndef STXH410_PIO_SBC_BASE
#define STXH410_PIO_SBC_BASE		0x09610000 /* 0-5 */
#endif
#ifndef STXH410_PIO_FRONT0_BASE
#define STXH410_PIO_FRONT0_BASE		0x09200000 /* 10-19 */
#endif
#ifndef STXH410_PIO_FRONT1_BASE
#define STXH410_PIO_FRONT1_BASE		0x09210000 /* 20 */
#endif
#ifndef STXH410_PIO_REAR_BASE
#define STXH410_PIO_REAR_BASE		0x09220000 /* 30-35 */
#endif
#ifndef STXH410_PIO_FLASH_BASE
#define STXH410_PIO_FLASH_BASE		0x09230000 /* 40-42 */
#endif
#define STM_PIO_BASE(x)							\
	(								\
		((x) < 6)						\
		? (STXH410_PIO_SBC_BASE +(0x1000*((x)-0)))		\
		:							\
		((x) < 20)						\
		? (STXH410_PIO_FRONT0_BASE +(0x1000*((x)-10)))		\
		:							\
		((x) < 21)						\
		? (STXH410_PIO_FRONT1_BASE +(0x1000*((x)-20)))		\
		:							\
		((x) < 36)						\
		? (STXH410_PIO_REAR_BASE+(0x1000*((x)-30)))		\
		:							\
		(STXH410_PIO_FLASH_BASE+(0x1000*((x)-40)))		\
	)

/*----------------------------------------------------------------------------*/

/* Configuration/Status Registers */
#ifndef STXH410_SYSCONF0_BASE
#define STXH410_SYSCONF0_BASE		0x09620000 /* 0-999 */
#endif
#ifndef STXH410_SYSCONF1_BASE
#define STXH410_SYSCONF1_BASE		0x09280000 /* 1000-1999 */
#endif
#ifndef STXH410_SYSCONF2_BASE
#define STXH410_SYSCONF2_BASE		0x09290000 /* 2000-2999 */
#endif
#ifndef STXH410_SYSCONF3_BASE
#define STXH410_SYSCONF3_BASE		0x092a0000 /* 3000-3999 */
#endif
#ifndef STXH410_SYSCONF4_BASE
#define STXH410_SYSCONF4_BASE		0x09600000 /* 4000-4999 */
#endif
#ifndef STXH410_SYSCONF5_BASE
#define STXH410_SYSCONF5_BASE		0x092b0000 /* 5000-5999 */
#endif
#ifndef STXH410_SYSCONF6_BASE
#define STXH410_SYSCONF6_BASE		0x092c0000 /* 6000-6999 */
#endif

#ifndef STXH410_LPM_SYSCONF_BASE
#define STXH410_LPM_SYSCONF_BASE	0x094b5100 /* LPM config regs. */
#endif

/*
 * STxH410 System Configuration "accessors"
 */
#if defined(__ASSEMBLER__)
#define STXH410_SYSCFG(x)	STXH410_SYSCFG x
	.macro STXH410_SYSCFG x:req
		.if ((\x) < 1000)
			.long (STXH410_SYSCONF0_BASE + ((\x)-0)*0x4)
		.elseif ((\x) < 2000)
			.long (STXH410_SYSCONF1_BASE + ((\x)-1000)*0x4)
		.elseif ((\x) < 3000)
			.long (STXH410_SYSCONF2_BASE + ((\x)-2000)*0x4)
		.elseif ((\x) < 4000)
			.long (STXH410_SYSCONF3_BASE + ((\x)-3000)*0x4)
		.elseif ((\x) < 5000)
			.long (STXH410_SYSCONF4_BASE + ((\x)-4000)*0x4)
		.elseif ((\x) < 6000)
			.long (STXH410_SYSCONF5_BASE + ((\x)-5000)*0x4)
		.else
			.long (STXH410_SYSCONF6_BASE + ((\x)-6000)*0x4)
		.endif
	.endm
#else	/* __ASSEMBLER__ */
#define STXH410_SYSCFG(x)							\
	(									\
		((x) < 1000)							\
		? STM_U32_REG(STXH410_SYSCONF0_BASE + ((x)-0)*0x4)		\
		:								\
		((x) < 2000)							\
		? STM_U32_REG(STXH410_SYSCONF1_BASE + ((x)-1000)*0x4)		\
		:								\
		((x) < 3000)							\
		? STM_U32_REG(STXH410_SYSCONF2_BASE + ((x)-2000)*0x4)		\
		:								\
		((x) < 4000)							\
		? STM_U32_REG(STXH410_SYSCONF3_BASE + ((x)-3000)*0x4)		\
		:								\
		((x) < 5000)							\
		? STM_U32_REG(STXH410_SYSCONF4_BASE + ((x)-4000)*0x4)		\
		:								\
		((x) < 6000)							\
		? STM_U32_REG(STXH410_SYSCONF5_BASE + ((x)-5000)*0x4)		\
		:								\
		STM_U32_REG(STXH410_SYSCONF6_BASE + ((x)-6000)*0x4)		\
	)
#endif	/* __ASSEMBLER__ */

#if defined(__ASSEMBLER__)
#define STXH410_LPM_SYSCFG(x)	STXH410_LPM_SYSCFG x
	.macro STXH410_LPM_SYSCFG x:req
		.long (STXH410_LPM_SYSCONF_BASE + ((\x)-0)*0x4)
	.endm
#else	/* __ASSEMBLER__ */
#define STXH410_LPM_SYSCFG(x)							\
	(									\
		STM_U32_REG(STXH410_LPM_SYSCONF_BASE + ((x)-0)*0x4)		\
	)
#endif	/* __ASSEMBLER__ */

/*
 * STxH410 System Status "accessors"
 */
#define STXH410_SYSSTS(x)	STXH410_SYSCFG(x)


/*----------------------------------------------------------------------------*/

/*
 * Does the FSM SPI Serial Flash Controller support 32-bit addresses on this SoC?
 */
#if !defined(CONFIG_STM_FSM_SUPPORTS_32_BIT_ADDRESSES)
#define CONFIG_STM_FSM_SUPPORTS_32_BIT_ADDRESSES	/* yes, it is supported */
#endif

/*
 * The STxH410 does not have a traditional "banked" EMI.
 */
#if !defined(CONFIG_CMD_BDI_DUMP_EMI_BANKS)
#define CONFIG_CMD_BDI_DUMP_EMI_BANKS		0	/* do not dump EMI banks */
#endif

/*----------------------------------------------------------------------------*/

/*
 * Pulse-Width Modulator in SBC.
 */
#ifndef STXH410_SBC_PWM_BASE
#define STXH410_SBC_PWM_BASE		(STXH410_SBC_LPM_BASE + 0x00110000)
#endif

#ifndef STXH410_SBC_PWM1_REG
#define STXH410_SBC_PWM1_REG		(STXH410_SBC_PWM_BASE + 0x04)
#endif

#define STXH410_PWM1_VOLTS(vcore)	((1135740-823*(vcore))/1000)	/* 1070=>255, 1380=>0 */

/*----------------------------------------------------------------------------*/

#include <stm/stxxxxxreg.h>

/*----------------------------------------------------------------------------*/


/*
 * Device ID register & bitfields
 */


/* Device ID values, masks & predicates */
#define STXH410_DEVID_410_VAL		0x056	/* Device ID for the STxH410 */
#define STXH410_DEVID_ID_SHIFT		12
#define STXH410_DEVID_ID_MASK		0x3ff
#define STXH410_DEVID_CUT_SHIFT		28
#define STXH410_DEVID_CUT_MASK		0xf

#define STXH410_DEVICEID_410(ID)	((((ID) >> STXH410_DEVID_ID_SHIFT) & STXH410_DEVID_ID_MASK) == STXH410_DEVID_410_VAL)
#define STXH410_DEVICEID_CUT(ID)	((((ID) >> STXH410_DEVID_CUT_SHIFT) & STXH410_DEVID_CUT_MASK) + 1)

#define CONFIG_STM_STMAC_BASE		0x09630000ul    /* GMAC #1 */
#define CONFIG_SYS_EMI_SPI_BASE		0x00000000      /* CSA: SPI Flash,  Physical 0x00000000 (4MiB) */

#define CONFIG_USB2_0_BASE                   0x9A00000
#define CONFIG_USB2_1_BASE                   0x9A80000

#define CONFIG_MMC_BOOT_MODE_1_BIT

/* --------------------------------------------------------------------
 *              Ethernet MAC resources (PAD and Retiming)
 * --------------------------------------------------------------------*/


#define SYSCONF(_reg)   ((unsigned long*)STXH410_SYSCFG(_reg))


#define DATA_IN(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_input, \
                .retime    = _retiming, \
        }

#define DATA_OUT(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_output, \
                .retime    = _retiming, \
        }

#define MDIO(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_output, \
                .retime    = _retiming, \
        }

#define MDC(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_output, \
                .retime    = _retiming, \
        }
/* On some boards MDIO line is missing Pull-up resistor, Enabling weak
 * internal PULL-UP overcomes the issue */
#define DATA_OUT_PU(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_input_with_pullup, \
                .retime    = _retiming, \
        }

#define CLOCK_IN(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_input, \
                .retime    = _retiming, \
        }

#define CLOCK_OUT(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_output, \
                .retime    = _retiming, \
        }

#define PHY_CLOCK(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .u.gmac.phy_clock = 1, \
                .direction = stm_pad_direction_unknown, \
                .retime    = _retiming, \
        }
#define MMC_IN(_port, _pin, _func) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_input, \
                .retime    = NULL, \
        }

#define MMC_OUT(_port, _pin, _func) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_output, \
                .retime    = NULL, \
        }

#define MMC_DATA_IN_PU(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_input_with_pullup, \
                .retime    = _retiming, \
        }

#define MMC_CLOCK_OUT(_port, _pin, _func, _retiming) \
        { \
                .pio       = { _port, _pin, _func, }, \
                .direction = stm_pad_direction_output, \
                .retime    = _retiming, \
        }

#define RET_NICLK2(_delay,_clk) (&(const struct stm_pio_control_retime_config){ \
                .retime = 0, \
                .clk = STM_RETIME_VALUE_CLK_ ## _clk, \
                .clknotdata = 1, \
                .double_edge = 0, \
                .invertclk = 0, \
                .delay = STM_RETIME_VALUE_DELAY_ ## _delay, \
                })

#endif /* __INCLUDE_STM_STXH410REG_H */
