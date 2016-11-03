/*
 * (C) Copyright 2009-2012 STMicroelectronics.
 *
 * Sean McGoogan <Sean.McGoogan@st.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#ifndef __INCLUDE_STM_SYSCONF_H
#define __INCLUDE_STM_SYSCONF_H

	/*
	 * if ('flag')
	 *	set bit 'bit' in variable 'reg'
	 * else
	 *	clear bit 'bit' in variable 'reg'
	 */
#define SET_SYSCONF_BIT(reg,flag,bit)			\
	do {						\
		if (flag)				\
		{	/* set bit 'bit' */		\
			reg |= (1ul<<(bit));		\
		}					\
		else					\
		{	/* clear bit 'bit' */		\
			reg &= ~(1ul<<(bit));		\
		}					\
	} while (0)


	/*
	 * if ('flag')
	 *	set bits 'lsb:msb' to 'yes' in variable 'reg'
	 * else
	 *	set bits 'lsb:msb' to 'no' in variable 'reg'
	 *
	 * Note: 'msb' must be >= 'lsb'.
	 */
#define SET_SYSCONF_BITS(reg,flag,lsb,msb,yes,no)	\
	do {						\
		const unsigned long _mask = 		\
			(1ul<<((msb)-(lsb)+1))-1ul;	\
		/* clear all bits in 'lsb':'msb' */	\
		reg &= ~(_mask<<(lsb));		\
		if (flag)				\
		{	/* set 'yes' in lsb:msb */	\
			reg |= ((yes)<<(lsb));	\
		}					\
		else					\
		{	/* set 'no' in lsb:msb */	\
			reg |= ((no)<<(lsb));		\
		}					\
	} while (0)


struct stm_pad_sysconf {
	volatile	unsigned long * const address;
	const		int lsb;
	const		int msb;
			unsigned long value;
};

#define STM_PAD_SYSCONF(_reg, _lsb, _msb, _value) \
	{ \
		.address = _reg, \
		.lsb     = _lsb, \
		.msb     = _msb, \
		.value   = _value, \
	}
#define STM_PAD_SYS_CFG_BANK(_bank, _reg, _lsb, _msb, _value) \
	{ \
		.address = SYSCONF(_bank,_reg), \
		.lsb     = _lsb, \
		.msb     = _msb, \
		.value   = _value, \
	}

extern void stm_configure_sysconfs(
	const struct stm_pad_sysconf * const sys_configs,
	const size_t num_sys);


#endif	/* __INCLUDE_STM_SYSCONF_H */


