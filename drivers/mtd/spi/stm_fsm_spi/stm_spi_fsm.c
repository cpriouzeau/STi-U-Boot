/*
 * Copyright (C) 2014-2015 STMicroelectronics.
 *
 * U-boot driver for FSM-SPI controller.
 * Author: Imran Khan (imran.khan@st.com)
 *
 * Based on FSM-SPI driver available in linux-3.10 kernel 
 * and written by: Angus Clark(angus.clark@st.com)
 *
 * SPDX-License-Identifier:	GPL-2.0+
 * 
 */

#include <common.h>
#include <malloc.h>
#include <asm/io.h>
#include <stm_spi_fsm.h>
#include <stm/soc.h>
#define FLASH_PROBE_FREQ	10		/* Probe freq. (MHz) */

/* Manufacturer functions */
extern int s25fl_clear_status_reg(struct stm_spi_fsm *fsm);
extern int n25q_clear_flags(struct stm_spi_fsm *fsm);

/*
 * NOTE: /!\ NO SUPPORT FOR ATMEL SPI IN FSM
 *           => USE BIT BANGING IF NECESSARY
 * Currently, Atmel spi are not supported by the FSM driver.
 * This might be possible (TBC) but it would at least require to 
 * the page dynamic, tune some commands and maybe change the FSM config
 */
static const struct {
	const uint8_t idcode;
	int (*probe) (struct stm_spi_fsm* fsm, uint8_t *idcode);
} flashes[] = {
	/* Keep it sorted by define name */
#ifdef CONFIG_SPI_FLASH_SPANSION
	{ 0x01, spi_flash_probe_spansion },
#endif
#ifdef CONFIG_SPI_FLASH_STMICRO /* Shared with Numonyx */
	{ 0x20, spi_flash_probe_stmicro  },
#endif
#ifdef CONFIG_SPI_FLASH_MACRONIX
	{ 0xc2, spi_flash_probe_macronix },
#endif
#ifdef CONFIG_SPI_FLASH_WINBOND
	{ 0xef, spi_flash_probe_winbond },
#endif
#ifdef CONFIG_SPI_FLASH_ATMEL
	{ 0x1f, spi_flash_probe_atmel },
#endif

};


static struct fsm_seqs_list default_seqs_list = {
	.dummy = {
		.data_size = TRANSFER_SIZE(0),
		.seq = {
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	}, 
	
	.read_jedec = {
		.data_size = TRANSFER_SIZE(8),
		.seq_opc[0] = (SEQ_OPC_PADS_1 |
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(FLASH_CMD_RDID)),
		.seq = {
			FSM_INST_CMD1,
			FSM_INST_DATA_READ,
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_READNOTWRITE |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	},
	
	.read_status_fifo = {
		.data_size = TRANSFER_SIZE(4),
		.seq_opc[0] = (SEQ_OPC_PADS_1 |
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(FLASH_CMD_RDSR)),
		.seq = {
			FSM_INST_CMD1,
			FSM_INST_DATA_READ,
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_READNOTWRITE |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	},
	
	.write_status = {
		.seq_opc[0] = (SEQ_OPC_PADS_1 | 
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(FLASH_CMD_WREN) | 
			       SEQ_OPC_CSDEASSERT),
		.seq_opc[1] = (SEQ_OPC_PADS_1 | 
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(FLASH_CMD_WRSR)),
		.seq = {
			FSM_INST_CMD1,
			FSM_INST_CMD2,
			FSM_INST_STA_WR1,
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_READNOTWRITE |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	},
	
	.wrvcr = {
		.seq_opc[0] = (SEQ_OPC_PADS_1 | 
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(FLASH_CMD_WREN) | 
			       SEQ_OPC_CSDEASSERT),
		.seq_opc[1] = (SEQ_OPC_PADS_1 | 
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(FLASH_CMD_WRVCR)),
		.seq = {
			FSM_INST_CMD1,
			FSM_INST_CMD2,
			FSM_INST_STA_WR1,
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_READNOTWRITE |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	},

	.erase_sector = {
		.seq_opc = {
			(SEQ_OPC_PADS_1 | 
			 SEQ_OPC_CYCLES(8) |
			 SEQ_OPC_OPCODE(FLASH_CMD_WREN) | 
			 SEQ_OPC_CSDEASSERT),

			(SEQ_OPC_PADS_1 | 
			 SEQ_OPC_CYCLES(8) |
			 SEQ_OPC_OPCODE(FLASH_CMD_SE)),
		},
		.seq = {
			FSM_INST_CMD1,
			FSM_INST_CMD2,
			FSM_INST_ADD1,
			FSM_INST_ADD2,
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_READNOTWRITE |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	},

	.erase_chip = {
		.seq_opc = {
			(SEQ_OPC_PADS_1 | 
			 SEQ_OPC_CYCLES(8) |
			 SEQ_OPC_OPCODE(FLASH_CMD_WREN) | 
			 SEQ_OPC_CSDEASSERT),

			(SEQ_OPC_PADS_1 | 
			 SEQ_OPC_CYCLES(8) |
			 SEQ_OPC_OPCODE(FLASH_CMD_CHIPERASE) | 
			 SEQ_OPC_CSDEASSERT),
		},
		.seq = {
			FSM_INST_CMD1,
			FSM_INST_CMD2,
			FSM_INST_WAIT,
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_ERASE |
			    SEQ_CFG_READNOTWRITE |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	},

	/* 
	 * Read & Write will be configured later
	 * depending on the spi and platform capability
	 */
};

static struct stm_spifsm_platform fsm_platform_data = {
	/* Init done to detect use on an unregistred fsm */
	.base = (unsigned int)(-1),
};

/* Default READ configurations, in order of preference */
static const struct seq_rw_config fsm_default_read_configs[] = {
	{FLASH_CAPS_READ_1_4_4, FLASH_CMD_READ_1_4_4,	0, 4, 4, 0x00, 2, 4},
	{FLASH_CAPS_READ_1_1_4, FLASH_CMD_READ_1_1_4,	0, 1, 4, 0x00, 4, 0},
	{FLASH_CAPS_READ_1_2_2, FLASH_CMD_READ_1_2_2,	0, 2, 2, 0x00, 4, 0},
	{FLASH_CAPS_READ_1_1_2, FLASH_CMD_READ_1_1_2,	0, 1, 2, 0x00, 0, 8},
	{FLASH_CAPS_READ_FAST,	FLASH_CMD_READ_FAST,	0, 1, 1, 0x00, 0, 8},
	{FLASH_CAPS_READ_WRITE, FLASH_CMD_READ,	        0, 1, 1, 0x00, 0, 0},

	/* terminating entry */
	{0x00,			 0,			0, 0, 0, 0x00, 0, 0},
};

/* Default WRITE configurations, in order of preference */
static const struct seq_rw_config fsm_default_write_configs[] = {
	{FLASH_CAPS_WRITE_1_4_4, FLASH_CMD_WRITE_1_4_4, 1, 4, 4, 0x00, 0, 0},
	{FLASH_CAPS_WRITE_1_1_4, FLASH_CMD_WRITE_1_1_4, 1, 1, 4, 0x00, 0, 0},
	{FLASH_CAPS_WRITE_1_2_2, FLASH_CMD_WRITE_1_2_2, 1, 2, 2, 0x00, 0, 0},
	{FLASH_CAPS_WRITE_1_1_2, FLASH_CMD_WRITE_1_1_2, 1, 1, 2, 0x00, 0, 0},
	{FLASH_CAPS_READ_WRITE,  FLASH_CMD_WRITE,       1, 1, 1, 0x00, 0, 0},

	/* terminating entry */
	{0x00,			  0,			 0, 0, 0, 0x00, 0, 0},
};

const struct seq_rw_config* fsm_get_default_read_configs (void)
{
	return (const struct seq_rw_config*) fsm_default_read_configs;
}

const struct seq_rw_config* fsm_get_default_write_configs (void)
{
	return (const struct seq_rw_config*) fsm_default_write_configs;
}

/* Configure 'addr_cfg' according to addressing mode */
extern int configure_erasesec_seq(struct fsm_seq *seq, int use_32bit_addr)
{
	int addr1_cycles = use_32bit_addr ? 16 : 8;

	seq->addr_cfg = (ADR_CFG_CYCLES_ADD1(addr1_cycles) |
			 ADR_CFG_PADS_1_ADD1 |
			 ADR_CFG_CYCLES_ADD2(16) |
			 ADR_CFG_PADS_1_ADD2 |
			 ADR_CFG_CSDEASSERT_ADD2);
	return 0;
}


extern void fsm_enter_quad_mode(struct stm_spi_fsm *fsm, int enter)
{
        uint8_t sta;

        fsm_read_status(fsm, FLASH_CMD_RDSR, &sta, 1);
        sta = (enter ? sta | MX25_STATUS_QE : sta & ~MX25_STATUS_QE);
        fsm_write_status(fsm, FLASH_CMD_WRSR, sta, 1, 1);
}

/*
 * FSM Private API
 * Right now these API are only used internally in this file don't need to be exported
 *
 */
extern int fsm_is_idle(struct stm_spi_fsm *fsm)
{
	return readl(fsm->plat->base + SPI_FAST_SEQ_STA) & 0x10;
}

extern unsigned int fsm_fifo_available(struct stm_spi_fsm *fsm)
{
	return (readl(fsm->plat->base + SPI_FAST_SEQ_STA) >> 5) & 0x7f;
}

extern void fsm_load_seq(struct stm_spi_fsm *fsm,
				const struct fsm_seq *const seq)
{
	int words = FSM_SEQ_SIZE/sizeof(uint32_t);
	const uint32_t *src = (const uint32_t *)seq;
	void *dst = (void*)(fsm->plat->base + SPI_FAST_SEQ_TRANSFER_SIZE);

	assert(fsm_is_idle(fsm));

	while (words--) {
		writel(*src, dst);
		src++;
		dst += 4;
	}
}

extern int fsm_wait_seq(struct stm_spi_fsm *fsm)
{
	u32 now = get_timer(0);
	int timeout = 0;

	while (!timeout) {
		if (get_timer(now) > FSM_MAX_WAIT_SEQ_MS)
			timeout = 1;

		if (fsm_is_idle(fsm))
			return 0;
	}

	debug("timeout on sequence completion\n");

	return 1;
}

extern int fsm_enter_32bitaddr(struct stm_spi_fsm *fsm, int enter)
{
        struct fsm_seq *seq = &(fsm->seqs->en32bitaddr);
        uint32_t cmd = enter ? FLASH_CMD_EN4B_ADDR : FLASH_CMD_EX4B_ADDR;

        seq->seq_opc[0] = (SEQ_OPC_PADS_1 |
                           SEQ_OPC_CYCLES(8) |
                           SEQ_OPC_OPCODE(cmd) |
                           SEQ_OPC_CSDEASSERT);

        fsm_load_seq(fsm, seq);

        fsm_wait_seq(fsm);

        return 0;
}

/* Dummy sequence to read one byte of data from flash into the FIFO */
static const struct fsm_seq fsm_seq_load_fifo_byte = {
	.data_size = TRANSFER_SIZE(1),
	.seq_opc[0] = (SEQ_OPC_PADS_1 |
		       SEQ_OPC_CYCLES(8) |
		       SEQ_OPC_OPCODE(FLASH_CMD_RDID)),
	.seq = {
		FSM_INST_CMD1,
		FSM_INST_DATA_READ,
		FSM_INST_STOP,
	},
	.seq_cfg = (SEQ_CFG_PADS_1 |
		    SEQ_CFG_READNOTWRITE |
		    SEQ_CFG_CSDEASSERT |
		    SEQ_CFG_STARTSEQ),
};

/*
 * Clear the data FIFO
 *
 * Typically, this is only required during driver initialisation, where no
 * assumptions can be made regarding the state of the FIFO.
 *
 * The process of clearing the FIFO is complicated by fact that while it is
 * possible for the FIFO to contain an arbitrary number of bytes [1], the
 * SPI_FAST_SEQ_STA register only reports the number of complete 32-bit words
 * present.  Furthermore, data can only be drained from the FIFO by reading
 * complete 32-bit words.
 *
 * With this in mind, a two stage process is used to the clear the FIFO:
 *
 *     1. Read any complete 32-bit words from the FIFO, as reported by the
 *        SPI_FAST_SEQ_STA register.
 *
 *     2. Mop up any remaining bytes.  At this point, it is not known if there
 *        are 0, 1, 2, or 3 bytes in the FIFO.  To handle all cases, a dummy FSM
 *        sequence is used to load one byte at a time, until a complete 32-bit
 *        word is formed; at most, 4 bytes will need to be loaded.
 *
 * [1] It is theoretically possible for the FIFO to contain an arbitrary number
 *     of bits.  However, since there are no known use-cases that leave
 *     incomplete bytes in the FIFO, only words and bytes are considered here.
 */
extern int fsm_clear_fifo(struct stm_spi_fsm *fsm)
{
	const struct fsm_seq *seq = &fsm_seq_load_fifo_byte;
	uint32_t words;
	int i;

	/* 1. Clear any 32-bit words */
	words = fsm_fifo_available(fsm);
	if (words) {
		for (i = 0; i < words; i++)
			readl(fsm->plat->base + SPI_FAST_SEQ_DATA_REG);
		debug("cleared %d words from FIFO\n", words);
	}

	/* 2. Clear any remaining bytes
	 *    - Load the FIFO, one byte at a time, until a complete 32-bit word
	 *      is available.
	 */
	for (i = 0, words = 0; i < 4 && !words; i++) {
		fsm_load_seq(fsm, seq);
		fsm_wait_seq(fsm);
		words = fsm_fifo_available(fsm);
	}

	/*    - A single word must be available now */
	if (words != 1) {
		debug("failed to clear bytes from the data FIFO\n");
		return 1;
	}

	/*    - Read the 32-bit word */
	readl(fsm->plat->base + SPI_FAST_SEQ_DATA_REG);

	debug("cleared %d byte(s) from the data FIFO\n", 4 - i);

	return 0;
}

extern int fsm_read_fifo(struct stm_spi_fsm *fsm,
			 uint32_t *buf, 
			 const uint32_t size)
{
	unsigned int avail;
	unsigned int remaining = size >> 2;
	unsigned int words;
	unsigned int word;

/*	debug("FSM: reading %d bytes from FIFO\n", size); */

	assert(((((unsigned int)buf) & 0x3) || (size & 0x3)) == 0);

	while (remaining) {
		while (!(avail = fsm_fifo_available(fsm)))
			;
		words = min(avail, remaining);
		remaining -= words;

		while(words--) {
			word = readl(fsm->plat->base + SPI_FAST_SEQ_DATA_REG);
			*buf++ = word;
		}
	}

	return size;
}

extern int fsm_write_fifo(struct stm_spi_fsm *fsm,
			  const uint32_t *buf, 
			  const uint32_t size)
{
	uint32_t words = size >> 2;

	debug("FSM: writing %d bytes to FIFO\n", size);

	assert(((((uint32_t)buf) & 0x3) || (size & 0x3)) == 0);

	while(words--) {
		writel(*buf ,fsm->plat->base + SPI_FAST_SEQ_DATA_REG);
		buf++;
	}

	return size;
}

/*
 * Serial Flash operations
 */
extern uint8_t fsm_wait_busy(struct stm_spi_fsm *fsm)
{
	struct fsm_seq *seq = &(fsm->seqs->read_status_fifo);
	uint32_t status;
	int timeout = 0;
	unsigned long now;
	unsigned int max_time_ms = FLASH_MAX_STA_WRITE_MS; 

	/* Use RDRS1 */
	seq->seq_opc[0] = (SEQ_OPC_PADS_1 |
			   SEQ_OPC_CYCLES(8) |
			   SEQ_OPC_OPCODE(FLASH_CMD_RDSR));

	/* Load read_status sequence */
	fsm_load_seq(fsm, seq);

	/*
	 * Repeat until busy bit is deasserted, or timeout, or error (S25FLxxxS)
	 */
	now = get_timer(0);
	while (!timeout) {

		if (get_timer(now) > max_time_ms)
			timeout = 1;

		fsm_wait_seq(fsm);

		fsm_read_fifo(fsm, &status, 4);

		if ((status & FLASH_STATUS_BUSY) == 0)
			return 0;

		if ((fsm->plat->configuration & CFG_S25FL_CHECK_ERROR_FLAGS) &&
		    ((status & S25FL_STATUS_P_ERR) ||
		     (status & S25FL_STATUS_E_ERR)))
			return (uint8_t)(status & 0xff);

		if (!timeout)
			/* Restart */
			writel(seq->seq_cfg, fsm->plat->base + SPI_FAST_SEQ_CFG);
	}

	debug("timeout on wait_busy\n");

	return FLASH_STATUS_TIMEOUT;
}

extern int fsm_read_jedec(struct stm_spi_fsm *fsm, 
			  uint8_t *const jedec)
{
	const struct fsm_seq *seq = &(fsm->seqs->read_jedec);
	uint32_t tmp[2];

	fsm_load_seq(fsm, seq);

	/* Read 8 bytes for alignement */
	fsm_read_fifo(fsm, tmp, sizeof(tmp));

	memcpy(jedec, tmp, 5);

	fsm_wait_seq(fsm);

	fsm_wait_busy(fsm);

	return 0;
}

extern int fsm_erase_sector(struct stm_spi_fsm *fsm, const uint32_t offset)
{
	struct fsm_seq *seq = &(fsm->seqs->erase_sector);
	uint8_t sta;
	int ret = 0;

	debug("FSM: erasing sector at 0x%08x\n", offset);

	/* Enter 32-bit address mode, if required */
	if (fsm->plat->configuration & CFG_ERASESEC_TOGGLE32BITADDR)
		fsm_enter_32bitaddr(fsm, 1);

	seq->addr1 = (offset >> 16) & 0xffff;
	seq->addr2 = offset & 0xffff;

	fsm_load_seq(fsm, seq);

	fsm_wait_seq(fsm);

	/* Wait for completion */
	sta = fsm_wait_busy(fsm);
	if (sta != 0) {
		ret = 1;
		if (fsm->plat->configuration & CFG_S25FL_CHECK_ERROR_FLAGS)
			s25fl_clear_status_reg(fsm);
	}

	/* N25Q: Check/Clear Error Flags */
	if (fsm->plat->configuration & CFG_N25Q_CHECK_ERROR_FLAGS) {
		fsm_read_status(fsm, N25Q_CMD_RFSR, &sta, 1);
		if (sta & N25Q_FLAGS_ERROR) {
			ret = 1;
			n25q_clear_flags(fsm);
		}
	}


	/* Exit 32-bit address mode, if required */
	if (fsm->plat->configuration & CFG_ERASESEC_TOGGLE32BITADDR)
		fsm_enter_32bitaddr(fsm, 0);

	return ret;
}

extern int fsm_erase_chip(struct stm_spi_fsm *fsm)
{
	const struct fsm_seq *seq = &(fsm->seqs->erase_chip);

	debug("FSM erasing chip\n");

	fsm_load_seq(fsm, seq);

	fsm_wait_seq(fsm);

	fsm_wait_busy(fsm);

	return 0;
}

extern int fsm_read_status(struct stm_spi_fsm *fsm, uint8_t cmd,
			   uint8_t *data, int bytes)
{
	struct fsm_seq *seq = &(fsm->seqs->read_status_fifo);
	uint32_t tmp;
	uint8_t *t = (uint8_t *)&tmp;
	int i;

	debug("FSM: reading STA[%s]\n",
	      (cmd == FLASH_CMD_RDSR) ? "1" : "2");

	BUG_ON(bytes != 1 && bytes != 2);

	seq->seq_opc[0] = (SEQ_OPC_PADS_1 | SEQ_OPC_CYCLES(8) |
			   SEQ_OPC_OPCODE(cmd)),

	fsm_load_seq(fsm, seq);

	fsm_read_fifo(fsm, &tmp, 4);

	for (i = 0; i < bytes; i++)
		data[i] = t[i];

	fsm_wait_seq(fsm);

	return 0;
}


extern int fsm_write_status(struct stm_spi_fsm *fsm, uint8_t cmd,
			    uint16_t data, int bytes, int wait_busy)
{
	struct fsm_seq *seq = &(fsm->seqs->write_status);

	debug("write 'status' register [0x%02x], %d byte(s), 0x%04x, %s wait-busy\n",
                cmd, bytes, data, wait_busy ? "with" : "no");

	BUG_ON(bytes != 1 && bytes != 2);

	/*
	 * S25FLxxxX: auto-clearing QUAD bit issue
	 *
	 * A Write Status operation is achieved by a 1-byte WRSR command.
	 * However, this seems to have the effect of clearing the QUAD bit in
	 * the Config register (other bits in the Config register are not
	 * affected).  As a workaround, we first read the Config register,
	 * combine with the new Status data, and then promote to a 2-byte WRSR
	 * command.
	 *
	 * W25QxxxX: auto-clearing QUAD bit and CMP bit issue
	 *
	 * Same workaround as S25FLxxxX
	 */
	if (cmd == FLASH_CMD_WRSR &&
	    bytes == 1 &&
	    (fsm->plat->configuration & CFG_WRSR_FORCE_16BITS)) {
		uint8_t cr;

		fsm_read_status(fsm, FLASH_CMD_RDSR2, &cr, 1);

		data = (data & 0xff) | ((uint16_t)cr << 8);
		bytes = 2;
	}

	seq->seq_opc[1] = (SEQ_OPC_PADS_1 | SEQ_OPC_CYCLES(8) |
			   SEQ_OPC_OPCODE(cmd));

	seq->status = (uint32_t)data | STA_PADS_1 | STA_CSDEASSERT;
	seq->seq[2] = (bytes == 1) ? FSM_INST_STA_WR1 : FSM_INST_STA_WR1_2;

	fsm_load_seq(fsm, seq);

	fsm_wait_seq(fsm);

	if (wait_busy)
		fsm_wait_busy(fsm);

	return 0;
}

extern int fsm_wrvcr(struct stm_spi_fsm *fsm, uint8_t data)
{
	struct fsm_seq *seq = &(fsm->seqs->wrvcr);

	debug("FSM: writing VCR 0x%02x\n", data);

	seq->status = (STA_DATA_BYTE1(data) |
		       STA_PADS_1 | STA_CSDEASSERT);

	fsm_load_seq(fsm, seq);

	fsm_wait_seq(fsm);

	return 0;
}

/*
 * FSM Configuration
 */
extern int fsm_set_mode(struct stm_spi_fsm *fsm, uint32_t mode)
{
	debug("FSM: mode 0x%x\n", mode);

	/* Wait for controller to accept mode change */
	if (!fsm->plat->capabilities.no_poll_mode_change) {
		while (!(readl(fsm->plat->base + SPI_STA_MODE_CHANGE) & 0x1))
			;
	}
	writel(mode, fsm->plat->base + SPI_MODESELECT);

	return 0;
}

extern int fsm_set_freq(struct stm_spi_fsm *fsm, uint32_t freq)
{
	uint32_t emi_freq;
	uint32_t clk_div;

	/* Timings set in terms of EMI clock... */
#ifdef CONFIG_SYS_EMI_FREQ
	emi_freq =  CONFIG_SYS_EMI_FREQ;
#else
	emi_freq =  100 * 1000000;
#endif
	freq *= 1000000;

	/* Calculate clk_div: multiple of 2, round up, 2 -> 128. Note, clk_div =
	 * 4 is not supported on some SoCs, use 6 instead */
	clk_div = 2*((emi_freq + (2*freq - 1))/(2*freq));
	if (clk_div < 2)
		clk_div = 2;
	else if (clk_div == 4 && fsm->plat->capabilities.no_clk_div_4)
		clk_div = 6;
	else if (clk_div > 128)
		clk_div = 128;
	
	if (clk_div <= 4)
		fsm->plat->fifo_dir_delay = 0;
	else if (clk_div <= 10)
		fsm->plat->fifo_dir_delay = 1;
	else
		fsm->plat->fifo_dir_delay = DIV_ROUND_UP(clk_div, 10);

	debug("FSM: emi_clk = %uHz, spi_freq = %uHZ, clock_div = %u, fsm_clk = %uHz\n",
	      emi_freq, freq, clk_div, emi_freq/clk_div);

	/* Set SPI_CLOCKDIV */
	writel(clk_div, fsm->plat->base + SPI_CLOCKDIV);

	return 0;
}

extern int fsm_read(struct stm_spi_fsm *fsm, 
		    uint8_t *const buf,
		    const uint32_t size, 
		    const uint32_t offset)
{
	struct fsm_seq *seq = &(fsm->seqs->read);
	uint32_t data_pads;
	uint32_t read_mask;
	size_t size_ub;
	size_t size_lb;
	size_t size_mop;
	uint32_t tmp[4];
	uint8_t *p;

	debug("FSM: reading %d bytes from 0x%08x\n", size, offset);

	/* Enter 32-bit address mode, if required */
	if (fsm->plat->configuration & CFG_READ_TOGGLE32BITADDR)
		fsm_enter_32bitaddr(fsm, 1);

	/* Must read in multiples of 32 cycles (or 32*pads/8 bytes) */
	data_pads = get_data_pads(seq);
	read_mask = (data_pads << 2) - 1;

	/* Handle non-aligned buf */
	if ((uint32_t)buf & 0x3) {
		p = memalign(size, 4);
	} else {
		p = buf;
	}

	/* Handle non-aligned size */
	size_ub = (size + read_mask) & ~read_mask;
	size_lb = size & ~read_mask;
	size_mop = size & read_mask;

	seq->data_size = TRANSFER_SIZE(size_ub);
	seq->addr1 = (offset >> 16) & 0xffff;
	seq->addr2 = offset & 0xffff;

	fsm_load_seq(fsm, seq);

	if (size_lb)
		fsm_read_fifo(fsm, (uint32_t *)p, size_lb);

	if (size_mop) {
		fsm_read_fifo(fsm, tmp, read_mask + 1);
		memcpy(p + size_lb, &tmp, size_mop);
	}

	/* Handle non-aligned buf */
	if ((uint32_t)buf & 0x3) {
		memcpy(buf, p, size);
		free(p);
	}

	/* Wait for sequence to finish */
	fsm_wait_seq(fsm);

	fsm_clear_fifo(fsm);

	/* Exit 32-bit address mode, if required */
	if (fsm->plat->configuration & CFG_READ_TOGGLE32BITADDR)
		fsm_enter_32bitaddr(fsm, 0);

	return 0;
}

extern int fsm_write(struct stm_spi_fsm *fsm, const uint8_t *const buf,
		     const size_t size, const uint32_t offset)
{
	struct fsm_seq *seq = &(fsm->seqs->write);
	uint32_t data_pads;
	uint32_t write_mask;
	size_t size_ub;
	size_t size_lb;
	size_t size_mop;
	uint32_t tmp[4];
	uint8_t *t = (uint8_t *)&tmp;
	int i;
	uint8_t *p;
	int ret = 0;
	uint8_t sta;

	debug("FSM: writing %d bytes to 0x%08x\n", size, offset);

	/* Enter 32-bit address mode, if required */
	if (fsm->plat->configuration & CFG_WRITE_TOGGLE32BITADDR)
		fsm_enter_32bitaddr(fsm, 1);

	/* Must write in multiples of 32 cycles (or 32*pads/8 bytes) */
	data_pads = get_data_pads(seq);
	write_mask = (data_pads << 2) - 1;

	/* Handle non-aligned buf */
	if ((uint32_t)buf & 0x3) {
		p = memalign(size, 4);
		memcpy(p, buf, size);
	} else {
		p = (uint8_t*) buf;
	}

	/* Handle non-aligned size */
	size_ub = (size + write_mask) & ~write_mask;
	size_lb = size & ~write_mask;
	size_mop = size & write_mask;

	seq->data_size = TRANSFER_SIZE(size_ub);
	seq->addr1 = (offset >> 16) & 0xffff;
	seq->addr2 = offset & 0xffff;

	if (fsm->plat->capabilities.dummy_on_write) {
		fsm_load_seq(fsm, &(fsm->seqs->dummy));
		readl(fsm->plat->base + SPI_FAST_SEQ_CFG);
	}

	/* Need to set FIFO to write mode, before writing data to FIFO (see
	 * GNBvb79594)
	 * Ensure at least SPI_FAST_SEQ_CFG[0,5,7] are all zero 
	 */
	writel(0x00040000, fsm->plat->base + SPI_FAST_SEQ_CFG);
	
	/* Ensure that the write is complete before proceding */
	if (fsm->plat->fifo_dir_delay == 0)
		readl(fsm->plat->base + SPI_FAST_SEQ_CFG);
	else
		udelay(fsm->plat->fifo_dir_delay);

	/* Write data to FIFO, before starting sequence (see GNBvd79593) */
	if (size_lb) {
		fsm_write_fifo(fsm, (uint32_t *)p, size_lb);
		p += size_lb;
	}

	/* Handle non-aligned size */
	if (size_mop) {
		memset(t, 0xff, write_mask + 1);	/* fill with 0xff's */
		for (i = 0; i < size_mop; i++)
			t[i] = *p++;

		fsm_write_fifo(fsm, tmp, write_mask + 1);
	}

	/* Start sequence */
	fsm_load_seq(fsm, seq);

	/* Wait for sequence to finish */
	fsm_wait_seq(fsm);

	/* Wait for completion */
	sta = fsm_wait_busy(fsm);
	if (sta != 0) {
		ret = 1;
		if (fsm->plat->configuration & CFG_S25FL_CHECK_ERROR_FLAGS)
			s25fl_clear_status_reg(fsm);
	}
	
	/* Handle non-aligned buf */
	if ((uint32_t)buf & 0x3)
		free(p);

	/* N25Q: Check/Clear Error Flags */
	if (fsm->plat->configuration & CFG_N25Q_CHECK_ERROR_FLAGS) {
		fsm_read_status(fsm, N25Q_CMD_RFSR, &sta, 1);
		if (sta & N25Q_FLAGS_ERROR) {
			ret = 1;
			n25q_clear_flags(fsm);
		}
	}

	/* Exit 32-bit address mode, if required */
	if (fsm->plat->configuration & CFG_WRITE_TOGGLE32BITADDR)
		fsm_enter_32bitaddr(fsm, 0);

	return ret;
}

extern int fsm_reset(struct stm_spi_fsm *fsm)
{
	struct stm_spifsm_platform* plat = fsm->plat;	

	/* Perform a soft reset of the FSM controller */
	if (!fsm->plat->capabilities.no_sw_reset) {
		writel(SEQ_CFG_SWRESET, plat->base + SPI_FAST_SEQ_CFG);
		udelay(1);
		writel(0, fsm->plat->base + SPI_FAST_SEQ_CFG);
	}

	/* Set clock to 'safe' frequency for device probe */
	fsm_set_freq(fsm, FLASH_PROBE_FREQ);

	/* Switch to FSM */
	fsm_set_mode(fsm, SPI_MODESELECT_FSM);

	/* Set timing parameters (use reset values for now (awaiting information
	 * from Validation Team)
	 */
	writel(SPI_CFG_DEVICE_ST |
	       SPI_CFG_MIN_CS_HIGH(0x0AA) |
	       SPI_CFG_CS_SETUPHOLD(0xa0) |
	       SPI_CFG_DATA_HOLD(0x00), plat->base + SPI_CONFIGDATA);
	writel(0x00000001, plat->base + SPI_STATUS_WR_TIME_REG);

	/*
	 * Set the FSM 'WAIT' delay to the minimum workable value.  Note, for
	 * our purposes, the WAIT instruction is used purely to achieve
	 * "sequence validity" rather than actually implement a delay.
	 */
	writel(0x000001, plat->base + SPI_PROGRAM_ERASE_TIME);

	/* Clear FIFO, just in case */
	fsm_clear_fifo(fsm);

	return 0;
}

/* Search for preferred configuration based on available capabilities */
static const struct seq_rw_config *search_seq_rw_configs(const struct seq_rw_config configs[], 
							 uint32_t capabilities)
{
	const struct seq_rw_config *config;

	for (config = configs; configs->cmd != 0; config++)
		if ((config->capabilities & capabilities) ==
		    config->capabilities)
			return config;

	return NULL;
}

/* Configure a READ/WRITE sequence according to configuration parameters */
static int configure_rw_seq(struct fsm_seq *seq, const struct seq_rw_config *cfg,
			    int use_32bit_addr)
{
	int i;
	int addr1_cycles, addr2_cycles;

	memset(seq, 0x00, FSM_SEQ_SIZE);

	i = 0;
	/* Add READ/WRITE OPC  */
	seq->seq_opc[i++] = (SEQ_OPC_PADS_1 |
			     SEQ_OPC_CYCLES(8) |
			     SEQ_OPC_OPCODE(cfg->cmd));

	/* Add WREN OPC for a WRITE sequence */
	if (cfg->write)
		seq->seq_opc[i++] = (SEQ_OPC_PADS_1 | SEQ_OPC_CYCLES(8) |
				     SEQ_OPC_OPCODE(FLASH_CMD_WREN) |
				     SEQ_OPC_CSDEASSERT);

	/* Address configuration (24 or 32-bit addresses) */
	addr1_cycles = use_32bit_addr ? 16 : 8;
	addr1_cycles /= cfg->addr_pads;
	addr2_cycles = 16 / cfg->addr_pads;
	seq->addr_cfg = ((addr1_cycles & 0x3f) << 0 |	/* ADD1 cycles */
			 (cfg->addr_pads - 1) << 6 |	/* ADD1 pads */
			 (addr2_cycles & 0x3f) << 16 |	/* ADD2 cycles */
			 ((cfg->addr_pads - 1) << 22));	/* ADD2 pads */

	/* Data/Sequence configuration */
	seq->seq_cfg = ((cfg->data_pads - 1) << 16 |
			SEQ_CFG_STARTSEQ |
			SEQ_CFG_CSDEASSERT);
	if (!cfg->write)
		seq->seq_cfg |= SEQ_CFG_READNOTWRITE;

	/* Mode configuration (no. of pads taken from addr cfg) */
	seq->mode = ((cfg->mode_data & 0xff) << 0 |	/* data */
		     (cfg->mode_cycles & 0x3f) << 16 |	/* cycles */
		     (cfg->addr_pads - 1) << 22);	/* pads */

	/* Dummy configuration (no. of pads taken from addr cfg) */
	seq->dummy = ((cfg->dummy_cycles & 0x3f) << 16 |	/* cycles */
		      (cfg->addr_pads - 1) << 22);		/* pads */


	/* Instruction sequence */
	i = 0;
	if (cfg->write)
		seq->seq[i++] = FSM_INST_CMD2;

	seq->seq[i++] = FSM_INST_CMD1;

	seq->seq[i++] = FSM_INST_ADD1;
	seq->seq[i++] = FSM_INST_ADD2;

	if (cfg->mode_cycles)
		seq->seq[i++] = FSM_INST_MODE;

	if (cfg->dummy_cycles)
		seq->seq[i++] = FSM_INST_DUMMY;

	seq->seq[i++] = cfg->write ? FSM_INST_DATA_WRITE : FSM_INST_DATA_READ;
	seq->seq[i++] = FSM_INST_STOP;

	return 0;
}

/* Configure preferred READ/WRITE sequence according to available
 * capabilities
 */
extern int fsm_search_configure_rw_seq(struct fsm_seq *seq,
				       const struct seq_rw_config *configs,
				       uint32_t capabilities)
{
	const struct seq_rw_config *config;

	config = search_seq_rw_configs(configs, capabilities);
	if (!config) {
		debug("FSM: failed to find suitable config\n");
		return 1;
	}

	if (configure_rw_seq(seq, config,
			     capabilities & FLASH_CAPS_32BITADDR) != 0) {
		debug("FSM: failed to configure READ/WRITE sequence\n");
		return 1;
	}

	return 0;
}

/* [DEFAULT] Configure READ/WRITE sequences */
extern int fsm_config_rw_seqs(struct stm_spi_fsm *fsm,
		       uint32_t spi_caps,
		       const struct seq_rw_config *r_confs,
		       const struct seq_rw_config *w_confs)
{
	if (fsm_search_configure_rw_seq(&(fsm->seqs->read),
					r_confs,
					spi_caps) != 0) {
		debug("FSM: failed to configure READ sequence "
			"according to capabilities [0x%08x]\n", spi_caps);
		return 1;
	}

	if (fsm_search_configure_rw_seq(&(fsm->seqs->write),
					w_confs,
					spi_caps) != 0) {
		debug("FSM: failed to configure WRITE sequence "
			"according to capabilities [0x%08x]\n", spi_caps);
		return 1;
	}

	/* Configure 'ERASE_SECTOR' sequence */
        configure_erasesec_seq(&(fsm->seqs->erase_sector),
                               spi_caps & FLASH_CAPS_32BITADDR);

	return 0;
}

/* [DEFAULT] Configure READ/WRITE sequences */
extern int fsm_config_rw_seqs_default(struct stm_spi_fsm *fsm,
			       uint32_t spi_caps)
{
	return fsm_config_rw_seqs(fsm, 
				  spi_caps,
				  fsm_default_read_configs,
				  fsm_default_write_configs);
}

/* Implement spi_flash.h interface */
struct spi_flash * spi_fsm_flash_probe( unsigned int bus, 
				    unsigned int cs,
				    unsigned int max_hz, 
				    unsigned int spi_mode)
{
	int i;
	struct stm_spi_fsm* fsm;
	uint8_t jedec[JEDEC_LEN];
	unsigned long sysconf = 0;

	if (fsm_platform_data.base == (uint32_t)(-1)) {
		printf("FSM: device on initialized\n");
		return NULL;
	}

	/* Currently, only one device is supported */
	if ((bus != 0) || (cs != 0)) {
		debug("FSM: no such device bus:%d, cs:%d\n",
		      bus, cs);
		return NULL;
	}

	fsm = malloc(sizeof(struct stm_spi_fsm));
	if (!fsm) {
		debug("FSM: Failed to allocate fsm\n");
		goto err_no_allocate_fsm;
	}

	fsm->plat = &fsm_platform_data;
	fsm->seqs = &default_seqs_list;

	fsm_reset(fsm);

	sysconf = readl(SYSCONF(SYSCONF_BOOT_MODE));
	if (((sysconf >> 4) & 0x7) == 0x7) {
		printf("EMMC is boot device. \n ");
		fsm->plat->capabilities.boot_from_spi = 0;
	} else {
		printf("SPI is boot device. \n ");
		fsm->plat->capabilities.boot_from_spi = 1;
	}
	fsm_read_jedec(fsm, jedec);

	/**/

	/* search the table for matches in shift and id */
	for (i = 0; i < ARRAY_SIZE(flashes); ++i)
		if (flashes[i].idcode == jedec[0]) {
			/* we have a match, call probe */
			if (flashes[i].probe(fsm, jedec) == 0);
				break;
		}
	
	if (i == ARRAY_SIZE(flashes)) {
		printf("FSM: Unsupported SPI flash:");
		for (i = 0; i < JEDEC_LEN; i++)
			printf(" %02x", jedec[i]);
		printf("\n");
		goto err_manufacturer_probe;
	}

	printf("FSM SF: Detected %s with sector size ", fsm->flash.name);
	print_size(fsm->flash.sector_size, ", total ");
	print_size(fsm->flash.size, "\n");      
	
	return &(fsm->flash);

err_manufacturer_probe:
	free(fsm);	
err_no_allocate_fsm:
	return NULL;
	
}

void spi_fsm_flash_free(struct spi_flash *flash)
{
	struct stm_spi_fsm* fsm;

	if(!flash)
		return;

	fsm = to_fsm(flash);
	if(fsm)
		free(fsm);		
}

extern int stm_spifsm_read(struct spi_flash *flash, 
		    u32 offset,
		    size_t len, 
		    void *buf)
{
	u32 bytes;
	u8 *b = (u8 *)buf;
	struct stm_spi_fsm* fsm = to_fsm(flash);
	
	debug("FSM: %s 0x%08x, len %zd\n",
	      "from", (u32)offset, len);

	if (!len)
		return 0;

	if (offset + len > flash->size)
		return -1;

	while (len > 0) {
		bytes = min(len, (size_t)flash->page_size);

		fsm_read(fsm, b, bytes, offset);

		b      += bytes;
		offset += bytes;
		len    -= bytes;
	}

	return 0;
}

extern int stm_spifsm_write(struct spi_flash *flash,
		     u32 offset,
		     size_t len, 
		     const void *buf)
{
	u32 page_offs;
	u32 bytes;
	u8 *b = (u8 *)buf;
	struct stm_spi_fsm* fsm = to_fsm(flash);

	debug("FSM: %s 0x%08x, len %zd\n",
	      "to", (u32)offset, len);
	
	if (!len)
		return 0;

	if (offset + len > flash->size)
		return -1;

	/* offset within page */
	page_offs = offset % flash->page_size;

	while (len) {

		/* write up to page boundary */
		bytes = min((size_t)(flash->page_size) - page_offs, len);

		fsm_write(fsm, b, bytes, offset);

		b      += bytes;
		offset += bytes;
		len    -= bytes;

		/* We are now page-aligned */
		page_offs = 0;
	}

	return 0;
}

extern int stm_spifsm_erase(struct spi_flash *flash,
		     u32 offset,
		     size_t len)
{
	struct stm_spi_fsm* fsm = to_fsm(flash);

	debug("FSM: %s 0x%08x, len 0x%08x\n",
	      "at", offset, len);

	if (offset + len > flash->size)
		return -1;

	if ((len % flash->sector_size)
	    || (offset % flash->sector_size))
		return -1;

	/* whole-chip erase? */
	if (len == flash->size && offset == 0) {
		if (fsm_erase_chip(fsm))
			return -1;
	} 
	else 
	{
		while (len) {
			if (fsm_erase_sector(fsm, offset))
				return -1;

			offset += flash->sector_size;
			len    -= flash->sector_size;
		}
	}

	return 0;
}

extern int stm_spifsm_register_device(struct stm_spifsm_platform* plat)
{	
	memcpy(&fsm_platform_data, plat, sizeof(struct stm_spifsm_platform));
	return 0;
}

/* Placeholder functions to avoid compilation issues while using 
 * SF framework */

void spi_init (void){/* Nothing to do*/}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	return NULL;
}
void spi_free_slave(struct spi_slave *slave) { /* Nothing to do */}
int spi_claim_bus(struct spi_slave *slave) { return 0; }
void spi_release_bus(struct spi_slave *slave)
{
	/* Nothing to do */
}
int  spi_xfer(struct spi_slave *slave, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	return 0;
}
