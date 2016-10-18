/*
 * Copyright (C) 2014-2015 STMicroelectronics.
 *
 * Author: Imran Khan (imran.khan@st.com)
 *
 * SPDX-License-Identifier:	GPL-2.0+
 * 
 */

#include <common.h>
#include <malloc.h>
#include <stm_spi_fsm.h>

static int mx25_config(struct stm_spi_fsm *fsm, const struct spi_fsm_flash_params *params);

#define MX25_CAPS (FLASH_CAPS_READ_WRITE	| \
		   FLASH_CAPS_READ_FAST		| \
		   FLASH_CAPS_READ_1_1_2	| \
		   FLASH_CAPS_READ_1_2_2	| \
		   FLASH_CAPS_READ_1_1_4	| \
		   FLASH_CAPS_READ_1_4_4	| \
		   FLASH_CAPS_SE_4K		| \
		   FLASH_CAPS_SE_32K)

/* Default READ configurations, in order of preference */
static struct seq_rw_config default_read_configs[] = {
        {FLASH_CAPS_READ_1_4_4, FLASH_CMD_READ_1_4_4,   0, 4, 4, 0x00, 2, 4},
        {FLASH_CAPS_READ_1_1_4, FLASH_CMD_READ_1_1_4,   0, 1, 4, 0x00, 4, 0},
        {FLASH_CAPS_READ_1_2_2, FLASH_CMD_READ_1_2_2,   0, 2, 2, 0x00, 4, 0},
        {FLASH_CAPS_READ_1_1_2, FLASH_CMD_READ_1_1_2,   0, 1, 2, 0x00, 0, 8},
        {FLASH_CAPS_READ_FAST,  FLASH_CMD_READ_FAST,    0, 1, 1, 0x00, 0, 8},
        {FLASH_CAPS_READ_WRITE, FLASH_CMD_READ,         0, 1, 1, 0x00, 0, 0},

        /* terminating entry */
        {0x00,                   0,                     0, 0, 0, 0x00, 0, 0},
};

static struct spi_fsm_flash_params macronix_spi_flash_table[] = {
	{
		.name = "mx25l3255e",
		.jedec_id = 0xc29e16,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 64,
		.caps = (MX25_CAPS | FLASH_CAPS_WRITE_1_4_4),
		.page_size = 256u,
		.max_freq = 86, 
		.config = mx25_config,
	},
	{
		.name = "MX25L2006E",
		.jedec_id = 0xc22012,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 4,
		.caps = MX25_CAPS,
		.page_size = 256u,
		.max_freq = 50,
		.config = mx25_config,
	},
	{
		.name = "MX25L4005",
		.jedec_id = 0xc22013,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 8,
		.caps = MX25_CAPS,
		.page_size = 256u,
		.max_freq = 50,
		.config = mx25_config,
	},
	{
		.name = "MX25L8005",
		.jedec_id = 0xc22014,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 16,
		.caps = MX25_CAPS,
		.page_size = 256u,
		.max_freq = 50,
		.config = mx25_config,
	},
	{
		.name = "MX25L1605D",
		.jedec_id = 0xc22015,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 32,
		.caps = MX25_CAPS,
		.page_size = 256u,
		.max_freq = 50,
		.config = mx25_config,
	},
	{
		.name = "MX25L3205D",
		.jedec_id = 0xc22016,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 64,
		.caps = MX25_CAPS,
		.page_size = 256u,
		.max_freq = 64,
		.config = mx25_config,
	},
	{
		.name = "MX25L6405D",
		.jedec_id = 0xc22017,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 128,
		.caps = MX25_CAPS,
		.page_size = 256u,
		.max_freq = 50,
		.config = mx25_config,
	},

	{
		.name = "MX25L12805",
		.jedec_id = 0xc22018,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 256,
		.caps = (MX25_CAPS | FLASH_CAPS_RESET),
		.page_size = 256u,
		.max_freq = 70, 
		.config = mx25_config,
	},
	{
		.name = "MX25L25635F",
		.jedec_id = 0xc22019,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 512,
		.caps = (MX25_CAPS | FLASH_CAPS_RESET),
		.page_size = 256u,
		.max_freq = 70, 
		.config = mx25_config,
	},
	{
		.name = "MX25L51235F",
		.jedec_id = 0xc2201a,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 1024,
		.caps = (MX25_CAPS | FLASH_CAPS_RESET),
		.page_size = 256u,
		.max_freq = 70, 
		.config = mx25_config,
	},
	{
		.name = "MX25L1655D",
		.jedec_id = 0xc22615,
		.ext_id = 0x0,
		.sector_size = 4 * 1024,
		.nr_sectors = 512,
		.caps = (MX25_CAPS | FLASH_CAPS_RESET),
		.page_size = 256u,
		.max_freq = 70, 
		.config = mx25_config,
	},
	{
		.name = "MX25L6455E",
		.jedec_id = 0xc22617,
		.ext_id = 0x0,
		.sector_size = 4 * 1024,
		.nr_sectors = 2048,
		.caps = (MX25_CAPS | FLASH_CAPS_RESET),
		.page_size = 256u,
		.max_freq = 70, 
		.config = mx25_config,
	},

	{
		.name = "MX25L12855E",
		.jedec_id = 0xc22618,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 256,
		.caps = (MX25_CAPS | FLASH_CAPS_RESET),
		.page_size = 256u,
		.max_freq = 70, 
		.config = mx25_config,
	},

	{
		.name = "MX25L25655F",
		.jedec_id = 0xc22619,
		.ext_id = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 512,
		.caps = (MX25_CAPS | FLASH_CAPS_RESET),
		.page_size = 256u,
		.max_freq = 70, 
		.config = mx25_config,
	},
	};

/* mx25 WRITE configurations, in order of preference */
static struct seq_rw_config mx25_write_configs[] = {
	{FLASH_CAPS_WRITE_1_4_4, MX25_CMD_WRITE_1_4_4,  1, 4, 4, 0x00, 0, 0},
	{FLASH_CAPS_READ_WRITE,  FLASH_CMD_WRITE,       1, 1, 1, 0x00, 0, 0},

	/* terminating entry */
	{0x00,			  0,			 0, 0, 0, 0x00, 0, 0},
};

static int mx25_configure_en32bitaddr_seq(struct fsm_seq *seq)
{
	seq->seq_opc[0] = (SEQ_OPC_PADS_1 |
			   SEQ_OPC_CYCLES(8) |
			   SEQ_OPC_OPCODE(FLASH_CMD_EN4B_ADDR) |
			   SEQ_OPC_CSDEASSERT);

	seq->seq[0] = FSM_INST_CMD1;
	seq->seq[1] = FSM_INST_WAIT;
	seq->seq[2] = FSM_INST_STOP;

	seq->seq_cfg = (SEQ_CFG_PADS_1 |
			SEQ_CFG_ERASE |
			SEQ_CFG_READNOTWRITE |
			SEQ_CFG_CSDEASSERT |
			SEQ_CFG_STARTSEQ);

	return 0;
}

static int mx25_config(struct stm_spi_fsm *fsm, const struct spi_fsm_flash_params *params)
{
	uint32_t data_pads;
	uint32_t capabilities = params->caps;
	uint8_t sta;

	/* Configure 'READ' sequence */
	if (fsm_search_configure_rw_seq(&(fsm->seqs->read),
					default_read_configs,
					params->caps) != 0) {
		debug("failed to configure READ sequence according to capabilities [0x%08x]\n",
			capabilities);
		return 1;
	}

	/* Configure 'WRITE' sequence */
	if (fsm_search_configure_rw_seq(&(fsm->seqs->write),
					mx25_write_configs,
					capabilities) != 0) {
		debug("failed to configure WRITE sequence according to capabilities [0x%08x]\n",
			capabilities);
		return 1;
	}

	/* Configure 'ERASE_SECTOR' sequence */
	configure_erasesec_seq(&(fsm->seqs->erase_sector),
			       capabilities & FLASH_CAPS_32BITADDR);

	/*
	 * Configure 32-bit Address Support
	 */
	if (params->caps & FLASH_CAPS_32BITADDR) {
		/* Configure 'enter_32bitaddr' FSM sequence */
		mx25_configure_en32bitaddr_seq(&(fsm->seqs->en32bitaddr));

		if (!fsm->plat->capabilities.boot_from_spi){
			/* If we can handle SoC resets, we enable 32-bit address
			 * mode pervasively */
			fsm_enter_32bitaddr(fsm, 1);
		} else {
			/* Else, enable/disable 32-bit addressing before/after
			 * each operation */
			fsm->plat->configuration = (CFG_READ_TOGGLE32BITADDR |
					      CFG_WRITE_TOGGLE32BITADDR |
					      CFG_ERASESEC_TOGGLE32BITADDR);
		}
	}

	/* Check status of 'QE' bit, update if required. */
	data_pads = ((fsm->seqs->read.seq_cfg >> 16) & 0x3) + 1;
	fsm_read_status(fsm, FLASH_CMD_RDSR, &sta, 1);
	if (data_pads == 4) {
		if (!(sta & MX25_STATUS_QE)) {
			/* Set 'QE' */
			sta |= MX25_STATUS_QE;
			fsm_write_status(fsm, FLASH_CMD_WRSR, sta, 1, 1);
		}

		/* impossible to lock/unlock mx25l3255e if QE bit is set */
		fsm->plat->configuration |= CFG_MX25_LOCK_TOGGLE_QE_BIT;
	} else {
		if (sta & MX25_STATUS_QE) {
			/* Clear 'QE' */
			sta &= ~MX25_STATUS_QE;
			fsm_write_status(fsm, FLASH_CMD_WRSR, sta, 1, 1);
		}
	}

	return 0;
}

int spi_flash_probe_macronix(struct stm_spi_fsm *fsm, uint8_t *idcode)
{
	struct spi_fsm_flash_params *params;
	unsigned int i;
	int ret;
	uint32_t jedec;

	jedec = idcode[0] << 16 | idcode[1] << 8 | idcode[2];

	for (i = 0; i < ARRAY_SIZE(macronix_spi_flash_table); i++) {
		params = &macronix_spi_flash_table[i];

		if (params->jedec_id == jedec) {
			break;
		}
	}

	if (i == ARRAY_SIZE(macronix_spi_flash_table)) {
		debug("FSM: Unsupported MACRONIX ID %06x\n", jedec);
		return -1;
	}

	/* Mask-out capabilities not supported by the platform */
        if (fsm->plat->capabilities.quad_mode == 0)
                params->caps &= ~FLASH_CAPS_QUAD;
        if (fsm->plat->capabilities.dual_mode == 0)
                params->caps &= ~FLASH_CAPS_DUAL;
	
	fsm->flash.name        = params->name;
	fsm->flash.page_size   = params->page_size;
	fsm->flash.sector_size = params->sector_size;
	fsm->flash.erase_size  = params->sector_size;
	fsm->flash.size        = params->sector_size * params->nr_sectors;		

	/* Determine 32-bit addressing requirement */
        if (fsm->flash.size > 0x1000000)
                params->caps |= FLASH_CAPS_32BITADDR;

	if (params->config) {
		ret = params->config(fsm, params);
	} else {
		ret = fsm_config_rw_seqs_default(fsm, params->caps);
	}
	
	if (ret)
		return ret;

	
	fsm->flash.write       = stm_spifsm_write;
	fsm->flash.erase       = stm_spifsm_erase;
	fsm->flash.read        = stm_spifsm_read;

	fsm_set_freq(fsm, params->max_freq);

	return 0;
}

