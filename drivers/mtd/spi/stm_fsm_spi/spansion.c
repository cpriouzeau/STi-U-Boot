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

#define S25FLXXXP_CAPS (FLASH_CAPS_READ_WRITE   | \
                        FLASH_CAPS_READ_1_1_2   | \
                        FLASH_CAPS_READ_1_2_2   | \
                        FLASH_CAPS_READ_1_1_4   | \
                        FLASH_CAPS_READ_1_4_4   | \
                        FLASH_CAPS_WRITE_1_1_4  | \
                        FLASH_CAPS_READ_FAST)    

#define S25FLXXXS_CAPS (S25FLXXXP_CAPS          | \
                        FLASH_CAPS_RESET)        

#define S25FL1XXK_CAPS (FLASH_CAPS_READ_WRITE   | \
                        FLASH_CAPS_READ_1_1_2   | \
                        FLASH_CAPS_READ_1_2_2   | \
                        FLASH_CAPS_READ_1_1_4   | \
                        FLASH_CAPS_READ_1_4_4   | \
                        FLASH_CAPS_READ_FAST)

/* QUAD bit in the configuration register of spansion */
#define S25FL129_CFG_QUAD (0x1 << 1)
		
static int s25fl_config(struct stm_spi_fsm *fsm, const struct spi_fsm_flash_params *params);

static struct spi_fsm_flash_params spansion_spi_flash_table[] = {
	{
		.name = "s25fl032p",
		.jedec_id = 0x010215,
		.ext_id = 0x4d00,
		.rdid_len = 5,
		.sector_size = 64 * 1024,
		.nr_sectors  = 64,
		.caps = S25FLXXXP_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl064p",
		.jedec_id = 0x010216,
		.ext_id = 0x4d00,
		.rdid_len = 5,
		.sector_size = 64 * 1024,
		.nr_sectors  = 128,
		.caps = S25FLXXXP_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl128p1",
		.jedec_id = 0x012018,
		.ext_id = 0x0300,
		.rdid_len = 5,
		.sector_size = 256 * 1024,
		.nr_sectors  = 64,
		.caps = (FLASH_CAPS_READ_WRITE | FLASH_CAPS_READ_FAST),
		.page_size = 256u,
		.max_freq = 104,
		.config = NULL,
	},
	{
		.name = "s25fl128p0",
		.jedec_id = 0x012018,
		.ext_id = 0x03001,
		.rdid_len = 5,
		.sector_size = 64 * 1024,
		.nr_sectors  = 256,
		.caps = (FLASH_CAPS_READ_WRITE | FLASH_CAPS_READ_FAST),
		.page_size = 256u,
		.max_freq = 104,
		.config = NULL,
	},
	{
		.name = "s25fl129p0",
		.jedec_id = 0x012018,
		.ext_id = 0x4d00,
		.rdid_len = 5,
		.sector_size = 256 * 1024,
		.nr_sectors  = 64,
		.caps = S25FLXXXP_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl129p1",
		.jedec_id = 0x012018,
		.ext_id = 0x4d01,
		.rdid_len = 5,
		.sector_size = 64 * 1024,
		.nr_sectors  = 256,
		.caps = S25FLXXXP_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl128s0",
		.jedec_id = 0x012018,
		.ext_id = 0x4d0080,
		.rdid_len = 6,
		.sector_size = 256 * 1024,
		.nr_sectors  = 64,
		.caps = S25FLXXXS_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl128s1",
		.jedec_id = 0x012018,
		.ext_id = 0x4d0180,
		.rdid_len = 6,
		.sector_size = 64 * 1024,
		.nr_sectors  = 256,
		.caps = S25FLXXXS_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl256s0",
		.jedec_id = 0x010219,
		.ext_id = 0x4d0080,
		.rdid_len = 6,
		.sector_size = 256 * 1024,
		.nr_sectors  = 128,
		.caps = S25FLXXXS_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl256s1",
		.jedec_id = 0x010219,
		.ext_id = 0x4d0180,
		.rdid_len = 6,
		.sector_size = 64 * 1024,
		.nr_sectors  = 512,
		.caps = S25FLXXXS_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl512s0",
		.jedec_id = 0x010220,
		.ext_id = 0x4d0080,
		.rdid_len = 6,
		.sector_size = 256 * 1024,
		.nr_sectors  = 256,
		.caps = S25FLXXXS_CAPS,
		.page_size = 256u,
		.max_freq = 80,
		.config = s25fl_config,
	},
	{
		.name = "s25fl116k",
		.jedec_id = 0x014015,
		.ext_id = 0,
		.rdid_len = 0,
		.sector_size = 64 * 1024,
		.nr_sectors  = 32,
		.caps = S25FL1XXK_CAPS,
		.page_size = 256u,
		.max_freq = 108,
		.config = s25fl_config,
	},
	{
		.name = "s25fl132k",
		.jedec_id = 0x014016,
		.ext_id = 0,
		.rdid_len = 0,
		.sector_size = 64 * 1024,
		.nr_sectors  = 64,
		.caps = S25FL1XXK_CAPS,
		.page_size = 256u,
		.max_freq = 108,
		.config = s25fl_config,
	},
	{
		.name = "s25fl164k",
		.jedec_id = 0x014017,
		.ext_id = 0,
		.rdid_len = 0,
		.sector_size = 64 * 1024,
		.nr_sectors  = 128,
		.caps = S25FL1XXK_CAPS,
		.page_size = 256u,
		.max_freq = 108,
		.config = s25fl_config,
	},
};


/*
 *[S25FLxxx] Configuration
 */
#define S25FL_CONFIG_QE			(0x1 << 1)
#define S25FL_CONFIG_TBPROT		(0x1 << 5)
#define S25FL1XXK_DEVICE_TYPE		0x40

/*
 * S25FLxxxS devices provide three ways of supporting 32-bit addressing: Bank
 * Register, Extended Address Modes, and a 32-bit address command set.  The
 * 32-bit address command set is used here, since it avoids any problems with
 * entering a state that is incompatible with the SPIBoot Controller.
 */
static struct seq_rw_config s25fl_read4_configs[] = {
	{FLASH_CAPS_READ_1_4_4,  FLASH_CMD_READ4_1_4_4,  0, 4, 4, 0x00, 2, 4},
	{FLASH_CAPS_READ_1_1_4,  FLASH_CMD_READ4_1_1_4,  0, 1, 4, 0x00, 0, 8},
	{FLASH_CAPS_READ_1_2_2,  FLASH_CMD_READ4_1_2_2,  0, 2, 2, 0x00, 4, 0},
	{FLASH_CAPS_READ_1_1_2,  FLASH_CMD_READ4_1_1_2,  0, 1, 2, 0x00, 0, 8},
	{FLASH_CAPS_READ_FAST,   FLASH_CMD_READ4_FAST,   0, 1, 1, 0x00, 0, 8},
	{FLASH_CAPS_READ_WRITE,  FLASH_CMD_READ4,        0, 1, 1, 0x00, 0, 0},
	{0x00,                   0,                      0, 0, 0, 0x00, 0, 0},
};

static struct seq_rw_config s25fl_write4_configs[] = {
	{FLASH_CAPS_WRITE_1_1_4, S25FL_CMD_WRITE4_1_1_4, 1, 1, 4, 0x00, 0, 0},
	{FLASH_CAPS_READ_WRITE,  S25FL_CMD_WRITE4,       1, 1, 1, 0x00, 0, 0},
	{0x00,                   0,                      0, 0, 0, 0x00, 0, 0},
};

static int s25fl_configure_erasesec_seq_32(struct fsm_seq *seq)
{
	seq->seq_opc[1] = (SEQ_OPC_PADS_1 |
			   SEQ_OPC_CYCLES(8) |
			   SEQ_OPC_OPCODE(S25FL_CMD_SE4));

	seq->addr_cfg = (ADR_CFG_CYCLES_ADD1(16) |
			 ADR_CFG_PADS_1_ADD1 |
			 ADR_CFG_CYCLES_ADD2(16) |
			 ADR_CFG_PADS_1_ADD2 |
			 ADR_CFG_CSDEASSERT_ADD2);
	return 0;
}

extern int s25fl_clear_status_reg(struct stm_spi_fsm *fsm)
{
	struct fsm_seq seq = {
		.seq_opc[0] = (SEQ_OPC_PADS_1 |
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(S25FL_CMD_CLSR) |
			       SEQ_OPC_CSDEASSERT),
		.seq_opc[1] = (SEQ_OPC_PADS_1 |
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(FLASH_CMD_WRDI) |
			       SEQ_OPC_CSDEASSERT),
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
	};

	fsm_load_seq(fsm, &seq);

	fsm_wait_seq(fsm);

	return 0;
}

static int s25fl_config(struct stm_spi_fsm *fsm, const struct spi_fsm_flash_params *params)
{
	uint32_t data_pads;
	uint8_t sr1, cr1;
	uint16_t sta_wr;
	int update_sr;

	if (params->caps & FLASH_CAPS_32BITADDR) {
		/*
		 * Configure Read/Write/Erase sequences according to S25FLxxx
		 * 32-bit address command set
		 */
		if (fsm_search_configure_rw_seq(&(fsm->seqs->read),
						s25fl_read4_configs,
						params->caps) != 0)
			return 1;

		if (fsm_search_configure_rw_seq(&(fsm->seqs->write),
						s25fl_write4_configs,
						params->caps) != 0)
			return 1;
		if (s25fl_configure_erasesec_seq_32(&(fsm->seqs->erase_sector)) != 0)
			return 1;

	} else {
		/* Use default configurations for 24-bit addressing */
		if (fsm_config_rw_seqs_default(fsm, params->caps) != 0)
			return 1;
	}

	/* WRSR must always cover CONFIG register to prevent loss of QUAD bit
	 * state
	 */
	fsm->plat->configuration |= CFG_WRSR_FORCE_16BITS;

	/*
	 * S25FLxxx devices support Program and Error error flags, with the
	 * exception of the S25FL1xxK family.
	 */
	if ((((params->jedec_id) << 8) && 0xff00) != S25FL1XXK_DEVICE_TYPE)
		fsm->plat->configuration |= CFG_S25FL_CHECK_ERROR_FLAGS;

	/* Check status of 'QE' bit, update if required. */
	data_pads = ((fsm->seqs->read.seq_cfg >> 16) & 0x3) + 1;
	fsm_read_status(fsm, FLASH_CMD_RDSR2, &cr1, 1);
	update_sr = 0;
	if (data_pads == 4) {
		if (!(cr1 & S25FL_CONFIG_QE)) {
			/* Set 'QE' */
			cr1 |= S25FL_CONFIG_QE;
			update_sr = 1;
		}
	} else {
		if (cr1 & S25FL_CONFIG_QE) {
			/* Clear 'QE' */
			cr1 &= ~S25FL_CONFIG_QE;
			update_sr = 1;
		}
	}
	if (update_sr) {
		fsm_read_status(fsm, FLASH_CMD_RDSR, &sr1, 1);
		sta_wr = ((uint16_t)cr1  << 8) | sr1;
		fsm_write_status(fsm, FLASH_CMD_WRSR, sta_wr, 2, 1);
	}

	return 0;
}

int spi_flash_probe_spansion(struct stm_spi_fsm *fsm, uint8_t *idcode)
{
	struct spi_fsm_flash_params *params;
	unsigned int i;
	int ret;
	uint32_t jedec;
	uint16_t ext_jedec;

	jedec = idcode[0] << 16 | idcode[1] << 8 | idcode[2];
	ext_jedec = idcode[3] << 8 | idcode[4];

	for (i = 0; i < ARRAY_SIZE(spansion_spi_flash_table); i++) {
		params = &spansion_spi_flash_table[i];

		if (params->jedec_id == jedec) {
			if (params->ext_id == 0){ /* Match any external id */
				break;
			} else if (params->ext_id == ext_jedec){
				if(params->rdid_len == 5){
					break;
				} else if (params->rdid_len == 6){
					if(params->ext_id == (idcode[3] << 16 | idcode[4] << 8 | idcode[5])){
						break;
					}
				}
			}
		}
	}

	if (i == ARRAY_SIZE(spansion_spi_flash_table)) {
		debug("FSM: Unsupported SPANSION ID %06x %04x\n", jedec, ext_jedec);
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

