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

#define N25Q_CAPS (FLASH_CAPS_READ_WRITE	| \
		   FLASH_CAPS_READ_FAST		| \
		   FLASH_CAPS_READ_1_1_2	| \
		   FLASH_CAPS_READ_1_2_2	| \
		   FLASH_CAPS_READ_1_1_4	| \
		   FLASH_CAPS_READ_1_4_4	| \
		   FLASH_CAPS_WRITE_1_1_2	| \
		   FLASH_CAPS_WRITE_1_2_2	| \
		   FLASH_CAPS_WRITE_1_1_4	| \
		   FLASH_CAPS_WRITE_1_4_4)


#define N25Q_32BITADDR_CAPS ((N25Q_CAPS		| \
			  FLASH_CAPS_RESET)	& \
			 ~FLASH_CAPS_WRITE_1_4_4)

#define M25P_CAPS (FLASH_CAPS_READ_WRITE | FLASH_CAPS_READ_FAST)

#define M25PX_CAPS (FLASH_CAPS_READ_WRITE	| \
		    FLASH_CAPS_READ_FAST	| \
		    FLASH_CAPS_READ_1_1_2	| \
		    FLASH_CAPS_WRITE_1_1_2)

#if 0 /* Redundant right now. Keep for future */
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
#endif
/* Default WRITE configurations, in order of preference */
static struct seq_rw_config default_write_configs[] = {
        {FLASH_CAPS_WRITE_1_4_4, FLASH_CMD_WRITE_1_4_4, 1, 4, 4, 0x00, 0, 0},
        {FLASH_CAPS_WRITE_1_1_4, FLASH_CMD_WRITE_1_1_4, 1, 1, 4, 0x00, 0, 0},
        {FLASH_CAPS_WRITE_1_2_2, FLASH_CMD_WRITE_1_2_2, 1, 2, 2, 0x00, 0, 0},
        {FLASH_CAPS_WRITE_1_1_2, FLASH_CMD_WRITE_1_1_2, 1, 1, 2, 0x00, 0, 0},
        {FLASH_CAPS_READ_WRITE,  FLASH_CMD_WRITE,       1, 1, 1, 0x00, 0, 0},

        /* terminating entry */
        {0x00,                    0,                     0, 0, 0, 0x00, 0, 0},
};

/*
 * [N25Qxxx] Configuration
 */
#define N25Q_VCR_DUMMY_CYCLES(x)	(((x) & 0xf) << 4)
#define N25Q_VCR_XIP_DISABLED		((uint8_t)0x1 << 3)
#define N25Q_VCR_WRAP_CONT		0x3

/* N25Q 3-byte Address READ configurations
 *	- 'FAST' variants configured for 8 dummy cycles.
 *
 * Note, the number of dummy cycles used for 'FAST' READ operations is
 * configurable and would normally be tuned according to the READ command and
 * operating frequency.  However, this applies universally to all 'FAST' READ
 * commands, including those used by the SPIBoot controller, and remains in
 * force until the device is power-cycled.  Since the SPIBoot controller is
 * hard-wired to use 8 dummy cycles, we must configure the device to also use 8
 * cycles.
 */
static struct seq_rw_config n25q_read3_configs[] = {
	{FLASH_CAPS_READ_1_4_4, FLASH_CMD_READ_1_4_4,	0, 4, 4, 0x00, 0, 8},
	{FLASH_CAPS_READ_1_1_4, FLASH_CMD_READ_1_1_4,	0, 1, 4, 0x00, 0, 8},
	{FLASH_CAPS_READ_1_2_2, FLASH_CMD_READ_1_2_2,	0, 2, 2, 0x00, 0, 8},
	{FLASH_CAPS_READ_1_1_2, FLASH_CMD_READ_1_1_2,	0, 1, 2, 0x00, 0, 8},
	{FLASH_CAPS_READ_FAST,	FLASH_CMD_READ_FAST,	0, 1, 1, 0x00, 0, 8},
	{FLASH_CAPS_READ_WRITE, FLASH_CMD_READ,	        0, 1, 1, 0x00, 0, 0},
	{0x00,			 0,			0, 0, 0, 0x00, 0, 0},
};

/* N25Q 4-byte Address READ configurations
 *	- use special 4-byte address READ commands (reduces overheads, and
 *        reduces risk of hitting watchdog resets issues).
 *	- 'FAST' variants configured for 8 dummy cycles (see note above.)
 */
static struct seq_rw_config n25q_read4_configs[] = {
	{FLASH_CAPS_READ_1_4_4, FLASH_CMD_READ4_1_4_4,	0, 4, 4, 0x00, 0, 8},
	{FLASH_CAPS_READ_1_1_4, FLASH_CMD_READ4_1_1_4,	0, 1, 4, 0x00, 0, 8},
	{FLASH_CAPS_READ_1_2_2, FLASH_CMD_READ4_1_2_2,	0, 2, 2, 0x00, 0, 8},
	{FLASH_CAPS_READ_1_1_2, FLASH_CMD_READ4_1_1_2,	0, 1, 2, 0x00, 0, 8},
	{FLASH_CAPS_READ_FAST,	FLASH_CMD_READ4_FAST,	0, 1, 1, 0x00, 0, 8},
	{FLASH_CAPS_READ_WRITE, FLASH_CMD_READ4,	0, 1, 1, 0x00, 0, 0},
	{0x00,			 0,			0, 0, 0, 0x00, 0, 0},
};

static int n25q_configure_en32bitaddr_seq(struct fsm_seq *seq)
{
	seq->seq_opc[0] = (SEQ_OPC_PADS_1 | SEQ_OPC_CYCLES(8) |
			   SEQ_OPC_OPCODE(FLASH_CMD_EN4B_ADDR));
	seq->seq_opc[1] = (SEQ_OPC_PADS_1 | SEQ_OPC_CYCLES(8) |
			   SEQ_OPC_OPCODE(FLASH_CMD_WREN) |
			   SEQ_OPC_CSDEASSERT);

	seq->seq[0] = FSM_INST_CMD2;
	seq->seq[1] = FSM_INST_CMD1;
	seq->seq[2] = FSM_INST_WAIT;
	seq->seq[3] = FSM_INST_STOP;

	seq->seq_cfg = (SEQ_CFG_PADS_1 |
			SEQ_CFG_ERASE |
			SEQ_CFG_READNOTWRITE |
			SEQ_CFG_CSDEASSERT |
			SEQ_CFG_STARTSEQ);

	return 0;
}

extern int n25q_clear_flags(struct stm_spi_fsm *fsm)
{
	struct fsm_seq seq = {
		.seq_opc[0] = (SEQ_OPC_PADS_1 |
			       SEQ_OPC_CYCLES(8) |
			       SEQ_OPC_OPCODE(N25Q_CMD_CLFSR) |
			       SEQ_OPC_CSDEASSERT),
		.seq = {
			FSM_INST_CMD1,
			FSM_INST_STOP,
		},
		.seq_cfg = (SEQ_CFG_PADS_1 |
			    SEQ_CFG_READNOTWRITE |
			    SEQ_CFG_CSDEASSERT |
			    SEQ_CFG_STARTSEQ),
	};

	fsm_load_seq(fsm, &seq);

	fsm_wait_seq(fsm);

	return 0;
}


static int n25q_config(struct stm_spi_fsm *fsm, const struct spi_fsm_flash_params *params)
{
	uint8_t vcr, sta;
	int ret = 0;

	/*
	 * Configure 'READ' sequence
	 */
	if (params->caps & FLASH_CAPS_32BITADDR)
		/* 32-bit addressing supported by N25Q 'READ4' commands */
		ret = fsm_search_configure_rw_seq(&(fsm->seqs->read),
						  n25q_read4_configs,
						  params->caps);
	else
		/* 24-bit addressing with 8 dummy cycles */
		ret = fsm_search_configure_rw_seq(&(fsm->seqs->read),
						  n25q_read3_configs,
						  params->caps);

	if (ret != 0) {
		debug("failed to configure READ sequence according to capabilities [0x%08x]\n",
			params->caps);
		return 1;
	}


	/*
	 * Configure 'WRITE' sequence (use default configs)
	 */
	ret = fsm_search_configure_rw_seq(&(fsm->seqs->write),
					  default_write_configs,
					  params->caps);
	if (ret != 0) {
		debug("failed to configure WRITE sequence according to capabilities [0x%08x]\n",
			params->caps);
		return 1;
	}

	/*
	 * Configure 'ERASE_SECTOR' sequence
	 */
	configure_erasesec_seq(&(fsm->seqs->erase_sector),
			       params->caps & FLASH_CAPS_32BITADDR);

	/*
	 * Configure 32-bit address support
	 */
	if (params->caps & FLASH_CAPS_32BITADDR) {
		/* Configure 'enter_32bitaddr' FSM sequence */
		n25q_configure_en32bitaddr_seq(&(fsm->seqs->en32bitaddr));

		if (!fsm->plat->capabilities.boot_from_spi){
			/* If we can handle SoC resets, we enable 32-bit address
			 * mode pervasively */
			fsm_enter_32bitaddr(fsm, 1);
		} else {
			/* Else, enable/disable for WRITE and ERASE operations
			 * (READ uses special commands) */
			fsm->plat->configuration |= (CFG_WRITE_TOGGLE32BITADDR |
					      CFG_ERASESEC_TOGGLE32BITADDR |
					      CFG_LOCK_TOGGLE32BITADDR);
		}
	}

	/*
	 * Check/Clear Error Flags
	 */
	fsm->plat->configuration |= CFG_N25Q_CHECK_ERROR_FLAGS;
	fsm_read_status(fsm, N25Q_CMD_RFSR, &sta, 1);
	if (sta & N25Q_FLAGS_ERROR)
		n25q_clear_flags(fsm);

	/*
	 * Configure device to use 8 dummy cycles
	 */
	vcr = (N25Q_VCR_DUMMY_CYCLES(8) | N25Q_VCR_XIP_DISABLED |
	       N25Q_VCR_WRAP_CONT);
	fsm_write_status(fsm, N25Q_CMD_WRVCR, vcr, 1, 0);

	return ret;
}
		
static struct spi_fsm_flash_params stmicro_spi_flash_table[] = {
	{
		.name = "ST N25Q032",
		.jedec_id = 0x20ba16,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.caps = N25Q_CAPS,
		.page_size = 256u,
		.max_freq = 95, 
		.config = n25q_config,
	},
	{
		.name = "ST N25Q064",
		.jedec_id = 0x20ba17,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.caps = N25Q_CAPS,
		.page_size = 256u,
		.max_freq = 95, 
		.config = n25q_config,
	},
	{
		.name = " ST N25Q128",
		.jedec_id = 0x20ba18,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 256,
		.caps = N25Q_CAPS,
		.page_size = 256u,
		.max_freq = 108, 
		.config = n25q_config,
	},
	{
		.name = "ST N25Q256",
		.jedec_id = 0x20ba19,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 512,
		.caps = N25Q_32BITADDR_CAPS,
		.page_size = 256u,
		.max_freq = 108, 
		.config = n25q_config,
	},
	{
		.name = "ST N25Q512",
		.jedec_id = 0x20ba20,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 1024,
		.caps = N25Q_32BITADDR_CAPS,
		.page_size = 256u,
		.max_freq = 108, 
		.config = n25q_config,
	},
	{
		.name = "ST N25Q00A",
		.jedec_id = 0x20ba21,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 2048,
		.caps = N25Q_32BITADDR_CAPS,
		.page_size = 256u,
		.max_freq = 108, 
		.config = n25q_config,
	},
	{
		.name = "ST M25P40",
		.jedec_id = 0x202013,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 8,
		.caps = M25P_CAPS,
		.page_size = 256u,
		.max_freq = 25, 
		.config = NULL,
	},

	{
		.name = "ST M25P80",
		.jedec_id = 0x202014,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 16,
		.caps = M25P_CAPS,
		.page_size = 256u,
		.max_freq = 25, 
		.config = NULL,
	},
	{
		.name = "ST M25P16",
		.jedec_id = 0x202015,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 32,
		.caps = M25P_CAPS,
		.page_size = 256u,
		.max_freq = 25, 
		.config = NULL,
	},
	{
		.name = "ST M25P32",
		.jedec_id = 0x202016,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 64,
		.caps = M25P_CAPS,
		.page_size = 256u,
		.max_freq = 50, 
		.config = NULL,
	},
	{
		.name = "ST M25P64",
		.jedec_id = 0x202017,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 128,
		.caps = M25P_CAPS,
		.page_size = 256u,
		.max_freq = 50, 
		.config = NULL,
	},
	{
		.name = "ST M25P128",
		.jedec_id = 0x202018,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 256 * 1024,
		.nr_sectors = 64,
		.caps = M25P_CAPS,
		.page_size = 256u,
		.max_freq = 50, 
		.config = NULL,
	},
	{
		.name = "ST M25PX80",
		.jedec_id = 0x207114,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.caps = M25PX_CAPS,
		.page_size = 256u,
		.max_freq = 75, 
		.config = NULL,
	},
	{
		.name = "ST M25PX16",
		.jedec_id = 0x207115,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.caps = M25PX_CAPS,
		.page_size = 256u,
		.max_freq = 75, 
		.config = NULL,
	},
	{
		.name = "ST M25PX32",
		.jedec_id = 0x207116,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 64,
		.caps = M25PX_CAPS,
		.page_size = 256u,
		.max_freq = 75, 
		.config = NULL,
	},
	{
		.name = "ST M25PX64",
		.jedec_id = 0x207117,
		.ext_id = 0x0,
		.rdid_len = 0x0,
		.sector_size = 64 * 1024,
		.nr_sectors = 128,
		.caps = M25PX_CAPS,
		.page_size = 256u,
		.max_freq = 75, 
		.config = NULL,
	},
};

int spi_flash_probe_stmicro(struct stm_spi_fsm *fsm, uint8_t *idcode)
{
	struct spi_fsm_flash_params *params;
	unsigned int i;
	int ret;
	uint32_t jedec;
	uint16_t ext_jedec;

	jedec = idcode[0] << 16 | idcode[1] << 8 | idcode[2];
	ext_jedec = idcode[3] << 8 | idcode[4];

	for (i = 0; i < ARRAY_SIZE(stmicro_spi_flash_table); i++) {
		params = &stmicro_spi_flash_table[i];

		if (params->jedec_id == jedec) {
			if (params->ext_id == 0) /* Match any external id */
				break;
			else if (params->ext_id == ext_jedec)
				break;
		}
	}

	if (i == ARRAY_SIZE(stmicro_spi_flash_table)) {
		debug("FSM: Unsupported STMICRO ID %06x %04x\n", jedec, ext_jedec);
		return -1;
	}

	/* Mask-out capabilities not supported by the platform */
        if (fsm->plat->capabilities.quad_mode == 0)
                params->caps &= ~FLASH_CAPS_QUAD;
        if (fsm->plat->capabilities.dual_mode == 0)
                params->caps &= ~FLASH_CAPS_DUAL;

        
	fsm->flash.name        = params->name;
	fsm->flash.page_size   = params->page_size;

	if (
		(idcode[0] == 0x20u)	&&	/* Manufacturer ID */
		(idcode[1] == 0x20u)	&&	/* Memory Type */
		(				/* Memory Capacity */
			(idcode[2] == 0x14u) ||	/* M25P80 */
			(idcode[2] == 0x15u) ||	/* M25P16 */
			(idcode[2] == 0x16u) ||	/* M25P32 */
			(idcode[2] == 0x17u) ||	/* M25P64 */
			(idcode[2] == 0x18u)	/* M25P128 */
		)
	   )
	{
		if(idcode[2] == 0x18u){
			fsm->flash.sector_size	= 256u<<10;
			fsm->flash.erase_size	= 256u<<10;
		} else {
			fsm->flash.sector_size	= 64u<<10;
			fsm->flash.erase_size	= 64u<<10;
		}
		fsm->flash.size		= 1u<<idcode[2];		
	}
	else if (
		(idcode[0] == 0x20u)	&&	/* Manufacturer ID */
		(idcode[1] == 0x71u)	&&	/* Memory Type */
		(				/* Memory Capacity */
			(idcode[2] == 0x14u) ||	/* M25PX80 */
			(idcode[2] == 0x15u) ||	/* M25PX16 */
			(idcode[2] == 0x16u) ||	/* M25PX32 */
			(idcode[2] == 0x17u)	/* M25PX64 */
		)
	   )
	{
		if((idcode[2] == 0x14u) || (idcode[2] == 0x15u)){
			fsm->flash.sector_size	= 4u<<10;
		}
		fsm->flash.erase_size	= fsm->flash.sector_size;
		fsm->flash.size	= 1u<<idcode[2];		
	}
	else if (
		(idcode[0] == 0x20u)	&&	/* Manufacturer ID */
		(idcode[1] == 0xBAu)	&&	/* Memory Type */
		(				/* Memory Capacity */
			(idcode[2] == 0x16u) ||	/* N25Q032 */
			(idcode[2] == 0x17u) ||	/* N25Q064 */
			(idcode[2] == 0x18u) ||	/* N25Q128 */
			(idcode[2] == 0x19u) ||	/* N25Q256 */
			(idcode[2] == 0x20u) ||	/* N25Q512 */
			(idcode[2] == 0x21u)	/* N25Q00A */
		)
	   )
	{
		fsm->flash.sector_size	= 64u<<10;
		fsm->flash.erase_size	= 64u<<10;
		if(idcode[2] == 0x16u || idcode[2] == 0x17u)
			fsm->flash.size  = 1u<<idcode[2];		
		else
			fsm->flash.size  = params->sector_size * params->nr_sectors;
	}

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

