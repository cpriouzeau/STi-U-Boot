/*
 * stm_spi_fsm.h    Support for STM SPI FSM Controller
 *
 * Author: Angus Clark <angus.clark@st.com>
 *         Imran Khan <imran.khan@st.com>
 *
 * Copyright (C) 2010 STMicroelectronics Limited
 *
 * May be copied or modified under the terms of the GNU General Public
 * License Version 2.0 only.  See linux/COPYING for more information.
 *
 */
#ifndef STM_SPI_FSM_H
#define STM_SPI_FSM_H

#include <spi_flash.h>

#define JEDEC_LEN 6

struct fsm_seq {
	uint32_t data_size;
	uint32_t addr1;
	uint32_t addr2;
	uint32_t addr_cfg;
	uint32_t seq_opc[5];
	uint32_t mode;
	uint32_t dummy;
	uint32_t status;
	uint8_t  seq[16];
	uint32_t seq_cfg;
} __attribute__((__packed__, aligned(4)));
#define FSM_SEQ_SIZE			sizeof(struct fsm_seq)

struct fsm_seqs_list {
	struct fsm_seq dummy;
	struct fsm_seq read_jedec;
	struct fsm_seq read_status_fifo;
	struct fsm_seq write_status;
	struct fsm_seq wrvcr;
	struct fsm_seq erase_sector;
	struct fsm_seq erase_chip;
	struct fsm_seq read;
	struct fsm_seq write;
	struct fsm_seq en32bitaddr;
};

/* FSM Structures needed for interfaces */
/* ------------------------------------ */

struct stm_spifsm_caps {
	/* Board/SoC/IP capabilities */
	int dual_mode:1;		/* DUAL mode */
	int quad_mode:1;		/* QUAD mode */

	 unsigned boot_from_spi:1;       /* Boot device is SPI */
	/* IP capabilities */
	int addr_32bit:1;		/* 32bit addressing supported */
	int no_poll_mode_change:1;	/* Polling MODE_CHANGE broken */
	int no_clk_div_4:1;		/* Bug prevents ClK_DIV=4 */
	int no_sw_reset:1;		/* S/W reset not possible */
	int dummy_on_write:1;		/* Bug requires "dummy" sequence on
					 * WRITE */
	int no_read_repeat:1;		/* READ repeat sequence broken -- UNUSED */
	int no_write_repeat:1;		/* WRITE repeat sequence broken -- UNUSED */
	enum {
		spifsm_no_read_status = 1,	/* READ_STA broken */
		spifsm_read_status_clkdiv4,	/* READ_STA only at CLK_DIV=4 */
	} read_status_bug;
};

struct stm_spifsm_platform {
	uint32_t		base;
	struct stm_spifsm_caps	capabilities;
	uint32_t		configuration;
	uint32_t		fifo_dir_delay;
};

struct stm_spi_fsm {
	struct spi_flash flash;
	struct stm_spifsm_platform* plat;
	struct fsm_seqs_list* seqs;
};

/* FSM Interface function */
/* ---------------------- */
int stm_spifsm_register_device(struct stm_spifsm_platform *data);

/* Parameters to configure a READ or WRITE FSM sequence */
struct seq_rw_config {
	uint32_t	capabilities;	/* capabilities to support config */
	uint8_t		cmd;		/* FLASH command */
	int		write;		/* Write Sequence */
	uint8_t		addr_pads;	/* No. of addr pads (MODE & DUMMY) */
	uint8_t		data_pads;	/* No. of data pads */
	uint8_t		mode_data;	/* MODE data */
	uint8_t		mode_cycles;	/* No. of MODE cycles */
	uint8_t		dummy_cycles;	/* No. of DUMMY cycles */
};

struct spi_fsm_flash_params {
	const char *name;
	uint32_t jedec_id;
	uint32_t ext_id;
	uint8_t	 rdid_len;
	uint32_t sector_size;
	uint16_t nr_sectors;
	uint32_t caps;
	uint16_t max_freq;
	uint32_t page_size;
	uint16_t pages_per_sector;
	int (*config)(struct stm_spi_fsm *fsm, 		      
		      const struct spi_fsm_flash_params* params);		      
};

#define FLASH_CMD_EN4B_ADDR     0xb7    /* Enter 4-byte address mode */
#define FLASH_CMD_EX4B_ADDR     0xe9    /* Exit 4-byte address mode */

/* [MX25xxx] Configure READ/Write sequences */
#define MX25_STATUS_QE                  (0x1 << 6)
#define MX25_SECURITY_WPSEL             (0x1 << 7)
#define MX25_CONFIG_TB                  (0x1 << 3)

#define MX25L32_DEVICE_ID               0x16
#define MX25L128_DEVICE_ID              0x18

/* Maximum operation times (in ms) */
#define FLASH_MAX_CHIP_ERASE_MS 500000          /* Chip Erase time */
#define FLASH_MAX_SEC_ERASE_MS  30000           /* Sector Erase time */
#define FLASH_MAX_PAGE_WRITE_MS 100             /* Write Page time */
#define FLASH_MAX_STA_WRITE_MS  4000            /* Write status reg time */
#define FSM_MAX_WAIT_SEQ_MS     1000            /* FSM execution time */

/*
 *  * Flags to tweak operation of default read/write/erase/lock routines
 *   */
#define CFG_READ_TOGGLE32BITADDR                0x00000001
#define CFG_WRITE_TOGGLE32BITADDR               0x00000002
#define CFG_LOCK_TOGGLE32BITADDR                0x00000004
#define CFG_ERASESEC_TOGGLE32BITADDR            0x00000008
#define CFG_S25FL_CHECK_ERROR_FLAGS             0x00000010
#define CFG_N25Q_CHECK_ERROR_FLAGS              0x00000020
#define CFG_WRSR_FORCE_16BITS                   0x00000040
#define CFG_RD_WR_LOCK_REG                      0x00000080
#define CFG_MX25_LOCK_TOGGLE_QE_BIT             0x00000100

/* N25Q Commands */
/*	- READ/CLEAR Flags Status register */
#define N25Q_CMD_RFSR		0x70
#define N25Q_CMD_CLFSR		0x50
#define N25Q_CMD_WRVCR		0x81
#define N25Q_CMD_RDVCR		0x85
#define N25Q_CMD_RDVECR		0x65
#define N25Q_CMD_RDNVCR		0xb5
#define N25Q_CMD_WRNVCR		0xb1
#define N25Q_CMD_RDLOCK		0xe8
#define N25Q_CMD_WRLOCK		0xe5


/* N25Q Flags Status Register: Error Flags */
#define N25Q_FLAGS_ERR_ERASE	(1 << 5)
#define N25Q_FLAGS_ERR_PROG	(1 << 4)
#define N25Q_FLAGS_ERR_VPP	(1 << 3)
#define N25Q_FLAGS_ERR_PROT	(1 << 1)
#define N25Q_FLAGS_ERROR	(N25Q_FLAGS_ERR_ERASE	| \
				 N25Q_FLAGS_ERR_PROG	| \
				 N25Q_FLAGS_ERR_VPP	| \
				 N25Q_FLAGS_ERR_PROT)

/* W25Q commands */
#define W25Q_CMD_RDLOCK		0x3d
#define W25Q_CMD_LOCK		0x36
#define W25Q_CMD_UNLOCK		0x39

/* MX25 Commands */
/*	- Read Security Register (home of '4BYTE' status bit!) */
#define MX25_CMD_WRITE_1_4_4	0x38
#define MX25_CMD_RDCR		0x15
#define MX25_CMD_RDSCUR		0x2b
#define MX25_CMD_RDSFDP		0x5a
#define MX25_CMD_SBLK		0x36
#define MX25_CMD_SBULK		0x39
#define MX25_CMD_RDBLOCK	0x3c
#define MX25_CMD_RDDPB		0xe0
#define MX25_CMD_WRDPB		0xe1

/* S25FLxxxS commands */
/*	- WRITE/ERASE 32-bit address commands */
#define S25FL_CMD_WRITE4	0x12	/* Note, opcode shared with clashes with
					 * 'FLASH_CMD_WRITE_1_4_4', as found on
					 * N25Qxxx devices! */
#define S25FL_CMD_WRITE4_1_1_4	0x34
#define S25FL_CMD_SE4		0xdc
/*	- Clear status register flags */
#define S25FL_CMD_CLSR		0x30
/*	- Read/Write 'DYB' lock bit */
#define S25FL_CMD_DYBWR		0xe1
#define S25FL_CMD_DYBRD		0xe0

/*      - S25FL Error Flags */
#define S25FL_STATUS_E_ERR      0x20
#define S25FL_STATUS_P_ERR      0x40
/*      - Timeout status */
#define FLASH_STATUS_TIMEOUT    0xff

/* [MX25xxx] Configure READ/Write sequences */
#define MX25_STATUS_QE                  (0x1 << 6)
#define MX25_SECURITY_WPSEL             (0x1 << 7)
#define MX25_CONFIG_TB                  (0x1 << 3)

#define MX25L32_DEVICE_ID               0x16
#define MX25L128_DEVICE_ID              0x18

/* 
 * Commands
 */
#define FLASH_CMD_WREN		0x06
#define FLASH_CMD_WRDI		0x04
#define FLASH_CMD_RDID		0x9f
#define FLASH_CMD_RDSR		0x05
#define FLASH_CMD_RDSR2		0x35
#define FLASH_CMD_WRSR		0x01
#define FLASH_CMD_SE_4K		0x20
#define FLASH_CMD_SE_32K	0x52
#define FLASH_CMD_SE		0xd8
#define FLASH_CMD_CHIPERASE	0xc7
#define FLASH_CMD_WRVCR		0x81
#define FLASH_CMD_RDVCR		0x85

#define FLASH_CMD_READ		0x03	/* READ */
#define FLASH_CMD_READ_FAST	0x0b	/* FAST READ */
#define FLASH_CMD_READ_1_1_2	0x3b	/* DUAL OUTPUT READ */
#define FLASH_CMD_READ_1_2_2	0xbb	/* DUAL I/O READ */
#define FLASH_CMD_READ_1_1_4	0x6b	/* QUAD OUTPUT READ */
#define FLASH_CMD_READ_1_4_4	0xeb	/* QUAD I/O READ */

#define FLASH_CMD_WRITE		0x02	/* PAGE PROGRAM */
#define FLASH_CMD_WRITE_1_1_2	0xa2	/* DUAL INPUT PROGRAM */
#define FLASH_CMD_WRITE_1_2_2	0xd2	/* DUAL INPUT EXT PROGRAM */
#define FLASH_CMD_WRITE_1_1_4	0x32	/* QUAD INPUT PROGRAM */
#define FLASH_CMD_WRITE_1_4_4	0x12	/* QUAD INPUT EXT PROGRAM */

/* READ commands with 32-bit addressing (N25Q256 and S25FLxxxS) */
#define FLASH_CMD_READ4         0x13
#define FLASH_CMD_READ4_FAST    0x0c
#define FLASH_CMD_READ4_1_1_2   0x3c
#define FLASH_CMD_READ4_1_2_2   0xbc
#define FLASH_CMD_READ4_1_1_4   0x6c
#define FLASH_CMD_READ4_1_4_4   0xec
/* 
 * Status register
 */
#define FLASH_STATUS_BUSY	0x01
#define FLASH_STATUS_WEL	0x02
#define FLASH_STATUS_BP0	0x04
#define FLASH_STATUS_BP1	0x08
#define FLASH_STATUS_BP2	0x10
#define FLASH_STATUS_TB		0x20
#define FLASH_STATUS_SP		0x40
#define FLASH_STATUS_SRWP0	0x80

/* 
 * Capabilities
 */
#define FLASH_CAPS_SINGLE	0x000000ff
#define FLASH_CAPS_READ_WRITE	0x00000001
#define FLASH_CAPS_READ_FAST	0x00000002
#define FLASH_CAPS_SE_4K	0x00000004
#define FLASH_CAPS_SE_32K	0x00000008
#define FLASH_CAPS_CE		0x00000010
#define FLASH_CAPS_32BITADDR	0x00000020
#define FLASH_CAPS_RESET        0x00000040
#define FLASH_CAPS_DUAL		0x0000ff00
#define FLASH_CAPS_READ_1_1_2	0x00000100
#define FLASH_CAPS_READ_1_2_2	0x00000200
#define FLASH_CAPS_READ_2_2_2	0x00000400
#define FLASH_CAPS_WRITE_1_1_2	0x00001000
#define FLASH_CAPS_WRITE_1_2_2	0x00002000
#define FLASH_CAPS_WRITE_2_2_2	0x00004000

#define FLASH_CAPS_QUAD		0x00ff0000
#define FLASH_CAPS_READ_1_1_4	0x00010000
#define FLASH_CAPS_READ_1_4_4	0x00020000
#define FLASH_CAPS_READ_4_4_4	0x00040000
#define FLASH_CAPS_WRITE_1_1_4	0x00100000
#define FLASH_CAPS_WRITE_1_4_4	0x00200000
#define FLASH_CAPS_WRITE_4_4_4	0x00400000

/*
 * FSM SPI Controller Registers
 */
#define SPI_CLOCKDIV			0x0010
#define SPI_MODESELECT			0x0018
#define SPI_CONFIGDATA			0x0020
#define SPI_STA_MODE_CHANGE		0x0028
#define SPI_FAST_SEQ_TRANSFER_SIZE	0x0100
#define SPI_FAST_SEQ_ADD1		0x0104
#define SPI_FAST_SEQ_ADD2		0x0108
#define SPI_FAST_SEQ_ADD_CFG		0x010c
#define SPI_FAST_SEQ_OPC1		0x0110
#define SPI_FAST_SEQ_OPC2		0x0114
#define SPI_FAST_SEQ_OPC3		0x0118
#define SPI_FAST_SEQ_OPC4		0x011c
#define SPI_FAST_SEQ_OPC5		0x0120
#define SPI_MODE_BITS			0x0124
#define SPI_DUMMY_BITS			0x0128
#define SPI_FAST_SEQ_FLASH_STA_DATA	0x012c
#define SPI_FAST_SEQ_1			0x0130
#define SPI_FAST_SEQ_2			0x0134
#define SPI_FAST_SEQ_3			0x0138
#define SPI_FAST_SEQ_4			0x013c
#define SPI_FAST_SEQ_CFG		0x0140
#define SPI_FAST_SEQ_STA		0x0144
#define SPI_QUAD_BOOT_SEQ_INIT_1	0x0148
#define SPI_QUAD_BOOT_SEQ_INIT_2	0x014c
#define SPI_QUAD_BOOT_READ_SEQ_1	0x0150
#define SPI_QUAD_BOOT_READ_SEQ_2	0x0154
#define SPI_PROGRAM_ERASE_TIME		0x0158
#define SPI_MULT_PAGE_REPEAT_SEQ_1	0x015c
#define SPI_MULT_PAGE_REPEAT_SEQ_2	0x0160
#define SPI_STATUS_WR_TIME_REG		0x0164
#define SPI_FAST_SEQ_DATA_REG		0x0300


/*
 * Register: SPI_MODESELECT
 */
#define SPI_MODESELECT_CONTIG		0x01
#define SPI_MODESELECT_FASTREAD		0x02
#define SPI_MODESELECT_DUALIO		0x04
#define SPI_MODESELECT_FSM		0x08
#define SPI_MODESELECT_QUADBOOT		0x10

/*
 * Register: SPI_CONFIGDATA
 */
#define SPI_CFG_DEVICE_ST		0x1
#define SPI_CFG_DEVICE_ATMEL		0x4
#define SPI_CFG_MIN_CS_HIGH(x)		(((x) & 0xfff) << 4)
#define SPI_CFG_CS_SETUPHOLD(x)		(((x) & 0xff) << 16)
#define SPI_CFG_DATA_HOLD(x)		(((x) & 0xff) << 24)

/*
 * Register: SPI_FAST_SEQ_TRANSFER_SIZE
 */
#define TRANSFER_SIZE(x)		((x) * 8)

/*
 * Register: SPI_FAST_SEQ_ADD_CFG
 */
#define ADR_CFG_CYCLES_ADD1(x)		((x) << 0)
#define ADR_CFG_PADS_1_ADD1		(0x0 << 6)
#define ADR_CFG_PADS_2_ADD1		(0x1 << 6)
#define ADR_CFG_PADS_4_ADD1		(0x3 << 6)
#define ADR_CFG_CSDEASSERT_ADD1		(1   << 8)
#define ADR_CFG_CYCLES_ADD2(x)		((x) << (0+16))
#define ADR_CFG_PADS_1_ADD2		(0x0 << (6+16))
#define ADR_CFG_PADS_2_ADD2		(0x1 << (6+16))
#define ADR_CFG_PADS_4_ADD2		(0x3 << (6+16))
#define ADR_CFG_CSDEASSERT_ADD2		(1   << (8+16))

/*
 * Register: SPI_FAST_SEQ_n
 */
#define SEQ_OPC_OPCODE(x)		((x) << 0)
#define SEQ_OPC_CYCLES(x)		((x) << 8)
#define SEQ_OPC_PADS_1			(0x0 << 14)
#define SEQ_OPC_PADS_2			(0x1 << 14)
#define SEQ_OPC_PADS_4			(0x3 << 14)
#define SEQ_OPC_CSDEASSERT		(1   << 16)

/*
 * Register: SPI_FAST_SEQ_CFG
 */
#define SEQ_CFG_STARTSEQ		(1 << 0)
#define SEQ_CFG_SWRESET			(1 << 5)
#define SEQ_CFG_CSDEASSERT		(1 << 6)
#define SEQ_CFG_READNOTWRITE		(1 << 7)
#define SEQ_CFG_ERASE			(1 << 8)
#define SEQ_CFG_PADS_1			(0x0 << 16)
#define SEQ_CFG_PADS_2			(0x1 << 16)
#define SEQ_CFG_PADS_4			(0x3 << 16)

/*
 * Register: SPI_MODE_BITS
 */
#define MODE_DATA(x)			(x & 0xff)
#define MODE_CYCLES(x)			((x & 0x3f) << 16)
#define MODE_PADS_1			(0x0 << 22)
#define MODE_PADS_2			(0x1 << 22)
#define MODE_PADS_4			(0x3 << 22)
#define DUMMY_CSDEASSERT		(1   << 24)

/*
 * Register: SPI_DUMMY_BITS
 */
#define DUMMY_CYCLES(x)			((x & 0x3f) << 16)
#define DUMMY_PADS_1			(0x0 << 22)
#define DUMMY_PADS_2			(0x1 << 22)
#define DUMMY_PADS_4			(0x3 << 22)
#define DUMMY_CSDEASSERT		(1   << 24)

/*
 * Register: SPI_FAST_SEQ_FLASH_STA_DATA
 */
#define STA_DATA_BYTE1(x)		((x & 0xff) << 0)
#define STA_DATA_BYTE2(x)		((x & 0xff) << 8)
#define STA_PADS_1			(0x0 << 16)
#define STA_PADS_2			(0x1 << 16)
#define STA_PADS_4			(0x3 << 16)
#define STA_CSDEASSERT			(0x1 << 20)
#define STA_RDNOTWR			(0x1 << 21)

/*
 * FSM SPI Instruction Opcodes
 */
#define FSM_OPC_CMD			0x1
#define FSM_OPC_ADD			0x2
#define FSM_OPC_STA			0x3
#define FSM_OPC_MODE			0x4
#define FSM_OPC_DUMMY			0x5
#define FSM_OPC_DATA			0x6
#define FSM_OPC_WAIT			0x7
#define FSM_OPC_JUMP			0x8
#define FSM_OPC_GOTO			0x9
#define FSM_OPC_STOP			0xF

/*
 * FSM SPI Instructions (== opcode + operand).
 */
#define FSM_INSTR(cmd, op)		((cmd) | ((op) << 4))

#define FSM_INST_CMD1			FSM_INSTR(FSM_OPC_CMD,	 1)
#define FSM_INST_CMD2			FSM_INSTR(FSM_OPC_CMD,	 2)
#define FSM_INST_CMD3			FSM_INSTR(FSM_OPC_CMD,	 3)
#define FSM_INST_CMD4			FSM_INSTR(FSM_OPC_CMD,	 4)
#define FSM_INST_CMD5			FSM_INSTR(FSM_OPC_CMD,	 5)

#define FSM_INST_ADD1			FSM_INSTR(FSM_OPC_ADD,	 1)
#define FSM_INST_ADD2			FSM_INSTR(FSM_OPC_ADD,	 2)

#define FSM_INST_DATA_WRITE		FSM_INSTR(FSM_OPC_DATA,	 1)
#define FSM_INST_DATA_READ		FSM_INSTR(FSM_OPC_DATA,	 2)

#define FSM_INST_STA_RD1		FSM_INSTR(FSM_OPC_STA, 0x1)
#define FSM_INST_STA_WR1		FSM_INSTR(FSM_OPC_STA, 0x1)
#define FSM_INST_STA_RD2		FSM_INSTR(FSM_OPC_STA, 0x2)
#define FSM_INST_STA_WR1_2		FSM_INSTR(FSM_OPC_STA, 0x3)

#define FSM_INST_MODE			FSM_INSTR(FSM_OPC_MODE,	 0)

#define FSM_INST_DUMMY			FSM_INSTR(FSM_OPC_DUMMY, 0)

#define FSM_INST_WAIT			FSM_INSTR(FSM_OPC_WAIT,	 0)

#define FSM_INST_STOP			FSM_INSTR(FSM_OPC_STOP,	 0)


/* Provide the default configuration if necessary */
const struct seq_rw_config* fsm_get_default_read_configs (void);
const struct seq_rw_config* fsm_get_default_write_configs(void);

static inline uint8_t get_data_pads(struct fsm_seq* seq)
{
	return ((seq->seq_cfg >> 16) & 0x3) + 1;
}

static inline struct stm_spi_fsm *to_fsm(struct spi_flash *flash)
{
	return container_of(flash, struct stm_spi_fsm, flash);
}

/* Internaly exported function */
int fsm_config_rw_seqs		(struct stm_spi_fsm *fsm, uint32_t spi_caps,
				 const struct seq_rw_config *r_confs,
				 const struct seq_rw_config *w_confs);
int fsm_config_rw_seqs_default	(struct stm_spi_fsm *fsm, uint32_t spi_caps);
int fsm_read_status(struct stm_spi_fsm *fsm, uint8_t cmd, uint8_t *data, int bytes);
int fsm_write_status(struct stm_spi_fsm *fsm, uint8_t cmd, uint16_t data, int bytes, int wait_busy);
int fsm_wrvcr			(struct stm_spi_fsm *fsm, uint8_t data);
int fsm_set_mode		(struct stm_spi_fsm *fsm, uint32_t mode);
int fsm_set_freq		(struct stm_spi_fsm *fsm, uint32_t freq);

/* Default interface */
struct spi_flash * spi_fsm_flash_probe( unsigned int bus, unsigned int cs, unsigned int max_hz, 
				    					unsigned int spi_mode);
void spi_fsm_flash_free(struct spi_flash *flash);
int stm_spifsm_read	(struct spi_flash *flash, u32 offset, size_t len, void *buf);
int stm_spifsm_write	(struct spi_flash *flash, u32 offset, size_t len, const void *buf);
int stm_spifsm_erase	(struct spi_flash *flash, u32 offset, size_t len);
extern void fsm_load_seq(struct stm_spi_fsm *fsm, const struct fsm_seq *const seq);
extern unsigned int fsm_fifo_available(struct stm_spi_fsm *fsm);
extern int fsm_is_idle(struct stm_spi_fsm *fsm);
extern unsigned int fsm_fifo_available(struct stm_spi_fsm *fsm);
extern int fsm_wait_seq(struct stm_spi_fsm *fsm);
extern int fsm_enter_32bitaddr(struct stm_spi_fsm *fsm, int enter);
extern int fsm_clear_fifo(struct stm_spi_fsm *fsm);
extern int fsm_read_fifo(struct stm_spi_fsm *fsm, uint32_t *buf, const uint32_t size);
extern int fsm_write_fifo(struct stm_spi_fsm *fsm, const uint32_t *buf, const uint32_t size);
extern uint8_t fsm_wait_busy(struct stm_spi_fsm *fsm);
extern int fsm_read_jedec(struct stm_spi_fsm *fsm, uint8_t *const jedec);
extern int fsm_erase_sector(struct stm_spi_fsm *fsm, const uint32_t offset);
extern int fsm_erase_chip(struct stm_spi_fsm *fsm);
extern int fsm_read_status(struct stm_spi_fsm *fsm, uint8_t cmd, uint8_t *data, int bytes);
extern int fsm_write_status(struct stm_spi_fsm *fsm, uint8_t cmd, uint16_t data, int bytes, int wait_busy);
extern int fsm_wrvcr(struct stm_spi_fsm *fsm, uint8_t data);
extern int fsm_set_mode(struct stm_spi_fsm *fsm, uint32_t mode);
extern int fsm_set_freq(struct stm_spi_fsm *fsm, uint32_t freq);
extern int fsm_read(struct stm_spi_fsm *fsm, uint8_t *const buf, const uint32_t size, const uint32_t offset);
extern int fsm_write(struct stm_spi_fsm *fsm, const uint8_t *const buf,
		     const size_t size, const uint32_t offset);
extern int fsm_reset(struct stm_spi_fsm *fsm);
extern int fsm_search_configure_rw_seq(struct fsm_seq *seq, const struct seq_rw_config *configs,
				       					uint32_t capabilities);
extern int fsm_config_rw_seqs(struct stm_spi_fsm *fsm, uint32_t spi_caps, 
				const struct seq_rw_config *r_confs, const struct seq_rw_config *w_confs);
extern int fsm_config_rw_seqs_default(struct stm_spi_fsm *fsm, uint32_t spi_caps);
extern int configure_erasesec_seq(struct fsm_seq *seq, int use_32bit_addr);
extern void fsm_enter_quad_mode(struct stm_spi_fsm *fsm, int enter);

/* Manufacturer Probe */
int spi_flash_probe_spansion(struct stm_spi_fsm* fsm, uint8_t *idcode);
int spi_flash_probe_stmicro (struct stm_spi_fsm *fsm, uint8_t *idcode);
int spi_flash_probe_macronix(struct stm_spi_fsm *fsm, uint8_t *idcode);

#endif /* STM_SPI_FSM_H */
