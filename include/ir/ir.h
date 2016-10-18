/*
 * Copyright (c) 2014, STMicroelectronics
 *	Imran Khan <imran.khan@st.com>
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#define LOG_ERR		1
#define LOG_NOTICE	2

#define PULSE_BIT       0x01000000
#define PULSE_MASK      0x00FFFFFF

#define LIRC_MODE2_SPACE     0x00000000
#define LIRC_MODE2_PULSE     0x01000000
#define LIRC_MODE2_FREQUENCY 0x02000000
#define LIRC_MODE2_TIMEOUT   0x03000000

#define LIRC_VALUE_MASK      0x00FFFFFF
#define LIRC_MODE2_MASK      0xFF000000

#define LIRC_SPACE(val) (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_SPACE)
#define LIRC_PULSE(val) (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_PULSE)
#define LIRC_FREQUENCY(val) (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_FREQUENCY)
#define LIRC_TIMEOUT(val) (((val)&LIRC_VALUE_MASK) | LIRC_MODE2_TIMEOUT)

#define LIRC_VALUE(val) ((val)&LIRC_VALUE_MASK)
#define LIRC_MODE2(val) ((val)&LIRC_MODE2_MASK)

#define LIRC_IS_SPACE(val) (LIRC_MODE2(val) == LIRC_MODE2_SPACE)
#define LIRC_IS_PULSE(val) (LIRC_MODE2(val) == LIRC_MODE2_PULSE)
#define LIRC_IS_FREQUENCY(val) (LIRC_MODE2(val) == LIRC_MODE2_FREQUENCY)
#define LIRC_IS_TIMEOUT(val) (LIRC_MODE2(val) == LIRC_MODE2_TIMEOUT)

#define LIRC_MODE2SEND(x) (x)
#define LIRC_SEND2MODE(x) (x)
#define LIRC_MODE2REC(x) ((x) << 16)
#define LIRC_REC2MODE(x) ((x) >> 16)

#define LIRC_MODE_RAW                  0x00000001
#define LIRC_MODE_PULSE                0x00000002
#define LIRC_MODE_MODE2                0x00000004
#define LIRC_MODE_LIRCCODE             0x00000010
#define LIRC_CAN_SEND_RAW              LIRC_MODE2SEND(LIRC_MODE_RAW)
#define LIRC_CAN_SEND_PULSE            LIRC_MODE2SEND(LIRC_MODE_PULSE)
#define LIRC_CAN_SEND_MODE2            LIRC_MODE2SEND(LIRC_MODE_MODE2)
#define LIRC_CAN_SEND_LIRCCODE         LIRC_MODE2SEND(LIRC_MODE_LIRCCODE)
#define LIRC_CAN_REC_RAW               LIRC_MODE2REC(LIRC_MODE_RAW)
#define LIRC_CAN_REC_PULSE             LIRC_MODE2REC(LIRC_MODE_PULSE)
#define LIRC_CAN_REC_MODE2             LIRC_MODE2REC(LIRC_MODE_MODE2)
#define LIRC_CAN_REC_LIRCCODE          LIRC_MODE2REC(LIRC_MODE_LIRCCODE)

#define LIRC_CAN_SEND_MASK             0x0000003f
#define LIRC_CAN_REC_MASK              LIRC_MODE2REC(LIRC_CAN_SEND_MASK)

#define LIRC_CAN_SEND(x) ((x)&LIRC_CAN_SEND_MASK)
#define LIRC_CAN_REC(x) ((x)&LIRC_CAN_REC_MASK)

#define LIRC_RELEASE_SUFFIX     "_UP"

#define TO_US(duration)                 DIV_ROUND_CLOSEST((duration), 1000)
#define TO_STR(is_pulse)                ((is_pulse) ? "pulse" : "space")


#ifndef true
typedef enum {false,true} bool;
#endif

#define lirc_t int

struct lirc_codec {
	int carrier_low;
	u64 gap_start;
	u64 gap_duration;
	bool gap;
	bool send_timeout_reports;
};

struct ir_device;

/*
 *  * From rc-raw.c
 *   * The Raw interface is specific to InfraRed. It may be a good idea to
 *    * split it later into a separate header.
 *     */

enum raw_event_type {
        IR_SPACE        = (1 << 0),
        IR_PULSE        = (1 << 1),
        IR_START_EVENT  = (1 << 2),
        IR_STOP_EVENT   = (1 << 3),
};

struct ir_raw_event {
        union {
                u32             duration;

                struct {
                        u32     carrier;
                        u8      duty_cycle;
                };
        };

        unsigned                pulse:1;
        unsigned                reset:1;
        unsigned                timeout:1;
        unsigned                carrier_report:1;
};

#define DEFINE_IR_RAW_EVENT(event) \
        struct ir_raw_event event = { \
                { .duration = 0 } , \
                .pulse = 0, \
                .reset = 0, \
                .timeout = 0, \
                .carrier_report = 0 }

static inline void init_ir_raw_event(struct ir_raw_event *ev)
{
        memset(ev, 0, sizeof(*ev));
}

#define IR_FIFO_SIZE 10240

struct ir_fifo {
	char *base;
	char *head, *tail;
	int len;
};

#define IR_MAX_DURATION         0xFFFFFFFF      /* a bit more than 4 seconds */
#define US_TO_NS(usec)          ((usec) * 1000)
#define MS_TO_US(msec)          ((msec) * 1000)
#define MS_TO_NS(msec)          ((msec) * 1000 * 1000)

struct ir_device {
	struct ir_fifo event_fifo;
	struct ir_fifo sample_fifo;
	struct lirc_codec lirc;
	int (*open)(struct ir_device *); /* Pointer to low level tstc function. */
	int (*close)(struct ir_device *); /* Pointer to low level tstc function. */
	int (*tstc)(struct ir_device *); /* Pointer to low level tstc function. */
	int (*getc)(struct ir_device *); /* Pointer to low level getc function. */
	void *priv; /* Low level driver specific data */
};
int ir_lowlevel_init(struct ir_device *ir_dev);


struct ir_device* ir_init(void);
char* decode_key(struct ir_device *dev);
int stm_irb_init(struct ir_device *gen_ir_dev);
int default_ir_init(void);
