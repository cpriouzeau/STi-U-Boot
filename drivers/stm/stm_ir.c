/*
* Driver for the STMicroelectronics IRDA devices
*
* Much of this work has been taken from LIRC driver, 
* for STMicroelectronics' IRDA devices, available in
* kernel.
*
* Copyright (C) 2004-2014 STMicroelectronics
* 	Giuseppe Cavallaro  <peppe.cavallaro@st.com>
*	Carl Shaw <carl.shaw@st.com>
*	Angelo Castello <angelo.castello@st.com>
*       Francesco Virlinzi <francesco.virlinzi@st.com>
*       Srinivas Kandagatla<srinivas.kandagatla@st.com>
* 
* Adapted for U-Boot:
* 	Imran Khan <imran.khan@st.com>
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

#include <common.h>
#include <command.h>
#include <stm/soc.h>
#include <stm/socregs.h>
#include <asm/io.h>
#include <malloc.h>
#include <stdio_dev.h>
#include <asm/byteorder.h>
#include <stm/clk.h>
#include <ir/ir.h>
#include "stm_ir.h"



#define IR_STM_NAME "lirc-stm"


/* General debugging */
#ifdef CONFIG_LIRC_STM_DEBUG
#define DPRINTK(fmt, args...) \
	pr_debug(IR_STM_NAME ": %s: " fmt, __func__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

/*
 * LiRC data
 */

#define RX_CLEAR_IRQ(dev, x)	writel((x), dev->rx_base + IRB_RX_INT_CLEAR)

#define RX_WORDS_IN_FIFO_OR_OVERRUN(dev)	\
			(readl(dev->rx_base + IRB_RX_STATUS) & 0xff04)
#define RX_WORDS_IN_FIFO(dev)	\
			(readl(dev->rx_base + IRB_RX_STATUS) & 0xff00)
#define RX_INT_PENDING(dev)	\
			(readl(dev->rx_base + IRB_RX_INT_STATUS) & 0x01)
#define RX_FIFO_FULL_OR_OVERRUN(dev)	\
			(readl(dev->rx_base + IRB_RX_STATUS) & 0x0704)


extern int handle_raw_event(struct ir_device *dev);
extern int enqueue_event(struct ir_device *dev, struct ir_raw_event *ev);

struct stm_plat_lirc_data stm_ir_pdata = {
	.irbclock	 = 0, /* use current_cpu data */
	.irbclkdiv	 = 0, /* automatically calculate */
	.irbperiodmult	 = 0,
	.irbperioddiv	 = 0,
	.irbontimemult	 = 0,
	.irbontimediv	 = 0,
	.irbrxmaxperiod  = 0x5000,
	.sysclkdiv	 = 1,
	.rxpolarity	 = 1,
	.subcarrwidth    = 0,
	.rxnoisesupwidth = 0,
	.rxuhfmode       = 0,
	.txenabled       = 0,
};

#if 0
static void ir_stm_tx_interrupt(int irq, void *data)
{
		return; /* NA */
}
#endif
static void ir_stm_tx_calc_clocks(struct stm_ir_device *dev,
				unsigned int clockfreq,
				unsigned int subwidthpercent)
{
		return; /* NA */
}

static int ir_stm_scd_set(struct stm_ir_device *dev, char enable)
{
	return 0; /* NA */
}
static void ir_stm_scd_reactivate(struct stm_ir_device *dev, int lastSymbol)
{
	return;	/* NA */
}
static void ir_stm_scd_set_flags(struct stm_ir_device *dev)
{
	return;	/* NA */
}

#ifdef CONFIG_PM
static void ir_stm_scd_restart(struct stm_ir_device *dev)
{
	return;	/* NA */
}
#endif /* CONFIG_PM */

static void ir_stm_scd_prefix_symbols(struct stm_ir_device *dev,
				struct ir_raw_event *first_ev)
{
	return;	/* NA */
}
static int ir_stm_scd_config(struct stm_ir_device *dev, unsigned long clk)
{
	return 0; /* NA */
}

static int ir_stm_tstc(struct ir_device *gen_ir_dev)
{
	struct stm_ir_device *dev = (struct stm_ir_device *)gen_ir_dev->priv;
	//if(RX_INT_PENDING(dev)){
	if(RX_WORDS_IN_FIFO_OR_OVERRUN(dev)){
		return true;
	} else {
		return false;
	}
}

#define RETRY_COUNT 10
static int ir_stm_getc(struct ir_device *gen_ir_dev)
{
	unsigned int 		symbol = 0;
	unsigned int		 mark = 0;
	struct stm_ir_device 	*dev = (struct stm_ir_device *)gen_ir_dev->priv;
	int 			lastSymbol = 0;
	int			retry     = RETRY_COUNT;
	int 			total 	  = 0;
        unsigned int    	symbol_arry[50];
        unsigned int    	mark_arry[50];
	static int 		start_sync;
	DEFINE_IR_RAW_EVENT(ev);

#if 1
	//printf("#### %s(): %d ##### \n", __func__, __LINE__);
	if (!start_sync) {
		/* for LIRC_MODE_MODE2 or LIRC_MODE_PULSE or LIRC_MODE_RAW
		 * lircd expects a long space first before a signal train
		 * to sync. */
		//printf("#### %s(): %d ##### \n", __func__, __LINE__);
		DEFINE_IR_RAW_EVENT(start_ev);
		start_ev.timeout = true;
		start_ev.pulse = false;
		start_sync = 1;
		enqueue_event(gen_ir_dev, &start_ev);
	}
	ir_stm_scd_set_flags(dev);
#endif
        while (1) {
		int 		index     = 0;
		int 		index1    = 0;
		int 		num_words = 0;
		unsigned int 	status    = 0;
		int 		clear_irq = 1;

		status = RX_WORDS_IN_FIFO(dev);
		if(!status)	//Check if any word present in IR Receiver FIFO
		{
		   // Check for any pulse in present in driver fifo
		   // If present process them, before sleeping to wait for key
		   if(total > 0)
		   {
			index = total;
                        total = 0;
                        goto ir_rocess_event;
		   }
 
		   if(retry == 0)
		      break;
                   else
		   {
		      mdelay(5);	
		      retry--;
		      continue;
		   }
		}
	        retry = RETRY_COUNT;
		status = readl(dev->rx_base + IRB_RX_STATUS);
		num_words = status & 0x00000F00;
		num_words = num_words >> 8;

		/* discard the entire collection in case of errors!  */
		if (unlikely(readl(dev->rx_base + IRB_RX_INT_STATUS) &
					LIRC_STM_IS_OVERRUN)) {
			writel(LIRC_STM_CLEAR_OVERRUN,dev->rx_base + IRB_RX_INT_CLEAR);
			DPRINTK("Rx FIFO overrun!!\n",);
		}

#if 1
                if (clear_irq) {
                        /*  Clear the interrupt
                         * and leave only the overrun irq enabled */
                        RX_CLEAR_IRQ(dev, LIRC_STM_CLEAR_IRQ);
                        writel(0x07, dev->rx_base + IRB_RX_INT_EN);
                        clear_irq = 0;
                }
#endif

		index = 0;
		lastSymbol = 0;
		while(index != num_words)
		{
		  symbol = (readl(dev->rx_base + IRB_RX_SYS));
                  mark   = (readl(dev->rx_base + IRB_RX_ON));
                  
		  symbol_arry[index+total] = symbol;
                  mark_arry[index+total]   = mark;
		  index++;

                  if (symbol == 0xFFFF){
		      lastSymbol=1;
                  }
                  else
                     lastSymbol = 0;
		}
		total = total + index;
		if(lastSymbol)
		  index = total;
		else
		  continue; 	 //Keep Storing Key

ir_rocess_event:
		index1 = 0;
		lastSymbol = 0;
		while(index1 != index)
		{
		     mark   = mark_arry[index1];
		     symbol = symbol_arry[index1];
		     if (symbol == 0xFFFF){
                         lastSymbol = 1;
                     }
		     else
			lastSymbol = 0;

                     if ((mark > 2) && (symbol > 1)) {
                          /* Make fine adjustments to timings */
                        symbol -= mark; /* to get space timing */
                        symbol *= dev->rx_symbol_mult;
                        symbol /= dev->rx_symbol_div;
                        mark *= dev->rx_pulse_mult;
                        mark /= dev->rx_pulse_div;
	
			ev.duration = US_TO_NS(mark);
                        ev.pulse = true;
                        if (dev->insert_scd_timing) {
                                ir_stm_scd_prefix_symbols(dev, &ev);
                                dev->insert_scd_timing = false;
                        }
                        enqueue_event(gen_ir_dev, &ev);
                        ev.duration = US_TO_NS(symbol);
                        ev.pulse = false;
                        ev.timeout = lastSymbol ? true : false;
                        enqueue_event(gen_ir_dev, &ev);
                        DPRINTK("PULSE : %d SPACE %d \n", mark, symbol);

		     }

                     if (lastSymbol == 1){
			 handle_raw_event(gen_ir_dev);
                  	 dev->insert_scd_timing = true;
                     }	
		     index1++;
		}
		num_words = 0;
		total     = 0;
	}/* while */

	RX_CLEAR_IRQ(dev, LIRC_STM_CLEAR_IRQ | 0x02);
	writel(LIRC_STM_ENABLE_IRQ, dev->rx_base + IRB_RX_INT_EN);
	ir_stm_scd_reactivate(dev, lastSymbol);
	return 0;
}

static void ir_stm_rx_flush(struct stm_ir_device *dev)
{
	ir_stm_scd_set(dev, 0);
	/* Disable receiver */
	writel(0x00, dev->rx_base + IRB_RX_EN);
	/* Disable interrupt */
	writel(0x20, dev->rx_base + IRB_RX_INT_EN);
	/* clean the buffer */
}

static void
ir_stm_rx_calc_clocks(struct stm_ir_device *dev, unsigned long baseclock)
{
	struct stm_plat_lirc_data *data = &stm_ir_pdata; /*TODO: Imran get rid of data pointer here
						  use global structure directly.*/
	unsigned int rx_max_symbol_per;


	if (data->irbclkdiv == 0) {
		/* Auto-calculate clock divisor */
		int freqdiff;
		dev->rx_sampling_freq_div = baseclock / 10000000;
		/* Work out the timing adjustment factors */
		freqdiff = baseclock - (dev->rx_sampling_freq_div * 10000000);
		/* freqdiff contains the difference between our clock and a
		 * true 10 MHz clock which the IR block wants
		 */
		if (freqdiff == 0) {
			/* no adjustment required - our clock is running at the
			 * required speed
			 */
			dev->rx_symbol_mult = 1;
			dev->rx_pulse_mult = 1;
			dev->rx_symbol_div = 1;
			dev->rx_pulse_div = 1;
		} else {
			/* adjustment is required */
			dev->rx_symbol_mult =
			    baseclock / (10000 * dev->rx_sampling_freq_div);

			if (freqdiff > 0) {
				/* our clock is running too fast */
				dev->rx_pulse_mult = 1000;
				dev->rx_pulse_div = dev->rx_symbol_mult;
				dev->rx_symbol_mult = dev->rx_pulse_mult;
				dev->rx_symbol_div = dev->rx_pulse_div;
			} else {
				/* our clock is running too slow */
				dev->rx_symbol_div = 1000;
				dev->rx_pulse_mult = dev->rx_symbol_mult;
				dev->rx_pulse_div = 1000;
			}

		}

	} else {
		dev->rx_sampling_freq_div = (data->irbclkdiv);
		dev->rx_symbol_mult = (data->irbperiodmult);
		dev->rx_symbol_div = (data->irbperioddiv);
	}

	writel(dev->rx_sampling_freq_div, dev->base + IRB_SAMPLE_RATE_COMM);
	/* maximum symbol period.
	 * Symbol periods longer than this will generate
	 * an interrupt and terminate a command
	 */
	if ((data->irbrxmaxperiod) != 0)
		rx_max_symbol_per =
		    (data->irbrxmaxperiod) *
		    dev->rx_symbol_mult / dev->rx_symbol_div;
	else
		rx_max_symbol_per = 0;

	writel(rx_max_symbol_per, dev->rx_base + IRB_MAX_SYM_PERIOD);
}

static int ir_stm_hardware_init(struct stm_ir_device *dev)
{
	unsigned int scwidth;
	int baseclock;
	struct stm_plat_lirc_data *data = dev->pdata;


	/* Set the polarity inversion bit to the correct state */
	writel(data->rxpolarity, dev->rx_base + IRB_RX_POLARITY_INV);

	/* Set the Rx noise suppression width */
	writel(data->rxnoisesupwidth, dev->rx_base + IRB_RX_NOISE_SUPPR);

	/*  Get or calculate the clock and timing adjustment values.
	 *  We can auto-calculate these in some cases
	 */
	baseclock = stm_get_sbc_comm_clk_rate() / data->sysclkdiv;

	ir_stm_rx_calc_clocks(dev, baseclock);
	/*  Set up the transmit timings  */
	if (data->subcarrwidth != 0)
		scwidth = data->subcarrwidth;
	else
		scwidth = 50;

	if (scwidth > 100)
		scwidth = 50;

	ir_stm_tx_calc_clocks(dev, baseclock, scwidth);

	ir_stm_scd_config(dev, baseclock);

	return 0;
}

/* outside interface: called on first open*/
static int stm_ir_open(struct ir_device *gen_ir_dev)
{
	struct stm_ir_device *dev = (struct stm_ir_device *)gen_ir_dev->priv;
//	if (!dev->enabled) {
		//unsigned long flags;
		
		/* enable interrupts and receiver */
		writel(LIRC_STM_ENABLE_IRQ, dev->rx_base + IRB_RX_INT_EN);		

		/* Amitabh.dutta@st.com
		 * Clean FIFO, required for some boards
		 * Which contains some garbage value in there fifo
                 * on IR sontroller start.
                 */
		//printf("Fifo clean loop...");
		while(RX_WORDS_IN_FIFO(dev))
		{
                  readl(dev->rx_base + IRB_RX_SYS);
                  readl(dev->rx_base + IRB_RX_ON);
		}

		/* Enable IR Recevier code processor */
		writel(0x01, dev->rx_base + IRB_RX_EN);
//		dev->enabled = true;

		ir_stm_scd_set(dev, 1);
		dev->insert_scd_timing = true;
//	} else
//		DPRINTK("Device Already Enabled\n");

	return 0;
}

#if 1
/* outside interface: called on device close*/
static int stm_ir_close(struct ir_device *gen_ir_dev)
{
	struct stm_ir_device *dev = (struct stm_ir_device *)gen_ir_dev->priv;
	/* The last close disable the receiver */
//	if (dev->enabled) {
		ir_stm_rx_flush(dev);
//	        dev->enabled = false;
//	}
	return 0;
}
#endif

#if defined(CONFIG_STM_IR)
extern int stm_irb_init(struct ir_device *gen_ir_dev)
{
	int ret = -1;
	struct stm_ir_device *ir_dev;
	struct stm_plat_lirc_data *pdata;

	pdata = &stm_ir_pdata; /*TODO: Imran avoid use of pdata pointer */

	ir_dev = malloc(sizeof(struct stm_ir_device));
	if(ir_dev == NULL){
		return ret;
	}
	ir_dev->pdata = pdata;

	ir_dev->base = CONFIG_SYS_STM_IRB_BASE;
	/* Configure for ir or uhf. rxuhfmode==1 is UHF */
	if (ir_dev->pdata->rxuhfmode)
		ir_dev->rx_base = ir_dev->base + 0x40;
	else
		ir_dev->rx_base = ir_dev->base;

	/* Configure relevant sysconf registers */
	irb_init();

	/* enable signal detection */
	ret = ir_stm_hardware_init(ir_dev);

	if(ret < 0)
		return ret;
	
	/* Assign the handler that will be used for input */
	gen_ir_dev->open = &stm_ir_open; 
	gen_ir_dev->close = &stm_ir_close; 
	gen_ir_dev->getc = &ir_stm_getc; 
	gen_ir_dev->tstc = &ir_stm_tstc; 
	gen_ir_dev->priv = ir_dev; 

	/* For U-boot call open, just after device has been
 	 * initialized for there is no LIRC framework. */
	stm_ir_open(gen_ir_dev);
	
	return 0;	
}
#endif


