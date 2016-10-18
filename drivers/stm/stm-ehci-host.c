/*
 * (C) Copyright 2008, Michael Trimarchi <trimarchimichael@yahoo.it>
 * Copyright (C) 2012-2014 STMicroelectronics Limited
 *	Sean McGoogan <Sean.McGoogan@st.com>
 * Copyright (C) 2015 STMicroelectronics Limited
 *	Imran Khan <imran.khan@st.com>
 *	Added Driver Model(DM) support. 
 *
 * Author: Michael Trimarchi <trimarchimichael@yahoo.it>
 * This code is based on ehci freescale driver, updated specifically
 * for the STMicroelectronics' EHCI controller, as found on ST40 SoCs.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <usb.h>
#include <stm/soc.h>
#include <../drivers/usb/host/ehci.h>
#if defined(CONFIG_DM) && defined(CONFIG_DM_USB)
#include <dm.h>
#include <dm/platform_data/ehci_stm.h>
#endif
#include <stm/stbus.h>


#define unlikely(x)		__builtin_expect(!!(x), 0)

#define STM_HC_LENGTH		0x10	/* should always be 16 for ST's SoCs */

#ifndef CONFIG_DM_USB
u32 get_st_ehci_base(int index)
{
	switch(index)
	{
		case 0:
			return (CONFIG_SYS_USB2_0_BASE + AHB2STBUS_EHCI_BASE_OFFSET);
		case 1:
			return (CONFIG_SYS_USB2_1_BASE + AHB2STBUS_EHCI_BASE_OFFSET);
		default:
			printf("Index out of range for the USB controller ... \n");
			return -1;
	}
}

/*
 * Create the appropriate control structures to manage
 * a new EHCI host controller.
 */
extern int ehci_hcd_init(
	const int index, enum usb_init_type init,
	struct ehci_hccr ** const hccr,
	struct ehci_hcor ** const hcor)
{
	uint32_t hc_length;
	uint32_t base_addr;

	/*
	 * First, we call the board- and CPU-specific initialization functions,
	 * which will enable the controller, and configure all the USB PIOs.
	 * We will also call the function STM_start_host_control(),
	 * which configures the AMBA interface for the IP block.
	 */
	usb_cpu_init();

	/*
	 * Save address for both the HCCR and HCOR registers in the EHCI
	 * H/W controller, into the 2 indirect-pointers "hccr", and "hcor".
	 * NOTE: this limits us to only ONE host-controller (QQQ: allow 1+)
	 */
	base_addr = get_st_ehci_base(index);
	if(base_addr == -1)
		return 1;
	*hccr = (struct ehci_hccr *)base_addr;

	hc_length = HC_LENGTH(ehci_readl(&(*hccr)->cr_capbase));
	BUG_ON(hc_length != STM_HC_LENGTH);

	*hcor = (struct ehci_hcor *)((uint32_t)*hccr + STM_HC_LENGTH);

#if 1
	printf("STM EHCI HCD initialization HCCR=0x%08x, HCOR=0x%08x, hc_length=%u\n",
		(uint32_t)*hccr, (uint32_t)*hcor, hc_length);
#endif

	return 0;	/* indicate success */
}


/*
 * Properly shutdown the EHCI host controller (HC).
 * This is a mandatory requirement before we can safely transition
 * from U-Boot to any OS (e.g. linux) … otherwise madness ensues!
 */
extern int ehci_hcd_stop(const int index)
{
	u32 base_addr = get_st_ehci_base(index);
	if(base_addr == -1 )
		return 1;
	struct ehci_hccr * const hccr =
		(struct ehci_hccr *)base_addr;
	struct ehci_hcor * const hcor =
		(struct ehci_hcor *)((uint32_t)hccr + STM_HC_LENGTH);
	uint32_t reg;
	const int numPorts = HCS_N_PORTS(ehci_readl(&hccr->cr_hcsparams));
	int port;

	/*
	 * Complete any current transactions, and HALT the HC.
	 */
	reg = ehci_readl(&hcor->or_usbcmd);
	reg &= ~(CMD_RUN);
	ehci_writel(&hcor->or_usbcmd, reg);

	/*
	 * Wait for the HC to be halted.
	 * This can take up to 16 micro-frames (i.e. 2ms).
	 */
	while (!(ehci_readl(&hcor->or_usbsts) & STS_HALT))
		;	/* wait until STS_HALT is asserted */

	/*
	 * power-down all the root ports.
	 */
	for (port=0; port<numPorts; port++) {
		ehci_writel(&hcor->or_portsc[port],
			EHCI_PS_OCC | EHCI_PS_PEC | EHCI_PS_CSC);
	}

	/*
	 * Finally, reset the controller (just in case)!
	 */
	reg = ehci_readl(&hcor->or_usbcmd);
	reg |= CMD_RESET;
	ehci_writel(&hcor->or_usbcmd, reg);

	/*
	 * … and wait for the reset to self-clear…
	 */
	while ((ehci_readl(&hcor->or_usbcmd) & CMD_RESET))
		;	/* wait until CMD_RESET is de-asserted */

	/* Call the board and CPU specific stop function */
	usb_cpu_stop();
	
	return 0;
}
#else
/**
 ** Contains pointers to register base addresses
 ** for the usb controller.
 **/

struct stm_ehci {
	struct ehci_ctrl ctrl;
	struct ehci_hccr *hccr;
	struct ehci_hcor *hcor;
};

static int stm_ehci_usb_probe(struct udevice *dev)
{
	uint32_t hc_length;

	struct stm_ehci_platdata *plat = dev_get_platdata(dev);
	struct stm_ehci *ctx = dev_get_priv(dev);


	/* Board and CPU specific initialization */
	usb_cpu_init();

	/* Populate EHCI info and register to EHCI framework */
	ctx->hccr = (struct ehci_hccr *)(plat->hcd_base + AHB2STBUS_EHCI_BASE_OFFSET);

	hc_length = HC_LENGTH(ehci_readl(&(ctx->hccr)->cr_capbase));
	BUG_ON(hc_length != STM_HC_LENGTH);

	ctx->hcor = (struct ehci_hcor *)((uint32_t)(ctx->hccr) + STM_HC_LENGTH);

	printf("STM EHCI HCD initialization HCCR=0x%08x, HCOR=0x%08x, hc_length=%u\n",
				(uint32_t)(ctx->hccr), (uint32_t)(ctx->hcor), hc_length);

	return ehci_register(dev, ctx->hccr, ctx->hcor, NULL, 0, USB_INIT_HOST);
}

static int stm_ehci_usb_remove(struct udevice *dev)
{
	struct stm_ehci *ctx = dev_get_priv(dev);
	int ret;
	uint32_t reg;
	struct ehci_hccr * const hccr =	ctx->hccr;
	struct ehci_hcor * const hcor = ctx->hcor;
	const int numPorts = HCS_N_PORTS(ehci_readl(&hccr->cr_hcsparams));
	int port;

	ret = ehci_deregister(dev);
	if (ret)
		return ret;

	/*
	 * Complete any current transactions, and HALT the HC.
	 */
	reg = ehci_readl(&hcor->or_usbcmd);
	reg &= ~(CMD_RUN);
	ehci_writel(&hcor->or_usbcmd, reg);

	/*
	 * Wait for the HC to be halted.
	 * This can take up to 16 micro-frames (i.e. 2ms).
	 */
	while (!(ehci_readl(&hcor->or_usbsts) & STS_HALT))
		;	/* wait until STS_HALT is asserted */

	/*
	 * power-down all the root ports.
	 */
	for (port=0; port<numPorts; port++) {
		ehci_writel(&hcor->or_portsc[port],
			EHCI_PS_OCC | EHCI_PS_PEC | EHCI_PS_CSC);
	}

	/*
	 * Finally, reset the controller (just in case)!
	 */
	reg = ehci_readl(&hcor->or_usbcmd);
	reg |= CMD_RESET;
	ehci_writel(&hcor->or_usbcmd, reg);

	/*
	 * … and wait for the reset to self-clear…
	 */
	while ((ehci_readl(&hcor->or_usbcmd) & CMD_RESET))
		;	/* wait until CMD_RESET is de-asserted */

	/* Call the board and CPU specific stop function */
	usb_cpu_stop();

	return 0;
}

U_BOOT_DRIVER(usb_ehci) = {
	.name   = "ehci_stm",
	.id     = UCLASS_USB,
	.probe = stm_ehci_usb_probe,
	.remove = stm_ehci_usb_remove,
	.ops    = &ehci_usb_ops,
	.priv_auto_alloc_size = sizeof(struct stm_ehci),
	.platdata_auto_alloc_size = sizeof(struct stm_ehci_platdata),
	.flags  = DM_FLAG_ALLOC_PRIV_DMA,
};
#endif

