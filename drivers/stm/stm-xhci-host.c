/*
 * Copyright (C) 2012 Samsung Electronics Co.Ltd
 *      Vivek Gautam <gautam.vivek@samsung.com>
 *      Vikas Sajjan <vikas.sajjan@samsung.com>
 *  Copyright (C) 2014 STMicroelectronics Limited
 *	Sachin Verma <sachin.verma@st.com>
 *
 * Copyright (C) 2015 STMicroelectronics Limited
 *	Imran Khan <imran.khan@st.com>
 *	Added Driver Model(DM) support. 
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 */

#include <common.h>
#include <malloc.h>
#include <usb.h>
#include <watchdog.h>
#include <asm-generic/errno.h>
#include <linux/compat.h>
#include <linux/usb/dwc3.h>

#include <../drivers/usb/host/xhci.h>
#include <dm.h>
#include <dm/platform_data/xhci_stm.h>
#include <stm/stbus.h>

/* Declare global data pointer */
//DECLARE_GLOBAL_DATA_PTR;

/**
 * Contains pointers to register base addresses
 * for the usb controller.
 */
struct stm_xhci {
#ifdef CONFIG_DM_USB
	struct usb_platdata usb_plat;
	struct xhci_ctrl ctrl;
#endif
	struct xhci_hccr *hcd;
	struct dwc3 *dwc3_reg;
};

#ifndef CONFIG_DM_USB
static struct stm_xhci stmxhci;
#endif

static int stm_xhci_core_init(struct stm_xhci *stmxhci)
{
    int ret;
    ret = dwc3_core_init(stmxhci->dwc3_reg);
    if (ret) {
        printf("stm_xhci_core_init:: failed to initialize core\n");
        return -EINVAL;
    }
    /* We are hard-coding DWC3 core to Host Mode */
    dwc3_set_mode(stmxhci->dwc3_reg, DWC3_GCTL_PRTCAP_HOST);
    return 0;
}

#ifndef CONFIG_DM_USB
int xhci_hcd_init(int index, struct xhci_hccr **hccr, struct xhci_hcor **hcor)
{
	struct stm_xhci *ctx = &stmxhci;

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
	ctx->hcd = (struct xhci_hccr *)STM_DWC3_CORE_BASE;
	ctx->dwc3_reg = (struct dwc3 *)((char *)(ctx->hcd) + DWC3_REG_OFFSET);

	stm_xhci_core_init(ctx);

    	*hccr = (ctx->hcd);
    	*hcor = (struct xhci_hcor *)(uintptr_t)((uint32_t)*hccr
                + HC_LENGTH(xhci_readl(&(*hccr)->cr_capbase)));

    	return 0;
}

void xhci_hcd_stop(int index)
{
	usb_cpu_stop();
	return;
}
#endif

#ifdef CONFIG_DM_USB
static int xhci_usb_probe(struct udevice *dev)
{
	struct stm_xhci_platdata *plat = dev_get_platdata(dev);
	struct stm_xhci *ctx = dev_get_priv(dev);
	struct xhci_hcor *hcor;

	ctx->hcd = (struct xhci_hccr *)plat->hcd_base;
	ctx->dwc3_reg = (struct dwc3 *)((char *)(ctx->hcd) + DWC3_REG_OFFSET);

	usb_cpu_init();
	stm_xhci_core_init(ctx);

	hcor = (struct xhci_hcor *)((uint32_t)ctx->hcd +
		HC_LENGTH(xhci_readl(&ctx->hcd->cr_capbase)));

	return xhci_register(dev, ctx->hcd, hcor);
}

static int xhci_usb_remove(struct udevice *dev)
{
        int ret;

        ret = xhci_deregister(dev);
        if (ret)
                return ret;

	usb_cpu_stop();
        return 0;
}

U_BOOT_DRIVER(usb_xhci) = {
        .name   = "xhci_stm",
        .id     = UCLASS_USB,
        .probe = xhci_usb_probe,
        .remove = xhci_usb_remove,
        .ops    = &xhci_usb_ops,
        .platdata_auto_alloc_size = sizeof(struct stm_xhci_platdata),
        .priv_auto_alloc_size = sizeof(struct stm_xhci),
	.flags  = DM_FLAG_ALLOC_PRIV_DMA,
};
#endif

