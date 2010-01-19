/*
 * include/linux/spi/adf702x.h
 *
 * Device characteristics are highly application specific
 * and may vary between boards and models. The platform_data for the
 * device's "struct device" holds this information.
 *
 * Copyright 2009-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __LINUX_SPI_ADF702X_H__
#define __LINUX_SPI_ADF702X_H__

#include <linux/if_ether.h>

#define MODEL_ADF7021	7021
#define MODEL_ADF7025	7025

struct adf702x_platform_data {
	/* Base reg base of SPORT controller */
	void __iomem *regs_base;
	unsigned dma_ch_rx;
	unsigned dma_ch_tx;
	unsigned irq_sport_err;
	unsigned gpio_int_rfs;
	u16 pin_req[7];
	u32 adf702x_model;
	const u32 *adf702x_regs;
	u32 tx_reg;
	u32 adf7025_tclkdiv;
	u8 mac_addr[ETH_ALEN];
};
#endif
