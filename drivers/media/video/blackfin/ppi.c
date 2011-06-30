/*
 * ppi.c Analog Devices Parallel Peripheral Interface driver
 *
 * Copyright (c) 2011 Scott Jiang <Scott.Jiang.Linux@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <asm/blackfin.h>
#include <asm/portmux.h>

#include <media/blackfin/ppi.h>

#define regr(reg)               readw((reg) + ppi_base)
#define regw(value, reg)        writew(value, ((reg) + ppi_base))

#define REG_PPI_CONTROL        0x00 /* PPI Control */
#define REG_PPI_STATUS         0x04 /* PPI Status */
#define REG_PPI_COUNT          0x08 /* Transfer Count */
#define REG_PPI_DELAY          0x0C /* Delay Count */
#define REG_PPI_FRAME          0x10 /* Lines Per Frame */

static int ppi_attach_irq(struct ppi_if *intf, irq_handler_t handler);
static void ppi_detach_irq(struct ppi_if *intf);
static int ppi_start(struct ppi_if *intf);
static int ppi_stop(struct ppi_if *intf);
static int ppi_set_params(struct ppi_if *intf, struct ppi_params *params);
static void ppi_update_addr(struct ppi_if *intf, unsigned long addr);

static const unsigned short ppi_req[] = {
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2,
	0,
};

static struct ppi_ops ppi_ops = {
	.attach_irq = ppi_attach_irq,
	.detach_irq = ppi_detach_irq,
	.start = ppi_start,
	.stop = ppi_stop,
	.set_params = ppi_set_params,
	.update_addr = ppi_update_addr,
};

static struct ppi_if ppi_intf = {
	.ops = &ppi_ops,
	.pin_req = ppi_req,
};

static void __iomem *ppi_base;
static resource_size_t  res_start, res_len;

static irqreturn_t ppi_irq_err(int irq, void *dev_id)
{
	unsigned short status;

	status = regr(REG_PPI_STATUS);
	if (printk_ratelimit())
		pr_info("%s: status = 0x%x\n", __func__, status);

	regw(0xff, REG_PPI_STATUS);

	return IRQ_HANDLED;
}

static int ppi_attach_irq(struct ppi_if *intf, irq_handler_t handler)
{
	if (request_dma(intf->dma_ch, "PPI_DMA") < 0) {
		pr_err("Unable to allocate DMA channel for PPI\n");
		return -EBUSY;
	}
	set_dma_callback(intf->dma_ch, handler, intf);

	if (request_irq(intf->irq_err, ppi_irq_err, IRQF_DISABLED,
				"PPI ERROR", intf)) {
		pr_err("Unable to allocate IRQ for PPI\n");
		free_dma(intf->dma_ch);
		return -EBUSY;
	}
	return 0;
}

static void ppi_detach_irq(struct ppi_if *intf)
{
	free_irq(intf->irq_err, intf);
	free_dma(intf->dma_ch);
}

static int ppi_start(struct ppi_if *intf)
{
	/* enable DMA */
	enable_dma(intf->dma_ch);

	/* enable PPI */
	intf->ppi_control |= PORT_EN;
	regw(intf->ppi_control, REG_PPI_CONTROL);

	SSYNC();
	return 0;
}

static int ppi_stop(struct ppi_if *intf)
{
	/* disable PPI */
	intf->ppi_control &= ~PORT_EN;
	regw(intf->ppi_control, REG_PPI_CONTROL);

	/* disable DMA */
	clear_dma_irqstat(intf->dma_ch);
	disable_dma(intf->dma_ch);

	SSYNC();
	return 0;
}

static int ppi_set_params(struct ppi_if *intf, struct ppi_params *params)
{
	intf->bytes_per_line = params->width * params->bpp;
	intf->lines_per_frame = params->height;

	/* config DMA */
	intf->dma_config = (DMA_FLOW_STOP | WNR | RESTART | WDSIZE_16 | DMA2D | DI_EN);
	set_dma_x_count(intf->dma_ch, intf->bytes_per_line >> 1);
	set_dma_x_modify(intf->dma_ch, 2);
	set_dma_y_count(intf->dma_ch, intf->lines_per_frame);
	set_dma_y_modify(intf->dma_ch, 2);
	set_dma_config(intf->dma_ch, intf->dma_config);

	/* config PPI */
	intf->ppi_control = params->ppi_control & ~PORT_EN;
	regw(intf->ppi_control, REG_PPI_CONTROL);
	regw(intf->bytes_per_line - 1, REG_PPI_COUNT);
	regw(intf->lines_per_frame, REG_PPI_FRAME);

	SSYNC();
	return 0;
}

static void ppi_update_addr(struct ppi_if *intf, unsigned long addr)
{
	set_dma_start_addr(intf->dma_ch, addr);
}

struct ppi_if *bfin_get_ppi_if(void)
{
	return &ppi_intf;
}
EXPORT_SYMBOL(bfin_get_ppi_if);

static int __devinit ppi_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res) {
		dev_err(&pdev->dev, "no DMA resource\n");
		ret = -ENODEV;
		goto err;
	}
	ppi_intf.dma_ch = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "no IRQ resource\n");
		ret = -ENODEV;
		goto err;
	}
	ppi_intf.irq_err = res->start;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no MEM resource\n");
		ret = -ENODEV;
		goto err;
	}

	res_start = res->start;
	res_len = res->end - res->start + 1;
	res = request_mem_region(res_start, res_len, res->name);
	if (!res) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto err;
	}

	ppi_base = ioremap(res_start, res_len);
	if (!ppi_base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_mem;
	}

	if (peripheral_request_list(ppi_intf.pin_req, KBUILD_MODNAME)) {
		dev_err(&pdev->dev, "request peripheral failed\n");
		ret = -EBUSY;
		goto err_ioremap;
	}

	dev_info(&pdev->dev, "ppi probe success\n");
	return 0;
err_ioremap:
	iounmap(ppi_base);
err_mem:
	release_mem_region(res_start, res_len);
err:
	return ret;
}

static int __devexit ppi_remove(struct platform_device *pdev)
{
	peripheral_free_list(ppi_intf.pin_req);
	iounmap(ppi_base);
	release_mem_region(res_start, res_len);
	return 0;
}

static struct platform_driver ppi_driver = {
	.driver = {
		.name   = "ppi",
		.owner = THIS_MODULE,
	},
	.probe = ppi_probe,
	.remove = __devexit_p(ppi_remove),
};

static int __init ppi_init(void)
{
	return platform_driver_register(&ppi_driver);
}

static void __exit ppi_exit(void)
{
	platform_driver_unregister(&ppi_driver);
}

device_initcall(ppi_init);
module_exit(ppi_exit);

MODULE_DESCRIPTION("Analog Devices Parallel Peripheral Interface driver");
MODULE_AUTHOR("Scott Jiang");
MODULE_LICENSE("GPL v2");
