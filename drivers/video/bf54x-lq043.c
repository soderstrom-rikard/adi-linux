/*
 * File:         drivers/video/bf54x-lq043.c
 * Based on:
 * Author:       Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * Created:
 * Description:  ADSP-BF54x Framebufer driver
 *
 *
 * Modified:
 *               Copyright 2004-2007 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/lcd.h>
#include <linux/i2c.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <asm/blackfin.h>
#include <asm/irq.h>
#include <asm/dpmc.h>
#include <asm/dma-mapping.h>
#include <asm/dma.h>
#include <asm/gpio.h>

#define DRIVER_NAME "bf54x-lq043"

#define BFIN_LCD_NBR_PALETTE_ENTRIES	256

static unsigned char *fb_buffer;	/* RGB Buffer */
static dma_addr_t dma_handle;
static int lq035_mmap;
static int lq035_open_cnt;
static DEFINE_SPINLOCK(bfin_lq035_lock);

static int nocursor;
module_param(nocursor, int, 0644);
MODULE_PARM_DESC(nocursor, "cursor enable/disable");

static int outp_rgb666;
module_param(outp_rgb666, int, 0);
MODULE_PARM_DESC(outp_rgb666, "Output 18-bit RGB666");

#define DISP     		GPIO_PE3

#define LCD_X_RES		480	/*Horizontal Resolution */
#define LCD_Y_RES		272	/* Vertical Resolution */

#define LCD_BPP			24	/* Bit Per Pixel */
#define	DMA_BUS_SIZE		32

#define ACTIVE_VIDEO_MEM_SIZE	(LCD_Y_RES*LCD_X_RES*(LCD_BPP/8))

/* 	-- Horizontal synchronizing --
 *
 * Timing characteristics taken from the SHARP LQ043T1DG01 datasheet
 * (LCY-W-06602A Page 9 of 22)
 *
 * Clock Frequency 	1/Tc Min 7.83 Typ 9.00 Max 9.26 MHz
 *
 * Period 		TH - 525 - Clock
 * Pulse width 		THp - 41 - Clock
 * Horizontal period 	THd - 480 - Clock
 * Back porch 		THb - 2 - Clock
 * Front porch 		THf - 2 - Clock
 *
 * -- Vertical synchronizing --
 * Period 		TV - 286 - Line
 * Pulse width 		TVp - 10 - Line
 * Vertical period 	TVd - 272 - Line
 * Back porch 		TVb - 2 - Line
 * Front porch 		TVf - 2 - Line
 */


#define	LCD_CLK         	(8*1000*1000)	/* 8MHz */

/* # active data to transfer after Horizontal Delay clock */
#define EPPI_HCOUNT		LCD_X_RES

/* # active lines to transfer after Vertical Delay clock */
#define EPPI_VCOUNT		LCD_Y_RES

/* Samples per Line = 480 (active data) + 45 (padding) */
#define EPPI_LINE		525

/* Lines per Frame = 272 (active data) + 14 (padding) */
#define EPPI_FRAME		286

/* FS1 (Hsync) Width (Typical)*/
#define EPPI_FS1W_HBL		41

/* FS1 (Hsync) Period (Typical) */
#define EPPI_FS1P_AVPL		525

/* Horizontal Delay clock after assertion of Hsync (Typical) */
#define EPPI_HDELAY		43

/* FS2 (Vsync) Width    = FS1 (Hsync) Period * 10 */
#define EPPI_FS2W_LVB		(EPPI_FS1P_AVPL * 10)

 /* FS2 (Vsync) Period   = FS1 (Hsync) Period * Lines per Frame */
#define EPPI_FS2P_LAVF		(EPPI_FS1P_AVPL * EPPI_FRAME)

/* Vertical Delay after assertion of Vsync (2 Lines) */
#define EPPI_VDELAY		12

#define EPPI_CLIP		0xFF00FF00

/* EPPI Control register configuration value for RGB out
 * - EPPI as Output
 * GP 2 frame sync mode,
 * Internal Clock generation disabled, Internal FS generation enabled,
 * Receives samples on EPPI_CLK raising edge, Transmits samples on EPPI_CLK falling edge,
 * FS1 & FS2 are active high,
 * DLEN = 6 (24 bits for RGB888 out) or 5 (18 bits for RGB666 out)
 * DMA Unpacking disabled when RGB Formating is enabled, otherwise DMA unpacking enabled
 * Swapping Enabled,
 * One (DMA) Channel Mode,
 * RGB Formatting Enabled for RGB666 output, disabled for RGB888 output
 * Regular watermark - when FIFO is 75% full,
 * Urgent watermark - when FIFO is 25% full
 */

#define EPPI_CONTROL		(0x68136E2E | SWAPEN)


static inline u16 get_eppi_clkdiv(u32 target_ppi_clk)
{
	u32 sclk = get_sclk();

	/* EPPI_CLK = (SCLK) / (2 * (EPPI_CLKDIV[15:0] + 1)) */

	return (((sclk / target_ppi_clk) / 2) - 1);
}

static void config_ppi(void)
{

	u16 eppi_clkdiv = get_eppi_clkdiv(LCD_CLK);

	bfin_write_EPPI0_FS1W_HBL(EPPI_FS1W_HBL);
	bfin_write_EPPI0_FS1P_AVPL(EPPI_FS1P_AVPL);
	bfin_write_EPPI0_FS2W_LVB(EPPI_FS2W_LVB);
	bfin_write_EPPI0_FS2P_LAVF(EPPI_FS2P_LAVF);
	bfin_write_EPPI0_CLIP(EPPI_CLIP);

	bfin_write_EPPI0_FRAME(EPPI_FRAME);
	bfin_write_EPPI0_LINE(EPPI_LINE);

	bfin_write_EPPI0_HCOUNT(EPPI_HCOUNT);
	bfin_write_EPPI0_HDELAY(EPPI_HDELAY);
	bfin_write_EPPI0_VCOUNT(EPPI_VCOUNT);
	bfin_write_EPPI0_VDELAY(EPPI_VDELAY);

	bfin_write_EPPI0_CLKDIV(eppi_clkdiv);

/*
 * DLEN = 6 (24 bits for RGB888 out) or 5 (18 bits for RGB666 out)
 * RGB Formatting Enabled for RGB666 output, disabled for RGB888 output
 */
	if (outp_rgb666) {
		bfin_write_EPPI0_CONTROL((EPPI_CONTROL & ~DLENGTH) | DLEN_18 | RGB_FMT_EN);
	}else {
		bfin_write_EPPI0_CONTROL((EPPI_CONTROL & ~DLENGTH) | DLEN_24 & ~RGB_FMT_EN);
	}

}

static int config_dma(void)
{

	set_dma_config(CH_EPPI0,
		       set_bfin_dma_config(DIR_READ, DMA_FLOW_AUTO,
					   INTR_DISABLE, DIMENSION_2D,
					   DATA_SIZE_32));
	set_dma_x_count(CH_EPPI0, (LCD_X_RES * LCD_BPP) / DMA_BUS_SIZE);
	set_dma_x_modify(CH_EPPI0, DMA_BUS_SIZE / 8);
	set_dma_y_count(CH_EPPI0, LCD_Y_RES);
	set_dma_y_modify(CH_EPPI0, DMA_BUS_SIZE / 8);
	set_dma_start_addr(CH_EPPI0, (unsigned long)fb_buffer);

	return 0;
}

static int request_ports(void)
{
	if (gpio_request(DISP, NULL)) {
		printk(KERN_ERR "Requesting GPIO %d faild\n", DISP);

		return -EFAULT;
	}

	gpio_direction_output(DISP);
	gpio_set_value(DISP, 1);

	bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 0xFFFF);
	bfin_write_PORTF_MUX(0);

	bfin_write_PORTG_FER(bfin_read_PORTG_FER() | 0x1F);
	bfin_write_PORTG_MUX(bfin_read_PORTG_MUX() & 0xFFFFFC00);

	if (!outp_rgb666) {
		bfin_write_PORTD_FER(bfin_read_PORTD_FER() | 0x3F);
		bfin_write_PORTD_MUX(bfin_read_PORTD_MUX() | 0xFFF);
	}
	SSYNC();

	gpio_set_value(DISP, 1);

	return 0;
}

static void free_ports(void)
{
	gpio_free(DISP);
}

static struct fb_info bfin_bf54x_fb;

static struct fb_var_screeninfo bfin_bf54x_fb_defined = {
	.bits_per_pixel = LCD_BPP,
	.activate = FB_ACTIVATE_TEST,
	.xres = LCD_X_RES,	/*default portrait mode RGB */
	.yres = LCD_Y_RES,
	.xres_virtual = LCD_X_RES,
	.yres_virtual = LCD_Y_RES,
	.height = -1,
	.width = -1,
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.red = {16, 8, 0},
	.green = {8, 8, 0},
	.blue = {0, 8, 0},
	.transp = {0, 0, 0},
};

static struct fb_fix_screeninfo bfin_bf54x_fb_fix __initdata = {
	.id = DRIVER_NAME,
	.smem_len = ACTIVE_VIDEO_MEM_SIZE,
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
	.line_length = LCD_X_RES * (LCD_BPP / 8),
	.accel = FB_ACCEL_NONE,
};

static int bfin_bf54x_fb_open(struct fb_info *info, int user)
{
	unsigned long flags;

	spin_lock_irqsave(&bfin_lq035_lock, flags);
	lq035_open_cnt++;
	spin_unlock_irqrestore(&bfin_lq035_lock, flags);

	if (lq035_open_cnt <= 1) {

		bfin_write_EPPI0_CONTROL(0);
		SSYNC();

		config_dma();
		config_ppi();

		/* start dma */
		enable_dma(CH_EPPI0);
		SSYNC();
		bfin_write_EPPI0_CONTROL(bfin_read_EPPI0_CONTROL() | EPPI_EN);
		SSYNC();
	}

	return 0;
}

static int bfin_bf54x_fb_release(struct fb_info *info, int user)
{
	unsigned long flags;

	spin_lock_irqsave(&bfin_lq035_lock, flags);
	lq035_open_cnt--;
	lq035_mmap = 0;
	spin_unlock_irqrestore(&bfin_lq035_lock, flags);

	if (lq035_open_cnt <= 0) {

		bfin_write_EPPI0_CONTROL(0);
		SSYNC();
		disable_dma(CH_EPPI0);
	}

	return 0;
}

static int bfin_bf54x_fb_check_var(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{

	if (var->bits_per_pixel != LCD_BPP) {
		pr_debug("%s: depth not supported: %u BPP\n", __FUNCTION__,
			 var->bits_per_pixel);
		return -EINVAL;
	}

	if (info->var.xres != var->xres || info->var.yres != var->yres ||
	    info->var.xres_virtual != var->xres_virtual ||
	    info->var.yres_virtual != var->yres_virtual) {
		pr_debug("%s: Resolution not supported: X%u x Y%u \n",
			 __FUNCTION__, var->xres, var->yres);
		return -EINVAL;
	}

	/*
	 *  Memory limit
	 */

	if ((info->fix.line_length * var->yres_virtual) > info->fix.smem_len) {
		pr_debug("%s: Memory Limit requested yres_virtual = %u\n",
			 __FUNCTION__, var->yres_virtual);
		return -ENOMEM;
	}

	return 0;
}

static int direct_mmap(struct fb_info *info, struct vm_area_struct *vma)
{

	unsigned long flags;

	if (lq035_mmap)
		return -1;

	spin_lock_irqsave(&bfin_lq035_lock, flags);
	lq035_mmap = 1;
	spin_unlock_irqrestore(&bfin_lq035_lock, flags);

	vma->vm_start = (unsigned long)(fb_buffer);

	vma->vm_end = vma->vm_start + ACTIVE_VIDEO_MEM_SIZE;
	/* For those who don't understand how mmap works, go read
	 *   Documentation/nommu-mmap.txt.
	 * For those that do, you will know that the VM_MAYSHARE flag
	 * must be set in the vma->vm_flags structure on noMMU
	 *   Other flags can be set, and are documented in
	 *   include/linux/mm.h
	 */
	vma->vm_flags |= VM_MAYSHARE;

	return 0;
}

int bfin_bf54x_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	if (nocursor)
		return 0;
	else
		return -EINVAL;	/* just to force soft_cursor() call */
}

static int bfin_bf54x_fb_setcolreg(u_int regno, u_int red, u_int green,
				   u_int blue, u_int transp,
				   struct fb_info *info)
{
	if (regno >= BFIN_LCD_NBR_PALETTE_ENTRIES)
		return -EINVAL;

	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {

		u32 value;
		/* Place color in the pseudopalette */
		if (regno > 16)
			return -EINVAL;

		red >>= (16 - info->var.red.length);
		green >>= (16 - info->var.green.length);
		blue >>= (16 - info->var.blue.length);

		value = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset);
		value &= 0xFFFFFF;

		((u32 *) (info->pseudo_palette))[regno] = value;

	}

	return 0;
}

static struct fb_ops bfin_bf54x_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = bfin_bf54x_fb_open,
	.fb_release = bfin_bf54x_fb_release,
	.fb_check_var = bfin_bf54x_fb_check_var,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = direct_mmap,
	.fb_cursor = bfin_bf54x_fb_cursor,
	.fb_setcolreg = bfin_bf54x_fb_setcolreg,
};

static int bl_get_brightness(struct backlight_device *bd)
{
	return 0;
}

static struct backlight_ops bfin_lq035fb_bl_ops = {
	.get_brightness = bl_get_brightness,
};

static struct backlight_device *bl_dev;

static int bfin_lcd_get_power(struct lcd_device *dev)
{
	return 0;
}

static int bfin_lcd_set_power(struct lcd_device *dev, int power)
{
	return 0;
}

static int bfin_lcd_get_contrast(struct lcd_device *dev)
{
	return 0;
}

static int bfin_lcd_set_contrast(struct lcd_device *dev, int contrast)
{

	return 0;
}

static int bfin_lcd_check_fb(struct fb_info *fi)
{
	if (!fi || (fi == &bfin_bf54x_fb))
		return 1;
	return 0;
}

static struct lcd_ops bfin_lcd_ops = {
	.get_power = bfin_lcd_get_power,
	.set_power = bfin_lcd_set_power,
	.get_contrast = bfin_lcd_get_contrast,
	.set_contrast = bfin_lcd_set_contrast,
	.check_fb = bfin_lcd_check_fb,
};

static struct lcd_device *lcd_dev;

static int __init bfin_bf54x_fb_init(void)
{
	printk(KERN_INFO DRIVER_NAME ": FrameBuffer initializing...");

	if (request_dma(CH_EPPI0, "CH_EPPI0") < 0) {
		printk(KERN_ERR DRIVER_NAME
		       ": couldn't request CH_EPPI0 DMA\n");
		return -EFAULT;
	}

	if (request_ports()) {
		printk(KERN_ERR DRIVER_NAME ": couldn't request gpio port.\n");
		free_dma(CH_EPPI0);
		return -EFAULT;
	}

	fb_buffer =
	    dma_alloc_coherent(NULL, ACTIVE_VIDEO_MEM_SIZE, &dma_handle,
			       GFP_KERNEL);

	if (NULL == fb_buffer) {
		printk(KERN_ERR DRIVER_NAME
		       ": couldn't allocate dma buffer.\n");
		free_dma(CH_EPPI0);
		free_ports();
		return -ENOMEM;
	}

	memset(fb_buffer, 0xff, ACTIVE_VIDEO_MEM_SIZE);

	bfin_bf54x_fb.screen_base = (void *)fb_buffer;
	bfin_bf54x_fb_fix.smem_start = (int)fb_buffer;

	bfin_bf54x_fb.fbops = &bfin_bf54x_fb_ops;
	bfin_bf54x_fb.var = bfin_bf54x_fb_defined;

	bfin_bf54x_fb.fix = bfin_bf54x_fb_fix;
	bfin_bf54x_fb.flags = FBINFO_DEFAULT;

	bfin_bf54x_fb.pseudo_palette = kmalloc(sizeof(u32) * 16, GFP_KERNEL);

	if (!bfin_bf54x_fb.pseudo_palette) {
		printk(KERN_ERR DRIVER_NAME
		       "Fail to allocate pseudo_palette\n");
		free_dma(CH_EPPI0);
		free_ports();
		dma_free_coherent(NULL, ACTIVE_VIDEO_MEM_SIZE, fb_buffer,
				  dma_handle);
		return -ENOMEM;
	}
	memset(bfin_bf54x_fb.pseudo_palette, 0, sizeof(u32) * 16);

	if (fb_alloc_cmap(&bfin_bf54x_fb.cmap, BFIN_LCD_NBR_PALETTE_ENTRIES, 0)
	    < 0) {
		printk(KERN_ERR DRIVER_NAME
		       "Fail to allocate colormap (%d entries)\n",
		       BFIN_LCD_NBR_PALETTE_ENTRIES);
		free_dma(CH_EPPI0);
		free_ports();
		dma_free_coherent(NULL, ACTIVE_VIDEO_MEM_SIZE, fb_buffer,
				  dma_handle);
		kfree(bfin_bf54x_fb.pseudo_palette);
		return -EFAULT;
	}

	if (register_framebuffer(&bfin_bf54x_fb) < 0) {
		printk(KERN_ERR DRIVER_NAME
		       ": unable to register framebuffer.\n");
		free_dma(CH_EPPI0);
		free_ports();
		dma_free_coherent(NULL, ACTIVE_VIDEO_MEM_SIZE, fb_buffer,
				  dma_handle);
		fb_buffer = NULL;
		kfree(bfin_bf54x_fb.pseudo_palette);
		fb_dealloc_cmap(&bfin_bf54x_fb.cmap);
		return -EINVAL;
	}

	bl_dev =
	    backlight_device_register("bf54x-bl", NULL, NULL,
				      &bfin_lq035fb_bl_ops);
	bl_dev->props.max_brightness = 255;

	lcd_dev = lcd_device_register(DRIVER_NAME, NULL, &bfin_lcd_ops);
	lcd_dev->props.max_contrast = 255, printk(KERN_INFO "Done.\n");
	return 0;
}

static void __exit bfin_bf54x_fb_exit(void)
{
	if (fb_buffer != NULL)
		dma_free_coherent(NULL, ACTIVE_VIDEO_MEM_SIZE, fb_buffer,
				  dma_handle);

	free_dma(CH_EPPI0);

	kfree(bfin_bf54x_fb.pseudo_palette);
	fb_dealloc_cmap(&bfin_bf54x_fb.cmap);

	lcd_device_unregister(lcd_dev);
	backlight_device_unregister(bl_dev);

	unregister_framebuffer(&bfin_bf54x_fb);

	free_ports();

	printk(KERN_INFO DRIVER_NAME ": Unregister LCD driver.\n");
}

MODULE_DESCRIPTION("Blackfin BF54x TFT LCD Driver");
MODULE_LICENSE("GPL");

module_init(bfin_bf54x_fb_init);
module_exit(bfin_bf54x_fb_exit);
