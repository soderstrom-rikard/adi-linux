/*
 * drivers/video/hitachi-tx09.c
 * Date: 2007-11-22
 *
 * Framebuffer driver for the Hitachi TX09D70VM1CDA TFT LCD written
 * by Harald Krapfenbauer <harald.krapfenbauer@bluetechnix.at>
 *
 * Thanks to Michael Hennerich from Analog Devices Inc. for his support
 * and the bf537-lq035.c TFT driver!
 *
 * For more information, please read the data sheet:
 * http://www.hitachi-displays-eu.com/doc/TX09D70VM1CDA.PDF
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/backlight.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <asm/dma.h>
#include <asm/portmux.h>

#define DRIVER_NAME "hitachi-tx09"

#define MAX_BRIGHTNESS 100
#define BFIN_LCD_NBR_PALETTE_ENTRIES	256

#undef BITREVERSED		/* colors bitreversed? (only needed for older EXT-CAM boards) */

#define PPI0_16 {P_PPI0_CLK, P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3, \
		P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7, P_PPI0_D8, P_PPI0_D9, P_PPI0_D10, \
		P_PPI0_D11, P_PPI0_D12, P_PPI0_D13, P_PPI0_D14, P_PPI0_D15, 0}

#ifdef CONFIG_BFIN537_BLUETECHNIX_CM

#define TIMER_DCLK 3
#define TIMER_HSYNC 0
#define TIMER_DTMG 1
#define TIMER_VSYNC 6
#define TIMER_BACKLIGHT 4
#define PCI_PIN GPIO_PF14
#define TIMERS {P_TMR0, P_TMR1, P_TMR3, P_TMR4, P_TMR6, 0}
#define WRITE_PPI_CONTROL(x) bfin_write_PPI_CONTROL(x)
#define READ_PPI_CONTROL bfin_read_PPI_CONTROL
#define WRITE_PPI_DELAY(x) bfin_write_PPI_DELAY(x)
#define WRITE_PPI_COUNT(x) bfin_write_PPI_COUNT(x)

#else

#ifdef CONFIG_BFIN561_BLUETECHNIX_CM

#define TIMER_DCLK 6
#define TIMER_HSYNC 8
#define TIMER_DTMG 9
#define TIMER_VSYNC 1
#define TIMER_BACKLIGHT 5
#define PCI_PIN GPIO_PF0
#define TIMERS {P_TMR1, P_TMR5, P_TMR6, P_TMR8, P_TMR9, 0}
#define WRITE_PPI_CONTROL(x) bfin_write_PPI0_CONTROL(x)
#define READ_PPI_CONTROL bfin_read_PPI0_CONTROL
#define WRITE_PPI_DELAY(x) bfin_write_PPI0_DELAY(x)
#define WRITE_PPI_COUNT(x) bfin_write_PPI0_COUNT(x)

#else
#error The Hitachi TX-09 frame buffer driver only supports Bluetechnix CM-BF537E and CM-BF561
#endif

#endif

#define BFIN_WRITE(a, b) CONCAT(bfin_write_TIMER, a, _, b)
#define BFIN_READ(a, b)  CONCAT(bfin_read_TIMER, a, _, b)
#define CONCAT(a, b, c, d) a ## b ## c ## d

static unsigned char *fb_buffer;	/* RGB Buffer */
static dma_addr_t dma_handle;
static unsigned long *dma_desc_table;
static unsigned long current_brightness;	/* backlight */
static int tx09_open_cnt;
static int tx09_mmap;
static struct backlight_device *bl_dev;
static int t_conf_done;
static DEFINE_SPINLOCK(tx09_lock);

static void set_backlight(int val)
{
	unsigned long timer_period;

	pr_debug("%s to %d\n", __FUNCTION__, val);

	if (val < 0)
		val = 0;
	if (val > MAX_BRIGHTNESS)
		val = MAX_BRIGHTNESS;

	current_brightness = val;

#ifdef CONFIG_BFIN561_BLUETECHNIX_CM
	bfin_write_TMRS8_DISABLE(1 << TIMER_BACKLIGHT);
#else
	bfin_write_TIMER_DISABLE(1 << TIMER_BACKLIGHT);
#endif
	SSYNC();

	timer_period = get_sclk() / 100000;
	BFIN_WRITE(TIMER_BACKLIGHT, WIDTH) ((timer_period * val) / 100);

#ifdef CONFIG_BFIN561_BLUETECHNIX_CM
	bfin_write_TMRS8_ENABLE(1 << TIMER_BACKLIGHT);
#else
	bfin_write_TIMER_ENABLE(1 << TIMER_BACKLIGHT);
#endif

	SSYNC();
}

static void start_timers(void)
{
	unsigned long flags;

	local_irq_save(flags);

#ifdef CONFIG_BFIN561_BLUETECHNIX_CM
	bfin_write_TMRS4_ENABLE((1 << (TIMER_HSYNC - 8)) |
				(1 << (TIMER_DTMG - 8)));
	bfin_write_TMRS8_ENABLE(1 << TIMER_VSYNC);
	SSYNC();
	bfin_write_TMRS8_ENABLE(1 << TIMER_DCLK);
#else
	bfin_write_TIMER_ENABLE((1 << TIMER_HSYNC) | (1 << TIMER_DTMG) |
				(1 << TIMER_VSYNC));
	SSYNC();
	bfin_write_TIMER_ENABLE(1 << TIMER_DCLK);
#endif
	SSYNC();
	mdelay(50);
	gpio_set_value(PCI_PIN, 1);
	SSYNC();
#ifdef CONFIG_BFIN561_BLUETECHNIX_CM
	bfin_write_TMRS8_ENABLE(1 << TIMER_BACKLIGHT);
#else
	bfin_write_TIMER_ENABLE(1 << TIMER_BACKLIGHT);
#endif
	SSYNC();

	local_irq_restore(flags);
}

static void stop_timers(void)
{
	unsigned long flags;
	long old_value = 0, new_value = 0;

	local_irq_save(flags);

#ifdef CONFIG_BFIN561_BLUETECHNIX_CM
	bfin_write_TMRS8_DISABLE(1 << TIMER_BACKLIGHT);
#else
	bfin_write_TIMER_DISABLE(1 << TIMER_BACKLIGHT);
#endif
	gpio_set_value(PCI_PIN, 0);

	while (1) {
		old_value = new_value;
		new_value = BFIN_READ(TIMER_VSYNC, COUNTER) ();
		if ((old_value - new_value) > 45000) {
#ifdef CONFIG_BFIN561_BLUETECHNIX_CM
			bfin_write_TMRS8_DISABLE((1 << TIMER_DCLK) |
						 (1 << TIMER_VSYNC));
			bfin_write_TMRS4_DISABLE((1 << (TIMER_HSYNC - 8)) |
						 (1 << (TIMER_DTMG - 8)));
#else
			bfin_write_TIMER_DISABLE((1 << TIMER_VSYNC) |
						 (1 << TIMER_HSYNC) |
						 (1 <<	TIMER_DTMG) |
						 (1 << TIMER_DCLK));
#endif
			break;
		}
	}

	local_irq_restore(flags);
}

static void config_timers(void)
{
	unsigned long timer_period = 0;

	pr_debug("%s\n", __FUNCTION__);

	/* stop timers */
#ifdef CONFIG_BFIN561_BLUETECHNIX_CM
	bfin_write_TMRS8_DISABLE((1 << TIMER_DCLK) | (1 << TIMER_VSYNC) |
				 (1 << TIMER_BACKLIGHT));
	bfin_write_TMRS4_DISABLE((1 << (TIMER_HSYNC - 8)) |
				 (1 << (TIMER_DTMG - 8)));
#else
	bfin_write_TIMER_DISABLE((1 << TIMER_VSYNC) | (1 << TIMER_HSYNC) |
				 (1 << TIMER_DTMG) | (1 << TIMER_DCLK) |
				 (1 << TIMER_BACKLIGHT));
#endif
	SSYNC();

	/* dclk clock output */
	BFIN_WRITE(TIMER_DCLK, CONFIG) (PERIOD_CNT | PULSE_HI | PWM_OUT);
	timer_period =
	    get_sclk() / (CONFIG_FB_HITACHI_TX09_REFRESHRATE * 89271);
	BFIN_WRITE(TIMER_DCLK, PERIOD) (timer_period);
	BFIN_WRITE(TIMER_DCLK, WIDTH) (timer_period / 2);
	SSYNC();

	/* brightness timer */
	BFIN_WRITE(TIMER_BACKLIGHT, CONFIG) (PERIOD_CNT | PULSE_HI | PWM_OUT);
	timer_period = get_sclk() / 100000;
	BFIN_WRITE(TIMER_BACKLIGHT, PERIOD) (timer_period);
	BFIN_WRITE(TIMER_BACKLIGHT, WIDTH) (timer_period - 1);	/* 100% duty cycle */
	SSYNC();

	/* hsync timer */
	BFIN_WRITE(TIMER_HSYNC, CONFIG) (CLK_SEL | TIN_SEL | PERIOD_CNT | PWM_OUT);	/* clocked by PPI_clk */
	BFIN_WRITE(TIMER_HSYNC, PERIOD) (273);	/* 240 + 33 blanking */
	BFIN_WRITE(TIMER_HSYNC, WIDTH) (5);
	SSYNC();

	/* dtmg timer */
	BFIN_WRITE(TIMER_DTMG, CONFIG) (CLK_SEL | TIN_SEL | PERIOD_CNT | PWM_OUT);	/* clocked by PPI_clk */
	BFIN_WRITE(TIMER_DTMG, PERIOD) (273);
	BFIN_WRITE(TIMER_DTMG, WIDTH) (33);
	SSYNC();

	/* vsync timer */
	BFIN_WRITE(TIMER_VSYNC, CONFIG) (CLK_SEL | TIN_SEL | PERIOD_CNT | PWM_OUT);	/* clocked by PPI_clk */
	BFIN_WRITE(TIMER_VSYNC, PERIOD) (89271);
	BFIN_WRITE(TIMER_VSYNC, WIDTH) (1911);
	SSYNC();

	t_conf_done = 1;
}

static void config_ppi(void)
{
	/* configure PPI registers */
	WRITE_PPI_DELAY(27);
	WRITE_PPI_COUNT(240 - 1);
	WRITE_PPI_CONTROL(0x380e);
}

static int config_dma(void)
{
	u32 i = 0;

	pr_debug("%s\n", __FUNCTION__);

	/* fill descriptor table */
	for (i = 0; i < 326; i++) {
		/* point to next desc table */
		dma_desc_table[2 * i] =
		    (unsigned long)&dma_desc_table[2 * i + 2];
	}
	/* last descriptor points to first */
	dma_desc_table[2 * 326] = (unsigned long)&dma_desc_table[0];

#ifdef CONFIG_FB_HITACHI_TX09_LANDSCAPE

	dma_desc_table[0 + 1] = (unsigned long)fb_buffer;
	for (i = 0; i < 7; i++) {
		/* blanking lines point to first line of fb_buffer */
		dma_desc_table[2 * i + 1] = (unsigned long)fb_buffer + 319 * 2;
	}
	for (i = 7; i < 327; i++) {
		/* visible lines */
		dma_desc_table[2 * i + 1] =
		    (unsigned long)fb_buffer + (319 - (i - 7)) * 2;
	}

#else				/* portrait mode */

	for (i = 0; i < 7; i++) {
		/* blanking lines point to first line of fb_buffer */
		dma_desc_table[2 * i + 1] = (unsigned long)fb_buffer;
	}
	for (i = 7; i < 327; i++) {
		/* visible lines */
		dma_desc_table[2 * i + 1] =
		    (unsigned long)fb_buffer + 2 * 240 * (i - 7);
	}

#endif

#ifdef CONFIG_FB_HITACHI_TX09_LANDSCAPE
	set_dma_x_count(CH_PPI, 240);
	set_dma_x_modify(CH_PPI, 2 * 320);
	set_dma_y_count(CH_PPI, 0);
	set_dma_y_modify(CH_PPI, 0);
	set_dma_next_desc_addr(CH_PPI, (unsigned long)dma_desc_table[2 * 326]);
#else
	set_dma_x_count(CH_PPI, 240);
	set_dma_x_modify(CH_PPI, 2);
	set_dma_y_count(CH_PPI, 0);
	set_dma_y_modify(CH_PPI, 0);
	set_dma_next_desc_addr(CH_PPI, (unsigned long)dma_desc_table[2 * 326]);
#endif

	set_dma_config(CH_PPI, 0x7404);

	return 0;
}

static int request_ports(int action)
{
	u16 ppi_req[] = PPI0_16;
	u16 tmr_req[] = TIMERS;

	if (action) {
		if (peripheral_request_list(ppi_req, DRIVER_NAME)) {
			printk(KERN_ERR DRIVER_NAME
			       ": Requesting Peripherals PPI faild\n");
			return -EFAULT;
		}

		if (peripheral_request_list(tmr_req, DRIVER_NAME)) {
			peripheral_free_list(ppi_req);
			printk(KERN_ERR DRIVER_NAME
			       ": Requesting Peripherals TMR faild\n");
			return -EFAULT;
		}

		if (gpio_request(PCI_PIN, DRIVER_NAME)) {
			peripheral_free_list(ppi_req);
			peripheral_free_list(tmr_req);
			printk(KERN_ERR ": Requesting GPIO %d faild\n", PCI_PIN);
			return -EFAULT;
		}

		gpio_direction_output(PCI_PIN, 0);
		SSYNC();
	} else {
		peripheral_free_list(ppi_req);
		peripheral_free_list(tmr_req);

		gpio_free(PCI_PIN);
	}
	return 0;
}

static struct fb_info tx09_fb;

static struct fb_var_screeninfo tx09_fb_defined = {
#ifdef CONFIG_FB_HITACHI_TX09_LANDSCAPE
	.xres = 320,
	.yres = 240,
	.xres_virtual = 320,
	.yres_virtual = 240,
#else
	.xres = 240,
	.yres = 320,
	.xres_virtual = 240,
	.yres_virtual = 320,
#endif
	.bits_per_pixel = 16,
	.activate = FB_ACTIVATE_TEST,
	.height = -1,
	.width = -1,
#ifdef BITREVERSED
	.red = {0, 5, 1},	/* offset, length, msb right */
	.green = {5, 6, 1},
	.blue = {11, 5, 1},
#else
	.red = {0, 5, 0},
	.green = {5, 6, 0},
	.blue = {11, 5, 0},
#endif
	.transp = {0, 0, 0},
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
};

static struct fb_fix_screeninfo tx09_fb_fix = {
	.id = DRIVER_NAME,
	.smem_len = 320 * 240 * 2,
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
#ifdef CONFIG_FB_HITACHI_TX09_LANDSCAPE
	.line_length = 320 * 2,
#else
	.line_length = 240 * 2,
#endif
	.accel = FB_ACCEL_NONE,
};

static int tx09_fb_open(struct fb_info *info, int user)
{
	unsigned long flags;

	pr_debug("%s\n", __FUNCTION__);

	spin_lock_irqsave(&tx09_lock, flags);
	tx09_open_cnt++;	/* increase counter */
	spin_unlock_irqrestore(&tx09_lock, flags);

	if (tx09_open_cnt <= 1) {	/* opened the first time */

		/* stop PPI */
		WRITE_PPI_CONTROL(0);
		SSYNC();

		/* configure dma stuff */
		config_dma();

		config_ppi();

		/* start dma */
		enable_dma(CH_PPI);
		SSYNC();

		/* start PPI */
		WRITE_PPI_CONTROL(READ_PPI_CONTROL() | PORT_EN);
		SSYNC();

		if (!t_conf_done)
			config_timers();

		start_timers();
	}

	return 0;
}

static int tx09_fb_release(struct fb_info *info, int user)
{
	unsigned long flags;

	pr_debug("%s\n", __FUNCTION__);

	spin_lock_irqsave(&tx09_lock, flags);
	tx09_open_cnt--;
	tx09_mmap = 0;
	spin_unlock_irqrestore(&tx09_lock, flags);

	if (tx09_open_cnt <= 0) {
		stop_timers();

		WRITE_PPI_CONTROL(0);
		SSYNC();

		disable_dma(CH_PPI);
	}

	return 0;
}

static int tx09_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	pr_debug("%s\n", __FUNCTION__);

	if (var->bits_per_pixel != 16) {
		pr_debug("%s: depth not supported: %u BPP\n", __FUNCTION__,
			 var->bits_per_pixel);
		return -EINVAL;
	}

	if (info->var.xres != var->xres || info->var.yres != var->yres ||
	    info->var.xres_virtual != var->xres_virtual
	    || info->var.yres_virtual != var->yres_virtual) {
		pr_debug("%s: Resolution not supported: X%u x Y%u\n",
			 __FUNCTION__, var->xres, var->yres);
		return -EINVAL;
	}

	if ((info->fix.line_length * var->yres_virtual) > info->fix.smem_len) {
		pr_debug("%s: Memory limit requested yres_virtual = %u\n",
			 __FUNCTION__, var->yres_virtual);
		return -ENOMEM;
	}

	return 0;
}

static int tx09_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long flags;

	pr_debug("%s\n", __FUNCTION__);

	if (tx09_mmap)
		return -1;	/* already mmap'ed */

	spin_lock_irqsave(&tx09_lock, flags);
	tx09_mmap = 1;
	spin_unlock_irqrestore(&tx09_lock, flags);

	vma->vm_start = (unsigned long)fb_buffer;
	vma->vm_end = vma->vm_start + 320 * 240 * 2;
	vma->vm_flags |=  VM_MAYSHARE | VM_SHARED;

	return 0;
}

int tx09_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{

#ifdef CONFIG_FB_HITACHI_TX09_CURSOR
	return -EINVAL;		/* just to force soft_cursor() call */
#else
	return 0;
#endif

}

static int tx09_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			     u_int transp, struct fb_info *info)
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
		value =
		    (red << info->var.red.offset) | (green << info->var.green.
						     offset) | (blue << info->
								var.blue.
								offset);
		value &= 0xFFFF;
		((u32 *) (info->pseudo_palette))[regno] = value;
	}
	return 0;
}

static struct fb_ops tx09_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = tx09_fb_open,
	.fb_release = tx09_fb_release,
	.fb_check_var = tx09_fb_check_var,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = tx09_fb_mmap,
	.fb_cursor = tx09_fb_cursor,
	.fb_setcolreg = tx09_fb_setcolreg,
};

static int bl_update_properties(struct backlight_device *bd)
{
	pr_debug("%s\n", __FUNCTION__);
	set_backlight(bd->props.brightness);
	return 0;
}

static int bl_get_brightness(struct backlight_device *bd)
{
	pr_debug("%s\n", __FUNCTION__);
	return current_brightness;
}

static struct backlight_ops tx09fb_bl_ops = {
	.get_brightness = bl_get_brightness,
	.update_status = bl_update_properties,
};

static int __init tx09_probe(struct platform_device *pdev)
{
	pr_debug("%s\n", __FUNCTION__);

	printk(KERN_INFO DRIVER_NAME ": FrameBuffer initializing...\n");

	/* dma channel */
	if (request_dma(CH_PPI, "BF533_PPI_DMA") < 0) {
		printk(KERN_ERR DRIVER_NAME ": couldn't request PPI dma.\n");
		return -EFAULT;
	}

	/* gpio ports */
	if (request_ports(1)) {
		printk(KERN_ERR DRIVER_NAME ": couldn't request gpio port.\n");
		free_dma(CH_PPI);
		return -EFAULT;
	}

	/* frame buffer */
	fb_buffer = dma_alloc_coherent(NULL, 240 * 320 * 2, &dma_handle, GFP_KERNEL);	/* 7 blanking lines, 2 bytes/pixel */
	if (fb_buffer == NULL) {
		printk(KERN_ERR DRIVER_NAME
		       ": couldn't allocate dma buffer.\n");
		free_dma(CH_PPI);
		request_ports(0);
		return -ENOMEM;
	}

	/* dma descriptor list */
#if L1_DATA_A_LENGTH != 0
	dma_desc_table =
	    (unsigned long *)l1_data_sram_alloc(sizeof(unsigned long) * 2 *
						(320 + 7));
#else
	dma_desc_table =
	    dma_alloc_coherent(NULL, sizeof(unsigned long) * 2 * (320 + 7),
			       &dma_handle, 0);
#endif

	if (dma_desc_table == NULL) {
		printk(KERN_ERR DRIVER_NAME
		       ": couldn't allocate dma descriptor.\n");
		free_dma(CH_PPI);
		request_ports(0);
		dma_free_coherent(NULL, 240 * 320 * 2, fb_buffer, dma_handle);
		return -ENOMEM;
	}

	memset(fb_buffer, 0, 240 * 320 * 2);

	tx09_fb.screen_base = (void *)fb_buffer;
	tx09_fb_fix.smem_start = (int)fb_buffer;

	tx09_fb.fbops = &tx09_fb_ops;
	tx09_fb.var = tx09_fb_defined;

	tx09_fb.fix = tx09_fb_fix;
	tx09_fb.flags = FBINFO_DEFAULT;

	/* pseudo palette */

	tx09_fb.pseudo_palette = kmalloc(sizeof(u32) * 16, GFP_KERNEL);

	if (!tx09_fb.pseudo_palette) {
		printk(KERN_ERR DRIVER_NAME
		       "Failed to allocate pseudo palette\n");
		free_dma(CH_PPI);
		request_ports(0);
		dma_free_coherent(NULL, 240 * 320 * 2, fb_buffer, dma_handle);
		return -ENOMEM;
	}
	memset(tx09_fb.pseudo_palette, 0, sizeof(u32) * 16);

	/* color map */
	if (fb_alloc_cmap(&tx09_fb.cmap, BFIN_LCD_NBR_PALETTE_ENTRIES, 0) < 0) {
		printk(KERN_ERR DRIVER_NAME
		       "Failed to allocate colormap (%d entries)\n",
		       BFIN_LCD_NBR_PALETTE_ENTRIES);
		free_dma(CH_PPI);
		request_ports(0);
		dma_free_coherent(NULL, 240 * 320 * 2, fb_buffer, dma_handle);
		kfree(tx09_fb.pseudo_palette);
		return -EFAULT;
	}

	/* register framebuffer */
	if (register_framebuffer(&tx09_fb) < 0) {
		printk(KERN_ERR DRIVER_NAME
		       ": unable to register framebuffer.\n");
		free_dma(CH_PPI);
		request_ports(0);
		dma_free_coherent(NULL, 240 * 320 * 2, fb_buffer, dma_handle);
		fb_buffer = NULL;
		return -EINVAL;
	}

	/* backlight device */
	bl_dev =
	    backlight_device_register("hitachi-bl", NULL, NULL, &tx09fb_bl_ops);
	bl_dev->props.max_brightness = MAX_BRIGHTNESS;

	printk(KERN_INFO "Done.\n");

	return 0;
}

static int tx09_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __FUNCTION__);

	if (fb_buffer != NULL)
		dma_free_coherent(NULL, 240 * 320 * 2, fb_buffer, dma_handle);

#if L1_DATA_A_LENGTH != 0
	if (dma_desc_table)
		l1_data_sram_free(dma_desc_table);
#else
	if (dma_desc_table)
		dma_free_coherent(NULL, sizeof(unsigned long) * 2 * (320 + 7),
				  &dma_handle, 0);
#endif

	stop_timers();

	free_dma(CH_PPI);

	kfree(tx09_fb.pseudo_palette);
	fb_dealloc_cmap(&tx09_fb.cmap);

	backlight_device_unregister(bl_dev);

	unregister_framebuffer(&tx09_fb);

	request_ports(0);

	printk(KERN_INFO DRIVER_NAME ": Unregistered LCD driver.\n");

	return 0;
}

#ifdef CONFIG_PM
static int tx09_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (tx09_open_cnt > 0) {
		WRITE_PPI_CONTROL(0);
		SSYNC();
		disable_dma(CH_PPI);
	}
	return 0;
}

static int tx09_resume(struct platform_device *pdev)
{
	if (tx09_open_cnt > 0) {
		WRITE_PPI_CONTROL(0);
		SSYNC();

		config_dma();
		config_ppi();

		enable_dma(CH_PPI);
		SSYNC();
		WRITE_PPI_CONTROL(READ_PPI_CONTROL() | PORT_EN);
		SSYNC();
	}
	return 0;
}
#else
#define tx09_suspend NULL
#define tx09_resume  NULL
#endif

static struct platform_driver tx09_driver = {
	.probe = tx09_probe,
	.remove = tx09_remove,
	.suspend = tx09_suspend,
	.resume = tx09_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __devinit tx09_driver_init(void)
{
	return platform_driver_register(&tx09_driver);
}

static void __exit tx09_driver_cleanup(void)
{
	platform_driver_unregister(&tx09_driver);
}

MODULE_DESCRIPTION("Hitachi TX09D70VM1CDA TFT LCD Driver");
MODULE_AUTHOR("Harald Krapfenbauer");
MODULE_LICENSE("GPL");

module_init(tx09_driver_init);
module_exit(tx09_driver_cleanup);
