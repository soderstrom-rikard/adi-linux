/*
  bfsi.c
  David Rowe 21 June 2006

  Functions for Linux device drivers on the Blackfin that
  support interfacing the Blackfin to Silicon Labs chips.

  These functions are in a separate file from the target wcfxs driver
  so they can be re-used with different drivers, for example unit
  test software.

  For various reasons the CPHA=1 (sofware controlled SPISEL)
  mode needs to be used, for example the SiLabs chip expects
  SPISEL to go high between 8 bit transfers and the timing
  the Si3050 expects (Figs 3 & 38 of 3050 data sheet) are
  closest to Fig 10-12 of the BF533 hardware reference manual.

  See also unittest tspi.c
*/

/*
  Copyright (C) 2006 David Rowe

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/bfin5xx_spi.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_BF561
static unsigned int dma_iobase[MAX_BLACKFIN_DMA_CHANNEL] = {
        DMA1_0_NEXT_DESC_PTR,
        DMA1_1_NEXT_DESC_PTR,
        DMA1_2_NEXT_DESC_PTR,
        DMA1_3_NEXT_DESC_PTR,
        DMA1_4_NEXT_DESC_PTR,
        DMA1_5_NEXT_DESC_PTR,
        DMA1_6_NEXT_DESC_PTR,
        DMA1_7_NEXT_DESC_PTR,
        DMA1_8_NEXT_DESC_PTR,
        DMA1_9_NEXT_DESC_PTR,
        DMA1_10_NEXT_DESC_PTR,
        DMA1_11_NEXT_DESC_PTR,
        DMA2_0_NEXT_DESC_PTR,
        DMA2_1_NEXT_DESC_PTR,
        DMA2_2_NEXT_DESC_PTR,
        DMA2_3_NEXT_DESC_PTR,
        DMA2_4_NEXT_DESC_PTR,
        DMA2_5_NEXT_DESC_PTR,
        DMA2_6_NEXT_DESC_PTR,
        DMA2_7_NEXT_DESC_PTR,
        DMA2_8_NEXT_DESC_PTR,
        DMA2_9_NEXT_DESC_PTR,
        DMA2_10_NEXT_DESC_PTR,
        DMA2_11_NEXT_DESC_PTR,
        MDMA1_D0_NEXT_DESC_PTR,
        MDMA1_S0_NEXT_DESC_PTR,
        MDMA1_D1_NEXT_DESC_PTR,
        MDMA1_S1_NEXT_DESC_PTR,
        MDMA2_D0_NEXT_DESC_PTR,
        MDMA2_S0_NEXT_DESC_PTR,
        MDMA2_D1_NEXT_DESC_PTR,
        MDMA2_S1_NEXT_DESC_PTR,
        IMDMA_D0_NEXT_DESC_PTR,
        IMDMA_S0_NEXT_DESC_PTR,
        IMDMA_D1_NEXT_DESC_PTR,
        IMDMA_S1_NEXT_DESC_PTR,
};
#else
static unsigned int dma_iobase[] =
{
        DMA0_NEXT_DESC_PTR,
        DMA1_NEXT_DESC_PTR,
        DMA2_NEXT_DESC_PTR,
        DMA3_NEXT_DESC_PTR,
        DMA4_NEXT_DESC_PTR,
        DMA5_NEXT_DESC_PTR,
        DMA6_NEXT_DESC_PTR,
        DMA7_NEXT_DESC_PTR,
#if (defined(CONFIG_BF537) || defined(CONFIG_BF534) || defined(CONFIG_BF536))
        DMA8_NEXT_DESC_PTR,
        DMA9_NEXT_DESC_PTR,
        DMA10_NEXT_DESC_PTR,
        DMA11_NEXT_DESC_PTR,
#endif
        MDMA_D0_NEXT_DESC_PTR,
        MDMA_S0_NEXT_DESC_PTR,
        MDMA_D1_NEXT_DESC_PTR,
        MDMA_S1_NEXT_DESC_PTR
};
#endif

/* enable this define to get verbose debugging info */
//#define BFIN_SPI_DEBUG  1

#ifdef BFIN_SPI_DEBUG
#define PRINTK(args...) printk(args)
#else
#define PRINTK(args...)
#endif

/* constants for isr cycle averaging */

#define TC    1024 /* time constant    */
#define LTC   10   /* base 2 log of TC */

static u8 *iTxBuffer1;
static u8 *iRxBuffer1;

static int samples_per_chunk;
static int internalclock = 0;
static int bfsi_debug = 0;
static int init_ok = 0;

/* isr callback installed by user */

static void (*bfsi_isr_callback)(u8 *read_samples, u8 *write_samples) = NULL;
static irqreturn_t sport0_rx_isr(int irq, void *dev_id);

/* debug variables */

static int readchunk_first = 0;
static int readchunk_second = 0;
static int readchunk_didntswap = 0;
static unsigned char* lastreadchunk;

static int writechunk_first = 0;
static int writechunk_second = 0;
static int writechunk_didntswap = 0;
static unsigned char* lastwritechunk;

/* previous and worst case number of cycles we took to process an
   interrupt */

static u16 isr_cycles_last = 0;
static u32 isr_cycles_worst = 0;
static u32 isr_cycles_average = 0; /* scaled up by 2x */
static u32 echo_sams = 0;

static int reset_bit;

static unsigned int sport_dma_tx;
static unsigned int sport_dma_rx;

/* using SPI framework */
struct wc_spi_device {
	struct spi_device *fxs;
	struct spi_device *fxo;
};

struct wc_spi_device *wc_spi;

#define FXS_CHANNEL 0
#define FXO_CHANNEL 1

static int card_mode[]={FXS_CHANNEL,FXS_CHANNEL,FXO_CHANNEL,FXO_CHANNEL};
/* sample cycles register of Blackfin */

static inline unsigned int cycles(void) {
  int ret;

   __asm__ __volatile__
   (
   "%0 = CYCLES;\n\t"
   : "=&d" (ret)
   :
   : "R1"
   );

   return ret;
}

/*------------------------- SPI FUNCTIONS -----------------------------*/

/*
   After much experimentation I found that (i) TIMOD=00 (i.e. using
   read_RDBR() to start transfer) was the best way to start transfers
   and (ii) polling RXS was the best way to end transfers, see p10-30
   and p10-31 of BF533 data book.

   chip_select is the _number_ of the chip select line, e.g. to use
   SPISEL2 chip_select = 2.
*/
struct wcfxs;
static u8 fxs_address[] = {0x01,0x02,0xff,0xff};
static u8 fxo_address[] = {0xff,0xff,0x00,0x08};

static int wcfxs_setreg(struct wcfxs *wc, int card, unsigned reg, unsigned char value)
{
	unsigned char buf[3];
	int res;
#if defined(TEST_SPI_DELAY)
	unsigned int start,end;
#endif
	struct spi_transfer t = {
		.tx_buf = &buf,
		.len	= 3,
	};
	struct spi_message m;

#if defined(TEST_SPI_DELAY)
	start = cycles();
#endif
	spi_message_init(&m);
	if (card_mode[card] == FXS_CHANNEL)
	{
		buf[0]=fxs_address[card];
		buf[1]=reg & 0x7f;
		buf[2]=value;
		spi_message_add_tail(&t,&m);
		res = spi_sync(wc_spi->fxs,&m);
	}
	else if(card_mode[card] == FXO_CHANNEL)
	{
		buf[0]=  0x20 + fxo_address[card];
		buf[1] = reg &0x7f;
		buf[2] = value;
		spi_message_add_tail(&t,&m);
		res = spi_sync(wc_spi->fxo,&m);
	}
	else res= -1;
#if defined(TEST_SPI_DELAY)
	end = cycles();
	if(start <= end) start = end - start;
	else {
		start= 0xffffffff-start;
		start += end;
	}
	printk(KERN_DEBUG "SF-DTX %d \n",start);
#endif
	return res;
}

static unsigned char wcfxs_getreg(struct wcfxs *wc, int card, unsigned char reg)
{
	unsigned char buf[2],value;
	struct spi_transfer t = {
		.tx_buf	= &buf,
		.len	= 2,
	};
	struct spi_transfer r = {
		.rx_buf = &value,
		.len	= 1,
	};
	struct spi_message m;
#if defined(TEST_SPI_DELAY)
	unsigned int start,end;
#endif

#if defined(TEST_SPI_DELAY)
	start = cycles();
#endif
	spi_message_init(&m);
	if (card_mode[card] == FXS_CHANNEL)
	{
		buf[0] = fxs_address[card];
		buf[1] = reg | 0x80;
		spi_message_add_tail(&t,&m);
		spi_message_add_tail(&r,&m);
		spi_sync(wc_spi->fxs,&m);
	}
	else if (card_mode[card] == FXO_CHANNEL)
	{
		buf[0] = 0x60 + fxo_address[card];
		buf[1] = reg & 0x7f;
		spi_message_add_tail(&t, &m);
		spi_message_add_tail(&r, &m);
		spi_sync(wc_spi->fxo,&m);
	}
#if defined(TEST_SPI_DELAY)
	end = cycles();
	if(start <= end) start = end - start;
	else {
		start= 0xffffffff-start;
		start += end;
	}
	printk(KERN_DEBUG "SF-DRX %d \n",start);
#endif
	return value;
}

static int __wait_access(struct wcfxs *wc, int card)
{
	unsigned char data;
	long origjiffies;
	int count = 0;

	#define MAX 6000 /* attempts */

	origjiffies = jiffies;
	/* Wait for indirect access */
	while (count++ < MAX)
	{
		data = wcfxs_getreg(wc, card, I_STATUS);

		if (!data)
			return 0;
	}

	if (count > (MAX-1))
		printk(" ##### Loop error (%02x) #####\n", data);

	return 0;
}

static unsigned char translate_3215(unsigned char address)
{
	int x;
	for (x=0;x<sizeof(indirect_regs)/sizeof(indirect_regs[0]);x++) {
		if (indirect_regs[x].address == address) {
			address = indirect_regs[x].altaddr;
			break;
		}
	}
	return address;
}

static int wcfxs_proslic_setreg_indirect(struct wcfxs *wc, int card, unsigned char address, unsigned short data)
{
	int res=-1;
#if defined(TEST_SPI_DELAY)
	unsigned int start,end;
#endif

#if defined(TEST_SPI_DELAY)
	start = cycles();
#endif
	/* Translate 3215 addresses */
	if (wc->flags[card] & FLAG_3215) {
		address = translate_3215(address);
		if (address == 255)
			return 0;
	}

	if(!__wait_access(wc, card)) {
		res = wcfxs_setreg(wc,card,IDA_LO,(unsigned char)(data &0xff));
		if(res) return res;

		res = wcfxs_setreg(wc,card,IDA_HI,(unsigned char)((data &0xff00)>>8));
		if(res) return res;

		res = wcfxs_setreg(wc,card,IAA,address);
	}
#if defined(TEST_SPI_DELAY)
	end = cycles();
	if(start <= end) start = end - start;
	else {
		start= 0xffffffff-start;
		start += end;
	}
	printk(KERN_DEBUG "SF-ITX %d \n",start);
#endif
	return res;
}

static int wcfxs_proslic_getreg_indirect(struct wcfxs *wc, int card, unsigned char address)
{
	int res=-1;
	unsigned char data1,data2;
#if defined(TEST_SPI_DELAY)
	unsigned int start,end;
#endif

#if defined(TEST_SPI_DELAY)
	start = cycles();
#endif
	/* Translate 3215 addresses */
	if (wc->flags[card] & FLAG_3215) {
		address = translate_3215(address);
		if (address == 255)
			return 0;
	}
	if (!__wait_access(wc, card)) {
		res = wcfxs_setreg(wc,card,IAA,address);
		if(res)  return res;

		if (!__wait_access(wc, card)) {
			data1 = wcfxs_getreg(wc,card,IDA_LO);
			data2 = wcfxs_getreg(wc,card,IDA_HI);
			res = data1 | (data2 << 8);
		}
	}
#if defined(TEST_SPI_DELAY)
	end = cycles();
	if(start <= end) start = end - start;
	else {
		start= 0xffffffff-start;
		start += end;
	}
	printk(KERN_DEBUG "SF-IRX %d \n",start);
#endif
	return res;
}

/*
   chip_select_mask: the logical OR of all the chip selects we wish
   to use for SPI, for example if we wish to use SPISEL2 and SPISEL3
   chip_select_mask = (1<<2) | (1<<3).

   baud:  The SPI clk divider value, see Blackfin Hardware data book,
   maximum speed when baud = 2, minimum when baud = 0xffff (0 & 1
   disable SPI port).

   The maximum SPI clk for the Si Labs 3050 is 16.4MHz.  On a
   100MHz system clock Blackfin this means baud=4 minimum (12.5MHz).
*/
static int __devinit fxs_spi_probe(struct spi_device *spi)
{
	dev_set_drvdata(&spi->dev,wc_spi->fxs);
	wc_spi->fxs = spi;
	return 0;
}

static int __devinit fxo_spi_probe(struct spi_device *spi)
{
	dev_set_drvdata(&spi->dev,wc_spi->fxo);
	wc_spi->fxo = spi;
	return 0;
}

static struct spi_driver fxs_spi_driver = {
	.driver = {
		.name	= "fxs-spi",
		.bus 	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= fxs_spi_probe,
};

static struct spi_driver fxo_spi_driver = {
	.driver = {
		.name = "fxo-spi",
		.bus	= &spi_bus_type,
		.owner 	= THIS_MODULE,
	},
	.probe	= fxo_spi_probe,
};

static int bfsi_spi_init(struct wcfxs *wc)
{
	int flag,chip;
	//set daisy chain
	unsigned char buf[2],res;
	struct spi_transfer t = {
		.tx_buf = &buf,
		.len	= 2,
	};
	struct spi_message m;
  	wc_spi = kmalloc(sizeof(struct wc_spi_device),GFP_KERNEL);
  	if(!wc_spi)
		return -ENOMEM;
  	flag = spi_register_driver(&fxs_spi_driver);
  	if( flag )
	  	return -ENODEV;
  	flag = spi_register_driver(&fxo_spi_driver);
  	if(flag)
	  	return -ENODEV;

	spi_message_init(&m);
	buf[0]=0;
	buf[1]=0x85;
	spi_message_add_tail(&t,&m);
	spi_sync(wc_spi->fxs,&m);
	for(chip=0;chip<2;chip++)
	{
		wcfxs_setreg(wc,chip,2,0xaa);
		res = wcfxs_getreg(wc,chip,2);
		if(res == 0xaa){
			printk("chip_%d is successfully set to daisy chain!\n",chip);
		}
		else
			printk("chip_%d is failed set to daisy chain!\n",chip);
	}
/*
	printk("    FXO daisy chain ...\n");
	for(chip=2;chip<4;chip++)
	{
		wcfxs_setreg(wc,chip,20,0xaa);
		res = wcfxs_getreg(wc,chip,20);
		if(res == 0xaa)
			printk("        chip_%d is successfully set to daisy chain!\n",chip);
		else
			printk("        chip_%d is failed set to daisy chain!\n",chip);

	}
*/
  	return 0;
}

void bfsi_disable_spi(void)
{
	printk("spi_unregister_driver -- FXS ...\n");
	spi_unregister_driver(&fxs_spi_driver);
	printk("spi_unregister_driver -- FXO ...\n");
	spi_unregister_driver(&fxo_spi_driver);
	printk("kfree wc_spi!\n");
	kfree(wc_spi);
	return;
}

#if defined(CONFIG_BF533)
const unsigned short reset_port[] = { GPIO_0, GPIO_1, GPIO_2, GPIO_3, GPIO_4, GPIO_5, GPIO_6, GPIO_7 };
#endif
#if defined(CONFIG_BF537)
const unsigned short reset_port[] = { 0, GPIO_PF10, 0, 0, GPIO_PF6, GPIO_PF5, GPIO_PF4, 0 };
#endif
/*-------------------------- RESET FUNCTION ----------------------------*/
void bfsi_reset(void) {
#if defined(CONFIG_J16_JUMPER)
	reset_bit = 8-CONFIG_J16_JUMPER;
#else
	reset_bit = 8-4;
#endif
	if (reset_port[reset_bit] == 0)
                PRINTK("Error: cannot set reset to this bit! \n");
	gpio_request(reset_port[reset_bit],NULL);
	PRINTK("toggle reset\n");
	gpio_direction_output(reset_port[reset_bit]);
       	PRINTK("set reset bit OK! \n");
	gpio_set_value(reset_port[reset_bit],0);
  	udelay(100);
	gpio_set_value(reset_port[reset_bit],1);

  	/*
     	p24 3050 data sheet, allow 1ms for PLL lock, with
     	less than 1ms (1000us) I found register 2 would have
     	a value of 0 rather than 3, indicating a bad reset.
  	*/
  	udelay(1000);
}

void bfsi_reset_free(void){
	gpio_free(reset_port[reset_bit]);
}

/*-------------------------- SPORT FUNCTIONS ----------------------------*/

/* Init serial port but dont enable just yet, we need to set up DMA first */

static void init_sport0(void)
{
	/* set up FSYNC and optionally SCLK using Blackfin Serial port */

	/* Note: internalclock option not working at this stage - Tx side
	   appears not to work, e.g. TFS pin never gets asserted. Not a
	   huge problem as the BF internal clock is not at quite the
	   right frequency (re-crystal of STAMP probably required), so
	   we really need an external clock anyway.  However it would
	   be nice to know why it doesnt work! */

	if (internalclock) {
		bfin_write_SPORT0_RCLKDIV(24);  /* approx 2.048MHz PCLK            */
		bfin_write_SPORT0_RFSDIV(255);  /* 8 kHz FSYNC with 2.048MHz PCLK  */
	}
	else {
		bfin_write_SPORT0_RFSDIV(255);  /* 8 kHz FSYNC with 2.048MHz PCLK  */
	}

	/* external tx clk, not data dependant, MSB first */
	bfin_write_SPORT0_TCR2(7);      /* 8 bit word length      */
	bfin_write_SPORT0_TCR1(0);

	/* rx enabled, MSB first, internal frame sync     */
	bfin_write_SPORT0_RCR2(7);      /* 8 bit word length      */
	if (internalclock) {
		bfin_write_SPORT0_RCR1(IRFS | IRCLK);
	}
	else {
		bfin_write_SPORT0_RCR1(IRFS);
	}

	/* Enable MCM 8 transmit & receive channels       */
	bfin_write_SPORT0_MTCS0(0x000000FF);
	bfin_write_SPORT0_MRCS0(0x000000FF);

	/* MCM window size of 8 with 0 offset             */
	bfin_write_SPORT0_MCMC1(0x0000);

	/* 0 bit delay between FS pulse and first data bit,
	   multichannel frame mode enabled,
	   multichannel tx and rx DMA packing enabled */
	bfin_write_SPORT0_MCMC2(0x001c);
}

/* init DMA for autobuffer mode, but dont enable yet */

static int init_dma_wc(void)
{
	struct dma_register *dma_rx = (struct dma_register*) dma_iobase[sport_dma_rx];
	struct dma_register *dma_tx = (struct dma_register*) dma_iobase[sport_dma_tx];

        if (request_dma(sport_dma_rx, "SPORT RX Data") == -EBUSY) {
                printk(KERN_ERR "Failed to request RX dma %d\n", sport_dma_rx);
                return -EBUSY;
        }
        if (request_dma(sport_dma_tx, "SPORT TX Data") == -EBUSY) {
		printk(KERN_ERR "Failed to request TX dma %d\n", (unsigned int) dma_tx);
                return -EBUSY;
        }
        if (set_dma_callback(sport_dma_rx, sport0_rx_isr, NULL) != 0) {
		printk(KERN_ERR "Failed to request RX irq %d\n", (unsigned int) dma_rx);
                return -EBUSY;
        }

#if L1_DATA_A_LENGTH != 0
	iRxBuffer1 = (char*)l1_data_sram_alloc(2*samples_per_chunk*8);
	iTxBuffer1 = (char*)l1_data_sram_alloc(2*samples_per_chunk*8);
#else
  	dma_addr_t addr;
    	iRxBuffer1 = (char*)dma_alloc_coherent(NULL, 2*samples_per_chunk*8, &addr, 0);
    	iTxBuffer1 = (char*)dma_alloc_coherent(NULL, 2*samples_per_chunk*8, &addr, 0);
#endif
	if (bfsi_debug){
    		printk("iRxBuffer1 = 0x%x\n", (int)iRxBuffer1);
    		printk("iTxBuffer1 = 0x%x\n", (int)iTxBuffer1);
	}

	dma_rx->irq_status |= 0x02;
	dma_rx->start_addr = (unsigned long)iRxBuffer1;
	dma_rx->x_count = samples_per_chunk*8;
	dma_rx->x_modify = 1;
	dma_rx->y_count = 2;
	dma_rx->y_modify = 1;
 	dma_rx->cfg = WNR | WDSIZE_8 | DI_EN | 0x1000 | DI_SEL | DMA2D;

	dma_tx->start_addr = (unsigned long)iTxBuffer1;
 	dma_tx->x_count = samples_per_chunk*8;
	dma_tx->x_modify = 1;
	dma_tx->y_count = 2;
	dma_tx->y_modify = 1;
	dma_tx->cfg = WDSIZE_8 | 0x1000 | DMA2D;

  	/* init test variables */
  	lastreadchunk = (unsigned char*)&iRxBuffer1[8*samples_per_chunk];
  	lastwritechunk = (unsigned char*)&iTxBuffer1[8*samples_per_chunk];
	return 0;
}

/* works out which write buffer is available for writing */
static u8 *isr_write_processing(void) {
	u8 *writechunk;
	int x;
	struct dma_register *dma_tx = (struct dma_register*) dma_iobase[sport_dma_tx];

	/* select which ping-pong buffer to write to */
	x = dma_tx->curr_addr_ptr  - (int)iTxBuffer1;

	/* for some reason x for tx tends to be 0xe and 0x4e, whereas
	   x for rx is 0x40 and 0x80.  Note sure why they would be
	   different.  We could perhaps consider having
	   different interrupts for tx and rx side.  Hope this
	   offset doesnt kill the echo cancellation, e.g. if we
	   get echo samples in rx before tx has sent them!
	*/
	if (x >= 8*samples_per_chunk) {
		writechunk = (unsigned char*)iTxBuffer1;
		writechunk_first++;
	}
	else {
		writechunk = (unsigned char*)iTxBuffer1 + samples_per_chunk*8;
		writechunk_second++;
	}

	/* make sure writechunk actually ping pongs */

	if (writechunk == lastwritechunk) {
		writechunk_didntswap++;
	}
	lastwritechunk = (unsigned char*)writechunk;

	return writechunk;
}

/* works out which read buffer is available for reading */
static u8 *isr_read_processing(void) {
	u8 *readchunk;
	int x;
	struct dma_register *dma_rx = (struct dma_register*) dma_iobase[sport_dma_rx];

	/* select which ping-pong buffer to write to */
	x = dma_rx->curr_addr_ptr  - (int)iRxBuffer1;

	/* possible values for x are 8*samples_per_chunk=0x40 at the
	   end of the first row and 2*8*samples_per_chunk=0x80 at the
	   end of the second row */
	if (x == 8*samples_per_chunk) {
		readchunk = iRxBuffer1;
		readchunk_first++;
	}
	else {
		readchunk = iRxBuffer1 + samples_per_chunk*8;
		readchunk_second++;
	}

	/* make sure readchunk actually ping pongs */

	if (readchunk == lastreadchunk) {
		readchunk_didntswap++;
	}
	lastreadchunk = (unsigned char*)readchunk;

	return readchunk;
}

/* called each time the DMA finishes one "line" */
static irqreturn_t sport0_rx_isr(int irq, void *dev_id)
{
  	unsigned int  start_cycles = cycles();
  	u8 *read_samples;
  	u8 *write_samples;
	struct dma_register *dma_rx = (struct dma_register*) dma_iobase[sport_dma_rx];

	dma_rx->irq_status = 0x0001;

  	read_samples = isr_read_processing();
  	write_samples = isr_write_processing();
  	if (bfsi_isr_callback != NULL) {
    		bfsi_isr_callback(read_samples, write_samples);
	}

  	isr_cycles_average = ( (u32)(TC-1)*isr_cycles_average +
			 (((u32)isr_cycles_last)<<1) + TC) >> LTC;

  	if (isr_cycles_last > isr_cycles_worst)
    	isr_cycles_worst = isr_cycles_last;

  	/* we sample right at the end to make sure we count cycles used to
     		measure cycles! */
  	isr_cycles_last = cycles() - start_cycles;

  	return IRQ_HANDLED;
}

static void enable_dma_sport0(void)
{
	/* enable DMAs */
	enable_dma(sport_dma_rx);
	enable_dma(sport_dma_tx);
#if defined(CONFIG_BF537)
	bfin_write_PORT_MUX(bfin_read_PORT_MUX() & ~(PJSE|PJCE(3)));
	SSYNC();
#endif
	/* enable sport0 Tx and Rx */
	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() | TSPEN);
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() | RSPEN);
	SSYNC();
}

static void disable_sport0(void)
{
	/* disable sport0 Tx and Rx */
	bfin_write_SPORT0_TCR1(bfin_read_SPORT0_TCR1() & (~TSPEN));
	bfin_write_SPORT0_RCR1(bfin_read_SPORT0_RCR1() & (~RSPEN));
	SSYNC();
	free_dma(sport_dma_tx);
	free_dma(sport_dma_rx);
}

int bfsi_proc_read(char *buf, char **start, off_t offset,
		    int count, int *eof, void *data)
{
	int len;

	len = sprintf(buf,
		      "readchunk_first.....: %d\n"
		      "readchunk_second....: %d\n"
		      "readchunk_didntswap.: %d\n"
		      "writechunk_first....: %d\n"
		      "writechunk_second...: %d\n"
		      "writechunk_didntswap: %d\n"
		      "isr_cycles_last.....: %d\n"
		      "isr_cycles_worst....: %d\n"
		      "isr_cycles_average..: %d\n"
		      "echo_sams...........: %d\n",
		      readchunk_first,
		      readchunk_second,
		      readchunk_didntswap,
		      writechunk_first,
		      writechunk_second,
		      writechunk_didntswap,
		      isr_cycles_last,
		      isr_cycles_worst,
		      isr_cycles_average>>1,
		      echo_sams);

	*eof=1;
	return len;
}

/*
   Wrapper for entire SPORT setup, returns 1 for success, 0 for failure.

   The SPORT code is designed to deliver small arrays of size samples
   every (125us * samples).  A ping-pong arrangement is used, so the
   address of the buffer will alternate every call between two possible
   values.

   The callback functions privide to the address of the current buffer
   for the read and write channels.  Read means the data was just
   read from the SPORT, so this is the "receive" PCM samples.  Write
   is the PCM data to be written to the SPORT.

   The callbacks are called in the context of an interrupt service
   routine, so treat any code them like an ISR.

   Once this function returns successfully the SPORT/DMA will be up
   and running, and calls to the isr callback will start.  For testing
   it is OK to set the callback function pointer to NULL, say if you
   just want to look at the debug information.

   If debug==1 then "cat /proc/bfsi" will display some debug
   information, something like:

     readchunk_first.....: 9264
     readchunk_second....: 9264
     readchunk_didntswap.: 0
     writechunk_first....: 9264
     writechunk_second...: 9264
     writechunk_didntswap: 0

   If all is well then "readchunk_didntswap" and "writechunk_didntswap"
   will be static and some very small number.  The first and second
   values should be at most one value different.  These variables
   indicate sucessful ping-pong operation.

   The numbers are incremented ever interrupt, for example if samples=8
   (typical for zaptel), then we get one interrupt every ms, or 1000
   interrupts per second.  This means the values for each first/second
   entry should go up 500 times per second.

   8 channels are sampled at once, so the size of the samples buffers
   is 8*samples (typically 64 bytes for zaptel).

   TODO:

   1/ It might be nice to modify this function allow user defined
      SPORT control reg settings, for example to change clock
      dividers and frame sync sources.  Or posible provide
      a bfsi_sport_set() function.

   2/ Modify the callbacks to provide user-dfine context information.

   3/ Modify init to define max number of channels, it is currently
      hard coded at 8.
*/

int bfsi_sport_init(
  void (*isr_callback)(u8 *read_samples, u8 *write_samples),
  int samples,
  int debug
)
{
  if (debug) {
    create_proc_read_entry("bfsi", 0, NULL, bfsi_proc_read, NULL);
    bfsi_debug = debug;
  }

  sport_dma_tx = CH_SPORT0_TX;
  sport_dma_rx = CH_SPORT0_RX;

  bfsi_isr_callback = isr_callback;
  samples_per_chunk = samples;

  init_sport0();
  if(init_dma_wc())
	  init_ok = 0;
  else
	  init_ok = 1;
  enable_dma_sport0();

  return init_ok;
}

/* shut down SPORT operation cleanly */

void bfsi_sport_close(void)
{
  disable_sport0();
  bfsi_reset_free();

  if (init_ok) {
    free_irq(IRQ_SPORT0_RX, NULL);
  }
#if L1_DATA_A_LENGTH != 0
  l1_data_sram_free(iTxBuffer1);
  l1_data_sram_free(iRxBuffer1);
#else
  dma_free_coherent(NULL, 2*samples_per_chunk*8, iTxBuffer1, 0);
  dma_free_coherent(NULL, 2*samples_per_chunk*8, iRxBuffer1, 0);
#endif
  remove_proc_entry("bfsi", NULL);
}
