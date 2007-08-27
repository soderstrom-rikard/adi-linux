#include <linux/kernel.h>	/* printk() */
#include <linux/delay.h>
#include <linux/errno.h>
#include <asm/blackfin.h>
#include <asm/gpio.h>
#include <asm/portmux.h>

#define SPI0_REGBASE       0xffc00500
#define DRV_NAME "mmc_pio_spi"

#define DEFINE_SPI_REG(reg, off) \
static inline u16 read_##reg(void) \
	{ return bfin_read16(SPI0_REGBASE + off); } \
static inline void write_##reg(u16 v) \
	{bfin_write16(SPI0_REGBASE + off, v); }

#define POLL_TIMEOUT 1000000000

DEFINE_SPI_REG(CTRL, 0x00)
DEFINE_SPI_REG(FLAG, 0x04)
DEFINE_SPI_REG(STAT, 0x08)
DEFINE_SPI_REG(TDBR, 0x0C)
DEFINE_SPI_REG(RDBR, 0x10)
DEFINE_SPI_REG(BAUD, 0x14)
DEFINE_SPI_REG(SHAW, 0x18)

static int cs_unselect_delay_ns;
static unsigned short spi_ssel;

void bfin_pio_spi_set_hz(unsigned int hz)
{
	u_long sclk = get_sclk();
	unsigned short baud = (sclk / (2 * hz));

	if ((sclk % (2 * hz)) > 0)
		baud++;

	write_BAUD(baud);
	SSYNC();
}

void bfin_pio_spi_select(void)
{
	write_STAT(0xFFFF);
	write_CTRL(0x5009);
	SSYNC();

	gpio_set_value(spi_ssel, 0);

	return;
}

void bfin_pio_spi_unselect(void)
{
	// cs_unselect_delay_ns is filescoped and set to 0 when not accessing MACs
	if (cs_unselect_delay_ns > 0)
		ndelay(cs_unselect_delay_ns);

	gpio_set_value(spi_ssel, 1);
	SSYNC();

	return;
}

unsigned int bfin_pio_spi_transfer(unsigned char *in, unsigned char out)
{
	/*
	   static unsigned char sr;
	   static unsigned char sd;
	 */
	unsigned int tc = POLL_TIMEOUT;
	unsigned short SPI;

	// Transmit data.
	write_TDBR(out);

	// Wait until d is transmitted.
	SPI = read_STAT();

	while ((SPI & 0x0008) && tc--) {
		SPI = read_STAT();
	}

	// Wait until recv buffer is ready.
	tc = POLL_TIMEOUT;
	while (!(SPI & 0x20) && tc--) {
		SPI = read_STAT();
	}
	if (!tc) {
		return 0;
	}
	// Read out recv buffer.
	*in = read_RDBR();

	// TODO: error handler
	return 1;
}

void print_spi_debug_info(void)
{
	printk
	    (KERN_INFO DRV_NAME":cs_#: 0x%x, baud=0x%x, flag=0x%x, ctrl=0x%x\n",
	     spi_ssel, read_BAUD(), read_FLAG(), read_CTRL());
}

// bdrate=0x100, flag=0xBF40, control_reg=0x1002, PORTF_FER=0x385F, PORT_MUX=0xA0
void bfin_pio_spi_init(int cs_chan)
{

	u16 pin_req[] = { P_SPI0_SCK, P_SPI0_MISO, P_SPI0_MOSI, 0 };
	spi_ssel = (u16) cs_chan;

	print_spi_debug_info();

	// Set FLAG to 0xFF00 to drive CS manually using GPIOS
	write_FLAG(0xFF00);
	write_CTRL(0x5009);

	if (peripheral_request_list(pin_req, DRV_NAME))
		printk(KERN_ERR DRV_NAME":Requesting Peripherals failed\n");

	if (gpio_request(spi_ssel, DRV_NAME)) {
		printk(KERN_ERR DRV_NAME":bfin_pio_spi: Failed ro request GPIO_%d\n",
		       spi_ssel);
	}
	gpio_direction_output(spi_ssel);

	write_BAUD(0x0100);

	SSYNC();
	bfin_pio_spi_unselect();
}
