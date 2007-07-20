#include <linux/kernel.h>	/* printk() */
#include <linux/delay.h>

#define SPI0_REGBASE       0xffc00500

static volatile unsigned short *pFIO_FLAG_S = (volatile unsigned short*)0xFFC00708;
static volatile unsigned short *pFIO_FLAG_C = (volatile unsigned short*)0xFFC00704;

#define ssync() __builtin_bfin_ssync()

#define DEFINE_SPI_REG(reg, off) \
static inline u16 read_##reg(void) \
            { return *(volatile unsigned short*)(SPI0_REGBASE + off); } \
static inline void write_##reg(u16 v) \
            {*(volatile unsigned short*)(SPI0_REGBASE + off) = v;\
             ssync();}

#define POLL_TIMEOUT 1000000000

DEFINE_SPI_REG(CTRL, 0x00)
DEFINE_SPI_REG(FLAG, 0x04)
DEFINE_SPI_REG(STAT, 0x08)
DEFINE_SPI_REG(TDBR, 0x0C)
DEFINE_SPI_REG(RDBR, 0x10)
DEFINE_SPI_REG(BAUD, 0x14)
DEFINE_SPI_REG(SHAW, 0x18)

static int cs_unselect_delay_ns = 0;
static unsigned short spi_mmc_cs_chan_mask = 0;

void bfin_pio_spi_set_hz(unsigned int hz)
{
    u_long sclk = get_sclk();
    unsigned short baud = (sclk / (2 * hz));

        if ((sclk % (2 * hz)) > 0)
                baud++;

	write_BAUD(baud);
	ssync();
}

void bfin_pio_spi_select( void )
{
	write_STAT(0xFFFF);
	write_CTRL(0x5009);
	ssync();

	*pFIO_FLAG_C = spi_mmc_cs_chan_mask;
	
	return;
}

void bfin_pio_spi_unselect( void )
{
	// cs_unselect_delay_ns is filescoped and set to 0 when not accessing MACs
	if(cs_unselect_delay_ns>0)
        ndelay(cs_unselect_delay_ns);

	*pFIO_FLAG_S = spi_mmc_cs_chan_mask;

	ssync();

	return;
}

unsigned int bfin_pio_spi_transfer(unsigned char* in, unsigned char out)
{
	/*
	static unsigned char sr;
	static unsigned char sd;
	*/
	unsigned int tc=POLL_TIMEOUT;
	unsigned short SPI;

	// Transmit data.
	write_TDBR(out);

	// Wait until d is transmitted.
	SPI = read_STAT();

	while((SPI & 0x0008) && tc--)
	{ 
		SPI = read_STAT();
	}

	// Wait until recv buffer is ready.
	tc=POLL_TIMEOUT;
	while( !(SPI & 0x20) && tc-- )
	{ 
		SPI = read_STAT();
	}
	if(!tc) {
		return 0;
    }
	// Read out recv buffer.
	*in = read_RDBR();

	// TODO: error handler
	return 1;
}

void print_spi_debug_info(void)
{
    printk("cs_mask: 0x%x, baud=0x%x, flag=0x%x, ctrl=0x%x, mux= 0x%x, fer=0x%x, dir=0x%x\n",
                spi_mmc_cs_chan_mask,
                read_BAUD(),
                read_FLAG(),
                read_CTRL(),
                bfin_read_PORT_MUX(),
                bfin_read_PORTF_FER(),
                bfin_read_FIO_DIR());

    while(0);
}

// bdrate=0x100, flag=0xBF40, control_reg=0x1002, PORTF_FER=0x385F, PORT_MUX=0xA0
void bfin_pio_spi_init(int cs_chan)
{
	spi_mmc_cs_chan_mask = 1 << cs_chan;
    
 //   unsigned short baud, flag, ctrl, fer, mux;

    print_spi_debug_info();

	// Set FLAG to 0xFF00 to drive CS manually using GPIOS
	write_FLAG(0xFF00);	
    write_CTRL(0x5009);

#if defined(CONFIG_BF534)|defined(CONFIG_BF536)|defined(CONFIG_BF537)
    // Enable MOSI,MISO and SCLK
    bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 1 << 11); 
    bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 1 << 12); 
    bfin_write_PORTF_FER(bfin_read_PORTF_FER() | 1 << 13);

    // Enable correct PFx pin as chip select signal
	bfin_write_PORTF_FER(bfin_read_PORTF_FER() & ~spi_mmc_cs_chan_mask);
	SSYNC();
#else
	// enable PFx as GPIO
	bfin_write_PORT_FER(bfin_read_PORT_FER() & ~spi_mmc_cs_chan_mask);
#endif

	// set PFx direction to OUTPUT
	bfin_write_FIO_DIR(bfin_read_FIO_DIR() | spi_mmc_cs_chan_mask);
	
	write_BAUD(0x0100);

	ssync();
	bfin_pio_spi_unselect( ) ;
}
