#include <asm/blackfin.h>
#include <linux/kernel.h>

void coreb_enable(void)
{
	bfin_write32(RCU0_SVECT1, COREB_L1_CODE_START);
	bfin_write32(RCU0_CRCTL, 0);
}

void bfin_coreb_start(void)
{
	bfin_write32(RCU0_CRCTL, 0x2);

	while (!(bfin_read32(RCU0_CRSTAT) & 0x2))
		continue;

	bfin_write32(RCU0_CRCTL, 0);
}

void bfin_coreb_stop(void)
{
	bfin_write32(RCU0_CRCTL, 0);
}

void bfin_coreb_reset(void)
{
	bfin_write32(RCU0_CRCTL, 0x2);

	while (!(bfin_read32(RCU0_CRSTAT) & 0x2))
		continue;

	bfin_write32(RCU0_CRCTL, 0);
}
