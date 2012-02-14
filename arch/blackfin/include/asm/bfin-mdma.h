
/*** User space interface: START ***/
struct user_mdma_state {
	unsigned int channel;
	volatile int done;
	struct dmasg dsc_src, dsc_dst;
};

#define BF_DMA_REQUEST _IOW('D', 0x00, struct user_dma_state)
#define BF_DMA_FREE    _IOW('D', 0x01, struct user_dma_state)
#define BF_DMA_RUN     _IOW('D', 0x02, struct user_dma_state)
#define BF_DMA_ARUN    _IOW('D', 0x03, struct user_dma_state)
/*** User space interface: END ***/

