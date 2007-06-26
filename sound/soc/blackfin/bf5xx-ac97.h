/*
 * linux/sound/arm/bf5xx-ac97.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BF5XX_AC97_H
#define _BF5XX_AC97_H

extern struct snd_ac97_bus_ops bf5xx_ac97_ops;

/*
 * frame communicated over the ac97 sport link
 */
struct ac97_frame {
	__u16 ac97_tag;		/* slot 0 */
#define TAG_VALID 0x8000
#define TAG_CMD   0x6000
#define TAG_PCM_LEFT   0x1000
#define TAG_PCM_RIGHT  0x0800
#define TAG_PCM  (TAG_PCM_LEFT|TAG_PCM_RIGHT)
	__u16 ac97_addr;	/* slot 1 */
	__u16 ac97_data;	/* slot 2 */
	__u32 ac97_pcm;		/* slot 3 and 4: left and right pcm data */
	__u16 stuff[11];	/* pad to 16 words */
} __attribute__ ((packed));

struct sport_ac97 {
	__u16	register_dirty[128/16];
	int *cmd_count;
};
extern struct snd_soc_cpu_dai bfin_ac97_dai;

void bf5xx_ac97_pcm16_to_frame(struct ac97_frame *dst, const __u16 *src, \
		size_t count);

void bf5xx_ac97_frame_to_pcm16(const struct ac97_frame *src, __u16 *dst, \
		size_t count);

#endif
