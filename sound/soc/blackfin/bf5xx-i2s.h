/*
 * linux/sound/arm/bf5xx-i2s.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BF5XX_I2S_H
#define _BF5XX_I2S_H

#define NUM_SPORT_I2S	4 /* lrg - must be cpu dependant way to select this */

extern struct snd_soc_cpu_dai bf5xx_i2s_dai;

struct audio_frame {
	u32 left_channel;
	u32 right_channel;
};

void bf5xx_pcm_to_frame(struct audio_frame *dst, const __u32 *src, \
		size_t count);

void bf5xx_frame_to_pcm(const struct audio_frame *src, __u32 *dst, \
		size_t count);

#endif
