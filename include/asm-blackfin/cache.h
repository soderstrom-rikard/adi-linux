/*
 * include/asm-blackfin/cache.h
 */
#ifndef __ARCH_BLACKFIN_CACHE_H
#define __ARCH_BLACKFIN_CACHE_H

/*
 * Bytes per L1 cache line
 * Blackfin loads 32 bytes for cache
 */
#define L1_CACHE_SHIFT	5
#define L1_CACHE_BYTES	(1 << L1_CACHE_SHIFT)

/*
 * Don't make __cacheline_aligned and
 * ____cacheline_aligned defined in include/linux/cache.h
 */
#define __cacheline_aligned
#define ____cacheline_aligned

/*
 * largest L1 which this arch supports
 */
#define L1_CACHE_SHIFT_MAX	5

#endif
