/*
 * File: include/asm-blackfin/mach-bf518/anomaly.h
 * Bugs: Enter bugs at http://blackfin.uclinux.org/
 *
 * Copyright (C) 2004-2008 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

/* This file shoule be up to date with:
 * TODO
 */

#ifndef _MACH_ANOMALY_H_
#define _MACH_ANOMALY_H_

/* Errors when SSYNC, CSYNC, or Loads to LT, LB and LC Registers Are Interrupted */
#define ANOMALY_05000312 (1)
/* bfrom_SysControl() Firmware Function Performs Improper System Reset */
#define ANOMALY_05000353 (1)
/* bfrom_SysControl() Firmware Routine Not Functional */
#define ANOMALY_05000386 (0)

/* Anomalies that don't exist on this proc */
#define ANOMALY_05000125 (0)
#define ANOMALY_05000158 (0)
#define ANOMALY_05000183 (0)
#define ANOMALY_05000198 (0)
#define ANOMALY_05000230 (0)
#define ANOMALY_05000244 (0)
#define ANOMALY_05000261 (0)
#define ANOMALY_05000263 (0)
#define ANOMALY_05000266 (0)
#define ANOMALY_05000273 (0)
#define ANOMALY_05000285 (0)
#define ANOMALY_05000307 (0)
#define ANOMALY_05000311 (0)
#define ANOMALY_05000323 (0)
#define ANOMALY_05000363 (0)

#endif
