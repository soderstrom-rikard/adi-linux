/*
 * volt.h - sysfs attributes associated with ADCs or other voltage sensors
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Copyright (c) 2008 Jonathan Cameron <jic23@cam.ac.uk>
 *
 */

#define IIO_DEV_ATTR_VOLT(_name, _show, _addr)			\
  IIO_DEVICE_ATTR(volt_##_name, S_IRUGO, _show, NULL, _addr)
