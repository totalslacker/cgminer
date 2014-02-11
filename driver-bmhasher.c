/*
 * cgminer driver for BMHasher boards
 *
 * Copyright 2014 Shannon Holand <shannonholl@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "logging.h"
#include "miner.h"
#include "util.h"


/* Probe SPI channel and register chip chain */
void BMH_detect(bool hotplug)
{
}

/* return value is nonces processed since previous call */
static int64_t BMH_scanwork(struct thr_info *thr)
{
	return 0;
}

/* queue two work items per chip in chain */
static bool BMH_queue_full(struct cgpu_info *cgpu)
{
	int queue_full = false;

	return queue_full;
}

static void BMH_flush_work(struct cgpu_info *cgpu)
{
}

struct device_drv bmhasher_drv = {
	.drv_id = DRIVER_bmhasher,
	.dname = "BMHasher",
	.name = "BMH",
	.drv_detect = BMH_detect,

	.hash_work = hash_queued_work,
	.scanwork = BMH_scanwork,
	.queue_full = BMH_queue_full,
	.flush_work = BMH_flush_work,
};
