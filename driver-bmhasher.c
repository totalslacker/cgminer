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


/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}


struct BMH_module {
	int num_cores;
	int last_queued_id;
	struct work *work[4];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
};

struct BMH_chain {
	struct cgpu_info *cgpu;
	int num_modules;
	int num_cores;
	struct BMH_module *modules;
	pthread_mutex_t lock;

	struct work_queue active_wq;
};

/********** driver interface */
void exit_BMH_chain(struct BMH_chain *bmh)
{
	if (bmh == NULL)
		return;
	free(bmh->modules);
	bmh->modules = NULL;
	free(bmh);
}


struct BMH_chain *init_BMH_chain(void *ctx)
{
	int i;
	struct BMH_chain *bmh = malloc(sizeof(*bmh));
	assert(bmh != NULL);

	applog(LOG_DEBUG, "BMH init chain");
	memset(bmh, 0, sizeof(*bmh));

	// reset all boards
	// if (!cmd_RESET_BCAST(a1) || !cmd_BIST_START(a1))
	// 	goto failure;

	bmh->num_modules = 1;
	bmh->num_cores = 1;

	applog(LOG_WARNING, "BMH: Found %d BMH modules",
	       bmh->num_modules);

	// if (bmh->num_modules == 0) {
	// 	bmh->num_modules = manual_chain_detect(bmh);
	// 	if (bmh->num_modules == 0)
	// 		goto failure;
	// }

	// if (!set_pll_config(a1, config_options.ref_clk_khz,
	// 		    config_options.sys_clk_khz))
	// 	goto failure;

	bmh->modules = calloc(bmh->num_modules, sizeof(struct BMH_module));
	if (bmh->modules == NULL) {
		applog(LOG_ERR, "BMH_chips allocation error");
		goto failure;
	}
	// if (!cmd_BIST_FIX_BCAST(a1))
	// 	goto failure;

	// for (i = 0; i < a1->num_modules; i++) {
	// 	int chip_id = i + 1;
	// 	if (!cmd_READ_REG(a1, chip_id)) {
	// 		applog(LOG_WARNING, "Failed to read register for "
	// 		       "chip %d -> disabling", chip_id);
	// 		a1->chips[i].num_cores = 0;
	// 		continue;
	// 	}
	// 	a1->chips[i].num_cores = a1->spi_rx[7];
	// 	a1->num_cores += a1->chips[i].num_cores;
	// 	applog(LOG_WARNING, "Found chip %d with %d active cores",
	// 	       chip_id, a1->chips[i].num_cores);
	// }
	applog(LOG_WARNING, "Found %d modules with total %d active cores",
	       bmh->num_modules, bmh->num_cores);

	mutex_init(&bmh->lock);
	INIT_LIST_HEAD(&bmh->active_wq.head);

	return bmh;

failure:
	exit_BMH_chain(bmh);
	return NULL;
}

static bool BMH_detect_one_chain(void *cfg)
{
	struct cgpu_info *cgpu;
	// struct spi_ctx *ctx = spi_init(cfg);

	// if (ctx == NULL)
	// 	return false;

	// struct BMH_chain *bmh = init_BMH_chain(ctx);
	struct BMH_chain *bmh = init_BMH_chain(NULL);
	if (bmh == NULL)
		return false;

	cgpu = malloc(sizeof(*cgpu));
	assert(cgpu != NULL);

	memset(cgpu, 0, sizeof(*cgpu));
	cgpu->drv = &bmhasher_drv;
	cgpu->name = "BMH";
	cgpu->threads = 1;

	cgpu->device_data = bmh;

	bmh->cgpu = cgpu;
	add_cgpu(cgpu);

	return true;
}

/* Probe SPI channel and register chip chain */
void BMH_detect(bool hotplug)
{
	int bus;
	int cs_line;

	/* no hotplug support for now */
	if (hotplug)
		return;

	// if (opt_bitmine_a1_options != NULL && parsed_config_options == NULL) {
	// 	int ref_clk;
	// 	int sys_clk;
	// 	int spi_clk;
	// 	int override_chip_num;

	// 	sscanf(opt_bitmine_a1_options, "%d:%d:%d:%d",
	// 	       &ref_clk, &sys_clk, &spi_clk,  &override_chip_num);
	// 	if (ref_clk != 0)
	// 		config_options.ref_clk_khz = ref_clk;
	// 	if (sys_clk != 0)
	// 		config_options.sys_clk_khz = sys_clk;
	// 	if (spi_clk != 0)
	// 		config_options.spi_clk_khz = spi_clk;
	// 	if (override_chip_num != 0)
	// 		config_options.override_chip_num = override_chip_num;

	// 	/* config options are global, scan them once */
	// 	parsed_config_options = &config_options;
	// }

	applog(LOG_DEBUG, "BMH_detect");
	// BMH_detect_one_chain(&cfg);
	BMH_detect_one_chain(NULL);
	// A1_hw_reset();
	// for (bus = 0; bus < MAX_SPI_BUS; bus++) {
	// 	for (cs_line = 0; cs_line < MAX_SPI_CS; cs_line++) {
	// 		struct spi_config cfg = default_spi_config;
	// 		cfg.mode = SPI_MODE_1;
	// 		cfg.speed = config_options.spi_clk_khz * 1000;
	// 		cfg.bus = bus;
	// 		cfg.cs_line = cs_line;
	// 		A1_detect_one_chain(&cfg);
	// 	}
	// }
}

/* return value is nonces processed since previous call */
static int64_t BMH_scanwork(struct thr_info *thr)
{
	applog(LOG_DEBUG, "BMH_scanwork");

	return 0;
}

/* queue two work items per chip in chain */
static bool BMH_queue_full(struct cgpu_info *cgpu)
{
	int queue_full = false;

	applog(LOG_DEBUG, "BMH_queue_full");

	return queue_full;
}

static void BMH_flush_work(struct cgpu_info *cgpu)
{
	applog(LOG_DEBUG, "BMH_flush_work");
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
