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
#include "usbutils.h"

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

	struct BMH_chain * chain;

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

/********** temporary helper for hexdumping SPI traffic */
#define DEBUG_HEXDUMP 1
static void hexdump(char *prefix, uint8_t *buff, int len)
{
#if DEBUG_HEXDUMP
	static char line[2048];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0)
			pos += sprintf(pos, "\n\t");
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(LOG_DEBUG, "%s", line);
#endif
}


enum PacketTypes
{
	kHelloPacketType,
	kWorkPacketType,
	kRequestNoncesPacketType,
	kStopPacketType,
};

typedef struct BMPacketHeader
{
	uint8_t		type;
	uint8_t		address;
	uint8_t		length;
	uint8_t		flags;
	uint16_t	crc16;
} BMPacketHeader;

typedef struct BMPacket
{
	BMPacketHeader	header;
	uint8_t			payload[];
} BMPacket;

typedef struct HelloResponsePacket
{
	BMPacketHeader	header;
	uint8_t			desiredQueueLength;
} HelloResponsePacket;

typedef struct WorkPacket
{
	BMPacketHeader	header;
	uint8_t			midstate[32];
	uint8_t			data[12];
} WorkPacket;

typedef struct StatusResponsePacket
{
	BMPacketHeader	header;
	uint8_t			remainingWork;
	uint8_t			desiredWork;
	uint8_t			remainingNonces;
	uint8_t			nonceCount;
	uint32_t		nonces[16];
} StatusResponsePacket;

uint16_t crc16(uint16_t crcval, void *data_p, int count)
{
    /* CRC-16 Routine for processing multiple part data blocks.
     * Pass 0 into 'crcval' for first call for any given block; for
     * subsequent calls pass the CRC returned by the previous call. */
    int 		xx;
    uint8_t * 	ptr = data_p;

    while (count-- > 0)
    {
        crcval = (uint16_t)( crcval ^ (uint16_t)(((uint16_t) *ptr++) << 8));
        for (xx=0; xx < 8; xx++)
        {
            if (crcval & 0x8000)
            {
            	crcval=(uint16_t) ((uint16_t) (crcval << 1) ^ 0x1021);
            }
            else 
            {
            	crcval = (uint16_t) (crcval << 1);
            }
        }
    }

    return (crcval);
}

int CRCPacket(BMPacket * packet, uint16_t * crc)
{
	*crc = 0xFFFF;

	// hash header except crc
	*crc = crc16(*crc, packet, sizeof(BMPacketHeader) - 2);

	// hash payload
	if (packet->header.length > 0)
	{
		*crc = crc16(*crc, packet->payload, packet->header.length - 2);
	}

	return 0;
}

int BuildPacket(BMPacket * packet, int length)
{
	packet->header.length = length;
	CRCPacket(packet, &packet->header.crc16);

	return 0;
}

int CheckPacket(BMPacket * packet)
{
	uint16_t crc;

	CRCPacket(packet, &crc);

	return (crc == packet->header.crc16) ? 0 : -1;
}

int SendPacket(struct BMH_chain * bmh, int moduleIndex, BMPacket * packet)
{
	int len, err, tried;
	int sendLength;
	int amountSent;

	// FIXME: Set this correctly...
	packet->header.address = moduleIndex;
	BuildPacket(packet, packet->header.length);
	sendLength = packet->header.length + sizeof(BMPacketHeader);

	hexdump("SendPacket", (uint8_t *) packet, sendLength);

	err = usb_write(bmh->cgpu, (char *) packet, sendLength, &amountSent, C_BMHASHER);
	if (err < 0 || amountSent < sendLength)
	{
		if (err > 0)
		{
			err = -1;
		}
		// N.B. thus !(*sent) directly implies err < 0 or *amount < send_len
		return err;
	}

	applog(LOG_DEBUG, "SendPacket done");

	return 0;
}

int ReceivePacket(struct BMH_chain * bmh, BMPacket * packet)
{
	int len, err, tried;
	int rxLength;

	applog(LOG_DEBUG, "ReceivePacket");

	for (int i = 0; i < 60; i++)
	{
		err = usb_read_ok_timeout(bmh->cgpu, (char *) packet, sizeof(BMPacketHeader), &rxLength, 999, C_BMHASHER);

		if (err >= 0 && rxLength == sizeof(BMPacketHeader))
		{
			break;
		}
		if ((err < 0 && err != LIBUSB_ERROR_TIMEOUT))
		{
			applog(LOG_DEBUG, "ReceivePacket: read error");
			// N.B. thus !(*sent) directly implies err < 0 or *amount < send_len
			return err;
		}

		cgsleep_ms(100);
	}

	// if still no header, then error
	if (rxLength < sizeof(BMPacketHeader))
	{
		applog(LOG_DEBUG, "ReceivePacket: packet header timeout");
		return LIBUSB_ERROR_TIMEOUT;
	}

	hexdump("ReceivePacket header", (uint8_t *) packet, sizeof(BMPacketHeader));
	applog(LOG_DEBUG, "ReceivePacket: payload length=%d", packet->header.length);

	if (packet->header.length > 0)
	{
		// this should come through pretty quickly...
		for (int i = 0; i < 3; i++)
		{
			err = usb_read_ok_timeout(bmh->cgpu, (char *) packet, packet->header.length, &rxLength, 10, C_BMHASHER);

			if (err >= 0 && rxLength == packet->header.length)
			{
				break;
			}

			if ((err < 0 && err != LIBUSB_ERROR_TIMEOUT))
			{
				// N.B. thus !(*sent) directly implies err < 0 or *amount < send_len
				return err;
			}

			cgsleep_ms(100);
		}

		// if still no payload, then error
		if (rxLength < packet->header.length)
		{
			applog(LOG_DEBUG, "ReceivePacket: packet payload timeout");
			return LIBUSB_ERROR_TIMEOUT;
		}

		hexdump("ReceivePacket payload", packet->payload, packet->header.length);
	}

	return sizeof(BMPacketHeader) + packet->header.length;
}

/********** driver interface */
void exit_BMH_chain(struct BMH_chain *bmh)
{
	if (bmh == NULL)
		return;
	free(bmh->modules);
	bmh->modules = NULL;
	free(bmh);
}


struct BMH_chain *init_BMH_chain(struct cgpu_info *cgpu)
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

	for (i = 0; i < bmh->num_modules; i++)
	{
		bmh->modules[i].chain = bmh;
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

static void BMH_initialise(struct cgpu_info *cgpu)
{
	int err, interface;

	applog(LOG_DEBUG, "BMH_initialise: usbinfo.nodev=%d", cgpu->usbinfo.nodev);

// TODO: does x-link bypass the other device FTDI? (I think it does)
//	So no initialisation required except for the master device?

	if (cgpu->usbinfo.nodev)
		return;

	interface = usb_interface(cgpu);
	// Reset
	err = usb_transfer(cgpu, FTDI_TYPE_OUT, FTDI_REQUEST_RESET,
				FTDI_VALUE_RESET, interface, C_RESET);

	applog(LOG_DEBUG, "%s%i: reset got err %d",
		cgpu->drv->name, cgpu->device_id, err);

	if (cgpu->usbinfo.nodev)
		return;

	usb_ftdi_set_latency(cgpu);

	if (cgpu->usbinfo.nodev)
		return;

	// Set data control
	err = usb_transfer(cgpu, FTDI_TYPE_OUT, FTDI_REQUEST_DATA,
				FTDI_VALUE_DATA_BAS, interface, C_SETDATA);

	applog(LOG_DEBUG, "%s%i: setdata got err %d",
		cgpu->drv->name, cgpu->device_id, err);

	if (cgpu->usbinfo.nodev)
		return;

	// Set the baud
	err = usb_transfer(cgpu, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, FTDI_VALUE_BAUD_BMH,
				(FTDI_INDEX_BAUD_BMH & 0xff00) | interface,
				C_SETBAUD);

	applog(LOG_DEBUG, "%s%i: setbaud got err %d",
		cgpu->drv->name, cgpu->device_id, err);

	if (cgpu->usbinfo.nodev)
		return;

	// Set Flow Control
	err = usb_transfer(cgpu, FTDI_TYPE_OUT, FTDI_REQUEST_FLOW,
				FTDI_VALUE_FLOW, interface, C_SETFLOW);

	applog(LOG_DEBUG, "%s%i: setflowctrl got err %d",
		cgpu->drv->name, cgpu->device_id, err);

	if (cgpu->usbinfo.nodev)
		return;

	// Set Modem Control
	err = usb_transfer(cgpu, FTDI_TYPE_OUT, FTDI_REQUEST_MODEM,
				FTDI_VALUE_MODEM, interface, C_SETMODEM);

	applog(LOG_DEBUG, "%s%i: setmodemctrl got err %d",
		cgpu->drv->name, cgpu->device_id, err);

	if (cgpu->usbinfo.nodev)
		return;

	// Clear any sent data
	err = usb_transfer(cgpu, FTDI_TYPE_OUT, FTDI_REQUEST_RESET,
				FTDI_VALUE_PURGE_TX, interface, C_PURGETX);

	applog(LOG_DEBUG, "%s%i: purgetx got err %d",
		cgpu->drv->name, cgpu->device_id, err);

	if (cgpu->usbinfo.nodev)
		return;

	// Clear any received data
	err = usb_transfer(cgpu, FTDI_TYPE_OUT, FTDI_REQUEST_RESET,
				FTDI_VALUE_PURGE_RX, interface, C_PURGERX);

	applog(LOG_DEBUG, "%s%i: purgerx got err %d",
		cgpu->drv->name, cgpu->device_id, err);
}

static struct cgpu_info *BMH_detect_one_chain(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *cgpu;
	// struct spi_ctx *ctx = spi_init(cfg);

	// if (ctx == NULL)
	// 	return false;

	applog(LOG_DEBUG, "BMH_detect_one_chain: dev=%p", dev);

	cgpu = usb_alloc_cgpu(&bmhasher_drv, 1);
	assert(cgpu != NULL);

	struct BMH_chain *bmh = init_BMH_chain(cgpu);
	if (bmh == NULL)
		return false;

	cgpu->name = "BMH";
	cgpu->threads = 1;
	cgpu->device_data = bmh;
	bmh->cgpu = cgpu;

	if (!usb_init(cgpu, dev, found))
		goto shin;

	BMH_initialise(cgpu);

	if (!add_cgpu(cgpu))
		goto unshin;

	update_usb_stats(cgpu);

	return cgpu;

unshin:

	applog(LOG_DEBUG, "BMH_detect_one_chain: unshin Error! Need to add cleanup...");
	usb_uninit(cgpu);

shin:
	applog(LOG_DEBUG, "BMH_detect_one_chain: shin Error! Need to add cleanup...");

	exit_BMH_chain(bmh);
	cgpu->device_data = NULL;

	cgpu = usb_free_cgpu(cgpu);

	return NULL;
}

/* Probe SPI channel and register chip chain */
void BMH_detect(bool hotplug)
{
	int bus;
	int cs_line;

	applog(LOG_DEBUG, "BMH_detect: hotplug=%d", hotplug);

	/* no hotplug support for now */
	if (hotplug)
		return;

	usb_detect(&bmhasher_drv, BMH_detect_one_chain);

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

	// BMH_detect_one_chain(&cfg);
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

static void testwork(struct work * work)
{
	static uint8_t jobData[] = {
		/* midstate */
		0x8C, 0x1F, 0xA3, 0x18, 0xD8, 0x0A, 0x25, 0x2C, 0xE4, 0xB7, 0xCD, 0x6D, 0x12, 0x2F, 0x80, 0x8F,
		0x17, 0xDC, 0xD8, 0x10, 0x04, 0x17, 0xEA, 0x3F, 0xE8, 0xF3, 0x71, 0x41, 0x70, 0xF3, 0x4B, 0x49,
		/* wdata */
		0xD6, 0x98, 0x8E, 0x01, 0x27, 0x1F, 0x66, 0x52, 0xB6, 0x0A, 0x10, 0x19,
		/* start-nonce, difficulty 1, end-nonce */
		0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x1D, 0xFF, 0xFF, 0xFF, 0xFF
	};
	uint32_t nonce;
	bool nonceOK;
	unsigned char *midstate = work->midstate;
	unsigned char *wdata = work->data + 64;
	uint8_t *hash_8 = (uint8_t *)(work->hash + 0);

	applog(LOG_DEBUG, "******************** testwork\n");

	memcpy(midstate, jobData, 32);
	memcpy(wdata, &jobData[32], 12);

	nonce = 0x99b18d18;
	nonceOK = test_nonce(work, nonce);
	applog(LOG_DEBUG, "nonce=0x%x nonceOK=%d\n", nonce, nonceOK);
	hexdump("HASH: ", hash_8, 32);

	nonce = 0x0cb2a63a;
	nonceOK = test_nonce(work, nonce);
	applog(LOG_DEBUG, "nonce=0x%x nonceOK=%d\n", nonce, nonceOK);
	hexdump("HASH: ", hash_8, 32);

	nonce = 0xde648f3f;
	nonceOK = test_nonce(work, nonce);
	applog(LOG_DEBUG, "nonce=0x%x nonceOK=%d\n", nonce, nonceOK);
	hexdump("HASH: ", hash_8, 32);

	nonce = 0x09c79cb9;
	nonceOK = test_nonce(work, nonce);
	applog(LOG_DEBUG, "nonce=0x%x nonceOK=%d\n", nonce, nonceOK);
	hexdump("HASH: ", hash_8, 32);

	nonce = 0xb3587bbe;
	nonceOK = test_nonce(work, nonce);
	applog(LOG_DEBUG, "nonce=0x%x nonceOK=%d\n", nonce, nonceOK);
	hexdump("HASH: ", hash_8, 32);

	applog(LOG_DEBUG, "******************** loser nonces\n");
	nonce = 0x47b8a662;
	nonceOK = test_nonce(work, nonce);
	applog(LOG_DEBUG, "nonce=0x%x nonceOK=%d\n", nonce, nonceOK);
	hexdump("HASH: ", hash_8, 32);

	nonce = 0x62a6b847;
	nonceOK = test_nonce(work, nonce);
	applog(LOG_DEBUG, "nonce=0x%x nonceOK=%d\n", nonce, nonceOK);
	hexdump("HASH: ", hash_8, 32);
}

static bool doneOnce = false;
static uint8_t gPacketBuffer[512];
static BMPacket * gPacket = (BMPacket *) gPacketBuffer;

/* queue two work items per chip in chain */
static bool BMH_queue_full(struct cgpu_info *cgpu)
{
	struct BMH_chain *bmh = cgpu->device_data;
	int queue_full = false;
	struct work *work;

	// applog(LOG_DEBUG, "BMH_queue_full");
	mutex_lock(&bmh->lock);
	applog(LOG_DEBUG, "BMH running queue_full: %d/%d",
	       bmh->active_wq.num_elems, bmh->num_modules);

	if (!doneOnce)
	{
		int length;

		applog(LOG_DEBUG, "BMH_flush_work: sending hello packet");
		gPacket->header.type = kHelloPacketType;
		gPacket->header.length = 0;
		SendPacket(bmh, 0, gPacket);

		length = ReceivePacket(bmh, gPacket);
		applog(LOG_DEBUG, "ReceivePacket: length=%d", length);
		if (length >= 0)
		{
			applog(LOG_DEBUG, "ReceivePacket: type=%d length=%d", gPacket->header.type, gPacket->header.length);
			hexdump("recevied packet", (uint8_t *) gPacket, length);
		}

		doneOnce = true;
	}

	if (bmh->active_wq.num_elems >= bmh->num_modules * 2) {
		// applog(LOG_DEBUG, "active_wq full");
		queue_full = true;
	} else {
		applog(LOG_DEBUG, "active_wq enquing");
		work = get_queued(cgpu);
		// testwork(work);
		wq_enqueue(&bmh->active_wq, work);
	}
	mutex_unlock(&bmh->lock);

	return queue_full;
}

static void BMH_flush_work(struct cgpu_info *cgpu)
{
	struct BMH_chain *bmh = cgpu->device_data;
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
