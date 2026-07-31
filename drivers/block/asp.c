/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * Apple ANS1 Storage Processor Driver
 *
 * Copyright (c) 2026 Nick Chan <towinchenmi@gmail.com>
 */

#define DEBUG 1
#include <linux/async.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mempool.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/once.h>
#include <linux/platform_device.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/pm_domain.h>
#include <linux/reset.h>
#include <linux/dmapool.h>

#define DEVICE_NAME	"asp"
#define DISK_NAME	"asp%dn%d"

#define AKF_V1_REMAP_AP_LO	0x8
#define AKF_V1_REMAP_AP_HI	0xc
#define AKF_V1_REMAP_IOP_LO	0x10
#define AKF_V1_REMAP_IOP_HI	0x14
#define AKF_V1_REMAP_SZ_LO	0x18
#define AKF_V1_REMAP_SZ_HI	0x1c
#define AKF_V1_ENDIANNESS	0x20

#define AKF_V1_LITTLE_ENDIAN	BIT(0)

#define AKF_V1_CPU_CONTROL		0x28
#define AKF_V1_CPU_CONTROL_START	BIT(4)

/*
 * The command buffer must be less than 262144 bytes, and must be alinged
 * to cacheline. Dividing equally yields 16320 bytes per tag.
 */
#define ASP_TAG_SIZE	16320
#define ASP_NUM_TAGS	16
#define ASP_QUEUE_SIZE  (ASP_TAG_SIZE * ASP_NUM_TAGS)

#define ASP_TIMEOUT_MS		30000
#define ASP_CMD_HDR_SIZE	0x30

#define ASP_CMD_MAX_SECTORS	(ASP_TAG_SIZE - ASP_CMD_HDR_SIZE) \
				/ sizeof(uint32_t)

/*
 * All ASP instances seen uses 4096 bytes sector size. Since this hardware is
 * already replaced by NVMe, it is unlikely any other sector sizes will be
 * used in practice.
 */
#define ASP_LBA_SHIFT	12
#define ASP_LBA_SIZE	BIT(ASP_LBA_SHIFT)

/* ASP Commands */
#define ASP_CMD_IDENTIFY	0x0
#define ASP_CMD_READ_USERAREA	0x10
#define ASP_CMD_WRITE_USERAREA	0x11

#define ASP_CMD_FORMAT_ALL	0x15
#define ASP_CMD_FORMAT_USERAREA	0x16
#define ASP_CMD_FORMAT_CLEAR	0x18

#define ASP_CMD_WRITE_UNLOCK	0x19

#define ASP_CMD_OP_READ_LLB		0x30
#define ASP_CMD_OP_READ_FW		0x31
#define ASP_CMD_OP_READ_UTILDM		0x32
#define ASP_CMD_OP_READ_DM		0x33
#define ASP_CMD_OP_READ_CTRLBITS	0x34
#define ASP_CMD_OP_READ_EFFACE		0x35
#define ASP_CMD_OP_READ_NVRAM		0x36
#define ASP_CMD_OP_READ_SYSCFG		0x37
#define ASP_CMD_OP_READ_PANICLOG	0x38

#define ASP_CMD_OP_WRITE_LLB		0x40
#define ASP_CMD_OP_WRITE_FW		0x41
#define ASP_CMD_OP_WRITE_UTILDM		0x42
#define ASP_CMD_OP_WRITE_DM		0x43
#define ASP_CMD_OP_WRITE_CTRLBITS	0x44
#define ASP_CMD_OP_WRITE_EFFACE		0x45
#define ASP_CMD_OP_WRITE_NVRAM		0x46
#define ASP_CMD_OP_WRITE_SYSCFG		0x47
#define ASP_CMD_OP_WRITE_PANICLOG	0x48

#define ASP_CMD_OP_GET_PPN_FW_VERSION	0x79
#define ASP_CMD_OP_POWER_CONFIG		0x80

/*
 * Auxiliary Storage Sizes
 *
 * Hardcode the sizes as there are no way to get the size of auxiliary
 * storage sizes from the firmware.
 */
#define ASP_FW_NUM_SECTORS		1024
#define ASP_LLB_NUM_SECTORS		254
#define ASP_UTILDM_NUM_SECTORS		2
#define ASP_DM_NUM_SECTORS		32
#define ASP_CTRLBITS_NUM_SECTORS 	32
#define ASP_EFFACE_NUM_SECTORS		32
#define ASP_NVRAM_NUM_SECTORS		32
#define ASP_SYSCFG_NUM_SECTORS		256
#define ASP_PANICLOG_NUM_SECTORS	256

/* Mailbox messages */
#define ASP_MSG_TYPE			GENMASK(3, 0)
#define ASP_MSG_TYPE_SET_CMDBUF		0
#define ASP_MSG_TYPE_SET_TAG_OFF	1
#define ASP_MSG_TYPE_COMPLETE		2
#define ASP_MSG_TYPE_SUBMIT		3
#define ASP_MSG_TYPE_UNK4		4

#define ASP_SET_CMDBUF_BASE		GENMASK_ULL(55, 16)
#define ASP_SET_CMDBUF_SIZE_17_6	GENMASK(15, 4)

#define ASP_SET_TAG_CMDBUF_SIZE_17_6	GENMASK_ULL(35, 24)
#define ASP_SET_TAG_CMDBUF_OFF_17_6	GENMASK(23, 12)
#define ASP_SET_TAG_CMDBUF_TAG		GENMASK(7, 4)

#define ASP_SUBMIT_OP			GENMASK(19, 12)
#define ASP_SUBMIT_OP_RING		0xff
#define ASP_SUBMIT_TAG			GENMASK(7, 4)

/*
 * This is wider than the submission tag, since ANS1 will send
 * two special tags during startup:
 *
 * 254 - sent automatically soon after ANS endpoint is enabled,
 * possibly ready signal
 *
 * 253 - reply of ASP_MSG_TYPE_SET_CMDBUF
 */
#define ASP_COMPLETION_TAG		GENMASK(11, 4)
#define ASP_COMPLETION_TAG_READY		254
#define ASP_COMPLETION_TAG_CMDBUF_OK		253

#define ASP_COMPLETION_STATUS		GENMASK(15, 12)
#define ASP_COMPLETION_STATUS_OK	0
#define ASP_COMPLETION_STATUS_IGNORED 	1 /* not exist command */
#define ASP_COMPLETION_STATUS_ABORT	2
#define ASP_COMPLETION_STATUS_INIT  	0xa /* Seen with the special tags */

/*
 * Without this flag the operation would still take place but instead of
 * the intended data garbage corresponding to the input data in 128-bit
 * block sizes would be used, so likely encryption.
 */
#define ASP_CMD_IO_FLAG_NO_ENCRYPTION BIT(3)

/*
 * this enum is used to generate hardware operations, so do not change the order
 * of items.
 */
enum asp_queue_type {
	ASP_QUEUE_TYPE_ADMIN = 0,
	ASP_QUEUE_TYPE_USERAREA,
	ASP_QUEUE_TYPE_LLB,
	ASP_QUEUE_TYPE_FW,
	ASP_QUEUE_TYPE_UTILDM,
	ASP_QUEUE_TYPE_DM,
	ASP_QUEUE_TYPE_CTRLBITS,
	ASP_QUEUE_TYPE_EFFACE,
	ASP_QUEUE_TYPE_NVRAM,
	ASP_QUEUE_TYPE_SYSCFG,
	ASP_QUEUE_TYPE_PANICLOG,
	ASP_QUEUE_TYPE_COUNT
};

struct asp_identify {
	u32 num_lba_formatted;
	u32 lba_size;
	u8 unk1[8];
	bool util_formatted;
	u8 unk2[7];
	u32 num_lba_raw;
	u8 unk3[20];
	u8 chip_id_bus0[6];
	u8 chip_id_bus1[6];
	u8 manufacturer_id_bus0[6];
	u8 manufacturer_id_bus1[6];
}__attribute__((packed));

struct asp_ppn_fw {
	u32 fw_ver_len;
	char fw_ver[0x10][2];
}__attribute__((packed));

struct asp_rw {
	u32 sglist[ASP_CMD_MAX_SECTORS];
};

struct asp_cmd_hdr {
	u8 op;
	u8 tag;
	u16 flags;
	u32 slba;
	u32 length;
	u8 pad[0x24];
}__attribute__((packed));

struct asp_queue {
	enum asp_queue_type type;
	struct gendisk *disk;
};

struct apple_asp {
	void *__iomem akf_base;
	sector_t capacity;
	struct device *dev;
	struct apple_rtkit *rtk;
	struct completion init_done;
	int init_ret;
	int instance;
	void *q;
	struct request_queue *admin_rq;
	struct blk_mq_tag_set tagset;
	union {
		struct asp_queue qd_head; /* for container_of() */
		struct asp_queue q_data[ASP_QUEUE_TYPE_COUNT];
	};
	dma_addr_t q_iova;
	mempool_t *iod_mempool;
	u32 lba_size;
	u8 ep;
};

struct asp_iod {
	struct asp_cmd_hdr cmd;
	int nr_mapped;
	struct scatterlist *sg;
	int nents;
	void *buffer;
	size_t buf_len;
	u8 status;
};

// BLK_STS_MEDIUM
static bool enable_reformat;
module_param(enable_reformat, bool, 0644);
MODULE_PARM_DESC(enable_reformat, "Automatically format unformatted media");

//static int asp_major;
static DEFINE_IDA(asp_instance_ida);

/*
 * The information about unknown commands in this function is derived by
 * attempting command execution when the write lock is still armed.
 */
static bool asp_is_write(struct asp_cmd_hdr *cmd)
{
	switch (cmd->op) {
		case ASP_CMD_IDENTIFY:
		case ASP_CMD_READ_USERAREA:
			return false;
		case ASP_CMD_WRITE_USERAREA:
		case ASP_CMD_FORMAT_ALL:
		case ASP_CMD_FORMAT_USERAREA:
		case ASP_CMD_FORMAT_CLEAR:
			return true;
		case ASP_CMD_WRITE_UNLOCK:
			return false;
		case ASP_CMD_OP_READ_LLB:
		case ASP_CMD_OP_READ_FW:
		case ASP_CMD_OP_READ_UTILDM:
		case ASP_CMD_OP_READ_DM:
		case ASP_CMD_OP_READ_CTRLBITS:
		case ASP_CMD_OP_READ_EFFACE:
		case ASP_CMD_OP_READ_NVRAM:
		case ASP_CMD_OP_READ_SYSCFG:
		case ASP_CMD_OP_READ_PANICLOG:
			return false;
		case ASP_CMD_OP_WRITE_LLB:
		case ASP_CMD_OP_WRITE_FW:
		case ASP_CMD_OP_WRITE_UTILDM:
		case ASP_CMD_OP_WRITE_DM:
		case ASP_CMD_OP_WRITE_CTRLBITS:
		case ASP_CMD_OP_WRITE_EFFACE:
		case ASP_CMD_OP_WRITE_NVRAM:
		case ASP_CMD_OP_WRITE_SYSCFG:
		case ASP_CMD_OP_WRITE_PANICLOG:
			return true;
		case 0x50:
		case 0x51:
		case 0x52:
			return true;
		case 0x72:
		case 0x73:
		case 0x74:
		case 0x75:
		case 0x76:
		case 0x77:
		case 0x78:
		case ASP_CMD_OP_GET_PPN_FW_VERSION:
			return false;
		case 0x7a:
			return true;
		case 0x7b:
		case 0x7c:
		case 0x7d:
		case 0x7e:
		case 0x7f:
		case 0x80:
		case 0x81:
		case 0x82:
		case 0x90:
			return false;
		case 0x91:
			return true;
		case 0xa0:
		case 0xa1:
			return false;
		default:
			BUG();
	};
}

static enum req_op asp_req_op(struct asp_cmd_hdr *cmd)
{
	return asp_is_write(cmd) ? REQ_OP_DRV_OUT : REQ_OP_DRV_IN;
}

static int apple_asp_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
			       unsigned int hctx_idx)
{
	hctx->driver_data = data;

	return 0;
}

static struct apple_asp *queue_to_apple_asp(struct asp_queue *aq)
{
	return container_of(aq - aq->type, struct apple_asp, qd_head);
}

static void *asp_q_cmd_hdr(struct apple_asp *asp, struct asp_cmd_hdr *cmd)
{
	return asp->q + (cmd->tag * ASP_TAG_SIZE);
}

static void *asp_q_cmd_data(struct apple_asp *asp, struct asp_cmd_hdr *cmd)
{
	return asp->q + (cmd->tag * ASP_TAG_SIZE) + ASP_CMD_HDR_SIZE;
}

static blk_status_t asp_setup_rw(struct asp_queue *qd, struct request *req,
				 bool write)
{
	struct asp_iod *iod = blk_mq_rq_to_pdu(req);
	struct asp_cmd_hdr *cmd = &iod->cmd;
	struct apple_asp *asp = queue_to_apple_asp(qd);
	struct req_iterator iter;
       	struct bio_vec bvec;

	switch (qd->type) {
		case ASP_QUEUE_TYPE_ADMIN:
			BUG();
		case ASP_QUEUE_TYPE_USERAREA:
			cmd->op = 0x10 | write;
			break;
		case ASP_QUEUE_TYPE_LLB:
		case ASP_QUEUE_TYPE_FW:
		case ASP_QUEUE_TYPE_UTILDM:
		case ASP_QUEUE_TYPE_DM:
		case ASP_QUEUE_TYPE_CTRLBITS:
		case ASP_QUEUE_TYPE_EFFACE:
		case ASP_QUEUE_TYPE_NVRAM:
		case ASP_QUEUE_TYPE_SYSCFG:
		case ASP_QUEUE_TYPE_PANICLOG:
			cmd->op = (write ? 0x40 : 0x30) | (qd->type - 2);
			break;
		default:
			BUG();
	}

	cmd->flags = ASP_CMD_IO_FLAG_NO_ENCRYPTION;
	cmd->slba = blk_rq_pos(req) >> 3;
	cmd->length = blk_rq_bytes(req) >> ASP_LBA_SHIFT;

	iod->sg = mempool_alloc(asp->iod_mempool, GFP_ATOMIC);
	if (!iod->sg)
		return BLK_STS_RESOURCE;

	sg_init_table(iod->sg, blk_rq_nr_phys_segments(req));
	iod->nents = blk_rq_map_sg(req, iod->sg);

	if (!iod->nents)
		goto out_free_sg;

	iod->nr_mapped = dma_map_sg_attrs(asp->dev, iod->sg, iod->nents,
					  rq_dma_dir(req), DMA_ATTR_NO_WARN);

	if (!iod->nr_mapped)
		goto out_free_sg;

	//dev_dbg(asp->dev, "queue type: %d nr_mapped: %d", qd->type, iod->nr_mapped);

	if (qd->type > ASP_QUEUE_TYPE_USERAREA
	    && rq_dma_dir(req) == DMA_FROM_DEVICE) {
		/*
		 * If reading auxiliary storage, then zero the dma memory
		 * beforehand, as the request gets truncated when trying to
		 * read "unwritten" sectors.
		 *
		 * This matches iOS' behavior except in the case where the
		 * first sector is unwritten due to an iOS bug.
		 */
		rq_for_each_segment(bvec, req, iter)
			memzero_bvec(&bvec);
	}

	return BLK_STS_OK;

out_free_sg:
	mempool_free(iod->sg, asp->iod_mempool);
	return BLK_STS_RESOURCE;
}

/*
 * The controller requires writes to auxiliary storage to start at sector 0,
 * so do not bother with their write support for now.
 */

/*
 * UTILDM, DM, CTRLBITS seems to be controller internal data and putting the
 * wrong things in them could cause persistent firmware crashes so do not
 * allow writes to them.
 *
 * The controller does not allow writing to UTILDM (maximum allowed size
 * for writing is 0), and DM has a minimum write size requirement of
 * 12 sectors, though this does not matter since we set it to read-only
 * anyways.
 */

static blk_status_t asp_setup_cmd(struct asp_queue *qd, struct request *req)
{
	blk_status_t ret;
	struct asp_iod *iod = blk_mq_rq_to_pdu(req);
	struct asp_cmd_hdr *cmd = &iod->cmd;

	cmd->tag = req->tag;

	switch (req_op(req)) {
		/* these are already setup */
		case REQ_OP_DRV_IN:
		case REQ_OP_DRV_OUT:
			ret = BLK_STS_OK;
			break;

		case REQ_OP_READ:
		case REQ_OP_WRITE:
			ret = asp_setup_rw(qd, req, req_op(req) == REQ_OP_WRITE);
			break;

		default:
			ret = BLK_STS_NOTSUPP;
			break;
	}

	return ret;
}

static blk_status_t apple_asp_submit_cmd(struct apple_asp *asp,
					 struct asp_cmd_hdr *cmd)
{
	int ret;

	memcpy(asp_q_cmd_hdr(asp, cmd), cmd, ASP_CMD_HDR_SIZE);

	dma_wmb();
	//dev_info(asp->dev, "tag: %d", cmd->tag);
	//BUG_ON(cmd->tag >= 16);

	u64 rtk_msg = FIELD_PREP(ASP_MSG_TYPE, ASP_MSG_TYPE_SUBMIT)
		      | FIELD_PREP(ASP_SUBMIT_TAG, cmd->tag)
		      | FIELD_PREP(ASP_SUBMIT_OP, ASP_SUBMIT_OP_RING);

	ret = apple_rtkit_send_message(asp->rtk, asp->ep, rtk_msg, NULL, false);
	if (ret == -ETIMEDOUT)
		return BLK_STS_TIMEOUT;
	else if (ret < 0)
		return BLK_STS_IOERR;

	return BLK_STS_OK;
}

static void asp_setup_hw_sgl(struct apple_asp *asp, struct asp_iod *iod)
{
	int i, hw_idx = 0;
	struct scatterlist *sg;
	u32 *hw_sgl = asp_q_cmd_data(asp, &iod->cmd);

	for_each_sg(iod->sg, sg, iod->nr_mapped, i) {
		unsigned int len = sg_dma_len(sg);
		dma_addr_t addr = sg_dma_address(sg);

		while (len) {
			//dev_dbg(asp->dev, "HW SGL IDX: %d addr 0x%llx len 0x%x", hw_idx, addr, len);

			hw_sgl[hw_idx++] = addr >> ASP_LBA_SHIFT;

			addr += ASP_LBA_SIZE;
			len -= ASP_LBA_SIZE;
		}

	}
	BUG_ON(hw_idx > ASP_CMD_MAX_SECTORS);
}

static bool apple_asp_is_aux_read(struct asp_cmd_hdr *cmd)
{
	switch (cmd->op) {
		case ASP_CMD_OP_READ_LLB:
		case ASP_CMD_OP_READ_FW:
		case ASP_CMD_OP_READ_UTILDM:
		case ASP_CMD_OP_READ_DM:
		case ASP_CMD_OP_READ_CTRLBITS:
		case ASP_CMD_OP_READ_EFFACE:
		case ASP_CMD_OP_READ_NVRAM:
		case ASP_CMD_OP_READ_SYSCFG:
		case ASP_CMD_OP_READ_PANICLOG:
			return true;
	}

	return false;
}

static blk_status_t apple_asp_queue_rq(struct blk_mq_hw_ctx *hctx,
				       const struct blk_mq_queue_data *bd)
{
	struct asp_queue *qd = hctx->queue->queuedata;
	struct apple_asp *asp = queue_to_apple_asp(qd);
	struct request *req = bd->rq;
	struct asp_iod *iod = blk_mq_rq_to_pdu(req);
	blk_status_t ret;

	ret = asp_setup_cmd(qd, req);
	if (ret)
		return ret;

	if (iod->sg)
		asp_setup_hw_sgl(asp, iod);

	blk_mq_start_request(req);

	ret = apple_asp_submit_cmd(asp, &iod->cmd);
	if (ret)
		return ret;

	return BLK_STS_OK;
}

/* don't need this i think */
#if 0
static int apple_asp_init_request(struct blk_mq_tag_set *set
				  struct request *req, unsigned int hctx_idx,
                                  int numa_node)
{
	struct apple_asp *asp = set->driver_data;
	struct asp_queue *qd = req->q->queuedata;
	struct asp_iod *iod = blk_mq_rq_to_pdu(req);

	return 0;
}
#endif

static blk_status_t asp_submit_sync_admin_cmd(struct request_queue *q,
					      struct asp_cmd_hdr *cmd,
					      void *buffer, size_t buf_len)
{
	struct request *req;
	struct asp_iod *iod;
	blk_status_t ret;
	blk_mq_req_flags_t blk_flags = 0;

	if (cmd->length)
		return -EOPNOTSUPP;

	req = blk_mq_alloc_request(q, asp_req_op(cmd), blk_flags);

	if (IS_ERR(req))
		return PTR_ERR(req);

	iod = blk_mq_rq_to_pdu(req);

	memcpy(&iod->cmd, cmd, ASP_CMD_HDR_SIZE);
	iod->buffer = buffer;
	iod->buf_len = buf_len;

	ret = blk_execute_rq(req, false);
	blk_mq_free_request(req);

	return ret;
}

#if 0
static enum blk_eh_timer_return apple_asp_timeout(struct request *req)
{
	struct asp_queue *qd = req->q->queuedata;
	struct asp_iod *iod = blk_mq_rq_to_pdu(req);
	struct apple_asp *asp = queue_to_apple_asp(qd);

	/* soft-reset the controller since commands cannot be aborted */
	dev_warn(asp-dev, "I/O %d:%d timeout: resetting controller",
			   qd->type, req->tag);

	/* freeze all queues while it is resetting */
	blk_mq_freeze_queue(asp->admin_rq);
	for (int i = ASP_QUEUE_TYPE_USERAREA; i < ASP_QUEUE_TYPE_COUNT; i++)
		blk_mq_freeze_queue(asp->q_data[i].disk->queue);


	if (!apple_rtkit_is_running(asp->rtk)) {
		/*
		 * the storage firmware is loaded by previous stages, so
		 * recovery cannot be implemented.
		 */
		dev_err(asp->dev, "RTKit has crashed; cannot recover.")
		return BLK_EH_DONE;
	}

	ret = apple_rtkit_shutdown(asp->rtk)
	if (ret < 0) {
		dev_err(asp->dev, "Failed to shutdown RTKit: %d", ret);
		return BLK_EH_DONE;
	}

	/* Turn it back on */

	blk_mq_unfreeze_queue(asp->admin_rq);
	for (int i = ASP_QUEUE_TYPE_USERAREA; i < ASP_QUEUE_TYPE_COUNT; i++)
		blk_mq_unfreeze_queue(asp->q_data[i].disk->queue);

	return BLK_EH_DONE;
}
#endif

static void apple_asp_complete_rq(struct request *rq)
{
	struct asp_queue *qd = rq->q->queuedata;
	struct asp_iod *iod = blk_mq_rq_to_pdu(rq);
	struct apple_asp *asp = queue_to_apple_asp(qd);

	if (iod->sg) {
		dma_unmap_sg(asp->dev, iod->sg, iod->nents, rq_dma_dir(rq));
		mempool_free(iod->sg, asp->iod_mempool);
	}

	switch (iod->status) {
		case ASP_COMPLETION_STATUS_OK:
			blk_mq_end_request(rq, BLK_STS_OK);
			break;
		case ASP_COMPLETION_STATUS_IGNORED:
			blk_mq_end_request(rq, BLK_STS_NOTSUPP);
			break;
		case ASP_COMPLETION_STATUS_ABORT:
			/*
			 * When reading auxiliary storage, if the starting
			 * sector is unwritten, then the request will fail with
			 * ASP_COMPLETION_STATUS_ABORT. Consider this case a
			 * success (the zeroing is already done in
			 * asp_setup_rw())
			 */
			if (apple_asp_is_aux_read(&iod->cmd)) {
				blk_mq_end_request(rq, BLK_STS_OK);
				break;
			}
		fallthrough;
		default:
			blk_mq_end_request(rq, BLK_STS_IOERR);
			break;
	}
}

static const struct blk_mq_ops apple_asp_queue_ops = {
	.queue_rq = apple_asp_queue_rq,
	.complete = apple_asp_complete_rq,
	//.init_request = apple_asp_init_request,
	.init_hctx = apple_asp_init_hctx,
	//.timeout = apple_asp_timeout,
};

/*
 * Compute reserve memory size needed for the driver to complete a
 * a request, even when under memory pressure, so things like swap
 * can work
 */
static inline size_t apple_asp_iod_alloc_size(void)
{
	return (ASP_CMD_MAX_SECTORS * sizeof(struct scatterlist));
}

static const struct block_device_operations asp_fops = {
	.owner          = THIS_MODULE,
};

/* called when ASP is ready to receive commands */
static int apple_asp_start_disk(struct apple_asp *asp)
{
	int ret;
	int disk_added;
	blk_status_t status;
	struct asp_cmd_hdr *cmd;
	struct asp_identify *identify;

	ret = ida_alloc(&asp_instance_ida, GFP_KERNEL);
	if (ret < 0)
		return ret;

	asp->instance = ret;
	asp->iod_mempool = mempool_create_kmalloc_pool(1,
				apple_asp_iod_alloc_size());

	if (!asp->iod_mempool) {
		ret = -ENOMEM;
		goto out_free_ida;
	}

	asp->tagset.ops = &apple_asp_queue_ops;
	asp->tagset.nr_hw_queues = 1;
	asp->tagset.queue_depth = ASP_NUM_TAGS - 1;
	asp->tagset.timeout = ASP_TIMEOUT_MS;
	asp->tagset.numa_node = NUMA_NO_NODE;
	asp->tagset.cmd_size = sizeof(struct asp_iod);
	asp->tagset.driver_data = asp;

	dev_dbg(asp->dev, "allocating tag set");
	ret = blk_mq_alloc_tag_set(&asp->tagset);

	if (ret < 0) {
		dev_err(asp->dev, "unable to allocate tag set");
		goto out_destroy_mempool;
	}

	for (int i = 0; i < ASP_QUEUE_TYPE_COUNT; i++)
		asp->q_data[i].type = i;

	dev_dbg(asp->dev, "allocating request queues");

	asp->admin_rq = blk_mq_alloc_queue(&asp->tagset, NULL, &asp->q_data[0]);
	if (IS_ERR(asp->admin_rq)) {
		ret = dev_err_probe(asp->dev, PTR_ERR(asp->admin_rq),
				    "unable to allocate admin request queue");
		goto out_free_tag_set;
	}

	// identify
	// one queue for each namespace
	cmd = kmalloc_obj(struct asp_cmd_hdr);
	if (IS_ERR(cmd)) {
		ret = -ENOMEM;
		goto out_free_admin_rq;
	}

	identify = kmalloc_obj(struct asp_identify);
	if (IS_ERR(identify)) {
		ret = -ENOMEM;
		goto out_free_cmd;
	}

	cmd->op = ASP_CMD_IDENTIFY;
	cmd->flags = 0;
	cmd->slba = 0;
	cmd->length = 0;

	status = asp_submit_sync_admin_cmd(asp->admin_rq, cmd, identify,
					   sizeof(struct asp_identify));
	if (status) {
		dev_dbg(asp->dev, "identify controller failure status: %d",
			status);
		ret = -EIO;
		goto out_free_id_data;
	}

	// TODO support auto reformat (with module arg?)
	if (!identify->util_formatted) {
		dev_err(asp->dev, "media not formatted");
		ret = -ENXIO;
		goto out_free_id_data;
	}

	if (identify->lba_size != ASP_LBA_SIZE) {
		dev_err(asp->dev, "unsupported LBA size %d",
			identify->lba_size);
		ret = -EOPNOTSUPP;
		goto out_free_id_data;
	}

	dev_dbg(asp->dev, "ASP block device %llu bytes",
		(u64)identify->num_lba_formatted * identify->lba_size);

	asp->capacity = identify->num_lba_formatted;

	kfree(identify);

	cmd->op = ASP_CMD_WRITE_UNLOCK;
	cmd->flags = 0;
	cmd->slba = 0;
	cmd->length = 0;

	status = asp_submit_sync_admin_cmd(asp->admin_rq, cmd, NULL, 0);
	if (status) {
		dev_err(asp->dev, "could not enable writes: %d", status);
		ret = -EIO;
		goto out_free_cmd;
	}

	dev_dbg(asp->dev, "Write enable OK");

	struct queue_limits lim = {
		.logical_block_size = ASP_LBA_SIZE,
		.max_hw_sectors = ASP_CMD_MAX_SECTORS,
		/* only constrianted by max_hw_sectors */
		.max_segments = USHRT_MAX,
		.max_segment_size = UINT_MAX,
	};

	for (int i = ASP_QUEUE_TYPE_USERAREA; i < ASP_QUEUE_TYPE_COUNT; i++) {
		struct gendisk *disk;

		asp->q_data[i].disk = blk_mq_alloc_disk(&asp->tagset, &lim,
							&asp->q_data[i]);
		disk = asp->q_data[i].disk;

		if (!disk) {
			ret = -ENOMEM;
			goto out_free_disk;
		}
		snprintf(disk->disk_name, sizeof(disk->disk_name), "asp%dn%d",
			 asp->instance, i);

		disk->fops = &asp_fops;
		disk->private_data = asp;
	}

	/*
	 * Auxiliary storage can only be written if starting LBA is zero,
	 * and certain ones has additional constriants, so don't bother
	 * with write support for them for now.
	 *
	 * Enabling writes to certain namespaces is also unsafe as putting
	 * the wrong things in some of them could cause persistent controller
	 * crashes.
	 *
	 * Eventually though it would be nie to support things like writing
	 * NVRAM, so set it to RO outside of the loop such that the statement
	 * can be removed on per-namespace basis.
	 */
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_LLB].disk, true);
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_FW].disk, true);
	/* write disallowed at controller level */
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_UTILDM].disk, true);
	/* unsafe to write */
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_DM].disk, true);
	/* unsafe to write */
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_CTRLBITS].disk, true);
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_EFFACE].disk, true);
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_NVRAM].disk, true);
	/*
	 * unrecoverable even with DFU, but some applications may want to
	 * write it anyways
	 */
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_SYSCFG].disk, true);
	set_disk_ro(asp->q_data[ASP_QUEUE_TYPE_PANICLOG].disk, true);

	/* no partitions on auxiliary storage */
	asp->q_data[ASP_QUEUE_TYPE_LLB].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_FW].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_UTILDM].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_DM].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_CTRLBITS].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_EFFACE].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_NVRAM].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_SYSCFG].disk->flags = GENHD_FL_NO_PART;
	asp->q_data[ASP_QUEUE_TYPE_PANICLOG].disk->flags = GENHD_FL_NO_PART;

	/* convert 4096-byte sectors to 512-byte sectors */
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_USERAREA].disk,
		     asp->capacity << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_LLB].disk,
		     ASP_LLB_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_FW].disk,
		     ASP_FW_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_UTILDM].disk,
		     ASP_UTILDM_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_DM].disk,
		     ASP_DM_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_CTRLBITS].disk,
		     ASP_CTRLBITS_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_EFFACE].disk,
		     ASP_EFFACE_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_NVRAM].disk,
		     ASP_NVRAM_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_SYSCFG].disk,
		     ASP_SYSCFG_NUM_SECTORS << 3);
	set_capacity(asp->q_data[ASP_QUEUE_TYPE_PANICLOG].disk,
		     ASP_PANICLOG_NUM_SECTORS << 3);

	for (int i = ASP_QUEUE_TYPE_USERAREA; i < ASP_QUEUE_TYPE_COUNT; i++) {
		/* TODO: sysfs attributes for serial number etc? */
		ret = device_add_disk(asp->dev, asp->q_data[i].disk, NULL);
		if (ret < 0)
			goto out_cleanup_disk;

		disk_added = i;
	}

	kfree(cmd);
	return 0;
out_cleanup_disk:
	/* make sure only disks that were added are removed */
	for (int i = ASP_QUEUE_TYPE_USERAREA; i <= disk_added; i++)
		del_gendisk(asp->q_data[i].disk);
out_free_disk:
	for (int i = ASP_QUEUE_TYPE_USERAREA; i < ASP_QUEUE_TYPE_COUNT; i++) {
		if (asp->q_data[i].disk)
			put_disk(asp->q_data[i].disk);
	}
out_free_id_data:
	kfree(identify);
out_free_cmd:
	kfree(cmd);
out_free_admin_rq:
	blk_mq_destroy_queue(asp->admin_rq);
	blk_put_queue(asp->admin_rq);
out_free_tag_set:
	blk_mq_free_tag_set(&asp->tagset);
out_destroy_mempool:
	mempool_destroy(asp->iod_mempool);
out_free_ida:
	ida_free(&asp_instance_ida, asp->instance);
	return ret;
}

static int apple_asp_setup_firmware(struct apple_asp *asp)
{
	struct device_node *mem_node;
	u64 fw_base;
	u64 fw_size;
	int idx;

	idx = of_property_match_string(asp->dev->of_node,
				       "memory-region-names", "firmware");

	mem_node = of_parse_phandle(asp->dev->of_node, "memory-region", idx);
	if (!mem_node)
		return dev_err_probe(asp->dev, -ENODEV,
				     "No memory-region found for firmware\n");

	if (of_property_read_reg(mem_node, 0, &fw_base, &fw_size))
		return dev_err_probe(asp->dev, -ENODEV,
				     "Could not read memory-region reg\n");

	dev_dbg(asp->dev, "fw_base: 0x%llx, fw_size: 0x%llx\n", fw_base, fw_size);

	/* make firmware available to ASP */
	writel(fw_base & 0xffffffff,
	       asp->akf_base + AKF_V1_REMAP_AP_LO);
	writel(fw_base >> 32, asp->akf_base + AKF_V1_REMAP_AP_HI);
	writel(0, asp->akf_base + AKF_V1_REMAP_IOP_LO);
	writel(0, asp->akf_base + AKF_V1_REMAP_IOP_HI);
	writel(fw_size & 0xffffffff,
	       asp->akf_base + AKF_V1_REMAP_SZ_LO);
	writel(fw_size >> 32, asp->akf_base + AKF_V1_REMAP_SZ_HI);
	writel(AKF_V1_LITTLE_ENDIAN,
	       asp->akf_base + AKF_V1_ENDIANNESS);

	/* start ASP */
	writel(AKF_V1_CPU_CONTROL_START,
	       asp->akf_base + AKF_V1_CPU_CONTROL);

	return 0;
}

static void apple_asp_rtkit_crashed(void *cookie, const void *crashlog,
				    size_t crashlog_size)
{
	struct apple_asp *asp = cookie;

        dev_err(asp->dev, "RTKit crashed; unable to recover without a reboot");
	// TODO cleanup and remove disk
}

// TODO NAND information exposed via sysfs

static void apple_asp_setup_queue(struct apple_asp *asp)
{
	u64 rtk_msg;

	dev_dbg(asp->dev, "Initializing command buffer");

	asp->q = dma_alloc_coherent(asp->dev, ASP_QUEUE_SIZE, &asp->q_iova,
				    GFP_KERNEL);

	if (!asp->q) {
		asp->init_ret = -ENOMEM;
		complete(&asp->init_done);
		return;
	}

	rtk_msg = FIELD_PREP(ASP_SET_CMDBUF_BASE, asp->q_iova)
		  | FIELD_PREP(ASP_SET_CMDBUF_SIZE_17_6, ASP_QUEUE_SIZE >> 6)
		  | FIELD_PREP(ASP_MSG_TYPE, ASP_MSG_TYPE_SET_CMDBUF);

	asp->init_ret = apple_rtkit_send_message(asp->rtk, asp->ep, rtk_msg,
						 NULL, false);
	return;
}

static void apple_asp_setup_tags(struct apple_asp *asp)
{
	u64 rtk_msg;

	for (u8 i = 0; i < ASP_NUM_TAGS; i++) {
		rtk_msg = FIELD_PREP(ASP_SET_TAG_CMDBUF_TAG, i)
			  | FIELD_PREP(ASP_SET_TAG_CMDBUF_SIZE_17_6,
				       ASP_TAG_SIZE >> 6)
			  | FIELD_PREP(ASP_SET_TAG_CMDBUF_OFF_17_6,
				       (ASP_TAG_SIZE * i) >> 6)
			  | FIELD_PREP(ASP_MSG_TYPE, ASP_MSG_TYPE_SET_TAG_OFF);

		asp->init_ret = apple_rtkit_send_message(asp->rtk, asp->ep,
							 rtk_msg, NULL, false);
		if (asp->init_ret < 0) {
			complete(&asp->init_done);
			return;
		}
	}
	complete(&asp->init_done);
}

static void apple_asp_handle_cq(struct apple_asp *asp, u8 tag, u8 status)
{
	struct request *rq;
	struct asp_iod *iod;

	rq = blk_mq_tag_to_rq(asp->tagset.tags[0], tag);
	if (unlikely(!rq)) {
		dev_warn(asp->dev, "invalid tag %d completed with status %d",
			 tag, status);
		return;
	}

	dma_rmb();

	iod = blk_mq_rq_to_pdu(rq);
	iod->status = status;

	if (iod->buffer) {
		void *cmdd = asp_q_cmd_data(asp, &iod->cmd);
		memcpy(iod->buffer, cmdd, iod->buf_len);
	}

	blk_mq_complete_request(rq);
}

static void apple_asp_rtkit_recv(void *cookie, u8 endpoint, u64 message)
{
	struct apple_asp *asp = cookie;

	if (endpoint != asp->ep) {
		dev_warn(asp->dev, "Message to unknown endpoint %d", endpoint);
		return;
	}

	if (asp->init_ret < 0)
		return;

	u8 type = FIELD_GET(ASP_MSG_TYPE, message);

	/* noise */
	if (type == ASP_MSG_TYPE_UNK4)
		return;

	if (type != ASP_MSG_TYPE_COMPLETE) {
		dev_warn(asp->dev, "Unhandled ASP message type %d", type);
		return;
	}

	u8 tag = FIELD_GET(ASP_COMPLETION_TAG, message);
	u8 status = FIELD_GET(ASP_COMPLETION_STATUS, message);

	if (tag == ASP_COMPLETION_TAG_READY
	    || tag == ASP_COMPLETION_TAG_CMDBUF_OK) {

		if (status != ASP_COMPLETION_STATUS_INIT) {
			dev_err(asp->dev,
				"Unexpected init message status: %d", status);
			asp->init_ret = -EIO;
			return;
		}

		if (tag == ASP_COMPLETION_TAG_READY)
			return apple_asp_setup_queue(asp);
		else if (tag == ASP_COMPLETION_TAG_CMDBUF_OK)
			return apple_asp_setup_tags(asp);
	}


	if (tag >= ASP_NUM_TAGS) {
		dev_warn(asp->dev, "ASP tag %d out of bounds\n", tag);
		return;
	}

	apple_asp_handle_cq(asp, tag, status);
}

static void apple_asp_rtkit_epmap_done(void *cookie)
{
	struct apple_asp *asp = cookie;
	int ret;
	u16 major_ver, minor_ver;

	apple_rtkit_protocol_version(asp->rtk, &major_ver, &minor_ver);

	if (major_ver == 10 && minor_ver < 2)
		asp->ep = 0x5;
	else if (major_ver == 10)
		asp->ep = 0x6;
	else
		asp->ep = 0x20;

	if (!asp->ep) {
		dev_err(asp->dev, "Could not determine ASP endpoint");
		return;
	}

	dev_dbg(asp->dev, "ASP endpoint is %d", asp->ep);

	ret = apple_rtkit_start_ep(asp->rtk, asp->ep);
	if (ret < 0) {
		dev_err(asp->dev, "Could not start ASP endpoint: %d", ret);
		return;
	}
}

static const struct apple_rtkit_ops apple_asp_rtkit_ops = {
	.crashed = apple_asp_rtkit_crashed,
	.recv_message = apple_asp_rtkit_recv,
	.epmap_done = apple_asp_rtkit_epmap_done,
};

static int apple_asp_probe(struct platform_device *pdev)
{
	struct apple_asp *asp;
        struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	asp = devm_kzalloc(dev, sizeof(*asp), GFP_KERNEL);
	if (!asp)
		return -ENOMEM;

	asp->akf_base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(asp->akf_base))
		return PTR_ERR(asp->akf_base);

	asp->dev = dev;

	/* limit of ARMv7 long descriptor format */
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40)))
		return -ENXIO;

	ret = apple_asp_setup_firmware(asp);
	if (ret < 0)
		return ret;

 	asp->rtk = devm_apple_rtkit_init(dev, asp, NULL, 0, &apple_asp_rtkit_ops);
	if (IS_ERR(asp->rtk)) {
		ret = dev_err_probe(dev, PTR_ERR(asp->rtk),
				    "Failed to initialize RTKit");
		goto out_stop_iop;
	}

	init_completion(&asp->init_done);

	ret = apple_rtkit_wake(asp->rtk);
	if (ret) {
		ret = dev_err_probe(dev, ret, "Failed to wake ASP");
		goto out_stop_iop;
	}

	/* error already printed in apple_asp_rtkit_epmap_done() */
	if (!asp->ep) {
		ret = -ENXIO;
		goto out_rtkit;
	}

	if (!wait_for_completion_timeout(&asp->init_done,
					msecs_to_jiffies(ASP_TIMEOUT_MS))) {
		ret = dev_err_probe(dev, -ETIMEDOUT, "ASP Init timeout");
		goto out_rtkit;
	}

	if (asp->init_ret < 0) {
		ret = dev_err_probe(dev, asp->init_ret, "ASP Init Error");
		goto out_rtkit;
	}

	platform_set_drvdata(pdev, asp);

	dev_dbg(dev, "ASP is alive!");

	ret = apple_asp_start_disk(asp);
	if (ret < 0)
		goto out_rtkit;

	return 0;

out_rtkit:
	apple_rtkit_shutdown(asp->rtk);
out_stop_iop:
	writel(0, asp->akf_base + AKF_V1_CPU_CONTROL_START);

	return ret;
}

static void apple_asp_remove(struct platform_device *pdev)
{
	dev_notice(&pdev->dev, "Removing ASP");

	struct apple_asp *asp = platform_get_drvdata(pdev);

	for (int i = ASP_QUEUE_TYPE_USERAREA; i < ASP_QUEUE_TYPE_COUNT; i++) {
		del_gendisk(asp->q_data[i].disk);
		put_disk(asp->q_data[i].disk);
	}

	blk_mq_destroy_queue(asp->admin_rq);
	blk_put_queue(asp->admin_rq);
	blk_mq_free_tag_set(&asp->tagset);

	if (apple_rtkit_is_running(asp->rtk))
		 apple_rtkit_shutdown(asp->rtk);

	ida_free(&asp_instance_ida, asp->instance);

	dma_free_coherent(asp->dev, ASP_QUEUE_SIZE, asp->q, asp->q_iova);

	writel(0, asp->akf_base + AKF_V1_CPU_CONTROL_START);
}


static const struct of_device_id apple_asp_of_match[] = {
	{ .compatible = "apple,s5l8960x-ans" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_asp_of_match);

static struct platform_driver apple_asp_driver = {
	.driver = {
		.name = "apple-asp",
		.of_match_table = apple_asp_of_match,
	},
	.probe = apple_asp_probe,
	.remove = apple_asp_remove,
	.shutdown = apple_asp_remove,
};

#if 0
static int apple_asp_init(void)
{
	int ret;

	pr_debug("Driver for Apple Annoying Non-stadnard Storage.");

	ret = register_blkdev(0, DEVICE_NAME);
	if (ret < 0) {
		pr_err("%s: register_blkdev() failed: %d\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&apple_asp_driver);
	if (ret < 0)
		unregister_blkdev(asp_major, DEVICE_NAME);

	return ret;
}

static void __exit apple_asp_exit(void)
{
	unregister_blkdev(asp_major, DEVICE_NAME);
}

module_init(apple_asp_init);
module_exit(apple_asp_exit);
#endif

module_platform_driver(apple_asp_driver);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple ANS1 Storage Processor Driver");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
