// SPDX-License-Identifier: GPL-2.0
/*
 * IMG DEC VXD Core component function implementations
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gfp.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "img_dec_common.h"
#include "vxd_pvdec_priv.h"

#define VXD_RENDEC_SIZE (5 * 1024 * 1024)

#define VXD_MSG_CNT_SHIFT 8
#define VXD_MSG_CNT_MASK 0xff00
#define VXD_MAX_MSG_CNT ((1 << VXD_MSG_CNT_SHIFT) - 1)
#define VXD_MSG_STR_MASK 0xff
#define VXD_INVALID_ID (-1)

#define MAP_FIRMWARE_TO_STREAM 1

/* Has to be used with VXD->mutex acquired! */
#define VXD_GEN_MSG_ID(VXD, STR_ID, MSG_ID) \
	do { \
		typeof(VXD) __VXD = VXD; \
		typeof(STR_ID) __STR_ID = STR_ID; \
		WARN_ON((__STR_ID) > VXD_MSG_STR_MASK); \
		(__VXD)->msg_cnt = (__VXD)->msg_cnt + 1 % (VXD_MAX_MSG_CNT); \
		(MSG_ID) = ((__VXD)->msg_cnt << VXD_MSG_CNT_SHIFT) | \
			((__STR_ID) & VXD_MSG_STR_MASK); \
	} while (0)

/* Have to be used with VXD->mutex acquired! */
#define VXD_RET_MSG_ID(VXD) ((VXD)->msg_cnt--)

#define VXD_MSG_ID_GET_STR_ID(MSG_ID) \
	((MSG_ID) & VXD_MSG_STR_MASK)

#define VXD_MSG_ID_GET_CNT(MSG_ID) \
	(((MSG_ID) & VXD_MSG_CNT_MASK) >> VXD_MSG_CNT_SHIFT)

static const char *drv_fw_name = "pvdec_full_bin.fw";

/* Driver context */
static struct {
	/* Available memory heaps. List of <struct vxd_heap> */
	struct list_head heaps;
	/* heap id for all internal allocations (rendec, firmware) */
	int internal_heap_id;

	/* Memory Management context for driver */
	struct mem_ctx *mem_ctx;

	/* List of associated <struct vxd_dev> */
	struct list_head devices;

	/* Virtual addresses of shared buffers, common for all streams. */
	struct {
		u32 fw_addr; /* Firmware blob */
		u32 rendec_addr; /* Rendec buffer */
	} virt_space;

	int initialised;
} vxd_drv;

/*
 * struct vxd_heap - node for heaps list
 * @id:   heap id
 * @list: Entry in <struct vxd_drv:heaps>
 */
struct vxd_heap {
	int id;
	struct list_head list;
};

static void img_mmu_callback(enum mmu_callback_type callback_type,
			     int buff_id, void *data)
{
	struct vxd_dev *vxd = data;

	if (!vxd)
		return;

	if (callback_type == MMU_CALLBACK_MAP)
		return;

	if (vxd->hw_on)
		vxd_pvdec_mmu_flush(vxd->dev, vxd->reg_base);
}

static int vxd_is_apm_required(struct vxd_dev *vxd)
{
	return vxd->hw_on;
}

/*
 * Power on the HW.
 * Call with vxd->mutex acquired.
 */
static int vxd_make_hw_on_locked(struct vxd_dev *vxd, u32 fw_ptd)
{
	u32 fw_size;
	struct vxd_fw_hdr *fw_hdr;
	struct vxd_ena_params ena_params;
	int ret;

	dev_dbg(vxd->dev, "%s:%d\n", __func__, __LINE__);

	if (vxd->hw_on)
		return 0;

	dev_dbg(vxd->dev, "%s: enabling HW\n", __func__);

	fw_size = vxd->firmware.fw_size;
	fw_hdr = vxd->firmware.hdr;
	if (!fw_size || !fw_hdr) {
		dev_err(vxd->dev, "%s: firmware missing!\n", __func__);
		return -ENOENT;
	}

	memset(&ena_params, 0, sizeof(struct vxd_ena_params));

	ena_params.fw_buf_size = fw_size - sizeof(struct vxd_fw_hdr);
	ena_params.fw_buf_virt_addr = vxd_drv.virt_space.fw_addr;
	ena_params.ptd = fw_ptd;
	ena_params.boot_poll.msleep_cycles = 50;
	ena_params.crc = 0;
	ena_params.rendec_addr = vxd_drv.virt_space.rendec_addr;
	ena_params.rendec_size = (VXD_NUM_PIX_PIPES(vxd->props) *
		VXD_RENDEC_SIZE) / 4096u;

	ena_params.secure = 0;
	ena_params.wait_dbg_fifo = 0;
	ena_params.mem_staller.data = NULL;
	ena_params.mem_staller.size = 0;

	ret = vxd_pvdec_ena(vxd->dev, vxd->reg_base, &ena_params,
			    fw_hdr, &vxd->freq_khz);
	/*
	 * Ignore the return code, proceed as usual, it will be returned anyway.
	 * The HW is turned on, so we can perform post mortem analysis,
	 * and collect the fw logs when available.
	 */

	vxd->hw_on = 1;

	return ret;
}

/*
 * Power off the HW.
 * Call with vxd->mutex acquired.
 */
static void vxd_make_hw_off_locked(struct vxd_dev *vxd, bool suspending)
{
	int ret;

	if (!vxd->hw_on)
		return;

	dev_dbg(vxd->dev, "%s:%d\n", __func__, __LINE__);

	ret = vxd_pvdec_dis(vxd->dev, vxd->reg_base);
	vxd->hw_on = 0;
	if (ret)
		dev_err(vxd->dev, "%s: failed to power off the VXD!\n",
			__func__);
}

/*
 * Moves all valid items from the queue of items being currently processed to
 * the pending queue.
 * Call with vxd->mutex locked
 */
static void vxd_rewind_msgs_locked(struct vxd_dev *vxd)
{
	struct vxd_item *item, *tmp;

	if (list_empty(&vxd->msgs))
		return;

	list_for_each_entry_safe(item, tmp, &vxd->msgs, list)
		list_move(&item->list, &vxd->pend);
}

static void vxd_report_item_locked(struct vxd_dev *vxd,
				   struct vxd_item *item, u32 flags)
{
	struct vxd_stream *stream;

	list_del(&item->list);
	stream = idr_find(&vxd->streams, item->stream_id);
	if (!stream) {
		/*
		 * Failed to find associated stream. Probably it was
		 * already destroyed -- drop the item
		 */
		dev_dbg(vxd->dev, "%s: drop item %p [0x%x]\n",
			__func__, item, item->msg_id);
		kfree(item);
	} else {
		item->msg.out_flags |= flags;
		list_add_tail(&item->list, &stream->ctx->items_done);
		dev_dbg(vxd->dev, "%s: waking %p\n", __func__, stream->ctx);

		dev_info(vxd->dev, "%s: signaling worker for %p\n", __func__,
			 stream->ctx);
		schedule_work(&stream->ctx->work);
	}
}

/*
 * Rewind all items to the pending queue and report those to listener.
 * Postpone the reset.
 * Call with vxd->mutex acquired.
 */
static void vxd_emrg_reset_locked(struct vxd_dev *vxd, u32 flags)
{
	dev_dbg(vxd->dev, "%s: enter\n", __func__);

	cancel_delayed_work(&vxd->dwork);

	vxd->emergency = 1;

	/*
	 * If the firmware sends more than one reply per item, it's possible
	 * that corresponding item was already removed from vxd-msgs, but the
	 * HW was still processing it and MMU page fault could happen and
	 * trigger execution of this function. So make sure that vxd->msgs
	 * is not empty before rewinding items.
	 */
	if (!list_empty(&vxd->msgs))
		/* Move all valid items to the pending queue */
		vxd_rewind_msgs_locked(vxd);

	{
		struct vxd_item *item, *tmp;

		list_for_each_entry_safe(item, tmp, &vxd->pend, list) {
			/*
			 * Exclusive items that were on the pending list
			 * must be reported as canceled
			 */
			if ((item->msg.out_flags & VXD_FW_MSG_FLAG_EXCL) &&
			    !item->msg_id)
				item->msg.out_flags |= VXD_FW_MSG_FLAG_CANCELED;

			vxd_report_item_locked(vxd, item, flags);
		}
	}
}

static void vxd_handle_io_error_locked(struct vxd_dev *vxd)
{
	struct vxd_item *item, *tmp;
	u32 pend_flags = !vxd->hw_on ? VXD_FW_MSG_FLAG_DEV_ERR :
		VXD_FW_MSG_FLAG_CANCELED;

	list_for_each_entry_safe(item, tmp, &vxd->msgs, list)
		vxd_report_item_locked(vxd, item, VXD_FW_MSG_FLAG_DEV_ERR);

	list_for_each_entry_safe(item, tmp, &vxd->pend, list)
		vxd_report_item_locked(vxd, item, pend_flags);
}

static void vxd_sched_worker_locked(struct vxd_dev *vxd, u32 delay_ms)
{
	unsigned long work_at = jiffies + msecs_to_jiffies(delay_ms);
	int ret;

	/*
	 * Try to queue the work.
	 * This may be also called from the worker context,
	 * so we need to re-arm anyway in case of error
	 */
	ret = schedule_delayed_work(&vxd->dwork, work_at - jiffies);
	if (ret) {
		/* Work is already in the queue */

		/*
		 * Check if new requested time is "before"
		 * the last "time" we scheduled this work at,
		 * if not, do nothing, the worker will do
		 * recalculation for APM/DWR afterwards
		 */
		if (time_before(work_at, vxd->work_sched_at)) {
			/*
			 * Canceling & rescheduling might be problematic,
			 * so just modify it, when needed
			 */
			ret = mod_delayed_work(system_wq, &vxd->dwork,
					       work_at - jiffies);
			if (!ret) {
				dev_err(vxd->dev,
					"%s: failed to modify work!\n",
					__func__);
			}
			/*
			 * Record the 'time' this work
			 * has been rescheduled at
			 */
			vxd->work_sched_at = work_at;
		}
	} else {
		/* Record the 'time' this work has been scheduled at */
		vxd->work_sched_at = work_at;
	}
}

static void vxd_monitor_locked(struct vxd_dev *vxd)
{
	/* HW is dead, not much sense in rescheduling */
	if (vxd->hw_dead)
		return;

	/*
	 * We are not processing anything, but pending list is not empty
	 * probably the message fifo is full, so retrigger the worker.
	 */
	if (!list_empty(&vxd->pend) && list_empty(&vxd->msgs))
		vxd_sched_worker_locked(vxd, 1);

	if (list_empty(&vxd->pend) && list_empty(&vxd->msgs) &&
	    vxd_is_apm_required(vxd)) {
		dev_dbg(vxd->dev, "%s: scheduling APM work (%d ms)!\n",
			__func__, vxd->hw_pm_delay);
		/*
		 * No items to process and no items being processed -
		 * disable the HW
		 */
		vxd->pm_start = jiffies;
		vxd_sched_worker_locked(vxd, vxd->hw_pm_delay);
		return;
	}

	if (vxd->hw_dwr_period > 0 && !list_empty(&vxd->msgs)) {
		dev_dbg(vxd->dev, "%s: scheduling DWR work (%d ms)!\n",
			__func__, vxd->hw_dwr_period);
		vxd->dwr_start = jiffies;
		vxd_sched_worker_locked(vxd, vxd->hw_dwr_period);
	}
}

/*
 * Take first item from pending list and submit it to the hardware.
 * Has to be called with vxd->mutex locked.
 */
static int vxd_sched_single_locked(struct vxd_dev *vxd)
{
	struct vxd_item *item = NULL;
	size_t msg_size;
	int ret;

	item = list_first_entry(&vxd->pend, struct vxd_item, list);

	msg_size = item->msg.payload_size / sizeof(u32);

	dev_dbg(vxd->dev, "%s: checking msg_size: %zu, item: %p\n",
		__func__, msg_size, item);

	/*
	 * In case of exclusive item check if hw/fw is
	 * currently processing anything.
	 * If so we need to wait until items are returned back.
	 */
	if ((item->msg.out_flags & VXD_FW_MSG_FLAG_EXCL) &&
	    !list_empty(&vxd->msgs) &&
	    /*
	     * We can move forward if message
	     * is about to be dropped.
	     */
	    !(item->msg.out_flags & VXD_FW_MSG_FLAG_DROP))

		ret = -EBUSY;
	else
		/*
		 * Check if there's enough space
		 * in comms RAM to submit the message.
		 */
		ret = vxd_pvdec_msg_fit(vxd->dev, vxd->reg_base, msg_size);

	if (ret == 0) {
		u16 msg_id;

		VXD_GEN_MSG_ID(vxd, item->stream_id, msg_id);

		/* submit the message to the hardware */
		ret = vxd_pvdec_send_msg(vxd->dev, vxd->reg_base,
					 (u32 *)item->msg.payload, msg_size,
					 msg_id, vxd);
		if (ret) {
			dev_err(vxd->dev, "%s: failed to send msg!\n",
				__func__);
			VXD_RET_MSG_ID(vxd);
		} else {
			if (item->msg.out_flags & VXD_FW_MSG_FLAG_DROP) {
				list_del(&item->list);
				kfree(item);
				dev_dbg(vxd->dev,
					"%s: drop msg 0x%x! (user requested)\n",
					__func__, msg_id);
			} else {
				item->msg_id = msg_id;
				dev_dbg(vxd->dev,
					"%s: moving item %p, id 0x%x to msgs\n",
					__func__, item, item->msg_id);
				list_move(&item->list, &vxd->msgs);
			}

			vxd_monitor_locked(vxd);
		}

	} else if (ret == -EINVAL) {
		dev_warn(vxd->dev, "%s: invalid msg!\n", __func__);
		vxd_report_item_locked(vxd, item, VXD_FW_MSG_FLAG_INV);
		/*
		 * HW is ok, the message was invalid, so don't return an
		 * error
		 */
		ret = 0;
	} else if (ret == -EBUSY) {
		/*
		 * Not enough space. Message is already in the pending queue,
		 * so it will be submitted once we've got space. Delayed work
		 * might have been canceled (if we are currently processing
		 * threaded irq), so make sure that DWR will trigger if it's
		 * enabled.
		 */
		vxd_monitor_locked(vxd);
	} else if (ret != 0) {
		dev_err(vxd->dev, "%s: failed to check space for msg!\n",
			__func__);
	}

	return ret;
}

/*
 * Take items from pending list and submit them to the hardware, if space is
 * available in the ring buffer.
 * Call with vxd->mutex locked
 */
static void vxd_schedule_locked(struct vxd_dev *vxd)
{
	bool emergency = vxd->emergency;
	int ret;

	/* if HW is dead, inform the UM and skip */
	if (vxd->hw_dead) {
		vxd_handle_io_error_locked(vxd);
		return;
	}

	if (!vxd->hw_on && !list_empty(&vxd->msgs))
		dev_err(vxd->dev,
			"%s: msgs not empty when the HW is off!\n", __func__);

	if (list_empty(&vxd->pend)) {
		vxd_monitor_locked(vxd);
		return;
	}

	/*
	 * If the emergency routine was fired, the hw was left ON,so the UM
	 * could do the post mortem analysis before submitting the next items.
	 * Now we can switch off the hardware.
	 */
	if (emergency) {
		vxd->emergency = 0;
		vxd_make_hw_off_locked(vxd, false);
		usleep_range(1000, 2000);
	}

	/* Try to schedule */
	ret = 0;
	while (!list_empty(&vxd->pend) && ret == 0) {
		struct vxd_item *item;
		struct vxd_stream *stream;

		item = list_first_entry(&vxd->pend, struct vxd_item, list);
		stream = idr_find(&vxd->streams, item->stream_id);

		ret = vxd_make_hw_on_locked(vxd, stream->ptd);
		if (ret) {
			dev_err(vxd->dev,
				"%s: failed to start HW!\n", __func__);
			vxd->hw_dead = 1;
			vxd_handle_io_error_locked(vxd);
			return;
		}

		ret = vxd_sched_single_locked(vxd);
	}

	if (ret != 0 && ret != -EBUSY) {
		dev_err(vxd->dev, "%s: failed to schedule, emrg: %d!\n",
			__func__, emergency);
		if (emergency) {
			/*
			 * Failed to schedule in the emergency mode --
			 * there's no hope. Power off the HW, mark all
			 * items as failed and return them.
			 */
			vxd_handle_io_error_locked(vxd);
			return;
		}
		/* Let worker try to handle it */
		vxd_sched_worker_locked(vxd, 0);
	}
}

static void stream_worker(struct work_struct *work)
{
	struct vxd_dec_ctx *ctx = container_of(work, struct vxd_dec_ctx, work);
	struct vxd_dev *vxd = ctx->dev;
	struct vxd_item *item;

	dev_dbg(vxd->dev, "%s: got work for ctx %p\n", __func__, ctx);

	mutex_lock(&ctx->mutex);

	while (!list_empty(&ctx->items_done)) {
		item = list_first_entry(&ctx->items_done, struct vxd_item,
					list);

		item->msg.out_flags &= VXD_FW_MSG_RD_FLAGS_MASK;

		dev_info(vxd->dev,
			 "%s: item: %p, payload_size: %d, flags: 0x%x\n",
			 __func__, item, item->msg.payload_size,
			 item->msg.out_flags);

		if (ctx->cb)
			ctx->cb(ctx->res_str_id, item->msg.payload,
				item->msg.payload_size, item->msg.out_flags);

		list_del(&item->list);
	}
	mutex_unlock(&ctx->mutex);
}

int vxd_create_ctx(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx)
{
	int ret = 0;

	dev_dbg(vxd->dev, "%s: enter\n", __func__);

	/* Create memory management context for HW buffers */
	ret = img_mem_create_ctx(&ctx->mem_ctx);
	if (ret) {
		dev_err(vxd->dev, "%s: failed to create mem context (err:%d)!\n",
			__func__, ret);
		return ret;
	}

	ret = img_mmu_ctx_create(vxd->dev, vxd->mmu_config_addr_width,
				 ctx->mem_ctx, vxd_drv.internal_heap_id,
				 img_mmu_callback, vxd, &ctx->mmu_ctx);
	if (ret) {
		dev_err(vxd->dev, "%s:%d: failed to create mmu ctx\n",
			__func__, __LINE__);
		ret = -EPERM;
		goto out_destroy_ctx;
	}

	ret = img_mmu_map(ctx->mmu_ctx, vxd->mem_ctx,
			  vxd->firmware.buf_id, vxd_drv.virt_space.fw_addr,
			  VXD_MMU_PTD_FLAG_READ_ONLY);
	if (ret) {
		dev_err(vxd->dev, "%s:%d: failed to map firmware buffer\n",
			__func__, __LINE__);
		ret = -EPERM;
		goto out_destroy_mmu_ctx;
	}

	ret = img_mmu_map(ctx->mmu_ctx, vxd->mem_ctx,
			  vxd->rendec_buf_id, vxd_drv.virt_space.rendec_addr,
			  VXD_MMU_PTD_FLAG_NONE);
	if (ret) {
		dev_err(vxd->dev, "%s:%d: failed to map rendec buffer\n",
			__func__, __LINE__);
		ret = -EPERM;
		goto out_unmap_fw;
	}

	ret = img_mmu_get_ptd(ctx->mmu_ctx, &ctx->ptd);
	if (ret) {
		dev_err(vxd->dev, "%s:%d: failed to get PTD\n",
			__func__, __LINE__);
		ret = -EPERM;
		goto out_unmap_rendec;
	}

	/* load fw - turned Hw on */
	ret = vxd_make_hw_on_locked(vxd, ctx->ptd);
	if (ret) {
		dev_err(vxd->dev, "%s:%d: failed to start HW\n",
			__func__, __LINE__);
		ret = -EPERM;
		vxd->hw_on = false;
		goto out_unmap_rendec;
	}

	INIT_WORK(&ctx->work, stream_worker);

	vxd->fw_refcnt++;

	return ret;

out_unmap_rendec:
	img_mmu_unmap(ctx->mmu_ctx, vxd->mem_ctx, vxd->rendec_buf_id);
out_unmap_fw:
	img_mmu_unmap(ctx->mmu_ctx, vxd->mem_ctx,
		      vxd->firmware.buf_id);

out_destroy_mmu_ctx:
	img_mmu_ctx_destroy(ctx->mmu_ctx);
out_destroy_ctx:
	img_mem_destroy_ctx(ctx->mem_ctx);
	return ret;
}

void vxd_destroy_ctx(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx)
{
	vxd->fw_refcnt--;

	flush_work(&ctx->work);

	img_mmu_unmap(ctx->mmu_ctx, vxd->mem_ctx, vxd->rendec_buf_id);

	img_mmu_unmap(ctx->mmu_ctx, vxd->mem_ctx, vxd->firmware.buf_id);

	img_mmu_ctx_destroy(ctx->mmu_ctx);

	img_mem_destroy_ctx(ctx->mem_ctx);

	if (vxd->fw_refcnt == 0) {
		dev_info(vxd->dev, "FW: put %s\n", drv_fw_name);
		/* Poke the monitor to finally switch off the hw, when needed */
		vxd_monitor_locked(vxd);
	}
}

/* Top half */
irqreturn_t vxd_handle_irq(struct device *dev)
{
	struct vxd_dev *vxd = dev_get_drvdata(dev);
	struct vxd_hw_state *hw_state = &vxd->state.hw_state;
	int ret;

	if (!vxd)
		return IRQ_NONE;

	ret = vxd_pvdec_clear_int(vxd->reg_base, &hw_state->irq_status);

	if (!hw_state->irq_status || ret == IRQ_NONE)
		dev_warn(dev, "Got spurious interrupt!\n");

	return ret;
}

static void vxd_drop_msg_locked(const struct vxd_dev *vxd)
{
	int ret;

	ret = vxd_pvdec_recv_msg(vxd->dev, vxd->reg_base, NULL, 0,
				 (struct vxd_dev *)vxd);
	if (ret)
		dev_warn(vxd->dev, "%s: failed to receive msg!\n", __func__);
}

static void vxd_dbg_dump_msg(const struct device *dev, const char *func,
			     const u32 *payload, size_t msg_size)
{
	unsigned int i;

	for (i = 0; i < msg_size; i++)
		dev_dbg(dev, "%s: msg %d: 0x%08x\n", func, i, payload[i]);
}

static struct vxd_item *vxd_get_orphaned_item_locked(struct vxd_dev *vxd,
						     u16 msg_id,
						     size_t msg_size)
{
	struct vxd_stream *stream;
	struct vxd_item *item;
	u16 str_id = VXD_MSG_ID_GET_STR_ID(msg_id);

	/* Try to find associated stream */
	stream = idr_find(&vxd->streams, str_id);
	if (!stream) {
		/* Failed to find associated stream. */
		dev_dbg(vxd->dev, "%s: failed to find str_id: %u\n",
			__func__, str_id);
		return NULL;
	}

	item = kzalloc(sizeof(*item) + (msg_size * sizeof(u32)),
		       GFP_KERNEL);
	if (!item)
		return NULL;

	item->msg.out_flags = 0;
	item->stream_id = str_id;
	item->msg.payload_size = msg_size * sizeof(u32);
	if (vxd_pvdec_recv_msg(vxd->dev, vxd->reg_base,
			       item->msg.payload, msg_size, vxd)) {
		dev_err(vxd->dev, "%s: failed to receive msg from VXD!\n",
			__func__);
		item->msg.out_flags |= VXD_FW_MSG_FLAG_DEV_ERR;
	}
	dev_dbg(vxd->dev, "%s: item: %p str_id: %u\n", __func__, item, str_id);
	/*
	 * Need to put this item on the vxd->msgs list.
	 * It will be removed after.
	 */
	list_add_tail(&item->list, &vxd->msgs);

	vxd_dbg_dump_msg(vxd->dev, __func__, item->msg.payload, msg_size);

	return item;
}

/*
 * Fetch and process a single message from the MTX->host ring buffer.
 * <no_more> parameter is used to indicate if there are more messages pending.
 * <fatal> parameter indicates if there is some serious situation detected.
 * Has to be called with vxd->mutex locked.
 */
static void vxd_handle_single_msg_locked(struct vxd_dev *vxd,
					 bool *no_more, bool *fatal)
{
	int ret;
	u16 msg_id, str_id;
	size_t msg_size; /* size in dwords */
	struct vxd_item *item = NULL, *tmp, *it;
	struct vxd_stream *stream;
	struct device *dev = vxd->dev;
	bool not_last_msg;

	/* get the message size and id */
	ret = vxd_pvdec_pend_msg_info(dev, vxd->reg_base, &msg_size, &msg_id,
				      &not_last_msg);
	if (ret) {
		dev_err(dev, "%s: failed to get pending msg size!\n", __func__);
		*no_more = true; /* worker will HW failure */
		return;
	}

	if (msg_size == 0) {
		*no_more = true;
		return;
	}
	*no_more = false;

	str_id = VXD_MSG_ID_GET_STR_ID(msg_id);
	dev_dbg(dev, "%s: [msg] size: %zu, cnt: %u, str_id: %u, id: 0x%x\n",
		__func__, msg_size, VXD_MSG_ID_GET_CNT(msg_id),
			str_id, msg_id);
	dev_dbg(dev, "%s: [msg] not last: %u\n", __func__, not_last_msg);

	cancel_delayed_work(&vxd->dwork);

	/* Find associated item */
	list_for_each_entry_safe_reverse(it, tmp, &vxd->msgs, list) {
		dev_dbg(dev, "%s: checking item %p [0x%x] [des: %d]\n",
			__func__, it, it->msg_id, it->destroy);
		if (it->msg_id == msg_id) {
			item = it;
			break;
		}
	}

	dev_dbg(dev, "%s: found item %p [destroy: %d]\n",
		__func__, item, item ? item->destroy : VXD_INVALID_ID);

	/* Find associated stream */
	stream = idr_find(&vxd->streams, str_id);
	/*
	 * Check for firmware condition in case
	 * when unexpected item is received.
	 */
	if (!item && !stream &&
	    vxd_pvdec_check_fw_status(dev, vxd->reg_base)) {
		struct vxd_item *orphan;
		/*
		 * Lets forward the fatal info to listeners first, relaying
		 * on the head of the msg queue.
		 */
		/* TODO: forward fatal info to all attached processes */
		item = list_entry(vxd->msgs.prev, struct vxd_item, list);
		orphan = vxd_get_orphaned_item_locked(vxd,
						      item->msg_id, msg_size);
		if (!orphan) {
			dev_warn(dev, "%s: drop msg 0x%x! (no orphan)\n",
				 __func__, item->msg_id);
			vxd_drop_msg_locked(vxd);
		}

		*fatal = true;
		return;
	}

	if ((item && item->destroy) || !stream) {
		/*
		 * Item was marked for destruction or we failed to find
		 * associated stream. Probably it was already destroyed --
		 * just ignore the message.
		 */
		if (item) {
			list_del(&item->list);
			kfree(item);
			item = NULL;
		}
		dev_warn(dev, "%s: drop msg 0x%x! (no owner)\n",
			 __func__, msg_id);
		vxd_drop_msg_locked(vxd);
		return;
	}

	/* Remove item from vxd->msgs list */
	if (item && item->msg_id == msg_id && !not_last_msg)
		list_del(&item->list);

	/*
	 * If there's no such item on a <being processed> list, or the one
	 * found is too small to fit the output, or it's not supposed to be
	 * released, allocate a new one.
	 */
	if (!item || (msg_size * sizeof(u32) > item->msg.payload_size) ||
	    not_last_msg) {
		struct vxd_item *new_item;

		new_item = kzalloc(sizeof(*new_item) +
				   (msg_size * sizeof(u32)), GFP_KERNEL);
		if (item) {
			if (!new_item) {
				/*
				 * Failed to allocate new item. Mark item as
				 * errored and continue best effort, provide
				 * only part of the message to the userspace
				 */
				dev_err(dev, "%s: failed to alloc new item!\n",
					__func__);
				msg_size = item->msg.payload_size / sizeof(u32);
				item->msg.out_flags |= VXD_FW_MSG_FLAG_DRV_ERR;
			} else {
				*new_item = *item;
				/*
				 * Do not free the old item if subsequent
				 * messages are expected (it also wasn't
				 * removed from the vxd->msgs list, so we are
				 * not losing a pointer here).
				 */
				if (!not_last_msg)
					kfree(item);
				item = new_item;
			}
		} else {
			if (!new_item) {
				/*
				 * We have no place to put the message, we have
				 * to drop it
				 */
				dev_err(dev, "%s: drop msg 0x%08x! (no mem)\n",
					__func__, msg_id);
				vxd_drop_msg_locked(vxd);
				return;
			}
			/*
			 * There was no corresponding item on the
			 * <being processed> list and we've allocated
			 * a new one. Initialize it
			 */
			new_item->msg.out_flags = 0;
			new_item->stream_id = str_id;
			item = new_item;
		}
	}
	ret = vxd_pvdec_recv_msg(dev, vxd->reg_base, item->msg.payload,
				 msg_size, vxd);
	if (ret) {
		dev_err(dev, "%s: failed to receive msg from VXD!\n", __func__);
		item->msg.out_flags |= VXD_FW_MSG_FLAG_DEV_ERR;
	}
	item->msg.payload_size = msg_size * sizeof(u32);

	vxd_dbg_dump_msg(dev, __func__, item->msg.payload, msg_size);

	dev_dbg(dev, "%s: adding to done list, item: %p, msg_size: %zu\n",
		__func__, item, msg_size);
	list_add_tail(&item->list, &stream->ctx->items_done);

	dev_info(dev, "%s: signaling worker for %p\n", __func__, stream->ctx);
	schedule_work(&stream->ctx->work);
}

/* Bottom half */
irqreturn_t vxd_handle_thread_irq(struct device *dev)
{
	bool no_more = false;
	bool fatal = false;
	struct vxd_dev *vxd = dev_get_drvdata(dev);
	struct vxd_hw_state *hw_state = &vxd->state.hw_state;
	irqreturn_t ret = IRQ_HANDLED;

	if (!vxd)
		return IRQ_NONE;

	mutex_lock(&vxd->mutex);

	/* Spurious interrupt? */
	if (unlikely(!vxd->hw_on || vxd->hw_dead)) {
		ret = IRQ_NONE;
		goto out_unlock;
	}

	/* Check for critical exception - only MMU faults for now */
	if (vxd_pvdec_check_irq(dev, vxd->reg_base,
				hw_state->irq_status) < 0) {
		dev_info(vxd->dev, "device MMU fault: resetting!!!\n");
		vxd_emrg_reset_locked(vxd, VXD_FW_MSG_FLAG_MMU_FAULT);
		goto out_unlock;
	}

	/*
	 * Single interrupt can correspond to multiple messages, handle them
	 * all.
	 */
	while (!no_more)
		vxd_handle_single_msg_locked(vxd, &no_more, &fatal);

	if (fatal) {
		dev_info(vxd->dev, "fw fatal condition: resetting!!!\n");
		/* Try to recover ... */
		vxd_emrg_reset_locked(vxd, VXD_FW_MSG_FLAG_FATAL);
	} else {
		/* Try to submit items to the HW */
		vxd_schedule_locked(vxd);
	}

out_unlock:
	hw_state->irq_status = 0;
	mutex_unlock(&vxd->mutex);

	return ret;
}

static void vxd_worker(struct work_struct *work)
{
	struct vxd_dev *vxd = container_of(work, struct vxd_dev, dwork.work);
	struct vxd_hw_state state = { 0 };
	struct vxd_item *item_tail;

	mutex_lock(&vxd->mutex);

	dev_dbg(vxd->dev, "%s: jif: %ld, pm: %ld dwr: %ld\n", __func__, jiffies,
		vxd->pm_start, vxd->dwr_start);

	/*
	 * Disable the hardware if it has been idle for vxd->hw_pm_delay
	 * milliseconds. Or simply leave the function without doing anything
	 * if the HW is not supposed to be turned off.
	 */
	if (list_empty(&vxd->pend) && list_empty(&vxd->msgs)) {
		if (vxd_is_apm_required(vxd)) {
			unsigned long dst = vxd->pm_start +
				msecs_to_jiffies(vxd->hw_pm_delay);

			if (time_is_before_eq_jiffies(dst)) {
				dev_dbg(vxd->dev, "%s: pm, power off\n",
					__func__);
				vxd_make_hw_off_locked(vxd, false);
			} else {
				unsigned long targ = dst - jiffies;

				dev_dbg(vxd->dev, "%s: pm, reschedule: %ld\n",
					__func__, targ);
				vxd_sched_worker_locked(vxd,
							jiffies_to_msecs(targ));
			}
		}
		goto out_unlock;
	}

	/*
	 * We are not processing anything, but pending list is not empty (if it
	 * was, we would enter <if statement> above. This can happen upon
	 * specific conditions, when input message occupies almost whole
	 * host->MTX ring buffer and is followed by large padding message.
	 */
	if (list_empty(&vxd->msgs)) {
		vxd_schedule_locked(vxd);
		goto out_unlock;
	}

	/* Skip emergency reset if it's disabled. */
	if (vxd->hw_dwr_period <= 0) {
		dev_dbg(vxd->dev, "%s: skip watchdog\n", __func__);
		goto out_unlock;
	} else {
		/* Recalculate DWR when needed */
		unsigned long dst = vxd->dwr_start +
				msecs_to_jiffies(vxd->hw_dwr_period);

		if (time_is_after_jiffies(dst)) {
			unsigned long targ = dst - jiffies;

			dev_dbg(vxd->dev, "%s: dwr, reschedule: %ld\n",
				__func__, targ);
			vxd_sched_worker_locked(vxd, jiffies_to_msecs(targ));
			goto out_unlock;
		}
	}

	/* Get ID of the oldest item being processed by the HW */
	item_tail = list_entry(vxd->msgs.prev, struct vxd_item, list);

	dev_dbg(vxd->dev, "%s: tail_item: %p, id: 0x%x\n", __func__, item_tail,
		item_tail->msg_id);

	/* Get HW and firmware state */
	vxd_pvdec_get_state(vxd->dev, vxd->reg_base,
			    VXD_NUM_PIX_PIPES(vxd->props), &state);

	if (vxd->state.msg_id_tail == item_tail->msg_id &&
	    !memcmp(&state, &vxd->state.hw_state,
		    sizeof(struct vxd_hw_state))) {
		vxd->state.msg_id_tail = 0;
		memset(&vxd->state.hw_state, 0, sizeof(vxd->state.hw_state));
		dev_info(vxd->dev, "device DWR(%ums) expired: resetting!!!\n",
			 vxd->hw_dwr_period);
		vxd_emrg_reset_locked(vxd, VXD_FW_MSG_FLAG_DWR);
	} else {
		/* Record current state */
		vxd->state.msg_id_tail = item_tail->msg_id;
		vxd->state.hw_state = state;

		/* Submit items to the HW, if space is available.  */
		vxd_schedule_locked(vxd);

		dev_dbg(vxd->dev, "%s: scheduling DWR work (%d ms)!\n",
			__func__, vxd->hw_dwr_period);
		vxd_sched_worker_locked(vxd, vxd->hw_dwr_period);
	}

out_unlock:
	mutex_unlock(&vxd->mutex);
}

/*
 * Lazy initialization of main driver context (when first core is probed -- we
 * need heap configuration from sysdev to allocate firmware buffers.
 */
int vxd_init(struct device *dev, struct vxd_dev *vxd,
	     const struct heap_config heap_configs[], int heaps)
{
	int ret, i;

	INIT_LIST_HEAD(&vxd_drv.heaps);
	vxd_drv.internal_heap_id = VXD_INVALID_ID;

	vxd_drv.mem_ctx = NULL;

	INIT_LIST_HEAD(&vxd_drv.devices);

	vxd_drv.virt_space.fw_addr = 0x42000;
	vxd_drv.virt_space.rendec_addr = 0xe0000000;

	vxd_drv.initialised = 0;

	dev_dbg(dev, "%s: vxd drv init, params:\n", __func__);

	/* Initialise memory management component */
	for (i = 0; i < heaps; i++) {
		struct vxd_heap *heap;

		dev_dbg(dev, "%s: adding heap of type %d\n",
			__func__, heap_configs[i].type);

		heap = kzalloc(sizeof(*heap), GFP_KERNEL);
		if (!heap) {
			ret = -ENOMEM;
			goto heap_add_failed;
		}

		ret = img_mem_add_heap(&heap_configs[i], &heap->id);
		if (ret < 0) {
			dev_err(dev, "%s: failed to init heap (type %d)!\n",
				__func__, heap_configs[i].type);
			kfree(heap);
			goto heap_add_failed;
		}
		list_add(&heap->list, &vxd_drv.heaps);

		/* Implicitly, first heap is used for internal allocations */
		if (vxd_drv.internal_heap_id < 0) {
			vxd_drv.internal_heap_id = heap->id;
			dev_dbg(dev, "%s: using heap %d for internal alloc\n",
				__func__, vxd_drv.internal_heap_id);
		}
	}

	/* Do not proceed if internal heap not defined */
	if (vxd_drv.internal_heap_id < 0) {
		dev_err(dev, "%s: failed to locate heap for internal alloc\n",
			__func__);
		ret = -EINVAL;
		/* Loop registered heaps just for sanity */
		goto heap_add_failed;
	}

	/* Create memory management context for HW buffers */
	ret = img_mem_create_ctx(&vxd_drv.mem_ctx);
	if (ret) {
		dev_err(dev, "%s: failed to create mem context (err:%d)!\n",
			__func__, ret);
		goto create_mem_context_failed;
	}

	vxd->mem_ctx = vxd_drv.mem_ctx;

	/* Allocate rendec buffer */
	ret = img_mem_alloc(dev, vxd_drv.mem_ctx, vxd_drv.internal_heap_id,
			    VXD_RENDEC_SIZE * VXD_NUM_PIX_PIPES(vxd->props),
			    0, &vxd->rendec_buf_id);
	if (ret) {
		dev_err(dev, "%s: alloc rendec buffer failed (err:%d)!\n",
			__func__, ret);
		goto create_mem_context_failed;
	}

	INIT_DELAYED_WORK(&vxd->dwork, vxd_worker);

	vxd_drv.initialised = 1;
	dev_dbg(dev, "%s: vxd drv init done\n", __func__);
	return 0;

create_mem_context_failed:
heap_add_failed:
	while (!list_empty(&vxd_drv.heaps)) {
		struct vxd_heap *heap;

		heap = list_first_entry(&vxd_drv.heaps, struct vxd_heap, list);
		list_del(&heap->list);
		img_mem_del_heap(heap->id);
		kfree(heap);
	}
	vxd_drv.internal_heap_id = VXD_INVALID_ID;
	return ret;
}

/*
 * Get internal_heap_id
 * TODO: Only error checking is if < 0, so if the stored value is < 0, then
 * just passing the value to caller still conveys error.
 * Caller must error check.
 */
int vxd_g_internal_heap_id(void)
{
	return vxd_drv.internal_heap_id;
}

void vxd_deinit(struct vxd_dev *vxd)
{
	cancel_delayed_work_sync(&vxd->dwork);
	vxd_make_hw_off_locked(vxd, false);

	/* Deallocate rendec buffer */
	img_mem_free(vxd_drv.mem_ctx, vxd->rendec_buf_id);

	/* Destroy memory management context */
	if (vxd_drv.mem_ctx) {
		img_mem_destroy_ctx(vxd_drv.mem_ctx);
		vxd_drv.mem_ctx = NULL;
	}

	/* Deinitialize memory management component */
	while (!list_empty(&vxd_drv.heaps)) {
		struct vxd_heap *heap;

		heap = list_first_entry(&vxd_drv.heaps, struct vxd_heap, list);
		list_del(&heap->list);
		img_mem_del_heap(heap->id);
		kfree(heap);
	}

	vxd_drv.internal_heap_id = VXD_INVALID_ID;
	vxd_drv.mem_ctx = NULL;
	vxd_drv.virt_space.fw_addr = 0x0;
	vxd_drv.virt_space.rendec_addr = 0x0;
	vxd_drv.initialised = 0;
}

static void vxd_fw_loaded(const struct firmware *fw, void *context)
{
	struct vxd_dev *vxd = context;
	size_t bin_size;
	int buf_id;
	struct vxd_fw_hdr *hdr;
	void *buf_kptr;
	int ret;

	dev_info(vxd->dev, "FW: acquired %s size %zu\n", drv_fw_name, fw->size);

	/* Sanity verification of the firmware */
	if (fw->size < sizeof(struct vxd_fw_hdr) || !fw->size) {
		dev_err(vxd->dev, "%s: firmware file too small!\n", __func__);
		goto out;
	}

	bin_size = fw->size - sizeof(struct vxd_fw_hdr);
	ret = img_mem_alloc(vxd->dev, vxd_drv.mem_ctx, vxd_drv.internal_heap_id,
			    bin_size, 0, &buf_id);
	if (ret) {
		dev_err(vxd->dev, "%s: failed to alloc fw buffer (err:%d)!\n",
			__func__, ret);
		goto out;
	}

	hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
	if (!hdr)
		goto out_release_buf;

	/* Store firmware header in vxd context */
	memcpy(hdr, fw->data, sizeof(struct vxd_fw_hdr));

	dev_info(vxd->dev, "FW: info cs: %u, bs: %u, id: 0x%08x, ts: %u\n",
		 hdr->core_size, hdr->blob_size,
		 hdr->firmware_id, hdr->timestamp);

	/* Check if header is consistent */
	if (hdr->core_size > bin_size || hdr->blob_size > bin_size) {
		dev_err(vxd->dev, "%s: got invalid firmware!\n", __func__);
		goto out_release_hdr;
	}

	/* Map the firmware buffer to CPU */
	ret = img_mem_map_km(vxd_drv.mem_ctx, buf_id);
	if (ret) {
		dev_err(vxd->dev, "%s: failed to map FW buf to cpu! (%d)\n",
			__func__, ret);
		goto out_release_hdr;
	}

	/* Copy firmware to device buffer */
	buf_kptr = img_mem_get_kptr(vxd_drv.mem_ctx, buf_id);
	memcpy(buf_kptr, fw->data + sizeof(struct vxd_fw_hdr),
	       fw->size - sizeof(struct vxd_fw_hdr));
	dev_dbg(vxd->dev, "%s: FW: copied to buffer %d kptr 0x%p\n",
		__func__, buf_id, buf_kptr);

	img_mem_sync_cpu_to_device(vxd_drv.mem_ctx, buf_id);

	vxd->firmware.fw_size = fw->size;
	vxd->firmware.buf_id = buf_id;
	vxd->firmware.hdr = hdr;

	release_firmware(fw);
	complete_all(&vxd->firmware_loading_complete);
	return;

out_release_hdr:
	kfree(hdr);
out_release_buf:
	img_mem_free(vxd_drv.mem_ctx, buf_id);
out:
	release_firmware(fw);
	complete_all(&vxd->firmware_loading_complete);
}

/*
 * Takes the firmware from the file system and allocates a buffer
 */
int vxd_prepare_fw(struct vxd_dev *vxd)
{
	int ret;

	/* Fetch firmware from the file system */
	init_completion(&vxd->firmware_loading_complete);

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				      drv_fw_name, vxd->dev, GFP_KERNEL, vxd,
				      vxd_fw_loaded);
	if (ret < 0) {
		dev_err(vxd->dev, "request_firmware_nowait err: %d\n", ret);
		complete_all(&vxd->firmware_loading_complete);
	}

	return ret;
}

/*
 * Cleans firmware resources
 */
void vxd_clean_fw_resources(struct vxd_dev *vxd)
{
	dev_dbg(vxd->dev, "%s:%d\n", __func__, __LINE__);

	wait_for_completion(&vxd->firmware_loading_complete);

	if (vxd->firmware.fw_size) {
		img_mem_free(vxd_drv.mem_ctx, vxd->firmware.buf_id);
		kfree(vxd->firmware.hdr);
		dev_info(vxd->dev, "FW: released %s\n", drv_fw_name);
		vxd->firmware.buf_id = VXD_INVALID_ID;
	}
}

/*
 * Submit a message to the VXD.
 * <ctx> is used to verify that requested stream id (item->stream_id) is valid
 * for this ctx
 */
int vxd_send_msg(struct vxd_dec_ctx *ctx, struct vxd_fw_msg *msg)
{
	struct vxd_dev *vxd = ctx->dev;
	size_t msg_size;
	struct vxd_item *item;
	struct vxd_stream *stream;
	int ret;

	if (msg->payload_size < VXD_MIN_INPUT_SIZE)
		return -EINVAL;

	if (msg->payload_size % sizeof(u32)) {
		dev_err(vxd->dev, "msg size not aligned! (%u)\n",
			msg->payload_size);
		return -EINVAL;
	}

	msg_size = VXD_MSG_SIZE(*msg);

	if (msg_size > VXD_MAX_INPUT_SIZE)
		return -EINVAL;

	/* Verify that the gap was left for stream PTD */
	if (msg->payload[VXD_PTD_MSG_OFFSET] != 0) {
		dev_err(vxd->dev, "%s: PTD gap missing!\n", __func__);
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&ctx->mutex);
	if (ret)
		return ret;

	stream = idr_find(&vxd->streams, ctx->stream.id);
	if (!stream) {
		dev_warn(vxd->dev, "%s: invalid stream id requested! (%u)\n",
			 __func__, ctx->stream.id);

		ret = -EINVAL;
		goto out_unlock;
	}

	item = kmalloc(sizeof(*item) + msg->payload_size, GFP_KERNEL);
	if (!item) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	memcpy(&item->msg, msg, msg_size);

	msg->out_flags &= VXD_FW_MSG_WR_FLAGS_MASK;
	item->stream_id = ctx->stream.id;
	item->msg_id = 0;
	item->msg.out_flags = msg->out_flags;
	item->destroy = 0;

	/*
	 * Inject the stream PTD into the message. It was already verified that
	 * there is enough space.
	 */
	item->msg.payload[VXD_PTD_MSG_OFFSET] = stream->ptd;

	list_add_tail(&item->list, &vxd->pend);
	dev_dbg(vxd->dev,
		"%s: added item %p to pend, ptd: 0x%x, str: %u flags: 0x%x\n",
		__func__, item, stream->ptd, stream->id, item->msg.out_flags);

	vxd_schedule_locked(vxd);

out_unlock:
	mutex_unlock(&ctx->mutex);

	return ret;
}

int vxd_suspend_dev(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vxd_dev *vxd = platform_get_drvdata(pdev);

	mutex_lock(&vxd->mutex);
	dev_dbg(dev, "%s: taking a nap!\n", __func__);

	/* Cancel the worker first */
	cancel_delayed_work(&vxd->dwork);

	/* Forcing hardware disable */
	vxd_make_hw_off_locked(vxd, true);

	/* Move all valid items to the pending queue */
	vxd_rewind_msgs_locked(vxd);

	mutex_unlock(&vxd->mutex);

	return 0;
}

int vxd_resume_dev(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vxd_dev *vxd = platform_get_drvdata(pdev);
	int ret = 0;

	mutex_lock(&vxd->mutex);
	dev_dbg(dev, "%s: waking up!\n", __func__);

	mutex_unlock(&vxd->mutex);

	return ret;
}

int vxd_map_buffer_sg(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx, u32 str_id,
		      u32 buff_id, struct sg_table *sgt, u32 virt_addr,
		      u32 map_flags)
{
	struct vxd_stream *stream;
	u32 flags = VXD_MMU_PTD_FLAG_NONE;
	int ret;

	ret = mutex_lock_interruptible(&ctx->mutex);
	if (ret)
		return ret;

	stream = idr_find(&vxd->streams, str_id);
	if (!stream) {
		dev_err(vxd->dev, "%s: stream %d not found!\n", __func__,
			str_id);
		ret = -EINVAL;
		goto out_unlock;
	}

	if ((map_flags & (VXD_MAP_FLAG_READ_ONLY | VXD_MAP_FLAG_WRITE_ONLY))
			== (VXD_MAP_FLAG_READ_ONLY | VXD_MAP_FLAG_WRITE_ONLY)) {
		dev_err(vxd->dev, "%s: Bogus mapping flags 0x%x!\n", __func__,
			map_flags);
		ret = -EINVAL;
		goto out_unlock;
	}

	/* Convert permission flags to internal definitions */
	if (map_flags & VXD_MAP_FLAG_READ_ONLY)
		flags |= VXD_MMU_PTD_FLAG_READ_ONLY;

	if (map_flags & VXD_MAP_FLAG_WRITE_ONLY)
		flags |= VXD_MMU_PTD_FLAG_WRITE_ONLY;

	ret = img_mmu_map_sg(stream->mmu_ctx, ctx->mem_ctx, buff_id, sgt, virt_addr,
			     flags);
	if (ret) {
		dev_err(vxd->dev, "%s: map failed!\n", __func__);
		goto out_unlock;
	}

	dev_dbg(vxd->dev,
		"%s: mapped buf %u to 0x%08x, str_id: %u flags: 0x%x\n",
		__func__, buff_id, virt_addr, str_id, flags);

out_unlock:
	mutex_unlock(&ctx->mutex);
	return ret;
}

int vxd_map_buffer(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx, u32 str_id,
		   u32 buff_id, u32 virt_addr, u32 map_flags)
{
	struct vxd_stream *stream;
	u32 flags = VXD_MMU_PTD_FLAG_NONE;
	int ret;

	ret = mutex_lock_interruptible(&ctx->mutex);
	if (ret)
		return ret;

	stream = idr_find(&vxd->streams, str_id);
	if (!stream) {
		dev_err(vxd->dev, "%s: stream %d not found!\n", __func__,
			str_id);
		ret = -EINVAL;
		goto out_unlock;
	}

	if ((map_flags & (VXD_MAP_FLAG_READ_ONLY | VXD_MAP_FLAG_WRITE_ONLY))
			== (VXD_MAP_FLAG_READ_ONLY | VXD_MAP_FLAG_WRITE_ONLY)) {
		dev_err(vxd->dev, "%s: Bogus mapping flags 0x%x!\n", __func__,
			map_flags);
		ret = -EINVAL;
		goto out_unlock;
	}

	/* Convert permission flags to internal definitions */
	if (map_flags & VXD_MAP_FLAG_READ_ONLY)
		flags |= VXD_MMU_PTD_FLAG_READ_ONLY;

	if (map_flags & VXD_MAP_FLAG_WRITE_ONLY)
		flags |= VXD_MMU_PTD_FLAG_WRITE_ONLY;

	ret = img_mmu_map(stream->mmu_ctx, ctx->mem_ctx, buff_id, virt_addr,
			  flags);
	if (ret) {
		dev_err(vxd->dev, "%s: map failed!\n", __func__);
		goto out_unlock;
	}

	dev_dbg(vxd->dev,
		"%s: mapped buf %u to 0x%08x, str_id: %u flags: 0x%x\n",
		__func__, buff_id, virt_addr, str_id, flags);

out_unlock:
	mutex_unlock(&ctx->mutex);
	return ret;
}

int vxd_unmap_buffer(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx,
		     u32 str_id, u32 buff_id)
{
	struct vxd_stream *stream;
	int ret;

	ret = mutex_lock_interruptible(&ctx->mutex);
	if (ret)
		return ret;

	stream = idr_find(&vxd->streams, str_id);
	if (!stream) {
		dev_err(vxd->dev, "%s: stream %d not found!\n", __func__,
			str_id);
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = img_mmu_unmap(stream->mmu_ctx, ctx->mem_ctx, buff_id);
	if (ret) {
		dev_err(vxd->dev, "%s: map failed!\n", __func__);
		goto out_unlock;
	}

	dev_dbg(vxd->dev, "%s: unmapped buf %u str_id: %u\n", __func__, buff_id,
		str_id);

out_unlock: mutex_unlock(&ctx->mutex);
	return ret;
}

