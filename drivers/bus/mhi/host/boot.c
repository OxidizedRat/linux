// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/mhi.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/vmalloc.h>
#include "internal.h"

#define CAIHONG_WCN_BHI_IOVA	0xa4000000
#define CAIHONG_WCN_BHI_TEST_PHYS	0xd2000000

/* Setup RDDM vector table for RDDM transfer and program RXVEC */
int mhi_rddm_prepare(struct mhi_controller *mhi_cntrl,
		     struct image_info *img_info)
{
	struct mhi_buf *mhi_buf = img_info->mhi_buf;
	struct bhi_vec_entry *bhi_vec = img_info->bhi_vec;
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	u32 sequence_id;
	unsigned int i;
	int ret;

	for (i = 0; i < img_info->entries - 1; i++, mhi_buf++, bhi_vec++) {
		bhi_vec->dma_addr = cpu_to_le64(mhi_buf->dma_addr);
		bhi_vec->size = cpu_to_le64(mhi_buf->len);
	}

	dev_dbg(dev, "BHIe programming for RDDM\n");

	mhi_write_reg(mhi_cntrl, base, BHIE_RXVECADDR_HIGH_OFFS,
		      upper_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_RXVECADDR_LOW_OFFS,
		      lower_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_RXVECSIZE_OFFS, mhi_buf->len);
	sequence_id = MHI_RANDOM_U32_NONZERO(BHIE_RXVECSTATUS_SEQNUM_BMSK);

	ret = mhi_write_reg_field(mhi_cntrl, base, BHIE_RXVECDB_OFFS,
				  BHIE_RXVECDB_SEQNUM_BMSK, sequence_id);
	if (ret) {
		dev_err(dev, "Failed to write sequence ID for BHIE_RXVECDB\n");
		return ret;
	}

	dev_dbg(dev, "Address: %p and len: 0x%zx sequence: %u\n",
		&mhi_buf->dma_addr, mhi_buf->len, sequence_id);

	return 0;
}

/* Collect RDDM buffer during kernel panic */
static int __mhi_download_rddm_in_panic(struct mhi_controller *mhi_cntrl)
{
	int ret;
	u32 rx_status;
	enum mhi_ee_type ee;
	const u32 delayus = 2000;
	u32 retry = (mhi_cntrl->timeout_ms * 1000) / delayus;
	const u32 rddm_timeout_us = 200000;
	int rddm_retry = rddm_timeout_us / delayus;
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;

	dev_dbg(dev, "Entered with pm_state:%s dev_state:%s ee:%s\n",
		to_mhi_pm_state_str(mhi_cntrl->pm_state),
		mhi_state_str(mhi_cntrl->dev_state),
		TO_MHI_EXEC_STR(mhi_cntrl->ee));

	/*
	 * This should only be executing during a kernel panic, we expect all
	 * other cores to shutdown while we're collecting RDDM buffer. After
	 * returning from this function, we expect the device to reset.
	 *
	 * Normally, we read/write pm_state only after grabbing the
	 * pm_lock, since we're in a panic, skipping it. Also there is no
	 * guarantee that this state change would take effect since
	 * we're setting it w/o grabbing pm_lock
	 */
	mhi_cntrl->pm_state = MHI_PM_LD_ERR_FATAL_DETECT;
	/* update should take the effect immediately */
	smp_wmb();

	/*
	 * Make sure device is not already in RDDM. In case the device asserts
	 * and a kernel panic follows, device will already be in RDDM.
	 * Do not trigger SYS ERR again and proceed with waiting for
	 * image download completion.
	 */
	ee = mhi_get_exec_env(mhi_cntrl);
	if (ee == MHI_EE_MAX)
		goto error_exit_rddm;

	if (ee != MHI_EE_RDDM) {
		dev_dbg(dev, "Trigger device into RDDM mode using SYS ERR\n");
		mhi_set_mhi_state(mhi_cntrl, MHI_STATE_SYS_ERR);

		dev_dbg(dev, "Waiting for device to enter RDDM\n");
		while (rddm_retry--) {
			ee = mhi_get_exec_env(mhi_cntrl);
			if (ee == MHI_EE_RDDM)
				break;

			udelay(delayus);
		}

		if (rddm_retry <= 0) {
			/* Hardware reset so force device to enter RDDM */
			dev_dbg(dev,
				"Did not enter RDDM, do a host req reset\n");
			mhi_soc_reset(mhi_cntrl);
			udelay(delayus);
		}

		ee = mhi_get_exec_env(mhi_cntrl);
	}

	dev_dbg(dev,
		"Waiting for RDDM image download via BHIe, current EE:%s\n",
		TO_MHI_EXEC_STR(ee));

	while (retry--) {
		ret = mhi_read_reg_field(mhi_cntrl, base, BHIE_RXVECSTATUS_OFFS,
					 BHIE_RXVECSTATUS_STATUS_BMSK, &rx_status);
		if (ret)
			return -EIO;

		if (rx_status == BHIE_RXVECSTATUS_STATUS_XFER_COMPL)
			return 0;

		udelay(delayus);
	}

	ee = mhi_get_exec_env(mhi_cntrl);
	ret = mhi_read_reg(mhi_cntrl, base, BHIE_RXVECSTATUS_OFFS, &rx_status);

	dev_err(dev, "RXVEC_STATUS: 0x%x\n", rx_status);

error_exit_rddm:
	dev_err(dev, "RDDM transfer failed. Current EE: %s\n",
		TO_MHI_EXEC_STR(ee));

	return -EIO;
}

/* Download RDDM image from device */
int mhi_download_rddm_image(struct mhi_controller *mhi_cntrl, bool in_panic)
{
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	u32 rx_status;

	if (in_panic)
		return __mhi_download_rddm_in_panic(mhi_cntrl);

	dev_dbg(dev, "Waiting for RDDM image download via BHIe\n");

	/* Wait for the image download to complete */
	wait_event_timeout(mhi_cntrl->state_event,
			   mhi_read_reg_field(mhi_cntrl, base,
					      BHIE_RXVECSTATUS_OFFS,
					      BHIE_RXVECSTATUS_STATUS_BMSK,
					      &rx_status) || rx_status,
			   msecs_to_jiffies(mhi_cntrl->timeout_ms));

	return (rx_status == BHIE_RXVECSTATUS_STATUS_XFER_COMPL) ? 0 : -EIO;
}
EXPORT_SYMBOL_GPL(mhi_download_rddm_image);

static void mhi_fw_load_error_dump(struct mhi_controller *mhi_cntrl)
{
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	void __iomem *base = mhi_cntrl->bhi;
	int ret, i;
	u32 val;
	struct {
		char *name;
		u32 offset;
	} error_reg[] = {
		{ "ERROR_CODE", BHI_ERRCODE },
		{ "ERROR_DBG1", BHI_ERRDBG1 },
		{ "ERROR_DBG2", BHI_ERRDBG2 },
		{ "ERROR_DBG3", BHI_ERRDBG3 },
		{ NULL },
	};

	read_lock_bh(pm_lock);
	if (MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		for (i = 0; error_reg[i].name; i++) {
			ret = mhi_read_reg(mhi_cntrl, base, error_reg[i].offset, &val);
			if (ret)
				break;
			dev_err(dev, "Reg: %s value: 0x%x\n", error_reg[i].name, val);
		}
	}
	read_unlock_bh(pm_lock);
}

static int mhi_fw_load_bhie(struct mhi_controller *mhi_cntrl,
			    const struct mhi_buf *mhi_buf)
{
	void __iomem *base = mhi_cntrl->bhie;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	u32 tx_status, sequence_id;
	unsigned long deadline;
	unsigned int poll_ms = 0;
	int ret;

	read_lock_bh(pm_lock);
	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		read_unlock_bh(pm_lock);
		return -EIO;
	}

	sequence_id = MHI_RANDOM_U32_NONZERO(BHIE_TXVECSTATUS_SEQNUM_BMSK);
	dev_dbg(dev, "Starting image download via BHIe. Sequence ID: %u\n",
		sequence_id);
	mhi_write_reg(mhi_cntrl, base, BHIE_TXVECADDR_HIGH_OFFS,
		      upper_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_TXVECADDR_LOW_OFFS,
		      lower_32_bits(mhi_buf->dma_addr));

	mhi_write_reg(mhi_cntrl, base, BHIE_TXVECSIZE_OFFS, mhi_buf->len);

	ret = mhi_write_reg_field(mhi_cntrl, base, BHIE_TXVECDB_OFFS,
				  BHIE_TXVECDB_SEQNUM_BMSK, sequence_id);
	read_unlock_bh(pm_lock);

	if (ret)
		return ret;

	/* Poll BHIE as a diagnostic for missing completion interrupts. */
	tx_status = 0;
	ret = 1;
	deadline = jiffies + msecs_to_jiffies(mhi_cntrl->timeout_ms);
	while (true) {
		if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state))
			break;

		mhi_read_reg_field(mhi_cntrl, base, BHIE_TXVECSTATUS_OFFS,
				   BHIE_TXVECSTATUS_STATUS_BMSK, &tx_status);
		if (tx_status)
			break;

		if (time_after_eq(jiffies, deadline)) {
			ret = 0;
			break;
		}

		msleep(100);
		poll_ms += 100;
		if (!(poll_ms % 5000))
			dev_info(dev, "BHIE poll heartbeat: elapsed=%u status=%x pm_state=%lx\n",
				 poll_ms, tx_status, mhi_cntrl->pm_state);
	}
	dev_info(dev, "BHIE transfer result: wait=%d session=%u status=%x pm_state=%lx\n",
		 ret, sequence_id, tx_status, mhi_cntrl->pm_state);
	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state) ||
	    tx_status != BHIE_TXVECSTATUS_STATUS_XFER_COMPL)
		return -EIO;

	return (!ret) ? -ETIMEDOUT : 0;
}

static int mhi_fw_load_bhi(struct mhi_controller *mhi_cntrl,
			    const struct mhi_buf *mhi_buf)
{
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	void __iomem *base = mhi_cntrl->bhi;
	u32 tx_status, session_id;
	u32 last_status = 0;
	unsigned long deadline;
	unsigned int poll_ms = 0;
	int ret;

	read_lock_bh(pm_lock);
	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		read_unlock_bh(pm_lock);
		goto invalid_pm_state;
	}

	session_id = MHI_RANDOM_U32_NONZERO(BHI_TXDB_SEQNUM_BMSK);
	dev_dbg(dev, "Starting image download via BHI. Session ID: %u\n",
		session_id);
	dev_info(dev, "BHI transfer start: session=%u dma=%pad size=%zu pm_state=%lx\n",
		 session_id, &mhi_buf->dma_addr, mhi_buf->len,
		 mhi_cntrl->pm_state);
	mhi_write_reg(mhi_cntrl, base, BHI_STATUS, 0);
	mhi_write_reg(mhi_cntrl, base, BHI_IMGADDR_HIGH, upper_32_bits(mhi_buf->dma_addr));
	mhi_write_reg(mhi_cntrl, base, BHI_IMGADDR_LOW, lower_32_bits(mhi_buf->dma_addr));
	mhi_write_reg(mhi_cntrl, base, BHI_IMGSIZE, mhi_buf->len);
	mhi_write_reg(mhi_cntrl, base, BHI_IMGTXDB, session_id);
	read_unlock_bh(pm_lock);

	/* Poll BHI as a diagnostic for missing completion interrupts. */
	tx_status = 0;
	ret = 1;
	deadline = jiffies + msecs_to_jiffies(mhi_cntrl->timeout_ms);
	while (true) {
		if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state))
			break;

		mhi_read_reg_field(mhi_cntrl, base, BHI_STATUS,
				   BHI_STATUS_MASK, &tx_status);
		if (tx_status)
			break;

		if (time_after_eq(jiffies, deadline)) {
			ret = 0;
			break;
		}

		msleep(100);
		poll_ms += 100;

		if (tx_status != last_status) {
			u32 exec_env = U32_MAX, errcode = U32_MAX, dbg1 = U32_MAX;

			mhi_read_reg(mhi_cntrl, base, BHI_EXECENV, &exec_env);
			mhi_read_reg(mhi_cntrl, base, BHI_ERRCODE, &errcode);
			mhi_read_reg(mhi_cntrl, base, BHI_ERRDBG1, &dbg1);
			dev_info(dev, "BHI poll: elapsed=%u status=%x exec_env=%x errcode=%x dbg1=%x pm_state=%lx\n",
				 poll_ms, tx_status, exec_env, errcode, dbg1,
				 mhi_cntrl->pm_state);
			last_status = tx_status;
		}

		if (!(poll_ms % 5000)) {
			u32 exec_env = U32_MAX, errcode = U32_MAX, dbg1 = U32_MAX;

			mhi_read_reg(mhi_cntrl, base, BHI_EXECENV, &exec_env);
			mhi_read_reg(mhi_cntrl, base, BHI_ERRCODE, &errcode);
			mhi_read_reg(mhi_cntrl, base, BHI_ERRDBG1, &dbg1);
			dev_info(dev, "BHI poll heartbeat: elapsed=%u status=%x exec_env=%x errcode=%x dbg1=%x pm_state=%lx\n",
				 poll_ms, tx_status, exec_env, errcode, dbg1,
				 mhi_cntrl->pm_state);
		}
	}
	dev_info(dev, "BHI transfer result: wait=%d session=%u status=%x pm_state=%lx\n",
		 ret, session_id, tx_status, mhi_cntrl->pm_state);
	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state))
		goto invalid_pm_state;

	if (tx_status == BHI_STATUS_ERROR) {
		dev_err(dev, "Image transfer failed\n");
		mhi_fw_load_error_dump(mhi_cntrl);
		goto invalid_pm_state;
	}

	return (!ret) ? -ETIMEDOUT : 0;

invalid_pm_state:
	{
		u32 exec_env = U32_MAX, errcode = U32_MAX, dbg1 = U32_MAX;
		u32 mhi_status = U32_MAX;

		mhi_read_reg(mhi_cntrl, base, BHI_EXECENV, &exec_env);
		mhi_read_reg(mhi_cntrl, base, BHI_ERRCODE, &errcode);
		mhi_read_reg(mhi_cntrl, base, BHI_ERRDBG1, &dbg1);
		mhi_read_reg(mhi_cntrl, mhi_cntrl->regs, MHISTATUS, &mhi_status);
		dev_err(dev, "BHI invalid state: session=%u status=%x pm_state=%x exec_env=%x errcode=%x dbg1=%x mhi_status=%x\n",
			session_id, tx_status, mhi_cntrl->pm_state, exec_env,
			errcode, dbg1, mhi_status);
	}

	return -EIO;
}

static void mhi_free_bhi_buffer(struct mhi_controller *mhi_cntrl,
				struct image_info *image_info)
{
	struct mhi_buf *mhi_buf = image_info->mhi_buf;

	if (mhi_buf->dma_addr == CAIHONG_WCN_BHI_IOVA) {
		struct iommu_domain *domain;

		domain = iommu_get_domain_for_dev(mhi_cntrl->cntrl_dev);
		if (domain)
			iommu_unmap(domain, CAIHONG_WCN_BHI_IOVA,
				    mhi_buf->len);
		if (mhi_buf->orig_dma_addr)
			dma_free_coherent(mhi_cntrl->cntrl_dev, mhi_buf->len,
					  mhi_buf->buf,
					  mhi_buf->orig_dma_addr);
		else
			free_contig_range(PHYS_PFN(CAIHONG_WCN_BHI_TEST_PHYS),
					  mhi_buf->len >> PAGE_SHIFT);
		kfree(image_info);
		return;
	}

	dma_free_coherent(mhi_cntrl->cntrl_dev, mhi_buf->len, mhi_buf->buf, mhi_buf->dma_addr);
	kfree(image_info);
}

void mhi_free_bhie_table(struct mhi_controller *mhi_cntrl,
			 struct image_info *image_info)
{
	int i;
	struct mhi_buf *mhi_buf = image_info->mhi_buf;

	for (i = 0; i < image_info->entries; i++, mhi_buf++)
		dma_free_coherent(mhi_cntrl->cntrl_dev, mhi_buf->len,
				  mhi_buf->buf, mhi_buf->dma_addr);

	kfree(image_info);
}

static int mhi_alloc_bhi_buffer(struct mhi_controller *mhi_cntrl,
				struct image_info **image_info,
				size_t alloc_size)
{
	struct image_info *img_info;
	struct mhi_buf *mhi_buf;

	img_info = kzalloc_flex(*img_info, mhi_buf, 1);
	if (!img_info)
		return -ENOMEM;

	/* Allocate and populate vector table */
	mhi_buf = img_info->mhi_buf;

	mhi_buf->len = alloc_size;
	mhi_buf->orig_dma_addr = 0;
	if (alloc_size == SZ_512K) {
		int ret = alloc_contig_range(PHYS_PFN(CAIHONG_WCN_BHI_TEST_PHYS),
					     PHYS_PFN(CAIHONG_WCN_BHI_TEST_PHYS +
						      alloc_size),
					     ACR_FLAGS_NONE, GFP_KERNEL);
		if (!ret)
			mhi_buf->buf = page_address(pfn_to_page(PHYS_PFN(CAIHONG_WCN_BHI_TEST_PHYS)));
		else
			dev_info(&mhi_cntrl->mhi_dev->dev,
				 "low BHI pool unavailable: %d\n", ret);
	}

	if (!mhi_buf->buf)
		mhi_buf->buf = dma_alloc_coherent(mhi_cntrl->cntrl_dev, mhi_buf->len,
						  &mhi_buf->dma_addr, GFP_KERNEL);
	if (!mhi_buf->buf)
		goto error_alloc_segment;

	if (alloc_size == SZ_512K) {
		struct iommu_domain *domain;
		phys_addr_t phys;
		phys_addr_t mapped;
		int ret;

		domain = iommu_get_domain_for_dev(mhi_cntrl->cntrl_dev);
		if (!domain)
			goto error_alloc_segment;

		if (is_vmalloc_addr(mhi_buf->buf))
			phys = page_to_phys(vmalloc_to_page(mhi_buf->buf));
		else
			phys = virt_to_phys(mhi_buf->buf);
		mhi_buf->orig_dma_addr = mhi_buf->dma_addr;
		mapped = iommu_iova_to_phys(domain, CAIHONG_WCN_BHI_IOVA);
		dev_info(&mhi_cntrl->mhi_dev->dev,
			 "fixed BHI IOVA mapping probe: iova=%pad phys=%pa mapped=%pa orig_dma=%pad\n",
			 &(dma_addr_t){ CAIHONG_WCN_BHI_IOVA }, &phys, &mapped,
			 &mhi_buf->orig_dma_addr);

		if (!mapped) {
			ret = iommu_map(domain, CAIHONG_WCN_BHI_IOVA, phys,
					alloc_size,
					IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE,
					GFP_KERNEL);
			if (ret) {
				dev_info(&mhi_cntrl->mhi_dev->dev,
					 "fixed BHI IOVA map failed: %d\n", ret);
				goto error_alloc_segment;
			}
			mhi_buf->dma_addr = CAIHONG_WCN_BHI_IOVA;
		} else if (mapped == phys) {
			mhi_buf->dma_addr = CAIHONG_WCN_BHI_IOVA;
		} else {
			dev_info(&mhi_cntrl->mhi_dev->dev,
				 "fixed BHI IOVA mismatch\n");
			goto error_alloc_segment;
		}
	}

	img_info->bhi_vec = NULL;
	img_info->entries = 1;
	*image_info = img_info;

	return 0;

error_alloc_segment:
	kfree(img_info);

	return -ENOMEM;
}

int mhi_alloc_bhie_table(struct mhi_controller *mhi_cntrl,
			 struct image_info **image_info,
			 size_t alloc_size)
{
	size_t seg_size = mhi_cntrl->seg_len;
	int segments = DIV_ROUND_UP(alloc_size, seg_size) + 1;
	int i;
	struct image_info *img_info;
	struct mhi_buf *mhi_buf;

	img_info = kzalloc_flex(*img_info, mhi_buf, segments);
	if (!img_info)
		return -ENOMEM;

	img_info->entries = segments;

	/* Allocate and populate vector table */
	mhi_buf = img_info->mhi_buf;
	for (i = 0; i < segments; i++, mhi_buf++) {
		size_t vec_size = seg_size;

		/* Vector table is the last entry */
		if (i == segments - 1)
			vec_size = sizeof(struct bhi_vec_entry) * i;

		mhi_buf->len = vec_size;
		mhi_buf->buf = dma_alloc_coherent(mhi_cntrl->cntrl_dev,
						  vec_size, &mhi_buf->dma_addr,
						  GFP_KERNEL);
		if (!mhi_buf->buf)
			goto error_alloc_segment;
	}

	img_info->bhi_vec = img_info->mhi_buf[segments - 1].buf;
	*image_info = img_info;

	return 0;

error_alloc_segment:
	for (--i, --mhi_buf; i >= 0; i--, mhi_buf--)
		dma_free_coherent(mhi_cntrl->cntrl_dev, mhi_buf->len,
				  mhi_buf->buf, mhi_buf->dma_addr);
	kfree(img_info);

	return -ENOMEM;
}

static void mhi_firmware_copy_bhie(struct mhi_controller *mhi_cntrl,
				   const u8 *buf, size_t remainder,
				   struct image_info *img_info)
{
	size_t to_cpy;
	struct mhi_buf *mhi_buf = img_info->mhi_buf;
	struct bhi_vec_entry *bhi_vec = img_info->bhi_vec;

	while (remainder) {
		to_cpy = min(remainder, mhi_buf->len);
		memcpy(mhi_buf->buf, buf, to_cpy);
		bhi_vec->dma_addr = cpu_to_le64(mhi_buf->dma_addr);
		bhi_vec->size = cpu_to_le64(to_cpy);

		buf += to_cpy;
		remainder -= to_cpy;
		bhi_vec++;
		mhi_buf++;
	}
}

static enum mhi_fw_load_type mhi_fw_load_type_get(const struct mhi_controller *mhi_cntrl)
{
	if (mhi_cntrl->fbc_download) {
		return MHI_FW_LOAD_FBC;
	} else {
		if (mhi_cntrl->seg_len)
			return MHI_FW_LOAD_BHIE;
		else
			return MHI_FW_LOAD_BHI;
	}
}

static int mhi_load_image_bhi(struct mhi_controller *mhi_cntrl, const u8 *fw_data, size_t size)
{
	struct image_info *image;
	int ret;

	ret = mhi_alloc_bhi_buffer(mhi_cntrl, &image, size);
	if (ret)
		return ret;

	/* Load the firmware into BHI vec table */
	memcpy(image->mhi_buf->buf, fw_data, size);
	dma_sync_single_for_device(mhi_cntrl->cntrl_dev,
				   image->mhi_buf->dma_addr, size,
				   DMA_TO_DEVICE);

	ret = mhi_fw_load_bhi(mhi_cntrl, &image->mhi_buf[image->entries - 1]);
	mhi_free_bhi_buffer(mhi_cntrl, image);

	return ret;
}

static int mhi_load_image_bhie(struct mhi_controller *mhi_cntrl, const u8 *fw_data, size_t size)
{
	struct image_info *image;
	int ret;

	ret = mhi_alloc_bhie_table(mhi_cntrl, &image, size);
	if (ret)
		return ret;

	mhi_firmware_copy_bhie(mhi_cntrl, fw_data, size, image);

	ret = mhi_fw_load_bhie(mhi_cntrl, &image->mhi_buf[image->entries - 1]);
	mhi_free_bhie_table(mhi_cntrl, image);

	return ret;
}

void mhi_fw_load_handler(struct mhi_controller *mhi_cntrl)
{
	const struct firmware *firmware = NULL;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	enum mhi_fw_load_type fw_load_type;
	enum mhi_pm_state new_state;
	const char *fw_name;
	const u8 *fw_data;
	size_t size, fw_sz;
	int ret;

	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state)) {
		dev_err(dev, "Device MHI is not in valid state\n");
		return;
	}

	/* save hardware info from BHI */
	ret = mhi_read_reg(mhi_cntrl, mhi_cntrl->bhi, BHI_SERIALNU,
			   &mhi_cntrl->serial_number);
	if (ret)
		dev_err(dev, "Could not capture serial number via BHI\n");

	/* wait for ready on pass through or any other execution environment */
	if (!MHI_FW_LOAD_CAPABLE(mhi_cntrl->ee))
		goto fw_load_ready_state;

	fw_name = (mhi_cntrl->ee == MHI_EE_EDL) ?
		mhi_cntrl->edl_image : mhi_cntrl->fw_image;

	/* check if the driver has already provided the firmware data */
	if (!fw_name && mhi_cntrl->fbc_download &&
	    mhi_cntrl->fw_data && mhi_cntrl->fw_sz) {
		if (!mhi_cntrl->sbl_size) {
			dev_err(dev, "fw_data provided but no sbl_size\n");
			goto error_fw_load;
		}

		size = mhi_cntrl->sbl_size;
		fw_data = mhi_cntrl->fw_data;
		fw_sz = mhi_cntrl->fw_sz;
		goto skip_req_fw;
	}

	if (!fw_name || (mhi_cntrl->fbc_download && (!mhi_cntrl->sbl_size ||
						     !mhi_cntrl->seg_len))) {
		dev_err(dev,
			"No firmware image defined or !sbl_size || !seg_len\n");
		goto error_fw_load;
	}

	ret = request_firmware(&firmware, fw_name, dev);
	if (ret) {
		dev_err(dev, "Error loading firmware: %d\n", ret);
		goto error_fw_load;
	}

	size = (mhi_cntrl->fbc_download) ? mhi_cntrl->sbl_size : firmware->size;

	/* SBL size provided is maximum size, not necessarily the image size */
	if (size > firmware->size)
		size = firmware->size;

	fw_data = firmware->data;
	fw_sz = firmware->size;

skip_req_fw:
	fw_load_type = mhi_fw_load_type_get(mhi_cntrl);
	if (fw_load_type == MHI_FW_LOAD_BHIE)
		ret = mhi_load_image_bhie(mhi_cntrl, fw_data, size);
	else
		ret = mhi_load_image_bhi(mhi_cntrl, fw_data, size);

	/* Error or in EDL mode, we're done */
	if (ret) {
		dev_err(dev, "MHI did not load image over BHI%s, ret: %d\n",
			fw_load_type == MHI_FW_LOAD_BHIE ? "e" : "",
			ret);
		dev_err(dev, "BHI image: fw=%s size=%zu fw_size=%zu load_type=%d ee=%d fbc=%d sbl=%zu seg=%zu\n",
			fw_name ? fw_name : "(inline)", size, fw_sz,
			fw_load_type, mhi_cntrl->ee, mhi_cntrl->fbc_download,
			mhi_cntrl->sbl_size, mhi_cntrl->seg_len);
		release_firmware(firmware);
		goto error_fw_load;
	}

	/* Wait for ready since EDL image was loaded */
	if (fw_name && fw_name == mhi_cntrl->edl_image) {
		release_firmware(firmware);
		goto fw_load_ready_state;
	}

	write_lock_irq(&mhi_cntrl->pm_lock);
	mhi_cntrl->dev_state = MHI_STATE_RESET;
	write_unlock_irq(&mhi_cntrl->pm_lock);

	/*
	 * If we're doing fbc, populate vector tables while
	 * device transitioning into MHI READY state
	 */
	if (fw_load_type == MHI_FW_LOAD_FBC) {
		/*
		 * Some FW combine two separate ELF images (SBL + WLAN FW) in a single
		 * file. Hence, check for the existence of the second ELF header after
		 * SBL. If present, load the second image separately.
		 */
		if (!memcmp(fw_data + mhi_cntrl->sbl_size, ELFMAG, SELFMAG)) {
			fw_data += mhi_cntrl->sbl_size;
			fw_sz -= mhi_cntrl->sbl_size;
		}

		ret = mhi_alloc_bhie_table(mhi_cntrl, &mhi_cntrl->fbc_image, fw_sz);
		if (ret) {
			release_firmware(firmware);
			goto error_fw_load;
		}

		/* Load the firmware into BHIE vec table */
		mhi_firmware_copy_bhie(mhi_cntrl, fw_data, fw_sz, mhi_cntrl->fbc_image);
	}

	release_firmware(firmware);

fw_load_ready_state:
	/* Transitioning into MHI RESET->READY state */
	ret = mhi_ready_state_transition(mhi_cntrl);
	if (ret) {
		dev_err(dev, "MHI did not enter READY state\n");
		goto error_ready_state;
	}

	dev_info(dev, "Wait for device to enter SBL or Mission mode\n");
	return;

error_ready_state:
	if (mhi_cntrl->fbc_image) {
		mhi_free_bhie_table(mhi_cntrl, mhi_cntrl->fbc_image);
		mhi_cntrl->fbc_image = NULL;
	}

error_fw_load:
	write_lock_irq(&mhi_cntrl->pm_lock);
	new_state = mhi_tryset_pm_state(mhi_cntrl, MHI_PM_FW_DL_ERR);
	write_unlock_irq(&mhi_cntrl->pm_lock);
	if (new_state == MHI_PM_FW_DL_ERR)
		wake_up_all(&mhi_cntrl->state_event);
}

int mhi_download_amss_image(struct mhi_controller *mhi_cntrl)
{
	struct image_info *image_info = mhi_cntrl->fbc_image;
	struct device *dev = &mhi_cntrl->mhi_dev->dev;
	enum mhi_pm_state new_state;
	int ret;

	if (!image_info)
		return -EIO;

	ret = mhi_fw_load_bhie(mhi_cntrl,
			       /* Vector table is the last entry */
			       &image_info->mhi_buf[image_info->entries - 1]);
	if (ret) {
		dev_err(dev, "MHI did not load AMSS, ret:%d\n", ret);
		write_lock_irq(&mhi_cntrl->pm_lock);
		new_state = mhi_tryset_pm_state(mhi_cntrl, MHI_PM_FW_DL_ERR);
		write_unlock_irq(&mhi_cntrl->pm_lock);
		if (new_state == MHI_PM_FW_DL_ERR)
			wake_up_all(&mhi_cntrl->state_event);
	}

	return ret;
}
