// SPDX-License-Identifier: GPL-2.0
/* Marvell Octeon EP (EndPoint) Ethernet Driver
 *
 * Copyright (C) 2020 Marvell.
 *
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/etherdevice.h>

#include "octep_ctrl_mbox.h"
#include "octep_config.h"
#include "octep_main.h"

/* Timeout in msecs for message response */
#define OCTEP_CTRL_MBOX_MSG_TIMEOUT_MS			100
/* Time in msecs to wait for message response */
#define OCTEP_CTRL_MBOX_MSG_WAIT_MS			10

/* Size of mbox info in bytes */
#define OCTEP_CTRL_MBOX_INFO_SZ				256
/* Size of mbox host to fw queue info in bytes */
#define OCTEP_CTRL_MBOX_H2FQ_INFO_SZ			16
/* Size of mbox fw to host queue info in bytes */
#define OCTEP_CTRL_MBOX_F2HQ_INFO_SZ			16

#define OCTEP_CTRL_MBOX_TOTAL_INFO_SZ	(OCTEP_CTRL_MBOX_INFO_SZ + \
					 OCTEP_CTRL_MBOX_H2FQ_INFO_SZ + \
					 OCTEP_CTRL_MBOX_F2HQ_INFO_SZ)

#define OCTEP_CTRL_MBOX_INFO_MAGIC_NUM(m)	(m)
#define OCTEP_CTRL_MBOX_INFO_BARMEM_SZ(m)	((m) + 8)
#define OCTEP_CTRL_MBOX_INFO_HOST_VERSION(m)   ((m) + 16)
#define OCTEP_CTRL_MBOX_INFO_HOST_STATUS(m)	((m) + 24)
#define OCTEP_CTRL_MBOX_INFO_FW_VERSION(m)     ((m) + 136)
#define OCTEP_CTRL_MBOX_INFO_FW_STATUS(m)	((m) + 144)

#define OCTEP_CTRL_MBOX_H2FQ_INFO(m)	((m) + OCTEP_CTRL_MBOX_INFO_SZ)
#define OCTEP_CTRL_MBOX_H2FQ_PROD(m)	(OCTEP_CTRL_MBOX_H2FQ_INFO(m))
#define OCTEP_CTRL_MBOX_H2FQ_CONS(m)	((OCTEP_CTRL_MBOX_H2FQ_INFO(m)) + 4)
#define OCTEP_CTRL_MBOX_H2FQ_SZ(m)	((OCTEP_CTRL_MBOX_H2FQ_INFO(m)) + 8)

#define OCTEP_CTRL_MBOX_F2HQ_INFO(m)	((m) + \
					 OCTEP_CTRL_MBOX_INFO_SZ + \
					 OCTEP_CTRL_MBOX_H2FQ_INFO_SZ)
#define OCTEP_CTRL_MBOX_F2HQ_PROD(m)	(OCTEP_CTRL_MBOX_F2HQ_INFO(m))
#define OCTEP_CTRL_MBOX_F2HQ_CONS(m)	((OCTEP_CTRL_MBOX_F2HQ_INFO(m)) + 4)
#define OCTEP_CTRL_MBOX_F2HQ_SZ(m)	((OCTEP_CTRL_MBOX_F2HQ_INFO(m)) + 8)

static const u32 mbox_hdr_sz = sizeof(union octep_ctrl_mbox_msg_hdr);

static u32 octep_ctrl_mbox_circq_inc(u32 index, u32 inc, u32 sz)
{
	return (index + inc) % sz;
}

static u32 octep_ctrl_mbox_circq_space(u32 pi, u32 ci, u32 sz)
{
	return sz - (abs(pi - ci) % sz);
}

static u32 octep_ctrl_mbox_circq_depth(u32 pi, u32 ci, u32 sz)
{
	return (abs(pi - ci) % sz);
}

int octep_ctrl_mbox_init(struct octep_ctrl_mbox *mbox)
{
	u64 magic_num, status, fw_versions;

	if (!mbox)
		return -EINVAL;

	if (!mbox->barmem) {
		pr_info("octep_ctrl_mbox : Invalid barmem %p\n", mbox->barmem);
		return -EINVAL;
	}

	magic_num = pete_readq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:86", OCTEP_CTRL_MBOX_INFO_MAGIC_NUM(mbox->barmem));
	if (magic_num != OCTEP_CTRL_MBOX_MAGIC_NUMBER) {
		pr_info("octep_ctrl_mbox : Invalid magic number %llx\n", magic_num);
		return -EINVAL;
	}

	status = pete_readq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:92", OCTEP_CTRL_MBOX_INFO_FW_STATUS(mbox->barmem));
	if (status != OCTEP_CTRL_MBOX_STATUS_READY) {
		pr_info("octep_ctrl_mbox : Firmware is not ready.\n");
		return -EINVAL;
	}

	fw_versions = pete_readq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:98", OCTEP_CTRL_MBOX_INFO_FW_VERSION(mbox->barmem));
	mbox->min_fw_version = ((fw_versions & 0xffffffff00000000ull) >> 32);
	mbox->max_fw_version = (fw_versions & 0xffffffff);
	mbox->barmem_sz = pete_readl("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:101", OCTEP_CTRL_MBOX_INFO_BARMEM_SZ(mbox->barmem));

	pete_writeq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:103", OCTEP_CTRL_MBOX_STATUS_INIT,
	       OCTEP_CTRL_MBOX_INFO_HOST_STATUS(mbox->barmem));

	mutex_init(&mbox->h2fq_lock);
	mutex_init(&mbox->f2hq_lock);

	mbox->h2fq.sz = pete_readl("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:109", OCTEP_CTRL_MBOX_H2FQ_SZ(mbox->barmem));
	mbox->h2fq.hw_prod = OCTEP_CTRL_MBOX_H2FQ_PROD(mbox->barmem);
	mbox->h2fq.hw_cons = OCTEP_CTRL_MBOX_H2FQ_CONS(mbox->barmem);
	mbox->h2fq.hw_q = mbox->barmem + OCTEP_CTRL_MBOX_TOTAL_INFO_SZ;

	mbox->f2hq.sz = pete_readl("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:114", OCTEP_CTRL_MBOX_F2HQ_SZ(mbox->barmem));
	mbox->f2hq.hw_prod = OCTEP_CTRL_MBOX_F2HQ_PROD(mbox->barmem);
	mbox->f2hq.hw_cons = OCTEP_CTRL_MBOX_F2HQ_CONS(mbox->barmem);
	mbox->f2hq.hw_q = mbox->barmem +
			  OCTEP_CTRL_MBOX_TOTAL_INFO_SZ +
			  mbox->h2fq.sz;

	pete_writeq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:121", mbox->version, OCTEP_CTRL_MBOX_INFO_HOST_VERSION(mbox->barmem));
	/* ensure ready state is seen after everything is initialized */
	wmb();
	pete_writeq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:124", OCTEP_CTRL_MBOX_STATUS_READY,
	       OCTEP_CTRL_MBOX_INFO_HOST_STATUS(mbox->barmem));

	pr_info("Octep ctrl mbox : Init successful.\n");

	return 0;
}

static void
octep_write_mbox_data(struct octep_ctrl_mbox_q *q, u32 *pi, u32 ci, void *buf, u32 w_sz)
{
	u8 __iomem *qbuf;
	u32 cp_sz;

	/* Assumption: Caller has ensured enough write space */
	qbuf = (q->hw_q + *pi);
	if (*pi < ci) {
		/* copy entire w_sz */
		memcpy_toio(qbuf, buf, w_sz);
		*pi = octep_ctrl_mbox_circq_inc(*pi, w_sz, q->sz);
	} else {
		/* copy up to end of queue */
		cp_sz = min((q->sz - *pi), w_sz);
		memcpy_toio(qbuf, buf, cp_sz);
		w_sz -= cp_sz;
		*pi = octep_ctrl_mbox_circq_inc(*pi, cp_sz, q->sz);
		if (w_sz) {
			/* roll over and copy remaining w_sz */
			buf += cp_sz;
			qbuf = (q->hw_q + *pi);
			memcpy_toio(qbuf, buf, w_sz);
			*pi = octep_ctrl_mbox_circq_inc(*pi, w_sz, q->sz);
		}
	}
}

int octep_ctrl_mbox_send(struct octep_ctrl_mbox *mbox, struct octep_ctrl_mbox_msg *msg)
{
	struct octep_ctrl_mbox_msg_buf *sg;
	struct octep_ctrl_mbox_q *q;
	u32 pi, ci, buf_sz, w_sz;
	int s;

	if (!mbox || !msg)
		return -EINVAL;

	if (pete_readq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:170", OCTEP_CTRL_MBOX_INFO_FW_STATUS(mbox->barmem)) != OCTEP_CTRL_MBOX_STATUS_READY)
		return -EIO;

	mutex_lock(&mbox->h2fq_lock);
	q = &mbox->h2fq;
	pi = pete_readl("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:175", q->hw_prod);
	ci = pete_readl("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:176", q->hw_cons);

	if (octep_ctrl_mbox_circq_space(pi, ci, q->sz) < (msg->hdr.s.sz + mbox_hdr_sz)) {
		mutex_unlock(&mbox->h2fq_lock);
		return -EAGAIN;
	}

	octep_write_mbox_data(q, &pi, ci, (void *)&msg->hdr, mbox_hdr_sz);
	buf_sz = msg->hdr.s.sz;
	for (s = 0; ((s < msg->sg_num) && (buf_sz > 0)); s++) {
		sg = &msg->sg_list[s];
		w_sz = (sg->sz <= buf_sz) ? sg->sz : buf_sz;
		octep_write_mbox_data(q, &pi, ci, sg->msg, w_sz);
		buf_sz -= w_sz;
	}
	pete_writel("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:191", pi, q->hw_prod);
	mutex_unlock(&mbox->h2fq_lock);

	return 0;
}

static void
octep_read_mbox_data(struct octep_ctrl_mbox_q *q, u32 pi, u32 *ci, void *buf, u32 r_sz)
{
	u8 __iomem *qbuf;
	u32 cp_sz;

	/* Assumption: Caller has ensured enough read space */
	qbuf = (q->hw_q + *ci);
	if (*ci < pi) {
		/* copy entire r_sz */
		memcpy_fromio(buf, qbuf, r_sz);
		*ci = octep_ctrl_mbox_circq_inc(*ci, r_sz, q->sz);
	} else {
		/* copy up to end of queue */
		cp_sz = min((q->sz - *ci), r_sz);
		memcpy_fromio(buf, qbuf, cp_sz);
		r_sz -= cp_sz;
		*ci = octep_ctrl_mbox_circq_inc(*ci, cp_sz, q->sz);
		if (r_sz) {
			/* roll over and copy remaining r_sz */
			buf += cp_sz;
			qbuf = (q->hw_q + *ci);
			memcpy_fromio(buf, qbuf, r_sz);
			*ci = octep_ctrl_mbox_circq_inc(*ci, r_sz, q->sz);
		}
	}
}

int octep_ctrl_mbox_recv(struct octep_ctrl_mbox *mbox, struct octep_ctrl_mbox_msg *msg)
{
	struct octep_ctrl_mbox_msg_buf *sg;
	u32 pi, ci, r_sz, buf_sz, q_depth;
	struct octep_ctrl_mbox_q *q;
	int s;

	if (pete_readq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:232", OCTEP_CTRL_MBOX_INFO_FW_STATUS(mbox->barmem)) != OCTEP_CTRL_MBOX_STATUS_READY)
		return -EIO;

	mutex_lock(&mbox->f2hq_lock);
	q = &mbox->f2hq;
	pi = pete_readl("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:237", q->hw_prod);
	ci = pete_readl("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:238", q->hw_cons);

	q_depth = octep_ctrl_mbox_circq_depth(pi, ci, q->sz);
	if (q_depth < mbox_hdr_sz) {
		mutex_unlock(&mbox->f2hq_lock);
		return -EAGAIN;
	}

	octep_read_mbox_data(q, pi, &ci, (void *)&msg->hdr, mbox_hdr_sz);
	buf_sz = msg->hdr.s.sz;
	for (s = 0; ((s < msg->sg_num) && (buf_sz > 0)); s++) {
		sg = &msg->sg_list[s];
		r_sz = (sg->sz <= buf_sz) ? sg->sz : buf_sz;
		octep_read_mbox_data(q, pi, &ci, sg->msg, r_sz);
		buf_sz -= r_sz;
	}
	pete_writel("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:254", ci, q->hw_cons);
	mutex_unlock(&mbox->f2hq_lock);

	return 0;
}

int octep_ctrl_mbox_uninit(struct octep_ctrl_mbox *mbox)
{
	if (!mbox)
		return -EINVAL;
	if (!mbox->barmem)
		return -EINVAL;

	pete_writeq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:267", 0, OCTEP_CTRL_MBOX_INFO_HOST_VERSION(mbox->barmem));
	pete_writeq("drivers/net/ethernet/marvell/octeon_ep/octep_ctrl_mbox.c:268", OCTEP_CTRL_MBOX_STATUS_INVALID,
	       OCTEP_CTRL_MBOX_INFO_HOST_STATUS(mbox->barmem));
	/* ensure uninit state is written before uninitialization */
	wmb();

	mutex_destroy(&mbox->h2fq_lock);
	mutex_destroy(&mbox->f2hq_lock);

	pr_info("Octep ctrl mbox : Uninit successful.\n");

	return 0;
}
