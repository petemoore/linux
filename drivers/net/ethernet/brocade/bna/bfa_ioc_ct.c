// SPDX-License-Identifier: GPL-2.0-only
/*
 * Linux network driver for QLogic BR-series Converged Network Adapter.
 */
/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014-2015 QLogic Corporation
 * All rights reserved
 * www.qlogic.com
 */

#include "bfa_ioc.h"
#include "cna.h"
#include "bfi.h"
#include "bfi_reg.h"
#include "bfa_defs.h"

#define bfa_ioc_ct_sync_pos(__ioc)	BIT(bfa_ioc_pcifn(__ioc))
#define BFA_IOC_SYNC_REQD_SH		16
#define bfa_ioc_ct_get_sync_ackd(__val) (__val & 0x0000ffff)
#define bfa_ioc_ct_clear_sync_ackd(__val) (__val & 0xffff0000)
#define bfa_ioc_ct_get_sync_reqd(__val) (__val >> BFA_IOC_SYNC_REQD_SH)
#define bfa_ioc_ct_sync_reqd_pos(__ioc) \
		(bfa_ioc_ct_sync_pos(__ioc) << BFA_IOC_SYNC_REQD_SH)

/*
 * forward declarations
 */
static bool bfa_ioc_ct_firmware_lock(struct bfa_ioc *ioc);
static void bfa_ioc_ct_firmware_unlock(struct bfa_ioc *ioc);
static void bfa_ioc_ct_reg_init(struct bfa_ioc *ioc);
static void bfa_ioc_ct2_reg_init(struct bfa_ioc *ioc);
static void bfa_ioc_ct_map_port(struct bfa_ioc *ioc);
static void bfa_ioc_ct2_map_port(struct bfa_ioc *ioc);
static void bfa_ioc_ct_isr_mode_set(struct bfa_ioc *ioc, bool msix);
static void bfa_ioc_ct_notify_fail(struct bfa_ioc *ioc);
static void bfa_ioc_ct_ownership_reset(struct bfa_ioc *ioc);
static bool bfa_ioc_ct_sync_start(struct bfa_ioc *ioc);
static void bfa_ioc_ct_sync_join(struct bfa_ioc *ioc);
static void bfa_ioc_ct_sync_leave(struct bfa_ioc *ioc);
static void bfa_ioc_ct_sync_ack(struct bfa_ioc *ioc);
static bool bfa_ioc_ct_sync_complete(struct bfa_ioc *ioc);
static void bfa_ioc_ct_set_cur_ioc_fwstate(
			struct bfa_ioc *ioc, enum bfi_ioc_state fwstate);
static enum bfi_ioc_state bfa_ioc_ct_get_cur_ioc_fwstate(struct bfa_ioc *ioc);
static void bfa_ioc_ct_set_alt_ioc_fwstate(
			struct bfa_ioc *ioc, enum bfi_ioc_state fwstate);
static enum bfi_ioc_state bfa_ioc_ct_get_alt_ioc_fwstate(struct bfa_ioc *ioc);
static enum bfa_status bfa_ioc_ct_pll_init(void __iomem *rb,
				enum bfi_asic_mode asic_mode);
static enum bfa_status bfa_ioc_ct2_pll_init(void __iomem *rb,
				enum bfi_asic_mode asic_mode);
static bool bfa_ioc_ct2_lpu_read_stat(struct bfa_ioc *ioc);

static const struct bfa_ioc_hwif nw_hwif_ct = {
	.ioc_pll_init	     = bfa_ioc_ct_pll_init,
	.ioc_firmware_lock   = bfa_ioc_ct_firmware_lock,
	.ioc_firmware_unlock = bfa_ioc_ct_firmware_unlock,
	.ioc_reg_init	     = bfa_ioc_ct_reg_init,
	.ioc_map_port	     = bfa_ioc_ct_map_port,
	.ioc_isr_mode_set    = bfa_ioc_ct_isr_mode_set,
	.ioc_notify_fail     = bfa_ioc_ct_notify_fail,
	.ioc_ownership_reset = bfa_ioc_ct_ownership_reset,
	.ioc_sync_start      = bfa_ioc_ct_sync_start,
	.ioc_sync_join       = bfa_ioc_ct_sync_join,
	.ioc_sync_leave	     = bfa_ioc_ct_sync_leave,
	.ioc_sync_ack	     = bfa_ioc_ct_sync_ack,
	.ioc_sync_complete   = bfa_ioc_ct_sync_complete,
	.ioc_set_fwstate     = bfa_ioc_ct_set_cur_ioc_fwstate,
	.ioc_get_fwstate     = bfa_ioc_ct_get_cur_ioc_fwstate,
	.ioc_set_alt_fwstate     = bfa_ioc_ct_set_alt_ioc_fwstate,
	.ioc_get_alt_fwstate     = bfa_ioc_ct_get_alt_ioc_fwstate,
};

static const struct bfa_ioc_hwif nw_hwif_ct2 = {
	.ioc_pll_init	     = bfa_ioc_ct2_pll_init,
	.ioc_firmware_lock   = bfa_ioc_ct_firmware_lock,
	.ioc_firmware_unlock = bfa_ioc_ct_firmware_unlock,
	.ioc_reg_init	     = bfa_ioc_ct2_reg_init,
	.ioc_map_port	     = bfa_ioc_ct2_map_port,
	.ioc_lpu_read_stat   = bfa_ioc_ct2_lpu_read_stat,
	.ioc_isr_mode_set    = NULL,
	.ioc_notify_fail     = bfa_ioc_ct_notify_fail,
	.ioc_ownership_reset = bfa_ioc_ct_ownership_reset,
	.ioc_sync_start      = bfa_ioc_ct_sync_start,
	.ioc_sync_join       = bfa_ioc_ct_sync_join,
	.ioc_sync_leave	     = bfa_ioc_ct_sync_leave,
	.ioc_sync_ack	     = bfa_ioc_ct_sync_ack,
	.ioc_sync_complete   = bfa_ioc_ct_sync_complete,
	.ioc_set_fwstate     = bfa_ioc_ct_set_cur_ioc_fwstate,
	.ioc_get_fwstate     = bfa_ioc_ct_get_cur_ioc_fwstate,
	.ioc_set_alt_fwstate     = bfa_ioc_ct_set_alt_ioc_fwstate,
	.ioc_get_alt_fwstate     = bfa_ioc_ct_get_alt_ioc_fwstate,
};

/* Called from bfa_ioc_attach() to map asic specific calls. */
void
bfa_nw_ioc_set_ct_hwif(struct bfa_ioc *ioc)
{
	ioc->ioc_hwif = &nw_hwif_ct;
}

void
bfa_nw_ioc_set_ct2_hwif(struct bfa_ioc *ioc)
{
	ioc->ioc_hwif = &nw_hwif_ct2;
}

/* Return true if firmware of current driver matches the running firmware. */
static bool
bfa_ioc_ct_firmware_lock(struct bfa_ioc *ioc)
{
	enum bfi_ioc_state ioc_fwstate;
	u32 usecnt;
	struct bfi_ioc_image_hdr fwhdr;

	/**
	 * If bios boot (flash based) -- do not increment usage count
	 */
	if (bfa_cb_image_get_size(bfa_ioc_asic_gen(ioc)) <
						BFA_IOC_FWIMG_MINSZ)
		return true;

	bfa_nw_ioc_sem_get(ioc->ioc_regs.ioc_usage_sem_reg);
	usecnt = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:125", ioc->ioc_regs.ioc_usage_reg);

	/**
	 * If usage count is 0, always return TRUE.
	 */
	if (usecnt == 0) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:131", 1, ioc->ioc_regs.ioc_usage_reg);
		bfa_nw_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:133", 0, ioc->ioc_regs.ioc_fail_sync);
		return true;
	}

	ioc_fwstate = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:137", ioc->ioc_regs.ioc_fwstate);

	/**
	 * Use count cannot be non-zero and chip in uninitialized state.
	 */
	BUG_ON(!(ioc_fwstate != BFI_IOC_UNINIT));

	/**
	 * Check if another driver with a different firmware is active
	 */
	bfa_nw_ioc_fwver_get(ioc, &fwhdr);
	if (!bfa_nw_ioc_fwver_cmp(ioc, &fwhdr)) {
		bfa_nw_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
		return false;
	}

	/**
	 * Same firmware version. Increment the reference count.
	 */
	usecnt++;
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:157", usecnt, ioc->ioc_regs.ioc_usage_reg);
	bfa_nw_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
	return true;
}

static void
bfa_ioc_ct_firmware_unlock(struct bfa_ioc *ioc)
{
	u32 usecnt;

	/**
	 * If bios boot (flash based) -- do not decrement usage count
	 */
	if (bfa_cb_image_get_size(bfa_ioc_asic_gen(ioc)) <
						BFA_IOC_FWIMG_MINSZ)
		return;

	/**
	 * decrement usage count
	 */
	bfa_nw_ioc_sem_get(ioc->ioc_regs.ioc_usage_sem_reg);
	usecnt = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:178", ioc->ioc_regs.ioc_usage_reg);
	BUG_ON(!(usecnt > 0));

	usecnt--;
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:182", usecnt, ioc->ioc_regs.ioc_usage_reg);

	bfa_nw_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
}

/* Notify other functions on HB failure. */
static void
bfa_ioc_ct_notify_fail(struct bfa_ioc *ioc)
{
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:191", __FW_INIT_HALT_P, ioc->ioc_regs.ll_halt);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:192", __FW_INIT_HALT_P, ioc->ioc_regs.alt_ll_halt);
	/* Wait for halt to take effect */
	pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:194", ioc->ioc_regs.ll_halt);
	pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:195", ioc->ioc_regs.alt_ll_halt);
}

/* Host to LPU mailbox message addresses */
static const struct {
	u32	hfn_mbox;
	u32	lpu_mbox;
	u32	hfn_pgn;
} ct_fnreg[] = {
	{ HOSTFN0_LPU_MBOX0_0, LPU_HOSTFN0_MBOX0_0, HOST_PAGE_NUM_FN0 },
	{ HOSTFN1_LPU_MBOX0_8, LPU_HOSTFN1_MBOX0_8, HOST_PAGE_NUM_FN1 },
	{ HOSTFN2_LPU_MBOX0_0, LPU_HOSTFN2_MBOX0_0, HOST_PAGE_NUM_FN2 },
	{ HOSTFN3_LPU_MBOX0_8, LPU_HOSTFN3_MBOX0_8, HOST_PAGE_NUM_FN3 }
};

/* Host <-> LPU mailbox command/status registers - port 0 */
static const struct {
	u32	hfn;
	u32	lpu;
} ct_p0reg[] = {
	{ HOSTFN0_LPU0_CMD_STAT, LPU0_HOSTFN0_CMD_STAT },
	{ HOSTFN1_LPU0_CMD_STAT, LPU0_HOSTFN1_CMD_STAT },
	{ HOSTFN2_LPU0_CMD_STAT, LPU0_HOSTFN2_CMD_STAT },
	{ HOSTFN3_LPU0_CMD_STAT, LPU0_HOSTFN3_CMD_STAT }
};

/* Host <-> LPU mailbox command/status registers - port 1 */
static const struct {
	u32	hfn;
	u32	lpu;
} ct_p1reg[] = {
	{ HOSTFN0_LPU1_CMD_STAT, LPU1_HOSTFN0_CMD_STAT },
	{ HOSTFN1_LPU1_CMD_STAT, LPU1_HOSTFN1_CMD_STAT },
	{ HOSTFN2_LPU1_CMD_STAT, LPU1_HOSTFN2_CMD_STAT },
	{ HOSTFN3_LPU1_CMD_STAT, LPU1_HOSTFN3_CMD_STAT }
};

static const struct {
	u32	hfn_mbox;
	u32	lpu_mbox;
	u32	hfn_pgn;
	u32	hfn;
	u32	lpu;
	u32	lpu_read;
} ct2_reg[] = {
	{ CT2_HOSTFN_LPU0_MBOX0, CT2_LPU0_HOSTFN_MBOX0, CT2_HOSTFN_PAGE_NUM,
	  CT2_HOSTFN_LPU0_CMD_STAT, CT2_LPU0_HOSTFN_CMD_STAT,
	  CT2_HOSTFN_LPU0_READ_STAT},
	{ CT2_HOSTFN_LPU1_MBOX0, CT2_LPU1_HOSTFN_MBOX0, CT2_HOSTFN_PAGE_NUM,
	  CT2_HOSTFN_LPU1_CMD_STAT, CT2_LPU1_HOSTFN_CMD_STAT,
	  CT2_HOSTFN_LPU1_READ_STAT},
};

static void
bfa_ioc_ct_reg_init(struct bfa_ioc *ioc)
{
	void __iomem *rb;
	int		pcifn = bfa_ioc_pcifn(ioc);

	rb = bfa_ioc_bar0(ioc);

	ioc->ioc_regs.hfn_mbox = rb + ct_fnreg[pcifn].hfn_mbox;
	ioc->ioc_regs.lpu_mbox = rb + ct_fnreg[pcifn].lpu_mbox;
	ioc->ioc_regs.host_page_num_fn = rb + ct_fnreg[pcifn].hfn_pgn;

	if (ioc->port_id == 0) {
		ioc->ioc_regs.heartbeat = rb + BFA_IOC0_HBEAT_REG;
		ioc->ioc_regs.ioc_fwstate = rb + BFA_IOC0_STATE_REG;
		ioc->ioc_regs.alt_ioc_fwstate = rb + BFA_IOC1_STATE_REG;
		ioc->ioc_regs.hfn_mbox_cmd = rb + ct_p0reg[pcifn].hfn;
		ioc->ioc_regs.lpu_mbox_cmd = rb + ct_p0reg[pcifn].lpu;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P0;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P1;
	} else {
		ioc->ioc_regs.heartbeat = rb + BFA_IOC1_HBEAT_REG;
		ioc->ioc_regs.ioc_fwstate = rb + BFA_IOC1_STATE_REG;
		ioc->ioc_regs.alt_ioc_fwstate = rb + BFA_IOC0_STATE_REG;
		ioc->ioc_regs.hfn_mbox_cmd = rb + ct_p1reg[pcifn].hfn;
		ioc->ioc_regs.lpu_mbox_cmd = rb + ct_p1reg[pcifn].lpu;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P1;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P0;
	}

	/*
	 * PSS control registers
	 */
	ioc->ioc_regs.pss_ctl_reg = rb + PSS_CTL_REG;
	ioc->ioc_regs.pss_err_status_reg = rb + PSS_ERR_STATUS_REG;
	ioc->ioc_regs.app_pll_fast_ctl_reg = rb + APP_PLL_LCLK_CTL_REG;
	ioc->ioc_regs.app_pll_slow_ctl_reg = rb + APP_PLL_SCLK_CTL_REG;

	/*
	 * IOC semaphore registers and serialization
	 */
	ioc->ioc_regs.ioc_sem_reg = rb + HOST_SEM0_REG;
	ioc->ioc_regs.ioc_usage_sem_reg = rb + HOST_SEM1_REG;
	ioc->ioc_regs.ioc_init_sem_reg = rb + HOST_SEM2_REG;
	ioc->ioc_regs.ioc_usage_reg = rb + BFA_FW_USE_COUNT;
	ioc->ioc_regs.ioc_fail_sync = rb + BFA_IOC_FAIL_SYNC;

	/**
	 * sram memory access
	 */
	ioc->ioc_regs.smem_page_start = rb + PSS_SMEM_PAGE_START;
	ioc->ioc_regs.smem_pg0 = BFI_IOC_SMEM_PG0_CT;

	/*
	 * err set reg : for notification of hb failure in fcmode
	 */
	ioc->ioc_regs.err_set = (rb + ERR_SET_REG);
}

static void
bfa_ioc_ct2_reg_init(struct bfa_ioc *ioc)
{
	void __iomem *rb;
	int		port = bfa_ioc_portid(ioc);

	rb = bfa_ioc_bar0(ioc);

	ioc->ioc_regs.hfn_mbox = rb + ct2_reg[port].hfn_mbox;
	ioc->ioc_regs.lpu_mbox = rb + ct2_reg[port].lpu_mbox;
	ioc->ioc_regs.host_page_num_fn = rb + ct2_reg[port].hfn_pgn;
	ioc->ioc_regs.hfn_mbox_cmd = rb + ct2_reg[port].hfn;
	ioc->ioc_regs.lpu_mbox_cmd = rb + ct2_reg[port].lpu;
	ioc->ioc_regs.lpu_read_stat = rb + ct2_reg[port].lpu_read;

	if (port == 0) {
		ioc->ioc_regs.heartbeat = rb + CT2_BFA_IOC0_HBEAT_REG;
		ioc->ioc_regs.ioc_fwstate = rb + CT2_BFA_IOC0_STATE_REG;
		ioc->ioc_regs.alt_ioc_fwstate = rb + CT2_BFA_IOC1_STATE_REG;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P0;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P1;
	} else {
		ioc->ioc_regs.heartbeat = rb + CT2_BFA_IOC1_HBEAT_REG;
		ioc->ioc_regs.ioc_fwstate = rb + CT2_BFA_IOC1_STATE_REG;
		ioc->ioc_regs.alt_ioc_fwstate = rb + CT2_BFA_IOC0_STATE_REG;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P1;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P0;
	}

	/*
	 * PSS control registers
	 */
	ioc->ioc_regs.pss_ctl_reg = rb + PSS_CTL_REG;
	ioc->ioc_regs.pss_err_status_reg = rb + PSS_ERR_STATUS_REG;
	ioc->ioc_regs.app_pll_fast_ctl_reg = rb + CT2_APP_PLL_LCLK_CTL_REG;
	ioc->ioc_regs.app_pll_slow_ctl_reg = rb + CT2_APP_PLL_SCLK_CTL_REG;

	/*
	 * IOC semaphore registers and serialization
	 */
	ioc->ioc_regs.ioc_sem_reg = rb + CT2_HOST_SEM0_REG;
	ioc->ioc_regs.ioc_usage_sem_reg = rb + CT2_HOST_SEM1_REG;
	ioc->ioc_regs.ioc_init_sem_reg = rb + CT2_HOST_SEM2_REG;
	ioc->ioc_regs.ioc_usage_reg = rb + CT2_BFA_FW_USE_COUNT;
	ioc->ioc_regs.ioc_fail_sync = rb + CT2_BFA_IOC_FAIL_SYNC;

	/**
	 * sram memory access
	 */
	ioc->ioc_regs.smem_page_start = rb + PSS_SMEM_PAGE_START;
	ioc->ioc_regs.smem_pg0 = BFI_IOC_SMEM_PG0_CT;

	/*
	 * err set reg : for notification of hb failure in fcmode
	 */
	ioc->ioc_regs.err_set = rb + ERR_SET_REG;
}

/* Initialize IOC to port mapping. */

#define FNC_PERS_FN_SHIFT(__fn)	((__fn) * 8)
static void
bfa_ioc_ct_map_port(struct bfa_ioc *ioc)
{
	void __iomem *rb = ioc->pcidev.pci_bar_kva;
	u32	r32;

	/**
	 * For catapult, base port id on personality register and IOC type
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:377", rb + FNC_PERS_REG);
	r32 >>= FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc));
	ioc->port_id = (r32 & __F0_PORT_MAP_MK) >> __F0_PORT_MAP_SH;

}

static void
bfa_ioc_ct2_map_port(struct bfa_ioc *ioc)
{
	void __iomem *rb = ioc->pcidev.pci_bar_kva;
	u32	r32;

	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:389", rb + CT2_HOSTFN_PERSONALITY0);
	ioc->port_id = ((r32 & __FC_LL_PORT_MAP__MK) >> __FC_LL_PORT_MAP__SH);
}

/* Set interrupt mode for a function: INTX or MSIX */
static void
bfa_ioc_ct_isr_mode_set(struct bfa_ioc *ioc, bool msix)
{
	void __iomem *rb = ioc->pcidev.pci_bar_kva;
	u32	r32, mode;

	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:400", rb + FNC_PERS_REG);

	mode = (r32 >> FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc))) &
		__F0_INTX_STATUS;

	/**
	 * If already in desired mode, do not change anything
	 */
	if ((!msix && mode) || (msix && !mode))
		return;

	if (msix)
		mode = __F0_INTX_STATUS_MSIX;
	else
		mode = __F0_INTX_STATUS_INTA;

	r32 &= ~(__F0_INTX_STATUS << FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc)));
	r32 |= (mode << FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc)));

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:419", r32, rb + FNC_PERS_REG);
}

static bool
bfa_ioc_ct2_lpu_read_stat(struct bfa_ioc *ioc)
{
	u32 r32;

	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:427", ioc->ioc_regs.lpu_read_stat);
	if (r32) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:429", 1, ioc->ioc_regs.lpu_read_stat);
		return true;
	}

	return false;
}

/* MSI-X resource allocation for 1860 with no asic block */
#define HOSTFN_MSIX_DEFAULT		64
#define HOSTFN_MSIX_VT_INDEX_MBOX_ERR	0x30138
#define HOSTFN_MSIX_VT_OFST_NUMVT	0x3013c
#define __MSIX_VT_NUMVT__MK		0x003ff800
#define __MSIX_VT_NUMVT__SH		11
#define __MSIX_VT_NUMVT_(_v)		((_v) << __MSIX_VT_NUMVT__SH)
#define __MSIX_VT_OFST_			0x000007ff
void
bfa_nw_ioc_ct2_poweron(struct bfa_ioc *ioc)
{
	void __iomem *rb = ioc->pcidev.pci_bar_kva;
	u32 r32;

	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:450", rb + HOSTFN_MSIX_VT_OFST_NUMVT);
	if (r32 & __MSIX_VT_NUMVT__MK) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:452", r32 & __MSIX_VT_OFST_,
			rb + HOSTFN_MSIX_VT_INDEX_MBOX_ERR);
		return;
	}

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:457", __MSIX_VT_NUMVT_(HOSTFN_MSIX_DEFAULT - 1) |
			HOSTFN_MSIX_DEFAULT * bfa_ioc_pcifn(ioc),
			rb + HOSTFN_MSIX_VT_OFST_NUMVT);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:460", HOSTFN_MSIX_DEFAULT * bfa_ioc_pcifn(ioc),
			rb + HOSTFN_MSIX_VT_INDEX_MBOX_ERR);
}

/* Cleanup hw semaphore and usecnt registers */
static void
bfa_ioc_ct_ownership_reset(struct bfa_ioc *ioc)
{
	bfa_nw_ioc_sem_get(ioc->ioc_regs.ioc_usage_sem_reg);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:469", 0, ioc->ioc_regs.ioc_usage_reg);
	bfa_nw_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);

	/*
	 * Read the hw sem reg to make sure that it is locked
	 * before we clear it. If it is not locked, writing 1
	 * will lock it instead of clearing it.
	 */
	pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:477", ioc->ioc_regs.ioc_sem_reg);
	bfa_nw_ioc_hw_sem_release(ioc);
}

/* Synchronized IOC failure processing routines */
static bool
bfa_ioc_ct_sync_start(struct bfa_ioc *ioc)
{
	u32 r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:485", ioc->ioc_regs.ioc_fail_sync);
	u32 sync_reqd = bfa_ioc_ct_get_sync_reqd(r32);

	/*
	 * Driver load time.  If the sync required bit for this PCI fn
	 * is set, it is due to an unclean exit by the driver for this
	 * PCI fn in the previous incarnation. Whoever comes here first
	 * should clean it up, no matter which PCI fn.
	 */

	if (sync_reqd & bfa_ioc_ct_sync_pos(ioc)) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:496", 0, ioc->ioc_regs.ioc_fail_sync);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:497", 1, ioc->ioc_regs.ioc_usage_reg);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:498", BFI_IOC_UNINIT, ioc->ioc_regs.ioc_fwstate);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:499", BFI_IOC_UNINIT, ioc->ioc_regs.alt_ioc_fwstate);
		return true;
	}

	return bfa_ioc_ct_sync_complete(ioc);
}
/* Synchronized IOC failure processing routines */
static void
bfa_ioc_ct_sync_join(struct bfa_ioc *ioc)
{
	u32 r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:509", ioc->ioc_regs.ioc_fail_sync);
	u32 sync_pos = bfa_ioc_ct_sync_reqd_pos(ioc);

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:512", (r32 | sync_pos), ioc->ioc_regs.ioc_fail_sync);
}

static void
bfa_ioc_ct_sync_leave(struct bfa_ioc *ioc)
{
	u32 r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:518", ioc->ioc_regs.ioc_fail_sync);
	u32 sync_msk = bfa_ioc_ct_sync_reqd_pos(ioc) |
					bfa_ioc_ct_sync_pos(ioc);

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:522", (r32 & ~sync_msk), ioc->ioc_regs.ioc_fail_sync);
}

static void
bfa_ioc_ct_sync_ack(struct bfa_ioc *ioc)
{
	u32 r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:528", ioc->ioc_regs.ioc_fail_sync);

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:530", r32 | bfa_ioc_ct_sync_pos(ioc), ioc->ioc_regs.ioc_fail_sync);
}

static bool
bfa_ioc_ct_sync_complete(struct bfa_ioc *ioc)
{
	u32 r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:536", ioc->ioc_regs.ioc_fail_sync);
	u32 sync_reqd = bfa_ioc_ct_get_sync_reqd(r32);
	u32 sync_ackd = bfa_ioc_ct_get_sync_ackd(r32);
	u32 tmp_ackd;

	if (sync_ackd == 0)
		return true;

	/**
	 * The check below is to see whether any other PCI fn
	 * has reinitialized the ASIC (reset sync_ackd bits)
	 * and failed again while this IOC was waiting for hw
	 * semaphore (in bfa_iocpf_sm_semwait()).
	 */
	tmp_ackd = sync_ackd;
	if ((sync_reqd &  bfa_ioc_ct_sync_pos(ioc)) &&
			!(sync_ackd & bfa_ioc_ct_sync_pos(ioc)))
		sync_ackd |= bfa_ioc_ct_sync_pos(ioc);

	if (sync_reqd == sync_ackd) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:556", bfa_ioc_ct_clear_sync_ackd(r32),
				ioc->ioc_regs.ioc_fail_sync);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:558", BFI_IOC_FAIL, ioc->ioc_regs.ioc_fwstate);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:559", BFI_IOC_FAIL, ioc->ioc_regs.alt_ioc_fwstate);
		return true;
	}

	/**
	 * If another PCI fn reinitialized and failed again while
	 * this IOC was waiting for hw sem, the sync_ackd bit for
	 * this IOC need to be set again to allow reinitialization.
	 */
	if (tmp_ackd != sync_ackd)
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:569", (r32 | sync_ackd), ioc->ioc_regs.ioc_fail_sync);

	return false;
}

static void
bfa_ioc_ct_set_cur_ioc_fwstate(struct bfa_ioc *ioc,
			       enum bfi_ioc_state fwstate)
{
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:578", fwstate, ioc->ioc_regs.ioc_fwstate);
}

static enum bfi_ioc_state
bfa_ioc_ct_get_cur_ioc_fwstate(struct bfa_ioc *ioc)
{
	return (enum bfi_ioc_state)pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:584", ioc->ioc_regs.ioc_fwstate);
}

static void
bfa_ioc_ct_set_alt_ioc_fwstate(struct bfa_ioc *ioc,
			       enum bfi_ioc_state fwstate)
{
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:591", fwstate, ioc->ioc_regs.alt_ioc_fwstate);
}

static enum bfi_ioc_state
bfa_ioc_ct_get_alt_ioc_fwstate(struct bfa_ioc *ioc)
{
	return (enum bfi_ioc_state)pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:597", ioc->ioc_regs.alt_ioc_fwstate);
}

static enum bfa_status
bfa_ioc_ct_pll_init(void __iomem *rb, enum bfi_asic_mode asic_mode)
{
	u32	pll_sclk, pll_fclk, r32;
	bool fcmode = (asic_mode == BFI_ASIC_MODE_FC);

	pll_sclk = __APP_PLL_SCLK_LRESETN | __APP_PLL_SCLK_ENARST |
		__APP_PLL_SCLK_RSEL200500 | __APP_PLL_SCLK_P0_1(3U) |
		__APP_PLL_SCLK_JITLMT0_1(3U) |
		__APP_PLL_SCLK_CNTLMT0_1(1U);
	pll_fclk = __APP_PLL_LCLK_LRESETN | __APP_PLL_LCLK_ENARST |
		__APP_PLL_LCLK_RSEL200500 | __APP_PLL_LCLK_P0_1(3U) |
		__APP_PLL_LCLK_JITLMT0_1(3U) |
		__APP_PLL_LCLK_CNTLMT0_1(1U);

	if (fcmode) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:616", 0, (rb + OP_MODE));
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:617", __APP_EMS_CMLCKSEL |
				__APP_EMS_REFCKBUFEN2 |
				__APP_EMS_CHANNEL_SEL,
				(rb + ETH_MAC_SER_REG));
	} else {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:622", __GLOBAL_FCOE_MODE, (rb + OP_MODE));
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:623", __APP_EMS_REFCKBUFEN1,
				(rb + ETH_MAC_SER_REG));
	}
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:626", BFI_IOC_UNINIT, (rb + BFA_IOC0_STATE_REG));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:627", BFI_IOC_UNINIT, (rb + BFA_IOC1_STATE_REG));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:628", 0xffffffffU, (rb + HOSTFN0_INT_MSK));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:629", 0xffffffffU, (rb + HOSTFN1_INT_MSK));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:630", 0xffffffffU, (rb + HOSTFN0_INT_STATUS));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:631", 0xffffffffU, (rb + HOSTFN1_INT_STATUS));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:632", 0xffffffffU, (rb + HOSTFN0_INT_MSK));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:633", 0xffffffffU, (rb + HOSTFN1_INT_MSK));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:634", pll_sclk |
		__APP_PLL_SCLK_LOGIC_SOFT_RESET,
		rb + APP_PLL_SCLK_CTL_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:637", pll_fclk |
		__APP_PLL_LCLK_LOGIC_SOFT_RESET,
		rb + APP_PLL_LCLK_CTL_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:640", pll_sclk |
		__APP_PLL_SCLK_LOGIC_SOFT_RESET | __APP_PLL_SCLK_ENABLE,
		rb + APP_PLL_SCLK_CTL_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:643", pll_fclk |
		__APP_PLL_LCLK_LOGIC_SOFT_RESET | __APP_PLL_LCLK_ENABLE,
		rb + APP_PLL_LCLK_CTL_REG);
	pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:646", rb + HOSTFN0_INT_MSK);
	udelay(2000);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:648", 0xffffffffU, (rb + HOSTFN0_INT_STATUS));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:649", 0xffffffffU, (rb + HOSTFN1_INT_STATUS));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:650", pll_sclk |
		__APP_PLL_SCLK_ENABLE,
		rb + APP_PLL_SCLK_CTL_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:653", pll_fclk |
		__APP_PLL_LCLK_ENABLE,
		rb + APP_PLL_LCLK_CTL_REG);

	if (!fcmode) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:658", __PMM_1T_RESET_P, (rb + PMM_1T_RESET_REG_P0));
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:659", __PMM_1T_RESET_P, (rb + PMM_1T_RESET_REG_P1));
	}
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:661", rb + PSS_CTL_REG);
	r32 &= ~__PSS_LMEM_RESET;
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:663", r32, (rb + PSS_CTL_REG));
	udelay(1000);
	if (!fcmode) {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:666", 0, (rb + PMM_1T_RESET_REG_P0));
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:667", 0, (rb + PMM_1T_RESET_REG_P1));
	}

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:670", __EDRAM_BISTR_START, (rb + MBIST_CTL_REG));
	udelay(1000);
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:672", rb + MBIST_STAT_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:673", 0, (rb + MBIST_CTL_REG));
	return BFA_STATUS_OK;
}

static void
bfa_ioc_ct2_sclk_init(void __iomem *rb)
{
	u32 r32;

	/*
	 * put s_clk PLL and PLL FSM in reset
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:685", rb + CT2_APP_PLL_SCLK_CTL_REG);
	r32 &= ~(__APP_PLL_SCLK_ENABLE | __APP_PLL_SCLK_LRESETN);
	r32 |= (__APP_PLL_SCLK_ENARST | __APP_PLL_SCLK_BYPASS |
		__APP_PLL_SCLK_LOGIC_SOFT_RESET);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:689", r32, (rb + CT2_APP_PLL_SCLK_CTL_REG));

	/*
	 * Ignore mode and program for the max clock (which is FC16)
	 * Firmware/NFC will do the PLL init appropriately
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:695", rb + CT2_APP_PLL_SCLK_CTL_REG);
	r32 &= ~(__APP_PLL_SCLK_REFCLK_SEL | __APP_PLL_SCLK_CLK_DIV2);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:697", r32, (rb + CT2_APP_PLL_SCLK_CTL_REG));

	/*
	 * while doing PLL init dont clock gate ethernet subsystem
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:702", rb + CT2_CHIP_MISC_PRG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:703", r32 | __ETH_CLK_ENABLE_PORT0,
	       rb + CT2_CHIP_MISC_PRG);

	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:706", rb + CT2_PCIE_MISC_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:707", r32 | __ETH_CLK_ENABLE_PORT1,
	       rb + CT2_PCIE_MISC_REG);

	/*
	 * set sclk value
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:713", rb + CT2_APP_PLL_SCLK_CTL_REG);
	r32 &= (__P_SCLK_PLL_LOCK | __APP_PLL_SCLK_REFCLK_SEL |
		__APP_PLL_SCLK_CLK_DIV2);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:716", r32 | 0x1061731b, rb + CT2_APP_PLL_SCLK_CTL_REG);

	/*
	 * poll for s_clk lock or delay 1ms
	 */
	udelay(1000);

	/*
	 * Dont do clock gating for ethernet subsystem, firmware/NFC will
	 * do this appropriately
	 */
}

static void
bfa_ioc_ct2_lclk_init(void __iomem *rb)
{
	u32 r32;

	/*
	 * put l_clk PLL and PLL FSM in reset
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:737", rb + CT2_APP_PLL_LCLK_CTL_REG);
	r32 &= ~(__APP_PLL_LCLK_ENABLE | __APP_PLL_LCLK_LRESETN);
	r32 |= (__APP_PLL_LCLK_ENARST | __APP_PLL_LCLK_BYPASS |
		__APP_PLL_LCLK_LOGIC_SOFT_RESET);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:741", r32, rb + CT2_APP_PLL_LCLK_CTL_REG);

	/*
	 * set LPU speed (set for FC16 which will work for other modes)
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:746", rb + CT2_CHIP_MISC_PRG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:747", r32, (rb + CT2_CHIP_MISC_PRG));

	/*
	 * set LPU half speed (set for FC16 which will work for other modes)
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:752", rb + CT2_APP_PLL_LCLK_CTL_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:753", r32, rb + CT2_APP_PLL_LCLK_CTL_REG);

	/*
	 * set lclk for mode (set for FC16)
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:758", rb + CT2_APP_PLL_LCLK_CTL_REG);
	r32 &= (__P_LCLK_PLL_LOCK | __APP_LPUCLK_HALFSPEED);
	r32 |= 0x20c1731b;
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:761", r32, (rb + CT2_APP_PLL_LCLK_CTL_REG));

	/*
	 * poll for s_clk lock or delay 1ms
	 */
	udelay(1000);
}

static void
bfa_ioc_ct2_mem_init(void __iomem *rb)
{
	u32 r32;

	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:774", rb + PSS_CTL_REG);
	r32 &= ~__PSS_LMEM_RESET;
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:776", r32, rb + PSS_CTL_REG);
	udelay(1000);

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:779", __EDRAM_BISTR_START, rb + CT2_MBIST_CTL_REG);
	udelay(1000);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:781", 0, rb + CT2_MBIST_CTL_REG);
}

static void
bfa_ioc_ct2_mac_reset(void __iomem *rb)
{
	volatile u32 r32;

	bfa_ioc_ct2_sclk_init(rb);
	bfa_ioc_ct2_lclk_init(rb);

	/*
	 * release soft reset on s_clk & l_clk
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:795", rb + CT2_APP_PLL_SCLK_CTL_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:796", r32 & ~__APP_PLL_SCLK_LOGIC_SOFT_RESET,
	       rb + CT2_APP_PLL_SCLK_CTL_REG);

	/*
	 * release soft reset on s_clk & l_clk
	 */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:802", rb + CT2_APP_PLL_LCLK_CTL_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:803", r32 & ~__APP_PLL_LCLK_LOGIC_SOFT_RESET,
	       rb + CT2_APP_PLL_LCLK_CTL_REG);

	/* put port0, port1 MAC & AHB in reset */
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:807", __CSI_MAC_RESET | __CSI_MAC_AHB_RESET,
	       rb + CT2_CSI_MAC_CONTROL_REG(0));
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:809", __CSI_MAC_RESET | __CSI_MAC_AHB_RESET,
	       rb + CT2_CSI_MAC_CONTROL_REG(1));
}

#define CT2_NFC_MAX_DELAY       1000
#define CT2_NFC_VER_VALID       0x143
#define BFA_IOC_PLL_POLL        1000000

static bool
bfa_ioc_ct2_nfc_halted(void __iomem *rb)
{
	volatile u32 r32;

	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:822", rb + CT2_NFC_CSR_SET_REG);
	if (r32 & __NFC_CONTROLLER_HALTED)
		return true;

	return false;
}

static void
bfa_ioc_ct2_nfc_resume(void __iomem *rb)
{
	volatile u32 r32;
	int i;

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:835", __HALT_NFC_CONTROLLER, rb + CT2_NFC_CSR_CLR_REG);
	for (i = 0; i < CT2_NFC_MAX_DELAY; i++) {
		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:837", rb + CT2_NFC_CSR_SET_REG);
		if (!(r32 & __NFC_CONTROLLER_HALTED))
			return;
		udelay(1000);
	}
	BUG_ON(1);
}

static enum bfa_status
bfa_ioc_ct2_pll_init(void __iomem *rb, enum bfi_asic_mode asic_mode)
{
	volatile u32 wgn, r32;
	u32 nfc_ver, i;

	wgn = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:851", rb + CT2_WGN_STATUS);

	nfc_ver = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:853", rb + CT2_RSC_GPR15_REG);

	if (wgn == (__A2T_AHB_LOAD | __WGN_READY) &&
	    nfc_ver >= CT2_NFC_VER_VALID) {
		if (bfa_ioc_ct2_nfc_halted(rb))
			bfa_ioc_ct2_nfc_resume(rb);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:859", __RESET_AND_START_SCLK_LCLK_PLLS,
				rb + CT2_CSI_FW_CTL_SET_REG);

		for (i = 0; i < BFA_IOC_PLL_POLL; i++) {
			r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:863", rb + CT2_APP_PLL_LCLK_CTL_REG);
			if (r32 & __RESET_AND_START_SCLK_LCLK_PLLS)
				break;
		}
		BUG_ON(!(r32 & __RESET_AND_START_SCLK_LCLK_PLLS));

		for (i = 0; i < BFA_IOC_PLL_POLL; i++) {
			r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:870", rb + CT2_APP_PLL_LCLK_CTL_REG);
			if (!(r32 & __RESET_AND_START_SCLK_LCLK_PLLS))
				break;
		}
		BUG_ON(r32 & __RESET_AND_START_SCLK_LCLK_PLLS);
		udelay(1000);

		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:877", rb + CT2_CSI_FW_CTL_REG);
		BUG_ON(r32 & __RESET_AND_START_SCLK_LCLK_PLLS);
	} else {
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:880", __HALT_NFC_CONTROLLER, (rb + CT2_NFC_CSR_SET_REG));
		for (i = 0; i < CT2_NFC_MAX_DELAY; i++) {
			r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:882", rb + CT2_NFC_CSR_SET_REG);
			if (r32 & __NFC_CONTROLLER_HALTED)
				break;
			udelay(1000);
		}

		bfa_ioc_ct2_mac_reset(rb);
		bfa_ioc_ct2_sclk_init(rb);
		bfa_ioc_ct2_lclk_init(rb);

		/* release soft reset on s_clk & l_clk */
		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:893", rb + CT2_APP_PLL_SCLK_CTL_REG);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:894", r32 & ~__APP_PLL_SCLK_LOGIC_SOFT_RESET,
				rb + CT2_APP_PLL_SCLK_CTL_REG);
		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:896", rb + CT2_APP_PLL_LCLK_CTL_REG);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:897", r32 & ~__APP_PLL_LCLK_LOGIC_SOFT_RESET,
				rb + CT2_APP_PLL_LCLK_CTL_REG);
	}

	/* Announce flash device presence, if flash was corrupted. */
	if (wgn == (__WGN_READY | __GLBL_PF_VF_CFG_RDY)) {
		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:903", rb + PSS_GPIO_OUT_REG);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:904", r32 & ~1, rb + PSS_GPIO_OUT_REG);
		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:905", rb + PSS_GPIO_OE_REG);
		pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:906", r32 | 1, rb + PSS_GPIO_OE_REG);
	}

	/*
	 * Mask the interrupts and clear any
	 * pending interrupts left by BIOS/EFI
	 */
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:913", 1, rb + CT2_LPU0_HOSTFN_MBOX0_MSK);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:914", 1, rb + CT2_LPU1_HOSTFN_MBOX0_MSK);

	/* For first time initialization, no need to clear interrupts */
	r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:917", rb + HOST_SEM5_REG);
	if (r32 & 0x1) {
		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:919", rb + CT2_LPU0_HOSTFN_CMD_STAT);
		if (r32 == 1) {
			pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:921", 1, rb + CT2_LPU0_HOSTFN_CMD_STAT);
			pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:922", rb + CT2_LPU0_HOSTFN_CMD_STAT);
		}
		r32 = pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:924", rb + CT2_LPU1_HOSTFN_CMD_STAT);
		if (r32 == 1) {
			pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:926", 1, rb + CT2_LPU1_HOSTFN_CMD_STAT);
			pete_readl("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:927", rb + CT2_LPU1_HOSTFN_CMD_STAT);
		}
	}

	bfa_ioc_ct2_mem_init(rb);

	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:933", BFI_IOC_UNINIT, rb + CT2_BFA_IOC0_STATE_REG);
	pete_writel("drivers/net/ethernet/brocade/bna/bfa_ioc_ct.c:934", BFI_IOC_UNINIT, rb + CT2_BFA_IOC1_STATE_REG);
	return BFA_STATUS_OK;
}
