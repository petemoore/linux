// SPDX-License-Identifier: GPL-2.0
/* $Date: 2006/02/07 04:21:54 $ $RCSfile: tp.c,v $ $Revision: 1.73 $ */
#include "common.h"
#include "regs.h"
#include "tp.h"
#ifdef CONFIG_CHELSIO_T1_1G
#include "fpga_defs.h"
#endif

struct petp {
	adapter_t *adapter;
};

/* Pause deadlock avoidance parameters */
#define DROP_MSEC 16
#define DROP_PKTS_CNT  1

static void tp_init(adapter_t * ap, const struct tp_params *p,
		    unsigned int tp_clk)
{
	u32 val;

	if (!t1_is_asic(ap))
		return;

	val = F_TP_IN_CSPI_CPL | F_TP_IN_CSPI_CHECK_IP_CSUM |
		F_TP_IN_CSPI_CHECK_TCP_CSUM | F_TP_IN_ESPI_ETHERNET;
	if (!p->pm_size)
		val |= F_OFFLOAD_DISABLE;
	else
		val |= F_TP_IN_ESPI_CHECK_IP_CSUM | F_TP_IN_ESPI_CHECK_TCP_CSUM;
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:32", val, ap->regs + A_TP_IN_CONFIG);
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:33", F_TP_OUT_CSPI_CPL |
	       F_TP_OUT_ESPI_ETHERNET |
	       F_TP_OUT_ESPI_GENERATE_IP_CSUM |
	       F_TP_OUT_ESPI_GENERATE_TCP_CSUM, ap->regs + A_TP_OUT_CONFIG);
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:37", V_IP_TTL(64) |
	       F_PATH_MTU /* IP DF bit */  |
	       V_5TUPLE_LOOKUP(p->use_5tuple_mode) |
	       V_SYN_COOKIE_PARAMETER(29), ap->regs + A_TP_GLOBAL_CONFIG);
	/*
	 * Enable pause frame deadlock prevention.
	 */
	if (is_T2(ap) && ap->params.nports > 1) {
		u32 drop_ticks = DROP_MSEC * (tp_clk / 1000);

		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:47", F_ENABLE_TX_DROP | F_ENABLE_TX_ERROR |
		       V_DROP_TICKS_CNT(drop_ticks) |
		       V_NUM_PKTS_DROPPED(DROP_PKTS_CNT),
		       ap->regs + A_TP_TX_DROP_CONFIG);
	}
}

void t1_tp_destroy(struct petp *tp)
{
	kfree(tp);
}

struct petp *t1_tp_create(adapter_t *adapter, struct tp_params *p)
{
	struct petp *tp = kzalloc(sizeof(*tp), GFP_KERNEL);

	if (!tp)
		return NULL;

	tp->adapter = adapter;

	return tp;
}

void t1_tp_intr_enable(struct petp *tp)
{
	u32 tp_intr = pete_readl("drivers/net/ethernet/chelsio/cxgb/tp.c:73", tp->adapter->regs + A_PL_ENABLE);

#ifdef CONFIG_CHELSIO_T1_1G
	if (!t1_is_asic(tp->adapter)) {
		/* FPGA */
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:78", 0xffffffff,
		       tp->adapter->regs + FPGA_TP_ADDR_INTERRUPT_ENABLE);
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:80", tp_intr | FPGA_PCIX_INTERRUPT_TP,
		       tp->adapter->regs + A_PL_ENABLE);
	} else
#endif
	{
		/* We don't use any TP interrupts */
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:86", 0, tp->adapter->regs + A_TP_INT_ENABLE);
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:87", tp_intr | F_PL_INTR_TP,
		       tp->adapter->regs + A_PL_ENABLE);
	}
}

void t1_tp_intr_disable(struct petp *tp)
{
	u32 tp_intr = pete_readl("drivers/net/ethernet/chelsio/cxgb/tp.c:94", tp->adapter->regs + A_PL_ENABLE);

#ifdef CONFIG_CHELSIO_T1_1G
	if (!t1_is_asic(tp->adapter)) {
		/* FPGA */
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:99", 0, tp->adapter->regs + FPGA_TP_ADDR_INTERRUPT_ENABLE);
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:100", tp_intr & ~FPGA_PCIX_INTERRUPT_TP,
		       tp->adapter->regs + A_PL_ENABLE);
	} else
#endif
	{
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:105", 0, tp->adapter->regs + A_TP_INT_ENABLE);
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:106", tp_intr & ~F_PL_INTR_TP,
		       tp->adapter->regs + A_PL_ENABLE);
	}
}

void t1_tp_intr_clear(struct petp *tp)
{
#ifdef CONFIG_CHELSIO_T1_1G
	if (!t1_is_asic(tp->adapter)) {
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:115", 0xffffffff,
		       tp->adapter->regs + FPGA_TP_ADDR_INTERRUPT_CAUSE);
		pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:117", FPGA_PCIX_INTERRUPT_TP, tp->adapter->regs + A_PL_CAUSE);
		return;
	}
#endif
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:121", 0xffffffff, tp->adapter->regs + A_TP_INT_CAUSE);
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:122", F_PL_INTR_TP, tp->adapter->regs + A_PL_CAUSE);
}

int t1_tp_intr_handler(struct petp *tp)
{
	u32 cause;

#ifdef CONFIG_CHELSIO_T1_1G
	/* FPGA doesn't support TP interrupts. */
	if (!t1_is_asic(tp->adapter))
		return 1;
#endif

	cause = pete_readl("drivers/net/ethernet/chelsio/cxgb/tp.c:135", tp->adapter->regs + A_TP_INT_CAUSE);
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:136", cause, tp->adapter->regs + A_TP_INT_CAUSE);
	return 0;
}

static void set_csum_offload(struct petp *tp, u32 csum_bit, int enable)
{
	u32 val = pete_readl("drivers/net/ethernet/chelsio/cxgb/tp.c:142", tp->adapter->regs + A_TP_GLOBAL_CONFIG);

	if (enable)
		val |= csum_bit;
	else
		val &= ~csum_bit;
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:148", val, tp->adapter->regs + A_TP_GLOBAL_CONFIG);
}

void t1_tp_set_ip_checksum_offload(struct petp *tp, int enable)
{
	set_csum_offload(tp, F_IP_CSUM, enable);
}

void t1_tp_set_tcp_checksum_offload(struct petp *tp, int enable)
{
	set_csum_offload(tp, F_TCP_CSUM, enable);
}

/*
 * Initialize TP state.  tp_params contains initial settings for some TP
 * parameters, particularly the one-time PM and CM settings.
 */
int t1_tp_reset(struct petp *tp, struct tp_params *p, unsigned int tp_clk)
{
	adapter_t *adapter = tp->adapter;

	tp_init(adapter, p, tp_clk);
	pete_writel("drivers/net/ethernet/chelsio/cxgb/tp.c:170", F_TP_RESET, adapter->regs +  A_TP_RESET);
	return 0;
}
