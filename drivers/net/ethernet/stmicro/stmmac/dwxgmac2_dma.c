// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (c) 2018 Synopsys, Inc. and/or its affiliates.
 * stmmac XGMAC support.
 */

#include <linux/iopoll.h>
#include "stmmac.h"
#include "dwxgmac2.h"

static int dwxgmac2_dma_reset(void __iomem *ioaddr)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:13", ioaddr + XGMAC_DMA_MODE);

	/* DMA SW reset */
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:16", value | XGMAC_SWR, ioaddr + XGMAC_DMA_MODE);

	return readl_poll_timeout(ioaddr + XGMAC_DMA_MODE, value,
				  !(value & XGMAC_SWR), 0, 100000);
}

static void dwxgmac2_dma_init(void __iomem *ioaddr,
			      struct stmmac_dma_cfg *dma_cfg, int atds)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:25", ioaddr + XGMAC_DMA_SYSBUS_MODE);

	if (dma_cfg->aal)
		value |= XGMAC_AAL;

	if (dma_cfg->eame)
		value |= XGMAC_EAME;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:33", value, ioaddr + XGMAC_DMA_SYSBUS_MODE);
}

static void dwxgmac2_dma_init_chan(void __iomem *ioaddr,
				   struct stmmac_dma_cfg *dma_cfg, u32 chan)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:39", ioaddr + XGMAC_DMA_CH_CONTROL(chan));

	if (dma_cfg->pblx8)
		value |= XGMAC_PBLx8;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:44", value, ioaddr + XGMAC_DMA_CH_CONTROL(chan));
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:45", XGMAC_DMA_INT_DEFAULT_EN, ioaddr + XGMAC_DMA_CH_INT_EN(chan));
}

static void dwxgmac2_dma_init_rx_chan(void __iomem *ioaddr,
				      struct stmmac_dma_cfg *dma_cfg,
				      dma_addr_t phy, u32 chan)
{
	u32 rxpbl = dma_cfg->rxpbl ?: dma_cfg->pbl;
	u32 value;

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:55", ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
	value &= ~XGMAC_RxPBL;
	value |= (rxpbl << XGMAC_RxPBL_SHIFT) & XGMAC_RxPBL;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:58", value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:60", upper_32_bits(phy), ioaddr + XGMAC_DMA_CH_RxDESC_HADDR(chan));
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:61", lower_32_bits(phy), ioaddr + XGMAC_DMA_CH_RxDESC_LADDR(chan));
}

static void dwxgmac2_dma_init_tx_chan(void __iomem *ioaddr,
				      struct stmmac_dma_cfg *dma_cfg,
				      dma_addr_t phy, u32 chan)
{
	u32 txpbl = dma_cfg->txpbl ?: dma_cfg->pbl;
	u32 value;

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:71", ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));
	value &= ~XGMAC_TxPBL;
	value |= (txpbl << XGMAC_TxPBL_SHIFT) & XGMAC_TxPBL;
	value |= XGMAC_OSP;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:75", value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:77", upper_32_bits(phy), ioaddr + XGMAC_DMA_CH_TxDESC_HADDR(chan));
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:78", lower_32_bits(phy), ioaddr + XGMAC_DMA_CH_TxDESC_LADDR(chan));
}

static void dwxgmac2_dma_axi(void __iomem *ioaddr, struct stmmac_axi *axi)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:83", ioaddr + XGMAC_DMA_SYSBUS_MODE);
	int i;

	if (axi->axi_lpi_en)
		value |= XGMAC_EN_LPI;
	if (axi->axi_xit_frm)
		value |= XGMAC_LPI_XIT_PKT;

	value &= ~XGMAC_WR_OSR_LMT;
	value |= (axi->axi_wr_osr_lmt << XGMAC_WR_OSR_LMT_SHIFT) &
		XGMAC_WR_OSR_LMT;

	value &= ~XGMAC_RD_OSR_LMT;
	value |= (axi->axi_rd_osr_lmt << XGMAC_RD_OSR_LMT_SHIFT) &
		XGMAC_RD_OSR_LMT;

	if (!axi->axi_fb)
		value |= XGMAC_UNDEF;

	value &= ~XGMAC_BLEN;
	for (i = 0; i < AXI_BLEN; i++) {
		switch (axi->axi_blen[i]) {
		case 256:
			value |= XGMAC_BLEN256;
			break;
		case 128:
			value |= XGMAC_BLEN128;
			break;
		case 64:
			value |= XGMAC_BLEN64;
			break;
		case 32:
			value |= XGMAC_BLEN32;
			break;
		case 16:
			value |= XGMAC_BLEN16;
			break;
		case 8:
			value |= XGMAC_BLEN8;
			break;
		case 4:
			value |= XGMAC_BLEN4;
			break;
		}
	}

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:129", value, ioaddr + XGMAC_DMA_SYSBUS_MODE);
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:130", XGMAC_TDPS, ioaddr + XGMAC_TX_EDMA_CTRL);
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:131", XGMAC_RDPS, ioaddr + XGMAC_RX_EDMA_CTRL);
}

static void dwxgmac2_dma_dump_regs(void __iomem *ioaddr, u32 *reg_space)
{
	int i;

	for (i = (XGMAC_DMA_MODE / 4); i < XGMAC_REGSIZE; i++)
		reg_space[i] = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:139", ioaddr + i * 4);
}

static void dwxgmac2_dma_rx_mode(void __iomem *ioaddr, int mode,
				 u32 channel, int fifosz, u8 qmode)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:145", ioaddr + XGMAC_MTL_RXQ_OPMODE(channel));
	unsigned int rqs = fifosz / 256 - 1;

	if (mode == SF_DMA_MODE) {
		value |= XGMAC_RSF;
	} else {
		value &= ~XGMAC_RSF;
		value &= ~XGMAC_RTC;

		if (mode <= 64)
			value |= 0x0 << XGMAC_RTC_SHIFT;
		else if (mode <= 96)
			value |= 0x2 << XGMAC_RTC_SHIFT;
		else
			value |= 0x3 << XGMAC_RTC_SHIFT;
	}

	value &= ~XGMAC_RQS;
	value |= (rqs << XGMAC_RQS_SHIFT) & XGMAC_RQS;

	if ((fifosz >= 4096) && (qmode != MTL_QUEUE_AVB)) {
		u32 flow = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:166", ioaddr + XGMAC_MTL_RXQ_FLOW_CONTROL(channel));
		unsigned int rfd, rfa;

		value |= XGMAC_EHFC;

		/* Set Threshold for Activating Flow Control to min 2 frames,
		 * i.e. 1500 * 2 = 3000 bytes.
		 *
		 * Set Threshold for Deactivating Flow Control to min 1 frame,
		 * i.e. 1500 bytes.
		 */
		switch (fifosz) {
		case 4096:
			/* This violates the above formula because of FIFO size
			 * limit therefore overflow may occur in spite of this.
			 */
			rfd = 0x03; /* Full-2.5K */
			rfa = 0x01; /* Full-1.5K */
			break;

		default:
			rfd = 0x07; /* Full-4.5K */
			rfa = 0x04; /* Full-3K */
			break;
		}

		flow &= ~XGMAC_RFD;
		flow |= rfd << XGMAC_RFD_SHIFT;

		flow &= ~XGMAC_RFA;
		flow |= rfa << XGMAC_RFA_SHIFT;

		pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:198", flow, ioaddr + XGMAC_MTL_RXQ_FLOW_CONTROL(channel));
	}

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:201", value, ioaddr + XGMAC_MTL_RXQ_OPMODE(channel));

	/* Enable MTL RX overflow */
	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:204", ioaddr + XGMAC_MTL_QINTEN(channel));
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:205", value | XGMAC_RXOIE, ioaddr + XGMAC_MTL_QINTEN(channel));
}

static void dwxgmac2_dma_tx_mode(void __iomem *ioaddr, int mode,
				 u32 channel, int fifosz, u8 qmode)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:211", ioaddr + XGMAC_MTL_TXQ_OPMODE(channel));
	unsigned int tqs = fifosz / 256 - 1;

	if (mode == SF_DMA_MODE) {
		value |= XGMAC_TSF;
	} else {
		value &= ~XGMAC_TSF;
		value &= ~XGMAC_TTC;

		if (mode <= 64)
			value |= 0x0 << XGMAC_TTC_SHIFT;
		else if (mode <= 96)
			value |= 0x2 << XGMAC_TTC_SHIFT;
		else if (mode <= 128)
			value |= 0x3 << XGMAC_TTC_SHIFT;
		else if (mode <= 192)
			value |= 0x4 << XGMAC_TTC_SHIFT;
		else if (mode <= 256)
			value |= 0x5 << XGMAC_TTC_SHIFT;
		else if (mode <= 384)
			value |= 0x6 << XGMAC_TTC_SHIFT;
		else
			value |= 0x7 << XGMAC_TTC_SHIFT;
	}

	/* Use static TC to Queue mapping */
	value |= (channel << XGMAC_Q2TCMAP_SHIFT) & XGMAC_Q2TCMAP;

	value &= ~XGMAC_TXQEN;
	if (qmode != MTL_QUEUE_AVB)
		value |= 0x2 << XGMAC_TXQEN_SHIFT;
	else
		value |= 0x1 << XGMAC_TXQEN_SHIFT;

	value &= ~XGMAC_TQS;
	value |= (tqs << XGMAC_TQS_SHIFT) & XGMAC_TQS;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:248", value, ioaddr +  XGMAC_MTL_TXQ_OPMODE(channel));
}

static void dwxgmac2_enable_dma_irq(void __iomem *ioaddr, u32 chan,
				    bool rx, bool tx)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:254", ioaddr + XGMAC_DMA_CH_INT_EN(chan));

	if (rx)
		value |= XGMAC_DMA_INT_DEFAULT_RX;
	if (tx)
		value |= XGMAC_DMA_INT_DEFAULT_TX;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:261", value, ioaddr + XGMAC_DMA_CH_INT_EN(chan));
}

static void dwxgmac2_disable_dma_irq(void __iomem *ioaddr, u32 chan,
				     bool rx, bool tx)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:267", ioaddr + XGMAC_DMA_CH_INT_EN(chan));

	if (rx)
		value &= ~XGMAC_DMA_INT_DEFAULT_RX;
	if (tx)
		value &= ~XGMAC_DMA_INT_DEFAULT_TX;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:274", value, ioaddr + XGMAC_DMA_CH_INT_EN(chan));
}

static void dwxgmac2_dma_start_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:281", ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));
	value |= XGMAC_TXST;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:283", value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:285", ioaddr + XGMAC_TX_CONFIG);
	value |= XGMAC_CONFIG_TE;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:287", value, ioaddr + XGMAC_TX_CONFIG);
}

static void dwxgmac2_dma_stop_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:294", ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));
	value &= ~XGMAC_TXST;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:296", value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:298", ioaddr + XGMAC_TX_CONFIG);
	value &= ~XGMAC_CONFIG_TE;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:300", value, ioaddr + XGMAC_TX_CONFIG);
}

static void dwxgmac2_dma_start_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:307", ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
	value |= XGMAC_RXST;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:309", value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:311", ioaddr + XGMAC_RX_CONFIG);
	value |= XGMAC_CONFIG_RE;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:313", value, ioaddr + XGMAC_RX_CONFIG);
}

static void dwxgmac2_dma_stop_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value;

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:320", ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
	value &= ~XGMAC_RXST;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:322", value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
}

static int dwxgmac2_dma_interrupt(void __iomem *ioaddr,
				  struct stmmac_extra_stats *x, u32 chan,
				  u32 dir)
{
	u32 intr_status = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:329", ioaddr + XGMAC_DMA_CH_STATUS(chan));
	u32 intr_en = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:330", ioaddr + XGMAC_DMA_CH_INT_EN(chan));
	int ret = 0;

	if (dir == DMA_DIR_RX)
		intr_status &= XGMAC_DMA_STATUS_MSK_RX;
	else if (dir == DMA_DIR_TX)
		intr_status &= XGMAC_DMA_STATUS_MSK_TX;

	/* ABNORMAL interrupts */
	if (unlikely(intr_status & XGMAC_AIS)) {
		if (unlikely(intr_status & XGMAC_RBU)) {
			x->rx_buf_unav_irq++;
			ret |= handle_rx;
		}
		if (unlikely(intr_status & XGMAC_TPS)) {
			x->tx_process_stopped_irq++;
			ret |= tx_hard_error;
		}
		if (unlikely(intr_status & XGMAC_FBE)) {
			x->fatal_bus_error_irq++;
			ret |= tx_hard_error;
		}
	}

	/* TX/RX NORMAL interrupts */
	if (likely(intr_status & XGMAC_NIS)) {
		x->normal_irq_n++;

		if (likely(intr_status & XGMAC_RI)) {
			x->rx_normal_irq_n++;
			ret |= handle_rx;
		}
		if (likely(intr_status & (XGMAC_TI | XGMAC_TBU))) {
			x->tx_normal_irq_n++;
			ret |= handle_tx;
		}
	}

	/* Clear interrupts */
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:369", intr_en & intr_status, ioaddr + XGMAC_DMA_CH_STATUS(chan));

	return ret;
}

static int dwxgmac2_get_hw_feature(void __iomem *ioaddr,
				   struct dma_features *dma_cap)
{
	u32 hw_cap;

	/*  MAC HW feature 0 */
	hw_cap = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:380", ioaddr + XGMAC_HW_FEATURE0);
	dma_cap->vlins = (hw_cap & XGMAC_HWFEAT_SAVLANINS) >> 27;
	dma_cap->rx_coe = (hw_cap & XGMAC_HWFEAT_RXCOESEL) >> 16;
	dma_cap->tx_coe = (hw_cap & XGMAC_HWFEAT_TXCOESEL) >> 14;
	dma_cap->eee = (hw_cap & XGMAC_HWFEAT_EEESEL) >> 13;
	dma_cap->atime_stamp = (hw_cap & XGMAC_HWFEAT_TSSEL) >> 12;
	dma_cap->av = (hw_cap & XGMAC_HWFEAT_AVSEL) >> 11;
	dma_cap->av &= !((hw_cap & XGMAC_HWFEAT_RAVSEL) >> 10);
	dma_cap->arpoffsel = (hw_cap & XGMAC_HWFEAT_ARPOFFSEL) >> 9;
	dma_cap->rmon = (hw_cap & XGMAC_HWFEAT_MMCSEL) >> 8;
	dma_cap->pmt_magic_frame = (hw_cap & XGMAC_HWFEAT_MGKSEL) >> 7;
	dma_cap->pmt_remote_wake_up = (hw_cap & XGMAC_HWFEAT_RWKSEL) >> 6;
	dma_cap->vlhash = (hw_cap & XGMAC_HWFEAT_VLHASH) >> 4;
	dma_cap->mbps_1000 = (hw_cap & XGMAC_HWFEAT_GMIISEL) >> 1;

	/* MAC HW feature 1 */
	hw_cap = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:396", ioaddr + XGMAC_HW_FEATURE1);
	dma_cap->l3l4fnum = (hw_cap & XGMAC_HWFEAT_L3L4FNUM) >> 27;
	dma_cap->hash_tb_sz = (hw_cap & XGMAC_HWFEAT_HASHTBLSZ) >> 24;
	dma_cap->rssen = (hw_cap & XGMAC_HWFEAT_RSSEN) >> 20;
	dma_cap->tsoen = (hw_cap & XGMAC_HWFEAT_TSOEN) >> 18;
	dma_cap->sphen = (hw_cap & XGMAC_HWFEAT_SPHEN) >> 17;

	dma_cap->addr64 = (hw_cap & XGMAC_HWFEAT_ADDR64) >> 14;
	switch (dma_cap->addr64) {
	case 0:
		dma_cap->addr64 = 32;
		break;
	case 1:
		dma_cap->addr64 = 40;
		break;
	case 2:
		dma_cap->addr64 = 48;
		break;
	default:
		dma_cap->addr64 = 32;
		break;
	}

	dma_cap->tx_fifo_size =
		128 << ((hw_cap & XGMAC_HWFEAT_TXFIFOSIZE) >> 6);
	dma_cap->rx_fifo_size =
		128 << ((hw_cap & XGMAC_HWFEAT_RXFIFOSIZE) >> 0);

	/* MAC HW feature 2 */
	hw_cap = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:425", ioaddr + XGMAC_HW_FEATURE2);
	dma_cap->pps_out_num = (hw_cap & XGMAC_HWFEAT_PPSOUTNUM) >> 24;
	dma_cap->number_tx_channel =
		((hw_cap & XGMAC_HWFEAT_TXCHCNT) >> 18) + 1;
	dma_cap->number_rx_channel =
		((hw_cap & XGMAC_HWFEAT_RXCHCNT) >> 12) + 1;
	dma_cap->number_tx_queues =
		((hw_cap & XGMAC_HWFEAT_TXQCNT) >> 6) + 1;
	dma_cap->number_rx_queues =
		((hw_cap & XGMAC_HWFEAT_RXQCNT) >> 0) + 1;

	/* MAC HW feature 3 */
	hw_cap = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:437", ioaddr + XGMAC_HW_FEATURE3);
	dma_cap->tbssel = (hw_cap & XGMAC_HWFEAT_TBSSEL) >> 27;
	dma_cap->fpesel = (hw_cap & XGMAC_HWFEAT_FPESEL) >> 26;
	dma_cap->estwid = (hw_cap & XGMAC_HWFEAT_ESTWID) >> 23;
	dma_cap->estdep = (hw_cap & XGMAC_HWFEAT_ESTDEP) >> 20;
	dma_cap->estsel = (hw_cap & XGMAC_HWFEAT_ESTSEL) >> 19;
	dma_cap->asp = (hw_cap & XGMAC_HWFEAT_ASP) >> 14;
	dma_cap->dvlan = (hw_cap & XGMAC_HWFEAT_DVLAN) >> 13;
	dma_cap->frpes = (hw_cap & XGMAC_HWFEAT_FRPES) >> 11;
	dma_cap->frpbs = (hw_cap & XGMAC_HWFEAT_FRPPB) >> 9;
	dma_cap->frpsel = (hw_cap & XGMAC_HWFEAT_FRPSEL) >> 3;

	return 0;
}

static void dwxgmac2_rx_watchdog(void __iomem *ioaddr, u32 riwt, u32 queue)
{
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:454", riwt & XGMAC_RWT, ioaddr + XGMAC_DMA_CH_Rx_WATCHDOG(queue));
}

static void dwxgmac2_set_rx_ring_len(void __iomem *ioaddr, u32 len, u32 chan)
{
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:459", len, ioaddr + XGMAC_DMA_CH_RxDESC_RING_LEN(chan));
}

static void dwxgmac2_set_tx_ring_len(void __iomem *ioaddr, u32 len, u32 chan)
{
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:464", len, ioaddr + XGMAC_DMA_CH_TxDESC_RING_LEN(chan));
}

static void dwxgmac2_set_rx_tail_ptr(void __iomem *ioaddr, u32 ptr, u32 chan)
{
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:469", ptr, ioaddr + XGMAC_DMA_CH_RxDESC_TAIL_LPTR(chan));
}

static void dwxgmac2_set_tx_tail_ptr(void __iomem *ioaddr, u32 ptr, u32 chan)
{
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:474", ptr, ioaddr + XGMAC_DMA_CH_TxDESC_TAIL_LPTR(chan));
}

static void dwxgmac2_enable_tso(void __iomem *ioaddr, bool en, u32 chan)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:479", ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	if (en)
		value |= XGMAC_TSE;
	else
		value &= ~XGMAC_TSE;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:486", value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));
}

static void dwxgmac2_qmode(void __iomem *ioaddr, u32 channel, u8 qmode)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:491", ioaddr + XGMAC_MTL_TXQ_OPMODE(channel));
	u32 flow = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:492", ioaddr + XGMAC_RX_FLOW_CTRL);

	value &= ~XGMAC_TXQEN;
	if (qmode != MTL_QUEUE_AVB) {
		value |= 0x2 << XGMAC_TXQEN_SHIFT;
		pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:497", 0, ioaddr + XGMAC_MTL_TCx_ETS_CONTROL(channel));
	} else {
		value |= 0x1 << XGMAC_TXQEN_SHIFT;
		pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:500", flow & (~XGMAC_RFE), ioaddr + XGMAC_RX_FLOW_CTRL);
	}

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:503", value, ioaddr +  XGMAC_MTL_TXQ_OPMODE(channel));
}

static void dwxgmac2_set_bfsize(void __iomem *ioaddr, int bfsize, u32 chan)
{
	u32 value;

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:510", ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
	value &= ~XGMAC_RBSZ;
	value |= bfsize << XGMAC_RBSZ_SHIFT;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:513", value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
}

static void dwxgmac2_enable_sph(void __iomem *ioaddr, bool en, u32 chan)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:518", ioaddr + XGMAC_RX_CONFIG);

	value &= ~XGMAC_CONFIG_HDSMS;
	value |= XGMAC_CONFIG_HDSMS_256; /* Segment max 256 bytes */
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:522", value, ioaddr + XGMAC_RX_CONFIG);

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:524", ioaddr + XGMAC_DMA_CH_CONTROL(chan));
	if (en)
		value |= XGMAC_SPH;
	else
		value &= ~XGMAC_SPH;
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:529", value, ioaddr + XGMAC_DMA_CH_CONTROL(chan));
}

static int dwxgmac2_enable_tbs(void __iomem *ioaddr, bool en, u32 chan)
{
	u32 value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:534", ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	if (en)
		value |= XGMAC_EDSE;
	else
		value &= ~XGMAC_EDSE;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:541", value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	value = pete_readl("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:543", ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan)) & XGMAC_EDSE;
	if (en && !value)
		return -EIO;

	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:547", XGMAC_DEF_FTOS, ioaddr + XGMAC_DMA_TBS_CTRL0);
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:548", XGMAC_DEF_FTOS, ioaddr + XGMAC_DMA_TBS_CTRL1);
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:549", XGMAC_DEF_FTOS, ioaddr + XGMAC_DMA_TBS_CTRL2);
	pete_writel("drivers/net/ethernet/stmicro/stmmac/dwxgmac2_dma.c:550", XGMAC_DEF_FTOS, ioaddr + XGMAC_DMA_TBS_CTRL3);
	return 0;
}

const struct stmmac_dma_ops dwxgmac210_dma_ops = {
	.reset = dwxgmac2_dma_reset,
	.init = dwxgmac2_dma_init,
	.init_chan = dwxgmac2_dma_init_chan,
	.init_rx_chan = dwxgmac2_dma_init_rx_chan,
	.init_tx_chan = dwxgmac2_dma_init_tx_chan,
	.axi = dwxgmac2_dma_axi,
	.dump_regs = dwxgmac2_dma_dump_regs,
	.dma_rx_mode = dwxgmac2_dma_rx_mode,
	.dma_tx_mode = dwxgmac2_dma_tx_mode,
	.enable_dma_irq = dwxgmac2_enable_dma_irq,
	.disable_dma_irq = dwxgmac2_disable_dma_irq,
	.start_tx = dwxgmac2_dma_start_tx,
	.stop_tx = dwxgmac2_dma_stop_tx,
	.start_rx = dwxgmac2_dma_start_rx,
	.stop_rx = dwxgmac2_dma_stop_rx,
	.dma_interrupt = dwxgmac2_dma_interrupt,
	.get_hw_feature = dwxgmac2_get_hw_feature,
	.rx_watchdog = dwxgmac2_rx_watchdog,
	.set_rx_ring_len = dwxgmac2_set_rx_ring_len,
	.set_tx_ring_len = dwxgmac2_set_tx_ring_len,
	.set_rx_tail_ptr = dwxgmac2_set_rx_tail_ptr,
	.set_tx_tail_ptr = dwxgmac2_set_tx_tail_ptr,
	.enable_tso = dwxgmac2_enable_tso,
	.qmode = dwxgmac2_qmode,
	.set_bfsize = dwxgmac2_set_bfsize,
	.enable_sph = dwxgmac2_enable_sph,
	.enable_tbs = dwxgmac2_enable_tbs,
};
